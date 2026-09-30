"""
The assembler takes an entry list, and the historical entry point is an adapter.

Two claims are checked here, and they are different claims.

The first is that the assembler does not need a ``VehicleModel``: two entries
named by hand assemble a vehicle, the placements are whatever the caller wrote
(``front``/``middle``, not ``front``/``rear``) and the prefixes are the entries'
own.  A vehicle that could only be assembled from the two-axle model would still
be the old hard-coded path wearing an entry list.

The second is that the historical entry point produces exactly what the entry
list produces.  ``compose_vehicle_runtime`` used to *be* the assembly; it is the
adapter now, and an adapter that quietly diverges is the failure mode that makes
the whole rework pointless -- so the comparison is field by field rather than
"it still returns something".
"""

from __future__ import annotations

import pytest

from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    SteeringSystemSpec,
    TireModelSpec,
    Vec3,
    VehicleModel,
    WheelSpec,
)
from suspension_multibody.subsystems.assembler import (
    AxleEntry,
    compose_entries_runtime,
)
from suspension_multibody.subsystems.vehicle_assembly import (
    VehicleRuntime,
    compose_vehicle_runtime,
)
from suspension_multibody.subsystems.vehicle_parts import _body_from_spec

_BODY_NAMES = (
    "rack",
    "upper_arm_L",
    "lower_arm_L",
    "upright_L",
    "tie_rod_L",
    "upper_arm_R",
    "lower_arm_R",
    "upright_R",
    "tie_rod_R",
)


def _axle(name: str, x: float, *, rack_fixed: bool = False) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        rack_fixed_to_chassis=rack_fixed,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500, z=500),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150, y=-500, z=500),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750, z=350),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500, z=100),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150, y=-500, z=100),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750, z=100),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450, z=250),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750, z=250),
            "WHEEL_CENTER": Vec3(x=x, y=-750, z=300),
            "RACK_CENTER": Vec3(x=x, y=0, z=250),
        },
        mass=MassSpec(sprung_mass=600),
        bodies=tuple(
            RigidBodySpec(
                name=body,
                mass=100,
                inertia=((100, 0, 0), (0, 100, 0), (0, 0, 100)),
            )
            for body in _BODY_NAMES
        ),
    )


def _tire() -> TireModelSpec:
    return TireModelSpec(
        kind="native_brush",
        unloaded_radius=300,
        maximum_compression=250,
        vertical_stiffness=200,
        cornering_stiffness=80_000,
        longitudinal_stiffness=120_000,
        relaxation_length=300,
    )


def _wheels(placement: str) -> tuple[WheelSpec, ...]:
    return tuple(
        WheelSpec(
            name=name,
            body=f"wheel_{name}",
            center_local=Vec3(),
            mass=22.0,
            axial_inertia=2,
            tire=_tire(),
        )
        for name in (f"{placement}_left", f"{placement}_right")
    )


def _vehicle() -> VehicleModel:
    return VehicleModel(
        chassis=RigidBodySpec(
            name="chassis",
            mass=1200,
            inertia=((1_000_000, 0, 0), (0, 1_200_000, 0), (0, 0, 1_500_000)),
        ),
        front_axle=_axle("front", 1400),
        rear_axle=_axle("rear", -1400, rack_fixed=True),
        wheels=_wheels("front") + _wheels("rear"),
        steering=SteeringSystemSpec(ratio=16, rack_damping=0),
    )


def _entries_for(model: VehicleModel) -> tuple[AxleEntry, AxleEntry]:
    """Write out, by hand, the entries a ``VehicleModel`` amounts to."""
    replaces = {"chassis": model.chassis.name, "ground": model.chassis.name}
    return (
        AxleEntry(
            placement="front",
            prefix="front_",
            axle=model.front_axle,
            replace_bodies=replaces,
            wheels=tuple(w for w in model.wheels if w.name.startswith("front_")),
        ),
        AxleEntry(
            placement="rear",
            prefix="rear_",
            axle=model.rear_axle,
            replace_bodies=replaces,
            wheels=tuple(w for w in model.wheels if w.name.startswith("rear_")),
        ),
    )


def test_the_assembler_needs_no_vehicle_model() -> None:
    """
    Two entries assemble a vehicle whose placements and prefixes are their own.

    ``middle`` is the point: the assembler cannot be classifying placements or
    counting axles, because it was never told that a vehicle has two axles or
    which one is the front.
    """
    entries = (
        AxleEntry(
            placement="front",
            prefix="front_",
            axle=_axle("front", 1400),
            replace_bodies={"chassis": "chassis", "ground": "chassis"},
            wheels=_wheels("front"),
        ),
        AxleEntry(
            placement="middle",
            prefix="mid_",
            axle=_axle("middle", 0, rack_fixed=True),
            replace_bodies={"chassis": "chassis", "ground": "chassis"},
            # The wheel-name vocabulary is still the four corporate names -- that
            # is `schema/vehicle.py`, opened by later work on this row -- so the
            # middle axle's wheel ends are named with the second half of it.  That
            # the assembler accepts them is the point: it keys the wheel tables by
            # the entry's own wheels and never derives anything from the name.
            wheels=_wheels("rear"),
        ),
    )

    composed = compose_entries_runtime(entries, chassis_name="chassis")

    assert isinstance(composed, VehicleRuntime)
    assert set(composed.axle_assemblies) == {"front", "middle"}
    # Every axle body carries its own entry's prefix, and the prefixed names are
    # exactly the axle's own bodies: nothing was renamed by a rule about ``front``
    # or ``rear``.
    for prefix, axle in (("front_", entries[0].axle), ("mid_", entries[1].axle)):
        names = {f"{prefix}{spec.name}" for spec in axle.bodies}
        assert names <= set(composed.bodies), prefix
    assert not [name for name in composed.bodies if name.startswith("front_mid_")]
    # Both entries handed their chassis body over to the vehicle's own, so the
    # vehicle has one chassis body and no prefixed copy of it.
    assert [name for name in composed.bodies if name.endswith("chassis")] == [
        "chassis"
    ]
    assert set(composed.wheel_specs) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    # The wheel centres are read off each entry's own bodies, both carrying that
    # entry's prefix -- the middle axle's wheels included.
    assert {
        wheel: body for wheel, (body, _center) in composed.wheel_centers.items()
    } == {
        "front_left": "front_wheel_hub_L",
        "front_right": "front_wheel_hub_R",
        "rear_left": "mid_wheel_hub_L",
        "rear_right": "mid_wheel_hub_R",
    }

@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_adapter_agrees_with_the_entry_list(mode: str) -> None:
    """
    ``compose_vehicle_runtime`` is the entry list, spelled differently.

    Compared field by field: bodies with their mass properties and their order,
    points, constraints, ideal constraints, elements, connections and the four
    wheel tables.  A field left out is a place the two can diverge unnoticed.
    """
    model = _vehicle()
    adapted = compose_vehicle_runtime(model, mode=mode)  # type: ignore[arg-type]
    engine = compose_entries_runtime(
        _entries_for(model),
        chassis_name=model.chassis.name,
        chassis_body=_body_from_spec(model.chassis),
        mode=mode,  # type: ignore[arg-type]
    )

    assert isinstance(adapted, VehicleRuntime)
    assert adapted.mode == engine.mode
    assert set(adapted.axle_assemblies) == set(engine.axle_assemblies) == {
        "front",
        "rear",
    }

    assert list(adapted.bodies) == list(engine.bodies)
    for name, body in adapted.bodies.items():
        other = engine.bodies[name]
        assert body.mass == pytest.approx(other.mass), name
        assert body.fixed == other.fixed, name
        assert (body.center_of_mass == other.center_of_mass).all(), name
        assert (body.inertia == other.inertia).all(), name

    assert list(adapted.points) == list(engine.points)
    for key, value in adapted.points.items():
        assert (value == engine.points[key]).all(), key

    for attribute in ("constraints", "ideal_constraints"):
        assert [
            (item.name, type(item).__name__)
            for item in getattr(adapted, attribute)
        ] == [
            (item.name, type(item).__name__)
            for item in getattr(engine, attribute)
        ], attribute
    assert [
        (type(item).__name__, getattr(item, "name", "")) for item in adapted.elements
    ] == [(type(item).__name__, getattr(item, "name", "")) for item in engine.elements]
    assert [connection.name for connection in adapted.connections] == [
        connection.name for connection in engine.connections
    ]
    assert [
        (connection.body_a, connection.body_b) for connection in adapted.connections
    ] == [
        (connection.body_a, connection.body_b) for connection in engine.connections
    ]

    assert list(adapted.wheel_specs) == list(engine.wheel_specs)
    assert list(adapted.wheel_centers) == list(engine.wheel_centers)
    for name, (body, center) in adapted.wheel_centers.items():
        other_body, other_center = engine.wheel_centers[name]
        assert body == other_body, name
        assert (center == other_center).all(), name
    assert adapted.wheel_body_names == engine.wheel_body_names
    assert list(adapted.wheel_rotations_local) == list(engine.wheel_rotations_local)
    for name, rotation in adapted.wheel_rotations_local.items():
        assert (rotation == engine.wheel_rotations_local[name]).all(), name

    assert adapted.total_mass == pytest.approx(engine.total_mass)
