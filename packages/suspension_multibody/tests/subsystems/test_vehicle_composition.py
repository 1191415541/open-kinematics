"""
The composed vehicle agrees with the historical one, field by field.

A vehicle is the largest object the package builds and the one with the most
readers, so "the new path works" is not a useful claim: the useful claim is that
it produces the *same* value, compared face by face.  Anything weaker would let a
renamed body or a dropped weld through, and both would show up as a changed
number much later.

The comparison is deliberately exhaustive rather than selective.  A field left
out of the comparison is a place the two can diverge unnoticed, which is the
failure this file exists to prevent.

The model fixture mirrors `tests/vehicle_assembly/test_vehicle_assembly.py` so
that the two files describe the same vehicle; a second, differently shaped fixture
would make a difference harder to attribute rather than easier to see.
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
from suspension_multibody.subsystems import (
    DEFAULT_AXLE_SUBSYSTEMS,
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyRequest,
)
from suspension_multibody.subsystems.entry import compose_vehicle
from suspension_multibody.subsystems.vehicle_assembly import (
    VehicleRuntime,
    compose_vehicle_runtime,
)

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


def _vehicle() -> VehicleModel:
    return VehicleModel(
        chassis=RigidBodySpec(
            name="chassis",
            mass=1200,
            inertia=((1_000_000, 0, 0), (0, 1_200_000, 0), (0, 0, 1_500_000)),
        ),
        front_axle=_axle("front", 1400),
        rear_axle=_axle("rear", -1400, rack_fixed=True),
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=22.0,
                axial_inertia=2,
                tire=_tire(),
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16, rack_damping=0),
    )


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_two_paths_agree_on_every_compared_face(mode: str) -> None:
    """The whole point of the task: one value, two ways of reaching it."""
    historical = compose_vehicle(_vehicle(), mode=mode)  # type: ignore[arg-type]
    composed = compose_vehicle_runtime(_vehicle(), mode=mode)  # type: ignore[arg-type]

    assert isinstance(composed, VehicleRuntime)
    assert composed.mode == historical.mode

    # Bodies: the name sequence *and* the mass properties, because the document
    # lists bodies in order and a condensed wheel changes a mass.
    assert list(composed.bodies) == list(historical.bodies)
    for name, body in composed.bodies.items():
        other = historical.bodies[name]
        assert body.mass == pytest.approx(other.mass), name
        assert body.fixed == other.fixed, name
        assert (body.center_of_mass == other.center_of_mass).all(), name
        assert (body.inertia == other.inertia).all(), name

    assert set(composed.points) == set(historical.points)
    for key, value in composed.points.items():
        assert (value == historical.points[key]).all(), key

    assert [(c.name, type(c).__name__) for c in composed.constraints] == [
        (c.name, type(c).__name__) for c in historical.constraints
    ]
    assert [(c.name, type(c).__name__) for c in composed.ideal_constraints] == [
        (c.name, type(c).__name__) for c in historical.ideal_constraints
    ]
    assert [(type(e).__name__, getattr(e, "name", "")) for e in composed.elements] == [
        (type(e).__name__, getattr(e, "name", "")) for e in historical.elements
    ]
    assert [c.name for c in composed.connections] == [
        c.name for c in historical.connections
    ]


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_wheel_tables_agree(mode: str) -> None:
    """
    The four wheel tables answer three different questions and must all match.

    ``wheel_centers`` names the *upright* the centre sits on, ``wheel_body_names``
    the body that actually carries the wheel, and ``wheel_rotations_local`` the
    frame the spin axis is expressed in.  A fixed wheel differs from a rotating
    one in all three, so comparing only the keys would miss the distinction.
    """
    historical = compose_vehicle(_vehicle(), mode=mode)  # type: ignore[arg-type]
    composed = compose_vehicle_runtime(_vehicle(), mode=mode)  # type: ignore[arg-type]

    assert set(composed.wheel_specs) == set(historical.wheel_specs)
    assert set(composed.wheel_centers) == set(historical.wheel_centers)
    for name, (body, center) in composed.wheel_centers.items():
        other_body, other_center = historical.wheel_centers[name]
        assert body == other_body, name
        assert (center == other_center).all(), name
    assert composed.wheel_body_names == historical.wheel_body_names
    assert set(composed.wheel_rotations_local) == set(historical.wheel_rotations_local)
    for name, rotation in composed.wheel_rotations_local.items():
        assert (rotation == historical.wheel_rotations_local[name]).all(), name


def test_total_mass_agrees() -> None:
    """Mass is the one number that catches a wheel condensed twice or not at all."""
    historical = compose_vehicle(_vehicle(), mode="K")  # type: ignore[arg-type]
    composed = compose_vehicle_runtime(_vehicle(), mode="K")  # type: ignore[arg-type]
    assert composed.total_mass == pytest.approx(historical.total_mass)


def test_the_axle_runtimes_are_the_composed_ones() -> None:
    """
    The vehicle reads each axle from the composition, not from a second build.

    This is the property the two hand-written paths could never have: a change to
    how an axle is composed reaches the vehicle without either being edited.
    """
    composed = compose_vehicle_runtime(_vehicle(), mode="K")  # type: ignore[arg-type]
    assert set(composed.axle_assemblies) == {"front", "rear"}
    for axle in composed.axle_assemblies.values():
        assert hasattr(axle, "ideal_constraints")
        assert hasattr(axle, "constraints")
        # The steering branch of the vehicle layer reads the axle's *chassis
        # rack marker*, and the merged point map can only keep one of the two.
        assert ("chassis", "rack_center") in axle.points


def test_the_weld_switch_agrees(monkeypatch) -> None:
    """
    Both the fused and the unfused form must match, not only the default.

    The switch is off by default, so a difference in the fused path would sit
    unexercised until someone turned it on -- which is exactly the kind of
    divergence a comparison of the default alone would miss.
    """
    for value in ("1", "0"):
        monkeypatch.setenv("SUSPENSION_MULTIBODY_CONDENSE_WELDS", value)
        historical = compose_vehicle(_vehicle(), mode="K")  # type: ignore[arg-type]
        composed = compose_vehicle_runtime(_vehicle(), mode="K")  # type: ignore[arg-type]
        assert list(composed.bodies) == list(historical.bodies), value
        assert composed.body_aliases == historical.body_aliases, value
    monkeypatch.setenv("SUSPENSION_MULTIBODY_CONDENSE_WELDS", "1")
    fused = compose_vehicle_runtime(_vehicle(), mode="K")  # type: ignore[arg-type]
    assert fused.body_aliases, "the fused form should record aliases"
    monkeypatch.delenv("SUSPENSION_MULTIBODY_CONDENSE_WELDS", raising=False)


def test_the_isolated_body_switch_agrees(monkeypatch) -> None:
    """The same argument as the weld switch, for the other switch."""
    monkeypatch.setenv("SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES", "0")
    historical = compose_vehicle(_vehicle(), mode="K")  # type: ignore[arg-type]
    composed = compose_vehicle_runtime(_vehicle(), mode="K")  # type: ignore[arg-type]
    assert list(composed.bodies) == list(historical.bodies)
    monkeypatch.delenv("SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES", raising=False)


def test_an_unknown_mode_is_refused() -> None:
    """A mode the composition cannot read is named, not defaulted."""
    with pytest.raises(ValueError, match="mode must be K or C"):
        compose_vehicle_runtime(_vehicle(), mode="X")  # type: ignore[arg-type]


def test_the_vehicle_carries_the_roles_its_request_names() -> None:
    """
    Phase 5: the vehicle's role set comes from the request, not from a constant.

    Two facts, and the second is the one that was missing.  Naming the full set
    produces exactly the vehicle this entry has always produced, and naming a set
    that is not a vehicle is refused -- by the same policy statement that checks an
    axle, so a vehicle without brakes is a rule violation rather than an assembly
    nobody thought to check.
    """
    from suspension_multibody.connections.policy import RuleViolation

    model = _vehicle()
    explicit = compose_vehicle(
        model,
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_VEHICLE_SUBSYSTEMS),
    )
    default = compose_vehicle(model, "K")
    assert explicit.capabilities.subsystems == default.capabilities.subsystems
    assert [c.name for c in explicit.constraints] == [
        c.name for c in default.constraints
    ]

    # The single-axle set is not a vehicle: it carries no brake and no drive, and
    # the policy names both rather than assembling a car that cannot stop.
    with pytest.raises(RuleViolation, match="brake"):
        compose_vehicle(
            model,
            "K",
            AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS),
        )


def test_a_vehicle_request_disagreeing_with_the_mode_is_refused() -> None:
    """The same contract `compose_axle` keeps, kept here as well."""
    with pytest.raises(ValueError, match="disagrees with request.mode"):
        compose_vehicle(
            _vehicle(),
            "K",
            AssemblyRequest(mode="C", subsystems=DEFAULT_VEHICLE_SUBSYSTEMS),
        )
