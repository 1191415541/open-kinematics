"""
The full-vehicle experiment assembly, and the asymmetry that is intentional.

The user's two assemblies are the suspension assembly (one axle plus optional
steering, wheels from the rig) and the full-vehicle assembly (both axles plus
steering, chassis, wheels, brake and drive).  These tests cover the vehicle half:
it carries all six roles, it shares one set of subsystem definitions with the axle
half, and -- the part most likely to be "fixed" by a later reader -- **steering is
optional on the axle side and required on the vehicle side**, on purpose.
"""

from __future__ import annotations

import pytest

from suspension_multibody.preparation.assembly import build_front_axle, build_vehicle
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


def _vehicle(*, tire_mass: float = 0.0) -> VehicleModel:
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
                tire_mass=tire_mass,
                axial_inertia=2,
                tire=_tire(),
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16, rack_damping=0),
    )


def test_the_vehicle_assembly_carries_all_six_roles() -> None:
    """
    Brake and drive are the vehicle's own (requirement 17 / D8).

    A vehicle rig asks for them rather than probing the body list, so the absence
    of either would be a silently weaker assembly.
    """
    assembly = build_vehicle(_vehicle(), "K")
    assert assembly.capabilities is not None
    assert assembly.capabilities.subsystems == frozenset(DEFAULT_VEHICLE_SUBSYSTEMS)
    assert {"brake", "drive"} <= assembly.capabilities.subsystems


def test_the_two_assemblies_share_one_set_of_subsystem_definitions() -> None:
    """
    The axle half and the vehicle half are the same six roles.

    The vehicle assembly is two axles plus the rest, so a role the axle defines
    and the vehicle does not (or the reverse) would mean two vocabularies for one
    architecture.  The vehicle's set is the axle's plus brake and drive, exactly.
    """
    axle = build_front_axle(_axle("front", 1400), "K")
    vehicle = build_vehicle(_vehicle(), "K")
    assert axle.capabilities is not None and vehicle.capabilities is not None
    axle_roles = axle.capabilities.subsystems
    vehicle_roles = vehicle.capabilities.subsystems
    assert vehicle_roles - axle_roles == {"brake", "drive"}
    assert axle_roles - vehicle_roles == set()


def test_the_vehicle_builds_its_own_wheels_unlike_the_axle() -> None:
    """A single axle gets its wheels from the rig; a vehicle builds them (D9)."""
    vehicle = build_vehicle(_vehicle(), "K")
    assert any(name.startswith("wheel_") for name in vehicle.bodies)
    axle = build_front_axle(_axle("front", 1400), "K")
    assert not any(name.startswith("wheel_") for name in axle.bodies)


def test_steering_is_required_on_the_vehicle_side() -> None:
    """
    The asymmetry is deliberate and must not be "fixed" (decision D5).

    Requirement 15 opens steering *only* on the single-axle side.  Removing the
    vehicle-side requirement would ripple through five vehicle families and their
    recorded baselines, so the schema keeps `steering` mandatory and this test is
    what tells a later reader that the difference is a decision.
    """
    fields = VehicleModel.model_fields
    assert "steering" in fields
    assert fields["steering"].is_required()


def test_a_vehicle_model_without_steering_is_refused() -> None:
    with pytest.raises(Exception) as error:
        VehicleModel(**{**_vehicle().model_dump(), "steering": None})
    assert "steering" in str(error.value)


def test_the_axle_side_does_allow_steering_to_be_absent() -> None:
    """The other half of the asymmetry, asserted in the same place so it is seen."""
    model = _axle("front", 1400)
    hardpoints = {
        name: point for name, point in model.hardpoints.items() if name != "rack_center"
    }
    without = build_front_axle(
        model.model_copy(update={"hardpoints": hardpoints}),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    assert without.capabilities is not None
    assert "steering" not in without.capabilities.subsystems


def test_a_declared_tire_mass_leaves_the_wheel_body_alone() -> None:
    """
    Declaring a tire share must not change the body's own mass.

    The two numbers say different things: `mass` is what the body carries, and
    `tire_mass` is how much of the wheel's inertia the tire owns.  Conflating them
    would move the vehicle's total mass, which is the one thing requirement 10
    must not do.
    """
    plain = build_vehicle(_vehicle(tire_mass=0.0), "K")
    shared = build_vehicle(_vehicle(tire_mass=5.0), "K")
    assert plain.total_mass == pytest.approx(shared.total_mass)
    for name, body in plain.bodies.items():
        assert shared.bodies[name].mass == pytest.approx(body.mass)


def test_the_tire_mass_field_is_invisible_to_the_model_hash() -> None:
    """
    A dump-visible field would invalidate every recorded full-vehicle result.

    `api.py` hashes `model.model_dump(mode="json")` into `Provenance.model_hash`,
    so the field has to be excluded -- declared, typed and validated, yet absent
    from the hash.
    """
    model = _vehicle(tire_mass=5.0)
    dump = model.model_dump(mode="json")
    assert all("tire_mass" not in wheel for wheel in dump["wheels"])
    assert model.wheels[0].tire_mass == 5.0
