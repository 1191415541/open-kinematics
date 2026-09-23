"""
The availability matrix: a single axle has no brake and no drive, a vehicle has both.

Requirement 17 / D8 is asymmetric on purpose and this file locks both sides:
Adams Car's own single-axle assembly (`acar_gs_front.asy`) carries only
suspension, steering and the testrig, while the full-vehicle assembly
(`acar_gs_full.asy`) carries `brake_system` and `powertrain`.  Making the two
sides symmetric would be an unrequested behaviour change on either side.

The axle side is also asserted *negatively*: asking a single-axle assembly for
the brake or drive role must be refused, by name.  An assembly that accepted the
role and then reported it in `capabilities` would be lying to the rig that binds
to it.
"""

from __future__ import annotations

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
from tests.benchmark_fixture import benchmark_model

#: The two roles under test, and the two roles every other test here assumes.
TORQUE_ROLES = ("brake", "drive")


def _axle(name: str, x: float) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=500.0),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=500.0),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750.0, z=350.0),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=100.0),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=100.0),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750.0, z=100.0),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450.0, z=250.0),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750.0, z=250.0),
            "WHEEL_CENTER": Vec3(x=x, y=-750.0, z=300.0),
            "RACK_CENTER": Vec3(x=x, y=0.0, z=250.0),
        },
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(
            RigidBodySpec(name=body, mass=10.0)
            for body in (
                "rack",
                "upper_arm_L",
                "upper_arm_R",
                "lower_arm_L",
                "lower_arm_R",
                "upright_L",
                "upright_R",
                "tie_rod_L",
                "tie_rod_R",
            )
        ),
    )


def _legal_vehicle() -> VehicleModel:
    """Return a vehicle the schema accepts: four wheels, steering, two axles."""
    return VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=_axle("front", 1_400.0),
        rear_axle=_axle("rear", -1_400.0),
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=20.0,
                axial_inertia=2.0,
                tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0),
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16.0),
    )


def test_the_axle_capability_set_excludes_brake_and_drive() -> None:
    capabilities = build_front_axle(benchmark_model(), "K").capabilities
    assert capabilities is not None
    for role in TORQUE_ROLES:
        assert role not in capabilities.subsystems, role
    # The four it does carry, so this is not "an empty set happened to pass".
    assert capabilities.subsystems == DEFAULT_AXLE_SUBSYSTEMS
    assert "steering" in capabilities.subsystems
    assert "wheel" in capabilities.subsystems


def test_the_vehicle_capability_set_includes_brake_and_drive() -> None:
    capabilities = build_vehicle(_legal_vehicle(), "K").capabilities
    assert capabilities is not None
    for role in TORQUE_ROLES:
        assert role in capabilities.subsystems, role
    assert capabilities.subsystems == DEFAULT_VEHICLE_SUBSYSTEMS
    # Both assemblies offer the wheel coordinates; only the axle has a steering
    # subsystem here, so both offer the rack names as well.
    assert capabilities.provides("wheel_drive_L")
    assert capabilities.provides("rack_drive")


def test_the_two_declarations_differ_exactly_by_brake_and_drive() -> None:
    assert DEFAULT_VEHICLE_SUBSYSTEMS - DEFAULT_AXLE_SUBSYSTEMS == set(TORQUE_ROLES)
    assert DEFAULT_AXLE_SUBSYSTEMS - DEFAULT_VEHICLE_SUBSYSTEMS == set()


def test_the_axle_refuses_a_brake_or_drive_request_by_name() -> None:
    for role in TORQUE_ROLES:
        request = AssemblyRequest(
            mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS | {role}
        )
        try:
            build_front_axle(benchmark_model(), "K", request)
        except ValueError as error:
            assert role in str(error), role
            assert "single-axle" in str(error), role
        else:  # pragma: no cover - the call must raise
            raise AssertionError(f"an axle assembly must refuse the {role} role")
