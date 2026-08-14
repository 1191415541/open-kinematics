"""Full-vehicle topology assembly tests."""

from suspension_multibody.model import build_vehicle
from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    SteeringSystemSpec,
    Vec3,
    VehicleModel,
    WheelSpec,
)


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
                "rack", "upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R",
                "upright_L", "upright_R", "tie_rod_L", "tie_rod_R",
            )
        ),
    )


def _vehicle() -> VehicleModel:
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
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16.0),
    )


def test_build_vehicle_contains_two_axles_and_four_spinning_wheels() -> None:
    assembly = build_vehicle(_vehicle())

    assert set(assembly.axle_assemblies) == {"front", "rear"}
    assert set(assembly.wheel_ids) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    assert len([item for item in assembly.constraints if "wheel_spin" in item.name]) == 4
    assert "front_upright_L" in assembly.component_ids
    assert "rear_upright_R" in assembly.component_ids


def test_build_vehicle_preserves_chassis_and_wheel_mass() -> None:
    assembly = build_vehicle(_vehicle())

    assert assembly.bodies["chassis"].mass == 1200.0
    assert assembly.total_mass == 1460.0
    assert all(assembly.wheel_center_local(name).shape == (3,) for name in assembly.wheel_ids)
