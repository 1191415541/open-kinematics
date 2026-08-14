"""Shared fixtures for suspension_multibody tests."""

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


@pytest.fixture
def minimal_model() -> FrontAxleModel:
    return FrontAxleModel(
        hardpoints={"LOWER_FRONT_LEFT": Vec3(x=100, y=-700, z=200)},
        mass=MassSpec(sprung_mass=1200),
    )


@pytest.fixture
def full_vehicle_model() -> VehicleModel:
    def axle(name: str, x: float) -> FrontAxleModel:
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

    return VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=axle("front", 1_400.0),
        rear_axle=axle("rear", -1_400.0),
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
