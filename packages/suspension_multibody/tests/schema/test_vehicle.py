"""Full-vehicle schema contracts."""

import pytest
from pydantic import ValidationError

from suspension_multibody.schema import (
    DrivelineSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    RoadSurfaceSpec,
    SteeringSystemSpec,
    TimeSignal,
    TireModelSpec,
    Vec3,
    VehicleDynamicCase,
    VehicleModel,
    WheelSpec,
)


def _axle(name: str) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={"LOWER_FRONT_LEFT": Vec3(x=100, y=-700, z=200)},
        mass=MassSpec(sprung_mass=600),
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
        chassis=RigidBodySpec(name="chassis", mass=1200),
        front_axle=_axle("front"),
        rear_axle=_axle("rear"),
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(x=0.0, y=0.0, z=-300.0),
                tire=TireModelSpec(kind="fiala"),
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16.0),
        driveline=DrivelineSpec(
            driven_wheels=("front_left", "front_right"),
            maximum_drive_torque=2_000.0,
            drive_split=(0.5, 0.5, 0.0, 0.0),
        ),
    )


def test_vehicle_schema_requires_four_named_wheels() -> None:
    vehicle = _vehicle()
    assert {wheel.name for wheel in vehicle.wheels} == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }


def test_vehicle_schema_rejects_duplicate_body_names() -> None:
    wheels = list(_vehicle().wheels)
    wheels[1] = wheels[1].model_copy(update={"body": wheels[0].body})
    with pytest.raises(ValidationError, match="body names"):
        VehicleModel(**{**_vehicle().model_dump(), "wheels": tuple(wheels)})


def test_road_surface_rejects_amplitude_for_plane() -> None:
    with pytest.raises(ValidationError, match="zero amplitude"):
        RoadSurfaceSpec(kind="plane", amplitude=1.0)


def test_fiala_schema_rejects_nonphysical_friction_parameters() -> None:
    with pytest.raises(ValidationError, match="UMAX must be positive"):
        TireModelSpec(kind="fiala", fiala_parameters={"UMAX": 0.0})
    with pytest.raises(ValidationError, match="UMAX must not be below UMIN"):
        TireModelSpec(
            kind="fiala",
            fiala_parameters={"UMIN": 1.0, "UMAX": 0.9},
        )


def test_dynamic_case_validates_initial_wheel_speed_names() -> None:
    case = VehicleDynamicCase(
        solver=DynamicSolverSettings(end_time=1.0, step_size=0.01),
        vehicle=_vehicle(),
        steering_input=TimeSignal(constant=0.0),
        initial_wheel_speeds=(("front_left", 10.0),),
    )
    assert case.vehicle.name == "full_vehicle"
    with pytest.raises(ValidationError, match="undefined wheel"):
        VehicleDynamicCase(
            **{
                **case.model_dump(),
                "initial_wheel_speeds": (("spare", 1.0),),
            }
        )


def test_dynamic_case_validates_direct_wheel_torque_signals() -> None:
    model = _vehicle()
    case = VehicleDynamicCase(
        solver=DynamicSolverSettings(end_time=1.0, step_size=0.01),
        vehicle=model,
        wheel_drive_torque=(
            ("rear_left", TimeSignal(constant=250.0)),
        ),
        wheel_brake_torque=(
            ("front_right", TimeSignal(constant=40.0)),
        ),
    )

    assert dict(case.wheel_drive_torque)["rear_left"].constant == 250.0
    assert dict(case.wheel_brake_torque)["front_right"].constant == 40.0

    with pytest.raises(ValidationError, match="non-negative"):
        VehicleDynamicCase(
            solver=DynamicSolverSettings(end_time=1.0, step_size=0.01),
            vehicle=model,
            wheel_brake_torque=(
                ("front_left", TimeSignal(constant=-1.0)),
            ),
        )

    with pytest.raises(ValidationError, match="undefined wheel"):
        VehicleDynamicCase(
            solver=DynamicSolverSettings(end_time=1.0, step_size=0.01),
            vehicle=model,
            wheel_drive_torque=(
                ("spare", TimeSignal(constant=1.0)),
            ),
        )
