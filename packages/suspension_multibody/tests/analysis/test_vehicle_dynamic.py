"""Vehicle time-domain analysis tests."""

from __future__ import annotations

from suspension_multibody.api import run_dynamic_case
from suspension_multibody.dynamics import TireKinematics, tire_model_from_spec
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    InitialBodyState,
    MassSpec,
    PrescribedMotion,
    TimeSignal,
    TireModelSpec,
    Vec3,
    VehicleBodyModel,
    WrenchInput,
    WrenchSignal,
)


def _hardpoints() -> dict[str, Vec3]:
    return {
        "UPPER_INBOARD_FRONT": Vec3(x=0, y=-300, z=300),
        "UPPER_INBOARD_REAR": Vec3(x=300, y=-300, z=300),
        "UPPER_OUTBOARD": Vec3(x=150, y=-700, z=250),
        "LOWER_INBOARD_FRONT": Vec3(x=0, y=-320, z=0),
        "LOWER_INBOARD_REAR": Vec3(x=320, y=-320, z=0),
        "LOWER_OUTBOARD": Vec3(x=150, y=-720, z=50),
        "TIE_ROD_INBOARD": Vec3(x=100, y=-250, z=100),
        "TIE_ROD_OUTBOARD": Vec3(x=180, y=-700, z=100),
        "WHEEL_CENTER": Vec3(x=160, y=-760, z=150),
        "RACK_CENTER": Vec3(x=100, y=0, z=100),
    }


def _model() -> FrontAxleModel:
    return FrontAxleModel(hardpoints=_hardpoints(), mass=MassSpec(sprung_mass=1000.0))


def _vehicle() -> VehicleBodyModel:
    return VehicleBodyModel(
        degrees_of_freedom=15,
        mass=1500.0,
        inertia=((600_000.0, 0.0, 0.0), (0.0, 1_800_000.0, 0.0), (0.0, 0.0, 2_000_000.0)),
        wheelbase=2800.0,
        front_track=1600.0,
        rear_track=1600.0,
    )


def test_vehicle_kc_dynamic_replays_body_roll() -> None:
    case = DynamicCaseSpec(
        mode="vehicle_kc_dynamic",
        solver=DynamicSolverSettings(end_time=0.02, step_size=0.01),
        vehicle=_vehicle(),
        prescribed_motions=(
            PrescribedMotion(
                target="body_roll",
                displacement=TimeSignal(times=(0.0, 0.02), values=(0.0, 0.1)),
            ),
        ),
    )

    bundle = run_dynamic_case(_model(), case)

    assert bundle.manifest.mode == "vehicle_kc_dynamic"
    assert bundle.samples[-1].metrics["roll_angle"] == 0.1


def test_vehicle_dynamic_integrates_body_wrench() -> None:
    case = DynamicCaseSpec(
        mode="vehicle_dynamic",
        solver=DynamicSolverSettings(
            end_time=0.1,
            step_size=0.1,
            gravity={"x": 0.0, "y": 0.0, "z": 0.0},
        ),
        vehicle=_vehicle(),
        initial_states=(InitialBodyState(body="vehicle_body"),),
        wrench_inputs=(
            WrenchInput(
                target="vehicle_body",
                wrench=WrenchSignal(fx=TimeSignal(constant=1500.0)),
            ),
        ),
    )

    bundle = run_dynamic_case(_model(), case)

    assert bundle.manifest.mode == "vehicle_dynamic"
    assert bundle.samples[-1].velocity.fx == 0.1


def test_fiala_and_pac2002_tire_models_are_bounded_by_friction() -> None:
    state = TireKinematics(normal_load=4000.0, slip_angle=0.2, slip_ratio=0.3)
    fiala = tire_model_from_spec(TireModelSpec(kind="fiala"))
    pac = tire_model_from_spec(TireModelSpec(kind="pac2002", parameter_source="adams_builtin"))

    assert abs(fiala.evaluate(state).fy) <= 4000.0
    assert abs(pac.evaluate(state).fx) <= 4000.0
