"""Axle time-domain analysis tests."""

from __future__ import annotations

from suspension_multibody.api import run_dynamic_case
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    MassSpec,
    PrescribedMotion,
    TimeSignal,
    Vec3,
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


def test_axle_dynamic_runs_time_series_with_motion_and_loads() -> None:
    model = FrontAxleModel(hardpoints=_hardpoints(), mass=MassSpec(sprung_mass=1000.0))
    case = DynamicCaseSpec(
        mode="axle_dynamic",
        solver=DynamicSolverSettings(end_time=0.02, step_size=0.01),
        prescribed_motions=(
            PrescribedMotion(
                target="wheel_travel_left",
                displacement=TimeSignal(times=(0.0, 0.02), values=(0.0, 5.0)),
            ),
        ),
        wrench_inputs=(
            WrenchInput(
                target="right",
                wrench=WrenchSignal(fz=TimeSignal(constant=25.0)),
            ),
        ),
    )

    bundle = run_dynamic_case(model, case)
    axle_samples = [sample for sample in bundle.samples if sample.body == "axle"]

    assert bundle.manifest.mode == "axle_dynamic"
    assert len(axle_samples) == 3
    assert axle_samples[-1].metrics["wheel_travel_left"] == 5.0
    assert axle_samples[-1].loads["right"].fz == 25.0
