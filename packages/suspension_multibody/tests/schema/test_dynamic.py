"""Dynamic schema tests."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    PrescribedMotion,
    TimeSignal,
    VehicleBodyModel,
    WrenchInput,
    WrenchSignal,
)


def _solver() -> DynamicSolverSettings:
    return DynamicSolverSettings(end_time=1.0, step_size=0.01)


def _vehicle() -> VehicleBodyModel:
    return VehicleBodyModel(
        mass=1500.0,
        inertia=((600_000.0, 0.0, 0.0), (0.0, 1_800_000.0, 0.0), (0.0, 0.0, 2_000_000.0)),
        wheelbase=2800.0,
        front_track=1600.0,
        rear_track=1600.0,
    )


def test_time_signal_interpolates_samples() -> None:
    signal = TimeSignal(times=(0.0, 1.0), values=(0.0, 10.0))

    assert signal.value_at(0.25) == pytest.approx(2.5)


def test_dynamic_case_requires_vehicle_for_vehicle_modes() -> None:
    with pytest.raises(ValidationError, match="vehicle body model"):
        DynamicCaseSpec(mode="vehicle_dynamic", solver=_solver())


def test_dynamic_case_rejects_prescribed_and_loaded_target_conflict() -> None:
    with pytest.raises(ValidationError, match="both prescribed and loaded"):
        DynamicCaseSpec(
            mode="axle_dynamic",
            solver=_solver(),
            prescribed_motions=(
                PrescribedMotion(
                    target="wheel_travel_left",
                    displacement=TimeSignal(constant=10.0),
                ),
            ),
            wrench_inputs=(
                WrenchInput(
                    target="wheel_travel_left",
                    wrench=WrenchSignal(fz=TimeSignal(constant=100.0)),
                ),
            ),
        )


def test_dynamic_vehicle_case_accepts_14_or_15_dof_boundary() -> None:
    case = DynamicCaseSpec(
        mode="vehicle_kc_dynamic",
        solver=_solver(),
        vehicle=_vehicle(),
    )

    assert case.vehicle is not None
    assert case.vehicle.degrees_of_freedom == 14
