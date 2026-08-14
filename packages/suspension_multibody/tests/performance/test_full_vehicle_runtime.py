"""Short full-vehicle runtime and residual regression gate."""

from __future__ import annotations

from time import perf_counter

import pytest

from suspension_multibody.analysis import (
    FullVehicleDynamicSolver,
    build_vehicle_maneuver_case,
)


@pytest.mark.performance
def test_full_vehicle_short_probe_runtime_and_residual(full_vehicle_model) -> None:
    case = build_vehicle_maneuver_case(
        full_vehicle_model,
        "step_steer",
        end_time=0.002,
        step_size=0.001,
    )
    started = perf_counter()
    run = FullVehicleDynamicSolver().run(case)
    elapsed = perf_counter() - started

    assert elapsed < 5.0
    assert len(run.samples) == 3
    assert max(sample.constraint_residual for sample in run.samples) <= (
        case.solver.projection_failure_tolerance
    )
    assert max(sample.velocity_residual for sample in run.samples) <= 1.0e-5
    assert sum(len(sample.events) for sample in run.samples) >= 1
