"""Regression tests for the unified artifact owner."""

from pathlib import Path

import numpy as np

from suspension_multibody.axle_dynamics.result import (
    AxleDynamicsResult,
    AxleRunDiagnostics,
)
from suspension_multibody.io import read_artifact, write_artifact
from suspension_multibody.results import TimeSeriesResult, TimeSeriesSample
from suspension_multibody.results.vehicle import VehicleDynamicsResult


def _axle_result() -> AxleDynamicsResult:
    times = np.asarray((0.0, 0.1), dtype=float)
    zeros = np.zeros(2, dtype=float)
    diagnostics = AxleRunDiagnostics(
        accepted=np.ones(2),
        internal_steps=np.ones(2),
        rejected_attempts=zeros,
        newton_iterations=np.ones(2),
        minimum_accepted_step_s=np.full(2, 0.1),
        maximum_accepted_step_s=np.full(2, 0.1),
        last_accepted_step_s=np.full(2, 0.1),
        position_residual=zeros,
        velocity_residual=zeros,
        dynamics_residual=zeros,
        active_contacts=np.ones(2),
        contact_events=zeros,
        local_error_ratio=zeros,
        energy_residual=zeros,
        failure_code=zeros,
        pinned_null_directions=zeros,
    )
    return AxleDynamicsResult(
        times_s=times,
        body_names=("body",),
        constraint_names=("joint",),
        spring_names=("spring",),
        bushing_names=(),
        anti_roll_bar_names=(),
        tire_names=("tire",),
        states=np.zeros((2, 1, 19)),
        constraint_wrench=np.zeros((2, 1, 6)),
        spring_output=np.zeros((2, 1, 7)),
        bushing_output=np.zeros((2, 0, 12)),
        anti_roll_output=np.zeros((2, 0, 3)),
        diagnostics=diagnostics,
        tire_output=np.zeros((2, 1, 40)),
        energy=np.zeros((2, 21)),
        metrics={"peak_force_n": 12.0},
    )


def _series(status: str = "success") -> TimeSeriesResult:
    return TimeSeriesResult.from_samples(
        (
            TimeSeriesSample(time=0.0, body="body", metrics={"heave": 0.0}),
            TimeSeriesSample(time=0.1, body="body", metrics={"heave": 0.5}),
        ),
        diagnostics=({"code": "ok"},),
        metrics={"peak_heave": 0.5},
        performance={"native_kernel_wall_time_s": 1.25},
        provenance={"model_hash": "model", "case_hash": "case"},
        mode="axle_dynamic",
        status=status,
        partial_evidence={"failed_sample_index": 2} if status == "partial" else {},
        run_id=f"series-{status}",
    )


def test_write_read_native_axle_artifact(tmp_path: Path) -> None:
    manifest = write_artifact(_axle_result(), tmp_path / "axle")
    loaded = read_artifact(manifest)

    assert loaded["manifest"]["artifact_type"] == "axle_dynamics_result"
    assert loaded["manifest"]["status"] == "success"
    assert loaded["manifest"]["metrics"]["peak_force_n"] == 12.0
    assert loaded["arrays"]["states"].shape == (2, 1, 19)
    assert loaded["arrays"]["tire_output"].shape == (2, 1, 40)


def test_write_read_native_vehicle_artifact(tmp_path: Path) -> None:
    axle = _axle_result()
    result = VehicleDynamicsResult(
        axle=axle,
        steering_names=("front_rack",),
        steering_output=np.zeros((2, 1, 4)),
        metrics={"steering_peak": 0.2},
    )
    loaded = read_artifact(write_artifact(result, tmp_path / "vehicle"))

    assert loaded["manifest"]["artifact_type"] == "vehicle_dynamics_result"
    assert loaded["manifest"]["metrics"]["steering_peak"] == 0.2
    assert loaded["arrays"]["steering_output"].shape == (2, 1, 4)


def test_write_read_time_series_with_empty_nested_fields(tmp_path: Path) -> None:
    loaded = read_artifact(write_artifact(_series(), tmp_path / "series"))

    assert loaded["manifest"]["artifact_type"] == "time_series_result"
    assert loaded["manifest"]["status"] == "success"
    assert loaded["manifest"]["metrics"]["peak_heave"] == 0.5
    assert loaded["manifest"]["performance"]["native_kernel_wall_time_s"] == 1.25
    assert loaded["arrays"]["times_s"].tolist() == [0.0, 0.1]
    assert loaded["tables"]["time_samples"][1]["body"] == "body"


def test_write_failed_partial_artifact_preserves_evidence(tmp_path: Path) -> None:
    partial = _series("partial")
    failure = RuntimeError("solver stopped at sample 2")
    loaded = read_artifact(
        write_artifact(
            None,
            tmp_path / "failed",
            partial=partial,
            status="failed",
            failure=failure,
        )
    )

    assert loaded["manifest"]["artifact_type"] == "time_series_result"
    assert loaded["manifest"]["status"] == "failed"
    assert loaded["failure_evidence"]["type"] == "RuntimeError"
    assert loaded["failure_evidence"]["message"] == "solver stopped at sample 2"
    assert loaded["partial_evidence"]["completed_sample_count"] == 2
    assert loaded["arrays"]["times_s"].tolist() == [0.0, 0.1]
def test_write_time_series_with_all_empty_nested_fields(tmp_path: Path) -> None:
    result = TimeSeriesResult.from_samples(
        (TimeSeriesSample(time=0.0, body="body"),),
        mode="axle_dynamic",
    )

    loaded = read_artifact(write_artifact(result, tmp_path / "empty-series"))

    assert loaded["manifest"]["artifact_type"] == "time_series_result"
    assert loaded["tables"]["time_samples"][0]["loads"] == "{}"


def test_write_failed_native_partial_artifact_uses_partial_result(tmp_path: Path) -> None:
    failure = RuntimeError("native solver failed")

    loaded = read_artifact(
        write_artifact(
            None,
            tmp_path / "failed-native",
            partial=_axle_result(),
            status="failed",
            failure=failure,
        )
    )

    assert loaded["manifest"]["artifact_type"] == "axle_dynamics_result"
    assert loaded["manifest"]["status"] == "failed"
    assert loaded["arrays"]["states"].shape == (2, 1, 19)
