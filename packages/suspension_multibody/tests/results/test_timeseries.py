from __future__ import annotations

import json
from types import MappingProxyType

import numpy as np
import pytest

from suspension_multibody.results import TimeSeriesResult, TimeSeriesSample


def _samples() -> tuple[TimeSeriesSample, ...]:
    return (
        TimeSeriesSample(
            time=0.0,
            body="axle",
            pose=np.array([1.0, 2.0, 3.0]),
            metrics={"heave": 0.0},
            loads={"right": {"fz": 25.0}},
            result={"sample_id": "s0"},
        ),
        TimeSeriesSample(
            time=0.1,
            body="axle",
            pose=np.array([1.5, 2.0, 3.0]),
            metrics={"heave": 0.5},
            loads={"right": {"fz": 30.0}},
            result={"sample_id": "s1"},
        ),
    )


def test_time_series_result_preserves_samples_diagnostics_metrics_and_provenance() -> None:
    result = TimeSeriesResult.from_samples(
        _samples(),
        diagnostics=( {"code": "warn", "severity": "warning"}, ),
        metrics={"peak_heave": 0.5},
        performance={"native_kernel_wall_time_s": 1.25},
        provenance={"model_hash": "model", "case_hash": "case"},
        mode="axle_dynamic",
        run_id="run-1",
    )

    assert result.times_s.tolist() == [0.0, 0.1]
    assert result.sample_count == 2
    assert result.result_objects[1] == {"sample_id": "s1"}
    assert result.diagnostics[0]["code"] == "warn"
    assert result.metrics["peak_heave"] == 0.5
    assert result.performance["native_kernel_wall_time_s"] == 1.25
    assert result.manifest.run_id == "run-1"
    assert result.manifest.sample_count == 2


def test_time_series_result_is_immutable_and_json_serializable() -> None:
    result = TimeSeriesResult.from_samples(
        _samples(),
        provenance={"nested": {"value": 1}},
        run_id="run-2",
    )

    assert isinstance(result.metrics, MappingProxyType)
    with pytest.raises(TypeError):
        result.provenance["nested"]["value"] = 2
    with pytest.raises(ValueError):
        result.times_s[0] = 99.0
    with pytest.raises(TypeError):
        result.samples[0].metrics["heave"] = 9.0
    json.dumps(result.as_dict())


def test_time_series_result_preserves_partial_and_failed_evidence() -> None:
    partial = TimeSeriesResult.from_samples(
        _samples()[:1],
        status="partial",
        partial_evidence={"failed_sample_index": 1, "failed_time_s": 0.1},
    )
    failed = TimeSeriesResult.from_samples(
        (),
        status="failed",
        failure_evidence={"code": "kernel_error", "message": "solver failed"},
    )

    assert partial.is_partial
    assert partial.partial == {"failed_sample_index": 1, "failed_time_s": 0.1}
    assert failed.is_failed
    assert failed.failure == {"code": "kernel_error", "message": "solver failed"}
    assert failed.times_s.size == 0


def test_time_series_result_rejects_invalid_status_and_time_grid() -> None:
    with pytest.raises(ValueError, match="unsupported time-series status"):
        TimeSeriesResult.from_samples((), status="unknown")
    with pytest.raises(ValueError, match="strictly increasing"):
        TimeSeriesResult.from_samples(
            _samples(),
            times_s=np.array([0.1, 0.0]),
        )
