from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.metrics import (
    compute_axle_metrics,
    compute_case_metrics,
    compute_vehicle_metrics,
    contact_metrics,
    convergence_metrics,
    peak,
    register_case_metric,
    registered_case_metric_families,
    residual_norm,
    rms,
    time_metrics,
)


class FakeResult:
    times_s = np.array([0.0, 0.5, 1.0])
    tire_output = np.zeros((3, 1, 7))
    tire_output[:, 0, 4] = [1.0, 2.0, 3.0]
    tire_output[:, 0, 5] = [0.0, 1.0, 0.0]
    tire_output[:, 0, 6] = [2.0, 2.0, 2.0]
    diagnostics = None
    performance = SimpleNamespace(available=False)


def test_common_metrics_validate_and_summarize_samples() -> None:
    assert peak([-2.0, 1.0]) == 2.0
    assert rms([3.0, 4.0]) == pytest.approx(np.sqrt(12.5))
    assert residual_norm([[3.0, 4.0], [0.0, 5.0]]).tolist() == [5.0, 5.0]
    assert time_metrics(FakeResult()) == {
        "sample_count": 3,
        "start_s": 0.0,
        "end_s": 1.0,
        "duration_s": 1.0,
    }
    assert convergence_metrics(FakeResult()) == {"available": False}
    assert contact_metrics(FakeResult()) == {"available": False}
    with pytest.raises(ValueError, match="must not be empty"):
        peak([])
    with pytest.raises(ValueError, match="finite"):
        rms([1.0, np.nan])


def test_axle_and_vehicle_metrics_keep_output_prefixes() -> None:
    axle_metrics = compute_axle_metrics(FakeResult())
    assert axle_metrics["maximum_normal_force_n"] == 3.0
    assert axle_metrics["rms_lateral_force_n"] == pytest.approx(2.0)
    assert axle_metrics["performance_available"] is False

    vehicle = SimpleNamespace(
        axle=FakeResult(),
        steering_output=np.array([[1.0, -2.0], [3.0, 0.0]]),
        native_kernel_wall_time_s=1.5,
    )
    vehicle_metrics = compute_vehicle_metrics(vehicle)
    assert vehicle_metrics["axle_maximum_normal_force_n"] == 3.0
    assert vehicle_metrics["maximum_steering_output"] == 3.0
    assert vehicle_metrics["native_kernel_wall_time_s"] == 1.5

def test_vehicle_metrics_ignore_not_applicable_steering_components() -> None:
    vehicle = SimpleNamespace(
        axle=FakeResult(),
        steering_output=np.array(
            [[[1.0, np.nan], [3.0, np.nan]], [[2.0, np.nan], [0.0, np.nan]]]
        ),
        native_kernel_wall_time_s=0.0,
    )
    metrics = compute_vehicle_metrics(vehicle)
    assert metrics["maximum_steering_output"] == 3.0
    assert metrics["rms_steering_output"] == pytest.approx(np.sqrt(3.5))
def test_axle_metrics_mark_empty_partial_results_unavailable() -> None:
    result = SimpleNamespace(
        times_s=np.asarray((), dtype=float),
        performance=SimpleNamespace(available=False),
        diagnostics=None,
    )
    metrics = compute_axle_metrics(result)
    assert metrics["status"] == "unavailable"
    assert metrics["sample_count"] == 0
    assert metrics["reason"] == "no completed samples"


def test_case_specific_metrics_cover_supported_families() -> None:
    expected = {
        "kc_quasi_static",
        "axle_dynamic",
        "vehicle_kc",
        "vehicle_kc_dynamic",
        "vehicle_dynamic",
        "handling",
        "ride_four_post",
        "ride_random_road",
    }
    assert set(registered_case_metric_families()) == expected
    assert compute_case_metrics("handling", FakeResult())["status"] == "not_applicable"


def test_case_specific_metrics_are_registered_by_family() -> None:
    assert compute_case_metrics("vehicle_kc_dynamic", FakeResult())["status"] == "not_applicable"
def test_vehicle_metrics_include_explicit_wheel_load_derivations() -> None:
    vehicle = SimpleNamespace(axle=FakeResult(), steering_output=None)
    metrics = compute_vehicle_metrics(
        vehicle,
        wheel_loads={
            "front_left": 100.0,
            "front_right": 120.0,
            "rear_left": 80.0,
            "rear_right": 90.0,
        },
    )
    assert metrics["normal_load_total"] == 390.0
    assert metrics["normal_load_front_axle"] == 220.0
    assert metrics["load_transfer_right_minus_left"] == 30.0


def test_case_specific_metric_registration_remains_extensible() -> None:

    def probe(value: int) -> int:
        return value + 1

    register_case_metric(" Probe ", probe)
    assert compute_case_metrics("probe", 4) == 5
    with pytest.raises(KeyError, match="already registered"):
        register_case_metric("probe", probe)
    register_case_metric("probe", lambda value: value + 2, replace=True)
    assert compute_case_metrics("PROBE", 4) == 6
