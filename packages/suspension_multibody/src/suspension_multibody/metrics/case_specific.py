"""Registered metrics for family-specific simulation results."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np

_CASE_METRICS: dict[str, Callable[..., Any]] = {}


def register_case_metric(
    family: str, function: Callable[..., Any], *, replace: bool = False
) -> Callable[..., Any]:
    """Register one family-specific metric function."""
    key = str(family).strip().lower()
    if not key:
        raise ValueError("case metric family cannot be empty")
    if key in _CASE_METRICS and not replace:
        raise KeyError(f"case metric already registered for {key}")
    _CASE_METRICS[key] = function
    return function


def compute_case_metrics(family: str, *args: Any, **kwargs: Any) -> Any:
    """Dispatch to the explicitly registered family-specific implementation."""
    key = str(family).strip().lower()
    try:
        function = _CASE_METRICS[key]
    except KeyError as error:
        available = ", ".join(sorted(_CASE_METRICS))
        raise KeyError(f"no case metric registered for {key}; available: {available}") from error
    return function(*args, **kwargs)


def case_metric_for(family: str) -> Callable[..., Any]:
    """Return the registered metric callable for one family."""
    key = str(family).strip().lower()
    try:
        return _CASE_METRICS[key]
    except KeyError as error:
        raise KeyError(f"no case metric registered for {key}") from error


def registered_case_metric_families() -> tuple[str, ...]:
    """Return registered families in stable order for audit and diagnostics."""
    return tuple(sorted(_CASE_METRICS))


def not_applicable_metrics(
    result: Any = None,
    *,
    family: str | None = None,
    reason: str = "no dedicated case-specific metrics are defined",
) -> dict[str, Any]:
    """Return explicit not-applicable evidence without fabricating values."""
    del result
    return {
        "status": "not_applicable",
        "family": None if family is None else str(family).strip().lower(),
        "reason": reason,
        "metrics": {},
    }


def wheel_metrics(state: Any, assembly: Any, side: str) -> dict[str, float]:
    """Compute wheel-center and orientation metrics for one vehicle side."""
    normalized = str(side).strip().upper()
    if normalized not in {"L", "R"}:
        raise ValueError(f"unknown wheel side {side!r}")
    side_name = "left" if normalized == "L" else "right"
    body = f"upright_{normalized}"
    pose = state.pose(body)
    rotation = np.asarray(pose.rotation, dtype=float)
    origin = np.asarray(pose.translation, dtype=float)
    center_local = np.asarray(assembly.point(body, "wheel_center"), dtype=float)
    center = origin + rotation @ center_local
    outward = -1.0 if normalized == "L" else 1.0
    lateral_axis_y = float(rotation[1, 1])
    camber_deg = -outward * float(
        np.degrees(np.arctan2(float(rotation[2, 1]), lateral_axis_y))
    )
    toe_deg = -outward * float(
        np.degrees(np.arctan2(float(rotation[0, 1]), lateral_axis_y))
    )
    return {
        f"{side_name}_wheel_center_x": float(center[0]),
        f"{side_name}_wheel_center_y": float(center[1]),
        f"{side_name}_wheel_center_z": float(center[2]),
        f"{side_name}_camber_deg": camber_deg,
        f"{side_name}_toe_deg": toe_deg,
    }


def compute_k_metrics(state: Any, assembly: Any) -> dict[str, float]:
    """Compute K&C geometry metrics with the established sign conventions."""
    metrics: dict[str, float] = {}
    metrics.update(wheel_metrics(state, assembly, "L"))
    metrics.update(wheel_metrics(state, assembly, "R"))
    metrics["track_mm"] = (
        metrics["right_wheel_center_y"] - metrics["left_wheel_center_y"]
    )
    metrics["wheel_center_z_mean"] = 0.5 * (
        metrics["left_wheel_center_z"] + metrics["right_wheel_center_z"]
    )
    metrics["wheel_center_z_difference"] = (
        metrics["right_wheel_center_z"] - metrics["left_wheel_center_z"]
    )
    metrics["camber_deg_difference"] = (
        metrics["right_camber_deg"] - metrics["left_camber_deg"]
    )
    metrics["toe_deg_difference"] = metrics["right_toe_deg"] - metrics["left_toe_deg"]
    return metrics


def _summarize_series_values(values: list[float]) -> dict[str, float]:
    array = np.asarray(values, dtype=float)
    return {
        "initial": float(array[0]),
        "final": float(array[-1]),
        "minimum": float(np.min(array)),
        "maximum": float(np.max(array)),
        "rms": float(np.sqrt(np.mean(np.square(array)))),
    }


def _kc_quasi_static_metrics(*args: Any, **kwargs: Any) -> dict[str, Any]:
    """Serve both single K&C states and time-series replay results."""
    if len(args) >= 2:
        return compute_k_metrics(args[0], args[1])
    if len(args) != 1 or kwargs:
        raise TypeError("kc_quasi_static metrics require a state/assembly or one result")
    result = args[0]
    samples = sorted(
        (sample for sample in getattr(result, "samples", ()) if sample.body == "axle"),
        key=lambda sample: sample.time,
    )
    if not samples:
        return {
            "status": "unavailable",
            "family": "kc_quasi_static",
            "reason": "no axle replay samples",
            "metrics": {},
        }
    names = sorted({key for sample in samples for key in sample.metrics})
    summaries: dict[str, dict[str, float]] = {}
    for name in names:
        values: list[float] = []
        for sample in samples:
            value = sample.metrics.get(name)
            if isinstance(value, (int, float, np.number)) and np.isfinite(float(value)):
                values.append(float(value))
        if len(values) == len(samples):
            summaries[name] = _summarize_series_values(values)
    return {
        "status": str(getattr(result, "status", "success")),
        "family": "kc_quasi_static",
        "sample_count": len(samples),
        "metrics": summaries,
    }


def _axle_dynamic_metrics(result: Any) -> dict[str, Any]:
    from .axle import compute_axle_metrics

    return compute_axle_metrics(result)


def _vehicle_dynamic_metrics(result: Any) -> dict[str, Any]:
    from .vehicle import compute_vehicle_metrics

    return compute_vehicle_metrics(result)


def _register_defaults() -> None:
    register_case_metric("kc_quasi_static", _kc_quasi_static_metrics)
    register_case_metric("axle_dynamic", _axle_dynamic_metrics)
    register_case_metric("vehicle_dynamic", _vehicle_dynamic_metrics)
    for family in (
        "vehicle_kc",
        "vehicle_kc_dynamic",
        "handling",
        "ride_four_post",
        "ride_random_road",
    ):
        register_case_metric(
            family,
            lambda result=None, _family=family: not_applicable_metrics(
                result, family=_family
            ),
        )


_register_defaults()


__all__ = [
    "case_metric_for",
    "compute_case_metrics",
    "compute_k_metrics",
    "not_applicable_metrics",
    "register_case_metric",
    "registered_case_metric_families",
    "wheel_metrics",
]
