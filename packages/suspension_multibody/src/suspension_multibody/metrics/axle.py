"""General metrics for typed single-axle results."""

from __future__ import annotations

from typing import Any

import numpy as np

from .common import contact_metrics, convergence_metrics, peak, rms, time_metrics

_TIRE_NORMAL = 4
_TIRE_LONGITUDINAL = 5
_TIRE_LATERAL = 6


def _tire_column(result: Any, column: int) -> np.ndarray | None:
    values = getattr(result, "tire_output", None)
    if values is None:
        return None
    array = np.asarray(values, dtype=float)
    if array.ndim != 3 or array.shape[2] <= column or array.shape[1] == 0:
        return None
    return array[:, :, column]


def compute_axle_metrics(result: Any) -> dict[str, Any]:
    """Return stable aggregate metrics without changing raw axle output."""
    times = np.asarray(getattr(result, "times_s", ()), dtype=float).reshape(-1)
    performance = getattr(result, "performance", None)
    performance_available = bool(
        performance is not None
        and bool(
            getattr(
                performance,
                "available",
                performance.get("available", False)
                if isinstance(performance, dict)
                else False,
            )
        )
    )
    if times.size == 0:
        return {
            "status": "unavailable",
            "reason": "no completed samples",
            "sample_count": 0,
            "performance_available": performance_available,
            "convergence_available": False,
            "contact_available": False,
        }
    metrics: dict[str, Any] = dict(time_metrics(result))
    for name, column in (
        ("normal_force_n", _TIRE_NORMAL),
        ("longitudinal_force_n", _TIRE_LONGITUDINAL),
        ("lateral_force_n", _TIRE_LATERAL),
    ):
        values = _tire_column(result, column)
        if values is None:
            continue
        metrics[f"maximum_{name}"] = peak(values)
        metrics[f"rms_{name}"] = rms(values)
    metrics["performance_available"] = performance_available
    metrics.update({f"convergence_{key}": value for key, value in convergence_metrics(result).items()})
    metrics.update({f"contact_{key}": value for key, value in contact_metrics(result).items()})
    return metrics


def axle_metrics(result: Any) -> dict[str, Any]:
    """Short alias for ``compute_axle_metrics``."""
    return compute_axle_metrics(result)


__all__ = ["axle_metrics", "compute_axle_metrics"]
