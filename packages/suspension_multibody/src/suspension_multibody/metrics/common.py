"""Small, semantics-neutral metric primitives."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np


def _finite_array(values: Any, *, name: str = "values") -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.size == 0:
        raise ValueError(f"{name} must not be empty")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def peak(values: Any, *, absolute: bool = True) -> float:
    """Return the maximum value or maximum absolute value."""
    array = _finite_array(values)
    return float(np.max(np.abs(array) if absolute else array))


def rms(values: Any) -> float:
    """Return the root-mean-square value of a finite sample array."""
    array = _finite_array(values)
    return float(np.sqrt(np.mean(np.square(array))))


def residual_norm(values: Any, *, axis: int = -1) -> np.ndarray:
    """Return Euclidean residual norms along one sample axis."""
    array = _finite_array(values)
    return np.linalg.norm(array, axis=axis)


def time_metrics(result: Any) -> dict[str, float | int]:
    """Summarise the time grid shared by raw and typed result objects."""
    times = _finite_array(result.times_s, name="result.times_s").reshape(-1)
    return {
        "sample_count": int(times.size),
        "start_s": float(times[0]),
        "end_s": float(times[-1]),
        "duration_s": float(times[-1] - times[0]),
    }


def convergence_metrics(result: Any) -> Mapping[str, float | int | bool]:
    """Summarise accepted samples without changing producer diagnostics semantics."""
    diagnostics = getattr(result, "diagnostics", None)
    if diagnostics is None:
        return {"available": False}
    if hasattr(diagnostics, "accepted"):
        accepted = np.asarray(diagnostics.accepted, dtype=bool)
        rejected = np.asarray(diagnostics.rejected_attempts, dtype=float)
        newton = np.asarray(diagnostics.newton_iterations, dtype=float)
    else:
        rows = np.asarray(diagnostics, dtype=float)
        if rows.ndim != 2 or rows.shape[1] < 4:
            return {"available": False}
        accepted = rows[:, 0] > 0.5
        rejected = rows[:, 2]
        newton = rows[:, 3]
    if accepted.size == 0:
        return {"available": False}
    return {
        "available": True,
        "accepted_fraction": float(np.mean(accepted)),
        "accepted_sample_count": int(np.count_nonzero(accepted)),
        "rejected_attempt_count": int(np.sum(rejected)),
        "maximum_newton_iterations": int(np.max(newton)),
    }


def contact_metrics(result: Any) -> Mapping[str, float | int | bool]:
    """Summarise contact activity when the result exposes that diagnostic."""
    diagnostics = getattr(result, "diagnostics", None)
    if diagnostics is None:
        return {"available": False}
    if hasattr(diagnostics, "active_contacts"):
        active = np.asarray(diagnostics.active_contacts, dtype=float)
        events = np.asarray(diagnostics.contact_events, dtype=float)
    else:
        rows = np.asarray(diagnostics, dtype=float)
        if rows.ndim != 2 or rows.shape[1] < 12:
            return {"available": False}
        active = rows[:, 10]
        events = rows[:, 11]
    return {
        "available": True,
        "maximum_active_contact_count": int(np.max(active)) if active.size else 0,
        "contact_event_count": int(np.sum(events)) if events.size else 0,
    }


def compute_common_metrics(result: Any) -> dict[str, Any]:
    """Return result-level common metrics without fabricating samples."""
    times = np.asarray(getattr(result, "times_s", ()), dtype=float).reshape(-1)
    if times.size == 0:
        return {
            "status": "unavailable",
            "reason": "no completed samples",
            "sample_count": 0,
            "convergence_available": False,
            "contact_available": False,
        }
    metrics: dict[str, Any] = dict(time_metrics(result))
    metrics["status"] = str(getattr(result, "status", "success"))
    metrics.update(
        {f"convergence_{key}": value for key, value in convergence_metrics(result).items()}
    )
    metrics.update(
        {f"contact_{key}": value for key, value in contact_metrics(result).items()}
    )
    return metrics
__all__ = [
    "compute_common_metrics",
    "contact_metrics",
    "convergence_metrics",
    "peak",
    "residual_norm",
    "rms",
    "time_metrics",
]
