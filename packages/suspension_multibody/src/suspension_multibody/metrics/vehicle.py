"""General metrics for explicit multibody vehicle results."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from .axle import compute_axle_metrics
from .common import peak, rms

_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


def wheel_load_metrics(loads: Mapping[str, float]) -> dict[str, float]:
    """Return stable derived metrics for four finite vehicle wheel loads."""
    missing = set(_WHEELS) - set(loads)
    extra = set(loads) - set(_WHEELS)
    if missing or extra:
        raise ValueError("wheel loads must contain exactly the four vehicle corners")
    values = {name: float(loads[name]) for name in _WHEELS}
    if any(not np.isfinite(value) for value in values.values()):
        raise ValueError("wheel loads must be finite")
    front = values["front_left"] + values["front_right"]
    rear = values["rear_left"] + values["rear_right"]
    left = values["front_left"] + values["rear_left"]
    right = values["front_right"] + values["rear_right"]
    return {
        **{f"normal_load_{name}": value for name, value in values.items()},
        "normal_load_total": front + rear,
        "normal_load_front_axle": front,
        "normal_load_rear_axle": rear,
        "normal_load_left_side": left,
        "normal_load_right_side": right,
        "load_transfer_front_minus_rear": front - rear,
        "load_transfer_right_minus_left": right - left,
    }


def _embedded_wheel_loads(result: Any) -> Mapping[str, float] | None:
    """Read explicit wheel-load evidence without inventing missing values."""
    for attribute in ("wheel_loads", "static_wheel_loads"):
        value = getattr(result, attribute, None)
        if value is None:
            continue
        if hasattr(value, "wheel_loads"):
            value = value.wheel_loads
        if isinstance(value, Mapping):
            return value
    return None


def compute_vehicle_metrics(
    result: Any,
    *,
    wheel_loads: Mapping[str, float] | None = None,
) -> dict[str, Any]:
    """Combine axle, steering, performance, and explicit wheel-load metrics."""
    axle = getattr(result, "axle", result)
    metrics = {f"axle_{key}": value for key, value in compute_axle_metrics(axle).items()}
    steering = getattr(result, "steering_output", None)
    if steering is not None:
        values = np.asarray(steering, dtype=float)
        finite_values = values[np.isfinite(values)]
        if finite_values.size:
            metrics["maximum_steering_output"] = peak(finite_values)
            metrics["rms_steering_output"] = rms(finite_values)
    explicit_loads = wheel_loads if wheel_loads is not None else _embedded_wheel_loads(result)
    if explicit_loads is not None:
        metrics.update(wheel_load_metrics(explicit_loads))
    wall_time = getattr(result, "native_kernel_wall_time_s", None)
    if wall_time is not None:
        metrics["native_kernel_wall_time_s"] = float(wall_time)
    return metrics


def vehicle_metrics(result: Any) -> dict[str, Any]:
    """Short alias for ``compute_vehicle_metrics``."""
    return compute_vehicle_metrics(result)


__all__ = ["compute_vehicle_metrics", "vehicle_metrics", "wheel_load_metrics"]
