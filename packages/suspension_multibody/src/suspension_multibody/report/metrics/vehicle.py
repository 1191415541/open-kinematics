"""
General metrics for explicit multibody vehicle results.

Moved here from ``metrics/vehicle.py``, which 08 deletes.  The wall-time key is
read off the decoded result; nothing here reaches into the kernel.

The wheel-load table is **not** defined here.  ``report.wheel_loads`` owns it --
it is generated there from the placements the run's own wheel ends state, so a
three-axle vehicle gets ``normal_load_axle_middle`` without this module naming an
axle -- and this module publishes that one table rather than a second copy of it.
The historical aggregate spellings (``normal_load_front_axle``,
``load_transfer_front_minus_rear``, ...) are part of that table's output, so the
values here are unchanged on a four-wheel vehicle.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np

from ..wheel_loads import wheel_load_channels
from .axle import compute_axle_metrics
from .common import peak, rms


def wheel_load_metrics(loads: Mapping[str, float]) -> dict[str, float]:
    """Return the wheel-load channel table for however many wheel ends there are."""
    return wheel_load_channels(loads)


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
