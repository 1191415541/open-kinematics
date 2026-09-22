"""
Wheel-load aggregation for report diagnostics.

Moved here from ``analysis/vehicle_physics.py``, which 08 keeps only for the A2
static wheel-load solve: summing four corner loads into axle and side totals is a
statement about a decoded result, not a physical law.  The sign conventions are
unchanged.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass(frozen=True)
class WheelLoadSummary:
    """Aggregated wheel normal loads with explicit sign conventions."""

    wheel_loads: dict[str, float]
    total: float
    front_axle: float
    rear_axle: float
    left_side: float
    right_side: float
    front_rear_delta: float
    right_left_delta: float


def summarize_wheel_loads(loads: Mapping[str, float]) -> WheelLoadSummary:
    """Aggregate four wheel loads; ``front_rear_delta`` is front minus rear."""
    missing = set(_WHEELS) - set(loads)
    if missing or set(loads) - set(_WHEELS):
        raise ValueError("wheel loads must contain exactly the four vehicle corners")
    values = {name: float(loads[name]) for name in _WHEELS}
    if any(not np.isfinite(value) for value in values.values()):
        raise ValueError("wheel loads must be finite")
    front = values["front_left"] + values["front_right"]
    rear = values["rear_left"] + values["rear_right"]
    left = values["front_left"] + values["rear_left"]
    right = values["front_right"] + values["rear_right"]
    return WheelLoadSummary(
        wheel_loads=values,
        total=front + rear,
        front_axle=front,
        rear_axle=rear,
        left_side=left,
        right_side=right,
        front_rear_delta=front - rear,
        right_left_delta=right - left,
    )


__all__ = ["WheelLoadSummary", "summarize_wheel_loads"]
