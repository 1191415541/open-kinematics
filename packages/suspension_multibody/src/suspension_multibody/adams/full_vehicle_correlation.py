"""Adapters from the real full-vehicle solver to Adams time-history channels."""

from __future__ import annotations

from typing import Any, Literal

from .time_domain import TimeHistory


def full_vehicle_time_history(
    run: Any,
    category: Literal["handling_stability", "ride"],
) -> TimeHistory:
    """Export the full solver's body and tire states using Adams channel names."""
    if len(run.samples) < 2:
        raise ValueError("full-vehicle run requires at least two samples")
    names = (
        (
            "steering_angle",
            "lateral_acceleration",
            "yaw_rate",
            "body_roll",
        )
        if category == "handling_stability"
        else (
            "body_heave",
            "body_pitch",
            "body_roll",
            "body_accel_z",
        )
    )
    all_units = {
        "steering_angle": "rad",
        "lateral_acceleration": "mm/s^2",
        "yaw_rate": "rad/s",
        "body_roll": "rad",
        "body_heave": "mm",
        "body_pitch": "rad",
        "body_accel_z": "mm/s^2",
    }
    return TimeHistory(
        time=tuple(sample.time for sample in run.samples),
        channels={
            name: tuple(sample.metrics[name] for sample in run.samples) for name in names
        },
        units={name: all_units[name] for name in names},
    )
