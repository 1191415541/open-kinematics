"""Adapters from the real full-vehicle solver to Adams time-history channels."""

from __future__ import annotations

import math
from typing import Any, Literal

import numpy as np

from .time_domain import TimeHistory


def full_vehicle_time_history(
    run: Any,
    category: Literal["handling_stability", "ride"],
    *,
    steering_ratio_m_per_rad: float | None = None,
    chassis_center_of_mass_m: tuple[float, float, float] | None = None,
) -> TimeHistory:
    """Export a full-vehicle run using the existing Adams channel contract."""
    if hasattr(run, "times_s") and hasattr(run, "body_state"):
        return _native_vehicle_time_history(
            run,
            category,
            steering_ratio_m_per_rad=steering_ratio_m_per_rad,
            chassis_center_of_mass_m=chassis_center_of_mass_m,
        )
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


def _native_vehicle_time_history(
    run: Any,
    category: Literal["handling_stability", "ride"],
    *,
    steering_ratio_m_per_rad: float | None,
    chassis_center_of_mass_m: tuple[float, float, float] | None,
) -> TimeHistory:
    """Convert native COM states to the legacy Adams scalar channels."""
    times = np.asarray(run.times_s, dtype=float)
    if times.ndim != 1 or len(times) < 2:
        raise ValueError("full-vehicle run requires at least two samples")
    chassis = np.asarray(run.body_state("chassis"), dtype=float)
    if chassis.shape != (len(times), 19):
        raise ValueError("native chassis state has an invalid shape")
    rotations = np.empty((len(times), 3, 3), dtype=float)
    euler = np.empty((len(times), 3), dtype=float)
    for index, state in enumerate(chassis):
        quaternion = state[3:7]
        norm = float(np.linalg.norm(quaternion))
        if not math.isfinite(norm) or norm <= 1e-12:
            raise ValueError("native chassis state contains an invalid quaternion")
        w, x, y, z = quaternion / norm
        rotations[index] = np.array(
            (
                (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
                (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
                (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
            ),
            dtype=float,
        )
        euler[index] = (
            math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))),
            math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
        )

    angular_velocity = chassis[:, 10:13]
    acceleration = chassis[:, 13:16]
    acceleration_body = np.einsum(
        "nji,nj->ni", rotations, acceleration, optimize=True
    )
    velocity_body = np.einsum(
        "nji,nj->ni", rotations, chassis[:, 7:10], optimize=True
    )
    initial_forward_velocity = float(velocity_body[0, 0])
    vehicle_axis_sign = -1.0 if initial_forward_velocity < 0.0 else 1.0
    yaw_rate_body = np.einsum(
        "nji,nj->ni", rotations, angular_velocity, optimize=True
    )[:, 2]

    center = (
        np.asarray(chassis_center_of_mass_m, dtype=float)
        if chassis_center_of_mass_m is not None
        else np.zeros(3, dtype=float)
    )
    center_world = np.einsum("nij,j->ni", rotations, center, optimize=True)
    position = chassis[:, :3] - center_world
    angular_acceleration = chassis[:, 16:19]
    origin_acceleration = acceleration.copy()
    for index in range(len(times)):
        origin_acceleration[index] -= np.cross(
            angular_acceleration[index], center_world[index]
        )
        origin_acceleration[index] -= np.cross(
            angular_velocity[index],
            np.cross(angular_velocity[index], center_world[index]),
        )

    channels: dict[str, tuple[float, ...]]
    units: dict[str, str]
    if category == "handling_stability":
        if steering_ratio_m_per_rad is None or steering_ratio_m_per_rad <= 0.0:
            raise ValueError(
                "native handling history requires a positive steering ratio in m/rad"
            )
        try:
            steering = np.asarray(run.steering_state("front_rack"), dtype=float)
            steering_angle = steering[:, 2] / steering_ratio_m_per_rad
        except KeyError:
            # Adams MOTION/4 直接规定方向盘转角，不能再把它解释为齿条位移。
            steering = np.asarray(run.steering_state("steering_input"), dtype=float)
            steering_angle = steering[:, 2]
        if steering.shape != (len(times), 4):
            raise ValueError("native steering output has an invalid shape")
        channels = {
            "steering_angle": tuple(steering_angle),
            "lateral_acceleration": tuple(
                vehicle_axis_sign * acceleration_body[:, 1] * 1000.0
            ),
            "yaw_rate": tuple(yaw_rate_body),
            "body_roll": tuple(euler[:, 0]),
        }
        units = {
            "steering_angle": "rad",
            "lateral_acceleration": "mm/s^2",
            "yaw_rate": "rad/s",
            "body_roll": "rad",
        }
    elif category == "ride":
        channels = {
            "body_heave": tuple(position[:, 2] * 1000.0),
            "body_pitch": tuple(euler[:, 1]),
            "body_roll": tuple(euler[:, 0]),
            "body_accel_z": tuple(origin_acceleration[:, 2] * 1000.0),
        }
        units = {
            "body_heave": "mm",
            "body_pitch": "rad",
            "body_roll": "rad",
            "body_accel_z": "mm/s^2",
        }
    else:
        raise ValueError(f"unsupported full-vehicle history category: {category}")
    return TimeHistory(
        time=tuple(float(value) for value in times),
        channels=channels,
        units=units,
    )
