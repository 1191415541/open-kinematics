"""Adapters from the real full-vehicle solver to Adams time-history channels."""

from __future__ import annotations

import math
from typing import Any, Literal

import numpy as np

from ..results.envelope import ResultEnvelope
from .time_domain import TimeHistory

#: The steering actuator kinds the vehicle schema declares.
#:
#: Repeated as a local alias rather than imported from the schema package: the
#: caller passes one value out of this set, so a fourth schema kind shows up as a
#: ``ty`` error at every call site instead of as a defaulted conversion here.
SteeringActuatorMode = Literal[
    "rack_translation",
    "prescribed_rotation",
    "prescribed_translation",
]


def full_vehicle_time_history(
    run: ResultEnvelope,
    category: Literal["handling_stability", "ride"],
    *,
    chassis_body_id: str,
    steering_ratio_m_per_rad: float | None = None,
    steering_channel: str | None = None,
    steering_actuator_mode: SteeringActuatorMode = "rack_translation",
    chassis_center_of_mass_m: tuple[float, float, float] | None = None,
) -> TimeHistory:
    """
    Export a full-vehicle run using the existing Adams channel contract.

    ``steering_channel`` is the name the run addresses its steering actuator by,
    and ``steering_actuator_mode`` is what the caller declared that actuator to
    be.  Both are required for a ``handling_stability`` history and are taken
    from the case's own ``vehicle.steering`` declaration; neither is inferred.
    """
    return _native_vehicle_time_history(
        run,
        category,
        chassis_body_id=chassis_body_id,
        steering_ratio_m_per_rad=steering_ratio_m_per_rad,
        steering_channel=steering_channel,
        steering_actuator_mode=steering_actuator_mode,
        chassis_center_of_mass_m=chassis_center_of_mass_m,
    )


def _native_vehicle_time_history(
    run: Any,
    category: Literal["handling_stability", "ride"],
    *,
    chassis_body_id: str,
    steering_ratio_m_per_rad: float | None,
    steering_channel: str | None,
    steering_actuator_mode: SteeringActuatorMode,
    chassis_center_of_mass_m: tuple[float, float, float] | None,
) -> TimeHistory:
    """Convert native COM states to the legacy Adams scalar channels."""
    times = np.asarray(run.times_s, dtype=float)
    if times.ndim != 1 or len(times) < 2:
        raise ValueError("full-vehicle run requires at least two samples")
    chassis = np.asarray(run.body_state(chassis_body_id), dtype=float)
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
        if steering_channel is None or not steering_channel.strip():
            raise ValueError(
                "native handling history requires the declared steering channel name"
            )
        # The conversion follows the *declared* actuator mode, not the channel
        # name.  Since p2-06 every steering actuator -- prescribed rotation
        # included -- is addressed by its declared ``channel_name``, so both kinds
        # answer to one name and a name lookup can no longer tell them apart.
        # Adams MOTION/4 prescribes the steering wheel angle directly; reading
        # that angle back through the rack ratio scales it by 1/ratio.
        prescribed_rotation = steering_actuator_mode == "prescribed_rotation"
        rack_ratio_m_per_rad = 0.0
        if not prescribed_rotation:
            if steering_ratio_m_per_rad is None or steering_ratio_m_per_rad <= 0.0:
                raise ValueError(
                    "native handling history requires a positive steering ratio "
                    "in m/rad"
                )
            rack_ratio_m_per_rad = float(steering_ratio_m_per_rad)
        try:
            steering = np.asarray(run.element_state(steering_channel), dtype=float)
        except KeyError as exc:
            raise ValueError(
                f"native handling history has no steering channel "
                f"{steering_channel!r}"
            ) from exc
        if steering.shape != (len(times), 4):
            raise ValueError("native steering output has an invalid shape")
        # Column 2 is the actuator target in its own coordinate: a rack
        # translation in metres, or a prescribed rotation in radians.
        steering_angle = (
            steering[:, 2]
            if prescribed_rotation
            else steering[:, 2] / rack_ratio_m_per_rad
        )
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
