"""Shared geometry conversion helpers for the native K&C contract."""

from __future__ import annotations

import numpy as np

from ...core.constraints import BallJoint, RevoluteJoint

MM = 1e-3


class NativeKcError(RuntimeError):
    """Raised when an assembly cannot be expressed for the native kernel."""


def rotation_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return the scalar-first quaternion for a rotation matrix (Shepperd)."""
    m = np.asarray(rotation, dtype=float)
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w, x, y, z = (
            0.25 * s,
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
        )
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (
            (m[2, 1] - m[1, 2]) / s,
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
        )
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (
            (m[0, 2] - m[2, 0]) / s,
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
        )
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (
            (m[1, 0] - m[0, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 2] + m[2, 1]) / s,
            0.25 * s,
        )
    quaternion = np.array([w, x, y, z], dtype=float)
    return tuple(float(v) for v in quaternion / np.linalg.norm(quaternion))


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    """Return the rotation matrix for a scalar-first quaternion."""
    w, x, y, z = (float(v) for v in quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _local_point(world: np.ndarray, body) -> np.ndarray:
    rotation = np.asarray(body.pose.rotation, dtype=float)
    offset = np.asarray(world, dtype=float) - np.asarray(body.pose.translation, dtype=float)
    return rotation.T @ offset


def _local_axis(world_axis: np.ndarray, body) -> np.ndarray:
    rotation = np.asarray(body.pose.rotation, dtype=float)
    axis = rotation.T @ np.asarray(world_axis, dtype=float)
    return axis / np.linalg.norm(axis)


def collapse_spherical_pairs(constraints):
    """
    Fold two spherical joints on the same body pair into one revolute joint.

    Two coincident-point constraints between the same two rigid bodies remove the
    same five degrees of freedom as a revolute joint about the line through the
    two points, but with six rows of rank five. The kernel requires independent
    constraint rows, so the contract model uses the equivalent full-rank form.
    """
    grouped: dict[tuple[str, str], list] = {}
    others = []
    for constraint in constraints:
        if isinstance(constraint, BallJoint):
            grouped.setdefault((constraint.body_a, constraint.body_b), []).append(constraint)
        else:
            others.append(constraint)

    joints = []
    for group in grouped.values():
        if len(group) == 1:
            joints.append(group[0])
            continue
        if len(group) != 2:
            raise NativeKcError(f"{len(group)} spherical joints share one body pair")
        first, second = group
        axis = np.asarray(second.point_a, dtype=float) - np.asarray(first.point_a, dtype=float)
        if np.linalg.norm(axis) <= 1e-9:
            raise NativeKcError("two mount points coincide; cannot form an axis")
        joints.append(
            RevoluteJoint(
                first.body_a,
                first.point_a,
                axis,
                first.body_b,
                first.point_b,
                axis,
                name=f"collapsed_{first.name}",
            )
        )
    return tuple(joints + others)
