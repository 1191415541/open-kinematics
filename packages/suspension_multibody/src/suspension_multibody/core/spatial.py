"""Spatial algebra and SE(3) operations used by the quasi-static solver."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

Array = np.ndarray


def skew(vector: Array) -> Array:
    """Return the cross-product matrix for a three-vector."""
    x, y, z = np.asarray(vector, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def cross3(first: Array, second: Array) -> Array:
    """Return the cross product of two three-vectors without axis dispatch."""
    ax, ay, az = first
    bx, by, bz = second
    return np.array(
        (ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx),
        dtype=float,
    )


def normalize_quaternion(quaternion: Iterable[float]) -> Array:
    """Normalize a scalar-first quaternion and reject degenerate input."""
    q = np.asarray(tuple(quaternion), dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        raise ValueError("quaternion must contain four finite values")
    norm_squared = float(q @ q)
    if norm_squared < 1e-28:
        raise ValueError("quaternion norm is zero")
    if abs(norm_squared - 1.0) <= 1e-14:
        return q.copy()
    return q / math.sqrt(norm_squared)


def quaternion_multiply(first: Array, second: Array) -> Array:
    """Multiply scalar-first quaternions."""
    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=float,
    )


def quaternion_conjugate(quaternion: Array) -> Array:
    """Return a scalar-first quaternion conjugate."""
    q = np.asarray(quaternion, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quaternion_to_matrix(quaternion: Array) -> Array:
    """Convert a unit quaternion to a proper rotation matrix."""
    return _quaternion_to_matrix_unit(normalize_quaternion(quaternion))


def _quaternion_to_matrix_unit(quaternion: Array) -> Array:
    """Convert an already normalized quaternion without revalidation."""
    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def rotation_vector_to_quaternion(rotation_vector: Array) -> Array:
    """Map a rotation vector to a unit quaternion with a small-angle branch."""
    vector = np.asarray(rotation_vector, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("rotation vector must contain three finite values")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-10:
        half = 0.5 * vector
        return normalize_quaternion(np.array([1.0, half[0], half[1], half[2]]))
    half_angle = 0.5 * angle
    scale = math.sin(half_angle) / angle
    return np.array([math.cos(half_angle), *(scale * vector)], dtype=float)


def _rotation_vector_to_quaternion_unchecked(rotation_vector: Array) -> Array:
    """Convert a trusted six-DoF trial increment without repeated validation."""
    vector = rotation_vector
    angle_squared = float(vector @ vector)
    if angle_squared < 1e-20:
        half = 0.5 * vector
        quaternion = np.array([1.0, half[0], half[1], half[2]], dtype=float)
        return quaternion / math.sqrt(float(quaternion @ quaternion))
    angle = math.sqrt(angle_squared)
    half_angle = 0.5 * angle
    scale = math.sin(half_angle) / angle
    return np.array(
        [math.cos(half_angle), scale * vector[0], scale * vector[1], scale * vector[2]],
        dtype=float,
    )


def quaternion_to_rotation_vector(quaternion: Array) -> Array:
    """Map a unit quaternion to its principal rotation vector."""
    q = normalize_quaternion(quaternion)
    if q[0] < 0:
        q = -q
    vector_norm = float(np.linalg.norm(q[1:]))
    if vector_norm < 1e-12:
        return 2.0 * q[1:]
    angle = 2.0 * math.atan2(vector_norm, q[0])
    return q[1:] * (angle / vector_norm)


@dataclass(frozen=True)
class SE3:
    """Rigid pose with translation in global coordinates and unit quaternion."""

    translation: Array
    quaternion: Array
    _rotation_matrix: Array = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        translation = np.asarray(self.translation, dtype=float)
        quaternion = normalize_quaternion(self.quaternion)
        if translation.shape != (3,) or not np.all(np.isfinite(translation)):
            raise ValueError("translation must contain three finite values")
        object.__setattr__(self, "translation", translation.copy())
        object.__setattr__(self, "quaternion", quaternion)
        rotation = _quaternion_to_matrix_unit(quaternion)
        rotation.setflags(write=False)
        object.__setattr__(self, "_rotation_matrix", rotation)

    @classmethod
    def identity(cls) -> SE3:
        """Create the identity pose."""
        return cls(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))

    @property
    def rotation(self) -> Array:
        """Return the global rotation matrix."""
        return self._rotation_matrix

    def inverse(self) -> SE3:
        """Return the inverse transform."""
        rotation = self.rotation
        inverse_rotation = rotation.T
        return SE3(
            -inverse_rotation @ self.translation, quaternion_conjugate(self.quaternion)
        )

    def compose(self, other: SE3) -> SE3:
        """Compose this transform with ``other``."""
        return SE3(
            self.translation + self.rotation @ other.translation,
            quaternion_multiply(self.quaternion, other.quaternion),
        )

    def transform_point(self, point_local: Array) -> Array:
        """Transform a local point to global coordinates."""
        point = np.asarray(point_local, dtype=float)
        if point.shape != (3,):
            raise ValueError("point must contain three values")
        return self.translation + self.rotation @ point

    def retract(self, increment: Array) -> SE3:
        """Apply a local 6D ``(translation, rotation-vector)`` increment."""
        delta = np.asarray(increment, dtype=float)
        if delta.shape != (6,) or not np.all(np.isfinite(delta)):
            raise ValueError("SE(3) increment must contain six finite values")
        delta_rotation = rotation_vector_to_quaternion(delta[3:])
        return SE3(
            self.translation + self.rotation @ delta[:3],
            quaternion_multiply(self.quaternion, delta_rotation),
        )

    @classmethod
    def _from_unchecked(cls, translation: Array, quaternion: Array) -> SE3:
        """Build a pose from trusted finite data produced by the integrator."""
        pose = object.__new__(cls)
        object.__setattr__(pose, "translation", translation)
        object.__setattr__(pose, "quaternion", quaternion)
        rotation = _quaternion_to_matrix_unit(quaternion)
        rotation.setflags(write=False)
        object.__setattr__(pose, "_rotation_matrix", rotation)
        return pose

    def _retract_unchecked(self, increment: Array) -> SE3:
        """Apply a trusted local increment without public API validation."""
        delta_rotation = _rotation_vector_to_quaternion_unchecked(increment[3:])
        return SE3._from_unchecked(
            self.translation + self.rotation @ increment[:3],
            quaternion_multiply(self.quaternion, delta_rotation),
        )

    def local_coordinates(self, other: SE3) -> Array:
        """Return the local increment taking this pose to ``other``."""
        relative = self.inverse().compose(other)
        return np.concatenate(
            (relative.translation, quaternion_to_rotation_vector(relative.quaternion))
        )


def wrench_local_to_global(pose: SE3, wrench_local: Array) -> Array:
    """Transform a force/moment wrench from a body origin to global origin."""
    wrench = np.asarray(wrench_local, dtype=float)
    if wrench.shape != (6,):
        raise ValueError("wrench must contain six values")
    force = pose.rotation @ wrench[:3]
    moment = pose.rotation @ wrench[3:] + cross3(pose.translation, force)
    return np.concatenate((force, moment))


def wrench_global_to_local(pose: SE3, wrench_global: Array) -> Array:
    """Transform a global-origin wrench to the body-origin local frame."""
    wrench = np.asarray(wrench_global, dtype=float)
    if wrench.shape != (6,):
        raise ValueError("wrench must contain six values")
    force_local = pose.rotation.T @ wrench[:3]
    moment_local = pose.rotation.T @ (
        wrench[3:] - cross3(pose.translation, wrench[:3])
    )
    return np.concatenate((force_local, moment_local))


def twist_local_to_global(pose: SE3, twist_local: Array) -> Array:
    """Transform a body-origin twist to a global-origin twist."""
    twist = np.asarray(twist_local, dtype=float)
    if twist.shape != (6,):
        raise ValueError("twist must contain six values")
    angular = pose.rotation @ twist[3:]
    linear = pose.rotation @ twist[:3] + cross3(pose.translation, angular)
    return np.concatenate((linear, angular))


def wrench_matrix(pose: SE3) -> Array:
    """Return the matrix implementing :func:`wrench_local_to_global`."""
    rotation = pose.rotation
    return np.block(
        [[rotation, np.zeros((3, 3))], [skew(pose.translation) @ rotation, rotation]]
    )


def wrench_translation_tangent(force_global: Array) -> Array:
    """Return ``d(moment)/d(translation)`` for a fixed global force."""
    force = np.asarray(force_global, dtype=float)
    if force.shape != (3,):
        raise ValueError("force must contain three values")
    return np.vstack((np.zeros((3, 3)), -skew(force)))
