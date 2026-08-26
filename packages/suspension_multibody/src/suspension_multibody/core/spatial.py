"""Spatial algebra and SE(3) operations used by the quasi-static solver."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

import numpy as np

try:
    from numba import njit as _njit

    _HAVE_NUMBA = True
except ImportError:  # pragma: no cover - numba is an optional acceleration
    _HAVE_NUMBA = False

    def _njit(*_args, **_kwargs):  # type: ignore[misc]
        def _decorator(func):
            return func

        return _decorator


Array = np.ndarray


@_njit(nogil=True, fastmath=True)
def _skew_numba(x: float, y: float, z: float) -> Array:
    result = np.empty((3, 3))
    result[0, 0] = 0.0
    result[0, 1] = -z
    result[0, 2] = y
    result[1, 0] = z
    result[1, 1] = 0.0
    result[1, 2] = -x
    result[2, 0] = -y
    result[2, 1] = x
    result[2, 2] = 0.0
    return result


def skew(vector: Array) -> Array:
    """Return the cross-product matrix for a three-vector."""
    x, y, z = np.asarray(vector, dtype=float)
    return _skew_numba(x, y, z)


@_njit(nogil=True, fastmath=True)
def _cross3_numba(ax: float, ay: float, az: float, bx: float, by: float, bz: float) -> Array:
    result = np.empty(3)
    result[0] = ay * bz - az * by
    result[1] = az * bx - ax * bz
    result[2] = ax * by - ay * bx
    return result


def cross3(first: Array, second: Array) -> Array:
    """Return the cross product of two three-vectors without axis dispatch."""
    ax, ay, az = first
    bx, by, bz = second
    return _cross3_numba(ax, ay, az, bx, by, bz)


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


@_njit(nogil=True, fastmath=True)
def _quaternion_multiply_numba(
    w1: float, x1: float, y1: float, z1: float,
    w2: float, x2: float, y2: float, z2: float,
) -> Array:
    result = np.empty(4)
    result[0] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    result[1] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    result[2] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    result[3] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return result


def quaternion_multiply(first: Array, second: Array) -> Array:
    """Multiply scalar-first quaternions."""
    w1, x1, y1, z1 = first
    w2, x2, y2, z2 = second
    return _quaternion_multiply_numba(w1, x1, y1, z1, w2, x2, y2, z2)


def quaternion_conjugate(quaternion: Array) -> Array:
    """Return a scalar-first quaternion conjugate."""
    q = np.asarray(quaternion, dtype=float)
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quaternion_to_matrix(quaternion: Array) -> Array:
    """Convert a unit quaternion to a proper rotation matrix."""
    return _quaternion_to_matrix_unit(normalize_quaternion(quaternion))


@_njit(nogil=True, fastmath=True)
def _quaternion_to_matrix_unit_numba(
    w: float, x: float, y: float, z: float
) -> Array:
    result = np.empty((3, 3))
    result[0, 0] = 1 - 2 * (y * y + z * z)
    result[0, 1] = 2 * (x * y - z * w)
    result[0, 2] = 2 * (x * z + y * w)
    result[1, 0] = 2 * (x * y + z * w)
    result[1, 1] = 1 - 2 * (x * x + z * z)
    result[1, 2] = 2 * (y * z - x * w)
    result[2, 0] = 2 * (x * z - y * w)
    result[2, 1] = 2 * (y * z + x * w)
    result[2, 2] = 1 - 2 * (x * x + y * y)
    return result


def _quaternion_to_matrix_unit(quaternion: Array) -> Array:
    """Convert an already normalized quaternion without revalidation."""
    w, x, y, z = quaternion
    return _quaternion_to_matrix_unit_numba(w, x, y, z)


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


@_njit(nogil=True, fastmath=True)
def _rotation_vector_to_quaternion_numba(
    rx: float, ry: float, rz: float
) -> Array:
    angle_squared = rx * rx + ry * ry + rz * rz
    if angle_squared < 1e-20:
        half_x = 0.5 * rx
        half_y = 0.5 * ry
        half_z = 0.5 * rz
        qw = 1.0
        norm_sq = qw * qw + half_x * half_x + half_y * half_y + half_z * half_z
        inv = 1.0 / math.sqrt(norm_sq)
        result = np.empty(4)
        result[0] = qw * inv
        result[1] = half_x * inv
        result[2] = half_y * inv
        result[3] = half_z * inv
        return result
    angle = math.sqrt(angle_squared)
    half_angle = 0.5 * angle
    scale = math.sin(half_angle) / angle
    result = np.empty(4)
    result[0] = math.cos(half_angle)
    result[1] = scale * rx
    result[2] = scale * ry
    result[3] = scale * rz
    return result


def _rotation_vector_to_quaternion_unchecked(rotation_vector: Array) -> Array:
    """Convert a trusted six-DoF trial increment without repeated validation."""
    vector = rotation_vector
    return _rotation_vector_to_quaternion_numba(
        float(vector[0]), float(vector[1]), float(vector[2])
    )


@_njit(nogil=True, fastmath=True)
def _quaternion_to_rotation_vector_numba(
    w: float, x: float, y: float, z: float
) -> Array:
    # Assume already normalized.
    if w < 0:
        w = -w
        x = -x
        y = -y
        z = -z
    vn = math.sqrt(x * x + y * y + z * z)
    if vn < 1e-12:
        result = np.empty(3)
        result[0] = 2.0 * x
        result[1] = 2.0 * y
        result[2] = 2.0 * z
        return result
    angle = 2.0 * math.atan2(vn, w)
    scale = angle / vn
    result = np.empty(3)
    result[0] = x * scale
    result[1] = y * scale
    result[2] = z * scale
    return result


def quaternion_to_rotation_vector(quaternion: Array) -> Array:
    """Map a unit quaternion to its principal rotation vector."""
    q = normalize_quaternion(quaternion)
    w, x, y, z = q
    return _quaternion_to_rotation_vector_numba(w, x, y, z)


@_njit(nogil=True, fastmath=True)
def _transform_point_numba(translation: Array, rotation: Array, point: Array) -> Array:
    result = np.empty(3)
    result[0] = translation[0] + rotation[0, 0] * point[0] + rotation[0, 1] * point[1] + rotation[0, 2] * point[2]
    result[1] = translation[1] + rotation[1, 0] * point[0] + rotation[1, 1] * point[1] + rotation[1, 2] * point[2]
    result[2] = translation[2] + rotation[2, 0] * point[0] + rotation[2, 1] * point[1] + rotation[2, 2] * point[2]
    return result


@_njit(nogil=True, fastmath=True)
def _point_jacobian_numba(rotation: Array, px: float, py: float, pz: float) -> Array:
    """Compute the 3x6 point Jacobian: [rotation | -rotation @ skew(point)]."""
    result = np.empty((3, 6))
    # Columns 0-2: rotation
    for i in range(3):
        for j in range(3):
            result[i, j] = rotation[i, j]
    # skew(point) columns: e0=[0,pz,-py], e1=[-pz,0,px], e2=[py,-px,0]
    # -R @ skew(point) = [-R@e0, -R@e1, -R@e2]
    # Column 3: -R @ [0, pz, -py]
    result[0, 3] = -(rotation[0, 1] * pz + rotation[0, 2] * (-py))
    result[1, 3] = -(rotation[1, 1] * pz + rotation[1, 2] * (-py))
    result[2, 3] = -(rotation[2, 1] * pz + rotation[2, 2] * (-py))
    # Column 4: -R @ [-pz, 0, px]
    result[0, 4] = -(rotation[0, 0] * (-pz) + rotation[0, 2] * px)
    result[1, 4] = -(rotation[1, 0] * (-pz) + rotation[1, 2] * px)
    result[2, 4] = -(rotation[2, 0] * (-pz) + rotation[2, 2] * px)
    # Column 5: -R @ [py, -px, 0]
    result[0, 5] = -(rotation[0, 0] * py + rotation[0, 1] * (-px))
    result[1, 5] = -(rotation[1, 0] * py + rotation[1, 1] * (-px))
    result[2, 5] = -(rotation[2, 0] * py + rotation[2, 1] * (-px))
    return result


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
        return _transform_point_numba(self.translation, self.rotation, np.asarray(point_local, dtype=float))

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
        delta_rotation = _rotation_vector_to_quaternion_numba(
            float(increment[3]), float(increment[4]), float(increment[5])
        )
        new_quaternion = _quaternion_multiply_numba(
            float(self.quaternion[0]), float(self.quaternion[1]),
            float(self.quaternion[2]), float(self.quaternion[3]),
            float(delta_rotation[0]), float(delta_rotation[1]),
            float(delta_rotation[2]), float(delta_rotation[3]),
        )
        return SE3._from_unchecked(
            self.translation + self.rotation @ increment[:3],
            new_quaternion,
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


@_njit(nogil=True, fastmath=True)
def _wrench_global_to_local_numba(rotation: Array, translation: Array, wrench: Array) -> Array:
    rt = rotation.T
    force = wrench[:3]
    moment = wrench[3:]
    # cross3(translation, force)
    cx = translation[1] * force[2] - translation[2] * force[1]
    cy = translation[2] * force[0] - translation[0] * force[2]
    cz = translation[0] * force[1] - translation[1] * force[0]
    result = np.empty(6)
    result[0] = rt[0, 0] * force[0] + rt[0, 1] * force[1] + rt[0, 2] * force[2]
    result[1] = rt[1, 0] * force[0] + rt[1, 1] * force[1] + rt[1, 2] * force[2]
    result[2] = rt[2, 0] * force[0] + rt[2, 1] * force[1] + rt[2, 2] * force[2]
    result[3] = rt[0, 0] * (moment[0] - cx) + rt[0, 1] * (moment[1] - cy) + rt[0, 2] * (moment[2] - cz)
    result[4] = rt[1, 0] * (moment[0] - cx) + rt[1, 1] * (moment[1] - cy) + rt[1, 2] * (moment[2] - cz)
    result[5] = rt[2, 0] * (moment[0] - cx) + rt[2, 1] * (moment[1] - cy) + rt[2, 2] * (moment[2] - cz)
    return result


def wrench_global_to_local(pose: SE3, wrench_global: Array) -> Array:
    """Transform a global-origin wrench to the body-origin local frame."""
    return _wrench_global_to_local_numba(
        pose.rotation, pose.translation, np.asarray(wrench_global, dtype=float)
    )


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
