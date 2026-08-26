"""Ideal joint and drive constraints with analytic local Jacobians."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field

import numpy as np

try:
    from numba import njit as _njit
except ImportError:  # pragma: no cover
    def _njit(*_args, **_kwargs):
        def _decorator(func):
            return func
        return _decorator

from .rigid_body import RigidBodyState
from .spatial import (
    Array,
    cross3,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_rotation_vector,
    skew,
)


@_njit(nogil=True, fastmath=True)
def _point_coincidence_residual_numba(
    ta: Array, ra: Array, pa: Array, tb: Array, rb: Array, pb: Array
) -> Array:
    """Residual is point_world(a, pa) minus point_world(b, pb)."""
    pax = ta[0] + ra[0, 0] * pa[0] + ra[0, 1] * pa[1] + ra[0, 2] * pa[2]
    pay = ta[1] + ra[1, 0] * pa[0] + ra[1, 1] * pa[1] + ra[1, 2] * pa[2]
    paz = ta[2] + ra[2, 0] * pa[0] + ra[2, 1] * pa[1] + ra[2, 2] * pa[2]
    pbx = tb[0] + rb[0, 0] * pb[0] + rb[0, 1] * pb[1] + rb[0, 2] * pb[2]
    pby = tb[1] + rb[1, 0] * pb[0] + rb[1, 1] * pb[1] + rb[1, 2] * pb[2]
    pbz = tb[2] + rb[2, 0] * pb[0] + rb[2, 1] * pb[1] + rb[2, 2] * pb[2]
    result = np.empty(3)
    result[0] = pax - pbx
    result[1] = pay - pby
    result[2] = paz - pbz
    return result


@_njit(nogil=True, fastmath=True)
def _normalize3_numba(x: float, y: float, z: float) -> Array:
    norm_squared = x * x + y * y + z * z
    if norm_squared < 1e-24:
        result = np.empty(3)
        result[0] = 0.0
        result[1] = 0.0
        result[2] = 1.0
        return result
    inv = 1.0 / math.sqrt(norm_squared)
    result = np.empty(3)
    result[0] = x * inv
    result[1] = y * inv
    result[2] = z * inv
    return result


def _normalize3(vector: Array) -> Array:
    """Normalize a finite three-vector using scalar norm arithmetic."""
    value = np.asarray(vector, dtype=float)
    return _normalize3_numba(float(value[0]), float(value[1]), float(value[2]))


def _basis_perpendicular(axis: Array) -> Array:
    """Return two orthonormal rows perpendicular to a unit axis."""
    vector = _normalize3(axis)
    helper = (
        np.array([1.0, 0.0, 0.0]) if abs(vector[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
    )
    first = _normalize3(cross3(vector, helper))
    second = cross3(vector, first)
    return np.vstack((first, second))


class Constraint(ABC):
    """Residual constraint interface."""

    name: str

    @abstractmethod
    def residual(self, state: RigidBodyState) -> Array:
        """Return a residual vector."""

    @abstractmethod
    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        """Return body-local Jacobian blocks keyed by body name."""


@dataclass(frozen=True)
class PointCoincidence(Constraint):
    """Three position constraints between two body points."""

    body_a: str
    point_a: Array
    body_b: str
    point_b: Array
    name: str = "point_coincidence"

    def residual(self, state: RigidBodyState) -> Array:
        pose_a = state.pose(self.body_a)
        pose_b = state.pose(self.body_b)
        return _point_coincidence_residual_numba(
            pose_a.translation, pose_a.rotation, np.asarray(self.point_a, dtype=float),
            pose_b.translation, pose_b.rotation, np.asarray(self.point_b, dtype=float),
        )

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        return {
            self.body_a: state.point_jacobian(self.body_a, self.point_a),
            self.body_b: -state.point_jacobian(self.body_b, self.point_b),
        }


@dataclass(frozen=True)
class BallJoint(PointCoincidence):
    """Ideal spherical joint."""

    name: str = "ball_joint"


@dataclass(frozen=True)
class WeldJoint(Constraint):
    """Six-constraint rigid connection that preserves relative pose."""

    body_a: str
    point_a: Array
    body_b: str
    point_b: Array
    name: str = "weld_joint"

    def residual(self, state: RigidBodyState) -> Array:
        point = state.point_world(self.body_a, self.point_a) - state.point_world(
            self.body_b, self.point_b
        )
        relative = quaternion_multiply(
            quaternion_conjugate(state.pose(self.body_a).quaternion),
            state.pose(self.body_b).quaternion,
        )
        return np.concatenate((point, quaternion_to_rotation_vector(relative)))

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        pose_a = state.pose(self.body_a)
        pose_b = state.pose(self.body_b)
        relative_rotation = pose_a.rotation.T @ pose_b.rotation
        rotation_a = np.hstack((np.zeros((3, 3)), -np.eye(3)))
        rotation_b = np.hstack((np.zeros((3, 3)), relative_rotation))
        return {
            self.body_a: np.vstack((state.point_jacobian(self.body_a, self.point_a), rotation_a)),
            self.body_b: np.vstack((-state.point_jacobian(self.body_b, self.point_b), rotation_b)),
        }


@dataclass(frozen=True)
class DistanceConstraint(Constraint):
    """One scalar fixed-distance constraint."""

    body_a: str
    point_a: Array
    body_b: str
    point_b: Array
    distance: float
    name: str = "distance"

    def residual(self, state: RigidBodyState) -> Array:
        delta = state.point_world(self.body_a, self.point_a) - state.point_world(
            self.body_b, self.point_b
        )
        return np.array([np.linalg.norm(delta) - self.distance])

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        delta = state.point_world(self.body_a, self.point_a) - state.point_world(
            self.body_b, self.point_b
        )
        norm = float(np.linalg.norm(delta))
        if norm < 1e-12:
            raise ValueError("distance constraint is singular at coincident points")
        direction = delta / norm
        return {
            self.body_a: direction[None, :]
            @ state.point_jacobian(self.body_a, self.point_a),
            self.body_b: -direction[None, :]
            @ state.point_jacobian(self.body_b, self.point_b),
        }


@dataclass(frozen=True)
class RevoluteJoint(Constraint):
    """Five-constraint ideal revolute joint."""

    body_a: str
    point_a: Array
    axis_a: Array
    body_b: str
    point_b: Array
    axis_b: Array
    name: str = "revolute_joint"

    def residual(self, state: RigidBodyState) -> Array:
        point = state.point_world(self.body_a, self.point_a) - state.point_world(
            self.body_b, self.point_b
        )
        axis_a = state.pose(self.body_a).rotation @ self.axis_a
        axis_b = state.pose(self.body_b).rotation @ self.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        basis = _basis_perpendicular(axis_a)
        return np.concatenate((point, basis @ cross3(axis_a, axis_b)))

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        pose_a = state.pose(self.body_a)
        pose_b = state.pose(self.body_b)
        axis_a = pose_a.rotation @ self.axis_a
        axis_b = pose_b.rotation @ self.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        basis = _basis_perpendicular(axis_a)
        point_a_jac = state.point_jacobian(self.body_a, self.point_a)
        point_b_jac = state.point_jacobian(self.body_b, self.point_b)
        d_axis_a = np.hstack((np.zeros((3, 3)), -pose_a.rotation @ skew(self.axis_a)))
        d_axis_b = np.hstack((np.zeros((3, 3)), -pose_b.rotation @ skew(self.axis_b)))
        d_cross_a = -skew(axis_b) @ d_axis_a
        d_cross_b = skew(axis_a) @ d_axis_b
        return {
            self.body_a: np.vstack((point_a_jac, basis @ d_cross_a)),
            self.body_b: np.vstack((-point_b_jac, basis @ d_cross_b)),
        }


@dataclass(frozen=True)
class PrismaticJoint(Constraint):
    """Five-constraint ideal prismatic joint along a body-A axis."""

    body_a: str
    point_a: Array
    axis_a: Array
    body_b: str
    point_b: Array
    axis_b: Array
    name: str = "prismatic_joint"

    def residual(self, state: RigidBodyState) -> Array:
        pose_a = state.pose(self.body_a)
        pose_b = state.pose(self.body_b)
        axis_a = pose_a.rotation @ self.axis_a
        axis_b = pose_b.rotation @ self.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        displacement = state.point_world(self.body_b, self.point_b) - state.point_world(
            self.body_a, self.point_a
        )
        relative_quaternion = quaternion_multiply(
            quaternion_conjugate(pose_a.quaternion), pose_b.quaternion
        )
        relative_vector = pose_a.rotation @ quaternion_to_rotation_vector(
            relative_quaternion
        )
        basis = _basis_perpendicular(axis_a)
        return np.concatenate(
            (
                basis @ displacement,
                basis @ cross3(axis_a, axis_b),
                [axis_a @ relative_vector],
            )
        )

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        pose_a = state.pose(self.body_a)
        pose_b = state.pose(self.body_b)
        axis_a = pose_a.rotation @ self.axis_a
        axis_b = pose_b.rotation @ self.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        basis = _basis_perpendicular(axis_a)
        point_a_jac = state.point_jacobian(self.body_a, self.point_a)
        point_b_jac = state.point_jacobian(self.body_b, self.point_b)
        d_axis_a = np.hstack((np.zeros((3, 3)), -pose_a.rotation @ skew(self.axis_a)))
        d_axis_b = np.hstack((np.zeros((3, 3)), -pose_b.rotation @ skew(self.axis_b)))
        d_cross_a = -skew(axis_b) @ d_axis_a
        d_cross_b = skew(axis_a) @ d_axis_b
        rotation_a = np.hstack((np.zeros((3, 3)), -pose_a.rotation))
        rotation_b = np.hstack((np.zeros((3, 3)), pose_b.rotation))
        return {
            self.body_a: np.vstack(
                (
                    -basis @ point_a_jac,
                    basis @ d_cross_a,
                    axis_a @ rotation_a,
                )
            ),
            self.body_b: np.vstack(
                (
                    basis @ point_b_jac,
                    basis @ d_cross_b,
                    axis_a @ rotation_b,
                )
            ),
        }


@dataclass(frozen=True)
class CoordinateDrive(Constraint):
    """Scalar point-coordinate displacement drive."""

    body: str
    point: Array
    axis: Array
    target: float
    name: str = "coordinate_drive"

    def residual(self, state: RigidBodyState) -> Array:
        axis = _normalize3(self.axis)
        return np.array([axis @ state.point_world(self.body, self.point) - self.target])

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        axis = _normalize3(self.axis)
        return {self.body: axis[None, :] @ state.point_jacobian(self.body, self.point)}


@dataclass(frozen=True)
class ConstraintSystem:
    """Stack constraints into a deterministic residual/Jacobian matrix."""

    constraints: tuple[Constraint, ...]
    _body_indices_cache: dict[str, int] = field(default_factory=dict, repr=False, compare=False)
    _n_bodies_cache: int = field(default=0, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not self._body_indices_cache and self.constraints:
            # Pre-compute body indices from the first constraint's bodies
            pass

    def _get_body_indices(self, body_order: tuple[str, ...]) -> dict[str, int]:
        if body_order and len(self._body_indices_cache) == len(body_order):
            cached = self._body_indices_cache
            if all(name in cached for name in body_order):
                return cached
        return {name: index for index, name in enumerate(body_order)}

    def residual(self, state: RigidBodyState) -> Array:
        if not self.constraints:
            return np.zeros(0)
        return np.concatenate(
            [constraint.residual(state) for constraint in self.constraints]
        )

    def jacobian(
        self, state: RigidBodyState, body_order: tuple[str, ...] | None = None
    ) -> Array:
        order = body_order or tuple(
            name for name, body in state.bodies.items() if not body.fixed
        )
        body_indices = self._get_body_indices(order)
        n_bodies = len(order)
        local_blocks = [constraint.jacobian(state) for constraint in self.constraints]
        total_rows = 0
        for local in local_blocks:
            first_block = next(iter(local.values()), None)
            total_rows += int(first_block.shape[0]) if first_block is not None else 0
        if total_rows == 0:
            return np.zeros((0, 6 * n_bodies))
        result = np.zeros((total_rows, 6 * n_bodies))
        row_offset = 0
        for local in local_blocks:
            first_block = next(iter(local.values()), None)
            row_count = int(first_block.shape[0]) if first_block is not None else 0
            if row_count:
                row_slice = slice(row_offset, row_offset + row_count)
                for name, block in local.items():
                    index = body_indices.get(name)
                    if index is not None:
                        result[row_slice, index * 6 : (index + 1) * 6] = block
            row_offset += row_count
        return result
