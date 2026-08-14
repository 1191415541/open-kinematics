"""Ideal joint and drive constraints with analytic local Jacobians."""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from .rigid_body import RigidBodyState
from .spatial import (
    Array,
    cross3,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_rotation_vector,
    skew,
)


def _normalize3(vector: Array) -> Array:
    """Normalize a finite three-vector using scalar norm arithmetic."""
    value = np.asarray(vector, dtype=float)
    norm_squared = float(value @ value)
    if norm_squared < 1e-24:
        raise ValueError("constraint axis is singular")
    return value / math.sqrt(norm_squared)


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
        return state.point_world(self.body_a, self.point_a) - state.point_world(
            self.body_b, self.point_b
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
        blocks: list[tuple[dict[str, Array], int]] = []
        total_rows = 0
        for constraint in self.constraints:
            local = constraint.jacobian(state)
            first_block = next(iter(local.values()), np.zeros((0, 0)))
            row_count = int(first_block.shape[0])
            blocks.append((local, row_count))
            total_rows += row_count
        if not blocks:
            return np.zeros((0, 6 * len(order)))
        result = np.zeros((total_rows, 6 * len(order)))
        body_indices = {name: index for index, name in enumerate(order)}
        row_offset = 0
        for local, row_count in blocks:
            row_slice = slice(row_offset, row_offset + row_count)
            for name, block in local.items():
                index = body_indices.get(name)
                if index is not None:
                    result[row_slice, index * 6 : (index + 1) * 6] = block
            row_offset += row_count
        return result
