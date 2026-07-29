"""Ideal joint and drive constraints with analytic local Jacobians."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from .rigid_body import RigidBodyState
from .spatial import (
    Array,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_rotation_vector,
    skew,
)


def _basis_perpendicular(axis: Array) -> Array:
    """Return two orthonormal rows perpendicular to a unit axis."""
    vector = np.asarray(axis, dtype=float)
    vector = vector / np.linalg.norm(vector)
    helper = np.array([1.0, 0.0, 0.0]) if abs(vector[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
    first = np.cross(vector, helper)
    first /= np.linalg.norm(first)
    second = np.cross(vector, first)
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
        return state.point_world(self.body_a, self.point_a) - state.point_world(self.body_b, self.point_b)

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
class DistanceConstraint(Constraint):
    """One scalar fixed-distance constraint."""

    body_a: str
    point_a: Array
    body_b: str
    point_b: Array
    distance: float
    name: str = "distance"

    def residual(self, state: RigidBodyState) -> Array:
        delta = state.point_world(self.body_a, self.point_a) - state.point_world(self.body_b, self.point_b)
        return np.array([np.linalg.norm(delta) - self.distance])

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        delta = state.point_world(self.body_a, self.point_a) - state.point_world(self.body_b, self.point_b)
        norm = float(np.linalg.norm(delta))
        if norm < 1e-12:
            raise ValueError("distance constraint is singular at coincident points")
        direction = delta / norm
        return {
            self.body_a: direction[None, :] @ state.point_jacobian(self.body_a, self.point_a),
            self.body_b: -direction[None, :] @ state.point_jacobian(self.body_b, self.point_b),
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
        point = state.point_world(self.body_a, self.point_a) - state.point_world(self.body_b, self.point_b)
        axis_a = state.pose(self.body_a).rotation @ self.axis_a
        axis_b = state.pose(self.body_b).rotation @ self.axis_b
        axis_a /= np.linalg.norm(axis_a)
        axis_b /= np.linalg.norm(axis_b)
        basis = _basis_perpendicular(axis_a)
        return np.concatenate((point, basis @ np.cross(axis_a, axis_b)))

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        pose_a = state.pose(self.body_a)
        pose_b = state.pose(self.body_b)
        axis_a = pose_a.rotation @ self.axis_a
        axis_b = pose_b.rotation @ self.axis_b
        axis_a /= np.linalg.norm(axis_a)
        axis_b /= np.linalg.norm(axis_b)
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
        axis_a /= np.linalg.norm(axis_a)
        axis_b /= np.linalg.norm(axis_b)
        displacement = state.point_world(self.body_b, self.point_b) - state.point_world(self.body_a, self.point_a)
        relative_quaternion = quaternion_multiply(
            quaternion_conjugate(pose_a.quaternion), pose_b.quaternion
        )
        relative_vector = pose_a.rotation @ quaternion_to_rotation_vector(relative_quaternion)
        basis = _basis_perpendicular(axis_a)
        return np.concatenate(
            (basis @ displacement, basis @ np.cross(axis_a, axis_b), [axis_a @ relative_vector])
        )

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        pose_a = state.pose(self.body_a)
        pose_b = state.pose(self.body_b)
        axis_a = pose_a.rotation @ self.axis_a
        axis_b = pose_b.rotation @ self.axis_b
        axis_a /= np.linalg.norm(axis_a)
        axis_b /= np.linalg.norm(axis_b)
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
            self.body_b: np.vstack((
                basis @ point_b_jac,
                basis @ d_cross_b,
                axis_a @ rotation_b,
            )),
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
        axis = np.asarray(self.axis, dtype=float)
        axis /= np.linalg.norm(axis)
        return np.array([axis @ state.point_world(self.body, self.point) - self.target])

    def jacobian(self, state: RigidBodyState) -> dict[str, Array]:
        axis = np.asarray(self.axis, dtype=float)
        axis /= np.linalg.norm(axis)
        return {self.body: axis[None, :] @ state.point_jacobian(self.body, self.point)}


@dataclass(frozen=True)
class ConstraintSystem:
    """Stack constraints into a deterministic residual/Jacobian matrix."""

    constraints: tuple[Constraint, ...]

    def residual(self, state: RigidBodyState) -> Array:
        if not self.constraints:
            return np.zeros(0)
        return np.concatenate([constraint.residual(state) for constraint in self.constraints])

    def jacobian(self, state: RigidBodyState, body_order: tuple[str, ...] | None = None) -> Array:
        order = body_order or tuple(name for name, body in state.bodies.items() if not body.fixed)
        blocks: list[Array] = []
        for constraint in self.constraints:
            local = constraint.jacobian(state)
            blocks.append(
                np.hstack([local.get(name, np.zeros((len(constraint.residual(state)), 6))) for name in order])
            )
        return np.vstack(blocks) if blocks else np.zeros((0, 6 * len(order)))
