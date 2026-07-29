"""Linear springs, bushings, tires, stops, anti-roll bars and gravity."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core.rigid_body import RigidBodyState
from ..core.spatial import (
    SE3,
    quaternion_to_rotation_vector,
)
from .base import ElementError, ForceEvaluation


def _point(state: RigidBodyState, body: str, local: np.ndarray) -> np.ndarray:
    return state.point_world(body, np.asarray(local, dtype=float))


def _add_wrench(
    target: dict[str, np.ndarray], body: str, force: np.ndarray, moment: np.ndarray
) -> None:
    wrench = np.concatenate((force, moment))
    target[body] = target.get(body, np.zeros(6)) + wrench


def _point_wrench(point: np.ndarray, force: np.ndarray) -> np.ndarray:
    return np.concatenate((force, np.cross(point, force)))


@dataclass(frozen=True)
class LinearSpringElement:
    """Two-point linear spring with free-length or preload representation."""

    name: str
    body_a: str
    point_a: np.ndarray
    body_b: str
    point_b: np.ndarray
    stiffness: float
    free_length: float | None = None
    reference_length: float | None = None
    preload: float = 0.0

    def __post_init__(self) -> None:
        if self.stiffness <= 0 or not np.isfinite(self.stiffness):
            raise ElementError("spring stiffness must be positive")
        if (self.free_length is None) == (self.reference_length is None):
            raise ElementError("spring requires free_length or reference_length")
        if self.free_length is not None and self.free_length <= 0:
            raise ElementError("free_length must be positive")

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        point_a = _point(state, self.body_a, self.point_a)
        point_b = _point(state, self.body_b, self.point_b)
        delta = point_b - point_a
        length = float(np.linalg.norm(delta))
        if length < 1e-12:
            raise ElementError("spring endpoints are coincident")
        unit = delta / length
        reference = (
            self.free_length if self.free_length is not None else self.reference_length
        )
        extension = length - reference  # type: ignore[operator]
        scalar = self.stiffness * extension + self.preload
        force_b = -scalar * unit
        force_a = -force_b
        transverse = scalar / length * (np.eye(3) - np.outer(unit, unit))
        tangent_bb = -(self.stiffness * np.outer(unit, unit) + transverse)
        tangent = np.block([[tangent_bb, -tangent_bb], [-tangent_bb, tangent_bb]])
        return ForceEvaluation(
            name=self.name,
            energy=0.5 * self.stiffness * extension**2 + self.preload * extension,
            body_wrenches_global={
                self.body_a: _point_wrench(point_a, force_a),
                self.body_b: _point_wrench(point_b, force_b),
            },
            tangent=tangent,
        )


@dataclass(frozen=True)
class StaticDamperElement:
    """Quasi-static gas/preload/friction damper along a two-point axis."""

    name: str
    body_a: str
    point_a: np.ndarray
    body_b: str
    point_b: np.ndarray
    gas_stiffness: float = 0.0
    gas_reference_length: float | None = None
    gas_reference_force: float = 0.0
    preload: float = 0.0
    friction: float = 0.0
    extension_sign: float = 1.0

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        point_a = _point(state, self.body_a, self.point_a)
        point_b = _point(state, self.body_b, self.point_b)
        delta = point_b - point_a
        length = float(np.linalg.norm(delta))
        if length < 1e-12:
            raise ElementError("damper endpoints are coincident")
        reference = self.gas_reference_length or length
        gas_force = self.gas_reference_force + self.gas_stiffness * (length - reference)
        scalar = gas_force + self.preload + self.friction * np.sign(self.extension_sign)
        unit = delta / length
        force_b = -scalar * unit
        force_a = -force_b
        tangent = self.gas_stiffness * np.outer(unit, unit)
        return ForceEvaluation(
            self.name,
            0.5 * self.gas_stiffness * (length - reference) ** 2
            + (self.gas_reference_force + self.preload) * (length - reference),
            {
                self.body_a: _point_wrench(point_a, force_a),
                self.body_b: _point_wrench(point_b, force_b),
            },
            tangent=np.block([[tangent, -tangent], [-tangent, tangent]]),
        )


@dataclass(frozen=True)
class BushingElement:
    """Local-frame linear 6x6 bushing between two body attachment frames."""

    name: str
    body_a: str
    body_b: str
    local_pose_a: SE3 = field(default_factory=SE3.identity)
    local_pose_b: SE3 = field(default_factory=SE3.identity)
    stiffness: np.ndarray = field(default_factory=lambda: np.eye(6))
    preload: np.ndarray = field(default_factory=lambda: np.zeros(6))

    def __post_init__(self) -> None:
        matrix = np.asarray(self.stiffness, dtype=float)
        preload = np.asarray(self.preload, dtype=float)
        if matrix.shape != (6, 6) or not np.allclose(matrix, matrix.T, atol=1e-9):
            raise ElementError("bushing stiffness must be symmetric 6x6")
        if np.linalg.eigvalsh(matrix).min() < -1e-9:
            raise ElementError("bushing stiffness must be positive semidefinite")
        if preload.shape != (6,):
            raise ElementError("bushing preload must contain six values")
        object.__setattr__(self, "stiffness", matrix.copy())
        object.__setattr__(self, "preload", preload.copy())

    def attachment_poses(self, state: RigidBodyState) -> tuple[SE3, SE3]:
        return (
            state.pose(self.body_a).compose(self.local_pose_a),
            state.pose(self.body_b).compose(self.local_pose_b),
        )

    def deformation(self, state: RigidBodyState) -> np.ndarray:
        pose_a, pose_b = self.attachment_poses(state)
        relative = pose_a.inverse().compose(pose_b)
        return np.concatenate(
            (relative.translation, quaternion_to_rotation_vector(relative.quaternion))
        )

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        pose_a, pose_b = self.attachment_poses(state)
        deformation = self.deformation(state)
        generalized = -self.stiffness @ deformation + self.preload
        force_global = pose_a.rotation @ generalized[:3]
        moment_global = pose_a.rotation @ generalized[3:]
        wrenches = {
            self.body_a: _point_wrench(pose_a.translation, -force_global)
            + np.concatenate((np.zeros(3), -moment_global)),
            self.body_b: _point_wrench(pose_b.translation, force_global)
            + np.concatenate((np.zeros(3), moment_global)),
        }
        return ForceEvaluation(
            name=self.name,
            energy=0.5 * float(deformation @ self.stiffness @ deformation)
            - float(self.preload @ deformation),
            body_wrenches_global=wrenches,
            tangent=-self.stiffness,
        )


@dataclass(frozen=True)
class VerticalTireElement:
    """Compression-only single-axis vertical tire at a wheel center."""

    name: str
    wheel_body: str
    wheel_center_local: np.ndarray
    stiffness: float
    unloaded_radius: float
    road_z: float = 0.0

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        center = _point(state, self.wheel_body, self.wheel_center_local)
        compression = self.road_z + self.unloaded_radius - center[2]
        if compression <= 0:
            return ForceEvaluation(self.name, 0.0, active=False, event="tire_unloaded")
        force = np.array([0.0, 0.0, self.stiffness * compression])
        return ForceEvaluation(
            self.name,
            0.5 * self.stiffness * compression**2,
            {self.wheel_body: _point_wrench(center, force)},
            active=True,
            tangent=np.array([[-self.stiffness]]),
        )


@dataclass(frozen=True)
class AntiRollBarElement:
    """Equivalent torsional anti-roll bar driven by link vertical travel."""

    name: str
    left_body: str
    left_point: np.ndarray
    right_body: str
    right_point: np.ndarray
    stiffness: float
    reference_difference: float = 0.0

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        left = _point(state, self.left_body, self.left_point)
        right = _point(state, self.right_body, self.right_point)
        difference = (right[2] - left[2]) - self.reference_difference
        scalar = self.stiffness * difference
        left_force = np.array([0.0, 0.0, scalar])
        right_force = -left_force
        return ForceEvaluation(
            self.name,
            0.5 * self.stiffness * difference**2,
            {
                self.left_body: _point_wrench(left, left_force),
                self.right_body: _point_wrench(right, right_force),
            },
            tangent=np.array(
                [[self.stiffness, -self.stiffness], [-self.stiffness, self.stiffness]]
            ),
        )


@dataclass(frozen=True)
class BumpStopElement:
    """Unilateral clearance plus post-contact linear stiffness."""

    name: str
    body_a: str
    point_a: np.ndarray
    body_b: str
    point_b: np.ndarray
    clearance: float
    stiffness: float
    direction: str = "bump"

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        point_a = _point(state, self.body_a, self.point_a)
        point_b = _point(state, self.body_b, self.point_b)
        distance_vector = point_b - point_a
        distance = float(np.linalg.norm(distance_vector))
        gap = distance - self.clearance
        active = gap < 0
        if not active or self.stiffness <= 0:
            return ForceEvaluation(self.name, 0.0, active=False, event="stop_clear")
        if distance < 1e-12:
            raise ElementError("bump stop endpoints are coincident")
        unit = distance_vector / distance
        force_b = self.stiffness * gap * unit
        return ForceEvaluation(
            self.name,
            0.5 * self.stiffness * gap**2,
            {
                self.body_a: _point_wrench(point_a, -force_b),
                self.body_b: _point_wrench(point_b, force_b),
            },
            active=True,
            event="stop_contact",
            tangent=np.array([[self.stiffness]]),
        )


@dataclass(frozen=True)
class GravityElement:
    """Constant gravity force for one rigid body."""

    name: str
    body: str
    mass: float
    gravity: float = 9810.0
    center_of_mass_local: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        center = _point(state, self.body, self.center_of_mass_local)
        force = np.array([0.0, 0.0, -self.mass * self.gravity])
        return ForceEvaluation(
            self.name, 0.0, {self.body: _point_wrench(center, force)}
        )
