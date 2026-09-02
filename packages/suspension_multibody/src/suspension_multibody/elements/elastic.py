"""Linear springs, bushings, tires, stops, anti-roll bars and gravity."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from ..core.rigid_body import RigidBodyState
from ..core.spatial import (
    SE3,
    cross3,
    quaternion_to_matrix,
    quaternion_to_rotation_vector,
)
from .base import ElementError, ForceEvaluation


def _curve_value(curve: tuple[tuple[float, float], ...], coordinate: float) -> float:
    """Evaluate a monotone piecewise-linear Adams force curve with extrapolation."""
    if not curve:
        raise ValueError("force curve is empty")
    points = np.asarray(curve, dtype=float)
    x = float(coordinate)
    if x <= points[0, 0]:
        left, right = points[0], points[1]
    elif x >= points[-1, 0]:
        left, right = points[-2], points[-1]
    else:
        index = int(np.searchsorted(points[:, 0], x, side="right")) - 1
        left, right = points[index], points[index + 1]
    slope = (right[1] - left[1]) / (right[0] - left[0])
    return float(left[1] + slope * (x - left[0]))


def _curve_slope(curve: tuple[tuple[float, float], ...], coordinate: float = 0.0) -> float:
    """Return the local slope of a monotone piecewise-linear curve."""
    if not curve:
        return 0.0
    points = np.asarray(curve, dtype=float)
    if coordinate <= points[0, 0]:
        left, right = points[0], points[1]
    elif coordinate >= points[-1, 0]:
        left, right = points[-2], points[-1]
    else:
        index = int(np.searchsorted(points[:, 0], coordinate, side="right")) - 1
        left, right = points[index], points[index + 1]
    return float((right[1] - left[1]) / (right[0] - left[0]))


def _curve_integral(
    curve: tuple[tuple[float, float], ...], coordinate: float
) -> float:
    """Integrate a piecewise-linear curve from zero to a coordinate."""
    if not curve or coordinate == 0.0:
        return 0.0
    points = np.asarray(curve, dtype=float)
    lower, upper = (
        (0.0, float(coordinate))
        if coordinate > 0.0
        else (float(coordinate), 0.0)
    )
    total = 0.0
    left = lower
    while left < upper:
        if left < points[0, 0]:
            right = min(upper, points[0, 0])
            total += points[0, 1] * (right - left)
        elif left >= points[-1, 0]:
            right = upper
            total += points[-1, 1] * (right - left)
        else:
            index = int(np.searchsorted(points[:, 0], left, side="right"))
            right = min(upper, points[index, 0])
            left_force = _curve_value(curve, left)
            right_force = _curve_value(curve, right)
            total += 0.5 * (left_force + right_force) * (right - left)
        if right <= left:
            break
        left = right
    return total if coordinate > 0.0 else -total


def _akima_slopes(curve: tuple[tuple[float, float], ...]) -> np.ndarray:
    """Return nodal slopes for the Adams/Native five-point Akima spline."""
    points = np.asarray(curve, dtype=float)
    count = len(points)
    slopes = np.zeros(count, dtype=float)
    if count < 2:
        return slopes
    secants = np.diff(points[:, 1]) / np.diff(points[:, 0])
    if count == 2:
        slopes[:] = secants[0]
        return slopes
    extended = np.zeros(count + 3, dtype=float)
    extended[2 : count + 1] = secants
    extended[1] = 2.0 * extended[2] - extended[3]
    extended[0] = 2.0 * extended[1] - extended[2]
    extended[count + 1] = 2.0 * extended[count] - extended[count - 1]
    extended[count + 2] = 2.0 * extended[count + 1] - extended[count]
    for index in range(count):
        weight_left = abs(extended[index + 3] - extended[index + 2])
        weight_right = abs(extended[index + 1] - extended[index])
        denominator = weight_left + weight_right
        slopes[index] = (
            (weight_left * extended[index + 1]
             + weight_right * extended[index + 2]) / denominator
            if denominator > 0.0
            else 0.5 * (extended[index + 1] + extended[index + 2])
        )
    return slopes


def _akima_value_slope(
    curve: tuple[tuple[float, float], ...],
    coordinate: float,
    slopes: np.ndarray | None = None,
) -> tuple[float, float]:
    """Evaluate an Akima curve and its derivative with Native's extrapolation."""
    if not curve:
        return 0.0, 0.0
    points = np.asarray(curve, dtype=float)
    if len(points) == 1 or coordinate <= points[0, 0]:
        return float(points[0, 1]), 0.0
    if coordinate >= points[-1, 0]:
        return float(points[-1, 1]), 0.0
    nodal_slopes = _akima_slopes(curve) if slopes is None else slopes
    index = int(np.searchsorted(points[:, 0], coordinate, side="left"))
    x0, x1 = points[index - 1, 0], points[index, 0]
    y0, y1 = points[index - 1, 1], points[index, 1]
    span = x1 - x0
    u = (coordinate - x0) / span
    u2 = u * u
    u3 = u2 * u
    h00 = 2.0 * u3 - 3.0 * u2 + 1.0
    h10 = u3 - 2.0 * u2 + u
    h01 = -2.0 * u3 + 3.0 * u2
    h11 = u3 - u2
    value = (
        h00 * y0
        + h10 * span * nodal_slopes[index - 1]
        + h01 * y1
        + h11 * span * nodal_slopes[index]
    )
    derivative = (
        (6.0 * u2 - 6.0 * u) / span * y0
        + (3.0 * u2 - 4.0 * u + 1.0) * nodal_slopes[index - 1]
        + (-6.0 * u2 + 6.0 * u) / span * y1
        + (3.0 * u2 - 2.0 * u) * nodal_slopes[index]
    )
    return float(value), float(derivative)


def _integrate_akima_segment(
    y0: float,
    y1: float,
    slope0: float,
    slope1: float,
    span: float,
    lower_u: float,
    upper_u: float,
) -> float:
    def primitive(u: float) -> float:
        u2 = u * u
        u3 = u2 * u
        u4 = u3 * u
        return (
            y0 * (0.5 * u4 - u3 + u)
            + span * slope0 * (0.25 * u4 - (2.0 / 3.0) * u3 + 0.5 * u2)
            + y1 * (-0.5 * u4 + u3)
            + span * slope1 * (0.25 * u4 - (1.0 / 3.0) * u3)
        )

    return span * (primitive(upper_u) - primitive(lower_u))


def _akima_integral(
    curve: tuple[tuple[float, float], ...], coordinate: float
) -> float:
    """Integrate the Akima curve from zero with constant end extrapolation."""
    if not curve or coordinate == 0.0:
        return 0.0
    points = np.asarray(curve, dtype=float)
    nodal_slopes = _akima_slopes(curve)
    lower, upper = min(0.0, coordinate), max(0.0, coordinate)
    total = 0.0
    left = lower
    while left < upper:
        if left < points[0, 0]:
            right = min(upper, points[0, 0])
            total += points[0, 1] * (right - left)
        elif left >= points[-1, 0]:
            total += points[-1, 1] * (upper - left)
            break
        else:
            index = int(np.searchsorted(points[:, 0], left, side="right"))
            right = min(upper, points[index, 0])
            span = points[index, 0] - points[index - 1, 0]
            total += _integrate_akima_segment(
                points[index - 1, 1],
                points[index, 1],
                nodal_slopes[index - 1],
                nodal_slopes[index],
                span,
                (left - points[index - 1, 0]) / span,
                (right - points[index - 1, 0]) / span,
            )
        if right <= left:
            break
        left = right
    return total if coordinate >= 0.0 else -total


def _point(state: RigidBodyState, body: str, local: np.ndarray) -> np.ndarray:
    return state.point_world(body, np.asarray(local, dtype=float))


def _add_wrench(
    target: dict[str, np.ndarray], body: str, force: np.ndarray, moment: np.ndarray
) -> None:
    wrench = np.concatenate((force, moment))
    target[body] = target.get(body, np.zeros(6)) + wrench


def _point_wrench(point: np.ndarray, force: np.ndarray) -> np.ndarray:
    return np.concatenate((force, cross3(point, force)))


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
    force_curve: tuple[tuple[float, float], ...] = ()

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
        scalar = (
            _curve_value(self.force_curve, extension) + self.preload
            if self.force_curve
            else self.stiffness * extension + self.preload
        )
        force_b = -scalar * unit
        force_a = -force_b
        slope = _curve_slope(self.force_curve, extension) if self.force_curve else self.stiffness
        transverse = scalar / length * (np.eye(3) - np.outer(unit, unit))
        tangent_bb = -(slope * np.outer(unit, unit) + transverse)
        tangent = np.block([[tangent_bb, -tangent_bb], [-tangent_bb, tangent_bb]])
        elastic_energy = (
            _curve_integral(self.force_curve, extension)
            if self.force_curve
            else 0.5 * self.stiffness * extension**2
        )
        return ForceEvaluation(
            name=self.name,
            energy=elastic_energy + self.preload * extension,
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
    viscous_damping: float = 0.0
    extension_sign: float = 1.0
    force_curve: tuple[tuple[float, float], ...] = ()

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
    """Local-frame six-axis bushing between two body attachment frames."""

    name: str
    body_a: str
    body_b: str
    local_pose_a: SE3 = field(default_factory=SE3.identity)
    local_pose_b: SE3 = field(default_factory=SE3.identity)
    stiffness: np.ndarray = field(default_factory=lambda: np.eye(6))
    damping: np.ndarray = field(default_factory=lambda: np.zeros((6, 6)))
    preload: np.ndarray = field(default_factory=lambda: np.zeros(6))
    force_curves: tuple[tuple[tuple[float, float], ...], ...] = ()
    force_curve_interpolation: Literal["piecewise_linear", "akima"] = (
        "piecewise_linear"
    )
    rotation_coordinates: Literal["rotation_vector", "cardan_xyz"] = (
        "rotation_vector"
    )

    def __post_init__(self) -> None:
        matrix = np.asarray(self.stiffness, dtype=float)
        damping = np.asarray(self.damping, dtype=float)
        preload = np.asarray(self.preload, dtype=float)
        if matrix.shape != (6, 6) or not np.allclose(matrix, matrix.T, atol=1e-9):
            raise ElementError("bushing stiffness must be symmetric 6x6")
        if damping.shape != (6, 6) or not np.allclose(damping, damping.T, atol=1e-9):
            raise ElementError("bushing damping must be symmetric 6x6")
        if np.linalg.eigvalsh(matrix).min() < -1e-9:
            raise ElementError("bushing stiffness must be positive semidefinite")
        if np.linalg.eigvalsh(damping).min() < -1e-9:
            raise ElementError("bushing damping must be positive semidefinite")
        if preload.shape != (6,):
            raise ElementError("bushing preload must contain six values")
        if self.rotation_coordinates not in ("rotation_vector", "cardan_xyz"):
            raise ElementError(
                "bushing rotation_coordinates must be rotation_vector or cardan_xyz"
            )
        if self.force_curve_interpolation not in ("piecewise_linear", "akima"):
            raise ElementError(
                "bushing force_curve_interpolation must be piecewise_linear or akima"
            )
        curves = tuple(
            tuple((float(x), float(y)) for x, y in curve)
            for curve in self.force_curves
        )
        if curves and len(curves) != 6:
            raise ElementError("bushing force_curves must contain six axis curves")
        for curve in curves:
            if curve and len(curve) < 2:
                raise ElementError("each bushing force curve requires at least two samples")
            if any(not np.isfinite(x) or not np.isfinite(y) for x, y in curve):
                raise ElementError("bushing force curves must contain finite samples")
            if any(right[0] <= left[0] for left, right in zip(curve, curve[1:])):
                raise ElementError("bushing force curve abscissas must be strictly increasing")
        object.__setattr__(self, "stiffness", matrix.copy())
        object.__setattr__(self, "damping", damping.copy())
        object.__setattr__(self, "preload", preload.copy())
        object.__setattr__(self, "force_curves", curves)

    def attachment_poses(self, state: RigidBodyState) -> tuple[SE3, SE3]:
        return (
            state.pose(self.body_a).compose(self.local_pose_a),
            state.pose(self.body_b).compose(self.local_pose_b),
        )

    def deformation(self, state: RigidBodyState) -> np.ndarray:
        pose_a, pose_b = self.attachment_poses(state)
        relative = pose_a.inverse().compose(pose_b)
        return np.concatenate(
            (relative.translation, self.rotational_deformation(relative.quaternion))
        )

    def rotational_deformation(self, relative_quaternion: np.ndarray) -> np.ndarray:
        if self.rotation_coordinates == "rotation_vector":
            return quaternion_to_rotation_vector(relative_quaternion)
        relative_rotation = quaternion_to_matrix(relative_quaternion)
        cosine_y = float(np.hypot(relative_rotation[0, 0], relative_rotation[0, 1]))
        if cosine_y <= 1.0e-10:
            raise ElementError("cardan_xyz bushing reached its gimbal singularity")
        return np.array(
            [
                np.arctan2(-relative_rotation[1, 2], relative_rotation[2, 2]),
                np.arctan2(relative_rotation[0, 2], cosine_y),
                np.arctan2(-relative_rotation[0, 1], relative_rotation[0, 0]),
            ],
            dtype=float,
        )

    def rotational_rate(
        self, relative_quaternion: np.ndarray, relative_omega: np.ndarray
    ) -> np.ndarray:
        if self.rotation_coordinates == "rotation_vector":
            return np.asarray(relative_omega, dtype=float)
        angles = self.rotational_deformation(relative_quaternion)
        sine_x, cosine_x = np.sin(angles[0]), np.cos(angles[0])
        sine_y, cosine_y = np.sin(angles[1]), np.cos(angles[1])
        if abs(cosine_y) <= 1.0e-10:
            raise ElementError("cardan_xyz bushing reached its gimbal singularity")
        y_rate = cosine_x * relative_omega[1] + sine_x * relative_omega[2]
        z_rate = (
            -sine_x * relative_omega[1] + cosine_x * relative_omega[2]
        ) / cosine_y
        return np.array(
            [relative_omega[0] - sine_y * z_rate, y_rate, z_rate], dtype=float
        )

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        pose_a, pose_b = self.attachment_poses(state)
        relative = pose_a.inverse().compose(pose_b)
        deformation = np.concatenate(
            (relative.translation, self.rotational_deformation(relative.quaternion))
        )
        elastic = self.stiffness @ deformation
        curve_slopes: dict[int, float] = {}
        for index, curve in enumerate(self.force_curves):
            if curve:
                if self.force_curve_interpolation == "akima":
                    elastic[index], curve_slopes[index] = _akima_value_slope(
                        curve, deformation[index]
                    )
                else:
                    elastic[index] = _curve_value(curve, deformation[index])
        generalized = -elastic + self.preload
        force_global = pose_a.rotation @ generalized[:3]
        moment_global = pose_a.rotation @ generalized[3:]
        wrenches = {
            self.body_a: _point_wrench(pose_a.translation, -force_global)
            + np.concatenate((np.zeros(3), -moment_global)),
            self.body_b: _point_wrench(pose_b.translation, force_global)
            + np.concatenate((np.zeros(3), moment_global)),
        }
        matrix_elastic = self.stiffness @ deformation
        elastic_energy = 0.5 * float(deformation @ matrix_elastic)
        for index, curve in enumerate(self.force_curves):
            if curve:
                elastic_energy += (
                    _akima_integral(curve, deformation[index])
                    if self.force_curve_interpolation == "akima"
                    else _curve_integral(curve, deformation[index])
                )
                elastic_energy -= 0.5 * deformation[index] * matrix_elastic[index]
        tangent = -self.stiffness.copy()
        for index, curve in enumerate(self.force_curves):
            if curve:
                tangent[index, index] = -(
                    curve_slopes[index]
                    if self.force_curve_interpolation == "akima"
                    else _curve_slope(curve, deformation[index])
                )
        return ForceEvaluation(
            name=self.name,
            energy=elastic_energy
            - float(self.preload @ deformation),
            body_wrenches_global=wrenches,
            tangent=tangent,
        )


@dataclass(frozen=True)
class PointWrenchElement:
    """A constant global wrench applied at a body-fixed point."""

    name: str
    body: str
    point_local: np.ndarray
    force_global: np.ndarray
    moment_global: np.ndarray

    def __post_init__(self) -> None:
        point = np.asarray(self.point_local, dtype=float)
        force = np.asarray(self.force_global, dtype=float)
        moment = np.asarray(self.moment_global, dtype=float)
        if (
            point.shape != (3,)
            or force.shape != (3,)
            or moment.shape != (3,)
            or not np.all(np.isfinite(np.concatenate((point, force, moment))))
        ):
            raise ElementError("point wrench must contain finite three-vectors")
        object.__setattr__(self, "point_local", point.copy())
        object.__setattr__(self, "force_global", force.copy())
        object.__setattr__(self, "moment_global", moment.copy())

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        point = _point(state, self.body, self.point_local)
        wrench = np.concatenate(
            (
                self.force_global,
                cross3(point, self.force_global) + self.moment_global,
            )
        )
        return ForceEvaluation(
            name=self.name,
            energy=0.0,
            body_wrenches_global={self.body: wrench},
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
    force_curve: tuple[tuple[float, float], ...] = ()

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        point_a = _point(state, self.body_a, self.point_a)
        point_b = _point(state, self.body_b, self.point_b)
        distance_vector = point_b - point_a
        distance = float(np.linalg.norm(distance_vector))
        gap = distance - self.clearance
        compression = max(0.0, -gap)
        active = compression > 0.0
        if not active or (self.stiffness <= 0 and not self.force_curve):
            return ForceEvaluation(self.name, 0.0, active=False, event="stop_clear")
        if distance < 1e-12:
            raise ElementError("bump stop endpoints are coincident")
        unit = distance_vector / distance
        scalar = (
            _curve_value(self.force_curve, compression)
            if self.force_curve
            else self.stiffness * compression
        )
        force_b = scalar * unit
        elastic_energy = (
            _curve_integral(self.force_curve, compression)
            if self.force_curve
            else 0.5 * self.stiffness * compression**2
        )
        tangent = _curve_slope(self.force_curve, compression) if self.force_curve else self.stiffness
        return ForceEvaluation(
            self.name,
            elastic_energy,
            {
                self.body_a: _point_wrench(point_a, -force_b),
                self.body_b: _point_wrench(point_b, force_b),
            },
            active=True,
            event="stop_contact",
            tangent=np.array([[tangent]]),
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
