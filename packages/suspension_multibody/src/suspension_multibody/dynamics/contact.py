"""Road geometry and unilateral tire-road contact evaluation."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from ..core.rigid_body import RigidBodyState
from ..core.spatial import cross3
from ..elements import ForceEvaluation
from ..elements.elastic import _point_wrench
from ..schema import RoadSurfaceSpec, TireModelSpec
from .forces import DynamicForceEvaluation
from .state import DynamicRigidBodyState
from .tires import TireForces, TireKinematics, TireModel, tire_model_from_spec


@dataclass(frozen=True)
class RoadQuery:
    """Road point, normal and local velocity at a queried horizontal position."""

    point: np.ndarray
    normal: np.ndarray
    velocity: np.ndarray
    friction_coefficient: float


@dataclass(frozen=True)
class TireContactResult:
    """Geometric and force state for one wheel at one time."""

    active: bool
    compression: float
    normal_load: float
    contact_point: np.ndarray
    normal: np.ndarray
    longitudinal: np.ndarray
    lateral: np.ndarray
    relative_velocity: np.ndarray
    slip_ratio: float
    slip_angle: float
    forces: TireForces


class RoadSurface:
    """Evaluate an analytic road surface in the vehicle engineering frame."""

    def __init__(self, spec: RoadSurfaceSpec) -> None:
        self.spec = spec
        normal = spec.normal.as_array()
        self._normal = normal / np.linalg.norm(normal)

    def query(
        self, position: np.ndarray, time: float, corner_index: int | None = None
    ) -> RoadQuery:
        position = np.asarray(position, dtype=float)
        if position.shape != (3,):
            raise ValueError("road query position must contain three values")
        origin = self.spec.origin.as_array()
        if abs(self._normal[2]) < 1e-12:
            raise ValueError("road normal must have a non-zero vertical component")
        base_height = origin[2] - (
            self._normal[0] * (position[0] - origin[0])
            + self._normal[1] * (position[1] - origin[1])
        ) / self._normal[2]
        height = base_height
        slope_x = 0.0
        corner_scale = 1.0 if corner_index is None else self.spec.corner_scales[corner_index]
        amplitude = self.spec.amplitude * corner_scale
        if self.spec.kind == "sine":
            phase = 2.0 * math.pi * (position[0] - origin[0]) / self.spec.wavelength
            phase += self.spec.phase
            height += amplitude * math.sin(phase)
            slope_x = (
                amplitude
                * 2.0
                * math.pi
                / self.spec.wavelength
                * math.cos(phase)
            )
        elif self.spec.kind in {"bump", "four_post"}:
            distance = position[0] - origin[0] - self.spec.bump_start
            if 0.0 < distance < self.spec.bump_length:
                ratio = distance / self.spec.bump_length
                height += 0.5 * amplitude * (1.0 - math.cos(2.0 * math.pi * ratio))
                slope_x = amplitude * math.pi / self.spec.bump_length * math.sin(2.0 * math.pi * ratio)
        elif self.spec.kind == "random_fourier":
            for frequency, scale, phase_offset in ((1.0, 1.0, 0.0), (2.7, 0.6, 1.2), (5.1, 0.35, 2.4)):
                phase = 2.0 * math.pi * frequency * (position[0] - origin[0]) / self.spec.wavelength
                phase += self.spec.phase + phase_offset
                height += amplitude * scale * math.sin(phase)
                slope_x += amplitude * scale * 2.0 * math.pi * frequency / self.spec.wavelength * math.cos(phase)
        road_velocity_z = 0.0
        if corner_index is not None and self.spec.corner_height_signals is not None:
            signal = self.spec.corner_height_signals[corner_index]
            height += signal.value_at(time)
            road_velocity_z = signal.derivative_at(time)
        normal = self._normal + np.array([-slope_x, 0.0, 0.0])
        normal /= np.linalg.norm(normal)
        point = np.array([position[0], position[1], height], dtype=float)
        return RoadQuery(
            point=point,
            normal=normal,
            velocity=np.array([0.0, 0.0, road_velocity_z], dtype=float),
            friction_coefficient=self.spec.friction_coefficient,
        )


def evaluate_tire_contact(
    state: DynamicRigidBodyState,
    *,
    wheel_body: str,
    spin_axis_local: np.ndarray,
    tire_spec: TireModelSpec,
    road: RoadSurface,
    time: float,
    corner_index: int | None = None,
    wheel_center_local: np.ndarray | None = None,
    tire_model: TireModel | None = None,
) -> TireContactResult:
    """Compute unilateral contact, slip state and tire forces for one wheel."""
    pose = state.pose_state.pose(wheel_body)
    center_local = (
        np.zeros(3)
        if wheel_center_local is None
        else np.asarray(wheel_center_local, dtype=float)
    )
    if center_local.shape != (3,) or not np.all(np.isfinite(center_local)):
        raise ValueError("wheel_center_local must contain three finite values")
    center = state.pose_state.point_world(wheel_body, center_local)
    query = road.query(center, time, corner_index)
    radius = tire_spec.unloaded_radius
    signed_distance = float((center - query.point) @ query.normal)
    compression = radius - signed_distance
    contact_point = center - signed_distance * query.normal
    if compression <= 0.0:
        zero = np.zeros(3)
        return TireContactResult(
            active=False,
            compression=0.0,
            normal_load=0.0,
            contact_point=contact_point,
            normal=query.normal,
            longitudinal=zero,
            lateral=zero,
            relative_velocity=zero,
            slip_ratio=0.0,
            slip_angle=0.0,
            forces=TireForces(0.0, 0.0, 0.0),
        )

    center_velocity = state.point_velocity_global(wheel_body, center_local)
    angular_velocity = pose.rotation @ state.velocities[wheel_body][3:]
    relative_velocity = center_velocity - query.velocity
    compression_rate = -float(relative_velocity @ query.normal)
    normal_load = max(
        0.0,
        tire_spec.vertical_stiffness * compression
        + tire_spec.vertical_damping * compression_rate,
    )
    axis = pose.rotation @ np.asarray(spin_axis_local, dtype=float)
    axis /= np.linalg.norm(axis)
    axis -= (axis @ query.normal) * query.normal
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-10:
        raise ValueError("wheel spin axis is parallel to road normal")
    axis /= axis_norm
    longitudinal = cross3(axis, query.normal)
    longitudinal /= np.linalg.norm(longitudinal)
    lateral = cross3(query.normal, longitudinal)
    # Slip is defined from the wheel-center velocity and the signed rolling
    # speed.  Including the rotational contact-point velocity in
    # ``longitudinal_speed`` double-counts the wheel spin and produces a
    # non-zero slip ratio for a freely rolling wheel.
    center_relative_velocity = center_velocity - query.velocity
    longitudinal_speed = float(center_relative_velocity @ longitudinal)
    lateral_speed = float(center_relative_velocity @ lateral)
    # With +X forward, +Y spindle axis and +Z road normal, negative right-hand
    # spindle speed produces the physical forward rolling contact velocity.
    wheel_speed = -float(angular_velocity @ axis) * radius
    contact_velocity = center_velocity + cross3(angular_velocity, -radius * query.normal)
    relative_velocity = contact_velocity - query.velocity
    denominator = max(abs(longitudinal_speed), tire_spec.minimum_slip_speed)
    slip_ratio = (wheel_speed - longitudinal_speed) / denominator
    slip_angle = math.atan2(lateral_speed, denominator)
    tire = tire_model if tire_model is not None else tire_model_from_spec(tire_spec)
    forces = tire.evaluate(
        TireKinematics(
            normal_load=normal_load,
            slip_angle=slip_angle,
            slip_ratio=slip_ratio,
            vertical_deflection=compression,
        )
    )
    return TireContactResult(
        active=normal_load > 0.0,
        compression=compression,
        normal_load=normal_load,
        contact_point=contact_point,
        normal=query.normal,
        longitudinal=longitudinal,
        lateral=lateral,
        relative_velocity=relative_velocity,
        slip_ratio=slip_ratio,
        slip_angle=slip_angle,
        forces=forces,
    )


@dataclass(frozen=True)
class ContactTireElement:
    """Dynamic force element that applies tire contact wrench to a wheel body."""

    name: str
    wheel_body: str
    spin_axis_local: np.ndarray
    tire_spec: TireModelSpec
    road: RoadSurface
    corner_index: int | None = None
    wheel_center_local: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Static equilibrium must not inject PAC2002 residual force/shift terms
    # into a free vehicle body.  Dynamic evaluation always uses the full model.
    static_vertical_only: bool = False
    _tire_model: TireModel = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "_tire_model", tire_model_from_spec(self.tire_spec))

    def evaluate(self, state: RigidBodyState) -> ForceEvaluation:
        """Evaluate static contact forces for equilibrium/trim solves."""
        dynamic_state = DynamicRigidBodyState.from_rigid_body_state(state)
        result = evaluate_tire_contact(
            dynamic_state,
            wheel_body=self.wheel_body,
            spin_axis_local=self.spin_axis_local,
            tire_spec=self.tire_spec,
            road=self.road,
            time=0.0,
            corner_index=self.corner_index,
            wheel_center_local=self.wheel_center_local,
            tire_model=self._tire_model,
        )
        return _contact_force_evaluation(
            self.name,
            self.wheel_body,
            result,
            normal_only=self.static_vertical_only,
        )

    def evaluate_dynamic(self, state: DynamicRigidBodyState, time: float) -> DynamicForceEvaluation:
        result = evaluate_tire_contact(
            state,
            wheel_body=self.wheel_body,
            spin_axis_local=self.spin_axis_local,
            tire_spec=self.tire_spec,
            road=self.road,
            time=time,
            corner_index=self.corner_index,
            wheel_center_local=self.wheel_center_local,
            tire_model=self._tire_model,
        )
        evaluation = _contact_force_evaluation(self.name, self.wheel_body, result)
        if not result.active:
            return DynamicForceEvaluation(
                name=evaluation.name,
                energy=evaluation.energy,
                active=False,
                events=("tire_unloaded",),
            )
        if self.static_vertical_only:
            force = result.forces.fz * result.normal
            moment = np.zeros(3)
        else:
            force = (
                result.forces.fx * result.longitudinal
                + result.forces.fy * result.lateral
                + result.forces.fz * result.normal
            )
            moment = (
                result.forces.mx * result.longitudinal
                + result.forces.my * result.lateral
                + result.forces.mz * result.normal
            )
        velocity = result.relative_velocity
        pose = state.pose_state.pose(self.wheel_body)
        angular_velocity = pose.rotation @ state.velocities[self.wheel_body][3:]
        wrench = _point_wrench(result.contact_point, force)
        wrench[3:] += moment
        return DynamicForceEvaluation(
            name=evaluation.name,
            energy=evaluation.energy,
            power=float(force @ velocity + moment @ angular_velocity),
            body_wrenches_global={self.wheel_body: wrench},
            active=True,
            events=("tire_contact",),
        )


def _contact_force_evaluation(
    name: str,
    wheel_body: str,
    result: TireContactResult,
    *,
    normal_only: bool = False,
) -> ForceEvaluation:
    """Convert a contact result to a static force-element evaluation."""
    if not result.active:
        return ForceEvaluation(name, 0.0, active=False, event="tire_unloaded")
    if normal_only:
        force = result.forces.fz * result.normal
        moment = np.zeros(3)
    else:
        force = (
            result.forces.fx * result.longitudinal
            + result.forces.fy * result.lateral
            + result.forces.fz * result.normal
        )
        moment = (
            result.forces.mx * result.longitudinal
            + result.forces.my * result.lateral
            + result.forces.mz * result.normal
        )
    wrench = _point_wrench(result.contact_point, force)
    wrench[3:] += moment
    return ForceEvaluation(
        name=name,
        energy=0.5 * result.normal_load * result.compression,
        body_wrenches_global={wheel_body: wrench},
        active=True,
    )
