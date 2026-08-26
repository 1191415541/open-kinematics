"""Closed SI schemas for the native axle dynamics solver."""

from __future__ import annotations

import math
from typing import Literal

import numpy as np
from pydantic import Field, field_validator, model_validator

from ..schema.common import StrictModel

Vec3Tuple = tuple[float, float, float]
QuaternionTuple = tuple[float, float, float, float]
Matrix3Tuple = tuple[tuple[float, float, float], ...]
Vector6Tuple = tuple[float, float, float, float, float, float]
Matrix6Tuple = tuple[tuple[float, float, float, float, float, float], ...]


def _finite(values: tuple[float, ...], label: str) -> tuple[float, ...]:
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain finite values")
    return values


class AxleBody(StrictModel):
    """One rigid body expressed in SI units."""

    name: str = Field(min_length=1)
    mass_kg: float = Field(ge=0)
    inertia_kg_m2: Matrix3Tuple
    position_m: Vec3Tuple = (0.0, 0.0, 0.0)
    quaternion_body_to_world: QuaternionTuple = (1.0, 0.0, 0.0, 0.0)
    linear_velocity_m_per_s: Vec3Tuple = (0.0, 0.0, 0.0)
    angular_velocity_rad_per_s: Vec3Tuple = (0.0, 0.0, 0.0)
    fixed: bool = False

    @field_validator("inertia_kg_m2", mode="before")
    @classmethod
    def _validate_inertia(cls, value: object) -> Matrix3Tuple:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("inertia_kg_m2 must be a finite 3x3 matrix")
        if not np.allclose(matrix, matrix.T, atol=1e-12):
            raise ValueError("inertia_kg_m2 must be symmetric")
        return tuple(tuple(float(item) for item in row) for row in matrix)

    @field_validator(
        "position_m",
        "linear_velocity_m_per_s",
        "angular_velocity_rad_per_s",
    )
    @classmethod
    def _validate_position(cls, value: Vec3Tuple) -> Vec3Tuple:
        return _finite(value, "position_m")  # type: ignore[return-value]

    @field_validator("quaternion_body_to_world")
    @classmethod
    def _validate_quaternion(
        cls, value: QuaternionTuple
    ) -> QuaternionTuple:
        _finite(value, "quaternion_body_to_world")
        norm = math.sqrt(sum(item * item for item in value))
        if norm <= 1e-12:
            raise ValueError("quaternion_body_to_world must be nonzero")
        return tuple(item / norm for item in value)  # type: ignore[return-value]

    @model_validator(mode="after")
    def _physical_mass(self) -> AxleBody:
        inertia = np.asarray(self.inertia_kg_m2, dtype=float)
        if not self.fixed:
            if self.mass_kg <= 0:
                raise ValueError("free bodies require positive mass_kg")
            if float(np.min(np.linalg.eigvalsh(inertia))) <= 0:
                raise ValueError("free body inertia_kg_m2 must be positive definite")
        return self


class AxleJoint(StrictModel):
    """One ideal joint with body-local markers and axes."""

    name: str = Field(min_length=1)
    kind: Literal[
        "spherical",
        "revolute",
        "prismatic",
        "fixed",
        "universal",
        "cylindrical",
        "inplane",
    ]
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    axis_a: Vec3Tuple = (0.0, 0.0, 1.0)
    axis_b: Vec3Tuple = (0.0, 0.0, 1.0)

    @model_validator(mode="after")
    def _valid_joint(self) -> AxleJoint:
        if self.body_a == self.body_b:
            raise ValueError("joint bodies must be different")
        for value, label in (
            (self.point_a_m, "point_a_m"),
            (self.point_b_m, "point_b_m"),
            (self.axis_a, "axis_a"),
            (self.axis_b, "axis_b"),
        ):
            _finite(value, label)
        if self.kind in {
            "revolute",
            "prismatic",
            "universal",
            "cylindrical",
            "inplane",
        }:
            if np.linalg.norm(self.axis_a) <= 1e-12:
                raise ValueError("axis_a must be nonzero")
            # An in-plane primitive is defined by body A's plane normal alone.
            if self.kind != "inplane" and np.linalg.norm(self.axis_b) <= 1e-12:
                raise ValueError("axis_b must be nonzero")
        return self


class AxleSpringDamper(StrictModel):
    """Passive axial spring-damper between two body-local points."""

    name: str = Field(min_length=1)
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    stiffness_n_per_m: float = Field(ge=0)
    compression_damping_n_s_per_m: float = Field(ge=0)
    rebound_damping_n_s_per_m: float = Field(ge=0)
    free_length_m: float = Field(ge=0)
    minimum_length_m: float | None = Field(default=None, ge=0)
    maximum_length_m: float | None = Field(default=None, ge=0)
    compression_stop_stiffness_n_per_m: float = Field(default=0.0, ge=0)
    compression_stop_damping_n_s_per_m: float = Field(default=0.0, ge=0)
    rebound_stop_stiffness_n_per_m: float = Field(default=0.0, ge=0)
    rebound_stop_damping_n_s_per_m: float = Field(default=0.0, ge=0)
    # Measured force-velocity curve. When given it replaces the two constant
    # damping coefficients: a real shock is neither linear nor symmetric about
    # zero velocity, so fitting one to two constants would not be the measured
    # element. Positive force resists extension.
    damper_curve_velocity_m_per_s: tuple[float, ...] = ()
    damper_curve_force_n: tuple[float, ...] = ()

    @model_validator(mode="after")
    def _valid_damper_curve(self) -> AxleSpringDamper:
        velocity = self.damper_curve_velocity_m_per_s
        force = self.damper_curve_force_n
        if len(velocity) != len(force):
            raise ValueError("damper curve velocity and force must pair up")
        if not velocity:
            return self
        if len(velocity) < 2:
            raise ValueError("a damper curve needs at least two points")
        _finite(velocity, "damper_curve_velocity_m_per_s")
        _finite(force, "damper_curve_force_n")
        if any(b <= a for a, b in zip(velocity, velocity[1:])):
            raise ValueError("damper curve velocity must strictly increase")
        return self

    @model_validator(mode="after")
    def _valid_stops(self) -> AxleSpringDamper:
        if (
            self.minimum_length_m is not None
            and self.maximum_length_m is not None
            and self.minimum_length_m >= self.maximum_length_m
        ):
            raise ValueError("minimum_length_m must be below maximum_length_m")
        if self.minimum_length_m is None and (
            self.compression_stop_stiffness_n_per_m > 0
            or self.compression_stop_damping_n_s_per_m > 0
        ):
            raise ValueError("compression stop parameters require minimum_length_m")
        if self.maximum_length_m is None and (
            self.rebound_stop_stiffness_n_per_m > 0
            or self.rebound_stop_damping_n_s_per_m > 0
        ):
            raise ValueError("rebound stop parameters require maximum_length_m")
        return self


class AxleBushing(StrictModel):
    """Passive six-axis bushing between two body-local frames."""

    name: str = Field(min_length=1)
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    frame_a_to_body_quaternion: QuaternionTuple = (1.0, 0.0, 0.0, 0.0)
    frame_b_to_body_quaternion: QuaternionTuple = (1.0, 0.0, 0.0, 0.0)
    reference_translation_in_frame_a_m: Vec3Tuple
    reference_quaternion_a_to_b: QuaternionTuple
    stiffness: Matrix6Tuple
    damping: Matrix6Tuple
    preload_in_frame_a_n_n_m: Vector6Tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    @field_validator("stiffness", "damping", mode="before")
    @classmethod
    def _validate_matrix6(cls, value: object) -> Matrix6Tuple:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (6, 6) or not np.all(np.isfinite(matrix)):
            raise ValueError("bushing matrices must be finite 6x6 matrices")
        if not np.allclose(matrix, matrix.T, atol=1e-12):
            raise ValueError("bushing matrices must be symmetric")
        if float(np.min(np.linalg.eigvalsh(matrix))) < -1e-10:
            raise ValueError("bushing matrices must be positive semidefinite")
        return tuple(tuple(float(item) for item in row) for row in matrix)

    @field_validator(
        "frame_a_to_body_quaternion",
        "frame_b_to_body_quaternion",
        "reference_quaternion_a_to_b",
    )
    @classmethod
    def _normalize_quaternion(cls, value: QuaternionTuple) -> QuaternionTuple:
        _finite(value, "bushing quaternion")
        norm = math.sqrt(sum(item * item for item in value))
        if norm <= 1e-12:
            raise ValueError("bushing quaternions must be nonzero")
        return tuple(item / norm for item in value)  # type: ignore[return-value]


class AxleAntiRollBar(StrictModel):
    """Passive torsional coupling between left and right bodies."""

    name: str = Field(min_length=1)
    body_a: str
    body_b: str
    axis_a: Vec3Tuple
    reference_quaternion_a_to_b: QuaternionTuple
    stiffness_n_m_per_rad: float = Field(ge=0)
    damping_n_m_s_per_rad: float = Field(ge=0)

    @field_validator("reference_quaternion_a_to_b")
    @classmethod
    def _normalize_reference(
        cls, value: QuaternionTuple
    ) -> QuaternionTuple:
        _finite(value, "reference_quaternion_a_to_b")
        norm = math.sqrt(sum(item * item for item in value))
        if norm <= 1e-12:
            raise ValueError("reference_quaternion_a_to_b must be nonzero")
        return tuple(item / norm for item in value)  # type: ignore[return-value]

    @model_validator(mode="after")
    def _valid_axis(self) -> AxleAntiRollBar:
        _finite(self.axis_a, "axis_a")
        if np.linalg.norm(self.axis_a) <= 1e-12:
            raise ValueError("axis_a must be nonzero")
        return self


class AxleTire(StrictModel):
    """Unilateral compliant tire contact attached to a wheel body."""

    name: str = Field(min_length=1)
    body: str
    center_local_m: Vec3Tuple = (0.0, 0.0, 0.0)
    spin_axis_local: Vec3Tuple = (0.0, 1.0, 0.0)
    forward_axis_local: Vec3Tuple = (1.0, 0.0, 0.0)
    unloaded_radius_m: float = Field(gt=0)
    maximum_compression_m: float = Field(gt=0)
    vertical_stiffness_n_per_m: float = Field(gt=0)
    vertical_damping_n_s_per_m: float = Field(ge=0)
    longitudinal_friction_coefficient: float = Field(gt=0)
    lateral_friction_coefficient: float = Field(gt=0)
    longitudinal_brush_stiffness_n_per_m: float = Field(gt=0)
    lateral_brush_stiffness_n_per_m: float = Field(gt=0)
    longitudinal_relaxation_length_m: float = Field(gt=0)
    lateral_relaxation_length_m: float = Field(gt=0)
    detached_relaxation_s: float = Field(gt=0)

    @model_validator(mode="after")
    def _valid_axes(self) -> AxleTire:
        _finite(self.spin_axis_local, "spin_axis_local")
        _finite(self.forward_axis_local, "forward_axis_local")
        spin = np.asarray(self.spin_axis_local, dtype=float)
        forward = np.asarray(self.forward_axis_local, dtype=float)
        if np.linalg.norm(spin) <= 1e-12 or np.linalg.norm(forward) <= 1e-12:
            raise ValueError("tire axes must be nonzero")
        if np.linalg.norm(np.cross(spin, forward)) <= 1e-12:
            raise ValueError("tire spin and forward axes must not be parallel")
        if self.maximum_compression_m >= self.unloaded_radius_m:
            raise ValueError("maximum_compression_m must be below tire radius")
        return self


class AxleDynamicsModel(StrictModel):
    """Complete native axle physical model."""

    schema_version: Literal[1] = 1
    name: str = Field(min_length=1)
    units: Literal["SI"] = "SI"
    coordinate_system: Literal["vehicle_x_rear_y_right_z_up"] = (
        "vehicle_x_rear_y_right_z_up"
    )
    bodies: tuple[AxleBody, ...]
    joints: tuple[AxleJoint, ...]
    springs: tuple[AxleSpringDamper, ...] = ()
    bushings: tuple[AxleBushing, ...] = ()
    anti_roll_bars: tuple[AxleAntiRollBar, ...] = ()
    tires: tuple[AxleTire, ...] = ()
    gravity_m_per_s2: Vec3Tuple = (0.0, 0.0, -9.80665)

    @model_validator(mode="after")
    def _closed_model(self) -> AxleDynamicsModel:
        names = [body.name for body in self.bodies]
        if len(names) != len(set(names)):
            raise ValueError("body names must be unique")
        if not any(body.fixed for body in self.bodies):
            raise ValueError("an axle model requires at least one fixed fixture body")
        known = set(names)
        element_names = [
            *(joint.name for joint in self.joints),
            *(spring.name for spring in self.springs),
            *(bushing.name for bushing in self.bushings),
            *(bar.name for bar in self.anti_roll_bars),
            *(tire.name for tire in self.tires),
        ]
        if len(element_names) != len(set(element_names)):
            raise ValueError("joint, spring, and tire names must be unique")
        for joint in self.joints:
            if joint.body_a not in known or joint.body_b not in known:
                raise ValueError(f"joint {joint.name!r} references an unknown body")
        for spring in self.springs:
            if spring.body_a not in known or spring.body_b not in known:
                raise ValueError(f"spring {spring.name!r} references an unknown body")
        for bushing in self.bushings:
            if bushing.body_a not in known or bushing.body_b not in known:
                raise ValueError(f"bushing {bushing.name!r} references an unknown body")
        for bar in self.anti_roll_bars:
            if bar.body_a not in known or bar.body_b not in known:
                raise ValueError(
                    f"anti-roll bar {bar.name!r} references an unknown body"
                )
        for tire in self.tires:
            if tire.body not in known:
                raise ValueError(f"tire {tire.name!r} references an unknown body")
        _finite(self.gravity_m_per_s2, "gravity_m_per_s2")
        return self


class AxleSolverSettings(StrictModel):
    """
    Native time-integration settings.

    ``ggl_generalized_alpha`` remains the native default.  The explicit HHT
    mode is used when a comparison manifest pins the same Adams HHT alpha.
    """

    integrator: Literal["ggl_generalized_alpha", "hht"] = (
        "ggl_generalized_alpha"
    )
    rho_inf: float = Field(default=0.8, gt=0, le=1)
    hht_alpha: float = Field(default=-0.3, ge=-1.0 / 3.0, le=0)
    initialization_mode: Literal[
        "static_equilibrium", "provided_consistent_state"
    ] = "static_equilibrium"
    adaptive_step: bool = True
    internal_step_s: float = Field(default=0.00025, gt=0)
    minimum_step_s: float = Field(default=1e-6, gt=0)
    maximum_step_s: float = Field(default=0.001, gt=0)
    local_relative_tolerance: float = Field(default=1e-5, gt=0)
    local_position_tolerance_m: float = Field(default=1e-7, gt=0)
    local_angle_tolerance_rad: float = Field(default=1e-7, gt=0)
    local_velocity_tolerance_m_per_s: float = Field(default=1e-6, gt=0)
    local_angular_velocity_tolerance_rad_per_s: float = Field(
        default=1e-6, gt=0
    )
    local_brush_tolerance_m: float = Field(default=1e-7, gt=0)
    contact_event_tolerance_s: float = Field(default=1e-6, gt=0)
    max_newton_iterations: int = Field(default=20, ge=1, le=100)
    max_line_search_iterations: int = Field(default=10, ge=1, le=30)
    position_tolerance_m: float = Field(default=1e-8, gt=0)
    velocity_tolerance_m_per_s: float = Field(default=1e-7, gt=0)
    dynamics_tolerance: float = Field(default=1e-8, gt=0)
    increment_tolerance: float = Field(default=1e-8, gt=0)

    @model_validator(mode="after")
    def _step_bounds(self) -> AxleSolverSettings:
        if self.minimum_step_s > self.internal_step_s:
            raise ValueError("minimum_step_s must not exceed internal_step_s")
        if self.internal_step_s > self.maximum_step_s:
            raise ValueError("internal_step_s must not exceed maximum_step_s")
        return self


class AxleHarmonicRoad(StrictModel):
    """
    A road height that both solvers evaluate from the same closed form.

    A sampled sine has to be approximated by one solver or the other, which
    makes the comparison a test of the interpolation rather than of the
    physics.  Declaring the harmonic analytically lets each side evaluate
    `offset + amplitude * sin(2*pi*frequency*t + phase)` exactly.
    """

    tire: str = Field(min_length=1)
    offset_m: float
    amplitude_m: float = Field(ge=0)
    frequency_hz: float = Field(gt=0)
    phase_rad: float = 0.0


class AxleDynamicsCase(StrictModel):
    """Sampled road and wheel-load inputs on one public time grid."""

    schema_version: Literal[1] = 1
    name: str = Field(min_length=1)
    times_s: tuple[float, ...]
    road_height_m: dict[str, tuple[float, ...]] = Field(default_factory=dict)
    road_velocity_m_per_s: dict[str, tuple[float, ...]] = Field(
        default_factory=dict
    )
    wheel_torque_n_m: dict[str, tuple[float, ...]] = Field(default_factory=dict)
    body_wrench_n_n_m: dict[str, tuple[Vector6Tuple, ...]] = Field(
        default_factory=dict
    )
    # When a tire appears here its road height and velocity come from the
    # closed form rather than from the sampled tables, so no interpolation
    # enters the comparison on either side.
    harmonic_roads: tuple[AxleHarmonicRoad, ...] = ()
    solver: AxleSolverSettings = Field(default_factory=AxleSolverSettings)

    @model_validator(mode="after")
    def _signals_match(self) -> AxleDynamicsCase:
        if len(self.times_s) < 2:
            raise ValueError("times_s requires at least two samples")
        if any(not math.isfinite(value) for value in self.times_s):
            raise ValueError("times_s must be finite")
        if any(b <= a for a, b in zip(self.times_s, self.times_s[1:])):
            raise ValueError("times_s must be strictly increasing")
        # A declared harmonic owns its tire's road signal outright: the sampled
        # tables are (re)filled from the same closed form the exported Adams
        # dataset uses, so a round trip through the manifest reproduces exactly
        # the same numbers rather than being rejected as a double definition.
        declared = [road.tire for road in self.harmonic_roads]
        if len(declared) != len(set(declared)):
            raise ValueError("each tire may declare at most one harmonic road")
        for road in self.harmonic_roads:
            angle = [
                2.0 * math.pi * road.frequency_hz * time + road.phase_rad
                for time in self.times_s
            ]
            rate = 2.0 * math.pi * road.frequency_hz
            object.__setattr__(
                self,
                "road_height_m",
                {
                    **self.road_height_m,
                    road.tire: tuple(
                        road.offset_m + road.amplitude_m * math.sin(value)
                        for value in angle
                    ),
                },
            )
            object.__setattr__(
                self,
                "road_velocity_m_per_s",
                {
                    **self.road_velocity_m_per_s,
                    road.tire: tuple(
                        road.amplitude_m * rate * math.cos(value)
                        for value in angle
                    ),
                },
            )
        for signals in (
            self.road_height_m,
            self.road_velocity_m_per_s,
            self.wheel_torque_n_m,
        ):
            for name, values in signals.items():
                if len(values) != len(self.times_s):
                    raise ValueError(f"signal {name!r} length must match times_s")
                if any(not math.isfinite(value) for value in values):
                    raise ValueError(f"signal {name!r} must be finite")
        for name, values in self.body_wrench_n_n_m.items():
            if len(values) != len(self.times_s):
                raise ValueError(f"signal {name!r} length must match times_s")
            if any(
                not math.isfinite(component)
                for wrench in values
                for component in wrench
            ):
                raise ValueError(f"signal {name!r} must be finite")
        return self
