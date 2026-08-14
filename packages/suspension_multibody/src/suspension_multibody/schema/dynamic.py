"""Time-domain dynamics schemas."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .common import (
    CoordinateSystem,
    Pose,
    Provenance,
    SchemaVersion,
    SixVector,
    StrictModel,
    UnitSystem,
    Vec3,
)
from .result import Diagnostic


class TimeSignal(StrictModel):
    """Scalar time signal with constant, hold, or linear interpolation."""

    times: tuple[float, ...] = ()
    values: tuple[float, ...] = ()
    constant: float | None = None
    interpolation: Literal["linear", "hold"] = "linear"

    @model_validator(mode="after")
    def _valid_signal(self) -> TimeSignal:
        if self.constant is not None:
            if self.times or self.values:
                raise ValueError("constant signal cannot also define samples")
            if not math.isfinite(self.constant):
                raise ValueError("constant signal value must be finite")
            return self
        if len(self.times) != len(self.values) or len(self.times) < 2:
            raise ValueError("sampled signal requires matching times and values")
        if any(not math.isfinite(value) for value in (*self.times, *self.values)):
            raise ValueError("signal samples must be finite")
        if any(b <= a for a, b in zip(self.times, self.times[1:])):
            raise ValueError("signal times must be strictly increasing")
        return self

    def value_at(self, time: float) -> float:
        """Evaluate the signal at ``time``."""
        if self.constant is not None:
            return self.constant
        if time <= self.times[0]:
            return self.values[0]
        if time >= self.times[-1]:
            return self.values[-1]
        for left, right, value_left, value_right in zip(
            self.times, self.times[1:], self.values, self.values[1:]
        ):
            if left <= time <= right:
                if self.interpolation == "hold":
                    return value_left
                ratio = (time - left) / (right - left)
                return value_left + ratio * (value_right - value_left)
        return self.values[-1]

    def derivative_at(self, time: float) -> float:
        """Return the piecewise-linear time derivative at ``time``."""
        if self.constant is not None or self.interpolation == "hold":
            return 0.0
        if time < self.times[0] or time > self.times[-1]:
            return 0.0
        for left, right, value_left, value_right in zip(
            self.times, self.times[1:], self.values, self.values[1:]
        ):
            if left <= time <= right:
                return (value_right - value_left) / (right - left)
        return 0.0


class WrenchSignal(StrictModel):
    """Six-component wrench signal."""

    fx: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    fy: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    fz: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    mx: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    my: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    mz: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))

    def value_at(self, time: float) -> SixVector:
        """Evaluate the wrench at ``time``."""
        return SixVector(
            fx=self.fx.value_at(time),
            fy=self.fy.value_at(time),
            fz=self.fz.value_at(time),
            mx=self.mx.value_at(time),
            my=self.my.value_at(time),
            mz=self.mz.value_at(time),
        )


class InitialBodyState(StrictModel):
    """Initial pose and spatial velocity for a dynamic body."""

    body: str
    pose: Pose = Field(default_factory=Pose)
    velocity: SixVector = Field(default_factory=SixVector)


class PrescribedMotion(StrictModel):
    """Time-varying scalar motion constraint."""

    target: str
    displacement: TimeSignal
    velocity: TimeSignal | None = None
    acceleration: TimeSignal | None = None
    frame: Literal["global", "body", "wheel_local"] = "global"


class WrenchInput(StrictModel):
    """Time-varying wrench applied to a body or stable target."""

    target: str
    wrench: WrenchSignal
    application_point: Vec3 = Field(default_factory=Vec3)
    frame: Literal["global", "body", "wheel_local"] = "global"
    moment_reference: Literal["global_origin", "body_origin", "application_point"] = (
        "global_origin"
    )


class DynamicSolverSettings(StrictModel):
    """Time integration and output settings."""

    start_time: float = 0.0
    end_time: float = Field(gt=0)
    step_size: float = Field(gt=0)
    internal_step_size: float = Field(default=1e-3, gt=0)
    min_internal_step_size: float = Field(default=1e-4, gt=0)
    adaptive_substepping: bool = True
    projection_failure_tolerance: float = Field(default=0.01, gt=0)
    output_step: float | None = Field(default=None, gt=0)
    integrator: Literal["semi_implicit_euler", "newmark", "generalized_alpha"] = (
        "semi_implicit_euler"
    )
    gravity: Vec3 = Vec3(x=0.0, y=0.0, z=-9810.0)
    # Engineering coordinates use mm and N while body masses are kg.  The
    # legacy value 1.0 preserves existing callers; Adams-compatible runs use
    # 1000.0 so kg-mm spatial inertia and mm/s^2 acceleration produce N/N-mm.
    mass_matrix_scale: float = Field(default=1.0, gt=0)
    global_velocity_damping: float = Field(default=0.0, ge=0)
    initial_force_ramp_time: float = Field(default=0.0, ge=0)
    max_linear_acceleration: float = Field(default=1.0e9, gt=0)
    max_angular_acceleration: float = Field(default=1.0e6, gt=0)
    max_linear_velocity: float = Field(default=1.0e9, gt=0)
    max_angular_velocity: float = Field(default=1.0e4, gt=0)
    velocity_recovery_enabled: bool = False
    velocity_recovery_linear_limit: float = Field(default=1.0e6, gt=0)
    velocity_recovery_angular_limit: float = Field(default=1.0e4, gt=0)
    constraint_tolerance: float = Field(default=1e-7, gt=0)
    velocity_tolerance: float = Field(default=1e-6, gt=0)
    event_tolerance: float = Field(default=1e-6, gt=0)
    constraint_stabilization_alpha: float = Field(default=8.0, ge=0)
    constraint_stabilization_beta: float = Field(default=20.0, ge=0)
    constraint_derivative_step: float = Field(default=1e-6, gt=0)
    projection_translation_limit: float = Field(default=100.0, gt=0)
    projection_rotation_limit: float = Field(default=0.25, gt=0)
    projection_max_iterations: int = Field(default=30, ge=1, le=200)
    projection_backtracking: int = Field(default=12, ge=1, le=30)
    max_corrector_iterations: int = Field(default=3, ge=1, le=12)
    reuse_constraint_linearization: bool = False
    newmark_beta: float = Field(default=0.25, gt=0, le=0.5)
    newmark_gamma: float = Field(default=0.5, gt=0, le=1.0)
    generalized_alpha_rho_inf: float = Field(default=0.8, gt=0, le=1.0)
    allow_static_element_downgrade: bool = False

    @model_validator(mode="after")
    def _time_window(self) -> DynamicSolverSettings:
        if self.end_time <= self.start_time:
            raise ValueError("end_time must be greater than start_time")
        if self.output_step is not None and self.output_step < self.step_size:
            raise ValueError("output_step must not be smaller than step_size")
        return self


class TireModelSpec(StrictModel):
    """Dynamic tire model selection and core parameters."""

    kind: Literal["vertical_linear", "fiala", "pac2002"] = "vertical_linear"
    parameter_source: Literal["user", "adams_builtin"] = "user"
    unloaded_radius: float = Field(default=300.0, gt=0)
    vertical_stiffness: float = Field(default=200.0, gt=0)
    cornering_stiffness: float = Field(default=80_000.0, gt=0)
    longitudinal_stiffness: float = Field(default=120_000.0, gt=0)
    friction_coefficient: float = Field(default=1.0, gt=0)
    relaxation_length: float = Field(default=0.0, ge=0)
    vertical_damping: float = Field(default=5.0, ge=0)
    minimum_slip_speed: float = Field(default=10.0, gt=0)
    pneumatic_trail: float = Field(default=50.0, ge=0)
    pac2002_coefficients: dict[str, float] = Field(default_factory=dict)

    @field_validator("pac2002_coefficients")
    @classmethod
    def _finite_pac2002_coefficients(cls, value: dict[str, float]) -> dict[str, float]:
        if any(not key or not math.isfinite(float(item)) for key, item in value.items()):
            raise ValueError("PAC2002 coefficients must have finite numeric values")
        return {str(key): float(item) for key, item in value.items()}


class VehicleBodyModel(StrictModel):
    """Simplified full-vehicle body model boundary."""

    name: str = "vehicle_body"
    degrees_of_freedom: Literal[14, 15] = 14
    mass: float = Field(gt=0)
    center_of_mass: Vec3 = Field(default_factory=Vec3)
    inertia: tuple[tuple[float, ...], ...]
    wheelbase: float = Field(gt=0)
    front_track: float = Field(gt=0)
    rear_track: float = Field(gt=0)

    @field_validator("inertia", mode="before")
    @classmethod
    def _inertia_shape(cls, value: object) -> tuple[tuple[float, ...], ...]:
        rows = tuple(tuple(float(item) for item in row) for row in value)  # type: ignore[union-attr]
        if len(rows) != 3 or any(len(row) != 3 for row in rows):
            raise ValueError("vehicle inertia must be a 3x3 matrix")
        if any(not math.isfinite(item) for row in rows for item in row):
            raise ValueError("vehicle inertia must contain finite values")
        return rows


class DynamicCaseSpec(StrictModel):
    """Versioned time-domain analysis case."""

    schema_version: SchemaVersion = 1
    name: str = "dynamic_case"
    mode: Literal["axle_dynamic", "vehicle_kc_dynamic", "vehicle_dynamic"]
    units: UnitSystem = UnitSystem.ENGINEERING
    coordinate_system: CoordinateSystem = CoordinateSystem.VEHICLE
    solver: DynamicSolverSettings
    initial_states: tuple[InitialBodyState, ...] = ()
    prescribed_motions: tuple[PrescribedMotion, ...] = ()
    wrench_inputs: tuple[WrenchInput, ...] = ()
    tire_model: TireModelSpec = Field(default_factory=TireModelSpec)
    vehicle: VehicleBodyModel | None = None
    checkpoint_path: str | None = None

    @model_validator(mode="after")
    def _dynamic_consistency(self) -> DynamicCaseSpec:
        if self.mode.startswith("vehicle") and self.vehicle is None:
            raise ValueError("vehicle dynamic cases require a vehicle body model")
        motion_targets = [item.target for item in self.prescribed_motions]
        if len(motion_targets) != len(set(motion_targets)):
            raise ValueError("prescribed motion targets must be unique")
        wrench_targets = [item.target for item in self.wrench_inputs]
        if len(wrench_targets) != len(set(wrench_targets)):
            raise ValueError("wrench input targets must be unique")
        conflicts = set(motion_targets) & set(wrench_targets)
        if conflicts:
            names = ", ".join(sorted(conflicts))
            raise ValueError(f"targets cannot be both prescribed and loaded: {names}")
        state_bodies = [item.body for item in self.initial_states]
        if len(state_bodies) != len(set(state_bodies)):
            raise ValueError("initial state bodies must be unique")
        return self


class DynamicTimeSample(StrictModel):
    """One time-history sample."""

    time: float
    body: str
    pose: Pose = Field(default_factory=Pose)
    velocity: SixVector = Field(default_factory=SixVector)
    acceleration: SixVector = Field(default_factory=SixVector)
    loads: dict[str, SixVector] = {}
    metrics: dict[str, float] = {}
    events: tuple[str, ...] = ()
    converged: bool = True


class DynamicManifest(StrictModel):
    """Manifest for a dynamic result bundle."""

    schema_version: SchemaVersion = 1
    format_version: str = "1.0"
    run_id: str
    mode: Literal["axle_dynamic", "vehicle_kc_dynamic", "vehicle_dynamic"]
    sample_count: int
    provenance: Provenance
    tables: tuple[str, ...] = ("time_samples", "diagnostics")


class DynamicResultBundle(StrictModel):
    """Dynamic result bundle with time-history samples."""

    manifest: DynamicManifest
    samples: tuple[DynamicTimeSample, ...] = ()
    diagnostics: tuple[Diagnostic, ...] = ()
