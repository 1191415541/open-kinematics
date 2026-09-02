"""Full-vehicle multibody model and time-domain case schemas."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .common import (
    CoordinateSystem,
    Pose,
    Quaternion,
    StrictModel,
    UnitSystem,
    Vec3,
)
from .dynamic import DynamicSolverSettings, InitialBodyState, TimeSignal, TireModelSpec
from .model import FrontAxleModel, RigidBodySpec


class AerodynamicDragSpec(StrictModel):
    """Quadratic aerodynamic drag applied to the chassis."""

    air_density: float = Field(gt=0)
    drag_coefficient: float = Field(ge=0)
    frontal_area: float = Field(gt=0)
    application_point: Vec3 = Field(default_factory=Vec3)
    forward_axis: Vec3 = Vec3(x=1.0, y=0.0, z=0.0)

    @model_validator(mode="after")
    def _valid_forward_axis(self) -> AerodynamicDragSpec:
        axis = self.forward_axis.as_array()
        if not math.isfinite(float(axis @ axis)) or float(axis @ axis) <= 1e-12:
            raise ValueError("aerodynamic forward_axis must be non-zero")
        return self


class WheelSpec(StrictModel):
    """One wheel-end rotational body and its tire parameters."""

    name: Literal["front_left", "front_right", "rear_left", "rear_right"]
    body: str
    center_local: Vec3
    steering_axis: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    spin_axis: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    forward_axis: Vec3 | None = None
    pose: Pose = Field(default_factory=Pose)
    inertia: tuple[tuple[float, ...], ...] | None = None
    mount_body: str | None = None
    mount_joint_kind: Literal["revolute", "fixed"] = "revolute"
    # Optional driveline actuator mapping. The axis is local to
    # ``drive_torque_body``; without it, drive torque keeps the historical
    # wheel-body application used by generic vehicle models.
    drive_torque_body: str | None = None
    drive_torque_reaction_body: str | None = None
    drive_torque_axis_local: Vec3 | None = None
    # 仅用于静态配平的被动转轴；动态积分仍保留完整轮端刚体。
    static_rotation_axis_local: Vec3 | None = None
    mass: float = Field(default=0.0, ge=0)
    axial_inertia: float = Field(default=1.0, gt=0)
    tire: TireModelSpec = Field(default_factory=TireModelSpec)
    driven: bool = False
    braked: bool = True

    @field_validator("inertia", mode="before")
    @classmethod
    def _inertia_shape(
        cls, value: object
    ) -> tuple[tuple[float, ...], ...] | None:
        if value is None:
            return None
        rows = tuple(tuple(float(item) for item in row) for row in value)  # type: ignore[union-attr]
        if len(rows) != 3 or any(len(row) != 3 for row in rows):
            raise ValueError("wheel inertia must be a 3x3 matrix")
        if any(not math.isfinite(item) for row in rows for item in row):
            raise ValueError("wheel inertia must contain finite values")
        return rows

    @model_validator(mode="after")
    def _vectors_and_inertia(self) -> WheelSpec:
        for name, vector in (
            ("steering_axis", self.steering_axis),
            ("spin_axis", self.spin_axis),
        ):
            values = vector.as_array()
            if not math.isfinite(float(values @ values)) or float(values @ values) <= 1e-12:
                raise ValueError(f"{name} must be a non-zero finite vector")
        if self.forward_axis is not None:
            forward = self.forward_axis.as_array()
            if not math.isfinite(float(forward @ forward)) or float(forward @ forward) <= 1e-12:
                raise ValueError("forward_axis must be a non-zero finite vector")
            spin = self.spin_axis.as_array()
            cross = (
                (spin[1] * forward[2] - spin[2] * forward[1]) ** 2
                + (spin[2] * forward[0] - spin[0] * forward[2]) ** 2
                + (spin[0] * forward[1] - spin[1] * forward[0]) ** 2
            )
            if float(cross) <= 1e-12 * float(spin @ spin) * float(forward @ forward):
                raise ValueError("forward_axis must not be parallel to spin_axis")
        if self.static_rotation_axis_local is not None:
            values = self.static_rotation_axis_local.as_array()
            if not math.isfinite(float(values @ values)) or float(values @ values) <= 1e-12:
                raise ValueError("static_rotation_axis_local must be non-zero")
        if (self.drive_torque_body is None) != (
            self.drive_torque_axis_local is None
        ):
            raise ValueError(
                "drive_torque_body and drive_torque_axis_local must be provided together"
            )
        if self.drive_torque_reaction_body is not None:
            if self.drive_torque_body is None:
                raise ValueError(
                    "drive_torque_reaction_body requires drive_torque_body"
                )
            if self.drive_torque_reaction_body == self.drive_torque_body:
                raise ValueError("drive torque body and reaction body must differ")
        if self.drive_torque_axis_local is not None:
            values = self.drive_torque_axis_local.as_array()
            if not math.isfinite(float(values @ values)) or float(values @ values) <= 1e-12:
                raise ValueError("drive_torque_axis_local must be non-zero")
        return self


class SteeringSystemSpec(StrictModel):
    """Rack-and-pinion steering actuator boundary."""

    rack_body: str = "rack"
    ratio: float = Field(gt=0)
    max_rack_displacement: float = Field(default=100.0, gt=0)
    input: Literal["rack_displacement", "steering_wheel_angle"] = "rack_displacement"
    rack_displacement_per_steering_wheel_angle: float | None = Field(default=None, gt=0)
    rack_stiffness: float = Field(default=20_000.0, gt=0)
    rack_damping: float = Field(default=500.0, ge=0)
    max_steering_angle: float = Field(default=math.pi, gt=0)
    actuator_mode: Literal[
        "rack_translation",
        "prescribed_rotation",
        "prescribed_translation",
    ] = "rack_translation"
    actuator_body: str | None = None
    actuator_reaction_body: str | None = None
    actuator_axis_local: Vec3 = Vec3(x=0.0, y=0.0, z=1.0)
    actuator_reference_rotation: Quaternion = Field(default_factory=Quaternion)


class JointCoordinateCouplerSpec(StrictModel):
    """Linear relation between two ideal-joint coordinates."""

    name: str
    joint_a: str
    coordinate_a: Literal["rotation", "translation"]
    scale_a: float
    joint_b: str
    coordinate_b: Literal["rotation", "translation"]
    scale_b: float

    @model_validator(mode="after")
    def _valid_relation(self) -> JointCoordinateCouplerSpec:
        if self.joint_a == self.joint_b:
            raise ValueError("a coordinate coupler requires two different joints")
        if not math.isfinite(self.scale_a) or not math.isfinite(self.scale_b):
            raise ValueError("coordinate coupler scales must be finite")
        if abs(self.scale_a) <= 1e-12 or abs(self.scale_b) <= 1e-12:
            raise ValueError("coordinate coupler scales must be non-zero")
        return self


class DrivelineSpec(StrictModel):
    """Simplified but torque-based brake and drive actuator contract."""

    driven_wheels: tuple[str, ...] = ()
    maximum_drive_torque: float = Field(default=0.0, ge=0)
    maximum_brake_torque: float = Field(default=10_000.0, ge=0)
    front_brake_bias: float = Field(default=0.6, ge=0, le=1)
    drive_split: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)

    @model_validator(mode="after")
    def _validate_distribution(self) -> DrivelineSpec:
        if any(name not in {"front_left", "front_right", "rear_left", "rear_right"} for name in self.driven_wheels):
            raise ValueError("driven_wheels contains an unknown wheel")
        if len(set(self.driven_wheels)) != len(self.driven_wheels):
            raise ValueError("driven_wheels must be unique")
        if any(value < 0 or not math.isfinite(value) for value in self.drive_split):
            raise ValueError("drive_split must contain finite non-negative values")
        if self.maximum_drive_torque > 0 and sum(self.drive_split) <= 0:
            raise ValueError("drive_split is required when drive torque is enabled")
        if sum(self.drive_split) > 0 and abs(sum(self.drive_split) - 1.0) > 1e-9:
            raise ValueError("drive_split must sum to one")
        return self


class RoadSurfaceSpec(StrictModel):
    """Analytic road surface queried by the tire contact evaluator."""

    kind: Literal["plane", "sine", "bump", "random_fourier", "four_post"] = "plane"
    origin: Vec3 = Field(default_factory=Vec3)
    normal: Vec3 = Vec3(x=0.0, y=0.0, z=1.0)
    amplitude: float = Field(default=0.0, ge=0)
    wavelength: float = Field(default=1_000.0, gt=0)
    phase: float = 0.0
    bump_start: float = 0.0
    bump_length: float = Field(default=500.0, gt=0)
    corner_scales: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    corner_height_signals: tuple[TimeSignal, TimeSignal, TimeSignal, TimeSignal] | None = None
    friction_coefficient: float = Field(default=1.0, gt=0)

    @model_validator(mode="after")
    def _normal_and_kind(self) -> RoadSurfaceSpec:
        normal = self.normal.as_array()
        if not all(math.isfinite(float(value)) for value in normal) or float(normal @ normal) <= 1e-12:
            raise ValueError("road normal must be a non-zero finite vector")
        if normal[2] <= 0.0:
            raise ValueError("road normal must point upward with a positive z component")
        if self.kind == "plane" and self.amplitude != 0.0:
            raise ValueError("plane road must have zero amplitude")
        if any(not math.isfinite(value) or value < 0.0 for value in self.corner_scales):
            raise ValueError("corner_scales must contain finite non-negative values")
        return self


class VehicleModel(StrictModel):
    """Explicit four-corner full-vehicle multibody model."""

    schema_version: int = Field(default=1, ge=1)
    name: str = "full_vehicle"
    units: UnitSystem = UnitSystem.ENGINEERING
    coordinate_system: CoordinateSystem = CoordinateSystem.VEHICLE
    chassis: RigidBodySpec
    front_axle: FrontAxleModel
    rear_axle: FrontAxleModel
    wheels: tuple[WheelSpec, ...]
    steering: SteeringSystemSpec
    driveline: DrivelineSpec = Field(default_factory=DrivelineSpec)
    coordinate_couplers: tuple[JointCoordinateCouplerSpec, ...] = ()
    aerodynamic_drag: AerodynamicDragSpec | None = None

    @model_validator(mode="after")
    def _topology(self) -> VehicleModel:
        names = tuple(wheel.name for wheel in self.wheels)
        required = {"front_left", "front_right", "rear_left", "rear_right"}
        if set(names) != required or len(names) != 4:
            raise ValueError("vehicle must define exactly one wheel for each of four corners")
        bodies = [self.chassis.name, *(wheel.body for wheel in self.wheels)]
        if len(set(bodies)) != len(bodies):
            raise ValueError("chassis and wheel body names must be unique")
        if any(name not in names for name in self.driveline.driven_wheels):
            raise ValueError("driveline references an undefined wheel")
        required_bodies = {
            "rack",
            "upper_arm_L",
            "upper_arm_R",
            "lower_arm_L",
            "lower_arm_R",
            "upright_L",
            "upright_R",
            "tie_rod_L",
            "tie_rod_R",
        }
        for axle_name, axle in (("front", self.front_axle), ("rear", self.rear_axle)):
            specs = {body.name: body for body in axle.bodies}
            if len(specs) != len(axle.bodies):
                raise ValueError(f"{axle_name} axle body names must be unique")
            if axle.topology == "symmetric_proxy":
                missing = sorted(required_bodies - set(specs))
                if missing:
                    raise ValueError(
                        f"{axle_name} axle requires positive mass specs for: {', '.join(missing)}"
                    )
                if any(specs[name].mass <= 0.0 for name in required_bodies):
                    raise ValueError(
                        f"{axle_name} axle body mass specs must be positive"
                    )
            elif not specs:
                raise ValueError(f"{axle_name} explicit axle has no body specs")
            joint_names = [joint.name for joint in axle.joints]
            if len(joint_names) != len(set(joint_names)):
                raise ValueError(f"{axle_name} explicit joint names must be unique")
            known = {self.chassis.name, *specs}
            for joint in axle.joints:
                if joint.body_a not in known or joint.body_b not in known:
                    raise ValueError(
                        f"{axle_name} joint {joint.name!r} references an undefined body"
                    )
        runtime_joint_names = {
            *(f"front_{joint.name}" for joint in self.front_axle.joints),
            *(f"rear_{joint.name}" for joint in self.rear_axle.joints),
        }
        coupler_names = [coupler.name for coupler in self.coordinate_couplers]
        if len(coupler_names) != len(set(coupler_names)):
            raise ValueError("coordinate coupler names must be unique")
        for coupler in self.coordinate_couplers:
            for joint_name in (coupler.joint_a, coupler.joint_b):
                if joint_name not in runtime_joint_names:
                    raise ValueError(
                        f"coordinate coupler {coupler.name!r} references an undefined joint"
                    )
        return self


class VehicleDynamicCase(StrictModel):
    """Time-domain case for the full-vehicle multibody solver."""

    name: str = "full_vehicle_dynamic"
    solver: DynamicSolverSettings
    vehicle: VehicleModel
    road: RoadSurfaceSpec = Field(default_factory=RoadSurfaceSpec)
    steering_input: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    brake_input: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    drive_input: TimeSignal = Field(default_factory=lambda: TimeSignal(constant=0.0))
    # These optional signals are direct per-wheel torque overrides in the
    # VehicleModel torque units.  They are intentionally separate from the
    # normalized global drive/brake commands.
    wheel_drive_torque: tuple[tuple[str, TimeSignal], ...] = ()
    wheel_brake_torque: tuple[tuple[str, TimeSignal], ...] = ()
    initial_wheel_speeds: tuple[tuple[str, float], ...] = ()
    initial_states: tuple[InitialBodyState, ...] = ()
    static_equilibrium: bool = False
    # ``auto`` retains ideal inboard joints when no physical bushing data is
    # present, and selects compliant C mode when the model supplies bushings.
    # Explicit K/C is retained for reproducible comparisons and is validated
    # by the native adapter instead of silently dropping elements.
    suspension_mode: Literal["auto", "K", "C"] = "auto"
    initial_forward_speed_mps: float = Field(default=0.0, ge=0)
    # 速度大小与车辆坐标轴方向分开表达；Adams 源模型可能沿 -X 行驶。
    initial_velocity_sign: Literal[-1, 1] = 1

    @model_validator(mode="after")
    def _initial_wheel_speeds(self) -> VehicleDynamicCase:
        names = {wheel.name for wheel in self.vehicle.wheels}
        provided = [name for name, _ in self.initial_wheel_speeds]
        if any(name not in names for name in provided):
            raise ValueError("initial_wheel_speeds references an undefined wheel")
        if len(provided) != len(set(provided)):
            raise ValueError("initial_wheel_speeds must be unique")
        if any(not math.isfinite(value) for _, value in self.initial_wheel_speeds):
            raise ValueError("initial wheel speeds must be finite")
        state_bodies = [item.body for item in self.initial_states]
        if len(state_bodies) != len(set(state_bodies)):
            raise ValueError("initial states must contain unique bodies")
        known_bodies = {self.vehicle.chassis.name}
        known_bodies.update(body.name for body in self.vehicle.front_axle.bodies)
        known_bodies.update(body.name for body in self.vehicle.rear_axle.bodies)
        known_bodies.update(
            f"front_{body.name}" for body in self.vehicle.front_axle.bodies
        )
        known_bodies.update(
            f"rear_{body.name}" for body in self.vehicle.rear_axle.bodies
        )
        known_bodies.update(wheel.body for wheel in self.vehicle.wheels)
        if any(name not in known_bodies for name in state_bodies):
            raise ValueError("initial states reference an undefined body")
        if not math.isfinite(self.initial_forward_speed_mps):
            raise ValueError("initial_forward_speed_mps must be finite")
        return self

    @model_validator(mode="after")
    def _direct_wheel_torque_signals(self) -> VehicleDynamicCase:
        names = {wheel.name for wheel in self.vehicle.wheels}

        def validate_signals(
            entries: tuple[tuple[str, TimeSignal], ...],
            label: str,
            nonnegative: bool,
        ) -> None:
            entry_names = [name for name, _ in entries]
            if any(name not in names for name in entry_names):
                raise ValueError(f"{label} references an undefined wheel")
            if len(entry_names) != len(set(entry_names)):
                raise ValueError(f"{label} must contain unique wheels")
            for name, signal in entries:
                values = (
                    (signal.constant,)
                    if signal.constant is not None
                    else signal.values
                )
                if nonnegative and any(value < 0.0 for value in values):
                    raise ValueError(f"{label}[{name!r}] must be non-negative")

        validate_signals(self.wheel_drive_torque, "wheel_drive_torque", False)
        validate_signals(self.wheel_brake_torque, "wheel_brake_torque", True)

        def is_zero(signal: TimeSignal) -> bool:
            values = (
                (signal.constant,)
                if signal.constant is not None
                else signal.values
            )
            return all(abs(value) <= 1e-12 for value in values)

        if self.wheel_drive_torque and not is_zero(self.drive_input):
            raise ValueError(
                "wheel_drive_torque cannot be combined with nonzero drive_input"
            )
        if self.wheel_brake_torque and not is_zero(self.brake_input):
            raise ValueError(
                "wheel_brake_torque cannot be combined with nonzero brake_input"
            )
        return self
