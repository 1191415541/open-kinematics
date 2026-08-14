"""Full-vehicle multibody model and time-domain case schemas."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field, model_validator

from .common import CoordinateSystem, Pose, StrictModel, UnitSystem, Vec3
from .dynamic import DynamicSolverSettings, InitialBodyState, TimeSignal, TireModelSpec
from .model import FrontAxleModel, RigidBodySpec


class WheelSpec(StrictModel):
    """One wheel-end rotational body and its tire parameters."""

    name: Literal["front_left", "front_right", "rear_left", "rear_right"]
    body: str
    center_local: Vec3
    steering_axis: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    spin_axis: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    pose: Pose = Field(default_factory=Pose)
    mass: float = Field(default=0.0, ge=0)
    axial_inertia: float = Field(default=1.0, gt=0)
    tire: TireModelSpec = Field(default_factory=TireModelSpec)
    driven: bool = False
    braked: bool = True

    @model_validator(mode="after")
    def _vectors_and_inertia(self) -> WheelSpec:
        for name, vector in (
            ("steering_axis", self.steering_axis),
            ("spin_axis", self.spin_axis),
        ):
            values = vector.as_array()
            if not math.isfinite(float(values @ values)) or float(values @ values) <= 1e-12:
                raise ValueError(f"{name} must be a non-zero finite vector")
        return self


class SteeringSystemSpec(StrictModel):
    """Rack-and-pinion steering actuator boundary."""

    rack_body: str = "rack"
    ratio: float = Field(gt=0)
    max_rack_displacement: float = Field(default=100.0, gt=0)
    input: Literal["rack_displacement", "steering_wheel_angle"] = "rack_displacement"
    rack_displacement_per_steering_wheel_angle: float | None = Field(default=None, gt=0)


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
            missing = sorted(required_bodies - set(specs))
            if missing:
                raise ValueError(
                    f"{axle_name} axle requires positive mass specs for: {', '.join(missing)}"
                )
            if any(specs[name].mass <= 0.0 for name in required_bodies):
                raise ValueError(f"{axle_name} axle body mass specs must be positive")
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
    initial_wheel_speeds: tuple[tuple[str, float], ...] = ()
    initial_states: tuple[InitialBodyState, ...] = ()
    static_equilibrium: bool = False
    initial_forward_speed_mps: float = Field(default=0.0, ge=0)

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
        known_bodies.update(wheel.body for wheel in self.vehicle.wheels)
        if any(name not in known_bodies for name in state_bodies):
            raise ValueError("initial states reference an undefined body")
        if not math.isfinite(self.initial_forward_speed_mps):
            raise ValueError("initial_forward_speed_mps must be finite")
        return self
