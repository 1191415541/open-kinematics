"""Front axle model schema."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .common import (
    CoordinateSystem,
    Pose,
    SchemaVersion,
    StrictModel,
    UnitSystem,
    Vec3,
)
from .elements import (
    AntiRollBar,
    BumpStop,
    Bushing6x6,
    LinearSpring,
    StaticDamper,
    VerticalTire,
)


class MassSpec(StrictModel):
    """Lumped or detailed mass definition."""

    sprung_mass: float = Field(gt=0)
    front_unsprung_ratio: float = Field(default=0.10, ge=0)
    rear_unsprung_ratio: float = Field(default=0.12, ge=0)
    axle_sprung_mass: float | None = Field(default=None, gt=0)
    wheel_load: float | None = Field(default=None, ge=0)
    unsprung_cg_offset: Vec3 = Field(default_factory=Vec3)
    detailed_unsprung: tuple[tuple[str, float, Vec3], ...] = ()

    @field_validator(
        "sprung_mass",
        "front_unsprung_ratio",
        "rear_unsprung_ratio",
        "axle_sprung_mass",
        "wheel_load",
    )
    @classmethod
    def _finite(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("mass values must be finite")
        return value

    @model_validator(mode="after")
    def _mass_consistency(self) -> MassSpec:
        if self.front_unsprung_ratio > 1 or self.rear_unsprung_ratio > 1:
            raise ValueError("unsprung ratios must not exceed one")
        detailed = sum(item[1] for item in self.detailed_unsprung)
        target = self.sprung_mass * self.front_unsprung_ratio
        if detailed > target + 1e-9:
            raise ValueError("detailed unsprung mass exceeds target unsprung mass")
        return self


class RigidBodySpec(StrictModel):
    name: str
    pose: Pose = Field(default_factory=Pose)
    mass: float = Field(default=0.0, ge=0)
    center_of_mass: Vec3 = Field(default_factory=Vec3)
    inertia: tuple[tuple[float, ...], ...] = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    fixed: bool = False
    # 仅用于静态配平的被动转轴；动态积分仍保留完整刚体自由度。
    static_rotation_axis_local: Vec3 | None = None

    @field_validator("inertia", mode="before")
    @classmethod
    def _inertia_shape(cls, value: object) -> tuple[tuple[float, ...], ...]:
        rows = tuple(tuple(float(item) for item in row) for row in value)  # type: ignore[union-attr]
        if len(rows) != 3 or any(len(row) != 3 for row in rows):
            raise ValueError("body inertia must be a 3x3 matrix")
        if any(not math.isfinite(item) for row in rows for item in row):
            raise ValueError("body inertia must contain finite values")
        return rows

    @model_validator(mode="after")
    def _static_axis(self) -> RigidBodySpec:
        if self.static_rotation_axis_local is not None:
            axis = self.static_rotation_axis_local.as_array()
            if not math.isfinite(float(axis @ axis)) or float(axis @ axis) <= 1e-12:
                raise ValueError("static_rotation_axis_local must be non-zero")
        return self


class IdealJointSpec(StrictModel):
    """One explicitly declared ideal joint in vehicle coordinates."""

    name: str
    kind: Literal[
        "spherical",
        "revolute",
        "prismatic",
        "fixed",
        "universal",
        "constant_velocity",
        "cylindrical",
        "inplane",
    ]
    body_a: str
    body_b: str
    point_a: Vec3
    point_b: Vec3
    axis_a: Vec3 = Vec3(x=0.0, y=0.0, z=1.0)
    axis_b: Vec3 = Vec3(x=0.0, y=0.0, z=1.0)
    axis_a_secondary: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    axis_b_secondary: Vec3 = Vec3(x=1.0, y=0.0, z=0.0)
    # Adams 源初态允许 CONVEL 保留有限的交叉轴相位偏置。
    constant_velocity_angle_target: float = Field(default=0.0, ge=-2.0, le=2.0)

    @model_validator(mode="after")
    def _valid_joint(self) -> IdealJointSpec:
        if self.body_a == self.body_b:
            raise ValueError("ideal joint bodies must be different")
        if self.kind in {
            "revolute",
            "prismatic",
            "universal",
            "constant_velocity",
            "cylindrical",
            "inplane",
        }:
            if self.axis_a.as_array() @ self.axis_a.as_array() <= 1e-12:
                raise ValueError("ideal joint axis_a must be nonzero")
            if self.kind != "inplane" and self.axis_b.as_array() @ self.axis_b.as_array() <= 1e-12:
                raise ValueError("ideal joint axis_b must be nonzero")
        if self.kind == "constant_velocity":
            if self.axis_a_secondary.as_array() @ self.axis_a_secondary.as_array() <= 1e-12:
                raise ValueError("constant_velocity axis_a_secondary must be nonzero")
            if self.axis_b_secondary.as_array() @ self.axis_b_secondary.as_array() <= 1e-12:
                raise ValueError("constant_velocity axis_b_secondary must be nonzero")
        return self


class HardpointPair(StrictModel):
    """A left-side point and its generated right-side mirror."""

    name: str
    left: Vec3

    @property
    def right(self) -> Vec3:
        return self.left.mirrored_y()


class FrontAxleModel(StrictModel):
    """Versioned symmetric double-wishbone plus rack model."""

    schema_version: SchemaVersion = 1
    name: str = "front_double_wishbone"
    units: UnitSystem = UnitSystem.ENGINEERING
    coordinate_system: CoordinateSystem = CoordinateSystem.VEHICLE
    hardpoints: dict[str, Vec3]
    bodies: tuple[RigidBodySpec, ...] = ()
    mass: MassSpec
    springs: tuple[LinearSpring, ...] = ()
    dampers: tuple[StaticDamper, ...] = ()
    bushings: tuple[Bushing6x6, ...] = ()
    tires: tuple[VerticalTire, ...] = ()
    anti_roll_bars: tuple[AntiRollBar, ...] = ()
    stops: tuple[BumpStop, ...] = ()
    side: Literal["left", "right"] = "left"
    topology: Literal["symmetric_proxy", "explicit"] = "symmetric_proxy"
    joints: tuple[IdealJointSpec, ...] = ()
    rack_axis: Vec3 = Vec3(x=0.0, y=1.0, z=0.0)
    rack_fixed_to_chassis: bool = False

    @field_validator("hardpoints")
    @classmethod
    def _hardpoints_nonempty(cls, value: dict[str, Vec3]) -> dict[str, Vec3]:
        if not value:
            raise ValueError("at least one hardpoint is required")
        if len(value) != len(set(value)):
            raise ValueError("hardpoint names must be unique")
        return value

    @model_validator(mode="after")
    def _symmetric_input(self) -> FrontAxleModel:
        if self.side != "left":
            raise ValueError(
                "v1 model input must be the left side; right is generated by mirroring"
            )
        if self.topology == "symmetric_proxy" and self.joints:
            raise ValueError(
                "symmetric_proxy topology cannot contain explicit joints"
            )
        return self
