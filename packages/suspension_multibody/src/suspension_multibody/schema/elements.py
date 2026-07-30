"""Force-element input schemas."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field, field_validator, model_validator

from .common import Pose, StrictModel, Vec3, positive


class LinearSpring(StrictModel):
    """Linear spring with either free length or reference preload."""

    name: str
    body_a: str
    body_b: str
    point_a: Vec3
    point_b: Vec3
    stiffness: float = Field(gt=0)
    free_length: float | None = Field(default=None, gt=0)
    reference_length: float | None = Field(default=None, gt=0)
    preload: float | None = None

    @field_validator("stiffness", "free_length", "reference_length")
    @classmethod
    def _finite_positive(cls, value: float | None) -> float | None:
        if value is not None:
            positive(value, name="spring parameter")
        return value

    @field_validator("preload")
    @classmethod
    def _finite_preload(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("spring preload must be finite")
        return value

    @model_validator(mode="after")
    def _length_definition(self) -> LinearSpring:
        if (self.free_length is None) == (self.preload is None):
            raise ValueError("spring requires exactly one of free_length or preload")
        if self.preload is not None and self.reference_length is None:
            raise ValueError("reference_length is required when preload is supplied")
        return self


class StaticDamper(StrictModel):
    """Quasi-static gas/preload/friction damper model."""

    name: str
    body_a: str
    body_b: str
    point_a: Vec3
    point_b: Vec3
    gas_stiffness: float = Field(default=0.0, ge=0)
    gas_reference_length: float | None = Field(default=None, gt=0)
    gas_reference_force: float = 0.0
    preload: float = 0.0
    friction: float = Field(default=0.0, ge=0)

    @field_validator(
        "gas_stiffness",
        "gas_reference_length",
        "gas_reference_force",
        "preload",
        "friction",
    )
    @classmethod
    def _finite(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("damper parameters must be finite")
        return value

    @model_validator(mode="after")
    def _gas_reference(self) -> StaticDamper:
        if self.gas_stiffness > 0 and self.gas_reference_length is None:
            raise ValueError("gas_reference_length is required for gas stiffness")
        return self


class Bushing6x6(StrictModel):
    """Local-frame linear 6x6 bushing with optional preload and clocking."""

    name: str
    body_a: str
    body_b: str
    pose_a: Pose = Field(default_factory=Pose)
    pose_b: Pose = Field(default_factory=Pose)
    stiffness: tuple[tuple[float, ...], ...]
    preload: tuple[float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    clocking_deg: float = 0.0
    symmetry_tolerance: float = 1e-9
    positive_tolerance: float = 1e-9

    @field_validator("stiffness", mode="before")
    @classmethod
    def _matrix_shape(cls, value: object) -> tuple[tuple[float, ...], ...]:
        rows = tuple(tuple(float(item) for item in row) for row in value)  # type: ignore[union-attr]
        if len(rows) != 6 or any(len(row) != 6 for row in rows):
            raise ValueError("bushing stiffness must be a 6x6 matrix")
        if any(not math.isfinite(item) for row in rows for item in row):
            raise ValueError("bushing stiffness must contain finite values")
        return rows

    @field_validator("preload")
    @classmethod
    def _preload_shape(cls, value: object) -> tuple[float, ...]:
        values = tuple(float(item) for item in value)  # type: ignore[union-attr]
        if len(values) != 6 or any(not math.isfinite(item) for item in values):
            raise ValueError("bushing preload must contain six finite values")
        return values

    @field_validator("clocking_deg")
    @classmethod
    def _clocking(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("clocking must be finite")
        return float(value)

    @model_validator(mode="after")
    def _symmetric_psd(self) -> Bushing6x6:
        import numpy as np

        matrix = np.asarray(self.stiffness, dtype=float)
        scale = np.diag([1.0, 1.0, 1.0, 1e-3, 1e-3, 1e-3])
        normalized = scale @ matrix @ scale
        if not np.allclose(matrix, matrix.T, rtol=0.0, atol=self.symmetry_tolerance):
            raise ValueError("bushing stiffness must be symmetric")
        if float(np.linalg.eigvalsh(normalized).min()) < -self.positive_tolerance:
            raise ValueError("bushing stiffness must be positive semidefinite")
        return self


class VerticalTire(StrictModel):
    """Compression-only, single-axis linear tire."""

    stiffness: float = Field(gt=0)
    unloaded_radius: float = Field(gt=0)
    contact_point: Vec3
    local_axis: Vec3 = Vec3(x=0.0, y=0.0, z=1.0)


class AntiRollBar(StrictModel):
    """Equivalent torsional anti-roll bar defined by key hardpoints."""

    name: str
    left_body_mount: Vec3
    right_body_mount: Vec3
    left_arm_end: Vec3
    right_arm_end: Vec3
    left_link_point: Vec3
    right_link_point: Vec3
    torsional_stiffness: float = Field(gt=0)


class BumpStop(StrictModel):
    """Clearance plus post-contact linear stop."""

    name: str
    body_a: str
    body_b: str
    point_a: Vec3
    point_b: Vec3
    clearance: float = Field(ge=0)
    stiffness: float = Field(ge=0)
    direction: Literal["bump", "rebound"] = "bump"
