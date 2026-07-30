"""Common, unit-aware schema primitives."""

from __future__ import annotations

import math
from enum import StrEnum
from typing import Any, ClassVar, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator

SchemaVersion = Literal[1]


class StrictModel(BaseModel):
    """Base for immutable, closed v1 input and output models."""

    model_config: ClassVar[ConfigDict] = ConfigDict(
        extra="forbid", frozen=True, validate_assignment=True, populate_by_name=True
    )


class UnitSystem(StrEnum):
    """External engineering unit systems supported by v1."""

    ENGINEERING = "mm-N-N*mm-kg-deg"
    SI = "m-N-N*m-kg-rad"


class CoordinateSystem(StrEnum):
    """Vehicle coordinate convention used by the solver."""

    VEHICLE = "vehicle"
    WHEEL_LOCAL = "wheel_local"


class Vec3(StrictModel):
    """A finite three-component vector."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @model_validator(mode="before")
    @classmethod
    def _accept_sequence(cls, value: Any) -> Any:
        if isinstance(value, (list, tuple)) and len(value) == 3:
            return {"x": value[0], "y": value[1], "z": value[2]}
        return value

    @field_validator("x", "y", "z")
    @classmethod
    def _finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("vector components must be finite")
        return float(value)

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    def as_array(self) -> Any:
        """Return a NumPy array without making NumPy a schema dependency."""
        import numpy as np

        return np.asarray(self.as_tuple(), dtype=float)

    def mirrored_y(self) -> Vec3:
        return Vec3(x=self.x, y=-self.y, z=self.z)


class Quaternion(StrictModel):
    """Unit quaternion in scalar-first ``(w, x, y, z)`` order."""

    w: float = 1.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @model_validator(mode="before")
    @classmethod
    def _accept_sequence(cls, value: Any) -> Any:
        if isinstance(value, (list, tuple)) and len(value) == 4:
            return {"w": value[0], "x": value[1], "y": value[2], "z": value[3]}
        return value

    @field_validator("w", "x", "y", "z")
    @classmethod
    def _finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("quaternion components must be finite")
        return float(value)

    @model_validator(mode="after")
    def _unit(self) -> Quaternion:
        norm = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        if norm < 1e-12 or abs(norm - 1.0) > 1e-6:
            raise ValueError("quaternion must have unit norm")
        return self

    def as_tuple(self) -> tuple[float, float, float, float]:
        return (self.w, self.x, self.y, self.z)


class Pose(StrictModel):
    """Rigid-body pose represented by translation and unit quaternion."""

    translation: Vec3 = Field(default_factory=Vec3)
    rotation: Quaternion = Field(default_factory=Quaternion)


class Provenance(StrictModel):
    """Reproducibility metadata carried by every result manifest."""

    package_version: str
    schema_version: SchemaVersion = 1
    format_version: str = "1.0"
    model_hash: str | None = None
    case_hash: str | None = None
    created_at: str | None = None
    coordinate_system: CoordinateSystem = CoordinateSystem.VEHICLE
    units: UnitSystem = UnitSystem.ENGINEERING
    solver_settings_hash: str | None = None
    adams_version: str | None = None
    adams_template: str | None = None


class SixVector(StrictModel):
    """Spatial force or displacement vector in ``(x,y,z,rx,ry,rz)`` order."""

    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    mx: float = 0.0
    my: float = 0.0
    mz: float = 0.0

    @field_validator("fx", "fy", "fz", "mx", "my", "mz")
    @classmethod
    def _finite(cls, value: float) -> float:
        if not math.isfinite(value):
            raise ValueError("six-vector components must be finite")
        return float(value)

    def as_tuple(self) -> tuple[float, float, float, float, float, float]:
        return (self.fx, self.fy, self.fz, self.mx, self.my, self.mz)

    def as_array(self) -> Any:
        import numpy as np

        return np.asarray(self.as_tuple(), dtype=float)


def positive(value: float, *, name: str, allow_zero: bool = False) -> float:
    """Validate a finite non-negative or positive scalar."""
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        qualifier = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {qualifier}")
    return float(value)
