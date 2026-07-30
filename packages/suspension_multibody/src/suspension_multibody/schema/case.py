"""K/C run and sweep schemas."""

from __future__ import annotations

import math
from typing import Annotated, Literal

from pydantic import Field, model_validator

from .common import CoordinateSystem, SchemaVersion, SixVector, StrictModel, UnitSystem


class RangeSweep(StrictModel):
    start: float
    stop: float
    steps: int = Field(ge=2)

    @model_validator(mode="after")
    def _finite(self) -> RangeSweep:
        if not math.isfinite(self.start) or not math.isfinite(self.stop):
            raise ValueError("sweep bounds must be finite")
        return self

    def values(self) -> tuple[float, ...]:
        import numpy as np

        return tuple(float(v) for v in np.linspace(self.start, self.stop, self.steps))


class ExplicitSweep(StrictModel):
    values: tuple[float, ...]

    @model_validator(mode="after")
    def _valid(self) -> ExplicitSweep:
        if len(self.values) < 2 or any(not math.isfinite(v) for v in self.values):
            raise ValueError("explicit sweep requires at least two finite values")
        return self


Sweep = Annotated[RangeSweep | ExplicitSweep, Field(discriminator=None)]


class DisplacementControl(StrictModel):
    kind: Literal["displacement"] = "displacement"
    target: str
    values: tuple[float, ...] | None = None
    sweep: RangeSweep | None = None

    @model_validator(mode="after")
    def _one_sequence(self) -> DisplacementControl:
        if (self.values is None) == (self.sweep is None):
            raise ValueError(
                "displacement control requires exactly one of values or sweep"
            )
        return self

    def expanded(self) -> tuple[float, ...]:
        return self.values if self.values is not None else self.sweep.values()  # type: ignore[union-attr]


class LoadControl(StrictModel):
    kind: Literal["load"] = "load"
    target: str
    values: tuple[SixVector, ...] | None = None
    sweep: RangeSweep | None = None

    @model_validator(mode="after")
    def _one_sequence(self) -> LoadControl:
        if (self.values is None) == (self.sweep is None):
            raise ValueError("load control requires exactly one of values or sweep")
        return self


class TrimForward(StrictModel):
    kind: Literal["forward"] = "forward"
    spring_preload: dict[str, float] = {}


class TrimInverse(StrictModel):
    kind: Literal["inverse"] = "inverse"
    ride_height: float | None = None
    axle_load: float | None = None
    wheel_load: float | None = None

    @model_validator(mode="after")
    def _one_target(self) -> TrimInverse:
        if (
            sum(
                value is not None
                for value in (self.ride_height, self.axle_load, self.wheel_load)
            )
            != 1
        ):
            raise ValueError("inverse trim requires exactly one target")
        return self


Trim = Annotated[TrimForward | TrimInverse, Field(discriminator="kind")]


class CaseSpec(StrictModel):
    """A mutually exclusive K or C quasi-static analysis run."""

    schema_version: SchemaVersion = 1
    name: str = "case"
    mode: Literal["K", "C"]
    units: UnitSystem = UnitSystem.ENGINEERING
    coordinate_system: CoordinateSystem = CoordinateSystem.VEHICLE
    trim: Trim = Field(default_factory=TrimForward)
    controls: tuple[DisplacementControl | LoadControl, ...] = ()
    external_loads: dict[str, SixVector] = {}
    left_right_mode: Literal["single", "symmetric", "opposite"] = "symmetric"
    worker_count: int = Field(default=1, ge=1)
    checkpoint_path: str | None = None

    @model_validator(mode="after")
    def _control_conflicts(self) -> CaseSpec:
        targets: dict[str, str] = {}
        for control in self.controls:
            kind = control.kind
            previous = targets.get(control.target)
            if previous is not None and previous != kind:
                raise ValueError(f"target {control.target!r} has conflicting controls")
            targets[control.target] = kind
        return self
