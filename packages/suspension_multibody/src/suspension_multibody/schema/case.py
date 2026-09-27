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

#: The three readings a K/C case can ask for.
DriveMode = Literal["kinematics", "force_balance", "pad"]

#: The exhaustive set, for validation error messages and for callers that must enumerate.
DRIVE_MODES: tuple[str, ...] = ("kinematics", "force_balance", "pad")

#: How the legacy ``drive_wheels`` boolean maps onto ``drive_mode``.
#:
#: The boolean predates the three-way field and is used across the repository (29 files),
#: so it keeps working rather than being migrated in one sweep.  The mapping is fixed
#: here, in one place, because two callers inferring it separately is how a silent
#: disagreement starts:
#:
#: * ``True``  -- the wheel centres are the driven coordinates, and the reading balances
#:   the elastic elements against them: ``force_balance``;
#: * ``False`` -- the C reading, where the wheel centre is loaded rather than placed and
#:   the tire is what reacts: ``pad``.
#:
#: ``kinematics`` has no legacy spelling: it did not exist as a distinct reading before,
#: so a caller that wants it says so by name.
DRIVE_WHEELS_TO_MODE: dict[bool, str] = {True: "force_balance", False: "pad"}


def drive_mode_for(drive_wheels: bool | None, default: str = "force_balance") -> str:
    """
    Resolve the legacy boolean and the three-way field into one mode name.

    ``None`` means the caller did not say, which yields ``default``.  This is the single
    place the legacy mapping is applied, so ``drive_wheels=True`` and
    ``drive_mode="force_balance"`` cannot drift apart.
    """
    if drive_wheels is None:
        return default
    return DRIVE_WHEELS_TO_MODE[bool(drive_wheels)]


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
    #: Which subsystems the assembly this case runs carries.  `None` means the
    #: default single-axle set, which is what every existing caller gets.
    #:
    #: It lives on the *case* rather than on the model because it is a property
    #: of the run: a single-axle model may be run with or without its steering
    #: subsystem (requirement 15), an assembly with no steering has no rack
    #: coordinate, and a rig drives it without one rather than zero-filling it.
    #: Making it a case input is what lets a caller ask for that run through the
    #: public entry instead of only through the assembly constructor.
    subsystems: frozenset[str] | None = None
    #: How this run decides the assembly's pose.
    #:
    #: ``kinematics``
    #:     solves the constraint equations alone.  Nothing elastic and no tire enters the
    #:     residual, so the run is pure geometry -- this is the reading a linkage study
    #:     wants, and it is the one whose numbers are frozen.
    #: ``force_balance``
    #:     the default.  The elastic elements the assembly carries (springs, dampers,
    #:     anti-roll bars, bushings, bump stops) balance against the driven targets, so
    #:     the arm mounts and the spring actually react load.  Tires stay out: in a
    #:     wheel-centre-driven reading the wheel is placed, not carried.
    #: ``pad``
    #:     drives a *ground height* instead of the wheel centre, so the tires carry the
    #:     wheel and the sweep is the pad moving under it.
    #:
    #: It lives on the case rather than on the model because the same assembly is read
    #: all three ways; it is a property of the analysis being run.
    drive_mode: DriveMode = "force_balance"
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
