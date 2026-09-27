"""
The analytic road surface.

A road here is a **geometry**, not a body: the tire's contact evaluator queries its height
at a point and forms the compression from that.  Nothing about it enters the equations of
motion as a rigid body, which is the property that makes it usable as a wheel pad -- the
pad is a height the wheel is measured against, not something the solver has to carry.

The class lives here rather than beside the vehicle model because it is a property of a
*surface*, shared by the axle and the full vehicle.  It was defined in
``schema/vehicle.py``, which imports the axle model, so the axle could not refer to it
without a cycle; a module that depends on neither direction resolves that.
"""

from __future__ import annotations

import math
from typing import Literal

from pydantic import Field, model_validator

from .common import StrictModel, Vec3
from .dynamic import TimeSignal

__all__ = ["RoadSurfaceSpec"]


class RoadSurfaceSpec(StrictModel):
    """
    Analytic road surface queried by the tire contact evaluator.

    ``wavelength`` and ``bump_length`` are strictly positive and ``corner_scales`` must be
    given, even for a ``plane`` that uses none of them.  That is not a quirk of this
    class: the kernel's own validation requires the same
    (``cpp/src/assembly/registration.cpp``), so a plane road without them is refused at
    model registration.  Stating the requirement here turns that into a construction-time
    error instead of a run-time one.

    ``plane`` must carry zero amplitude; a non-zero one would make the "kind" a lie.
    """

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
