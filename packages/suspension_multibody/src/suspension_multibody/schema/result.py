"""Stable result protocol schemas."""

from __future__ import annotations

import math
from typing import Literal

from pydantic import model_validator

from .common import Pose, Provenance, SchemaVersion, SixVector, StrictModel


class Diagnostic(StrictModel):
    code: str
    severity: Literal["info", "warning", "error"]
    message: str
    state_id: str | None = None


class WheelResponse(StrictModel):
    """Wheel-center response in vehicle coordinates (mm, mm, mm, rad, rad, rad)."""

    x_mm: float
    y_mm: float
    z_mm: float
    rx_rad: float
    ry_rad: float
    rz_rad: float


class CResponse(StrictModel):
    """Physical C response retained alongside scalar C-minus-K metrics."""

    wheel_left: WheelResponse
    wheel_right: WheelResponse
    secant_compliance_left: tuple[tuple[float, ...], ...]
    secant_compliance_right: tuple[tuple[float, ...], ...]

    @model_validator(mode="after")
    def _matrices_are_finite_6x6(self) -> CResponse:
        for matrix in (
            self.secant_compliance_left,
            self.secant_compliance_right,
        ):
            if len(matrix) != 6 or any(len(row) != 6 for row in matrix):
                raise ValueError("secant compliance must be a 6x6 matrix")
            if any(not math.isfinite(value) for row in matrix for value in row):
                raise ValueError("secant compliance must contain finite values")
        return self


class StateResult(StrictModel):
    state_id: str
    mode: Literal["K", "C"]
    drives: dict[str, float] = {}
    external_loads: dict[str, SixVector] = {}
    poses: dict[str, Pose] = {}
    metrics: dict[str, float] = {}
    c_response: CResponse | None = None
    tire_compression: dict[str, float] = {}
    constraint_residual: float = 0.0
    force_residual: float = 0.0
    moment_residual: float = 0.0
    converged: bool = False
    diagnostics: tuple[Diagnostic, ...] = ()


class ComponentLoad(StrictModel):
    state_id: str
    component: str
    endpoint: str
    global_load: SixVector
    local_load: SixVector
    utilization: float | None = None
    over_limit: bool = False


class BushingResult(StrictModel):
    state_id: str
    bushing: str
    deformation: SixVector
    load: SixVector
    strain_energy: float
    stiffness_id: str
    zero_load_pose: Pose


class Manifest(StrictModel):
    schema_version: SchemaVersion = 1
    format_version: str = "1.0"
    run_id: str
    mode: Literal["K", "C"]
    state_count: int
    provenance: Provenance
    tables: tuple[str, ...] = ("states", "component_loads", "bushings", "diagnostics")


class ResultBundle(StrictModel):
    manifest: Manifest
    states: tuple[StateResult, ...] = ()
    component_loads: tuple[ComponentLoad, ...] = ()
    bushings: tuple[BushingResult, ...] = ()
    diagnostics: tuple[Diagnostic, ...] = ()
