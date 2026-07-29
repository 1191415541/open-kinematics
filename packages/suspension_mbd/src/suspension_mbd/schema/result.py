"""Stable result protocol schemas."""

from __future__ import annotations

from typing import Literal

from .common import Pose, Provenance, SchemaVersion, SixVector, StrictModel


class Diagnostic(StrictModel):
    code: str
    severity: Literal["info", "warning", "error"]
    message: str
    state_id: str | None = None


class StateResult(StrictModel):
    state_id: str
    mode: Literal["K", "C"]
    drives: dict[str, float] = {}
    external_loads: dict[str, SixVector] = {}
    poses: dict[str, Pose] = {}
    metrics: dict[str, float] = {}
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
