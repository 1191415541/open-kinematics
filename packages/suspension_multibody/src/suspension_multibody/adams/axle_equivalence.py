"""Strict, evidence-backed dynamic axle comparison against Adams."""

from __future__ import annotations

import hashlib
import json
import math
import shutil
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal, cast

import numpy as np
from pydantic import Field, model_validator

from ..axle_dynamics import (
    BODY_STATE_COLUMNS,
    CONSTRAINT_WRENCH_COLUMNS,
    SPRING_OUTPUT_COLUMNS,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleDynamicsResult,
    native_build_metadata,
    run_axle_dynamics,
    write_axle_dynamics_artifact,
)
from ..io import canonical_hash
from ..schema.common import StrictModel
from .axle_channels import axle_history_from_result
from .axle_contract import (
    AxleChannelBindings,
    DynamicAxleManifest,
    load_axle_acceptance_contract,
    load_axle_channel_contract,
    read_dynamic_axle_manifest,
)
from .time_domain import TimeHistory, write_time_history

AXLE_EVIDENCE_CONTRACT = "dynamic-axle-evidence-v1"
AXLE_COMPARISON_CONTRACT = "dynamic-axle-comparison-v1"
_EVIDENCE_FILE = "axle_evidence_bundle.json"
_ADAMS_REQUIRED_SUFFIXES = (".adm", ".cmd", ".msg", ".res")
_NATIVE_REQUIRED_NAMES = ("manifest.json", "arrays.npz")
_DIAGNOSTIC_GATES = (
    "run_completed",
    "solver_internal_gates_passed",
    "energy_gate_passed",
    "time_convergence_passed",
)
TIRE_FORCE_CHANNELS = (
    "left.tire_normal_force",
    "right.tire_normal_force",
    "left.tire_longitudinal_force",
    "right.tire_longitudinal_force",
    "left.tire_lateral_force",
    "right.tire_lateral_force",
)
# 兼容已有调用方；名称本身不再决定实际轮胎模型。
PAC2002_TIRE_FORCE_CHANNELS = TIRE_FORCE_CHANNELS


class AxleContactEvent(StrictModel):
    """One internally localized unilateral-contact transition."""

    tire: str = Field(min_length=1)
    transition: Literal["enter", "exit"]
    time_s: float = Field(ge=0.0)


class AxleInitializationEvidence(StrictModel):
    """Independent consistent initial state plus comparable physical gates."""

    translations_m: dict[str, tuple[float, float, float]]
    rotation_vectors_rad: dict[str, tuple[float, float, float]]
    wheel_loads_n: dict[str, float]
    component_forces_n: dict[str, float]
    component_moments_n_m: dict[str, float]
    constraint_position_max_m: float = Field(ge=0.0)
    constraint_velocity_max_m_per_s: float = Field(ge=0.0)
    state: dict[str, Any]
    state_sha256: str = Field(min_length=64, max_length=64)

    @model_validator(mode="after")
    def _state_hash_matches(self) -> AxleInitializationEvidence:
        if canonical_hash(self.state) != self.state_sha256:
            raise ValueError("initialization state_sha256 does not match state")
        return self


@dataclass(frozen=True)
class AxleEvidenceBundle:
    """Canonical channels and immutable raw evidence from one independent runner."""

    case: str
    producer_id: str
    producer_kind: Literal["msc.adams", "open-kinematics.native"]
    manifest_sha256: str
    history: TimeHistory
    initialization: AxleInitializationEvidence
    contact_events: tuple[AxleContactEvent, ...]
    diagnostics: Mapping[str, object]
    raw_artifacts: Mapping[str, str]

    def as_dict(self) -> dict[str, object]:
        return {
            "contract": AXLE_EVIDENCE_CONTRACT,
            "case": self.case,
            "producer_id": self.producer_id,
            "producer_kind": self.producer_kind,
            "manifest_sha256": self.manifest_sha256,
            "history": self.history.as_dict(),
            "initialization": self.initialization.model_dump(mode="json"),
            "contact_events": [
                event.model_dump(mode="json") for event in self.contact_events
            ],
            "diagnostics": dict(self.diagnostics),
            "raw_artifacts": dict(self.raw_artifacts),
        }


def initialization_evidence_from_result(
    model: AxleDynamicsModel,
    result: AxleDynamicsResult,
    bindings: AxleChannelBindings,
) -> AxleInitializationEvidence:
    """Capture the complete native initial state without sharing it as input."""
    if len(result.times_s) == 0:
        raise ValueError("cannot create initialization evidence from an empty result")
    body_state = {
        name: {
            column: float(value)
            for column, value in zip(
                BODY_STATE_COLUMNS,
                result.body_state(name)[0],
            )
        }
        for name in result.body_names
    }
    constraint_state = {
        name: {
            column: float(value)
            for column, value in zip(
                CONSTRAINT_WRENCH_COLUMNS,
                result.joint_wrench_on_body_b(name)[0],
            )
        }
        for name in result.constraint_names
    }
    spring_state = {
        name: {
            column: float(value)
            for column, value in zip(
                SPRING_OUTPUT_COLUMNS,
                result.spring_state(name)[0],
            )
        }
        for name in result.spring_names
    }
    state: dict[str, Any] = {
        "time_s": float(result.times_s[0]),
        "bodies": body_state,
        "constraint_wrench_on_body_b": constraint_state,
        "springs": spring_state,
        "bushings": {
            name: [float(value) for value in result.bushing_state(name)[0]]
            for name in result.bushing_names
        },
        "anti_roll_bars": {
            name: [float(value) for value in result.anti_roll_bar_state(name)[0]]
            for name in result.anti_roll_bar_names
        },
        "tires": {
            name: [float(value) for value in result.tire_state(name)[0]]
            for name in result.tire_names
        },
        "energy": [float(value) for value in result.energy[0]],
    }
    translations = {
        name: tuple(
            float(value)
            for value in result.body_state(name)[0, :3]
        )
        for name in result.body_names
    }
    rotations = {
        name: tuple(
            float(value)
            for value in _quaternion_log(
                result.body_state(name)[0, 3:7]
            )
        )
        for name in result.body_names
    }
    wheel_loads = {
        "left": float(result.tire_state(bindings.left_tire)[0, 4]),
        "right": float(result.tire_state(bindings.right_tire)[0, 4]),
    }
    component_forces: dict[str, float] = {}
    component_moments: dict[str, float] = {}
    for name in result.spring_names:
        for column, value in zip(
            SPRING_OUTPUT_COLUMNS[2:],
            result.spring_state(name)[0, 2:],
        ):
            component_forces[f"spring:{name}:{column}"] = float(value)
    for name in result.constraint_names:
        values = result.joint_wrench_on_body_b(name)[0]
        for axis, value in zip("xyz", values[:3]):
            component_forces[f"constraint:{name}:force_{axis}"] = float(value)
        for axis, value in zip("xyz", values[3:]):
            component_moments[f"constraint:{name}:moment_{axis}"] = float(value)
    for name in result.bushing_names:
        values = result.bushing_state(name)[0, 6:12]
        for axis, value in zip("xyz", values[:3]):
            component_forces[f"bushing:{name}:force_{axis}"] = float(value)
        for axis, value in zip("xyz", values[3:]):
            component_moments[f"bushing:{name}:moment_{axis}"] = float(value)
    for name in result.anti_roll_bar_names:
        component_moments[f"anti_roll_bar:{name}:axis_torque"] = float(
            result.anti_roll_bar_state(name)[0, 2]
        )
    return AxleInitializationEvidence(
        translations_m=translations,
        rotation_vectors_rad=rotations,
        wheel_loads_n=wheel_loads,
        component_forces_n=component_forces,
        component_moments_n_m=component_moments,
        constraint_position_max_m=float(result.diagnostics.position_residual[0]),
        constraint_velocity_max_m_per_s=float(
            result.diagnostics.velocity_residual[0]
        ),
        state=state,
        state_sha256=canonical_hash(state),
    )


def write_axle_evidence_bundle(
    *,
    output_dir: str | Path,
    manifest: DynamicAxleManifest,
    producer_id: str,
    producer_kind: Literal["msc.adams", "open-kinematics.native"],
    history: TimeHistory,
    initialization: AxleInitializationEvidence,
    contact_events: Sequence[AxleContactEvent],
    diagnostics: Mapping[str, object],
    raw_artifacts: Sequence[str | Path],
) -> Path:
    """Write one independently produced, raw-file-backed evidence bundle."""
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    if not producer_id:
        raise ValueError("producer_id cannot be empty")
    _validate_history(history, manifest)
    _validate_diagnostics(diagnostics)
    hashes = _hash_declared_artifacts(destination, raw_artifacts)
    _validate_raw_contract(producer_kind, hashes)
    bundle = AxleEvidenceBundle(
        case=manifest.case.name,
        producer_id=producer_id,
        producer_kind=producer_kind,
        manifest_sha256=manifest.sha256,
        history=history,
        initialization=initialization,
        contact_events=tuple(contact_events),
        diagnostics=dict(diagnostics),
        raw_artifacts=hashes,
    )
    path = destination / _EVIDENCE_FILE
    path.write_text(
        json.dumps(bundle.as_dict(), indent=2, sort_keys=False),
        encoding="utf-8",
    )
    return path


def read_axle_evidence_bundle(
    path: str | Path,
    *,
    manifest: DynamicAxleManifest | None = None,
) -> AxleEvidenceBundle:
    """Read evidence and verify its contract, state hash, and every raw file."""
    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("dynamic axle evidence root must be an object")
    if payload.get("contract") != AXLE_EVIDENCE_CONTRACT:
        raise ValueError("unsupported dynamic axle evidence contract")
    case = payload.get("case")
    producer_id = payload.get("producer_id")
    producer_kind = payload.get("producer_kind")
    manifest_sha256 = payload.get("manifest_sha256")
    if not isinstance(case, str) or not case:
        raise ValueError("dynamic axle evidence case must be non-empty")
    if not isinstance(producer_id, str) or not producer_id:
        raise ValueError("dynamic axle evidence producer_id must be non-empty")
    if producer_kind not in {"msc.adams", "open-kinematics.native"}:
        raise ValueError("dynamic axle evidence has invalid producer_kind")
    if not isinstance(manifest_sha256, str) or len(manifest_sha256) != 64:
        raise ValueError("dynamic axle evidence has invalid manifest_sha256")
    history_payload = payload.get("history")
    initialization_payload = payload.get("initialization")
    events_payload = payload.get("contact_events")
    diagnostics = payload.get("diagnostics")
    raw = payload.get("raw_artifacts")
    if not isinstance(history_payload, Mapping):
        raise ValueError("dynamic axle evidence history must be an object")
    if not isinstance(initialization_payload, Mapping):
        raise ValueError("dynamic axle evidence initialization must be an object")
    if not isinstance(events_payload, list):
        raise ValueError("dynamic axle evidence contact_events must be a list")
    if not isinstance(diagnostics, Mapping):
        raise ValueError("dynamic axle evidence diagnostics must be an object")
    if not isinstance(raw, Mapping) or not raw:
        raise ValueError("dynamic axle evidence raw_artifacts are required")
    history = TimeHistory.from_mapping(cast(Mapping[str, object], history_payload))
    initialization = AxleInitializationEvidence.model_validate(
        initialization_payload
    )
    events = tuple(AxleContactEvent.model_validate(item) for item in events_payload)
    raw_artifacts = {str(name): str(value) for name, value in raw.items()}
    _validate_diagnostics(diagnostics)
    _validate_raw_contract(cast(Any, producer_kind), raw_artifacts)
    for relative_path, expected_hash in raw_artifacts.items():
        artifact = source.parent / relative_path
        if not artifact.is_file():
            raise ValueError(f"dynamic axle raw artifact is missing: {relative_path}")
        if _file_hash(artifact) != expected_hash:
            raise ValueError(
                f"dynamic axle raw artifact hash changed: {relative_path}"
            )
    bundle = AxleEvidenceBundle(
        case=case,
        producer_id=producer_id,
        producer_kind=cast(Any, producer_kind),
        manifest_sha256=manifest_sha256,
        history=history,
        initialization=initialization,
        contact_events=events,
        diagnostics={str(name): value for name, value in diagnostics.items()},
        raw_artifacts=raw_artifacts,
    )
    if manifest is not None:
        if bundle.manifest_sha256 != manifest.sha256:
            raise ValueError("evidence manifest hash does not match shared manifest")
        if bundle.case != manifest.case.name:
            raise ValueError("evidence case does not match shared manifest")
        _validate_history(bundle.history, manifest)
    return bundle


def compare_axle_evidence(
    *,
    manifest_path: str | Path,
    adams_evidence_path: str | Path,
    native_evidence_path: str | Path,
    output_path: str | Path | None = None,
) -> dict[str, object]:
    """Run initialization, transient, harmonic, event, and diagnostic gates."""
    manifest = read_dynamic_axle_manifest(manifest_path)
    adams_path = Path(adams_evidence_path).resolve()
    native_path = Path(native_evidence_path).resolve()
    if adams_path.parent == native_path.parent:
        raise ValueError("Adams and native evidence must use different directories")
    adams = read_axle_evidence_bundle(adams_path, manifest=manifest)
    native = read_axle_evidence_bundle(native_path, manifest=manifest)
    if adams.producer_kind != "msc.adams":
        raise ValueError("reference evidence must be produced by msc.adams")
    if native.producer_kind != "open-kinematics.native":
        raise ValueError("candidate evidence must be produced by native solver")
    if adams.producer_id == native.producer_id:
        raise ValueError("independent runners require different producer_id values")

    acceptance = cast(Mapping[str, object], manifest.payload["acceptance"])
    initialization = _compare_initialization(
        adams.initialization,
        native.initialization,
        acceptance,
    )
    report: dict[str, object] = {
        "contract": AXLE_COMPARISON_CONTRACT,
        "case": manifest.case.name,
        "manifest_sha256": manifest.sha256,
        "reference_producer_id": adams.producer_id,
        "candidate_producer_id": native.producer_id,
        "initialization": initialization,
        "transient_comparison_performed": bool(initialization["passed"]),
    }
    if not initialization["passed"]:
        report.update(
            {
                "status": "BLOCKED",
                "passed": False,
                "failure_attribution": ["initialization_or_parameter_mismatch"],
            }
        )
        return _write_report(report, output_path)

    transient = compare_strict_axle_histories(
        adams.history,
        native.history,
        acceptance=acceptance,
        case_name=manifest.case.name,
        harmonic_frequency_hz=_harmonic_frequency(manifest),
        reference_events=adams.contact_events,
        candidate_events=native.contact_events,
    )
    diagnostics = {
        "reference": _diagnostic_gate(adams.diagnostics),
        "candidate": _diagnostic_gate(native.diagnostics),
    }
    diagnostics_passed = bool(
        diagnostics["reference"]["passed"]
        and diagnostics["candidate"]["passed"]
    )
    passed = bool(transient["passed"] and diagnostics_passed)
    report.update(
        {
            "status": "PASS" if passed else "FAIL",
            "transient": transient,
            "diagnostics": diagnostics,
            "passed": passed,
            "failure_attribution": _failure_attribution(
                transient,
                diagnostics,
            ),
        }
    )
    return _write_report(report, output_path)


def run_native_axle_manifest(
    manifest_path: str | Path,
    output_dir: str | Path,
    *,
    producer_id: str = "open-kinematics.native",
) -> Path:
    """Run one manifest independently and retain convergence evidence."""
    manifest_source = Path(manifest_path)
    manifest = read_dynamic_axle_manifest(manifest_source)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    copied_manifest = destination / "dynamic_axle_manifest.json"
    shutil.copy2(manifest_source, copied_manifest)

    started = time.perf_counter()
    result = run_axle_dynamics(manifest.model, manifest.case)
    elapsed_s = time.perf_counter() - started
    refined_case = _refined_case(manifest.case)
    refined_started = time.perf_counter()
    refined_result = run_axle_dynamics(manifest.model, refined_case)
    refined_elapsed_s = time.perf_counter() - refined_started

    primary_manifest = write_axle_dynamics_artifact(
        result,
        manifest.model,
        manifest.case,
        destination / "native_result",
    )
    refined_manifest = write_axle_dynamics_artifact(
        refined_result,
        manifest.model,
        refined_case,
        destination / "native_refined_result",
    )
    history = axle_history_from_result(
        manifest.model,
        result,
        manifest.bindings,
        case=manifest.case,
    )
    refined_history = axle_history_from_result(
        manifest.model,
        refined_result,
        manifest.bindings,
        case=refined_case,
    )
    history_path = write_time_history(history, destination / "native_history.json")
    refined_history_path = write_time_history(
        refined_history, destination / "native_refined_history.json"
    )
    convergence = _time_convergence_gate(
        history,
        refined_history,
        cast(Mapping[str, object], manifest.payload["acceptance"]),
        primary_result=result,
        refined_result=refined_result,
        primary_case=manifest.case,
        refined_case=refined_case,
    )
    solver_gate = _native_solver_gate(
        result,
        cast(Mapping[str, object], manifest.payload["acceptance"]),
    )
    energy_gate = _native_energy_gate(
        result,
        manifest.case.name,
        cast(Mapping[str, object], manifest.payload["acceptance"]),
    )
    timing = {
        "producer_id": producer_id,
        "manifest_sha256": manifest.sha256,
        "primary_wall_time_s": elapsed_s,
        "refined_wall_time_s": refined_elapsed_s,
        "primary_realtime_ratio": elapsed_s
        / (manifest.case.times_s[-1] - manifest.case.times_s[0]),
        "native_build": native_build_metadata(),
        "timing_boundary": {
            "include_model_build": False,
            "include_static_trim": (
                manifest.case.solver.initialization_mode == "static_equilibrium"
            ),
            "include_result_serialization": False,
        },
    }
    timing_path = destination / "native_timing.json"
    timing_path.write_text(
        json.dumps(timing, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    diagnostics: dict[str, object] = {
        "run_completed": True,
        "solver_internal_gates_passed": bool(solver_gate["passed"]),
        "energy_gate_passed": bool(energy_gate["passed"]),
        "time_convergence_passed": bool(convergence["passed"]),
        "solver_internal": solver_gate,
        "energy": energy_gate,
        "time_convergence": convergence,
        "timing": timing,
    }
    events = tuple(
        AxleContactEvent(
            tire=event.tire,
            transition=event.transition,
            time_s=event.time_s,
        )
        for event in result.contact_events
    )
    return write_axle_evidence_bundle(
        output_dir=destination,
        manifest=manifest,
        producer_id=producer_id,
        producer_kind="open-kinematics.native",
        history=history,
        initialization=initialization_evidence_from_result(
            manifest.model,
            result,
            manifest.bindings,
        ),
        contact_events=events,
        diagnostics=diagnostics,
        raw_artifacts=(
            copied_manifest,
            primary_manifest,
            primary_manifest.parent / "arrays.npz",
            refined_manifest,
            refined_manifest.parent / "arrays.npz",
            history_path,
            refined_history_path,
            timing_path,
        ),
    )


def compare_strict_axle_histories(
    reference: TimeHistory,
    candidate: TimeHistory,
    *,
    acceptance: Mapping[str, object] | None = None,
    case_name: str,
    harmonic_frequency_hz: float | None = None,
    include_harmonic: bool = True,
    reference_events: Sequence[AxleContactEvent] = (),
    candidate_events: Sequence[AxleContactEvent] = (),
) -> dict[str, object]:
    """Compare on the exact common grid; interpolation is never performed."""
    contract = acceptance or load_axle_acceptance_contract()
    core = tuple(cast(Sequence[str], contract["core_channels"]))
    _validate_history_pair(reference, candidate, core)
    time = np.asarray(reference.time, dtype=float)
    channels = {
        name: _channel_metrics(
            time,
            np.asarray(reference.channels[name], dtype=float),
            np.asarray(candidate.channels[name], dtype=float),
            _unit_category(cast(Mapping[str, str], reference.units or {})[name]),
            contract,
            derived_from_public_balance=name.startswith("fixture."),
        )
        for name in core
    }
    harmonic: dict[str, object] | None = None
    if case_name == "road_sine" and include_harmonic:
        if harmonic_frequency_hz is None or harmonic_frequency_hz <= 0.0:
            raise ValueError("road_sine comparison requires harmonic_frequency_hz")
        harmonic = _harmonic_comparison(
            reference,
            candidate,
            core,
            contract,
            harmonic_frequency_hz,
        )
    events: dict[str, object] | None = None
    if case_name == "tire_liftoff_and_recontact":
        events = _event_comparison(
            reference,
            candidate,
            reference_events,
            candidate_events,
            core,
            contract,
        )
    passed = bool(
        all(cast(bool, value["passed"]) for value in channels.values())
        and (harmonic is None or bool(harmonic["passed"]))
        and (events is None or bool(events["passed"]))
    )
    return {
        "time_alignment": "common_manifest_grid",
        "interpolation": "none",
        "phase_metric": "cross_correlation_lag_ms",
        "sample_count": len(time),
        "channels": channels,
        "harmonic": harmonic,
        "contact_events": events,
        "passed": passed,
    }


def compare_tire_force_histories(
    reference: TimeHistory,
    candidate: TimeHistory,
    *,
    tire_model: Literal["pac2002", "native_brush"],
    acceptance: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """按显式轮胎模型在公共时间网格上比较六个轮胎力通道。."""
    if tire_model not in {"pac2002", "native_brush"}:
        raise ValueError(f"unsupported tire comparison model: {tire_model!r}")
    names = TIRE_FORCE_CHANNELS
    for history, label in ((reference, "reference"), (candidate, "candidate")):
        missing = [name for name in names if name not in history.channels]
        if missing:
            raise ValueError(f"{label} history is missing tire channels: {missing}")
        if history.units is None or any(name not in history.units for name in names):
            raise ValueError(f"{label} history lacks explicit tire channel units")
    reference_subset = TimeHistory(
        time=reference.time,
        channels={name: reference.channels[name] for name in names},
        units={name: reference.units[name] for name in names},
    )
    candidate_subset = TimeHistory(
        time=candidate.time,
        channels={name: candidate.channels[name] for name in names},
        units={name: candidate.units[name] for name in names},
    )
    contract = acceptance or load_axle_acceptance_contract()
    _validate_history_pair(reference_subset, candidate_subset, names)
    time = np.asarray(reference_subset.time, dtype=float)
    channels = {
        name: _channel_metrics(
            time,
            np.asarray(reference_subset.channels[name], dtype=float),
            np.asarray(candidate_subset.channels[name], dtype=float),
            _unit_category(reference_subset.units[name]),
            contract,
        )
        for name in names
    }
    return {
        "contract": "tire-force-comparison-v2",
        "tire_model": tire_model,
        "channels_expected": list(names),
        "time_alignment": "common_manifest_grid",
        "interpolation": "none",
        "sample_count": len(time),
        "channels": channels,
        "passed": bool(all(value["passed"] for value in channels.values())),
    }


def compare_pac2002_tire_force_histories(
    reference: TimeHistory,
    candidate: TimeHistory,
    *,
    acceptance: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """兼容入口：比较明确标记为 PAC2002 的六个轮胎力通道。."""
    return compare_tire_force_histories(
        reference,
        candidate,
        tire_model="pac2002",
        acceptance=acceptance,
    )


def compare_native_brush_tire_force_histories(
    reference: TimeHistory,
    candidate: TimeHistory,
    *,
    acceptance: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """比较明确标记为生成式 native brush 的六个轮胎力通道。."""
    return compare_tire_force_histories(
        reference,
        candidate,
        tire_model="native_brush",
        acceptance=acceptance,
    )


def audit_axle_time_convergence(
    primary: TimeHistory,
    refined: TimeHistory,
    *,
    acceptance: Mapping[str, object] | None = None,
) -> dict[str, object]:
    """Measure one solver's h versus h/2 refinement on the common grid."""
    contract = acceptance or load_axle_acceptance_contract()
    return _time_convergence_gate(primary, refined, contract)


def _refined_case(case: AxleDynamicsCase) -> AxleDynamicsCase:
    payload = case.model_dump(mode="json")
    solver = dict(cast(Mapping[str, object], payload["solver"]))
    internal = 0.5 * float(solver["internal_step_s"])
    solver.update(
        {
            "adaptive_step": False,
            "internal_step_s": internal,
            "maximum_step_s": internal,
            "minimum_step_s": min(float(solver["minimum_step_s"]), internal),
        }
    )
    payload["solver"] = solver
    return AxleDynamicsCase.model_validate(payload)


def _native_solver_gate(
    result: AxleDynamicsResult,
    acceptance: Mapping[str, object],
) -> dict[str, object]:
    gate = cast(Mapping[str, object], acceptance["solver_internal_gates"])
    quaternion_norm_error = float(
        np.max(
            np.abs(
                np.linalg.norm(result.states[:, :, 3:7], axis=2) - 1.0
            )
        )
    )
    checks = {
        "accepted": bool(np.all(result.diagnostics.accepted)),
        "dynamics_residual": float(
            np.max(result.diagnostics.dynamics_residual)
        )
        <= float(gate["normalized_dynamics_residual"]),
        "constraint_position": float(
            np.max(result.diagnostics.position_residual)
        )
        <= float(gate["constraint_position_m"]),
        "constraint_velocity": float(
            np.max(result.diagnostics.velocity_residual)
        )
        <= float(gate["constraint_velocity_m_per_s"]),
        "quaternion_norm": quaternion_norm_error
        <= float(gate["quaternion_norm_error"]),
    }
    return {
        "maximum_dynamics_residual": float(
            np.max(result.diagnostics.dynamics_residual)
        ),
        "maximum_constraint_position_m": float(
            np.max(result.diagnostics.position_residual)
        ),
        "maximum_constraint_velocity_m_per_s": float(
            np.max(result.diagnostics.velocity_residual)
        ),
        "maximum_quaternion_norm_error": quaternion_norm_error,
        "checks": checks,
        "passed": bool(all(checks.values())),
    }


def _native_energy_gate(
    result: AxleDynamicsResult,
    case_name: str,
    acceptance: Mapping[str, object],
) -> dict[str, object]:
    gate = cast(Mapping[str, object], acceptance["solver_internal_gates"])
    cumulative_work = np.cumsum(result.energy[:, 11])
    cumulative_residual = np.cumsum(result.energy[:, 3])
    elastic = np.sum(result.energy[:, 15:21], axis=1)
    floor = float(
        cast(Mapping[str, object], gate["energy_normalization"])[
            "energy_floor_j"
        ]
    )
    normalization = max(
        abs(float(result.energy[0, 2])),
        float(np.max(np.abs(cumulative_work))),
        float(np.max(np.abs(elastic))),
        floor,
    )
    closure = float(np.max(np.abs(cumulative_residual))) / normalization
    contact_case = case_name == "tire_liftoff_and_recontact"
    limit = float(
        gate[
            "contact_event_case_energy_closure_relative"
            if contact_case
            else "smooth_case_energy_closure_relative"
        ]
    )
    nonnegative_dissipation = bool(
        np.all(result.energy[:, 7:10] >= -1e-12)
    )
    status = bool(np.all(result.energy[:, 13] == 1.0))
    return {
        "class": "contact_event" if contact_case else "smooth",
        "maximum_relative_closure_error": closure,
        "limit": limit,
        "normalization_j": normalization,
        "nonnegative_physical_dissipation": nonnegative_dissipation,
        "energy_status": status,
        "passed": bool(
            closure <= limit and nonnegative_dissipation and status
        ),
    }


def _time_convergence_gate(
    primary: TimeHistory,
    refined: TimeHistory,
    acceptance: Mapping[str, object],
    *,
    primary_result: AxleDynamicsResult | None = None,
    refined_result: AxleDynamicsResult | None = None,
    primary_case: AxleDynamicsCase | None = None,
    refined_case: AxleDynamicsCase | None = None,
) -> dict[str, object]:
    _validate_history_pair(
        refined,
        primary,
        tuple(cast(Sequence[str], acceptance["core_channels"])),
    )
    channel_contract = load_axle_channel_contract()
    floors = cast(Mapping[str, object], acceptance["amplitude_floors"])
    state_errors: dict[str, float] = {}
    load_errors: dict[str, float] = {}
    for name, reference_values in refined.channels.items():
        reference = np.asarray(reference_values, dtype=float)
        candidate = np.asarray(primary.channels[name], dtype=float)
        category = _unit_category(
            str(channel_contract["channels"][name]["unit"])
        )
        error = _rms(candidate-reference) / max(
            float(np.max(reference)-np.min(reference)),
            float(floors[category]),
        )
        target = (
            load_errors
            if category in {"force_n", "moment_n_m"}
            else state_errors
        )
        target[name] = error
    convergence = cast(
        Mapping[str, object],
        cast(Mapping[str, object], acceptance["solver_internal_gates"])[
            "time_convergence"
        ],
    )
    maximum_state = max(state_errors.values(), default=0.0)
    maximum_load = max(load_errors.values(), default=0.0)
    step_evidence: dict[str, object] | None = None
    if (
        primary_result is not None
        and refined_result is not None
        and primary_case is not None
        and refined_case is not None
    ):
        primary_steps = _accepted_step_evidence(primary_result, primary_case)
        refined_steps = _accepted_step_evidence(refined_result, refined_case)
        step_evidence = {
            "primary": primary_steps,
            "refined": refined_steps,
            "target_ratio": (
                refined_case.solver.internal_step_s
                / primary_case.solver.internal_step_s
            ),
            "passed": bool(primary_steps["passed"] and refined_steps["passed"]),
        }
    step_passed = True if step_evidence is None else bool(step_evidence["passed"])
    return {
        "primary_step_s": (
            None
            if primary_case is None
            else primary_case.solver.internal_step_s
        ),
        "refined_step_ratio": (
            None
            if primary_case is None or refined_case is None
            else refined_case.solver.internal_step_s
            / primary_case.solver.internal_step_s
        ),
        "step_evidence": step_evidence,
        "maximum_state_nrmse": maximum_state,
        "maximum_load_nrmse": maximum_load,
        "state_limit": float(convergence["state_nrmse_h_vs_h2_max"]),
        "load_limit": float(convergence["load_nrmse_h_vs_h2_max"]),
        "state_channels": state_errors,
        "load_channels": load_errors,
        "passed": bool(
            maximum_state
            <= float(convergence["state_nrmse_h_vs_h2_max"])
            and maximum_load
            <= float(convergence["load_nrmse_h_vs_h2_max"])
            and step_passed
        ),
    }


def _accepted_step_evidence(
    result: AxleDynamicsResult,
    case: AxleDynamicsCase,
) -> dict[str, object]:
    """核验求解器实际接受的步长，而不是只核验配置值。."""
    diagnostics = result.diagnostics
    accepted = np.asarray(diagnostics.accepted, dtype=float) > 0.5
    arrays = {
        "minimum": np.asarray(diagnostics.minimum_accepted_step_s, dtype=float),
        "maximum": np.asarray(diagnostics.maximum_accepted_step_s, dtype=float),
        "last": np.asarray(diagnostics.last_accepted_step_s, dtype=float),
    }
    finite = bool(
        all(np.all(np.isfinite(values)) for values in arrays.values())
    )
    observed = np.concatenate(
        tuple(values[accepted & (values > 0.0)] for values in arrays.values())
    )
    target = float(case.solver.internal_step_s)
    tolerance = max(1.0e-12, abs(target) * 1.0e-8)
    checks = {
        "all_outputs_accepted": bool(np.all(accepted)),
        "finite_step_diagnostics": finite,
        "within_configured_maximum": bool(
            finite
            and observed.size > 0
            and float(np.max(observed))
            <= float(case.solver.maximum_step_s) + tolerance
        ),
        "fixed_step_matches_target": bool(
            not case.solver.adaptive_step
            and finite
            and observed.size > 0
            and float(np.max(np.abs(observed - target))) <= tolerance
        )
        if not case.solver.adaptive_step
        else True,
    }
    return {
        "adaptive_step": case.solver.adaptive_step,
        "requested_internal_step_s": target,
        "configured_maximum_step_s": case.solver.maximum_step_s,
        "actual_minimum_step_s": (
            float(np.min(observed)) if observed.size else None
        ),
        "actual_maximum_step_s": (
            float(np.max(observed)) if observed.size else None
        ),
        "checks": checks,
        "passed": bool(all(checks.values())),
    }


def _validate_history(history: TimeHistory, manifest: DynamicAxleManifest) -> None:
    contract = load_axle_channel_contract()
    names = tuple(contract["channels"])
    if set(history.channels) != set(names):
        raise ValueError("evidence channels do not match frozen channel set")
    expected_units = {
        name: str(contract["channels"][name]["unit"])
        for name in names
    }
    if history.units != expected_units:
        raise ValueError("evidence channel units do not match frozen SI units")
    expected_time = np.asarray(manifest.case.times_s, dtype=float)
    actual_time = np.asarray(history.time, dtype=float)
    if len(actual_time) != len(expected_time) or not np.allclose(
        actual_time,
        expected_time,
        rtol=0.0,
        atol=1e-12,
    ):
        raise ValueError("evidence time grid differs from common manifest grid")


def _validate_history_pair(
    reference: TimeHistory,
    candidate: TimeHistory,
    core: tuple[str, ...],
) -> None:
    if set(reference.channels) != set(core):
        raise ValueError("reference history does not contain every core channel")
    if set(candidate.channels) != set(core):
        raise ValueError("candidate history does not contain every core channel")
    if reference.units != candidate.units:
        raise ValueError("reference and candidate channel units differ")
    if reference.units is None or set(reference.units) != set(core):
        raise ValueError("strict axle histories require explicit units")
    reference_time = np.asarray(reference.time, dtype=float)
    candidate_time = np.asarray(candidate.time, dtype=float)
    if len(reference_time) != len(candidate_time) or not np.allclose(
        reference_time,
        candidate_time,
        rtol=0.0,
        atol=1e-12,
    ):
        raise ValueError(
            "strict axle comparison requires the identical time grid; "
            "interpolation is forbidden"
        )


def _channel_metrics(
    time: np.ndarray,
    reference: np.ndarray,
    candidate: np.ndarray,
    category: str,
    acceptance: Mapping[str, object],
    *,
    derived_from_public_balance: bool = False,
) -> dict[str, float | bool | int]:
    comparison = cast(Mapping[str, object], acceptance["comparison"])
    nrmse_contract = cast(Mapping[str, object], comparison["nrmse"])
    peak_contract = cast(Mapping[str, object], comparison["peak"])
    floors = cast(Mapping[str, object], acceptance["amplitude_floors"])
    absolute_limits = cast(
        Mapping[str, object],
        acceptance["absolute_error_limits"],
    )
    floor = float(floors[category])
    absolute_limit = float(absolute_limits[category])
    error = candidate - reference
    reference_range = float(np.max(reference) - np.min(reference))
    nrmse = _rms(error) / max(reference_range, floor)
    edge_exclusion_key = (
        "derived_channel_edge_exclusion_samples"
        if derived_from_public_balance
        else "edge_exclusion_samples"
    )
    edge_exclusion = int(peak_contract.get(edge_exclusion_key, 0))
    if edge_exclusion < 0:
        raise ValueError("peak edge exclusion is outside the history")
    if edge_exclusion >= len(time):
        # 短单元测试或短诊断历史没有足够的公共点，保留完整历史而不是
        # 让比较器因派生通道的窗口配置本身失败。
        edge_exclusion = 0
    peak_slice = slice(0, len(time) - edge_exclusion or None)
    reference_index = int(np.argmax(np.abs(reference[peak_slice])))
    candidate_index = int(np.argmax(np.abs(candidate[peak_slice])))
    reference_peak = float(reference[reference_index])
    candidate_peak = float(candidate[candidate_index])
    peak_relative = abs(candidate_peak - reference_peak) / max(
        abs(reference_peak),
        floor,
    )
    peak_timing_applicable = abs(reference_peak) >= floor
    peak_timing = (
        abs(float(time[candidate_index]) - float(time[reference_index]))
        if peak_timing_applicable
        else 0.0
    )
    phase_lag_ms = _phase_lag_ms(time, reference, candidate)
    maximum_absolute = float(np.max(np.abs(error)))
    checks = {
        "nrmse": nrmse <= float(nrmse_contract["maximum"]),
        "peak_relative": peak_relative <= float(peak_contract["maximum"]),
        "peak_timing": (
            not peak_timing_applicable
            or peak_timing <= float(peak_contract["timing_max_s"])
        ),
        "maximum_absolute": maximum_absolute <= absolute_limit,
    }
    return {
        "nrmse": nrmse,
        "reference_peak": reference_peak,
        "candidate_peak": candidate_peak,
        "peak_relative_error": peak_relative,
        "peak_timing_applicable": peak_timing_applicable,
        "peak_timing_error_s": peak_timing,
        "peak_window_edge_exclusion_samples": edge_exclusion,
        "phase_lag_ms": phase_lag_ms,
        "phase_absolute_error_ms": abs(phase_lag_ms),
        "maximum_absolute_error": maximum_absolute,
        "amplitude_floor": floor,
        "absolute_error_limit": absolute_limit,
        **{f"{name}_passed": bool(value) for name, value in checks.items()},
        "passed": bool(all(checks.values())),
    }


def _harmonic_comparison(
    reference: TimeHistory,
    candidate: TimeHistory,
    core: tuple[str, ...],
    acceptance: Mapping[str, object],
    frequency_hz: float,
) -> dict[str, object]:
    comparison = cast(Mapping[str, object], acceptance["comparison"])
    harmonic = cast(Mapping[str, object], comparison["harmonic"])
    settle_s = min(10.0 / frequency_hz, 2.0)
    analysis_duration = float(harmonic["analysis_cycles"]) / frequency_hz
    end_s = settle_s + analysis_duration
    time = np.asarray(reference.time, dtype=float)
    mask = (time >= settle_s - 1e-12) & (time <= end_s + 1e-12)
    if int(np.count_nonzero(mask)) < 4 or time[-1] < end_s - 1e-12:
        raise ValueError(
            "road_sine history is too short for frozen settle and analysis windows"
        )
    channel_contract = load_axle_channel_contract()
    floors = cast(Mapping[str, object], acceptance["amplitude_floors"])
    channel_reports: dict[str, object] = {}
    for name in core:
        unit = str(channel_contract["channels"][name]["unit"])
        floor = float(floors[_unit_category(unit)])
        reference_amplitude, reference_phase = _fit_harmonic(
            time[mask],
            np.asarray(reference.channels[name], dtype=float)[mask],
            frequency_hz,
        )
        candidate_amplitude, candidate_phase = _fit_harmonic(
            time[mask],
            np.asarray(candidate.channels[name], dtype=float)[mask],
            frequency_hz,
        )
        amplitude_relative = abs(
            candidate_amplitude - reference_amplitude
        ) / max(reference_amplitude, floor)
        phase_applicable = reference_amplitude >= floor
        phase_error = (
            abs(_wrap_degrees(candidate_phase - reference_phase))
            if phase_applicable
            else 0.0
        )
        amplitude_passed = amplitude_relative <= float(
            harmonic["amplitude_relative_max"]
        )
        phase_passed = (
            not phase_applicable
            or phase_error <= float(harmonic["phase_absolute_max_deg"])
        )
        channel_reports[name] = {
            "reference_amplitude": reference_amplitude,
            "candidate_amplitude": candidate_amplitude,
            "amplitude_relative_error": amplitude_relative,
            "reference_phase_deg": reference_phase,
            "candidate_phase_deg": candidate_phase,
            "phase_absolute_error_deg": phase_error,
            "phase_applicable": phase_applicable,
            "amplitude_passed": amplitude_passed,
            "phase_passed": phase_passed,
            "passed": bool(amplitude_passed and phase_passed),
        }
    return {
        "frequency_hz": frequency_hz,
        "settle_end_s": settle_s,
        "analysis_end_s": end_s,
        "channels": channel_reports,
        "passed": bool(
            all(cast(bool, item["passed"]) for item in channel_reports.values())
        ),
    }


def _event_comparison(
    reference: TimeHistory,
    candidate: TimeHistory,
    reference_events: Sequence[AxleContactEvent],
    candidate_events: Sequence[AxleContactEvent],
    core: tuple[str, ...],
    acceptance: Mapping[str, object],
) -> dict[str, object]:
    comparison = cast(Mapping[str, object], acceptance["comparison"])
    event_contract = cast(Mapping[str, object], comparison["event"])
    if not reference_events or not candidate_events:
        raise ValueError(
            "contact-event case requires internal enter/exit event evidence"
        )
    if len(reference_events) != len(candidate_events):
        return {
            "passed": False,
            "reason": "contact event counts differ",
            "reference_count": len(reference_events),
            "candidate_count": len(candidate_events),
        }
    timing_limit = float(event_contract["contact_enter_exit_time_max_s"])
    window_s = float(event_contract["compare_pre_post_windows_s"])
    time = np.asarray(reference.time, dtype=float)
    pairs: list[dict[str, object]] = []
    all_passed = True
    for reference_event, candidate_event in zip(
        reference_events,
        candidate_events,
    ):
        identity_passed = bool(
            reference_event.tire == candidate_event.tire
            and reference_event.transition == candidate_event.transition
        )
        timing_error = abs(reference_event.time_s - candidate_event.time_s)
        timing_passed = timing_error <= timing_limit
        mask = (
            (time >= reference_event.time_s - window_s - 1e-12)
            & (time <= reference_event.time_s + window_s + 1e-12)
        )
        if int(np.count_nonzero(mask)) < 2:
            raise ValueError("contact event window has fewer than two public samples")
        window_channels = {
            name: _channel_metrics(
                time[mask],
                np.asarray(reference.channels[name], dtype=float)[mask],
                np.asarray(candidate.channels[name], dtype=float)[mask],
                _unit_category(cast(Mapping[str, str], reference.units or {})[name]),
                acceptance,
            )
            for name in core
        }
        window_passed = all(
            cast(bool, value["passed"]) for value in window_channels.values()
        )
        pair_passed = bool(identity_passed and timing_passed and window_passed)
        all_passed = all_passed and pair_passed
        pairs.append(
            {
                "tire": reference_event.tire,
                "transition": reference_event.transition,
                "reference_time_s": reference_event.time_s,
                "candidate_time_s": candidate_event.time_s,
                "timing_error_s": timing_error,
                "identity_passed": identity_passed,
                "timing_passed": timing_passed,
                "window_channels": window_channels,
                "window_passed": window_passed,
                "passed": pair_passed,
            }
        )
    return {"pairs": pairs, "passed": all_passed}


def _compare_initialization(
    reference: AxleInitializationEvidence,
    candidate: AxleInitializationEvidence,
    acceptance: Mapping[str, object],
) -> dict[str, object]:
    gate = cast(Mapping[str, object], acceptance["initialization_gate"])
    translations = _mapping_vector_error(
        reference.translations_m,
        candidate.translations_m,
        float(gate["translation_m"]),
    )
    rotations = _mapping_vector_error(
        reference.rotation_vectors_rad,
        candidate.rotation_vectors_rad,
        float(gate["angle_rad"]),
    )
    wheel_loads = _mapping_relative_error(
        reference.wheel_loads_n,
        candidate.wheel_loads_n,
        float(gate["wheel_load_relative"]),
        float(
            cast(Mapping[str, object], acceptance["amplitude_floors"])[
                "force_n"
            ]
        ),
    )
    forces = _mapping_scalar_error(
        reference.component_forces_n,
        candidate.component_forces_n,
        float(gate["component_force_n"]),
    )
    moments = _mapping_scalar_error(
        reference.component_moments_n_m,
        candidate.component_moments_n_m,
        float(gate["component_moment_n_m"]),
    )
    constraint_position = max(
        reference.constraint_position_max_m,
        candidate.constraint_position_max_m,
    )
    constraint_velocity = max(
        reference.constraint_velocity_max_m_per_s,
        candidate.constraint_velocity_max_m_per_s,
    )
    position_passed = constraint_position <= float(gate["constraint_position_m"])
    velocity_passed = constraint_velocity <= float(
        gate["constraint_velocity_m_per_s"]
    )
    passed = bool(
        translations["passed"]
        and rotations["passed"]
        and wheel_loads["passed"]
        and forces["passed"]
        and moments["passed"]
        and position_passed
        and velocity_passed
    )
    return {
        "translations": translations,
        "rotations": rotations,
        "wheel_loads": wheel_loads,
        "component_forces": forces,
        "component_moments": moments,
        "constraint_position_max_m": constraint_position,
        "constraint_velocity_max_m_per_s": constraint_velocity,
        "constraint_position_passed": position_passed,
        "constraint_velocity_passed": velocity_passed,
        "passed": passed,
    }


def _mapping_vector_error(
    reference: Mapping[str, tuple[float, float, float]],
    candidate: Mapping[str, tuple[float, float, float]],
    limit: float,
) -> dict[str, object]:
    _require_same_keys(reference, candidate, "initialization vector")
    errors = {
        name: float(
            np.max(
                np.abs(
                    np.asarray(candidate[name], dtype=float)
                    - np.asarray(reference[name], dtype=float)
                )
            )
        )
        for name in reference
    }
    maximum = max(errors.values(), default=0.0)
    return {
        "maximum_absolute_error": maximum,
        "limit": limit,
        "errors": errors,
        "passed": maximum <= limit,
    }


def _mapping_scalar_error(
    reference: Mapping[str, float],
    candidate: Mapping[str, float],
    limit: float,
) -> dict[str, object]:
    _require_same_keys(reference, candidate, "initialization scalar")
    errors = {
        name: abs(float(candidate[name]) - float(reference[name]))
        for name in reference
    }
    maximum = max(errors.values(), default=0.0)
    return {
        "maximum_absolute_error": maximum,
        "limit": limit,
        "errors": errors,
        "passed": maximum <= limit,
    }


def _mapping_relative_error(
    reference: Mapping[str, float],
    candidate: Mapping[str, float],
    limit: float,
    floor: float,
) -> dict[str, object]:
    _require_same_keys(reference, candidate, "initialization relative")
    errors = {
        name: abs(float(candidate[name]) - float(reference[name]))
        / max(abs(float(reference[name])), floor)
        for name in reference
    }
    maximum = max(errors.values(), default=0.0)
    return {
        "maximum_relative_error": maximum,
        "limit": limit,
        "errors": errors,
        "passed": maximum <= limit,
    }


def _require_same_keys(
    reference: Mapping[str, object],
    candidate: Mapping[str, object],
    label: str,
) -> None:
    if set(reference) != set(candidate):
        missing = sorted(set(reference) - set(candidate))
        unexpected = sorted(set(candidate) - set(reference))
        raise ValueError(f"{label} keys differ; missing={missing}, unexpected={unexpected}")


def _fit_harmonic(
    time: np.ndarray,
    values: np.ndarray,
    frequency_hz: float,
) -> tuple[float, float]:
    omega_time = 2.0 * math.pi * frequency_hz * time
    design = np.column_stack(
        (
            np.sin(omega_time),
            np.cos(omega_time),
            np.ones_like(time),
        )
    )
    coefficients, *_ = np.linalg.lstsq(design, values, rcond=None)
    sine, cosine = float(coefficients[0]), float(coefficients[1])
    return math.hypot(sine, cosine), math.degrees(math.atan2(cosine, sine))


def _wrap_degrees(value: float) -> float:
    return (value + 180.0) % 360.0 - 180.0


def _unit_category(unit: str) -> str:
    categories = {
        "m": "translation_m",
        "m/s": "linear_velocity_m_per_s",
        "m/s^2": "linear_acceleration_m_per_s2",
        "rad": "angle_rad",
        "rad/s": "angular_velocity_rad_per_s",
        "N": "force_n",
        "N*m": "moment_n_m",
    }
    try:
        return categories[unit]
    except KeyError as exc:
        raise ValueError(f"unsupported frozen axle channel unit {unit!r}") from exc


def _harmonic_frequency(manifest: DynamicAxleManifest) -> float | None:
    metadata = cast(Mapping[str, object], manifest.payload["case_metadata"])
    value = metadata.get("harmonic_frequency_hz")
    return float(value) if isinstance(value, (int, float)) else None


def _validate_diagnostics(diagnostics: Mapping[str, object]) -> None:
    missing = [name for name in _DIAGNOSTIC_GATES if name not in diagnostics]
    if missing:
        raise ValueError(f"evidence diagnostics miss required gates: {missing}")
    for name in _DIAGNOSTIC_GATES:
        if not isinstance(diagnostics[name], bool):
            raise ValueError(f"evidence diagnostic {name} must be boolean")


def _diagnostic_gate(diagnostics: Mapping[str, object]) -> dict[str, object]:
    checks = {name: bool(diagnostics[name]) for name in _DIAGNOSTIC_GATES}
    return {"checks": checks, "passed": bool(all(checks.values()))}


def _hash_declared_artifacts(
    destination: Path,
    raw_artifacts: Sequence[str | Path],
) -> dict[str, str]:
    if not raw_artifacts:
        raise ValueError("raw_artifacts cannot be empty")
    root = destination.resolve()
    hashes: dict[str, str] = {}
    for raw in raw_artifacts:
        path = Path(raw)
        # A relative path may already be rooted at the current directory, since
        # the runners build artifact paths from their own output directory. Only
        # resolve against the evidence directory when that is not the case,
        # otherwise a relative output directory is joined twice.
        if path.is_absolute() or path.is_file():
            absolute = path.resolve()
        else:
            absolute = (destination / path).resolve()
        try:
            relative = absolute.relative_to(root).as_posix()
        except ValueError as exc:
            raise ValueError("raw artifacts must stay inside the evidence directory") from exc
        if not absolute.is_file():
            raise ValueError(f"declared raw artifact is missing: {relative}")
        hashes[relative] = _file_hash(absolute)
    return dict(sorted(hashes.items()))


def _validate_raw_contract(
    producer_kind: Literal["msc.adams", "open-kinematics.native"],
    raw_artifacts: Mapping[str, str],
) -> None:
    names = tuple(raw_artifacts)
    if producer_kind == "msc.adams":
        missing = [
            suffix
            for suffix in _ADAMS_REQUIRED_SUFFIXES
            if not any(name.lower().endswith(suffix) for name in names)
        ]
        if missing:
            raise ValueError(f"Adams evidence misses required raw files: {missing}")
    else:
        missing = [
            required
            for required in _NATIVE_REQUIRED_NAMES
            if not any(Path(name).name == required for name in names)
        ]
        if missing:
            raise ValueError(f"native evidence misses required raw files: {missing}")


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def _phase_lag_ms(
    time: np.ndarray,
    reference: np.ndarray,
    candidate: np.ndarray,
) -> float:
    """Estimate candidate lag by the maximum discrete cross-correlation."""
    if len(time) < 3:
        return 0.0
    reference_centered = reference - np.mean(reference)
    candidate_centered = candidate - np.mean(candidate)
    if (
        np.linalg.norm(reference_centered) <= np.finfo(float).eps
        or np.linalg.norm(candidate_centered) <= np.finfo(float).eps
    ):
        return 0.0
    correlation = np.correlate(
        candidate_centered,
        reference_centered,
        mode="full",
    )
    lag_samples = int(np.argmax(correlation)) - (len(reference) - 1)
    step = float(np.median(np.diff(time)))
    return 1000.0 * lag_samples * step


def _quaternion_log(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float).copy()
    q /= np.linalg.norm(q)
    if q[0] < 0.0:
        q = -q
    vector_norm = float(np.linalg.norm(q[1:]))
    if vector_norm <= 1e-14:
        return 2.0 * q[1:]
    angle = 2.0 * math.atan2(vector_norm, float(q[0]))
    return q[1:] * (angle / vector_norm)


def _failure_attribution(
    transient: Mapping[str, object],
    diagnostics: Mapping[str, Mapping[str, object]],
) -> list[str]:
    reasons: list[str] = []
    if not diagnostics["reference"]["passed"]:
        reasons.append("adams_solver_or_export_diagnostics")
    if not diagnostics["candidate"]["passed"]:
        reasons.append("native_numerical_error")
    events = transient.get("contact_events")
    if isinstance(events, Mapping) and cast(Mapping[str, object], events).get(
        "passed"
    ) is not True:
        reasons.append("contact_model_or_event_localization_difference")
    harmonic = transient.get("harmonic")
    if isinstance(harmonic, Mapping) and cast(Mapping[str, object], harmonic).get(
        "passed"
    ) is not True:
        reasons.append("dynamic_model_or_parameter_difference")
    channels = cast(Mapping[str, Mapping[str, object]], transient["channels"])
    if any(not value["passed"] for value in channels.values()):
        reasons.append("model_parameter_coordinate_or_numerical_difference")
    return reasons


def _write_report(
    report: dict[str, object],
    output_path: str | Path | None,
) -> dict[str, object]:
    if output_path is not None:
        destination = Path(output_path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_text(
            json.dumps(report, indent=2, sort_keys=True),
            encoding="utf-8",
        )
    return report
