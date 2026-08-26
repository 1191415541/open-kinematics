"""Versioned axle-dynamics schema and result I/O."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from pydantic import ValidationError

from .. import __version__
from ..io import canonical_hash
from .result import (
    ANTI_ROLL_OUTPUT_COLUMNS,
    BODY_STATE_COLUMNS,
    BUSHING_OUTPUT_COLUMNS,
    CONSTRAINT_WRENCH_COLUMNS,
    DIAGNOSTIC_COLUMNS,
    ENERGY_COLUMNS,
    PERFORMANCE_COLUMNS,
    SPRING_OUTPUT_COLUMNS,
    TIRE_OUTPUT_COLUMNS,
    AxleDynamicsResult,
)
from .schema import AxleDynamicsCase, AxleDynamicsModel


def _read(path: str | Path) -> dict[str, Any]:
    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
        data = (
            json.loads(text)
            if source.suffix.lower() == ".json"
            else yaml.safe_load(text)
        )
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read axle schema file {source}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"axle schema file {source} must contain an object")
    if data.get("schema_version") != 1:
        raise ValueError(
            f"unsupported schema_version {data.get('schema_version')!r}; expected 1"
        )
    return data


def load_axle_dynamics_model(path: str | Path) -> AxleDynamicsModel:
    """Load a closed SI axle dynamics model from YAML or JSON."""
    try:
        return AxleDynamicsModel.model_validate(_read(path))
    except ValidationError as exc:
        raise ValueError(f"invalid AxleDynamicsModel in {path}: {exc}") from exc


def load_axle_dynamics_case(path: str | Path) -> AxleDynamicsCase:
    """Load an axle dynamics case from YAML or JSON."""
    try:
        return AxleDynamicsCase.model_validate(_read(path))
    except ValidationError as exc:
        raise ValueError(f"invalid AxleDynamicsCase in {path}: {exc}") from exc


def write_axle_dynamics_artifact(
    result: AxleDynamicsResult | None,
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    output_dir: str | Path,
    *,
    failure: Exception | None = None,
) -> Path:
    """Write self-describing raw arrays and a reproducibility manifest."""
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    from .native import native_build_metadata

    model_payload = model.model_dump(mode="json")
    case_payload = case.model_dump(mode="json")
    failure_row = getattr(failure, "failure_diagnostics", None)
    failed_index = getattr(failure, "failed_sample_index", None)
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "artifact_type": "axle_dynamics_result",
        "status": "failed" if failure is not None else "success",
        "package_version": __version__,
        "model_name": model.name,
        "case_name": case.name,
        "model_sha256": canonical_hash(model_payload),
        "case_sha256": canonical_hash(case_payload),
        "model": model_payload,
        "case": case_payload,
        "native_build": native_build_metadata(),
        "completed_sample_count": 0 if result is None else len(result.times_s),
        "failed_sample_index": failed_index,
        "failed_time_s": getattr(failure, "failed_time_s", None),
        "native_status": getattr(failure, "status", 0),
        "performance": (
            None if result is None else asdict(result.performance)
        ),
        "error": None if failure is None else str(failure),
        "failure_diagnostics": (
            None
            if failure_row is None
            else {
                name: float(value)
                for name, value in zip(DIAGNOSTIC_COLUMNS, failure_row)
            }
        ),
        "layouts": {
            "body_state": BODY_STATE_COLUMNS,
            "constraint_wrench": CONSTRAINT_WRENCH_COLUMNS,
            "spring_output": SPRING_OUTPUT_COLUMNS,
            "bushing_output": BUSHING_OUTPUT_COLUMNS,
            "anti_roll_output": ANTI_ROLL_OUTPUT_COLUMNS,
            "diagnostics": DIAGNOSTIC_COLUMNS,
            "tire_output": TIRE_OUTPUT_COLUMNS,
            "energy": ENERGY_COLUMNS,
            "performance": PERFORMANCE_COLUMNS,
        },
        "arrays_file": "arrays.npz" if result is not None else None,
    }
    if result is not None:
        np.savez_compressed(
            destination / "arrays.npz",
            times_s=result.times_s,
            body_names=np.asarray(result.body_names),
            constraint_names=np.asarray(result.constraint_names),
            spring_names=np.asarray(result.spring_names),
            bushing_names=np.asarray(result.bushing_names),
            anti_roll_bar_names=np.asarray(result.anti_roll_bar_names),
            tire_names=np.asarray(result.tire_names),
            states=result.states,
            constraint_wrench=result.constraint_wrench,
            spring_output=result.spring_output,
            bushing_output=result.bushing_output,
            anti_roll_output=result.anti_roll_output,
            diagnostics=np.column_stack(
                tuple(
                    getattr(result.diagnostics, field)
                    for field in (
                        "accepted",
                        "internal_steps",
                        "rejected_attempts",
                        "newton_iterations",
                        "minimum_accepted_step_s",
                        "maximum_accepted_step_s",
                        "last_accepted_step_s",
                        "position_residual",
                        "velocity_residual",
                        "dynamics_residual",
                        "active_contacts",
                        "contact_events",
                        "local_error_ratio",
                        "energy_residual",
                        "failure_code",
                        "pinned_null_directions",
                    )
                )
            ),
            tire_output=result.tire_output,
            energy=result.energy,
            contact_event_time_s=np.asarray(
                [event.time_s for event in result.contact_events],
                dtype=np.float64,
            ),
            contact_event_tire=np.asarray(
                [event.tire for event in result.contact_events],
            ),
            contact_event_transition=np.asarray(
                [event.transition for event in result.contact_events],
            ),
        )
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return manifest_path
