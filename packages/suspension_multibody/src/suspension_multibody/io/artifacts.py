"""Unified artifact read/write protocol for public simulation outputs."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from .. import __version__
from .results import (
    META_KEY,
    _bushing_row,
    _component_row,
    _state_row,
)

ARTIFACT_SCHEMA_VERSION = 1
ARTIFACT_FORMAT_VERSION = "1.0"


def write_artifact(
    result: Any,
    output_dir: str | Path,
    *,
    request: Any | None = None,
    model: Any | None = None,
    case: Any | None = None,
    metrics: Mapping[str, Any] | None = None,
    status: str | None = None,
    failure: BaseException | None = None,
    partial: Any | None = None,
    formats: tuple[str, ...] = ("parquet", "csv"),
) -> Path:
    """
    Write one success, partial, or failed result artifact.

    The writer accepts the formal result objects used by the public services:
    ``ResultBundle``, ``TimeSeriesResult``, ``AxleDynamicsResult`` and
    ``VehicleDynamicsResult``.  Domain inputs are optional so historical and
    failure-only artifacts can still be written with the evidence available.
    """
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)

    storage_result = result if result is not None else partial
    artifact_type = _artifact_type(storage_result, partial)
    normalized_status = _status(result, status=status, failure=failure, partial=partial)
    model_payload = _jsonable(_dump(model)) if model is not None else None
    case_payload = _jsonable(_dump(case)) if case is not None else None
    request_payload = _jsonable(_dump(request)) if request is not None else None
    failure_payload = _failure_payload(failure)
    if not failure_payload and result is not None:
        failure_payload = _jsonable(dict(getattr(result, "failure_evidence", {}) or {}))
    partial_evidence = _partial_evidence(result, partial, failure)
    _validate_status_evidence(normalized_status, failure_payload, partial_evidence)

    manifest: dict[str, Any] = {
        "schema_version": ARTIFACT_SCHEMA_VERSION,
        "format_version": ARTIFACT_FORMAT_VERSION,
        "artifact_type": artifact_type,
        "status": normalized_status,
        "package_version": __version__,
        "run_id": _run_id(storage_result, partial),
        "model_hash": _hash_payload(model_payload),
        "model_sha256": _hash_payload(model_payload),
        "case_hash": _hash_payload(case_payload),
        "case_sha256": _hash_payload(case_payload),
        "model": model_payload,
        "case": case_payload,
        "request": request_payload,
        "metrics": _jsonable(dict(metrics or _metrics(storage_result))),
        "diagnostics": _jsonable(_diagnostics(storage_result)),
        "performance": _jsonable(_performance(storage_result)),
        "failure_evidence": failure_payload,
        "partial_evidence": partial_evidence,
        "channels": {},
        "layouts": {},
        "time_grid": {},
        "arrays_file": None,
        "tables": [],
    }

    if artifact_type == "result_bundle":
        _write_result_bundle(storage_result, destination, manifest, formats)
    elif artifact_type == "time_series_result":
        _write_time_series(storage_result, destination, manifest, formats)
    elif artifact_type in {"axle_dynamics_result", "vehicle_dynamics_result"}:
        _write_native_result(storage_result, destination, manifest)
    else:
        _write_generic_result(storage_result, destination, manifest)

    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, default=str),
        encoding="utf-8",
    )
    return manifest_path


def read_artifact(path: str | Path) -> dict[str, Any]:
    """Read a unified artifact directory into manifest, arrays, and tables."""
    source = Path(path)
    directory = source if source.is_dir() else source.parent
    manifest_path = source / "manifest.json" if source.is_dir() else source
    if not manifest_path.is_file():
        raise FileNotFoundError(f"artifact manifest not found: {manifest_path}")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    arrays: dict[str, np.ndarray] = {}
    arrays_name = manifest.get("arrays_file")
    if arrays_name:
        arrays_path = directory / str(arrays_name)
        if arrays_path.is_file():
            with np.load(arrays_path, allow_pickle=False) as loaded:
                arrays = {name: loaded[name].copy() for name in loaded.files}

    tables: dict[str, list[dict[str, Any]]] = {}
    for name in manifest.get("tables", []):
        parquet_path = directory / f"{name}.parquet"
        csv_path = directory / f"{name}.csv"
        if parquet_path.is_file():
            tables[str(name)] = pq.read_table(parquet_path).to_pylist()
        elif csv_path.is_file():
            with csv_path.open(newline="", encoding="utf-8") as stream:
                tables[str(name)] = list(csv.DictReader(stream))

    time_samples: list[dict[str, Any]] = []
    samples_file = manifest.get("samples_file")
    if samples_file:
        samples_path = directory / str(samples_file)
        if samples_path.is_file():
            payload = json.loads(samples_path.read_text(encoding="utf-8"))
            if isinstance(payload, list):
                time_samples = payload

    diagnostics = manifest.get("diagnostics")
    diagnostics_path = directory / "diagnostics.json"
    if diagnostics_path.is_file():
        diagnostics = json.loads(diagnostics_path.read_text(encoding="utf-8"))

    return {
        "manifest": manifest,
        "arrays": arrays,
        "tables": tables,
        "time_samples": time_samples,
        "diagnostics": diagnostics,
        "performance": manifest.get("performance"),
        "metrics": manifest.get("metrics"),
        "failure_evidence": manifest.get("failure_evidence"),
        "partial_evidence": manifest.get("partial_evidence"),
    }


def _artifact_type(result: Any, partial: Any | None) -> str:
    value = result if result is not None else partial
    if value is None:
        return "unknown_result"
    from ..results import TimeSeriesResult
    from ..schema import ResultBundle

    if isinstance(value, ResultBundle):
        return "result_bundle"
    if isinstance(value, TimeSeriesResult):
        return "time_series_result"
    if hasattr(value, "axle") and hasattr(value, "steering_output"):
        return "vehicle_dynamics_result"
    if hasattr(value, "states") and hasattr(value, "constraint_wrench"):
        return "axle_dynamics_result"
    return "generic_result"


def _status(
    result: Any,
    *,
    status: str | None,
    failure: BaseException | None,
    partial: Any | None,
) -> str:
    if status is not None:
        normalized = str(status).strip().lower()
    elif failure is not None and result is None:
        normalized = "failed"
    elif partial is not None:
        normalized = "partial"
    else:
        normalized = str(getattr(result, "status", "success")).strip().lower()
    if normalized not in {"success", "partial", "failed", "unavailable", "not_applicable"}:
        raise ValueError(f"unsupported artifact status {normalized!r}")
    return normalized


def _dump(value: Any) -> Any:
    if value is None:
        return None
    if hasattr(value, "model_dump"):
        return value.model_dump(mode="json")
    if hasattr(value, "as_dict"):
        return value.as_dict()
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, Mapping):
        return dict(value)
    return value


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    if is_dataclass(value):
        return _jsonable(asdict(value))
    return value


def _hash_payload(value: Any) -> str | None:
    if value is None:
        return None
    from .results import canonical_hash

    return canonical_hash(value)


def _run_id(result: Any, partial: Any | None) -> str | None:
    value = result if result is not None else partial
    if value is None:
        return None
    run_id = getattr(value, "run_id", None)
    return None if run_id is None else str(run_id)


def _metrics(result: Any) -> Mapping[str, Any]:
    if result is None:
        return {}
    metrics = getattr(result, "metrics", None)
    if isinstance(metrics, Mapping):
        return metrics
    return {}


def _performance(result: Any) -> Any:
    if result is None:
        return {}
    performance = getattr(result, "performance", None)
    if performance is not None:
        return _dump(performance)
    return {}


def _diagnostics(result: Any) -> Any:
    if result is None:
        return {}
    diagnostics = getattr(result, "diagnostics", None)
    if diagnostics is None:
        return {}
    return _dump(diagnostics)


def _failure_payload(failure: BaseException | None) -> dict[str, Any]:
    if failure is None:
        return {}
    payload: dict[str, Any] = {
        "type": type(failure).__name__,
        "message": str(failure),
    }
    for name in (
        "status",
        "failed_sample_index",
        "failed_time_s",
        "failure_diagnostics",
    ):
        value = getattr(failure, name, None)
        if value is not None:
            payload[name] = _jsonable(value)
    return payload

def _validate_status_evidence(
    status: str,
    failure_evidence: Mapping[str, Any],
    partial_evidence: Mapping[str, Any],
) -> None:
    """Require traceable evidence for explicit partial and failed artifacts."""
    if status == "partial" and not partial_evidence:
        raise ValueError("partial artifacts require non-empty partial_evidence")
    if status == "failed" and not failure_evidence:
        raise ValueError("failed artifacts require non-empty failure_evidence")


def _partial_evidence(
    result: Any,
    partial: Any | None,
    failure: BaseException | None,
) -> dict[str, Any]:
    evidence: dict[str, Any] = {}
    value = partial
    if value is not None:
        evidence["available"] = True
        evidence["result_type"] = type(value).__name__
        count = getattr(value, "times_s", None)
        if count is not None:
            evidence["completed_sample_count"] = len(count)
    if failure is not None:
        for name in ("failed_sample_index", "failed_time_s"):
            item = getattr(failure, name, None)
            if item is not None:
                evidence[name] = _jsonable(item)
    if result is not None and getattr(result, "partial_evidence", None):
        evidence.update(_jsonable(getattr(result, "partial_evidence")))
    return evidence


def _write_result_bundle(
    bundle: Any,
    destination: Path,
    manifest: dict[str, Any],
    formats: tuple[str, ...],
) -> None:
    manifest.update(_jsonable(bundle.manifest.model_dump(mode="json")))
    manifest["artifact_type"] = "result_bundle"
    tables = {
        "states": [_state_row(row) for row in bundle.states],
        "component_loads": [_component_row(row) for row in bundle.component_loads],
        "bushings": [_bushing_row(row) for row in bundle.bushings],
        "diagnostics": [_jsonable(_dump(row)) for row in bundle.diagnostics],
    }
    for name, rows in tables.items():
        _write_table_files(destination, name, rows, formats)
    manifest["tables"] = list(tables)
    manifest["layouts"] = {name: sorted({key for row in rows for key in row}) for name, rows in tables.items()}


def _write_time_series(
    result: Any,
    destination: Path,
    manifest: dict[str, Any],
    formats: tuple[str, ...],
) -> None:
    payload = result.as_dict()
    outer_status = manifest.get("status")
    outer_failure = dict(manifest.get("failure_evidence") or {})
    outer_partial = dict(manifest.get("partial_evidence") or {})
    manifest.update(_jsonable(payload["manifest"]))
    manifest.update(
        {
            "artifact_type": "time_series_result",
            "arrays_file": "arrays.npz",
            "samples_file": "time_samples.json",
            "time_grid": {"count": len(result.times_s), "values": result.times_s.tolist()},
            "channels": sorted(
                {
                    key
                    for sample in result.samples
                    for key in sample.metrics
                }
            ),
        }
    )
    if outer_status is not None:
        manifest["status"] = outer_status
    if outer_failure:
        manifest["failure_evidence"] = {
            **dict(manifest.get("failure_evidence") or {}),
            **outer_failure,
        }
    if outer_partial:
        manifest["partial_evidence"] = {
            **dict(manifest.get("partial_evidence") or {}),
            **outer_partial,
        }
    np.savez_compressed(
        destination / "arrays.npz",
        times_s=np.asarray(result.times_s, dtype=np.float64),
        sample_times_s=np.asarray([sample.time for sample in result.samples], dtype=np.float64),
        sample_body=np.asarray([sample.body for sample in result.samples]),
        sample_converged=np.asarray([sample.converged for sample in result.samples], dtype=bool),
    )
    rows = [sample.as_dict() for sample in result.samples]
    (destination / "time_samples.json").write_text(
        json.dumps(rows, indent=2, sort_keys=True, default=str), encoding="utf-8"
    )
    _write_table_files(destination, "time_samples", rows, formats)
    diagnostics = _jsonable(result.diagnostics)
    (destination / "diagnostics.json").write_text(
        json.dumps(diagnostics, indent=2, sort_keys=True, default=str), encoding="utf-8"
    )
    manifest["tables"] = ["time_samples"]
    manifest["layouts"] = {
        "time_samples": sorted({key for row in rows for key in row}),
        "diagnostics": sorted(diagnostics) if isinstance(diagnostics, Mapping) else [],
    }


def _write_native_result(
    result: Any,
    destination: Path,
    manifest: dict[str, Any],
) -> None:
    from ..axle_dynamics.result import (
        ANTI_ROLL_OUTPUT_COLUMNS,
        BODY_STATE_COLUMNS,
        BUSHING_OUTPUT_COLUMNS,
        CONSTRAINT_WRENCH_COLUMNS,
        DIAGNOSTIC_COLUMNS,
        ENERGY_COLUMNS,
        PERFORMANCE_COLUMNS,
        SPRING_OUTPUT_COLUMNS,
        TIRE_OUTPUT_COLUMNS,
    )

    axle = getattr(result, "axle", result)
    diagnostics = axle.diagnostics
    arrays: dict[str, Any] = {
        "times_s": axle.times_s,
        "body_names": np.asarray(axle.body_names),
        "constraint_names": np.asarray(axle.constraint_names),
        "spring_names": np.asarray(axle.spring_names),
        "bushing_names": np.asarray(axle.bushing_names),
        "anti_roll_bar_names": np.asarray(axle.anti_roll_bar_names),
        "tire_names": np.asarray(axle.tire_names),
    }
    if hasattr(result, "steering_output"):
        arrays["steering_names"] = np.asarray(getattr(result, "steering_names", ()))
    arrays.update(
        {
            "states": axle.states,
            "constraint_wrench": axle.constraint_wrench,
            "spring_output": axle.spring_output,
            "bushing_output": axle.bushing_output,
            "anti_roll_output": axle.anti_roll_output,
            "diagnostics": _diagnostics_array(diagnostics),
            "tire_output": axle.tire_output,
            "energy": axle.energy,
            "contact_event_time_s": np.asarray(
                [event.time_s for event in axle.contact_events], dtype=np.float64
            ),
            "contact_event_tire": np.asarray([event.tire for event in axle.contact_events]),
            "contact_event_transition": np.asarray(
                [event.transition for event in axle.contact_events]
            ),
        }
    )
    if hasattr(result, "steering_output"):
        arrays["steering_output"] = (
            result.steering_output
            if result.steering_output is not None
            else np.empty((len(result.times_s), 0, 4), dtype=np.float64)
        )
    np.savez_compressed(destination / "arrays.npz", **arrays)
    manifest.update(
        {
            "artifact_type": (
                "vehicle_dynamics_result"
                if hasattr(result, "steering_output")
                else "axle_dynamics_result"
            ),
            "arrays_file": "arrays.npz",
            "time_grid": {"count": len(axle.times_s), "values": axle.times_s.tolist()},
            "channels": {
                "body": list(axle.body_names),
                "constraint": list(axle.constraint_names),
                "spring": list(axle.spring_names),
                "bushing": list(axle.bushing_names),
                "anti_roll_bar": list(axle.anti_roll_bar_names),
                "tire": list(axle.tire_names),
            },
            "layouts": {
                "body_state": list(BODY_STATE_COLUMNS),
                "constraint_wrench": list(CONSTRAINT_WRENCH_COLUMNS),
                "spring_output": list(SPRING_OUTPUT_COLUMNS),
                "bushing_output": list(BUSHING_OUTPUT_COLUMNS),
                "anti_roll_output": list(ANTI_ROLL_OUTPUT_COLUMNS),
                "diagnostics": list(DIAGNOSTIC_COLUMNS),
                "tire_output": list(TIRE_OUTPUT_COLUMNS),
                "energy": list(ENERGY_COLUMNS),
                "performance": list(PERFORMANCE_COLUMNS),
            },
        }
    )
    if hasattr(result, "steering_output"):
        manifest["layouts"]["steering_output"] = [
            "coordinate_m_or_angle_rad",
            "rate_per_s",
            "target_m_or_angle_rad",
            "actuator_force_or_torque",
        ]


def _write_generic_result(
    result: Any,
    destination: Path,
    manifest: dict[str, Any],
) -> None:
    payload = _jsonable(_dump(result))
    (destination / "result.json").write_text(
        json.dumps(payload, indent=2, sort_keys=True, default=str), encoding="utf-8"
    )
    manifest["result_file"] = "result.json"


def _diagnostics_array(diagnostics: Any) -> np.ndarray:
    fields = (
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
    return np.column_stack(tuple(np.asarray(getattr(diagnostics, field)) for field in fields))


def _write_table_files(
    destination: Path,
    name: str,
    rows: list[dict[str, Any]],
    formats: tuple[str, ...],
) -> None:
    normalized_rows = [
        {key: _table_value(value) for key, value in row.items()}
        for row in rows
    ]
    metadata = {
        "format_version": ARTIFACT_FORMAT_VERSION,
        "schema_version": str(ARTIFACT_SCHEMA_VERSION),
        "package_version": __version__,
    }
    if "parquet" in formats:
        table = (
            pa.Table.from_pylist(normalized_rows)
            if normalized_rows
            else pa.table({"_empty": pa.array([], type=pa.string())})
        )
        table = table.replace_schema_metadata({META_KEY.encode(): json.dumps(metadata).encode()})
        pq.write_table(table, destination / f"{name}.parquet")
    if "csv" in formats:
        columns = sorted({key for row in normalized_rows for key in row})
        with (destination / f"{name}.csv").open("w", newline="", encoding="utf-8") as stream:
            if columns:
                writer = csv.DictWriter(stream, fieldnames=columns)
                writer.writeheader()
                writer.writerows({key: row.get(key) for key in columns} for row in normalized_rows)
            else:
                stream.write("\n")


def _table_value(value: Any) -> Any:
    """Flatten nested values before writing Arrow/CSV table rows."""
    if isinstance(value, (Mapping, list, tuple, np.ndarray)):
        return json.dumps(_jsonable(value), sort_keys=True, default=str)
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    return value
