"""Unified artifact read/write protocol for public simulation outputs."""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, is_dataclass
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import pyarrow.parquet as pq

from .. import __version__

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
    inputs: Mapping[str, Any] | None = None,
) -> Path:
    """
    Write one success, partial, or failed result artifact.

    Every solved or partial result is a ResultEnvelope. Failure-only artifacts
    preserve the available diagnostics without inventing solved channels.
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
    if not failure_payload and result is not None and normalized_status in {"failed", "partial"}:
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

    if artifact_type == "multibody_result":
        _write_multibody_result(storage_result, destination, manifest)
    else:
        _write_generic_result(storage_result, destination, manifest)

    if inputs is not None:
        # A sidecar rather than a manifest key: an existing run's artifact stays
        # byte-for-byte what it was, and a file-driven run says which bytes it read.
        (destination / "inputs.json").write_text(
            json.dumps(_jsonable(dict(inputs)), indent=2, sort_keys=True, default=str),
            encoding="utf-8",
        )
        manifest["inputs_file"] = "inputs.json"

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
    from ..results.envelope import ResultEnvelope

    if isinstance(value, ResultEnvelope):
        return "multibody_result"
    raise TypeError("artifact writer requires ResultEnvelope")


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
    if hasattr(value, "to_payload"):
        return value.to_payload()
    if hasattr(value, "to_document"):
        return value.to_document()
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


def _write_multibody_result(result: Any, destination: Path, manifest: dict[str, Any]) -> None:
    """Persist native channels and their stable identity manifest without dispatch."""
    arrays = {"times_s": result.times_s, **result.named_blocks}
    np.savez_compressed(destination / "arrays.npz", **arrays)
    native = _jsonable(result.raw.document)
    (destination / "native_result.json").write_text(json.dumps(native, indent=2, sort_keys=True), encoding="utf-8")
    model = result.model.to_document()
    manifest.update({
        "artifact_type": "multibody_result", "arrays_file": "arrays.npz", "result_file": "native_result.json",
        "model_fingerprint": result.model_fingerprint,
        "model": model, "model_hash": _hash_payload(model), "model_sha256": _hash_payload(model),
        "time_grid": {"count": len(result.times_s), "values": result.times_s.tolist()},
        "channels": _jsonable(result.raw.metadata),
        "layouts": {name: {"shape": list(values.shape), "dtype": str(values.dtype)} for name, values in arrays.items()},
    })


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
