"""Independent JSON/Parquet/CSV result protocol."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import pyarrow as pa
import pyarrow.parquet as pq

from .. import __version__
from ..schema import DynamicResultBundle, ResultBundle

META_KEY = "suspension_multibody_meta"
FORMAT_VERSION = "1.0"


def canonical_hash(value: Any) -> str:
    """Hash canonical JSON data with stable key ordering."""
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def write_bundle(
    bundle: ResultBundle,
    output_dir: str | Path,
    *,
    formats: tuple[str, ...] = ("parquet", "csv"),
) -> Path:
    """Write manifest and result tables, returning the manifest path."""
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    manifest_path = directory / "manifest.json"
    manifest = bundle.manifest.model_dump(mode="json")
    manifest["provenance"]["package_version"] = manifest["provenance"].get(
        "package_version", __version__
    )
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
    )
    tables = {
        "states": [_state_row(row) for row in bundle.states],
        "component_loads": [_component_row(row) for row in bundle.component_loads],
        "bushings": [_bushing_row(row) for row in bundle.bushings],
        "diagnostics": [row.model_dump(mode="json") for row in bundle.diagnostics],
    }
    metadata = {
        "format_version": FORMAT_VERSION,
        "schema_version": "1",
        "package_version": __version__,
        "run_id": bundle.manifest.run_id,
    }
    for name, rows in tables.items():
        if "parquet" in formats:
            _write_parquet(directory / f"{name}.parquet", rows, metadata)
        if "csv" in formats:
            _write_csv(directory / f"{name}.csv", rows)
    return manifest_path


def write_dynamic_bundle(
    bundle: DynamicResultBundle,
    output_dir: str | Path,
    *,
    formats: tuple[str, ...] = ("parquet", "csv"),
) -> Path:
    """Write a dynamic manifest and time-history tables."""
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    manifest_path = directory / "manifest.json"
    manifest = bundle.manifest.model_dump(mode="json")
    manifest["provenance"]["package_version"] = manifest["provenance"].get(
        "package_version", __version__
    )
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
    )
    rows = [_dynamic_sample_row(row) for row in bundle.samples]
    diagnostics = [row.model_dump(mode="json") for row in bundle.diagnostics]
    metadata = {
        "format_version": FORMAT_VERSION,
        "schema_version": "1",
        "package_version": __version__,
        "run_id": bundle.manifest.run_id,
    }
    for name, table_rows in {
        "time_samples": rows,
        "diagnostics": diagnostics,
    }.items():
        if "parquet" in formats:
            _write_parquet(directory / f"{name}.parquet", table_rows, metadata)
        if "csv" in formats:
            _write_csv(directory / f"{name}.csv", table_rows)
    return manifest_path


def read_table(path: str | Path) -> list[dict[str, Any]]:
    """Read a CSV or Parquet result table into dictionaries."""
    source = Path(path)
    if source.suffix.lower() == ".parquet":
        return pq.read_table(source).to_pylist()
    with source.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def _write_parquet(
    path: Path, rows: list[dict[str, Any]], metadata: dict[str, str]
) -> None:
    table = (
        pa.Table.from_pylist(rows)
        if rows
        else pa.table({"_empty": pa.array([], type=pa.string())})
    )
    encoded = {META_KEY.encode(): json.dumps(metadata, sort_keys=True).encode()}
    table = table.replace_schema_metadata(encoded)
    pq.write_table(table, path)


def _write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    columns = sorted({key for row in rows for key in row})
    with path.open("w", newline="", encoding="utf-8") as stream:
        if not columns:
            stream.write("\n")
            return
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in columns} for row in rows)


def _state_row(row: Any) -> dict[str, Any]:
    data = row.model_dump(mode="json")
    for key in (
        "drives",
        "external_loads",
        "poses",
        "metrics",
        "c_response",
        "tire_compression",
        "diagnostics",
    ):
        data[key] = json.dumps(data[key], sort_keys=True, default=str)
    return data


def _component_row(row: Any) -> dict[str, Any]:
    data = row.model_dump(mode="json")
    data["global_load"] = json.dumps(data["global_load"], sort_keys=True)
    data["local_load"] = json.dumps(data["local_load"], sort_keys=True)
    return data


def _bushing_row(row: Any) -> dict[str, Any]:
    data = row.model_dump(mode="json")
    data["deformation"] = json.dumps(data["deformation"], sort_keys=True)
    data["load"] = json.dumps(data["load"], sort_keys=True)
    data["zero_load_pose"] = json.dumps(data["zero_load_pose"], sort_keys=True)
    return data


def _dynamic_sample_row(row: Any) -> dict[str, Any]:
    data = row.model_dump(mode="json")
    for key in ("pose", "velocity", "acceleration", "loads", "metrics", "events"):
        data[key] = json.dumps(data[key], sort_keys=True, default=str)
    return data
