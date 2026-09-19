"""Independent JSON/Parquet/CSV result protocol."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path
from typing import Any

import pyarrow.parquet as pq

META_KEY = "suspension_multibody_meta"


def canonical_hash(value: Any) -> str:
    """Hash canonical JSON data with stable key ordering."""
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), default=str
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def read_table(path: str | Path) -> list[dict[str, Any]]:
    """Read a CSV or Parquet result table into dictionaries."""
    source = Path(path)
    if source.suffix.lower() == ".parquet":
        return pq.read_table(source).to_pylist()
    with source.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


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
