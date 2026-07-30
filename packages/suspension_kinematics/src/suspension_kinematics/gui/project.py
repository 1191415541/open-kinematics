"""Shared JSON project file helpers for GUI workbenches."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

PROJECT_SCHEMA_VERSION = 1
PROJECT_FILE_EXTENSION = ".okproj.json"


def build_project_document(
    *,
    module: str,
    system_type: str,
    name: str,
    hardpoints: object,
    parameters: dict[str, Any],
    simulation: dict[str, Any],
    version: str | None = None,
    units: str | None = None,
    curves: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
    """Build the shared GUI project JSON envelope."""
    data: dict[str, Any] = {
        "schema_version": PROJECT_SCHEMA_VERSION,
        "module": module,
        "system_type": system_type,
        "name": name,
        "hardpoints": hardpoints,
        "parameters": parameters,
        "simulation": simulation,
    }
    if version is not None:
        data["version"] = version
    if units is not None:
        data["units"] = units
    if curves is not None:
        data["curves"] = curves
    return data


def read_project_document(path: str | Path) -> dict[str, Any]:
    """Read a GUI project JSON document."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Project file must contain a JSON object")
    return data


def write_project_document(data: dict[str, Any], path: str | Path) -> None:
    """Write a GUI project JSON document."""
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def is_project_document(data: dict[str, Any]) -> bool:
    """Return whether data uses the shared GUI project envelope."""
    return data.get("schema_version") == PROJECT_SCHEMA_VERSION and "module" in data
