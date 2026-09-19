"""Versioned axle-dynamics schema and result I/O."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import yaml
from pydantic import ValidationError

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
