"""Versioned JSON/YAML schema loading."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, TypeVar

import yaml
from pydantic import BaseModel, ValidationError

from .case import CaseSpec
from .model import FrontAxleModel
from .result import ResultBundle

T = TypeVar("T", bound=BaseModel)


def _read(path: str | Path) -> Any:
    source = Path(path)
    try:
        text = source.read_text(encoding="utf-8")
        data = (
            json.loads(text)
            if source.suffix.lower() == ".json"
            else yaml.safe_load(text)
        )
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise ValueError(f"cannot read schema file {source}: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError(f"schema file {source} must contain an object")
    version = data.get("schema_version")
    if version != 1:
        raise ValueError(f"unsupported schema_version {version!r}; expected 1")
    return data


def load_model(path: str | Path) -> FrontAxleModel:
    """Load and validate a model YAML/JSON file."""
    return _validate(_read(path), FrontAxleModel, path)


def load_case(path: str | Path) -> CaseSpec:
    """Load and validate a case YAML/JSON file."""
    return _validate(_read(path), CaseSpec, path)


def load_result(path: str | Path) -> ResultBundle:
    """Load and validate a result JSON file."""
    return _validate(_read(path), ResultBundle, path)


def _validate(data: Any, model: type[T], path: str | Path) -> T:
    try:
        return model.model_validate(data)
    except ValidationError as exc:
        raise ValueError(f"invalid {model.__name__} in {path}: {exc}") from exc
