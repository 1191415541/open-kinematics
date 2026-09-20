"""Versioned JSON/YAML schema loading."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, TypeVar

import yaml
from pydantic import BaseModel, ValidationError

from .case import CaseSpec
from .dynamic import DynamicCaseSpec, DynamicResultBundle
from .model import FrontAxleModel
from .vehicle import VehicleDynamicCase, VehicleModel

T = TypeVar("T", bound=BaseModel)


def _read(
    path: str | Path, *, version_path: tuple[str, ...] = ("schema_version",)
) -> Any:
    """
    Read one schema file and check the version found at ``version_path``.

    Every v1 input document carries ``schema_version`` at the document root.
    The historical ``DynamicResultBundle`` instead keeps its version inside
    ``manifest``, so the result loader points at that nested key rather than
    accepting a second root-level version field: the strict models keep
    forbidding unknown keys.
    """
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
    version = _version_at(data, version_path)
    if version != 1:
        raise ValueError(f"unsupported schema_version {version!r}; expected 1")
    return data


def _version_at(data: dict[str, Any], version_path: tuple[str, ...]) -> Any:
    current: Any = data
    for key in version_path:
        if not isinstance(current, dict):
            return None
        current = current.get(key)
    return current


def load_model(path: str | Path) -> FrontAxleModel:
    """Load and validate a model YAML/JSON file."""
    return _validate(_read(path), FrontAxleModel, path)


def load_case(path: str | Path) -> CaseSpec:
    """Load and validate a case YAML/JSON file."""
    return _validate(_read(path), CaseSpec, path)


def load_dynamic_case(path: str | Path) -> DynamicCaseSpec:
    """Load and validate a dynamic case YAML/JSON file."""
    return _validate(_read(path), DynamicCaseSpec, path)


def load_vehicle_model(path: str | Path) -> VehicleModel:
    """加载并校验整车模型 YAML/JSON 文件."""
    return _validate(_read(path), VehicleModel, path)


def load_vehicle_dynamic_case(path: str | Path) -> VehicleDynamicCase:
    """加载并校验整车动态算例 YAML/JSON 文件."""
    return _validate(_read(path), VehicleDynamicCase, path)


def load_dynamic_result(path: str | Path) -> DynamicResultBundle:
    """
    Load and validate a dynamic result JSON file.

    The bundle keeps its version in ``manifest.schema_version``; the document
    root still forbids a second ``schema_version`` key.
    """
    return _validate(
        _read(path, version_path=("manifest", "schema_version")),
        DynamicResultBundle,
        path,
    )


def _validate(data: Any, model: type[T], path: str | Path) -> T:
    try:
        return model.model_validate(data)
    except ValidationError as exc:
        raise ValueError(f"invalid {model.__name__} in {path}: {exc}") from exc
