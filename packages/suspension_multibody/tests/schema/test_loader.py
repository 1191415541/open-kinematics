"""Versioned YAML/JSON loader tests."""

from pathlib import Path

import pytest

from suspension_multibody.schema import load_case, load_model


def test_loaders_reject_unknown_schema_version(tmp_path: Path) -> None:
    path = tmp_path / "case.yaml"
    path.write_text("schema_version: 2\nmode: K\n", encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported schema_version"):
        load_case(path)


def test_load_model_yaml(tmp_path: Path) -> None:
    path = tmp_path / "model.yaml"
    path.write_text(
        "schema_version: 1\nhardpoints:\n  A: [1, -2, 3]\nmass:\n  sprung_mass: 1000\n",
        encoding="utf-8",
    )
    assert load_model(path).hardpoints["A"].z == 3
