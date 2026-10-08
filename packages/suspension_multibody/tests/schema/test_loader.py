"""Versioned YAML/JSON loader tests."""

from pathlib import Path

import pytest
import yaml

from suspension_multibody.schema import load_case
from suspension_multibody.schema.model import AxleDeclaration


def test_loaders_reject_unknown_schema_version(tmp_path: Path) -> None:
    path = tmp_path / "case.yaml"
    path.write_text("schema_version: 2\nmode: K\n", encoding="utf-8")
    with pytest.raises(ValueError, match="unsupported schema_version"):
        load_case(path)


def test_a_model_document_reads_into_the_declaration_type(tmp_path: Path) -> None:
    """
    A v1 model document is still readable -- through its schema type.

    The two v1 *model loaders* were retired with the modelling route (G5/D1): a
    user authors an ``AssemblyDocument``, not an ``AxleDeclaration`` file.  What
    survives is the format itself, so the same document still reads into the
    same declaration, which is what this pins in place of the removed loader.
    """
    path = tmp_path / "model.yaml"
    path.write_text(
        "schema_version: 1\nhardpoints:\n  A: [1, -2, 3]\nmass:\n  sprung_mass: 1000\n",
        encoding="utf-8",
    )
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    assert AxleDeclaration.model_validate(document).hardpoints["A"].z == 3
