"""Package boundary and public API contract tests."""

from __future__ import annotations

import ast
from pathlib import Path

import suspension_multibody


def test_public_import_is_independent() -> None:
    assert suspension_multibody.__version__
    assert not any(name == "kinematics" for name in suspension_multibody.__dict__)


def test_business_source_does_not_import_legacy_package() -> None:
    source_root = Path(suspension_multibody.__file__).parent
    for path in source_root.rglob("*.py"):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(
                    alias.name != "kinematics"
                    and not alias.name.startswith("kinematics.")
                    for alias in node.names
                )
            if isinstance(node, ast.ImportFrom):
                assert node.module is None or not node.module.startswith("kinematics")
