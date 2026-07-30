"""Executable package dependency-boundary checks."""

from __future__ import annotations

import ast
import json
import subprocess
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).parents[4]
PACKAGE_SOURCES = {
    "suspension_contracts": ROOT
    / "packages"
    / "suspension_contracts"
    / "src"
    / "suspension_contracts",
    "suspension_kinematics": ROOT
    / "packages"
    / "suspension_kinematics"
    / "src"
    / "suspension_kinematics",
    "suspension_multibody": ROOT
    / "packages"
    / "suspension_multibody"
    / "src"
    / "suspension_multibody",
}
FORBIDDEN_IMPORTS = {
    "suspension_contracts": {"suspension_kinematics", "suspension_multibody"},
    "suspension_kinematics": {"suspension_multibody"},
    "suspension_multibody": {"suspension_kinematics"},
}


def _import_roots(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    roots: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            roots.update(alias.name.split(".", maxsplit=1)[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            roots.add(node.module.split(".", maxsplit=1)[0])
    return roots


def _dependencies(package: str) -> list[str]:
    pyproject = ROOT / "packages" / package / "pyproject.toml"
    return tomllib.loads(pyproject.read_text(encoding="utf-8"))["project"][
        "dependencies"
    ]


def test_contract_is_the_only_shared_solver_dependency() -> None:
    kinematics_dependencies = _dependencies("suspension_kinematics")
    multibody_dependencies = _dependencies("suspension_multibody")
    contract_dependency = "suspension-contracts>=0.1.0,<0.2.0"

    assert contract_dependency in kinematics_dependencies
    assert contract_dependency in multibody_dependencies
    assert not any(
        dependency.startswith("suspension-multibody")
        for dependency in kinematics_dependencies
    )
    assert not any(
        dependency.startswith("suspension-kinematics")
        for dependency in multibody_dependencies
    )


def test_product_sources_do_not_import_peer_products() -> None:
    for package, source_root in PACKAGE_SOURCES.items():
        forbidden = FORBIDDEN_IMPORTS[package]
        for path in source_root.rglob("*.py"):
            assert not (_import_roots(path) & forbidden), path


def test_contract_package_has_no_runtime_dependencies() -> None:
    assert _dependencies("suspension_contracts") == []


def test_default_imports_do_not_load_adams() -> None:
    script = """
import importlib
import json
import sys

for package in (
    "suspension_contracts",
    "suspension_kinematics",
    "suspension_multibody",
):
    importlib.import_module(package)

print(json.dumps(sorted(
    name
    for name in sys.modules
    if name == "suspension_multibody.adams"
    or name.startswith("suspension_multibody.adams.")
)))
"""
    completed = subprocess.run(
        [sys.executable, "-c", script],
        cwd=ROOT,
        capture_output=True,
        check=False,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout) == []
