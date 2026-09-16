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
    "suspension_kernel": ROOT
    / "packages"
    / "suspension_kernel"
    / "src"
    / "suspension_kernel",
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
#: Every product package that must not be imported by the package named in the
#: key.  `suspension_kernel` is deliberately in every list: it is the shared
#: generic core, so a dependency from it back into any product would invert the
#: layering the epic is built on.
FORBIDDEN_IMPORTS = {
    "suspension_contracts": {
        "suspension_kernel",
        "suspension_kinematics",
        "suspension_multibody",
    },
    "suspension_kernel": {
        "suspension_contracts",
        "suspension_kinematics",
        "suspension_multibody",
    },
    "suspension_kinematics": {"suspension_kernel", "suspension_multibody"},
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
    """
    Exactly one product may depend on the contract package, and no peers.

    The original assertion only checked a handful of negative cases, so it
    claimed a uniqueness property it never tested.  This now computes the
    actual set of inter-package dependencies and asserts its shape.
    """
    packages = (
        "suspension_contracts",
        "suspension_kernel",
        "suspension_kinematics",
        "suspension_multibody",
    )
    distribution_names = {
        "suspension_contracts": "suspension-contracts",
        "suspension_kernel": "suspension-kernel",
        "suspension_kinematics": "suspension-kinematics",
        "suspension_multibody": "suspension-multibody",
    }
    dependency_sets = {
        package: {
            _distribution_name(dependency)
            for dependency in _dependencies(package)
            if _distribution_name(dependency) in distribution_names.values()
        }
        for package in packages
    }

    # The generic kernel depends on no other package in the workspace.
    assert dependency_sets["suspension_kernel"] == set()
    # The contract package depends on no product either.
    assert dependency_sets["suspension_contracts"] == set()
    # Products may take the contract and the kernel, but never each other.
    assert "suspension-multibody" not in dependency_sets["suspension_kinematics"]
    assert "suspension-kinematics" not in dependency_sets["suspension_multibody"]
    # The axle product must reach the kernel: it loads the shared library
    # through the kernel binding rather than owning the build itself.
    assert "suspension-kernel" in dependency_sets["suspension_multibody"]
    # Uniqueness, stated as a set equality rather than as spot checks: the only
    # thing both products depend on is the contract package.  The kernel is
    # consumed by the axle product alone -- it hosts the multibody solver, and
    # the kinematics product has no C++ core.
    shared = (
        dependency_sets["suspension_kinematics"]
        & dependency_sets["suspension_multibody"]
    )
    assert shared == {"suspension-contracts"}
    assert "suspension-kernel" not in dependency_sets["suspension_kinematics"]
    assert "suspension-contracts" in dependency_sets["suspension_kinematics"]
    assert "suspension-contracts" in dependency_sets["suspension_multibody"]


def _distribution_name(requirement: str) -> str:
    """Reduce a PEP 508 requirement string to its distribution name."""
    return requirement.split(">=")[0].split("==")[0].split("<")[0].strip()


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
        # Decode explicitly: this repository's path is non-ASCII, and a locale
        # codec raises on bytes the child prints.
        encoding="utf-8",
        errors="replace",
    )

    assert completed.returncode == 0, completed.stderr
    assert json.loads(completed.stdout) == []
