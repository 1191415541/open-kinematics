from __future__ import annotations

import ast
import importlib
from pathlib import Path

import pytest

from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
from suspension_multibody.cases.kc_quasi_static.contract import (
    solver_settings_document as legacy_solver_settings_document,
)
from suspension_multibody.cases.vehicle_dynamic import _solver_block
from suspension_multibody.kernel import native as kernel_native
from suspension_multibody.kernel.solver import solver_settings_document

ROOT = Path(__file__).parents[4]
KERNEL_SOURCE = (
    ROOT
    / "packages"
    / "suspension_multibody"
    / "src"
    / "suspension_multibody"
    / "kernel"
)
AXLE_SOURCE = (
    ROOT
    / "packages"
    / "suspension_multibody"
    / "src"
    / "suspension_multibody"
    / "axle_dynamics"
)
PREPARATION_SOURCE = (
    ROOT
    / "packages"
    / "suspension_multibody"
    / "src"
    / "suspension_multibody"
    / "preparation"
)


def _imported_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    modules: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            modules.add(node.module)
    return modules


def test_kernel_sources_do_not_import_axle_dynamics() -> None:
    imported = {
        module
        for path in KERNEL_SOURCE.glob("*.py")
        for module in _imported_modules(path)
    }
    assert all("axle_dynamics" not in module for module in imported)


def test_shared_solver_serializer_matches_legacy_paths() -> None:
    settings = AxleSolverSettings(
        integrator="hht",
        rho_inf=0.7,
        hht_alpha=-0.2,
        adaptive_step=False,
        internal_step_s=0.0005,
    )

    expected = solver_settings_document(settings)
    assert expected == _solver_block(settings)
    assert expected == legacy_solver_settings_document(settings, times_s=(0.0, 0.1))
    assert len(expected) == 21


def test_the_kernel_runtime_owns_the_library_loader() -> None:
    """The strict target: exactly one ctypes loader, owned by the kernel package."""
    for name in (
        "load_library",
        "native_build_metadata",
        "NativeKernelUnavailableError",
        "_library_path",
        "_canonical_kernel_library",
        "_same_content",
        "_require_fresh_mirror",
        "_NATIVE_KERNEL_ABI_VERSION",
        "_NATIVE_VEHICLE_KERNEL_ABI_VERSION",
        "_NATIVE_CORE_ABI_VERSION",
    ):
        assert hasattr(kernel_native, name), name


def test_the_axle_package_ships_no_second_kernel_facade() -> None:
    """`axle_dynamics.native` was deleted, so importing it must fail."""
    assert not (AXLE_SOURCE / "native.py").exists()
    with pytest.raises(ModuleNotFoundError):
        importlib.import_module("suspension_multibody.axle_dynamics.native")


def test_the_axle_package_forwards_the_kernel_runtime_names() -> None:
    """The public axle names stay source-compatible by being the kernel's own."""
    from suspension_multibody.axle_dynamics import (
        NativeKernelUnavailableError,
        native_build_metadata,
    )

    assert NativeKernelUnavailableError is kernel_native.NativeKernelUnavailableError
    assert native_build_metadata is kernel_native.native_build_metadata


def test_preparation_authors_data_and_never_solves_or_decodes() -> None:
    """
    Subtask 06 acceptance 1: `preparation` converts author input, nothing else.

    The package turns a caller's model and case into the contract documents the
    kernel reads.  It must not reach for the kernel, must not solve, and must not
    decode a result -- those are the runner's, the kernel's and `results`' jobs.
    The check is on imports because that is what an accidental dependency looks
    like before it becomes a call.
    """
    forbidden = ("kernel", "results", "simulation.backend", "io")
    for path in sorted(PREPARATION_SOURCE.rglob("*.py")):
        offending = {
            module
            for module in _imported_modules(path)
            if any(module.endswith(f".{name}") or module == name for name in forbidden)
        }
        assert not offending, f"{path}: {sorted(offending)}"


def test_preparation_does_not_reach_the_solver_entry_points() -> None:
    """A prepared document is handed to the runner; preparation never runs it."""
    source = "\n".join(
        path.read_text(encoding="utf-8")
        for path in sorted(PREPARATION_SOURCE.rglob("*.py"))
    )

    for call in ("run_request", "run_contract", "suspension_kernel_run", "load_library"):
        assert call not in source, call
