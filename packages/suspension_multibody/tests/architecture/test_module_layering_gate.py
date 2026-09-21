"""
Gate tests for ``check_module_layering.py`` (subtask 02).

The gate has to prove three different things, and each one is tested in both
directions: the real tree is scanned down to the translation units, the target
module set is enforced while the legacy set is registered or refused, and every
rule can actually fail when a violation is injected into a fixture.
"""

from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import cast

import pytest

ROOT = Path(__file__).parents[4]
SCRIPT = ROOT / "packages" / "suspension_kernel" / "scripts" / "check_module_layering.py"
KERNEL_ROOT = ROOT / "packages" / "suspension_kernel"
BASELINE = KERNEL_ROOT / "layering_baseline.json"


def _load_gate():
    spec = importlib.util.spec_from_file_location("check_module_layering", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("failed to load check_module_layering")
    module = importlib.util.module_from_spec(spec)
    # dataclass() needs the module to be registered while it is executed.
    sys.modules["check_module_layering"] = module
    spec.loader.exec_module(module)
    return module


gate = _load_gate()


# --------------------------------------------------------------------------- #
# fixtures
# --------------------------------------------------------------------------- #


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _include_lines(items: list[str]) -> str:
    return "".join(f'#include "{item}"\n' for item in items)


def _header(symbol: str, includes: list[str]) -> str:
    return (
        "#pragma once\n\n"
        + _include_lines(includes)
        + f"\nnamespace axle_kernel {{\ndouble {symbol}(double value);\n}}\n"
    )


def _module(root: Path, name: str, *, includes: list[str] | None = None) -> None:
    """Write one module: ``functions.hpp`` plus a type header it includes."""
    symbol = name.replace("mb_", "")
    _write(root / "cpp" / "include" / name / "types.hpp", _type_header(symbol))
    _write(
        root / "cpp" / "include" / name / "functions.hpp",
        _header(symbol, includes or [f"{name}/types.hpp"]),
    )


def _type_header(symbol: str) -> str:
    return (
        "#pragma once\n\n"
        f"namespace axle_kernel {{\nstruct {symbol.title()} {{}};\n}}\n"
    )


def _unit(root: Path, name: str, *, includes: list[str], calls: str = "") -> None:
    """Write one translation unit that includes and calls other modules."""
    directory = name.replace("mb_", "")
    body = (
        f"namespace axle_kernel {{\n"
        f"double {name}_use(double value) {{ return {calls or 'value'}(value); }}\n"
        f"}}\n"
    )
    _write(
        root / "cpp" / "src" / directory / "unit.cpp",
        _include_lines([f"{name}/functions.hpp", *includes]) + "\n" + body,
    )


def _baseline_for(root: Path, **overrides: object) -> dict[str, object]:
    """Record a reviewed baseline from the fixture as it stands."""
    state = gate.scan(root)
    baseline = gate.baseline_state(state)
    present = set(state["modules"])
    header_edges = {tuple(edge) for edge in state["header_edges"]}
    baseline.update(
        {
            "header_only_edges": [list(edge) for edge in state["header_edges"]],
            "reviewed_new_edges": [
                {"edge": list(edge), "verdict": "reviewed fixture edge"}
                for edge in state["source_edges"]
                if tuple(edge) not in header_edges
            ],
            "legacy_modules": gate.present_legacy_modules(present),
            "kept_modules": sorted(
                module
                for module in present
                if module not in gate.LEGACY_MODULES
                and module not in gate.TARGET_MODULES
            ),
            "target_modules": list(gate.TARGET_MODULES),
            "migration_map": gate._serialisable_migration_map(),
            "notes": "fixture",
        }
    )
    baseline.update(overrides)
    return baseline


def _evaluate(root: Path, baseline: dict[str, object], **kwargs: object):
    return gate.evaluate(gate.scan(root), baseline, **kwargs)


def _kinds(findings) -> set[str]:
    return {finding.kind for finding in findings}


@pytest.fixture()
def two_module_root(tmp_path: Path) -> Path:
    """
    ``mb_beta`` depends on ``mb_alpha`` and ``mb_delta``.

    The header include, the translation-unit include and the unit's symbol
    reference are all present, and the unit-only edge ``mb_beta -> mb_delta``
    exists in the source scan alone.
    """
    root = tmp_path / "kernel"
    _module(root, "mb_alpha")
    _module(root, "mb_delta")
    _module(root, "mb_beta", includes=["mb_beta/types.hpp", "mb_alpha/types.hpp"])
    _unit(
        root,
        "mb_beta",
        includes=["mb_alpha/functions.hpp", "mb_delta/functions.hpp"],
        calls="alpha",
    )
    return root


# --------------------------------------------------------------------------- #
# the real tree is scanned down to the translation units
# --------------------------------------------------------------------------- #


def test_real_scan_covers_cpp_files_and_source_edges() -> None:
    state = gate.scan(KERNEL_ROOT)
    assert len(state["headers"]) >= 50
    assert len(state["cpp_files"]) >= 60
    assert state["source_edges"], "the scan produced no source edges"
    header_edges = {tuple(edge) for edge in state["header_edges"]}
    source_only = {tuple(edge) for edge in state["source_edges"]} - header_edges
    assert source_only, "source scanning added no information over the headers"
    # The ABI translation units include module headers the header-only scan
    # never saw, which is exactly why the gate had to be extended.
    assert ("abi", "mb_cases") in source_only
    # Subtask 03 moved `constraint_rows` out of the model accessor into
    # `mb_joint`, so `mb_model -> mb_joint` is gone; `mb_cases` reading the
    # model is the surviving source-only edge the assertion pins instead.
    assert ("mb_cases", "mb_model") in source_only
    evidence = {
        (record["module"], record["target"]): record["kinds"]
        for record in state["source_edge_evidence"]
    }
    assert "tu_include" in evidence[("mb_cases", "mb_model")]
    assert any("symbol_reference" in kinds for kinds in evidence.values()), (
        "no symbol reference evidence was produced"
    )


def test_real_baseline_records_both_edge_sets_with_evidence() -> None:
    """A new source edge cannot be waved through by re-reading the baseline."""
    baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
    assert baseline["header_edges"], "the baseline records no header edges"
    assert baseline["source_edges"], "the baseline records no source edges"
    assert baseline["header_only_edges"], "the previous edge list is not kept"
    recorded = {
        (record["module"], record["target"])
        for record in baseline["source_edge_evidence"]
    }
    assert recorded == {tuple(edge) for edge in baseline["source_edges"]}
    assert baseline["reviewed_new_edges"], "no edge was reviewed into the ledger"


def test_real_scan_covers_private_headers_under_cpp_src() -> None:
    """``cpp/src/**`` private headers must not escape the layering gate."""
    state = gate.scan(KERNEL_ROOT)
    headers = [str(header) for header in state["headers"]]
    private = [header for header in headers if header.startswith("cpp/src/")]
    assert private, "no cpp/src private header was scanned"
    # The private case header includes mb_base/mb_contract/mb_input headers, so
    # its module's edges are now visible to the header graph.
    case_edges = {
        tuple(edge) for edge in state["header_edges"] if edge[0] == "mb_cases"
    }
    assert any(
        edge[1] in {"mb_base", "mb_contract", "mb_input"} for edge in case_edges
    )


def test_extern_c_abi_declarations_enter_the_symbol_table() -> None:
    """``extern "C"`` ABI declarations must be scanned, not skipped as bodies."""
    declared = gate._declared_symbols(
        KERNEL_ROOT / "cpp" / "axle_dynamics" / "axle_kernel.hpp"
    )
    for symbol in (
        "suspension_kernel_run",
        "suspension_kernel_capabilities",
        "suspension_kernel_contract_version",
    ):
        assert symbol in declared, f"{symbol} was not declared in the scan"


def _run_gate(*arguments: str, env: dict[str, str] | None = None):
    environment = dict(os.environ)
    environment.update(env or {})
    return subprocess.run(
        [sys.executable, str(SCRIPT), *arguments],
        capture_output=True,
        check=False,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=environment,
        cwd=ROOT,
    )


def test_strict_gate_passes_on_the_current_tree() -> None:
    completed = _run_gate("--strict")
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "source edges" in completed.stdout
    assert "cpp translation units" in completed.stdout


def test_strict_final_gate_passes_on_the_current_tree() -> None:
    # Subtask 04 removed the last legacy modules (`mb_vehicle`, `mb_suspension`)
    # and broke the last module cycle, so the final gate is green: no legacy
    # module, no legacy path, no missing target module, no cycle, no mutual
    # edge.  The remaining blockers of the epic live in the Python layers
    # (05-08), not in the kernel layering.
    completed = _run_gate("--strict", "--final")
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert "legacy modules present     : 0" in completed.stdout
    assert "module cycles (SCC size>1) : 0" in completed.stdout
    assert "mutual (reverse) edges     : 0" in completed.stdout


def test_mode_cannot_be_switched_through_the_environment() -> None:
    """The migration exemption is a CLI flag, never an environment variable."""
    hostile = {
        "SUSPENSION_KERNEL_LAYERING": "final",
        "SUSPENSION_KERNEL_LAYERING_MODE": "final",
        "MB_LAYERING_MODE": "final",
        "MB_LAYERING_ALLOW_LEGACY": "1",
        "SUSPENSION_KERNEL_ALLOW_LEGACY": "1",
    }
    source = SCRIPT.read_text(encoding="utf-8")
    assert "os.environ" not in source
    assert "getenv" not in source
    tolerated = _run_gate("--strict", env=hostile)
    assert tolerated.returncode == 0, tolerated.stdout
    # Since subtask 04 the final gate is green on the current tree, so the
    # hostile environment cannot be told from the flag only by the exit code;
    # it must not flip the mode either way.
    refused = _run_gate("--strict", "--final", env=hostile)
    assert refused.returncode == 0, refused.stdout + refused.stderr


# --------------------------------------------------------------------------- #
# module set: target modules present, legacy modules registered or refused
# --------------------------------------------------------------------------- #


def test_target_module_set_detects_presence_and_absence() -> None:
    assert gate.missing_target_modules(gate.TARGET_MODULES) == []
    missing = gate.missing_target_modules({"mb_input"})
    assert set(missing) == set(gate.TARGET_MODULES) - {"mb_input"}
    assert missing, "the absence direction never fires"
    assert set(gate.TARGET_MODULES) == {
        "mb_numeric",
        "mb_dual",
        "mb_config",
        "mb_linear",
        "mb_joint",
        "mb_input",
        "mb_solve_static",
        "mb_solve_dynamic",
        "mb_element",
        "mb_assembly",
        "mb_force",
    }
    assert set(gate.LEGACY_MODULES) == {
        "mb_base",
        "mb_vehicle",
        "mb_suspension",
        "mb_integrator",
        "mb_static",
        "mb_linalg",
        "mb_constraint",
    }


def test_fixture_can_register_new_and_legacy_modules(tmp_path: Path) -> None:
    root = tmp_path / "kernel"
    for module in gate.TARGET_MODULES:
        _module(root, module)
    present = set(gate.scan(root)["modules"])
    assert gate.missing_target_modules(present) == []
    assert gate.present_legacy_modules(present) == []

    _module(root, "mb_base")
    present = set(gate.scan(root)["modules"])
    assert gate.present_legacy_modules(present) == ["mb_base"]


def test_migration_accepts_a_registered_legacy_module_and_final_refuses_it(
    tmp_path: Path,
) -> None:
    root = tmp_path / "kernel"
    for module in gate.TARGET_MODULES:
        _module(root, module)
    _module(root, "mb_base")
    baseline = _baseline_for(root, legacy_modules=["mb_base"])
    migration = _evaluate(root, baseline, mode="migration", strict=True)
    assert "legacy_module_present" not in _kinds(migration)
    assert "missing_target_module" not in _kinds(migration)

    final = _evaluate(root, baseline, mode="final", strict=True)
    assert "legacy_module_present" in _kinds(final)
    assert "legacy_path_present" in _kinds(final)


def test_target_modules_alone_satisfy_the_final_mode(tmp_path: Path) -> None:
    """The end state is reachable, so the gate is not a permanent failure."""
    root = tmp_path / "kernel"
    for module in gate.TARGET_MODULES:
        _module(root, module)
    baseline = _baseline_for(root)
    assert _evaluate(root, baseline, mode="final", strict=True) == []


def test_unregistered_legacy_module_is_a_finding(tmp_path: Path) -> None:
    root = tmp_path / "kernel"
    _module(root, "mb_vehicle")
    baseline = _baseline_for(root, legacy_modules=[])
    findings = _evaluate(root, baseline, strict=True)
    assert "unregistered_legacy_module" in _kinds(findings)


def test_unexplained_legacy_edge_is_a_finding(tmp_path: Path) -> None:
    root = tmp_path / "kernel"
    _module(root, "mb_vehicle")
    _module(root, "mb_base")
    _unit(root, "mb_vehicle", includes=["mb_base/functions.hpp"], calls="base")
    baseline = _baseline_for(root)
    assert baseline["source_edges"], "the fixture produced no source edge"
    # Drop the mapping that explains the legacy endpoint.
    baseline["migration_map"] = {}
    findings = _evaluate(root, baseline, strict=True)
    kinds = _kinds(findings)
    assert "unexplained_legacy_edge" in kinds
    assert "migration_map_drift" in kinds


# --------------------------------------------------------------------------- #
# injected violations: each rule must fail
# --------------------------------------------------------------------------- #


def test_injected_reverse_edge_fails(two_module_root: Path) -> None:
    baseline = _baseline_for(two_module_root)
    assert ("mb_beta", "mb_alpha") in {
        tuple(edge)
        for edge in cast(list[list[str]], baseline["header_edges"])
    }
    _write(
        two_module_root / "cpp" / "include" / "mb_alpha" / "functions.hpp",
        _header("alpha", ["mb_alpha/types.hpp", "mb_beta/functions.hpp"]),
    )
    findings = _evaluate(two_module_root, baseline, strict=True)
    kinds = _kinds(findings)
    assert "reverse_edge" in kinds
    assert "cycle" in kinds


def test_injected_cycle_fails(tmp_path: Path) -> None:
    root = tmp_path / "kernel"
    _module(root, "mb_alpha")
    _module(root, "mb_beta", includes=["mb_beta/types.hpp", "mb_alpha/functions.hpp"])
    _module(root, "mb_gamma", includes=["mb_gamma/types.hpp", "mb_beta/functions.hpp"])
    baseline = _baseline_for(root)
    _write(
        root / "cpp" / "include" / "mb_alpha" / "functions.hpp",
        _header("alpha", ["mb_alpha/types.hpp", "mb_gamma/functions.hpp"]),
    )
    findings = _evaluate(root, baseline, strict=True)
    kinds = _kinds(findings)
    assert "cycle" in kinds
    assert "new_header_edge" in kinds


def test_self_include_swap_with_same_count_fails(two_module_root: Path) -> None:
    """A swapped self-include must fail by content, not only by count."""
    baseline = _baseline_for(two_module_root)
    baseline["self_includes"] = ["cpp/include/mb_alpha/other.hpp"]
    _write(
        two_module_root / "cpp" / "include" / "mb_alpha" / "functions.hpp",
        '#pragma once\n\n#include "functions.hpp"\n',
    )
    findings = _evaluate(two_module_root, baseline, strict=True)
    assert "self_include" in _kinds(findings)


def test_injected_self_include_fails(two_module_root: Path) -> None:
    baseline = _baseline_for(two_module_root)
    _write(
        two_module_root / "cpp" / "include" / "mb_alpha" / "functions.hpp",
        '#pragma once\n\n#include "functions.hpp"\n',
    )
    findings = _evaluate(two_module_root, baseline, strict=True)
    assert "self_include" in _kinds(findings)


def test_injected_aggregate_include_fails(two_module_root: Path) -> None:
    header = two_module_root / "cpp" / "include" / "mb_alpha" / "functions.hpp"
    _write(
        two_module_root / "cpp" / "include" / "mb_beta" / "vector.hpp",
        "#pragma once\n\nnamespace axle_kernel { struct Vec3 {}; }\n",
    )
    _write(header, _header("alpha", ["mb_alpha/types.hpp", "mb_beta/vector.hpp"]))
    baseline = _baseline_for(two_module_root)
    assert baseline["aggregate_includes"] == []
    _write(header, _header("alpha", ["mb_alpha/types.hpp", "mb_beta/functions.hpp"]))
    findings = _evaluate(two_module_root, baseline, strict=True)
    assert "aggregate_include" in _kinds(findings)


def test_injected_unregistered_edge_fails(two_module_root: Path) -> None:
    baseline = _baseline_for(two_module_root)
    _module(
        two_module_root,
        "mb_gamma",
        includes=["mb_gamma/types.hpp", "mb_alpha/functions.hpp"],
    )
    findings = _evaluate(two_module_root, baseline, strict=True)
    kinds = _kinds(findings)
    assert "new_header_edge" in kinds
    assert "unreviewed_edge" in kinds
    assert "unregistered_module" in kinds


def test_reviewed_ledger_is_required_for_edges_outside_the_header_baseline(
    two_module_root: Path,
) -> None:
    baseline = _baseline_for(two_module_root)
    # The source edge beta -> alpha is real but is not part of the header-only
    # baseline: it must be in the reviewed ledger.
    assert baseline["source_edge_evidence"]
    baseline["reviewed_new_edges"] = []
    findings = _evaluate(two_module_root, baseline, strict=True)
    assert "unreviewed_edge" in _kinds(findings)


def test_missing_source_evidence_fails(two_module_root: Path) -> None:
    baseline = _baseline_for(two_module_root)
    assert baseline["source_edge_evidence"]
    baseline["source_edge_evidence"] = []
    findings = _evaluate(two_module_root, baseline, strict=True)
    assert "missing_source_evidence" in _kinds(findings)


def test_baseline_without_source_edges_cannot_be_compared(
    two_module_root: Path,
) -> None:
    baseline = _baseline_for(two_module_root)
    del baseline["source_edges"]
    baseline["version"] = 1
    findings = _evaluate(two_module_root, baseline, strict=True)
    assert "baseline_schema" in _kinds(findings)


def test_clean_fixture_passes_strictly(two_module_root: Path) -> None:
    """A fixture with a clean layering passes, so the gate is usable."""
    baseline = _baseline_for(two_module_root)
    assert _evaluate(two_module_root, baseline, strict=True) == []
