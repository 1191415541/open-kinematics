"""
Tests for the Python responsibility and deletion gate (subtask 02).

Every rule is exercised on a fixture in a temporary directory, in both
directions: the violation is detected and it fails the gate, while the clean
form of the same file passes.  The production tree is only ever read.
"""

from __future__ import annotations

import ast
import hashlib
import importlib.util
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).parents[4]
PACKAGE_ROOT = ROOT / "packages" / "suspension_multibody"
GATE_SCRIPT = Path(__file__).with_name("legacy_surface_gate.py")
REGISTRY = Path(__file__).with_name("legacy_surface_registry.json")


def _load_gate():
    spec = importlib.util.spec_from_file_location("legacy_surface_gate", GATE_SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("failed to load legacy_surface_gate")
    module = importlib.util.module_from_spec(spec)
    sys.modules["legacy_surface_gate"] = module
    spec.loader.exec_module(module)
    return module


gate = _load_gate()


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _package(tmp_path: Path, name: str = "pkg") -> Path:
    """Create a package root shaped like ``packages/suspension_multibody``."""
    root = tmp_path / name
    _write(root / "src" / "suspension_multibody" / "__init__.py", "")
    return root


def _module_path(root: Path, relative: str) -> Path:
    return root / "src" / "suspension_multibody" / relative


def _rules(findings) -> set[str]:
    return {finding.rule for finding in findings}


def _symbols(findings, rule: str) -> set[str]:
    return {finding.symbol for finding in findings if finding.rule == rule}


def _registry_file(tmp_path: Path, entries: list[dict[str, object]]) -> Path:
    path = tmp_path / "registry.json"
    path.write_text(json.dumps({"version": 1, "entry": entries}), encoding="utf-8")
    return path


def _run_gate(*arguments: str):
    return subprocess.run(
        [sys.executable, str(GATE_SCRIPT), *arguments],
        capture_output=True,
        check=False,
        text=True,
        encoding="utf-8",
        errors="replace",
        cwd=ROOT,
    )


# --------------------------------------------------------------------------- #
# the live tree
# --------------------------------------------------------------------------- #


def test_live_registry_is_exact_and_matches_every_finding() -> None:
    """Migration mode: every live finding is registered and none is stale."""
    findings = gate.scan_tree(PACKAGE_ROOT)
    registered = gate.load_registry(REGISTRY)
    live = [
        finding
        for finding in findings
        if finding.scope != "test" and finding.rule != "legacy_forwarding_shell"
    ]
    assert gate.evaluate(
        findings, mode=gate.MODE_MIGRATION, registered=registered
    ) == []
    assert gate.stale_registrations(findings, registered) == []
    assert live, "the live tree has no registered legacy import at all"
    keys = {(finding.path, finding.symbol) for finding in live}
    entries = {(str(entry["path"]), str(entry["symbol"])) for entry in registered}
    assert keys == entries


def test_registry_entries_name_their_owner_and_successor() -> None:
    registered = gate.load_registry(REGISTRY)
    assert registered
    for entry in registered:
        assert entry["owner"], entry
        assert entry["successor"], entry
        assert entry["rule"].startswith("legacy_module_")
        assert entry["symbol"] in gate.LEGACY_PACKAGES


def test_migration_mode_passes_and_final_mode_fails_on_the_live_tree() -> None:
    migration = _run_gate("--check", "--package-root", str(PACKAGE_ROOT))
    assert migration.returncode == 0, migration.stdout + migration.stderr
    final = _run_gate("--check", "--final", "--package-root", str(PACKAGE_ROOT))
    assert final.returncode == 1
    assert "legacy package still present" in final.stdout


def test_scanning_only_reads_the_production_tree() -> None:
    sources = sorted(
        (PACKAGE_ROOT / "src").rglob("*.py")
    )
    before = {
        path: hashlib.sha256(path.read_bytes()).hexdigest() for path in sources
    }
    gate.scan_tree(PACKAGE_ROOT)
    after = {
        path: hashlib.sha256(path.read_bytes()).hexdigest() for path in sources
    }
    assert before == after


# --------------------------------------------------------------------------- #
# legacy imports: static, relative, alias and dynamic
# --------------------------------------------------------------------------- #


def test_relative_alias_and_absolute_legacy_imports_are_detected(
    tmp_path: Path,
) -> None:
    root = _package(tmp_path)
    _write(
        _module_path(root, "service.py"),
        "from .core import rigid_body\n"
        "from . import elements as element_api\n"
        "from suspension_multibody.metrics import compute_axle_metrics\n"
        "from suspension_multibody import analysis\n\n"
        "def build(model):\n"
        "    return compute_axle_metrics(model)\n",
    )
    findings = gate.scan_tree(root)
    assert _rules(findings) == {"legacy_module_import"}
    assert _symbols(findings, "legacy_module_import") == {
        "core",
        "elements",
        "metrics",
        "analysis",
    }
    # Without a registration the gate fails; with one it passes.
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION) != []
    registered = [
        {
            "path": finding.path,
            "scope": finding.scope,
            "rule": finding.rule,
            "symbol": finding.symbol,
        }
        for finding in findings
    ]
    assert gate.evaluate(
        findings, mode=gate.MODE_MIGRATION, registered=registered
    ) == []
    assert gate.evaluate(findings, mode=gate.MODE_FINAL, registered=registered) != []


def test_dynamic_imports_are_detected(tmp_path: Path) -> None:
    root = _package(tmp_path)
    _write(
        _module_path(root, "loader.py"),
        "import importlib\n"
        "from importlib import import_module\n\n"
        "def load():\n"
        "    importlib.import_module('suspension_multibody.model.vehicle')\n"
        "    import_module('suspension_multibody.core')\n"
        "    return __import__('suspension_multibody.analysis.time_signals')\n",
    )
    findings = gate.scan_tree(root)
    assert _rules(findings) == {"legacy_module_dynamic_import"}
    assert _symbols(findings, "legacy_module_dynamic_import") == {
        "model",
        "core",
        "analysis",
    }
    assert gate.evaluate(findings, mode=gate.MODE_FINAL) != []


def test_dynamic_import_through_a_module_constant_is_detected(tmp_path: Path) -> None:
    """``import_module(_X)`` where ``_X`` names a retired package must fire."""
    root = _package(tmp_path)
    _write(
        _module_path(root, "loader.py"),
        "from importlib import import_module\n\n"
        "_LEGACY_MODULE = \"suspension_multibody.core\"\n"
        "_LIVE_MODULE = \"suspension_multibody.results.decoder\"\n\n"
        "def load():\n"
        "    return import_module(_LEGACY_MODULE)\n"
        "def load_live():\n"
        "    return import_module(_LIVE_MODULE)\n",
    )
    findings = gate.scan_tree(root)
    assert _rules(findings) == {"legacy_module_dynamic_import"}
    assert _symbols(findings, "legacy_module_dynamic_import") == {"core"}


def test_registration_occurrences_are_verified(tmp_path: Path) -> None:
    """A stale ``occurrences`` count must fail, not only a vanished key."""
    root = _package(tmp_path)
    _write(
        _module_path(root, "real.py"),
        "from .core import rigid_body\n\n\ndef use(body):\n    return body\n",
    )
    findings = [
        finding
        for finding in gate.scan_tree(root)
        if finding.rule == "legacy_module_import"
    ]
    assert len(findings) == 1
    registered = [
        {
            "path": findings[0].path,
            "scope": findings[0].scope,
            "rule": findings[0].rule,
            "symbol": findings[0].symbol,
            "occurrences": 3,
        }
    ]
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION, registered=registered) == []
    stale = gate.stale_registrations(findings, registered)
    assert stale, "an outdated occurrences count was not reported"
    assert "occurrences" in stale[0].symbol


def test_a_module_that_imports_nothing_retired_passes(tmp_path: Path) -> None:
    root = _package(tmp_path)
    _write(
        _module_path(root, "preparation/assembly/types.py"),
        "from dataclasses import dataclass\n\n"
        "@dataclass\nclass Joint:\n    name: str\n",
    )
    _write(
        _module_path(root, "results/decoder.py"),
        "from suspension_multibody.results.raw import decode_result\n",
    )
    assert gate.scan_tree(root) == []


def test_findings_inside_tests_are_tolerated_until_the_deletion() -> None:
    findings = [
        gate.SurfaceFinding(
            path="tests/architecture/test_x.py",
            scope="test",
            rule="legacy_module_import",
            symbol="core",
            line=1,
        )
    ]
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION) == []
    assert gate.evaluate(findings, mode=gate.MODE_FINAL) == findings


def test_stale_registration_is_reported(tmp_path: Path) -> None:
    root = _package(tmp_path)
    _write(_module_path(root, "clean.py"), "value = 1\n")
    registered = [
        {
            "path": "src/suspension_multibody/core/old.py",
            "scope": "production",
            "rule": "legacy_module_import",
            "symbol": "core",
        }
    ]
    findings = gate.scan_tree(root)
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION, registered=registered) == []
    assert [finding.path for finding in gate.stale_registrations(findings, registered)] == [
        "src/suspension_multibody/core/old.py"
    ]
    registry = _registry_file(tmp_path, registered)
    completed = _run_gate("--check", "--package-root", str(root), "--registry", str(registry))
    assert completed.returncode == 1
    assert "stale registry entry" in completed.stdout


# --------------------------------------------------------------------------- #
# report must not call native or solve
# --------------------------------------------------------------------------- #


def test_report_calling_native_or_solving_is_detected(tmp_path: Path) -> None:
    root = _package(tmp_path)
    _write(
        _module_path(root, "report/native_bypass.py"),
        "from suspension_multibody.kernel.solver import solver_settings_document\n"
        "from ..native import run as native_run\n"
        "from suspension_kernel import run_contract\n\n"
        "def build(model, case):\n"
        "    native_run(model)\n"
        "    return run_contract(model, case)\n",
    )
    findings = gate.scan_tree(root)
    rules = _rules(findings)
    assert "report_native_import" in rules
    assert "report_native_call" in rules
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION) != []
    assert gate.evaluate(findings, mode=gate.MODE_FINAL) != []


def test_report_reading_decoded_results_passes(tmp_path: Path) -> None:
    root = _package(tmp_path)
    _write(
        _module_path(root, "report/metrics/wheel_loads.py"),
        "from suspension_multibody.results.decoder import decode_result\n\n"
        "def wheel_loads(bundle):\n"
        "    return [row.force_z for row in decode_result(bundle)]\n",
    )
    assert gate.scan_tree(root) == []


def test_native_tokens_outside_the_report_scope_are_not_findings(tmp_path: Path) -> None:
    root = _package(tmp_path)
    _write(
        _module_path(root, "simulation/backend.py"),
        "from suspension_multibody.kernel import run_contract\n\n"
        "def run(model, case):\n"
        "    return run_contract(model, case)\n",
    )
    assert gate.scan_tree(root) == []


def test_report_importing_or_running_preparation_is_detected(tmp_path: Path) -> None:
    """The report publishes what came back; it must not author an input."""
    root = _package(tmp_path)
    _write(
        _module_path(root, "report/authors_inputs.py"),
        "from suspension_multibody.preparation.signals import time_grid\n"
        "from ..preparation.assembly import build_front_axle\n\n"
        "def replay(model, case):\n"
        "    return build_front_axle(model, 'K'), time_grid(case)\n",
    )
    findings = gate.scan_tree(root)
    rules = _rules(findings)
    assert "report_preparation_import" in rules
    assert "report_preparation_call" in rules
    imported = _symbols(findings, "report_preparation_import")
    assert "suspension_multibody.preparation.signals" in imported
    assert "suspension_multibody.preparation.assembly" in imported
    called = _symbols(findings, "report_preparation_call")
    assert {"build_front_axle", "time_grid"} <= called
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION) != []
    assert gate.evaluate(findings, mode=gate.MODE_FINAL) != []


def test_report_recomputing_a_constitutive_law_is_detected(tmp_path: Path) -> None:
    """A force law the kernel already answered must not be rebuilt here."""
    root = _package(tmp_path)
    _write(
        _module_path(root, "report/rebuilds_the_law.py"),
        "from suspension_multibody.results import decode_element_wrench\n\n"
        "_LAWS = {'spring': spring_force}\n\n"
        "def rebuild(record):\n"
        "    wrench = decode_element_wrench(record)\n"
        "    return spring_force(wrench, 1000.0)\n",
    )
    findings = gate.scan_tree(root)
    assert _rules(findings) == {"report_constitutive_call"}
    assert _symbols(findings, "report_constitutive_call") == {"spring_force"}
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION) != []
    assert gate.evaluate(findings, mode=gate.MODE_FINAL) != []


def test_the_live_report_tree_stays_inside_its_boundary() -> None:
    """
    The real ``report`` package passes every rule, and the scan sees it.

    The fixture tests above prove each rule can fire; this one proves the live
    tree is clean in both directions -- no boundary finding at all, and, read
    independently of the scanner's rules, no import of native/kernel/solver or
    preparation code in any module under ``report/``.
    """
    report_root = PACKAGE_ROOT / "src" / "suspension_multibody" / "report"
    modules = sorted(report_root.rglob("*.py"))
    assert report_root.is_dir()
    assert len(modules) >= 5, (
        "the report tree is too small for the assertion to mean anything"
    )

    findings = gate.scan_tree(PACKAGE_ROOT)
    boundary = [finding for finding in findings if gate.is_report_scope(finding.path)]
    assert boundary == [], [finding.describe() for finding in boundary]
    assert not {
        "report_native_import",
        "report_native_call",
        "report_preparation_import",
        "report_preparation_call",
        "report_constitutive_call",
    } & _rules(findings)

    imported: set[str] = set()
    for path in modules:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                resolved = gate.resolve_import(path, node.module, node.level)
                if resolved:
                    imported.add(resolved)
    assert imported, "the report tree imports nothing at all"
    assert [name for name in sorted(imported) if gate._native_of(name)] == []
    assert [name for name in sorted(imported) if gate._preparation_of(name)] == []


# --------------------------------------------------------------------------- #
# old paths must not survive as forwarding shells
# --------------------------------------------------------------------------- #


def test_forwarding_shell_is_detected(tmp_path: Path) -> None:
    root = _package(tmp_path)
    shell = _module_path(root, "core_compat.py")
    _write(shell, "from suspension_multibody.core import rigid_body\n\n__all__ = ['rigid_body']\n")
    assert gate.is_forwarding_shell(shell) is True
    findings = gate.scan_tree(root)
    assert "legacy_forwarding_shell" in _rules(findings)
    assert gate.evaluate(findings, mode=gate.MODE_MIGRATION) != []


def test_package_facade_and_real_module_are_not_shells(tmp_path: Path) -> None:
    root = _package(tmp_path)
    facade = _module_path(root, "io/__init__.py")
    _write(facade, "from .artifacts import write_artifact\n")
    real = _module_path(root, "io/artifacts.py")
    _write(real, "def write_artifact(path):\n    return path\n")
    assert gate.is_forwarding_shell(facade) is False
    assert gate.is_forwarding_shell(real) is False
    assert gate.scan_tree(root) == []


def test_legacy_package_directories_are_refused_in_the_final_mode(
    tmp_path: Path,
) -> None:
    root = _package(tmp_path)
    _write(_module_path(root, "core/__init__.py"), "")
    _write(_module_path(root, "metrics/__init__.py"), "")
    present = gate.present_legacy_packages(root)
    assert [Path(path).name for path in present] == ["core", "metrics"]
    completed = _run_gate("--check", "--final", "--package-root", str(root))
    assert completed.returncode == 1
    assert "legacy package still present" in completed.stdout
    clean = _package(tmp_path, "clean")
    _write(_module_path(clean, "report/__init__.py"), "")
    assert gate.present_legacy_packages(clean) == []
    assert _run_gate("--check", "--final", "--package-root", str(clean)).returncode == 0


# --------------------------------------------------------------------------- #
# the scanner reads text, it never imports what it scans
# --------------------------------------------------------------------------- #


def test_scanner_does_not_execute_the_scanned_module(tmp_path: Path) -> None:
    root = _package(tmp_path)
    _write(
        _module_path(root, "explodes.py"),
        "raise RuntimeError('the scanner imported me')\n",
    )
    _write(
        _module_path(root, "service.py"),
        "import os\n\nraise SystemExit('imported at scan time')\n",
    )
    findings = gate.scan_tree(root)
    assert findings == []


def test_registry_scope_and_rule_values_are_known(tmp_path: Path) -> None:
    registered = gate.load_registry(REGISTRY)
    assert {entry["scope"] for entry in registered} <= {"production", "script"}
    assert all(
        entry["rule"] in {"legacy_module_import", "legacy_module_dynamic_import"}
        for entry in registered
    )
    assert gate.load_registry(_registry_file(tmp_path, [])) == []
