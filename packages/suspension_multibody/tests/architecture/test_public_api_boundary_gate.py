from __future__ import annotations

import ast
import tempfile
import tomllib
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).parents[4]
SOURCE_ROOT = ROOT / "packages" / "suspension_multibody" / "src" / "suspension_multibody"
SCRIPT_ROOT = ROOT / "packages" / "suspension_multibody" / "scripts"
TEST_ROOT = ROOT / "packages" / "suspension_multibody" / "tests"
ALLOWLIST_PATH = (
    ROOT
    / ".codex-tasks"
    / "20260919-public-api-simulation-cutover"
    / "tasks"
    / "01-boundary-inventory"
    / "LEGACY_ALLOWLIST.toml"
)


@dataclass(frozen=True)
class BoundaryFinding:
    path: str
    scope: str
    rule: str
    symbol: str
    line: int

    @property
    def key(self) -> tuple[str, str, str, str, int]:
        return self.path, self.scope, self.rule, self.symbol, self.line


def _scope_for_path(path: Path) -> str:
    path_text = path.as_posix()
    if "/tests/" in path_text:
        return "test"
    if "/scripts/" in path_text:
        return "script"
    return "production"


def _relative_path(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return f"<external>/{path.name}"


def _is_case_contract_name(name: str) -> bool:
    return name.startswith("run_") and name.endswith("_contract") and name != "run_contract"


def _is_case_module(module: str | None) -> bool:
    return bool(
        module
        and (
            module == "cases"
            or module.startswith("cases.")
            or ".cases." in module
            or module.endswith(".cases")
        )
    )


def _is_native_facade_import(module: str | None, imported_name: str) -> bool:
    if imported_name != "native":
        return False
    return bool(module and (module == "axle_dynamics" or module.endswith(".axle_dynamics")))


def _is_backend_owner(path: Path, scope: str, function_scope: str) -> bool:
    return (
        scope == "production"
        and path.as_posix().endswith("/simulation/backend.py")
        and function_scope == "run"
    )


def _is_result_decoder_owner(path: Path) -> bool:
    path_text = path.as_posix()
    return path_text.endswith("/results/raw.py") or path_text.endswith(
        "/results/decoder.py"
    )


def _is_history_compat_owner(path: Path) -> bool:
    """Identify the historical-read boundary owners."""
    path_text = path.as_posix()
    return (
        path_text.endswith("/schema/loader.py")
        or path_text.endswith("/schema/__init__.py")
        or path_text.endswith("/adams/time_domain.py")
    )


def _scan_file(path: Path, *, test_native_only: bool = False) -> list[BoundaryFinding]:
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source)
    scope = _scope_for_path(path)
    relative = _relative_path(path)
    findings: list[BoundaryFinding] = []
    stack: list[str] = []

    def add(rule: str, symbol: str, node: ast.AST) -> None:
        findings.append(
            BoundaryFinding(
                path=relative,
                scope=scope,
                rule=rule,
                symbol=symbol,
                line=int(getattr(node, "lineno", 0)),
            )
        )

    def visit(node: ast.AST) -> None:
        entered = isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
        if entered:
            stack.append(node.name)  # type: ignore[union-attr]
        function_scope = stack[-1] if stack else "<module>"

        if isinstance(node, ast.ImportFrom):
            module = node.module
            for alias in node.names:
                imported_name = alias.name
                bound_name = alias.asname or imported_name.split(".")[-1]
                if _is_native_facade_import(module, imported_name):
                    if test_native_only and scope == "test":
                        add("axle_native_facade_import", imported_name, node)
                    elif not test_native_only:
                        add("axle_native_facade_import", imported_name, node)
                if test_native_only:
                    continue
                if bound_name == "run_contract" and module and module.endswith(".kernel"):
                    if not _is_backend_owner(path, scope, function_scope):
                        add("direct_kernel_run_contract", bound_name, node)
                if _is_case_contract_name(imported_name) and _is_case_module(module):
                    if not relative.startswith(
                        "packages/suspension_multibody/src/suspension_multibody/cases/"
                    ):
                        add("direct_case_contract_import", imported_name, node)
                if bound_name == "decode_contract_run" and not _is_result_decoder_owner(path):
                    add("direct_raw_decoder", bound_name, node)
                if bound_name in {
                    "write_bundle",
                    "write_dynamic_bundle",
                    "write_axle_dynamics_artifact",
                    "write_vehicle_dynamics_artifact",
                }:
                    add("legacy_artifact_writer", bound_name, node)
                if bound_name == "DynamicResultBundle" and not _is_history_compat_owner(path):
                    add("dynamic_bundle_reference", bound_name, node)

        elif isinstance(node, ast.Import):
            if not test_native_only:
                for alias in node.names:
                    imported_name = alias.name.split(".")[-1]
                    if imported_name == "run_contract" and alias.name.endswith(
                        ".kernel.run_contract"
                    ):
                        if not _is_backend_owner(path, scope, function_scope):
                            add("direct_kernel_run_contract", imported_name, node)

        elif isinstance(node, ast.Call):
            function = node.func
            called_name = (
                function.id
                if isinstance(function, ast.Name)
                else function.attr
                if isinstance(function, ast.Attribute)
                else ""
            )
            if not test_native_only:
                if called_name == "run_contract":
                    if not _is_backend_owner(path, scope, function_scope):
                        add("direct_kernel_run_contract", called_name, node)
                if _is_case_contract_name(called_name) and not relative.startswith(
                    "packages/suspension_multibody/src/suspension_multibody/cases/"
                ):
                    add("direct_case_contract_call", called_name, node)
                if called_name == "decode_contract_run" and not _is_result_decoder_owner(path):
                    add("direct_raw_decoder", called_name, node)
                if called_name in {
                    "write_bundle",
                    "write_dynamic_bundle",
                    "write_axle_dynamics_artifact",
                    "write_vehicle_dynamics_artifact",
                }:
                    add("legacy_artifact_writer", called_name, node)
                if called_name == "DynamicResultBundle" and not _is_history_compat_owner(path):
                    add("dynamic_bundle_creation", called_name, node)

        for child in ast.iter_child_nodes(node):
            visit(child)
        if entered:
            stack.pop()

    visit(tree)
    return findings


def _scan_repository() -> list[BoundaryFinding]:
    findings: list[BoundaryFinding] = []
    for root in (SOURCE_ROOT, SCRIPT_ROOT):
        for path in sorted(root.rglob("*.py")):
            findings.extend(_scan_file(path))
    for path in sorted(TEST_ROOT.rglob("*.py")):
        findings.extend(_scan_file(path, test_native_only=True))
    return findings


def _allowlist_records() -> list[dict[str, object]]:
    data = tomllib.loads(ALLOWLIST_PATH.read_text(encoding="utf-8"))
    return [dict(entry) for entry in data.get("entry", [])]


def _allowlist_keys() -> set[tuple[str, str, str, str, int]]:
    return {
        (
            str(entry["path"]),
            str(entry["scope"]),
            str(entry["rule"]),
            str(entry["symbol"]),
            int(entry["line"]),
        )
        for entry in _allowlist_records()
    }


def test_allowlist_is_exact_and_strict_scoped() -> None:
    data = tomllib.loads(ALLOWLIST_PATH.read_text(encoding="utf-8"))
    entries = _allowlist_records()
    required = {
        "path",
        "scope",
        "rule",
        "symbol",
        "line",
        "reason",
        "owner_task",
        "remove_when",
        "validation",
    }
    assert data["version"] == 2
    assert data["mode"] == "strict"
    assert all(required <= entry.keys() for entry in entries)
    assert all(entry["scope"] in {"production", "script", "test"} for entry in entries)
    assert all(
        not any(token in str(entry[field]) for token in ("*", "..."))
        for entry in entries
        for field in ("path", "symbol")
    )
    # Strict mode is what makes an empty allowlist meaningful: the scan finds
    # nothing and the file lists nothing, so a new bypass cannot hide behind a
    # stale entry.
    assert len(entries) == len(_allowlist_keys()) == len(_scan_repository())

def test_boundary_gate_matches_known_baseline() -> None:
    findings = _scan_repository()
    allowed = _allowlist_keys()
    unexpected = sorted(
        (finding for finding in findings if finding.key not in allowed),
        key=lambda finding: finding.key,
    )
    assert not unexpected, "unallowlisted architecture bypasses: " + "; ".join(
        f"{item.path}:{item.line} [{item.scope}/{item.rule}/{item.symbol}]"
        for item in unexpected
    )


def test_boundary_gate_rejects_new_direct_native_bypass() -> None:
    with tempfile.TemporaryDirectory() as temporary_directory:
        fixture = Path(temporary_directory) / "fixture.py"
        fixture.write_text(
            "from suspension_multibody.kernel import run_contract\n"
            "\n"
            "def bypass(model, case):\n"
            "    return run_contract(model, case)\n",
            encoding="utf-8",
        )
        findings = _scan_file(fixture)
    assert any(finding.rule == "direct_kernel_run_contract" for finding in findings)


def test_backend_is_the_only_direct_kernel_submission_owner() -> None:
    backend_path = SOURCE_ROOT / "simulation" / "backend.py"
    backend_tree = ast.parse(backend_path.read_text(encoding="utf-8"))
    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "run_contract"
        for node in ast.walk(backend_tree)
    )

    allowed = _allowlist_keys()
    non_backend_calls = [
        finding
        for finding in _scan_repository()
        if finding.rule == "direct_kernel_run_contract"
        and finding.scope in {"production", "script"}
        and not finding.path.endswith("/simulation/backend.py")
    ]
    assert all(finding.key in allowed for finding in non_backend_calls)
    assert all(finding.line > 0 for finding in non_backend_calls)
