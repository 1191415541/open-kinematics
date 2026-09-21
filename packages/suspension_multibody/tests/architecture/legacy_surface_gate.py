"""
Python responsibility and deletion gate for ``suspension_multibody``.

Subtask 02 of the architecture-deviation epic needs the *Python* half of the
gate before 06-08 move anything: the retired packages must not be imported by
live code, ``report`` must not call native or solve, and an old path must not be
kept alive by a forwarding shell that only re-exports.

The scanner is AST based and deliberately never imports the scanned module: it
reads and parses text only, so it can run on the production tree, on a script,
or on a fixture in a temporary directory.

Rules
-----

``legacy_module_import``
    A file outside the retired packages imports one of ``core``, ``elements``,
    ``model``, ``analysis``, ``metrics`` -- absolutely, relatively, or through an
    alias.
``legacy_module_dynamic_import``
    The same, reached through ``importlib.import_module`` or ``__import__``.
``report_native_import`` / ``report_native_call``
    Inside ``report``: importing native/kernel/solver code, or calling a solve
    or a native entry point.  The report consumes decoded results; it does not
    produce them.
``legacy_forwarding_shell``
    A non-package module whose whole body is imports and re-exports.  A package
    ``__init__.py`` is a facade, not a shell, and is not reported.

Modes
-----

``MODE_MIGRATION``
    Registered legacy imports are tolerated while their owners migrate, but
    every live finding must be registered: an unregistered finding fails, and a
    registration without a finding fails as stale, so the registry can only
    shrink.  Findings in tests are reported, not enforced, because 08 removes
    them together with the packages.
``MODE_FINAL``
    Nothing is tolerated: a retired package that still exists, any import of it
    in any scope, a forwarding shell, and a native call from ``report`` all
    fail.

The mode is an explicit CLI flag; there is no environment override.

    python legacy_surface_gate.py --check              # migration mode
    python legacy_surface_gate.py --check --final      # end state
"""

from __future__ import annotations

import argparse
import ast
import json
from dataclasses import dataclass
from pathlib import Path

PACKAGE = "suspension_multibody"
LEGACY_PACKAGES = ("core", "elements", "model", "analysis", "metrics")
REPORT_DIRECTORY = "report"
#: A name that means native/solver code: the report boundary must not cross it.
NATIVE_TOKENS = ("native", "kernel", "solver", "axle_dynamics")
SOLVE_NAMES = (
    "run_contract",
    "solve",
    "solve_static",
    "solve_dynamic",
    "run_solver",
)

MODE_MIGRATION = "migration"
MODE_FINAL = "final"


@dataclass(frozen=True)
class SurfaceFinding:
    """One Python boundary violation, with the key a registry entry matches."""

    path: str
    scope: str
    rule: str
    symbol: str
    line: int

    @property
    def key(self) -> tuple[str, str, str, str]:
        return self.path, self.scope, self.rule, self.symbol

    def describe(self) -> str:
        return f"{self.path}:{self.line} [{self.scope}/{self.rule}/{self.symbol}]"


# --------------------------------------------------------------------------- #
# scanning
# --------------------------------------------------------------------------- #


def _relative(path: Path, root: Path) -> str:
    try:
        return path.relative_to(root).as_posix()
    except ValueError:
        return f"<external>/{path.name}"


def scope_of(relative: str) -> str:
    """Classify a path as production, script or test."""
    if f"/{relative}".startswith("/tests/"):
        return "test"
    if f"/{relative}".startswith("/scripts/"):
        return "script"
    return "production"


def module_name(path: Path) -> str | None:
    """Return the dotted module name of a file inside the package, if known."""
    parts = path.with_suffix("").parts
    if PACKAGE not in parts:
        return None
    # The layout is ``.../src/<package>/...``; the last occurrence of the
    # package name is the source root, the earlier one is only a directory.
    index = len(parts) - 1 - parts[::-1].index(PACKAGE)
    name = ".".join(parts[index:])
    return name.removesuffix(".__init__")


def is_report_scope(relative: str) -> bool:
    """Report whether a path belongs to the report package."""
    return f"/{REPORT_DIRECTORY}/" in f"/{relative}"


def _legacy_of(module: str | None) -> str | None:
    """Return the retired package a dotted module resolves to, if any."""
    if not module:
        return None
    parts = module.split(".")
    if parts[0] != PACKAGE or len(parts) < 2:
        return None
    return parts[1] if parts[1] in LEGACY_PACKAGES else None

def own_legacy_package(relative: str) -> str | None:
    """Return the retired package a file itself belongs to, if any."""
    parts = relative.split("/")
    if PACKAGE not in parts:
        return None
    index = len(parts) - 1 - parts[::-1].index(PACKAGE)
    rest = parts[index + 1 :]
    return rest[0] if rest and rest[0] in LEGACY_PACKAGES else None


def _native_of(module: str | None) -> bool:
    if not module:
        return False
    parts = module.split(".")
    if parts[0] == "suspension_kernel":
        return True
    return any(token in parts for token in NATIVE_TOKENS)


def resolve_import(path: Path, module: str | None, level: int) -> str | None:
    """Resolve an import to a dotted module, following relative levels."""
    if not level:
        return module
    owner = module_name(path)
    if owner is None:
        return module
    parts = owner.split(".")
    # ``level=1`` names the package that contains the file: a package
    # ``__init__.py`` *is* that package, a plain module is one level below it.
    package_parts = parts if path.name == "__init__.py" else parts[:-1]
    keep = package_parts[: len(package_parts) - (level - 1)]
    if module:
        keep = [*keep, *module.split(".")]
    return ".".join(keep)


def _called_name(function: ast.expr) -> str:
    if isinstance(function, ast.Name):
        return function.id
    if isinstance(function, ast.Attribute):
        return function.attr
    return ""


def _call_receiver(function: ast.expr) -> str:
    """Return the leftmost name of an attribute chain, e.g. ``native``."""
    node: ast.expr = function
    while isinstance(node, ast.Attribute):
        node = node.value
    return node.id if isinstance(node, ast.Name) else ""


def _literal_import(
    arguments: list[ast.expr], constants: dict[str, str]
) -> str | None:
    if not arguments:
        return None
    first = arguments[0]
    if isinstance(first, ast.Constant) and isinstance(first.value, str):
        return first.value
    if isinstance(first, ast.Name):
        return constants.get(first.id)
    return None


def _module_constants(tree: ast.Module) -> dict[str, str]:
    """Return the module-level ``NAME = "literal"`` string constants."""
    constants: dict[str, str] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            if (
                isinstance(target, ast.Name)
                and isinstance(node.value, ast.Constant)
                and isinstance(node.value.value, str)
            ):
                constants[target.id] = node.value.value
    return constants


def scan_file(path: Path, *, root: Path) -> list[SurfaceFinding]:
    """Scan one file and return its boundary findings."""
    relative = _relative(path, root)
    scope = scope_of(relative)
    report_scope = is_report_scope(relative)
    # Inside a retired package the imports are the package's own internals:
    # 08 deletes them as a unit, so they are not separate migration entries.
    retired = own_legacy_package(relative) is not None
    tree = ast.parse(path.read_text(encoding="utf-8"))
    findings: list[SurfaceFinding] = []

    constants = _module_constants(tree)
    def add(rule: str, symbol: str, node: ast.AST) -> None:
        finding = SurfaceFinding(
            path=relative,
            scope=scope,
            rule=rule,
            symbol=symbol,
            line=int(getattr(node, "lineno", 0)),
        )
        if finding not in findings:
            findings.append(finding)

    def check_import(module: str | None, node: ast.AST, *, dynamic: bool) -> None:
        legacy = None if retired else _legacy_of(module)
        if legacy is not None:
            add(
                "legacy_module_dynamic_import" if dynamic else "legacy_module_import",
                legacy,
                node,
            )
        if report_scope and _native_of(module):
            add("report_native_import", str(module), node)

    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                check_import(alias.name, node, dynamic=False)
        elif isinstance(node, ast.ImportFrom):
            resolved = resolve_import(path, node.module, node.level)
            check_import(resolved, node, dynamic=False)
            for alias in node.names:
                # ``from . import core`` and ``from pkg import core`` name the
                # retired package only in the imported name.
                if resolved:
                    check_import(f"{resolved}.{alias.name}", node, dynamic=False)
        elif isinstance(node, ast.Call):
            name = _called_name(node.func)
            if name in {"import_module", "__import__"}:
                literal = _literal_import(node.args, constants)
                if literal is not None:
                    check_import(literal, node, dynamic=True)
                continue
            if not report_scope:
                continue
            receiver = _call_receiver(node.func)
            if (
                name in SOLVE_NAMES
                or any(token in name for token in NATIVE_TOKENS)
                or receiver in NATIVE_TOKENS
            ):
                add("report_native_call", name or receiver, node)
    return findings


def is_forwarding_shell(path: Path, *, relative: str | None = None) -> bool:
    """
    Return whether a module exists only to keep a retired import path alive.

    A shell is a non-package module whose whole body is imports, ``__all__`` and
    re-export assignments, and which is either on a retired path or forwards a
    retired package.  A package ``__init__.py`` is a facade, and a thin module
    that re-exports live symbols is a normal module.
    """
    if path.name == "__init__.py":
        return False
    body = [
        node
        for node in ast.parse(path.read_text(encoding="utf-8")).body
        if not (
            isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str)
        )
    ]
    if not body:
        return False
    statements: list[ast.Import | ast.ImportFrom] = []
    for node in body:
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            statements.append(node)
            continue
        if isinstance(node, ast.Assign) and all(
            isinstance(target, ast.Name) and target.id == "__all__"
            for target in node.targets
        ):
            continue
        return False
    if own_legacy_package(relative or path.name) is not None:
        return True
    for statement in statements:
        if isinstance(statement, ast.Import):
            if any(_legacy_of(alias.name) for alias in statement.names):
                return True
        elif _legacy_of(resolve_import(path, statement.module, statement.level)):
            return True
    return False


def scan_tree(root: Path | str, *, include_tests: bool = True) -> list[SurfaceFinding]:
    """
    Scan a package root: ``src/<package>``, ``scripts`` and optionally ``tests``.

    Only those trees are read, so a build directory or a generated artifact next
    to them cannot influence the gate.
    """
    root = Path(root)
    bases = [root / "src" / PACKAGE, root / "scripts"]
    if include_tests:
        bases.append(root / "tests")
    findings: list[SurfaceFinding] = []
    for base in bases:
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*.py")):
            if "__pycache__" in path.parts:
                continue
            relative = _relative(path, root)
            findings.extend(scan_file(path, root=root))
            if scope_of(relative) != "test" and is_forwarding_shell(
                path, relative=relative
            ):
                findings.append(
                    SurfaceFinding(
                        path=relative,
                        scope=scope_of(relative),
                        rule="legacy_forwarding_shell",
                        symbol=path.stem,
                        line=1,
                    )
                )
    return findings


def present_legacy_packages(root: Path | str) -> list[str]:
    """Return the retired package directories that still exist."""
    source_root = Path(root) / "src" / PACKAGE
    if not source_root.is_dir():
        return []
    return [
        candidate.as_posix()
        for candidate in sorted(source_root.iterdir())
        if candidate.is_dir() and candidate.name in LEGACY_PACKAGES
    ]


# --------------------------------------------------------------------------- #
# evaluation
# --------------------------------------------------------------------------- #


def _entry_key(entry: dict[str, object]) -> tuple[str, str, str, str]:
    return (
        str(entry["path"]),
        str(entry["scope"]),
        str(entry["rule"]),
        str(entry["symbol"]),
    )


def evaluate(
    findings: list[SurfaceFinding],
    *,
    mode: str = MODE_MIGRATION,
    registered: list[dict[str, object]] | None = None,
) -> list[SurfaceFinding]:
    """Return the fatal findings of one gate run."""
    registered_keys = {_entry_key(entry) for entry in registered or []}
    fatal: list[SurfaceFinding] = []
    for finding in findings:
        if mode == MODE_FINAL:
            fatal.append(finding)
            continue
        if finding.scope == "test":
            # 08 removes these together with the packages they import.
            continue
        if finding.key in registered_keys:
            continue
        fatal.append(finding)
    return fatal


def stale_registrations(
    findings: list[SurfaceFinding], registered: list[dict[str, object]]
) -> list[SurfaceFinding]:
    """Return registrations that no longer match a live finding."""
    live = {finding.key for finding in findings}
    live_counts: dict[tuple[str, str, str, str], int] = {}
    for finding in findings:
        live_counts[finding.key] = live_counts.get(finding.key, 0) + 1
    stale: list[SurfaceFinding] = []
    for entry in registered:
        key = _entry_key(entry)
        symbol = str(entry["symbol"])
        if key not in live:
            stale.append(_stale_finding(entry, symbol))
        elif "occurrences" in entry and int(entry["occurrences"]) != live_counts.get(
            key, 0
        ):
            stale.append(
                _stale_finding(
                    entry,
                    f"{symbol} (occurrences {entry['occurrences']} "
                    f"!= live {live_counts.get(key, 0)})",
                )
            )
    return stale


def _stale_finding(entry: dict[str, object], symbol: str) -> SurfaceFinding:
    return SurfaceFinding(
        path=str(entry["path"]),
        scope=str(entry["scope"]),
        rule=str(entry["rule"]),
        symbol=symbol,
        line=0,
    )


def load_registry(path: Path | str) -> list[dict[str, object]]:
    """Load the migration registry."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return [dict(entry) for entry in data.get("entry", [])]


def report(findings: list[SurfaceFinding], *, mode: str) -> None:
    """Print a summary of a gate run."""
    counts: dict[str, int] = {}
    for finding in findings:
        counts[finding.rule] = counts.get(finding.rule, 0) + 1
    print(f"mode      : {mode}")
    print(f"findings  : {len(findings)}")
    for rule in sorted(counts):
        print(f"  {rule}: {counts[rule]}")


def main(argv: list[str] | None = None) -> int:
    """Run the gate and return its exit status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", default=None)
    parser.add_argument("--registry", default=None)
    parser.add_argument("--final", action="store_true")
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args(argv)

    package_root = (
        Path(args.package_root).resolve()
        if args.package_root
        else Path(__file__).resolve().parents[2]
    )
    registry_path = (
        Path(args.registry).resolve()
        if args.registry
        else Path(__file__).with_name("legacy_surface_registry.json")
    )
    mode = MODE_FINAL if args.final else MODE_MIGRATION

    findings = scan_tree(package_root)
    registered = load_registry(registry_path) if registry_path.is_file() else []
    report(findings, mode=mode)
    fatal = evaluate(findings, mode=mode, registered=registered)
    failures = [finding.describe() for finding in fatal]
    if mode == MODE_MIGRATION:
        failures.extend(
            f"stale registry entry: {finding.path} [{finding.rule}/{finding.symbol}]"
            for finding in stale_registrations(findings, registered)
        )
    else:
        failures.extend(
            f"legacy package still present: {path}"
            for path in present_legacy_packages(package_root)
        )
    if not failures:
        print("\nOK: no unregistered Python boundary violation")
        return 0
    print("\nFAIL")
    for failure in failures:
        print(f"  - {failure}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
