#!/usr/bin/env python
"""
Assert that solving happens natively and legacy paths fail closed.

Two questions are answered separately:

* does any *native* entry module (``preparation/vehicle_dynamic.py``,
  ``results/vehicle.py``, ``vehicle/service.py``, ``axle_dynamics/``) import or
  name the Python equilibrium solver?  Any hit is a failure.
* how many *legacy* consumers (``api.py``, ``analysis/``, ``adams/``) still do?
  Non-zero is expected before the migration and becomes a failure under
  ``--strict``.

The question is now trivially answered: the Python equilibrium solver -- and the
whole ``solver/`` package with it -- has been **deleted**, so nothing can reach
it.  Keeping the check is still worth it, because it turns "we deleted it" into
an assertion that can fail: reintroducing such a solve under this name re-breaks
the gate, and dropping the check would leave only the memory that it once passed.
Only the Python equilibrium-solver package and the ``EquilibriumSolver``
symbol are treated as violations; unrelated uses of the word "solver"
(``solver_settings``, a linear-solver handle, ...) are ignored on purpose.
"""

from __future__ import annotations

import argparse
import ast
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE_ROOT / "src" / "suspension_multibody"
NATIVE_PATHS = (
    SOURCE / "preparation" / "vehicle_dynamic.py",
    SOURCE / "vehicle" / "service.py",
    SOURCE / "results" / "vehicle.py",
    SOURCE / "axle_dynamics",
)
LEGACY_CONSUMERS = (SOURCE / "api.py", SOURCE / "analysis", SOURCE / "adams")
SYMBOL = "EquilibriumSolver"


def _python_files(target: Path) -> list[Path]:
    return sorted(target.rglob("*.py")) if target.is_dir() else [target]


def _hits(target: Path) -> list[tuple[str, int, str]]:
    found: list[tuple[str, int, str]] = []
    for file in _python_files(target):
        if not file.is_file():
            continue
        rel = str(file.relative_to(PACKAGE_ROOT)).replace("\\", "/")
        tree = ast.parse(file.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                module = node.module
                if module == "solver" or module.endswith(".solver"):
                    found.append((rel, node.lineno, f"from {module} import ..."))
                if any(alias.name == SYMBOL for alias in node.names):
                    found.append((rel, node.lineno, f"{SYMBOL} imported"))
            elif isinstance(node, ast.Name) and node.id == SYMBOL:
                found.append((rel, node.lineno, SYMBOL))
            elif isinstance(node, ast.Attribute) and node.attr == SYMBOL:
                found.append((rel, node.lineno, f".{SYMBOL}"))
    return found


def main() -> int:
    """Report solver references and return the gate status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--strict",
        action="store_true",
        help="require zero legacy consumers as well (the post-migration gate)",
    )
    args = parser.parse_args()

    native: list[tuple[str, int, str]] = []
    for target in NATIVE_PATHS:
        native.extend(_hits(target))
    legacy: list[tuple[str, int, str]] = []
    for target in LEGACY_CONSUMERS:
        legacy.extend(_hits(target))

    print(f"native-path references to the Python solver : {len(native)}")
    for rel, line, detail in native:
        print(f"  {rel}:{line} {detail}")
    print(f"legacy consumers still using the solver     : {len(legacy)}")
    for rel, line, detail in legacy[:12]:
        print(f"  {rel}:{line} {detail}")
    if len(legacy) > 12:
        print(f"  ... {len(legacy) - 12} more")

    if native:
        print("\nFAIL: the native path must not touch the Python solver")
        return 1
    if args.strict and legacy:
        print("\nFAIL (--strict): legacy consumers still call the Python solver")
        return 1
    print("\nOK" if not args.strict else "\nOK (strict)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
