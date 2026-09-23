"""
The bypass has to fail: `outputs/` may not solve, submit native or prepare.

`derived.py`'s docstring claims a derived output is reproducible from the run's own
outputs.  A claim like that decays the moment someone adds a convenient import, so
it is checked by reading the package's own source rather than by trusting the
review that added it.

The scanner is tested in both directions: it must flag a module that reaches for
the solver, and it must pass a clean one.  A gate that cannot fail is not a gate.
"""

from __future__ import annotations

import ast
import subprocess
import sys
from pathlib import Path

import pytest

#: The repository root, from `packages/suspension_multibody/tests/outputs/`.
_ROOT = Path(__file__).resolve().parents[4]

#: The package whose source is being checked.
_OUTPUTS = _ROOT / "packages/suspension_multibody/src/suspension_multibody/outputs"

#: Module stems a derived output must never import.  These are the layers that
#: would turn "arithmetic over the run's outputs" back into "ask the solver".
_FORBIDDEN_MODULES = (
    "kernel",
    "native",
    "solver",
    "preparation",
    "elements",
    "core",
    "analysis",
)

#: Call names that would recompute a constitutive law or run the solver.
_FORBIDDEN_CALLS = (
    "run_case",
    "run_dynamic_case",
    "run_vehicle_dynamics",
    "solve",
    "submit",
    "build_front_axle",
    "build_vehicle",
    "compute_axle_metrics",
    "compute_vehicle_metrics",
    "run_contract",
)


def _violations(source: str, *, path: str = "<test>") -> list[str]:
    """
    Return the ways `source` reaches outside a derived output's own arithmetic.

    Parsed rather than grepped: the word "kernel" appears in the module's prose,
    and a text search would either flag the documentation or force the
    documentation to avoid the word.
    """
    tree = ast.parse(source, filename=path)
    found: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                root = alias.name.split(".")[0]
                if root in _FORBIDDEN_MODULES:
                    found.append(f"{path}:{node.lineno} imports {alias.name}")
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            parts = [node.level and "" or "", *module.split(".")]
            if any(part in _FORBIDDEN_MODULES for part in parts if part):
                found.append(f"{path}:{node.lineno} imports from {module}")
        elif isinstance(node, ast.Call):
            name = ""
            if isinstance(node.func, ast.Name):
                name = node.func.id
            elif isinstance(node.func, ast.Attribute):
                name = node.func.attr
            if name in _FORBIDDEN_CALLS:
                found.append(f"{path}:{node.lineno} calls {name}")
    return found


def test_the_scanner_flags_a_module_that_reaches_for_the_solver(tmp_path: Path) -> None:
    """The scanner must be able to fail, or it proves nothing."""
    offending = tmp_path / "offending.py"
    offending.write_text(
        "from suspension_multibody.kernel import run_contract\n"
        "def go(result):\n"
        "    return run_contract(result)\n",
        encoding="utf-8",
    )
    found = _violations(offending.read_text(encoding="utf-8"), path=str(offending))
    assert any("kernel" in item for item in found)
    assert any("run_contract" in item for item in found)


def test_the_scanner_passes_a_clean_module(tmp_path: Path) -> None:
    clean = tmp_path / "clean.py"
    clean.write_text(
        "import numpy as np\n"
        "def go(values):\n"
        "    return float(np.max(values))\n",
        encoding="utf-8",
    )
    assert _violations(clean.read_text(encoding="utf-8"), path=str(clean)) == []


def test_the_outputs_package_never_reaches_outside_its_own_arithmetic() -> None:
    sources = sorted(_OUTPUTS.glob("*.py"))
    assert sources, "the outputs package should have sources to check"
    violations: list[str] = []
    for source in sources:
        violations.extend(
            _violations(source.read_text(encoding="utf-8"), path=str(source))
        )
    assert violations == [], "\n".join(violations)


def test_the_legacy_surface_gate_stays_green() -> None:
    """A new package must not become a legacy import site."""
    gate = _ROOT / "packages/suspension_multibody/tests/architecture/legacy_surface_gate.py"
    completed = subprocess.run(
        [sys.executable, str(gate), "--check"],
        capture_output=True,
        text=True,
        cwd=str(_ROOT),
        check=False,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr


@pytest.mark.parametrize("stem", _FORBIDDEN_MODULES)
def test_each_forbidden_stem_is_actually_detected(tmp_path: Path, stem: str) -> None:
    """Every name on the forbidden list is enforced, not just documented."""
    offending = tmp_path / "probe.py"
    offending.write_text(f"from suspension_multibody.{stem} import thing\n", encoding="utf-8")
    found = _violations(offending.read_text(encoding="utf-8"), path=str(offending))
    assert found, stem
