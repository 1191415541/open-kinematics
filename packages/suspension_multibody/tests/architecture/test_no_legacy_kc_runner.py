from __future__ import annotations

import re
from pathlib import Path

LEGACY_SYMBOLS = (
    "run_k_grid",
    "run_c_paths",
    "to_native_model",
    "design_separation",
    "k_reference_pose",
    "neutral_k_metrics",
    "origin_wrench",
    "compliance_matrix",
    "axle_drives",
    "_bodies",
    "_joints",
    "_bushings",
    "_contract_body_states",
)


def _exact_symbol_pattern(symbol: str) -> re.Pattern[str]:
    return re.compile(rf"(?<![A-Za-z0-9_]){re.escape(symbol)}(?![A-Za-z0-9_])")


def test_legacy_kc_parity_file_is_absent() -> None:
    tests = Path(__file__).parents[1] / "cases" / "kc_quasi_static"
    assert not (tests / "test_native_kc_parity.py").exists()


def test_legacy_kc_symbols_are_absent_from_public_sources() -> None:
    package = Path(__file__).parents[2] / "src" / "suspension_multibody"
    violations: list[str] = []
    for path in package.rglob("*.py"):
        text = path.read_text(encoding="utf-8")
        for symbol in LEGACY_SYMBOLS:
            if _exact_symbol_pattern(symbol).search(text):
                violations.append(f"{path.relative_to(package)}:{symbol}")
    assert not violations


def test_contract_kc_runners_are_gone() -> None:
    """The strict target: the case layer authors documents, it does not run them."""
    package = (
        Path(__file__).parents[2]
        / "src"
        / "suspension_multibody"
        / "cases"
        / "kc_quasi_static"
    )
    workflow = (package / "workflow.py").read_text(encoding="utf-8")
    exports = (package / "__init__.py").read_text(encoding="utf-8")
    assert "def run_k_grid_contract(" not in workflow
    assert "def run_c_paths_contract(" not in workflow
    assert '"run_k_grid_contract"' not in exports
    assert '"run_c_paths_contract"' not in exports
