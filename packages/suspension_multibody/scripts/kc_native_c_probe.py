#!/usr/bin/env python
"""
Gate script: native C paths vs the frozen Python snapshot.

The paths are solved through the unified simulation service -- a
``SimulationRequest`` carrying the model and case documents in, one raw contract
result out -- and scored against the frozen snapshot with the strict C gate's
tolerances (translation ``1e-6 mm + 1e-4*|ref|``, rotation
``1e-8 rad + 1e-4*|ref|``).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from suspension_multibody.api import validate
from suspension_multibody.authoring import migrate_v1_kc_case
from suspension_multibody.cases.kc_quasi_static.load_paths import LoadPath
from suspension_multibody.cases.kc_quasi_static.settings import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
)
from suspension_multibody.report.kc_evidence import c_records
from suspension_multibody.results.envelope import ResultEnvelope
from suspension_multibody.simulation import run_compiled

BASELINE = Path("packages/suspension_multibody/tests/data/kc_baseline")
OUT = Path("artifacts/kc-native-probe")

LEVELS = 11
MAXIMUM = 1.0


def _compliant_model():
    """Import the test fixture so the gate and the test share one model."""
    import importlib.util

    path = Path("packages/suspension_multibody/tests/cases/kc_quasi_static/kc_fixtures.py")
    spec = importlib.util.spec_from_file_location("kc_quasi_static_fixture", path)
    module = importlib.util.module_from_spec(spec)  # ty: ignore[invalid-argument-type]
    assert spec.loader is not None  # ty: ignore[possibly-unbound-attribute]
    spec.loader.exec_module(module)  # ty: ignore[possibly-unbound-attribute]
    return module._compliant_model()  # ty: ignore[unresolved-attribute]


def tolerance(index: int, reference: float) -> float:
    """Return the strict C tolerance for one component (mm or rad)."""
    magnitude = abs(reference)
    return (1e-6 + 1e-4 * magnitude) if index < 3 else (1e-8 + 1e-4 * magnitude)


def c_path_states(model, *, paths: tuple[str, ...]) -> list[dict[str, object]]:
    """Solve the C load paths through the unified simulation service."""
    assembly, case = migrate_v1_kc_case(
        model, mode="C",
        name="kc-c",
        paths=tuple(paths),
        levels=LEVELS,
        maximum=MAXIMUM,
        side_mode="single",
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
    )
    result = run_compiled(validate(assembly, case)).result
    if not isinstance(result, ResultEnvelope):
        raise TypeError("K/C documents must produce ResultEnvelope")
    return c_records(result, frames={side: "wheel.sub.json.wheel_center_"+side for side in ("L", "R")},
        paths=paths, levels=LEVELS, maximum=MAXIMUM)


def main() -> int:
    """Run the C paths natively and score them against the frozen snapshot."""
    axes = tuple(path.name for path in LoadPath.standard())
    produced = c_path_states(_compliant_model(), paths=axes)
    frozen = json.loads((BASELINE / "c_states.json").read_text(encoding="utf-8"))
    expected = {state["case_id"]: state for state in frozen}
    worst, worst_component = 0.0, None
    for state in produced:
        reference = expected[state["case_id"]]
        for key in ("deformation_left", "deformation_right"):
            actual = np.asarray(state[key], dtype=float)
            wanted = np.asarray(reference[key], dtype=float)
            for index in range(6):
                ratio = abs(actual[index] - wanted[index]) / tolerance(index, wanted[index])
                if ratio > worst:
                    worst, worst_component = ratio, (state["case_id"], key, index)
    print(f"checked {len(produced)} C states, worst error / tolerance ratio {worst:.6g}")
    print("worst component:", worst_component)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "c_states.json").write_text(
        json.dumps(produced, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"wrote candidate C states -> {OUT / 'c_states.json'}")
    return 0 if worst < 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
