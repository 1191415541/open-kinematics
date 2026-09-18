#!/usr/bin/env python
"""
Gate script: native C paths vs the frozen Python snapshot.

Thin wrapper over ``suspension_multibody.native_kc``; the tolerances are the
strict C gate's (translation ``1e-6 mm + 1e-4*|ref|``, rotation
``1e-8 rad + 1e-4*|ref|``).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from suspension_multibody.model import build_front_axle
from suspension_multibody.native_kc import run_c_paths_contract
from suspension_multibody.native_kc.load_paths import LoadPath

BASELINE = Path("packages/suspension_multibody/tests/data/kc_baseline")
OUT = Path("artifacts/kc-native-probe")


def _compliant_model():
    """Import the test fixture so the gate and the test share one model."""
    import importlib.util

    path = Path("packages/suspension_multibody/tests/native_kc/test_native_kc_parity.py")
    spec = importlib.util.spec_from_file_location("native_kc_parity_fixture", path)
    module = importlib.util.module_from_spec(spec)  # ty: ignore[invalid-argument-type]
    assert spec.loader is not None  # ty: ignore[possibly-unbound-attribute]
    spec.loader.exec_module(module)  # ty: ignore[possibly-unbound-attribute]
    return module._compliant_model()  # ty: ignore[unresolved-attribute]


def tolerance(index: int, reference: float) -> float:
    """Return the strict C tolerance for one component (mm or rad)."""
    magnitude = abs(reference)
    return (1e-6 + 1e-4 * magnitude) if index < 3 else (1e-8 + 1e-4 * magnitude)


def main() -> int:
    """Run the C paths natively and score them against the frozen snapshot."""
    assembly = build_front_axle(_compliant_model(), "C")
    axes = tuple(path.name for path in LoadPath.standard())
    produced = run_c_paths_contract(assembly, paths=axes)
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
