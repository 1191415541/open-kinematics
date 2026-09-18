#!/usr/bin/env python
"""
Gate script: native K grid vs the frozen Python snapshot.

The run goes through the contract boundary -- model and case documents in, one
result document out -- so this gate scores the path a caller actually uses, not
a parallel ctypes route kept alive for the gate.  The tolerances are the strict
Adams gate's.
"""

from __future__ import annotations

import json
from pathlib import Path

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.model import build_front_axle
from suspension_multibody.native_kc import run_k_grid_contract

BASELINE = Path("packages/suspension_multibody/tests/data/kc_baseline")
OUT = Path("artifacts/kc-native-probe")


def tolerance(field: str, reference: float) -> float:
    """Return the strict K tolerance for one field (mm or deg)."""
    return (
        0.1 + 0.002 * abs(reference)
        if field.endswith("_mm")
        else 0.02 + 0.005 * abs(reference)
    )


def main() -> int:
    """Run the K grid natively and score it against the frozen snapshot."""
    assembly = build_front_axle(benchmark_model(), "K")
    produced = run_k_grid_contract(assembly)
    frozen = json.loads((BASELINE / "k_states.json").read_text(encoding="utf-8"))
    expected = {state["case_id"]: state for state in frozen}
    worst, worst_field = 0.0, None
    for state in produced:
        reference = expected[state["case_id"]]
        for field, value in state.items():
            if field not in reference or not isinstance(value, float):
                continue
            ratio = abs(value - float(reference[field])) / tolerance(
                field, float(reference[field])
            )
            if ratio > worst:
                worst, worst_field = ratio, (state["case_id"], field)
    print(f"checked {len(produced)} K states, worst error / tolerance ratio {worst:.6g}")
    print("worst component:", worst_field)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "k_states.json").write_text(
        json.dumps(produced, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"wrote candidate K states -> {OUT / 'k_states.json'}")
    return 0 if worst < 1.0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
