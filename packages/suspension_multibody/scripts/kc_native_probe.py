#!/usr/bin/env python
"""
Gate script: native K grid vs the frozen Python snapshot.

The run goes through the unified simulation service -- a ``SimulationRequest``
carrying the model and case documents in, one raw contract result out -- so this
gate scores the path a caller actually uses, not a parallel ctypes route kept
alive for the gate.  The tolerances are the strict Adams gate's.
"""

from __future__ import annotations

import json
from pathlib import Path

from suspension_multibody.api import validate
from suspension_multibody.authoring import migrate_v1_kc_case
from suspension_multibody.cases.kc_quasi_static.settings import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
)
from suspension_multibody.report.kc_evidence import k_records
from suspension_multibody.results.envelope import ResultEnvelope
from suspension_multibody.schema.model import AxleDeclaration
from suspension_multibody.simulation import run_compiled

BASELINE = Path("packages/suspension_multibody/tests/data/kc_baseline")
OUT = Path("artifacts/kc-native-probe")
#: The declarative benchmark-axle fixture, read by explicit path: this gate must
#: not import a test package.
BENCHMARK_FIXTURE = (
    Path(__file__).resolve().parents[3]
    / "packages/suspension_multibody/tests/data/benchmark_axle.json"
)

WHEEL_VALUES_MM = (-10.0, 0.0, 10.0)
RACK_VALUES_MM = (-5.0, 0.0, 5.0)


def benchmark_model() -> AxleDeclaration:
    """Build the shared benchmark axle from the declarative fixture."""
    payload = json.loads(BENCHMARK_FIXTURE.read_text(encoding="utf-8"))
    return AxleDeclaration.model_validate(payload["model"])


def tolerance(field: str, reference: float) -> float:
    """Return the strict K tolerance for one field (mm or deg)."""
    return (
        0.1 + 0.002 * abs(reference)
        if field.endswith("_mm")
        else 0.02 + 0.005 * abs(reference)
    )


def k_grid_states(model: AxleDeclaration) -> list[dict[str, object]]:
    """Solve the K grid through the unified simulation service."""
    assembly, case = migrate_v1_kc_case(
        model, mode="K",
        name="kc-k",
        wheel_values_mm=WHEEL_VALUES_MM,
        rack_values_mm=RACK_VALUES_MM,
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
    )
    result = run_compiled(validate(assembly, case)).result
    if not isinstance(result, ResultEnvelope):
        raise TypeError("K/C documents must produce ResultEnvelope")
    return k_records(result, frames={side: "wheel.sub.json.wheel_center_"+side for side in ("L", "R")},
        wheel_values=WHEEL_VALUES_MM, rack_values=RACK_VALUES_MM)


def main() -> int:
    """Run the K grid natively and score it against the frozen snapshot."""
    produced = k_grid_states(benchmark_model())
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
