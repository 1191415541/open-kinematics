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

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.cases.kc_quasi_static import (
    NativeKcError,
    case_document,
    model_document,
)
from suspension_multibody.cases.kc_quasi_static.workflow import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
    _side_fields,
)
from suspension_multibody.model import build_front_axle
from suspension_multibody.simulation import SimulationRequest, run_request

BASELINE = Path("packages/suspension_multibody/tests/data/kc_baseline")
OUT = Path("artifacts/kc-native-probe")

WHEEL_VALUES_MM = (-10.0, 0.0, 10.0)
RACK_VALUES_MM = (-5.0, 0.0, 5.0)


def tolerance(field: str, reference: float) -> float:
    """Return the strict K tolerance for one field (mm or deg)."""
    return (
        0.1 + 0.002 * abs(reference)
        if field.endswith("_mm")
        else 0.02 + 0.005 * abs(reference)
    )


def k_grid_states(assembly) -> list[dict[str, object]]:
    """Solve the K grid through the unified simulation service."""
    model = model_document(assembly, name="native-k", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="kc-k",
        wheel_values_mm=WHEEL_VALUES_MM,
        rack_values_mm=RACK_VALUES_MM,
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=case,
        )
    ).raw
    left_states = run.body_state("upright_L")
    right_states = run.body_state("upright_R")
    records: list[dict[str, object]] = []
    for index, entry in enumerate(run.cases):
        wheel = WHEEL_VALUES_MM[index // len(RACK_VALUES_MM)]
        rack = RACK_VALUES_MM[index % len(RACK_VALUES_MM)]
        case_id = f"k-w{wheel:+.0f}-r{rack:+.0f}"
        # The case layer expands the grid in document order; checking the name
        # it reported turns a silent reordering into a failure.
        if str(entry["name"]) != case_id:
            raise NativeKcError(
                f"the kernel expanded {entry['name']!r} where {case_id!r} was expected"
            )
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        record: dict[str, object] = {
            "case_id": case_id,
            "wheel_travel_mm": float(wheel),
            "rack_displacement_mm": float(rack),
        }
        record.update(
            _side_fields(assembly, "L", left_states[last, :3], left_states[last, 3:7])
        )
        record.update(
            _side_fields(assembly, "R", right_states[last, :3], right_states[last, 3:7])
        )
        records.append(record)
    return records


def main() -> int:
    """Run the K grid natively and score it against the frozen snapshot."""
    assembly = build_front_axle(benchmark_model(), "K")
    produced = k_grid_states(assembly)
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
