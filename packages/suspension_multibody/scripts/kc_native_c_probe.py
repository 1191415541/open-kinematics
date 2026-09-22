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

from suspension_multibody.cases.kc_quasi_static import (
    AXIS_ORDER,
    MM,
    NativeKcError,
    case_document,
    model_document,
    quaternion_to_rotation,
)
from suspension_multibody.cases.kc_quasi_static.load_paths import LoadPath
from suspension_multibody.cases.kc_quasi_static.workflow import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
    SIDES,
    _assembling_pose,
    _side_fields,
    quaternion_conjugate,
    quaternion_multiply,
    wheel_center_world,
)
from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.preparation.geometry import quaternion_to_rotation_vector
from suspension_multibody.simulation import SimulationRequest, run_request

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


def c_path_states(assembly, *, paths: tuple[str, ...]) -> list[dict[str, object]]:
    """Solve the C load paths through the unified simulation service."""
    model = model_document(assembly, name="native-c", drive_wheels=False)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="kc-c",
        paths=tuple(paths),
        levels=LEVELS,
        maximum=MAXIMUM,
        side_mode="single",
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=False,
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
    # The C deformation is measured against the neutral K pose, which is the
    # assembling pose: at the design separation every driven target has zero
    # residual, so the reference is the model document's own initial state.
    reference = {side: _assembling_pose(model, side) for side in SIDES}
    # The K reference the C response is measured from.  The driven case's zero
    # target resolves to the separation the model was assembled with, so the
    # assembling pose *is* the K reference -- the same thing the Python solver's
    # `KReferenceCache` solves for, reached without solving it again.
    reference_metrics = {
        ("left" if side == "L" else "right"): _side_fields(
            assembly, side, reference[side][0], reference[side][1]
        )
        for side in SIDES
    }

    records: list[dict[str, object]] = []
    for index, entry in enumerate(run.cases):
        axis = paths[index // LEVELS]
        position_in_path = index % LEVELS
        if position_in_path == 0:
            level = -MAXIMUM
        elif position_in_path == LEVELS - 1:
            level = MAXIMUM
        else:
            level = -MAXIMUM + position_in_path * (2.0 * MAXIMUM / (LEVELS - 1))
        case_id = f"c-{axis}-{level:+.2f}"
        if str(entry["name"]) != case_id:
            raise NativeKcError(
                f"the kernel expanded {entry['name']!r} where {case_id!r} was expected"
            )
        load = [0.0] * 6
        load[AXIS_ORDER.index(axis)] = float(level)
        record = {
            "case_id": case_id,
            "path": axis,
            "level": float(level),
            "side_mode": "single",
            "load_left": list(load),
            "load_right": [0.0] * 6,
        }
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        metrics: dict[str, dict[str, float]] = {}
        for side, key, states in (
            ("L", "deformation_left", left_states),
            ("R", "deformation_right", right_states),
        ):
            state = states[last]
            metrics["left" if side == "L" else "right"] = _side_fields(
                assembly, side, state[:3], state[3:7]
            )
            ref_position, ref_quaternion = reference[side]
            centre = wheel_center_world(assembly, side, state[:3], state[3:7])
            ref_centre = wheel_center_world(assembly, side, ref_position, ref_quaternion)
            relative = quaternion_multiply(
                quaternion_conjugate(np.asarray(ref_quaternion, dtype=float)),
                np.asarray(state[3:7], dtype=float),
            )
            rotation = quaternion_to_rotation(
                ref_quaternion
            ) @ quaternion_to_rotation_vector(relative)
            record[key] = [
                float(value)
                for value in np.concatenate(((centre - ref_centre) / MM, rotation))
            ]
        record["metrics"] = metrics
        record["c_minus_k"] = {
            key: float(value - reference_metrics[side][key])
            for side in ("left", "right")
            for key, value in metrics[side].items()
        }
        records.append(record)
    return records


def main() -> int:
    """Run the C paths natively and score them against the frozen snapshot."""
    assembly = build_front_axle(_compliant_model(), "C")
    axes = tuple(path.name for path in LoadPath.standard())
    produced = c_path_states(assembly, paths=axes)
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
