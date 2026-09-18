"""
Native K/C equivalence against the frozen Python snapshot.

The snapshot under ``tests/data/kc_baseline`` was produced by the Python
quasi-static solvers and is the reference the migration has to reproduce.  The
tolerances are the ones the strict Adams gates already use, so the same numbers
govern both comparisons:

* K  -- position ``0.1 mm + 0.002*|ref|``, angle ``0.02 deg + 0.005*|ref|``
  (``adams/strict_k.py``);
* C  -- translation ``1e-6 mm + 1e-4*|ref|``, rotation ``1e-8 rad + 1e-4*|ref|``
  (``adams/strict_c.py``).
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.model import build_front_axle
from suspension_multibody.native_kc import run_c_paths, run_k_grid
from suspension_multibody.native_kc.load_paths import LoadPath
from suspension_multibody.schema import Bushing6x6, FrontAxleModel, Pose, Vec3

BASELINE = Path(__file__).parents[1] / "data" / "kc_baseline"
SIDES = ("left", "right")


def _compliant_model() -> FrontAxleModel:
    """Return the synthetic bushing model the C snapshot was recorded with."""
    base = benchmark_model()
    stiffness = tuple(
        tuple(
            10_000.0 if row == column and row < 3
            else 10_000_000.0 if row == column
            else 0.0
            for column in range(6)
        )
        for row in range(6)
    )

    def mount(name: str) -> Pose:
        point = base.hardpoints[name]
        if hasattr(point, "x"):          # pydantic Vec3
            x, y, z = float(point.x), float(point.y), float(point.z)
        elif isinstance(point, dict):
            x, y, z = float(point["x"]), float(point["y"]), float(point["z"])
        else:
            x, y, z = (float(v) for v in point)
        return Pose(translation=Vec3(x=x, y=y, z=z))

    bushings = tuple(
        Bushing6x6(
            name=f"{body}_{index}",
            body_a="chassis",
            body_b=body,
            pose_a=mount(name),
            pose_b=mount(name),
            stiffness=stiffness,
        )
        for body, names in (
            ("upper_arm", ("uca_front", "uca_rear")),
            ("lower_arm", ("lca_front", "lca_rear")),
        )
        for index, name in enumerate(names)
    )
    return base.model_copy(
        update={"bushings": bushings, "name": f"{base.name}_compliant"}
    )


def _snapshot(name: str) -> dict[str, dict]:
    payload = json.loads((BASELINE / name).read_text(encoding="utf-8"))
    return {state["case_id"]: state for state in payload}


def _k_tolerance(field: str, reference: float) -> float:
    magnitude = abs(reference)
    if field.endswith("_mm"):
        return 0.1 + 0.002 * magnitude
    return 0.02 + 0.005 * magnitude


def _c_tolerance(index: int, reference: float) -> float:
    magnitude = abs(reference)
    return (1e-6 + 1e-4 * magnitude) if index < 3 else (1e-8 + 1e-4 * magnitude)


def test_native_k_grid_matches_the_python_snapshot() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    expected = _snapshot("k_states.json")
    produced = run_k_grid(assembly)
    assert {state["case_id"] for state in produced} == set(expected)
    worst = 0.0
    for state in produced:
        reference = expected[state["case_id"]]
        for field, value in state.items():
            if field not in reference or not isinstance(value, float):
                continue
            error = abs(value - float(reference[field]))
            ratio = error / _k_tolerance(field, float(reference[field]))
            worst = max(worst, ratio)
    assert worst < 1.0, f"K grid is {worst:.3f}x the strict gate tolerance"


def test_native_c_paths_match_the_python_snapshot() -> None:
    assembly = build_front_axle(_compliant_model(), "C")
    expected = _snapshot("c_states.json")
    axes = tuple(path.name for path in LoadPath.standard())
    produced = run_c_paths(assembly, paths=axes)
    assert {state["case_id"] for state in produced} == set(expected)
    worst = 0.0
    worst_case = None
    for state in produced:
        reference = expected[state["case_id"]]
        for key in ("deformation_left", "deformation_right"):
            actual = np.asarray(state[key], dtype=float)
            wanted = np.asarray(reference[key], dtype=float)
            for index in range(6):
                error = abs(actual[index] - wanted[index])
                ratio = error / _c_tolerance(index, wanted[index])
                if ratio > worst:
                    worst, worst_case = ratio, (state["case_id"], key, index)
    assert worst < 1.0, f"C paths are {worst:.3f}x the strict gate tolerance at {worst_case}"
