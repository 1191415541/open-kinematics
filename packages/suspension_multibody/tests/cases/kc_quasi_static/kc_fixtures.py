from __future__ import annotations

import json
from pathlib import Path

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.schema import Bushing6x6, FrontAxleModel, Pose, Vec3

BASELINE = Path(__file__).parents[2] / "data" / "kc_baseline"


def _compliant_model() -> FrontAxleModel:
    """Return the synthetic bushing model used by the C snapshot gates."""
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
        if hasattr(point, "x"):
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
