"""
Declarative benchmark-axle fixture loader.

The fixture itself is data -- ``tests/data/benchmark_axle.json`` -- and this
module only reads it, by explicit path, on every call.  It keeps no module-level
model, so no test can observe state another test left behind.  It replaces
``analysis/benchmarks.py`` for the tests, which 08 deletes; the gate scripts read
the same file directly and never import a test module.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from suspension_multibody.schema import FrontAxleModel

#: The one declarative fixture every K/C gate, probe and test measures against.
FIXTURE = Path(__file__).with_name("data") / "benchmark_axle.json"


def _payload() -> dict[str, Any]:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def benchmark_model() -> FrontAxleModel:
    """Return the fixed non-proprietary axle the K/C gates share."""
    return FrontAxleModel.model_validate(_payload()["model"])


def benchmark_grid() -> tuple[tuple[float, ...], tuple[float, ...]]:
    """Return the fixed 10x10 wheel-travel x rack work-point grid."""
    grid = _payload()["grid"]
    return (
        tuple(float(value) for value in grid["wheel_values_mm"]),
        tuple(float(value) for value in grid["rack_values_mm"]),
    )


__all__ = ["FIXTURE", "benchmark_grid", "benchmark_model"]
