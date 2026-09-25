"""
Synthetic fixtures for the extension proof: data, and the loader that reads it.

The fixtures here are **synthetic** and say so, in the file and in this module's
name.  Decision D2 authorises them for exactly one purpose: proving that a new
topology and a new physical bench reach the real native solve without touching
the central assembly, compiler or solver.  They are not engineering models and
nothing here claims accuracy.

Two fixtures:

* ``synthetic_trailing_arm_axle.json`` -- a left/right trailing-arm single axle.
  Its connection graph is deliberately unlike the built-in double wishbone: one
  arm per side on a single chassis revolute, no upper arm, no ball joint, no tie
  rod, no rack.  A rename would not produce this graph;
* ``synthetic_load_bench.json`` -- a physical loading bench built from elements
  that already exist (a fixed frame, a prismatic actuator, a spring and a damper),
  so it contributes real bodies, joints, motions and force elements rather than a
  registration entry.

Both are read by explicit path on every call, so no test can observe state another
test left behind.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from suspension_multibody.schema import FrontAxleModel

DIRECTORY = Path(__file__).resolve().parents[1] / "data" / "composable"

#: The synthetic trailing-arm axle.  **Not an engineering model.**
TRAILING_ARM = DIRECTORY / "synthetic_trailing_arm_axle.json"

#: The synthetic physical loading bench.  **Not an engineering model.**
LOAD_BENCH = DIRECTORY / "synthetic_load_bench.json"


def _payload(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def trailing_arm_payload() -> dict[str, Any]:
    """Return the trailing-arm fixture whole, model and expectations together."""
    return _payload(TRAILING_ARM)


def trailing_arm_model() -> FrontAxleModel:
    """Return the synthetic trailing-arm axle as a model."""
    return FrontAxleModel.model_validate(trailing_arm_payload()["model"])


def trailing_arm_expected() -> dict[str, Any]:
    """Return the fixture's independently derived expectations."""
    return dict(trailing_arm_payload()["expected"])


def load_bench_payload() -> dict[str, Any]:
    """Return the synthetic loading-bench fixture whole."""
    return _payload(LOAD_BENCH)


__all__ = [
    "DIRECTORY",
    "LOAD_BENCH",
    "TRAILING_ARM",
    "load_bench_payload",
    "trailing_arm_expected",
    "trailing_arm_model",
    "trailing_arm_payload",
]
