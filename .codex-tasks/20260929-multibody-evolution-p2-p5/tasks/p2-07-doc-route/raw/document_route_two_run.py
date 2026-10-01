#!/usr/bin/env python
"""
Run the document route end to end and print the numbers the evidence records.

This is a second-run utility over the acceptance test's own fixtures rather
than a copy of them: it imports
``packages/suspension_multibody/tests/cases/test_rotational_torque_document.py``
by path and calls its ``_run`` / ``_omega_y`` helpers, so every figure printed
here is the figure the test asserts on.  A copy would be able to drift from the
test and would then be evidence of nothing.

    uv run --no-sync python \\
        .codex-tasks/.../tasks/p2-07-doc-route/raw/document_route_two_run.py

Printed:

* the driven and reaction bodies' ``omega_y`` history for both runs;
* the two final rates and their difference, next to the test's tolerance;
* whether the omitted reference pose is byte-identical to the explicit one.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "packages/suspension_multibody/src"))

import numpy as np  # noqa: E402

TEST = (
    ROOT
    / "packages"
    / "suspension_multibody"
    / "tests"
    / "cases"
    / "test_rotational_torque_document.py"
)

spec = importlib.util.spec_from_file_location("document_route_acceptance", TEST)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(module)

with_run = module._run(with_element=True)
without_run = module._run(with_element=False)
explicit = module._run(with_element=True, reference_quaternion=True)


def _row(state: "np.ndarray", body: int) -> list[float]:
    return [float(value) for value in module._omega_y(state, body)]


print("with    driven   omega_y:", _row(with_run, 1))
print("without driven   omega_y:", _row(without_run, 1))
print("with    reaction omega_y:", _row(with_run, 0))
print("without reaction omega_y:", _row(without_run, 0))
driven_with = float(module._omega_y(with_run, 1)[-1])
driven_without = float(module._omega_y(without_run, 1)[-1])
reaction_with = float(module._omega_y(with_run, 0)[-1])
reaction_without = float(module._omega_y(without_run, 0)[-1])
print(f"driven   final: with={driven_with!r} without={driven_without!r} "
      f"difference={abs(driven_with - driven_without)!r}")
print(f"reaction final: with={reaction_with!r} without={reaction_without!r} "
      f"difference={abs(reaction_with - reaction_without)!r}")
print("tolerance:", module.TOLERANCE)
print(
    "omitted reference pose is byte-identical to the explicit identity:",
    bool(np.array_equal(explicit, with_run)),
)
