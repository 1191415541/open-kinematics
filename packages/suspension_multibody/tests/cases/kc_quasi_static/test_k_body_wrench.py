"""
A K sweep can carry a constant body wrench.

The ideal K model is a kinematic solution: the wheel drives plus the rigid joints
determine the pose, so a force applied to the knuckle cannot move it.  The load
still has to *reach* the solver -- it loads the constraints, which is what the
reaction wrench reports -- and dropping it silently would be the failure this
boundary exists to prevent.

The assertions below separate the two: the pose must be bit-identical with and
without the load, and the reactions must not be.
"""

from __future__ import annotations

import numpy as np
import pytest
from suspension_contracts import validate_case

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.kernel import run_contract
from suspension_multibody.model import build_front_axle

_TIMES = {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3}


def _case(*, body_wrench: list[dict] | None = None) -> dict:
    section: dict[str, object] = {
        "axes": [{"coordinate": "wheel_drive_L", "values_mm": [0.0]}],
        "drive": "wheel_center",
    }
    if body_wrench is not None:
        section["body_wrench"] = body_wrench
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "k-body-wrench",
        "time": dict(_TIMES),
        "k": section,
    }


def test_a_body_wrench_moves_the_reactions_and_not_the_kinematics() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="k-body-wrench", drive_wheels=True)
    loaded_case = _case(body_wrench=[{"body": "upright_L", "wrench": [100.0, 0.0, 0.0, 0.0, 5000.0, 0.0]}])
    validate_case(loaded_case)

    unloaded = run_contract(model, _case())
    loaded = run_contract(model, loaded_case)

    # The pose, not the whole row: the unused rate columns carry NaNs on both
    # sides, and the force changes the Newton path by round-off even though the
    # pose itself is kinematically determined.
    assert np.allclose(
        loaded.block("body_state")[:, :, :7],
        unloaded.block("body_state")[:, :, :7],
        rtol=0.0,
        atol=1e-12,
    ), "a force moved an ideal K solution, which the kinematics forbid"
    assert not np.array_equal(
        loaded.block("constraint_wrench"), unloaded.block("constraint_wrench")
    ), "the declared load never reached the solver"


def test_an_unknown_body_in_a_body_wrench_is_refused() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="k-body-wrench", drive_wheels=True)
    case = _case(body_wrench=[{"body": "no_such_body", "wrench": [1.0, 0.0, 0.0, 0.0, 0.0, 0.0]}])
    with pytest.raises(Exception, match="no_such_body"):
        run_contract(model, case)
