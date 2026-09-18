"""
The C load list and the left/right side modes.

A load path is a scalar sweep along one axis; `c.loads` is the same thing said
explicitly, and it is also the only way to state a load with several components
at once.  The two spellings must agree wherever they describe the same loads,
because the frozen strict-C baselines are written in the first and the product
API asks for the second.

The side modes decide whether the marker the document names is loaded alone, in
phase with its mirror, or in anti-phase.  The mirror has to be *named*: a
document that loads one side and means two would otherwise be answered with a
half load and no complaint.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.kernel import run_contract
from suspension_multibody.model import build_front_axle
from suspension_multibody.native_kc import case_document, model_document

from .test_native_kc_parity import _compliant_model

_TIMES_S = (0.0, 1e-3)


def _c_case(loads, *, side_mode: str = "single", mirror: bool = True) -> dict:
    section: dict[str, object] = {
        "load_marker": "wheel_center_L",
        "side_mode": side_mode,
        "loads": loads,
    }
    if mirror:
        section["mirror_marker"] = "wheel_center_R"
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "c-loads",
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "c": section,
    }


def _upright_names() -> tuple[str, str]:
    return "upright_L", "upright_R"


def test_a_mirror_is_required_when_both_sides_are_loaded() -> None:
    assembly = build_front_axle(_compliant_model(), "C")
    model = model_document(assembly, name="c-loads", drive_wheels=False)
    case = _c_case([{"fz": 100.0}], side_mode="symmetric", mirror=False)
    with pytest.raises(Exception, match="mirror_marker"):
        run_contract(model, case)


def test_an_explicit_load_list_is_the_sweep_it_replaces() -> None:
    """`c.loads` and `paths`+`levels`+`maximum` must agree load for load."""
    assembly = build_front_axle(_compliant_model(), "C")
    model = model_document(assembly, name="c-loads", drive_wheels=False)
    maximum = 100.0
    explicit = _c_case(
        [{"my": value} for value in (-maximum, 0.0, maximum)]
    )
    sweep = case_document(
        assembly,
        family="kc_quasi_static",
        name="c-loads",
        paths=("my",),
        levels=3,
        maximum=maximum,
        times_s=_TIMES_S,
        drive_wheels=False,
    )
    produced = run_contract(model, explicit)
    reference = run_contract(model, sweep)
    assert len(produced.cases()) == 3
    assert np.array_equal(
        produced.block("body_state"), reference.block("body_state")
    ), "the explicit load list and the sweep disagreed on the same loads"


def test_the_side_modes_load_the_mirror_in_phase_and_in_anti_phase() -> None:
    assembly = build_front_axle(_compliant_model(), "C")
    model = model_document(assembly, name="c-loads", drive_wheels=False)
    load = [{"fz": 100.0}]
    states = {}
    for mode in ("single", "symmetric", "opposite"):
        run = run_contract(model, _c_case(load, side_mode=mode))
        bodies = list(run.document["manifest"]["bodies"])
        block = run.block("body_state")
        last = int(run.cases()[0]["sample_offset"]) + int(run.cases()[0]["sample_count"]) - 1
        left, right = _upright_names()
        states[mode] = (
            block[last, bodies.index(left), :3],
            block[last, bodies.index(right), :3],
        )

    left_assembled = np.asarray(
        next(b for b in model["bodies"] if b["name"] == "upright_L")["position"],
        dtype=float,
    ) * 1e-3
    right_assembled = np.asarray(
        next(b for b in model["bodies"] if b["name"] == "upright_R")["position"],
        dtype=float,
    ) * 1e-3

    single_left, single_right = states["single"]
    assert np.allclose(single_right, right_assembled, rtol=0.0, atol=1e-12), (
        "a single-sided load moved the mirror"
    )
    assert not np.allclose(single_left, left_assembled, rtol=0.0, atol=1e-9), (
        "the loaded side did not move, so the mirror assertions would be vacuous"
    )

    symmetric_right = states["symmetric"][1] - right_assembled
    opposite_right = states["opposite"][1] - right_assembled
    assert not np.allclose(symmetric_right, 0.0, rtol=0.0, atol=1e-9)
    # Not the exact negative: the load is applied at a body-fixed marker, so the
    # lever arm turns with the pose and the response carries an even-order term.
    # At 100 N on a 1e4 N/mm bushing that term is a couple of parts in a
    # thousand; a wrong mirror or a wrong sign would be out by 100%.
    assert np.allclose(
        opposite_right, -symmetric_right, rtol=0.01, atol=1e-12
    ), "the anti-phase load was not the mirror image of the in-phase one"

    # The mirror moves the other way along one axis only when the load does:
    # the loaded side sees the same load in both modes.
    assert np.allclose(
        states["symmetric"][0], states["opposite"][0], rtol=0.0, atol=1e-12
    ), "the loaded side changed with the mirror's sign"
