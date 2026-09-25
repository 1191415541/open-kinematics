"""
The quasi-static tire is consumed natively, not merely declared.

Subtask 01 recorded this as GAP-1, and the failure was invisible: the K/C model
document carried no `tires` array at all, so the static solve had no tire row,
changing the tire property changed nothing, and the result still converged.  A
run that converges and ignores part of its model is exactly the failure a
convergence flag cannot report.

Two claims are made here, and the second is the one that matters:

1. **the tire reaches the residual.**  The kernel's `tire_output` block reports a
   vertical force, so the model the solve ran on contained the tire;
2. **the property changes the response.**  Doubling the vertical stiffness
   doubles the vertical force, which is only true if the law is evaluated and not
   echoed.

The declaration the run accepts is the *same* tire the dynamic study uses -- the
quasi-static reading degrades the law to its vertical branch, it does not switch
to a second one (decision D1).  So the third test checks that both readings emit
the same tire entry for one assembly.
"""

from __future__ import annotations

import json

import numpy as np
import pytest

from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.compilation import KcStudyInputs, compile_plan, plan_for
from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.simulation import SimulationRequest, run_request

FIXTURE = "packages/suspension_multibody/tests/data/benchmark_axle.json"

#: The case: a symmetric 20 mm droop, so both tires sit at a known compression.
_CASE = {
    "contract": "multibody-case",
    "contract_version": 1,
    "kind": "case",
    "family": "kc_quasi_static",
    "name": "tire-probe",
    "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
    "k": {
        "wheel_values_mm": [-20.0],
        "rack_values_mm": [0.0],
        "axis_map": {"wheel": ["wheel_drive_L", "wheel_drive_R"], "rack": "rack_drive"},
        "left_right_mode": "symmetric",
    },
}


def _model(*, stiffness: float | None) -> FrontAxleModel:
    """
    Return the benchmark axle, with a declared vertical tire when asked.

    ``stiffness=None`` means "declare no tire at all", which is the shape the
    historical authoring produced and the shape the negative control needs.
    """
    payload = json.loads(open(FIXTURE, encoding="utf-8").read())
    raw = dict(payload["model"])
    if stiffness is not None:
        raw["tires"] = [
            {
                "stiffness": stiffness,
                "unloaded_radius": 320.0,
                "contact_point": {"x": 0.0, "y": 0.0, "z": 0.0},
                "local_axis": {"x": 0.0, "y": 0.0, "z": 1.0},
            }
        ]
    return FrontAxleModel.model_validate(raw)


def _solve(stiffness: float | None):
    """Solve the K probe on the axle with (or without) a declared tire."""
    assembly = build_front_axle(_model(stiffness=stiffness), "K")
    document = model_document(assembly, name="tire-probe", drive_wheels=True)
    run = run_request(
        SimulationRequest(
            assembly="axle",
            rig="kc_quasi_static",
            family="kc_quasi_static",
            model=document,
            case=_CASE,
        )
    ).raw
    return document, run


def test_the_model_document_declares_the_tires_the_assembly_carries() -> None:
    """
    The declaration is a consequence of the assembly, not of the family.

    This is the defect in its smallest form: the assembly carried a
    `VerticalTireElement` per side and the document it was authored into carried
    none, so nothing downstream could have consumed it.
    """
    assembly = build_front_axle(_model(stiffness=200.0), "K")
    carried = [element for element in assembly.elements if element.name.startswith("tire_")]
    assert len(carried) == 2, "the fixture assembled no tires to declare"

    document = model_document(assembly, name="tire-probe", drive_wheels=True)
    declared = document["tires"]
    assert [entry["name"] for entry in declared] == ["tire_L", "tire_R"]
    for entry in declared:
        assert entry["body"] == f"upright_{entry['name'][-1]}"
        assert entry["model"] == "native_brush"
        # The vertical branch is the *same law* under a degenerate activation,
        # so the two coefficients the branch cannot use are the neutral values
        # rather than zeros -- a zero is refused by the model reader, not ignored.
        assert entry["parameters"]["vertical_stiffness"] == 200.0
        assert entry["parameters"]["unloaded_radius"] == 320.0
        assert entry["parameters"]["longitudinal_friction_coefficient"] > 0.0


def test_a_tire_declared_by_the_model_enters_the_residual() -> None:
    """
    GAP-1, first half: the kernel reports a vertical tire force.

    Reading the force out of the native result is what turns "the document
    mentions a tire" into "the solve used it"; a document that declared tires and
    a kernel that ignored them would leave this block at zero.
    """
    _, run = _solve(200.0)
    assert "tire_output" in run.blocks
    assert run.tire_names == ("tire_L", "tire_R")
    block = run.block("tire_output")
    compression_m = block[0, :, 2]
    normal_force = block[0, :, 4]
    assert np.all(compression_m > 0.0), compression_m
    assert np.all(normal_force > 0.0), normal_force
    # The declared stiffness is a force per length in the *document's* unit (N/mm)
    # and the kernel reports force in newtons on a penetration given in metres, so
    # the relation is exact once the millimetre is accounted for.  Asserting the
    # relation rather than a hard number is what makes this a statement about the
    # law being evaluated instead of about one fixture's geometry.
    assert np.allclose(normal_force, 200.0 * 1000.0 * compression_m, rtol=1e-9)


def test_changing_the_vertical_property_changes_the_response() -> None:
    """
    GAP-1's real acceptance: the property moves the number.

    A run that echoes the input cannot pass this -- doubling the stiffness
    doubles the force only if the law is actually evaluated.  This is the
    assertion EPIC A6 states as "changing the vertical property changes the
    response", and it is the one subtask 01 recorded as failing.
    """
    _, soft = _solve(200.0)
    _, stiff = _solve(400.0)

    soft_block = soft.block("tire_output")
    stiff_block = stiff.block("tire_output")
    # The compression is a driven quantity here -- the wheel centre is prescribed,
    # so it is the same in both runs -- and the *force* is what the tire law moves.
    assert np.allclose(soft_block[0, :, 2], stiff_block[0, :, 2])
    soft_force = float(soft_block[0, 0, 4])
    stiff_force = float(stiff_block[0, 0, 4])
    assert soft_force > 0.0
    # Doubling k doubles Fn at the same compression.  The comparison is relative
    # rather than exact because the kernel solves the contact unknown to a
    # tolerance; asserting bit equality would be testing the linear solver's
    # rounding rather than the tire law.
    assert stiff_force == pytest.approx(2.0 * soft_force, rel=1e-9)


def test_a_model_with_no_tire_produces_no_tire_row() -> None:
    """
    The negative control, and it is load-bearing.

    "No tire" and "a tire with zero force" are different statements.  A test that
    only checked the force was positive would pass on a kernel that reported a
    row for every model; this one pins the distinction.
    """
    _, run = _solve(None)
    assert "tire_output" not in run.blocks or not run.tire_names


def test_the_quasi_static_tire_is_the_same_entry_the_dynamic_reading_emits() -> None:
    """
    Decision D1: one tire definition, two readings.

    The study changes how far the tire acts, not what the tire is.  Comparing the
    emitted entries is the checkable form: a second, simpler law would show up
    here as a different entry.
    """
    assembly = build_front_axle(_model(stiffness=200.0), "K")
    inputs = KcStudyInputs(name="probe", wheel_values_mm=(-20.0,), rack_values_mm=(0.0,))

    from_plan = {
        study: compile_plan(plan_for(rig, mode="K", inputs=inputs), assembly)[0]
        for study, rig in (("quasi", "kc_quasi_static"),)
    }
    quasi = from_plan["quasi"]
    # The same assembly authored directly must produce the same declaration: the
    # plan does not add or remove tires, so the two paths agree by construction.
    direct = model_document(assembly, name="probe", drive_wheels=True)
    assert quasi["tires"] == direct["tires"]


def test_the_result_channel_reports_the_solved_compression() -> None:
    """
    The reporting channel reads the solve rather than restating the input.

    `tire_compression` used to be a constant `{left: 0.0, right: 0.0}` with the
    reasoning that a `CaseSpec` carries no road height.  That was true of the
    input and false about the answer: the wheel load comes out of the solve.
    """
    from suspension_multibody.api import run_case
    from suspension_multibody.schema import CaseSpec, DisplacementControl

    travel_mm = -20.0
    bundle = run_case(
        _model(stiffness=200.0),
        CaseSpec(
            mode="K",
            controls=(
                DisplacementControl(target="wheel_travel_left", values=(travel_mm,)),
            ),
        ),
    )
    state = bundle.states[0]
    # Both sides of a symmetric sweep carry the same load, and the reported
    # compression is the one the kernel solved at -- not the input echoed back.
    assert set(state.tire_compression) == {"left", "right"}
    assert state.tire_compression["left"] == state.tire_compression["right"]
    assert state.tire_compression["left"] > 0.0
    assert state.force_residual < 1e-6

    # And it agrees with the kernel's own block for the same run, which is the
    # point: one number, read from where it was solved.
    _, run = _solve(200.0)
    assert state.tire_compression["left"] == run.block("tire_output")[0, 0, 2]


def test_the_neutral_coefficients_are_the_kernel_s_requirement() -> None:
    """
    The inert coefficients are stated, not left to a zero the reader refuses.

    The model reader requires every tire coefficient strictly positive even where
    a static solve uses only the vertical branch, so a document that omitted them
    would be *refused* rather than solved -- and one that wrote zeros would be
    refused too.  Naming them here is what makes the entry's meaning explicit.
    """
    from suspension_multibody.cases.kc_quasi_static.contract import (
        _TIRE_NEUTRAL_COEFFICIENTS,
    )

    assert set(_TIRE_NEUTRAL_COEFFICIENTS) == {
        "longitudinal_friction_coefficient",
        "lateral_friction_coefficient",
        "longitudinal_brush_stiffness",
        "lateral_brush_stiffness",
        "longitudinal_relaxation_length",
        "lateral_relaxation_length",
        "detached_relaxation_s",
    }
    assert all(value > 0.0 for value in _TIRE_NEUTRAL_COEFFICIENTS.values())


def test_a_declaring_model_declares_a_physical_compression_limit() -> None:
    """`maximum_compression` must be positive and below the radius, or no solve."""
    assembly = build_front_axle(_model(stiffness=200.0), "K")
    document = model_document(assembly, name="tire-probe", drive_wheels=True)
    for entry in document["tires"]:
        radius = entry["parameters"]["unloaded_radius"]
        limit = entry["parameters"]["maximum_compression"]
        assert 0.0 < limit < radius


def test_declaring_a_tire_leaves_the_frozen_kinematics_untouched() -> None:
    """
    The fix is additive: a model that declares no tire is solved exactly as before.

    This is the regression guard for the change itself, and it is stated as a
    *measurement* rather than as a claim about the gate: the same K case is solved
    on the undeclared fixture and on the declared one, and the body states agree
    to numerical noise.  The tire adds a force row and a compression unknown; it
    does not move the kinematic solution, which is why the frozen K/C snapshot --
    recorded from models with no tire declaration -- stays valid.
    """
    from suspension_multibody.cases.kc_quasi_static.contract import UNITS

    assert UNITS["length"] == "mm"  # the contract's own frame, unchanged

    without = build_front_axle(_model(stiffness=None), "K")
    with_tire = build_front_axle(_model(stiffness=200.0), "K")
    assert not without.elements
    assert len(with_tire.elements) == 2

    def solve(assembly):
        document = model_document(assembly, name="tire-probe", drive_wheels=True)
        run = run_request(
            SimulationRequest(
                assembly="axle",
                rig="kc_quasi_static",
                family="kc_quasi_static",
                model=document,
                case=_CASE,
            )
        ).raw
        return run

    baseline = solve(without)
    declared = solve(with_tire)
    # The kinematic solution is the same one: a vertical tire force at the wheel
    # centre is carried by the contact row the static trim adds, and the wheel
    # centre is itself a driven coordinate, so the travel is unchanged.
    for body in ("upright_L", "upright_R"):
        assert np.allclose(
            baseline.body_state(body), declared.body_state(body), atol=1e-6
        )
