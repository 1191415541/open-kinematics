"""
The contract boundary: two documents in, one result document out.

These tests are the acceptance for the boundary switch.  They run the same K
grid and C load paths the ctypes entry point runs, but they never build an
`AxleInput`: the model and the case travel as versioned documents and the
kernel expands the grid itself.  The numbers are compared against the same
frozen snapshot with the same tolerances, so "the new boundary agrees with the
old one" is a measurement rather than a claim.
"""

from __future__ import annotations

import json

import numpy as np
import pytest
from suspension_contracts import validate_result

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.kernel import contract_version, run_contract
from suspension_multibody.model import build_front_axle
from suspension_multibody.native_kc import case_document, model_document
from suspension_multibody.native_kc.workflow import (
    AXIS_ORDER,
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
    run_c_paths_contract,
    run_k_grid_contract,
)

from .test_native_kc_parity import (
    _c_tolerance,
    _compliant_model,
    _k_tolerance,
    _snapshot,
)


def test_contract_entry_point_reports_its_version() -> None:
    assert contract_version() == 1


def test_k_grid_through_the_contract_boundary_matches_the_snapshot() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    expected = _snapshot("k_states.json")
    produced = run_k_grid_contract(assembly)
    assert {state["case_id"] for state in produced} == set(expected)
    worst = 0.0
    for state in produced:
        reference = expected[state["case_id"]]
        for field, value in state.items():
            if field not in reference or not isinstance(value, float):
                continue
            ratio = abs(value - float(reference[field])) / _k_tolerance(
                field, float(reference[field])
            )
            worst = max(worst, ratio)
    assert worst < 1.0, f"contract K grid is {worst:.3f}x the strict gate tolerance"


def test_c_paths_through_the_contract_boundary_match_the_snapshot() -> None:
    assembly = build_front_axle(_compliant_model(), "C")
    expected = _snapshot("c_states.json")
    produced = run_c_paths_contract(assembly, paths=AXIS_ORDER)
    assert {state["case_id"] for state in produced} == set(expected)
    worst = 0.0
    worst_case = None
    for state in produced:
        reference = expected[state["case_id"]]
        for key in ("deformation_left", "deformation_right"):
            actual = np.asarray(state[key], dtype=float)
            wanted = np.asarray(reference[key], dtype=float)
            for index in range(6):
                ratio = abs(actual[index] - wanted[index]) / _c_tolerance(
                    index, wanted[index]
                )
                if ratio > worst:
                    worst, worst_case = ratio, (state["case_id"], key, index)
    assert worst < 1.0, (
        f"contract C paths are {worst:.3f}x the strict gate tolerance at {worst_case}"
    )


def test_result_document_satisfies_the_result_schema() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="schema-k", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="schema-k",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    run = run_contract(model, case)
    validate_result(run.document)
    identity = run.document["case_identity"]
    assert len(identity["model_sha256"]) == 64
    assert len(identity["case_sha256"]) == 64
    # The expanded case list is the kernel's answer, not the document's: one
    # document asked for a one-point grid and got exactly that.
    assert [entry["name"] for entry in run.cases()] == ["k-w+0-r+0"]


def test_an_unimplemented_family_fails_closed() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="closed", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="closed",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    # A family the contract declares but the case layer does not implement has
    # to be refused by name, not silently run as something else.  `comparison`
    # is the stable choice: it is a comparison gate rather than a solver family,
    # so it is never going to be implemented here.
    case["family"] = "comparison"
    with pytest.raises(Exception, match="not implemented"):
        run_contract(model, case)


def test_the_case_layer_expands_the_grid_and_the_load_paths() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="expand", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="expand",
        wheel_values_mm=(-10.0, 10.0),
        rack_values_mm=(-5.0, 5.0),
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    run = run_contract(model, case)
    names = [entry["name"] for entry in run.cases()]
    assert names == ["k-w-10-r-5", "k-w-10-r+5", "k-w+10-r-5", "k-w+10-r+5"]
    bodies = run.document["manifest"]["bodies"]
    assert run.block("body_state").shape == (4 * len(DEFAULT_TIMES), len(bodies), 19)
    assert json.dumps(run.document, sort_keys=True)
