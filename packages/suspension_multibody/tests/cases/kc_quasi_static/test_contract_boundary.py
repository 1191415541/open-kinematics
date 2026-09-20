"""
The contract boundary: two documents in, one result document out.

These tests are the acceptance for the boundary switch.  They run the same K
grid and C load paths the ctypes entry point runs, but they never build an
`AxleInput`: the model and the case travel as versioned documents and the
kernel expands the grid itself.  The numbers are compared against the same
frozen snapshot with the same tolerances, so "the new boundary agrees with the
old one" is a measurement rather than a claim.

The records the snapshots are scored against come from the production helpers
the Adams gates use; the case-layer runners that used to duplicate that
assembly are gone.
"""

from __future__ import annotations

import json
from collections.abc import Mapping

import numpy as np
import pytest
from suspension_contracts import validate_result

from suspension_multibody.adams.reference import _k_grid_states
from suspension_multibody.adams.strict_c import _c_path_records
from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.cases.kc_quasi_static import case_document, model_document
from suspension_multibody.cases.kc_quasi_static.load_paths import LoadPath
from suspension_multibody.cases.kc_quasi_static.workflow import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
)
from suspension_multibody.kernel import contract_version
from suspension_multibody.model import build_front_axle
from suspension_multibody.preparation.kc_quasi_static import (
    KcQuasiStaticCase,
    KcQuasiStaticPrepared,
)
from suspension_multibody.simulation import (
    CompilerRegistry,
    DocumentPairCompiler,
    SimulationRequest,
    compile_request,
    default_preparation_registry,
    prepare_request,
    run_request,
)

from .kc_fixtures import _c_tolerance, _compliant_model, _k_tolerance, _snapshot


def _run(model: dict, case: dict):
    """Run one K+C document pair through the unified simulation runner."""
    return run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=model,
            case=case,
        )
    ).raw


def _plain_document(value):
    """
    Return the JSON value tree behind a read-only result document.

    ``RawContractResult`` freezes what the kernel wrote, so the schema and JSON
    checks unwind the mapping proxies before reading the document.
    """
    if isinstance(value, Mapping):
        return {key: _plain_document(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain_document(item) for item in value]
    return value


def test_contract_entry_point_reports_its_version() -> None:
    assert contract_version() == 1


def test_k_grid_through_the_contract_boundary_matches_the_snapshot() -> None:
    assembly = build_front_axle(benchmark_model(), "K")
    expected = _snapshot("k_states.json")
    produced = _k_grid_states(
        assembly,
        wheel_values_mm=(-10.0, 0.0, 10.0),
        rack_values_mm=(-5.0, 0.0, 5.0),
    )
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
    produced = [
        record
        for path in LoadPath.standard()
        for record in _c_path_records(assembly, path)
    ]
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
    run = _run(model, case)
    validate_result(_plain_document(run.document))
    identity = run.document["case_identity"]
    assert len(identity["model_sha256"]) == 64
    assert len(identity["case_sha256"]) == 64
    # The expanded case list is the kernel's answer, not the document's: one
    # document asked for a one-point grid and got exactly that.
    assert [entry["name"] for entry in run.cases] == ["k-w+0-r+0"]


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
    # The family is declared in the contract but has no compiler behind it, and
    # the kernel is the layer that has to refuse it by name.  Carrying the pair
    # through a document-pair compiler is what reaches that refusal; the runner
    # itself would stop one step earlier, at "no compiler registered".
    registry = CompilerRegistry()
    registry.register(DocumentPairCompiler("axle", "comparison"))
    with pytest.raises(Exception, match="not implemented"):
        run_request(
            SimulationRequest(
                assembly="axle",
                family="comparison",
                model=model,
                case=case,
            ),
            registry=registry,
        )


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
    run = _run(model, case)
    names = [entry["name"] for entry in run.cases]
    assert names == ["k-w-10-r-5", "k-w-10-r+5", "k-w+10-r-5", "k-w+10-r+5"]
    bodies = run.document["manifest"]["bodies"]
    assert run.block("body_state").shape == (4 * len(DEFAULT_TIMES), len(bodies), 19)
    assert json.dumps(_plain_document(run.document), sort_keys=True)


def test_the_family_prepares_the_k_documents_through_the_default_registry() -> None:
    """The domain path: an assembly and a K case, prepared and then compiled."""
    assembly = build_front_axle(benchmark_model(), "K")
    request = SimulationRequest(
        assembly="axle",
        family="kc_quasi_static",
        model=assembly,
        case=KcQuasiStaticCase(
            name="prepared-k",
            wheel_values_mm=(0.0,),
            rack_values_mm=(0.0,),
            times_s=DEFAULT_TIMES,
            settings=DEFAULT_SETTINGS,
            drive_wheels=True,
        ),
    )

    registry = default_preparation_registry()
    assert ("axle", "kc_quasi_static") in registry.keys()
    result = prepare_request(request)

    assert isinstance(result.value, KcQuasiStaticPrepared)
    assert result.context["prepared_simulation"].value is result.value
    assert result.context["kc_assembly"] is assembly
    assert result.context["model_document"] == result.value.model_document
    assert result.context["case_document"]["family"] == "kc_quasi_static"
    # The compiler consumes the prepared context rather than authoring anything.
    compiled = compile_request(result.request)
    assert compiled.model_document == result.value.model_document
    assert compiled.case_document == result.value.case_document
    # And the submission is one the kernel accepts: the one-point grid the case
    # states is the one it expands.
    run = run_request(request).raw
    assert run.status == "success"
    assert [entry["name"] for entry in run.cases] == ["k-w+0-r+0"]


def test_a_document_request_bypasses_preparation_and_still_validates_the_contract(
    monkeypatch,
) -> None:
    from suspension_multibody.preparation import kc_quasi_static as kc_preparation

    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="bypass", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="bypass",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    calls: list[SimulationRequest] = []
    monkeypatch.setattr(
        kc_preparation, "prepare_request", lambda request: calls.append(request)
    )
    request = SimulationRequest(
        assembly="axle", family="kc_quasi_static", model=model, case=case
    )

    bypassed = prepare_request(request)

    # The request already carries its documents, so no family preparation runs
    # and the axle is not assembled a second time.
    assert calls == []
    assert "prepared_simulation" not in bypassed.context
    compiled = compile_request(bypassed.request)
    assert compiled.case_document == case
    # Bypassing preparation is not skipping the compiler: the documents still
    # have to satisfy the family's contract identity.
    wrong_family = dict(case)
    wrong_family["family"] = "comparison"
    with pytest.raises(ValueError, match="case family"):
        compile_request(
            SimulationRequest(
                assembly="axle",
                family="kc_quasi_static",
                model=model,
                case=wrong_family,
            )
        )
