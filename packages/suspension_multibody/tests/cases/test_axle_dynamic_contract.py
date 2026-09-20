"""
The dynamic axle family through the contract boundary.

This family is the one with a frozen byte-for-byte hash, so the bar is not
"agrees to a tolerance": the contract path has to reproduce what the ctypes path
produced, exactly, for every case in the acceptance matrix.  Anything less would
mean the boundary changed the answer, and a boundary that changes the answer is
not a boundary.

The comparison covers every array the result publishes for every case in
`_CASE_DURATIONS`: body states, constraint wrenches, per-sample diagnostics, and
the element, tire and energy ledgers.  The ledgers were scratch while the flat
ABI was the only route to them, so their equality used to be *argued* from the
states; now that the contract carries them it is asserted, which is also what
lets the flat route be retired without losing a number.  The artifact-level hash
remains the end-to-end gate.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import run_axle_dynamics
from suspension_multibody.simulation import SimulationRequest, run_request

_SCRIPT = (
    Path(__file__).resolve().parents[2] / "scripts" / "run_axle_dynamics_acceptance.py"
)

#: The diagnostic row the kernel writes, in the order it writes it.  The names
#: are the ones `AxleRunDiagnostics` exposes for the same columns.
_DIAGNOSTIC_FIELDS = (
    "accepted",
    "internal_steps",
    "rejected_attempts",
    "newton_iterations",
    "minimum_accepted_step_s",
    "maximum_accepted_step_s",
    "last_accepted_step_s",
    "position_residual",
    "velocity_residual",
    "dynamics_residual",
    "active_contacts",
    "contact_events",
    "local_error_ratio",
    "energy_residual",
    "failure_code",
    "pinned_null_directions",
)


def _acceptance():
    """Load the acceptance script, which owns the frozen model and cases."""
    spec = importlib.util.spec_from_file_location("axle_acceptance_fixture", _SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_DRIVEN_FIXTURE = (
    Path(__file__).resolve().parents[1] / "axle_dynamics" / "test_driven_coordinate.py"
)


def _driven_fixture():
    """Load the driven-coordinate fixture, which owns that model and its case."""
    spec = importlib.util.spec_from_file_location("driven_fixture", _DRIVEN_FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_a_driven_coordinate_survives_the_contract_path() -> None:
    """
    A model with a driven coordinate must not lose its constraint row.

    The acceptance matrix does not cover this: its model declares no driven
    coordinates, so the case layer could drop them -- and did -- while every case
    in that matrix stayed bit-identical.  The *count* is what matters, because the
    wrench block is indexed by row: one missing row mislabels every row after it.
    The states agreed even when it was broken, because the axis was unloaded;
    a case that drives the coordinate is what makes the row observable.
    """
    fixture = _driven_fixture()
    model = fixture._translation_model(
        driven=(fixture._slide("slide"),), initial_velocity_m_per_s=(2.0, 0.0, 0.0)
    )
    case = fixture._case(
        samples=101,
        target=tuple(2.0 * 0.01 * index for index in range(101)),
        rate=tuple(2.0 for _ in range(101)),
    )
    reference = run_axle_dynamics(model, case)
    produced = run_request(
        SimulationRequest(
            assembly="axle", family="axle_dynamic", model=model, case=case
        )
    ).raw

    assert produced.block("constraint_wrench").shape == reference.constraint_wrench.shape, (
        "the contract path dropped a constraint row"
    )
    assert np.array_equal(produced.block("constraint_wrench"), reference.constraint_wrench)
    assert np.array_equal(produced.block("body_state"), reference.states)
    assert np.array_equal(produced.block("energy"), reference.energy)


def _diagnostics_matrix(result) -> np.ndarray:
    return np.column_stack(
        [
            np.asarray(getattr(result.diagnostics, field), dtype=float)
            for field in _DIAGNOSTIC_FIELDS
        ]
    )


@pytest.fixture(scope="module")
def acceptance():
    return _acceptance()


@pytest.mark.parametrize(
    "case_name",
    [
        "static_equilibrium",
        "road_pulse",
        "opposite_phase_road",
        "braking",
        "combined_load",
        "tire_liftoff_and_recontact",
    ],
)
def test_contract_path_reproduces_the_ctypes_path(acceptance, case_name: str) -> None:
    model = acceptance.build_axle_model()
    case = acceptance.build_case(case_name)
    reference = run_axle_dynamics(model, case)
    produced = run_request(
        SimulationRequest(
            assembly="axle", family="axle_dynamic", model=model, case=case
        )
    ).raw

    assert produced.status == "success"
    assert [entry["name"] for entry in produced.cases] == [case_name]
    assert np.array_equal(produced.block("body_state"), reference.states)
    for block, ledger in (
        ("constraint_wrench", reference.constraint_wrench),
        ("tire_output", reference.tire_output),
        ("spring_output", reference.spring_output),
        ("bushing_output", reference.bushing_output),
        ("energy", reference.energy),
    ):
        if ledger.shape[1] == 0:
            # A descriptor cannot express a zero extent, so an empty ledger is
            # an absent block rather than a block of nothing.
            assert block not in produced.blocks
            continue
        assert np.array_equal(produced.block(block), ledger), block
    assert np.array_equal(
        produced.block("constraint_wrench"), reference.constraint_wrench
    )
    sample_count = reference.times_s.size
    assert np.array_equal(
        produced.block("diagnostics")[:sample_count], _diagnostics_matrix(reference)
    )


def test_the_model_document_declares_metres(acceptance) -> None:
    from suspension_multibody.cases import axle_dynamic_model_document

    document, _ = axle_dynamic_model_document(acceptance.build_axle_model())
    # The dynamic family's own objects are SI, so the document says so and the
    # kernel scales by one; a millimetre round trip would cost an ulp per value.
    assert document["units"]["length"] == "m"


def test_the_case_document_describes_its_sample_tables(acceptance) -> None:
    from suspension_multibody.cases.axle_dynamic import case_document

    model = acceptance.build_axle_model()
    case = acceptance.build_case("opposite_phase_road")
    document, blob = case_document(model, case)
    assert document["family"] == "axle_dynamic"
    assert document["time"]["step_s"] == pytest.approx(1e-3)
    roles = [(entry["role"], entry.get("tire")) for entry in document["blobs"]]
    assert roles == [
        ("road_height", "tire_l"),
        ("road_velocity", "tire_l"),
        ("road_height", "tire_r"),
        ("road_velocity", "tire_r"),
    ]
    # Every descriptor has to point at a real range of the blob it travels in.
    for entry in document["blobs"]:
        assert entry["offset"] + entry["length"] <= len(blob)
        assert entry["dtype"] == "float64"


def test_an_unknown_blob_role_is_rejected(acceptance) -> None:
    from suspension_contracts import pack_container

    from suspension_multibody.cases import axle_dynamic_model_document
    from suspension_multibody.cases.axle_dynamic import case_document
    from suspension_multibody.kernel import KernelContractError
    from suspension_multibody.simulation import (
        CompilerRegistry,
        DocumentPairCompiler,
        SimulationRequest,
        run_request,
    )

    model = acceptance.build_axle_model()
    # `static_equilibrium` carries no sample tables at all, so this needs a
    # case that does before there is a role to corrupt.
    document, blob = case_document(model, acceptance.build_case("road_pulse"))
    document["blobs"][0]["role"] = "road_curvature"
    # The pair is authored here rather than by a family compiler, so the
    # document-pair compiler is what carries it; the kernel is still the layer
    # that has to refuse the role instead of running a case it cannot read.
    registry = CompilerRegistry()
    registry.register(DocumentPairCompiler("axle", "axle_dynamic"))
    with pytest.raises(KernelContractError, match="unknown blob role"):
        run_request(
            SimulationRequest(
                assembly="axle",
                family="axle_dynamic",
                model=axle_dynamic_model_document(model)[0],
                case=document,
                context={"case_payload": pack_container(document, blob)},
            ),
            registry=registry,
        )


def test_the_family_prepares_its_documents_through_the_default_registry(
    acceptance,
) -> None:
    """The domain path: the registry authors the documents the compiler frames."""
    from suspension_contracts import unpack_container

    from suspension_multibody.cases import axle_dynamic_model_document
    from suspension_multibody.cases.axle_dynamic import case_document
    from suspension_multibody.preparation.axle_dynamic import AxleDynamicPrepared
    from suspension_multibody.simulation import (
        compile_request,
        default_preparation_registry,
        prepare_request,
    )

    model = acceptance.build_axle_model()
    case = acceptance.build_case("static_equilibrium")
    request = SimulationRequest(
        assembly="axle", family="axle_dynamic", model=model, case=case
    )

    registry = default_preparation_registry()
    assert ("axle", "axle_dynamic") in registry.keys()
    result = prepare_request(request)

    assert isinstance(result.value, AxleDynamicPrepared)
    assert result.context["prepared_simulation"].value is result.value
    assert result.context["axle_dynamic_prepared"] is result.value
    # The compiler consumes the preparation: it frames the prepared documents
    # and their payloads into the submission.
    compiled = compile_request(result.request)
    assert compiled.model_document == result.value.model_document
    assert compiled.case_document == result.value.case_document
    assert unpack_container(compiled.model_payload) == (
        result.value.model_document,
        result.value.model_payload,
    )
    assert unpack_container(compiled.case_payload) == (
        result.value.case_document,
        result.value.case_payload,
    )
    assert axle_dynamic_model_document(model)[0] == result.value.model_document
    assert case_document(model, case)[0] == result.value.case_document


def test_a_document_request_bypasses_preparation_and_still_validates_identity(
    acceptance, monkeypatch
) -> None:
    from suspension_contracts import pack_container

    from suspension_multibody.cases import axle_dynamic_model_document
    from suspension_multibody.cases.axle_dynamic import case_document
    from suspension_multibody.preparation import axle_dynamic as axle_preparation
    from suspension_multibody.simulation import compile_request, prepare_request

    model = acceptance.build_axle_model()
    model_document, model_blob = axle_dynamic_model_document(model)
    case_document_emitted, case_blob = case_document(
        model, acceptance.build_case("road_pulse")
    )
    calls: list[SimulationRequest] = []
    monkeypatch.setattr(
        axle_preparation, "prepare_request", lambda request: calls.append(request)
    )
    context = {
        "model_payload": pack_container(model_document, model_blob),
        "case_payload": pack_container(case_document_emitted, case_blob),
    }
    request = SimulationRequest(
        assembly="axle",
        family="axle_dynamic",
        model=model_document,
        case=case_document_emitted,
        context=context,
    )

    bypassed = prepare_request(request)

    # The request already carries its documents, so no family preparation runs
    # and the axle model is not authored a second time.
    assert calls == []
    assert "prepared_simulation" not in bypassed.context
    compiled = compile_request(bypassed.request)
    assert compiled.model_document == model_document
    assert compiled.case_document == case_document_emitted
    # Bypassing preparation is not skipping the compiler: the request identity
    # is still validated.
    with pytest.raises(ValueError, match="request kind"):
        compile_request(
            SimulationRequest(
                assembly="axle",
                family="axle_dynamic",
                model=model_document,
                case=case_document_emitted,
                request_kind="wrong",
                context=context,
            )
        )


@pytest.mark.parametrize(
    ("target", "field", "bad_value"),
    [
        ("model", "contract", "wrong"),
        ("model", "contract_version", -1),
        ("model", "kind", "case"),
        ("case", "contract", "wrong"),
        ("case", "contract_version", -1),
        ("case", "kind", "model"),
        ("case", "family", "handling"),
    ],
)
@pytest.mark.parametrize("with_prepared", [False, True])
def test_document_bypass_rejects_invalid_contract_identity(
    target, field, bad_value, with_prepared, monkeypatch
) -> None:
    from suspension_contracts import CONTRACT_VERSION

    from suspension_multibody.simulation import (
        PreparationRegistry,
        compile_request,
        prepare_request,
    )

    model = {
        "contract": "multibody-model",
        "contract_version": CONTRACT_VERSION,
        "kind": "model",
    }
    case = {
        "contract": "multibody-case",
        "contract_version": CONTRACT_VERSION,
        "kind": "case",
        "family": "axle_dynamic",
    }
    (model if target == "model" else case)[field] = bad_value
    registry = PreparationRegistry()

    def refuse(*args):
        pytest.fail("document bypass must not consult preparation registry")

    monkeypatch.setattr(registry, "resolve", refuse)
    request = SimulationRequest(
        "axle", "axle_dynamic", model=model, case=case,
        context={"axle_dynamic_prepared": object()} if with_prepared else {},
    )
    prepared = prepare_request(request, registry=registry)
    assert prepared.request is request
    with pytest.raises(ValueError, match="requires"):
        compile_request(prepared.request)


@pytest.mark.parametrize("document_side", ["both", "model", "case"])
@pytest.mark.parametrize("staged", [False, True])
def test_document_request_runs_without_domain_decoding(
    acceptance, monkeypatch, document_side, staged
) -> None:
    from suspension_contracts import pack_container

    from suspension_multibody.cases import axle_dynamic_model_document
    from suspension_multibody.cases.axle_dynamic import case_document
    from suspension_multibody.preparation import axle_dynamic as axle_preparation
    from suspension_multibody.simulation import compile_request, prepare_request
    from suspension_multibody.simulation.backend import NativeContractBackend

    model = acceptance.build_axle_model()
    case = acceptance.build_case("road_pulse")
    model_doc, model_blob = axle_dynamic_model_document(model)
    case_doc, case_blob = case_document(model, case)

    def refuse_preparation(request):
        pytest.fail("document request must not repeat preparation")

    monkeypatch.setattr(axle_preparation, "prepare_request", refuse_preparation)
    request = SimulationRequest(
        "axle", "axle_dynamic",
        model=model if document_side == "case" else model_doc,
        case=case if document_side == "model" else case_doc,
        context={
            "model_payload": pack_container(model_doc, model_blob),
            "model_document": model_doc,
            "case_document": case_doc,
            "case_payload": pack_container(case_doc, case_blob),
            "axle_dynamic_prepared": object(),
        },
    )
    submissions = []

    class CountingBackend:
        def run(self, compiled):
            submissions.append(compiled)
            return NativeContractBackend().run(compiled)

    submitted = compile_request(prepare_request(request).request) if staged else request
    run = run_request(submitted, backend=CountingBackend())

    assert len(submissions) == 1
    assert run.status == "success"
    assert run.result is None
    assert run.raw.block("body_state").shape[0] == len(run.raw.times_s)
    assert len(run.raw.times_s) > 0
    assert np.isfinite(run.raw.block("body_state")).all()
