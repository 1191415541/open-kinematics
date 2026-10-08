"""One compiler frames all resolved graphs and retains native failure evidence."""
from dataclasses import replace

import numpy as np
import pytest
from suspension_contracts import unpack_container

from suspension_multibody.api import validate
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from suspension_multibody.kernel import KernelContractError
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.simulation import run_compiled

from ._documents import PROTOCOLS, compiled, documents, simple_compiled, synthetic_run


@pytest.mark.parametrize("protocol", PROTOCOLS)
def test_document_compiler_materializes_contracts_and_metadata(protocol):
    source, case = documents(protocol)
    result = validate(source, case)
    assert result.case_document["family"] == protocol
    assert result.request.assembly == "generic"
    assert result.request.model.name == source.name
    assert result.layout == {"document_order": ["model", "case"], "payload_order": ["model", "case"]}
    assert result.metadata["compiler"] == "ResolvedModelCompiler"
    assert result.metadata["family"] == protocol
    assert result.metadata["model_fingerprint"] == result.request.model.fingerprint
    for payload, document in zip(result.payloads(), result.documents(), strict=True):
        assert isinstance(payload, bytes)
        assert unpack_container(payload)[0] == document


def test_compiler_preserves_pinned_binary_resources():
    from ..authoring.test_unified_document_loader import EXAMPLES

    bundle = DocumentLoader(resource_root=EXAMPLES).load("tire.assembly.json", "dynamic.case.json")
    model = bundle.resolve()
    result = compile_resolved(model, plan_from_case(bundle.case.to_payload()))
    assert unpack_container(result.model_payload)[1] == model.resource_payload
    assert model.resource_payload
    assert result.metadata["resource_manifest"] == model.to_document()["resource_manifest"]


def test_compiler_checks_contract_version_and_explicit_protocol_identity():
    source, case = documents()
    with pytest.raises(AuthoringError, match="contract_version"):
        validate(source, {**case, "contract_version": 2})
    result = validate(source, case)
    with pytest.raises(ValueError, match="disagrees"):
        compile_resolved(result.request.model, result.request.case,
            request=replace(result.request, family="vehicle_dynamic"))


@pytest.mark.parametrize("protocol", ["axle_dynamic", "vehicle_dynamic"])
def test_compiler_frames_the_same_resolved_entities(protocol):
    result = compiled(protocol)
    graph = result.request.model.to_document()
    assert result.model_document["bodies"] == graph["bodies"]
    assert result.model_document["joints"] == graph["joints"]
    assert result.model_document["elements"] == graph["elements"]
    assert unpack_container(result.model_payload)[0] == result.model_document
    assert unpack_container(result.case_payload)[0] == result.case_document


def test_compiler_never_mutates_the_source_ir_when_adding_driven_joint():
    from ..physics.test_wheel_spin_boundary import plan, wheel_assembly

    model = DocumentLoader().load(wheel_assembly(), plan().to_document()).resolve()
    given = model.to_document()
    result = compile_resolved(model, plan())
    assert model.to_document() == given
    assert result.model_document["joints"][:-1] == given["joints"]
    assert result.model_document["joints"][-1]["type"] == "driven_rotation"
    assert result.metadata["model_fingerprint"] == model.fingerprint


def test_compiler_requires_resolved_inputs():
    result = simple_compiled()
    with pytest.raises(TypeError, match="ResolvedModel"):
        compile_resolved(object(), result.request.case)
    with pytest.raises(TypeError, match="ResolvedSolvePlan"):
        compile_resolved(result.request.model, object())
    assert isinstance(result.request.model, ResolvedModel)
    assert isinstance(result.request.case, ResolvedSolvePlan)


def test_run_compiled_preserves_partial_status_diagnostics_and_performance():
    submission = simple_compiled()
    diagnostics = np.zeros((3, 16))
    diagnostics[1, 0] = 1

    class PartialBackend:
        def run(self, submitted):
            return synthetic_run(submitted, status="partial", diagnostics=diagnostics)

    result = run_compiled(submission, backend=PartialBackend())
    assert result.status == "partial"
    assert result.raw.diagnostics.shape == (1, 16)
    assert result.raw.performance["available"] is True


def test_run_compiled_preserves_failed_raw_result():
    class FailedBackend:
        def run(self, submitted):
            return synthetic_run(submitted, status="failed", manifest={"failure_message": "trim failed"})

    result = run_compiled(simple_compiled(), backend=FailedBackend())
    assert result.status == "failed"
    assert result.raw.metadata["failure_message"] == "trim failed"


def test_native_backend_attaches_partial_raw_result(monkeypatch):
    from suspension_multibody.simulation import backend

    submission = simple_compiled()
    diagnostics = np.zeros((3, 16))
    diagnostics[1, 0] = 1
    partial = synthetic_run(submission, status="partial", diagnostics=diagnostics)

    def raise_partial(*args, **kwargs):
        raise KernelContractError("trim failed", partial_run=partial)

    monkeypatch.setattr(backend, "run_contract", raise_partial)
    with pytest.raises(KernelContractError) as caught:
        run_compiled(submission)
    error = caught.value
    assert error.partial_raw_result is not None
    assert error.partial_raw_result.status == "partial"
    assert error.partial_raw_result.performance["available"] is True
    assert error.partial_raw_result.diagnostics.shape == (1, 16)
