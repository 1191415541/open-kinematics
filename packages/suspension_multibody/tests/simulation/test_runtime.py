"""Resolved submissions retain identity and partial evidence."""
import json

import pytest

from suspension_multibody.kernel import KernelContractError
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.results.raw import RawContractResult
from suspension_multibody.simulation import SimulationRequest, run_compiled

from ._documents import PROTOCOLS, compiled, simple_compiled, synthetic_run


class FakeBackend:
    def __init__(self):
        self.compiled = None
        self.submissions = 0

    def run(self, submitted):
        self.compiled = submitted
        self.submissions += 1
        return synthetic_run(submitted)


def test_request_normalizes_dimensions_context_and_kind():
    request = SimulationRequest(assembly=" Machine ", family=" AXLE_DYNAMIC ",
        request_kind=" Probe ", context={"source": "test"})
    assert request.assembly == "machine"
    assert request.family == "axle_dynamic"
    assert request.kind == "probe"
    assert request.context == {"source": "test"}
    with pytest.raises(ValueError, match="assembly"):
        SimulationRequest(assembly="", family="axle_dynamic")


def test_compiled_submission_preserves_layout_metadata_and_payloads():
    submission = simple_compiled()
    assert submission.layout["document_order"] == ["model", "case"]
    assert submission.metadata["compiler"] == "ResolvedModelCompiler"
    assert submission.metadata["payload_schema"] == "contract_document_pair"
    assert all(isinstance(payload, bytes) for payload in submission.payloads())


def test_run_compiled_uses_the_backend_contract_exactly_once():
    submission, backend = simple_compiled(), FakeBackend()
    result = run_compiled(submission, backend=backend)
    assert result.status == "success"
    assert result.request is submission.request
    assert result.compiled is backend.compiled is submission
    assert backend.submissions == 1
    assert isinstance(result.result, ResultEnvelope)
    assert json.loads(json.dumps(dict(result.raw.model_document), default=dict)) == submission.model_document
    assert json.loads(json.dumps(dict(result.raw.case_document), default=dict)) == submission.case_document


@pytest.mark.parametrize("protocol", PROTOCOLS)
def test_every_protocol_uses_the_same_submitter_and_result_type(protocol):
    submission, backend = compiled(protocol), FakeBackend()
    result = run_compiled(submission, backend=backend)
    assert result.compiled.metadata["compiler"] == "ResolvedModelCompiler"
    assert isinstance(result.raw, RawContractResult)
    assert isinstance(result.result, ResultEnvelope)
    assert backend.submissions == 1


def test_run_compiled_attaches_partial_raw_result_before_reraising():
    class PartialBackend:
        def run(self, submitted):
            raise KernelContractError("solver stopped after partial progress",
                partial_run=synthetic_run(submitted, status="partial", manifest={"failed_sample_index": 1}))

    with pytest.raises(KernelContractError) as caught:
        run_compiled(simple_compiled(), backend=PartialBackend())
    assert isinstance(caught.value.partial_raw_result, RawContractResult)
    assert caught.value.partial_raw_result.status == "partial"
    assert caught.value.partial_raw_result.failure_evidence["failed_sample_index"] == 1
