"""One neutral adaptation feeds entity queries without family dispatch."""
import numpy as np
import pytest

from suspension_multibody.kernel import ContractRun
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.results.raw import RawContractResult, _decode_contract_run
from suspension_multibody.simulation import run_compiled

from ..simulation._documents import simple_compiled, synthetic_run


def test_native_adaptation_preserves_status_and_times():
    raw = _decode_contract_run(ContractRun(document={"status": "success", "manifest": {"bodies": ["body"]}},
        blocks={"body_state": np.zeros((2, 1, 19))}, model_document={"tires": []}, times_s=np.array([0, .1])))
    assert isinstance(raw, RawContractResult)
    assert raw.status == "success"
    assert raw.times_s.tolist() == [0, .1]


def test_native_adaptation_rejects_an_unrecognized_result():
    with pytest.raises(TypeError, match="ContractRun"):
        _decode_contract_run(object())


def test_envelope_preserves_raw_identity_without_a_compatibility_projection():
    submission = simple_compiled()
    raw = _decode_contract_run(synthetic_run(submission))
    result = ResultEnvelope(raw, submission.request.model)
    assert result.raw is raw
    assert result.model is submission.request.model
    assert result.body_ids == raw.body_names
    np.testing.assert_array_equal(result.body_state(result.body_ids[0]), raw.states[:, 0])


def test_every_result_type_is_the_uniform_envelope():
    from ..simulation._documents import PROTOCOLS, compiled

    class Backend:
        def run(self, value):
            return synthetic_run(value)

    for protocol in PROTOCOLS:
        result = run_compiled(compiled(protocol), backend=Backend())
        assert isinstance(result.result, ResultEnvelope)
        assert result.result.status == result.status
