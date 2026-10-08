"""Neutral native channels remain immutable and independent of protocol."""
import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.kernel import ContractRun, KernelContractError
from suspension_multibody.results import ChannelRegistry, ResultEnvelope
from suspension_multibody.results.raw import _decode_contract_run

from ..simulation._documents import documents


def test_neutral_result_decodes_common_contract_surface():
    result = simulate(*documents()).result
    assert isinstance(result, ResultEnvelope)
    assert result.status == "success"
    assert result.times_s.shape == (3,)
    assert result.body_ids
    assert result.named_blocks["body_state"].shape[0] == 3
    assert result.diagnostics.shape[0] == 3
    assert "residual_calls" in result.performance
    with pytest.raises(KernelContractError, match="no block"):
        result.raw.block("missing")


def test_neutral_result_preserves_model_metadata_and_read_only_arrays():
    result = simulate(*documents()).result
    assert result.tire_ids == ("wheel.tire",)
    assert result.raw.model_document["tires"][0]["name"] == result.tire_ids[0]
    with pytest.raises(ValueError):
        result.named_blocks["body_state"][0, 0, 0] = 0
    with pytest.raises(TypeError):
        result.raw.document["status"] = "failed"


def test_channel_registry_uses_frozen_contract_order():
    registry = ChannelRegistry.load()
    assert registry.schema_version == 1
    assert registry.rotation_sign == "right_hand_rule"
    assert registry.channel_names[0] == "sprung_body.heave"
    assert registry.channel("left.wheel_center_z")["unit"] == "m"
    with pytest.raises(TypeError):
        registry.contract["schema_version"] = 2


def test_neutral_result_tire_metadata_and_block_access():
    run = ContractRun(document={"status": "success", "manifest": {
        "bodies": ["body"], "cases": [{"sample_offset": 0, "sample_count": 2}]}},
        blocks={"body_state": np.zeros((2, 1, 19)), "diagnostics": np.full((4, 16), np.nan),
                "tire_output": np.ones((2, 1, 41))},
        model_document={"tires": [{"name": "tire_L"}]}, times_s=np.array([0, .1]))
    raw = _decode_contract_run(run)
    assert raw.tire_names == ("tire_L",)
    assert raw.tire_state("tire_L").shape == (2, 41)
