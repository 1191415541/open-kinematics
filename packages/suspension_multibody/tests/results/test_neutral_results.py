from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.cases.kc_quasi_static import case_document, model_document
from suspension_multibody.cases.kc_quasi_static.workflow import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
)
from suspension_multibody.kernel import ContractRun, KernelContractError, run_contract
from suspension_multibody.model import build_front_axle
from suspension_multibody.results import (
    ChannelRegistry,
    CommonResult,
)
from suspension_multibody.results.raw import decode_contract_run


def _run() -> ContractRun:
    assembly = build_front_axle(benchmark_model(), "K")
    model = model_document(assembly, name="result-test", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="result-test",
        wheel_values_mm=(0.0,),
        rack_values_mm=(0.0,),
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    return run_contract(model, case)


def test_neutral_result_decodes_common_contract_surface() -> None:
    result = decode_contract_run(_run())

    assert isinstance(result, CommonResult)
    assert result.status == "success"
    assert result.times_s.shape == (len(DEFAULT_TIMES),)
    assert result.body_names
    assert result.states.shape[0] == len(DEFAULT_TIMES)
    assert result.diagnostics is not None
    assert result.diagnostics.shape[0] == len(DEFAULT_TIMES)
    assert result.performance["available"] is False
    assert "residual_calls" in result.performance
    with pytest.raises(KernelContractError, match="no block"):
        result.block("missing")


def test_neutral_result_preserves_model_metadata_and_read_only_arrays() -> None:
    result = decode_contract_run(_run())

    assert result.tire_names == ()
    with pytest.raises(ValueError):
        result.states[0, 0, 0] = 0.0
    with pytest.raises(TypeError):
        result.document["status"] = "failed"


def test_channel_registry_uses_frozen_contract_order() -> None:
    registry = ChannelRegistry.load()

    assert registry.schema_version == 1
    assert registry.rotation_sign == "right_hand_rule"
    assert registry.channel_names
    assert registry.channel_names[0] == "sprung_body.heave"
    assert registry.channel("left.wheel_center_z")["unit"] == "m"
    with pytest.raises(TypeError):
        registry.contract["schema_version"] = 2


def test_neutral_result_tire_metadata_and_block_access() -> None:
    run = ContractRun(
        document={
            "status": "success",
            "manifest": {
                "bodies": ["body"],
                "cases": [{"sample_offset": 0, "sample_count": 2}],
            },
        },
        blocks={
            "body_state": np.zeros((2, 1, 19)),
            "diagnostics": np.full((4, 16), np.nan),
            "tire_output": np.ones((2, 1, 41)),
        },
        model_document={"tires": [{"name": "tire_L"}]},
        times_s=np.array([0.0, 0.1]),
    )
    result = decode_contract_run(run)

    assert result.tire_names == ("tire_L",)
    assert result.tire_state("tire_L").shape == (2, 41)
