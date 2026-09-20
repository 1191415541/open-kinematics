from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.kernel import ContractRun
from suspension_multibody.results import (
    AxleResult,
    decode_result,
    decoder_for,
)
from suspension_multibody.results.raw import RawContractResult


def _run() -> ContractRun:
    return ContractRun(
        document={"status": "success", "manifest": {"bodies": ["body"]}},
        blocks={"body_state": np.zeros((2, 1, 19))},
        model_document={"tires": []},
        case_document={"family": "probe"},
        times_s=np.array([0.0, 0.1]),
    )


def test_decoder_falls_back_to_neutral_result_without_typed_inputs() -> None:
    result = decode_result(_run(), assembly="axle", family="axle_dynamic")

    assert isinstance(result, RawContractResult)
    assert result.status == "success"
    assert result.times_s.tolist() == [0.0, 0.1]


def test_decoder_for_binds_normalized_dimensions() -> None:
    result = decoder_for(" AXLE ", " probe ")(_run())

    assert isinstance(result, RawContractResult)
    with pytest.raises(TypeError, match="ContractRun"):
        decoder_for("axle", "probe")(object())


def test_axle_result_alias_preserves_existing_result_type() -> None:
    from suspension_multibody.axle_dynamics.result import AxleDynamicsResult

    assert AxleResult is AxleDynamicsResult
def test_vehicle_decoder_preserves_axle_and_steering_compatibility(monkeypatch) -> None:
    from types import SimpleNamespace

    from suspension_multibody.results import decode_vehicle_result
    from suspension_multibody.results.vehicle import VehicleDynamicsResult

    axle = SimpleNamespace()
    prepared = SimpleNamespace(steering=SimpleNamespace(names=("rack",)))
    run = ContractRun(
        document={"status": "success", "manifest": {"bodies": ["body"]}},
        blocks={
            "body_state": np.zeros((2, 1, 19)),
            "steering_output": np.array([[[1.0]], [[2.0]]]),
        },
        model_document={"tires": []},
        times_s=np.array([0.0, 0.1]),
    )
    monkeypatch.setattr(
        "suspension_multibody.results.vehicle._vehicle_axle_result",
        lambda _prepared, _run: axle,
    )

    result = decode_vehicle_result(prepared, run, native_kernel_wall_time_s=1.5)

    assert isinstance(result, VehicleDynamicsResult)
    assert result.axle is axle
    assert result.steering_names == ("rack",)
    assert result.steering_state("rack").ravel().tolist() == [1.0, 2.0]
    assert result.native_kernel_wall_time_s == 1.5
