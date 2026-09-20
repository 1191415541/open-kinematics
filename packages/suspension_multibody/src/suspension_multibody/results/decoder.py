"""Result decoding dispatch by assembly and case family."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

from ..kernel import ContractRun
from .raw import RawContractResult, _decode_contract_run

Decoder = Callable[..., Any]


def decode_result(
    run: ContractRun | RawContractResult | Any,
    *,
    assembly: str,
    family: str,
    model: Any = None,
    case: Any = None,
    prepared: Any = None,
    stop: int | None = None,
    steering_names: tuple[str, ...] | None = None,
    steering_output: Any = None,
    native_kernel_wall_time_s: float = 0.0,
) -> Any:
    """Decode a run into the strongest compatible result type available."""
    normalized_assembly = str(assembly).strip().lower()
    normalized_family = str(family).strip().lower()

    if (
        normalized_assembly == "axle"
        and normalized_family == "axle_dynamic"
        and model is not None
        and case is not None
    ):
        from ..axle_dynamics.schema import AxleDynamicsCase, AxleDynamicsModel
        from .axle import decode_axle_result

        if isinstance(model, AxleDynamicsModel) and isinstance(case, AxleDynamicsCase):
            return decode_axle_result(model, case, run, stop=stop)

    if (
        normalized_assembly == "vehicle"
        and normalized_family == "vehicle_dynamic"
        and prepared is not None
    ):
        from .vehicle import decode_vehicle_result

        return decode_vehicle_result(
            prepared,
            run,
            steering_names=steering_names,
            steering_output=steering_output,
            native_kernel_wall_time_s=native_kernel_wall_time_s,
            stop=stop,
        )

    if isinstance(run, RawContractResult):
        return run
    return _decode_contract_run(run)


def decoder_for(assembly: str, family: str) -> Decoder:
    """Return a small callable that fixes the assembly/family selection."""
    normalized_assembly = str(assembly).strip().lower()
    normalized_family = str(family).strip().lower()

    def decode(run: Any, **kwargs: Any) -> Any:
        return decode_result(
            run,
            assembly=normalized_assembly,
            family=normalized_family,
            **kwargs,
        )

    return decode


__all__ = ["Decoder", "decoder_for", "decode_result"]
