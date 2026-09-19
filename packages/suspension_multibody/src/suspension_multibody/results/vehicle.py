"""Unified vehicle result adapter with compatibility-preserving fields."""

from __future__ import annotations

from typing import Any

from ..vehicle_dynamics import VehicleDynamicsResult
from .decoder import decode_result
from .raw import RawContractResult

VehicleResult = VehicleDynamicsResult


def decode_vehicle_result(
    prepared: Any,
    run: Any,
    *,
    steering_names: tuple[str, ...] | None = None,
    steering_output: Any = None,
    native_kernel_wall_time_s: float = 0.0,
    stop: int | None = None,
) -> VehicleResult:
    """
    Decode one vehicle contract run into the existing vehicle result type.

    The current vehicle model intentionally remains an explicit multibody result
    with one compatibility ``axle`` view; this adapter does not invent separate
    front/rear reduced-order results that the native output does not provide.
    """
    from ..vehicle_dynamics import VehicleDynamicsResult, _vehicle_axle_result

    if stop is None:
        axle = _vehicle_axle_result(prepared, run)
    else:
        axle = _vehicle_axle_result(prepared, run, stop=stop)
    raw = run if isinstance(run, RawContractResult) else decode_result(
        run,
        assembly="vehicle",
        family="vehicle_dynamic",
    )
    if steering_output is None and "steering_output" in raw.named_blocks:
        steering_output = raw.block("steering_output").copy()
    if steering_names is None:
        steering_names = tuple(getattr(prepared.steering, "names", ()))
    return VehicleDynamicsResult(
        axle=axle,
        steering_names=tuple(steering_names),
        steering_output=steering_output,
        native_kernel_wall_time_s=float(native_kernel_wall_time_s),
    )


def vehicle_result_from_run(
    prepared: Any,
    run: Any,
    *,
    steering_names: tuple[str, ...] | None = None,
    steering_output: Any = None,
    native_kernel_wall_time_s: float = 0.0,
) -> VehicleResult:
    """Named alias for callers that prefer result-oriented terminology."""
    return decode_vehicle_result(
        prepared,
        run,
        steering_names=steering_names,
        steering_output=steering_output,
        native_kernel_wall_time_s=native_kernel_wall_time_s,
    )


__all__ = [
    "VehicleDynamicsResult",
    "VehicleResult",
    "decode_vehicle_result",
    "vehicle_result_from_run",
]
