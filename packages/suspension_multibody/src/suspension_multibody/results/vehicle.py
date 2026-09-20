"""
Vehicle result type and its family result adapter.

This module owns the whole-vehicle reporting type and the two mapping helpers
that turn one native contract run into it.  It is a *family adapter*: the
unified raw-to-typed dispatcher stays in :mod:`suspension_multibody.results.decoder`,
and nothing here prepares a request or submits anything to native.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np

from ..axle_dynamics.result import AxleDynamicsResult
from ..preparation.vehicle_dynamic import _PRESCRIBED_STEERING_TYPES
from .decoder import decode_result
from .raw import RawContractResult


@dataclass(frozen=True)
class VehicleDynamicsResult:
    """整车运行结果；底层状态和诊断保持 native axle 结果协议."""

    axle: AxleDynamicsResult
    steering_names: tuple[str, ...] = ()
    steering_output: np.ndarray | None = None
    native_kernel_wall_time_s: float = 0.0
    static_wheel_loads: Mapping[str, float] | None = None
    metrics: dict[str, Any] = field(default_factory=dict)

    @property
    def times_s(self) -> np.ndarray:
        return self.axle.times_s

    @property
    def body_names(self) -> tuple[str, ...]:
        return self.axle.body_names

    @property
    def tire_names(self) -> tuple[str, ...]:
        return self.axle.tire_names

    @property
    def states(self) -> np.ndarray:
        """返回所有刚体的状态数组."""
        return self.axle.states

    @property
    def diagnostics(self):
        return self.axle.diagnostics

    @property
    def performance(self):
        return self.axle.performance

    def body_state(self, body: str) -> np.ndarray:
        return self.axle.body_state(body)

    def tire_state(self, tire: str) -> np.ndarray:
        return self.axle.tire_state(tire)

    def steering_state(self, actuator: str) -> np.ndarray:
        if self.steering_output is None:
            raise KeyError("this run has no steering actuator output")
        try:
            index = self.steering_names.index(actuator)
        except ValueError as exc:
            raise KeyError(f"unknown steering actuator {actuator!r}") from exc
        return self.steering_output[:, index, :]

    def joint_wrench(self, joint: str) -> np.ndarray:
        """返回一个约束在 body_b 上的世界坐标系力和力矩."""
        return self.axle.joint_wrench_on_body_b(joint)

    def spring_state(self, spring: str) -> np.ndarray:
        """返回一个弹簧阻尼器的长度、速度和力分量."""
        return self.axle.spring_state(spring)

    def bushing_state(self, bushing: str) -> np.ndarray:
        """返回一个衬套的局部变形和力."""
        return self.axle.bushing_state(bushing)


VehicleResult = VehicleDynamicsResult


def _contract_constraint_names(prepared) -> tuple[str, ...]:
    """Return the kernel's constraint-row names in contract order."""
    return (
        *(joint.name for joint in prepared.native_model.joints),
        *(
            prepared.steering.names[index]
            for index, kind in enumerate(prepared.steering.actuator_type)
            if int(kind) in _PRESCRIBED_STEERING_TYPES
        ),
        *(driven.name for driven in prepared.native_model.driven_coordinates),
    )


def _vehicle_axle_result(prepared, run, *, stop: int | None = None):
    """Map the common contract result blocks through the axle reporting adapter."""
    # The axle adapter owns the diagnostics, performance and ledger layout.  A
    # vehicle differs only in that a prescribed steering actuator adds a
    # constraint row, so reuse the adapter and replace that one name list rather
    # than maintaining a second, subtly different decoder here.
    from ..axle_dynamics.contract_run import build_result

    result = build_result(
        prepared.native_model,  # ty: ignore[invalid-argument-type]
        prepared.axle_case,
        run,
        stop=stop,
    )
    return replace(result, constraint_names=_contract_constraint_names(prepared))


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
