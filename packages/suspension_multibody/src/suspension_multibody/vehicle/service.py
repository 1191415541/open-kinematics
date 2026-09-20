"""
The high-level whole-vehicle dynamic execution service.

One call prepares the vehicle once, submits it through the unified runner, and
returns the typed result with its static wheel loads, metrics and native wall
time.  A native failure keeps its partial result and diagnostics instead of
collapsing into a bare message.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace

import numpy as np

from ..preparation.vehicle_dynamic import prepare_vehicle_run
from ..results.raw import RawContractResult
from ..results.vehicle import VehicleDynamicsResult
from ..schema import VehicleDynamicCase, VehicleModel
from ..simulation import SimulationRequest, run_request


def run_vehicle_dynamics(
    model: VehicleModel, case: VehicleDynamicCase
) -> VehicleDynamicsResult:
    """运行一个真实前后悬架、车身和轮端的 native 整车动力学算例."""
    from time import perf_counter

    from ..analysis.vehicle_physics import compute_static_wheel_loads
    from ..axle_dynamics.contract_run import safe_failure_row
    from ..axle_dynamics.errors import NativeAxleError
    from ..kernel import KernelContractError
    from ..metrics import compute_case_metrics
    from ..results.decoder import decode_result

    prepared = prepare_vehicle_run(model, case)
    static_wheel_loads: Mapping[str, float] | None = None
    try:
        static_wheel_loads = compute_static_wheel_loads(model).wheel_loads
    except (ValueError, np.linalg.LinAlgError):
        static_wheel_loads = None
    started = perf_counter()
    try:
        simulation_run = run_request(
            SimulationRequest(
                assembly="vehicle",
                family="vehicle_dynamic",
                model=model,
                case=case,
                context={"prepared": prepared},
            )
        )
    except KernelContractError as error:
        partial = error.partial_raw_result
        if not isinstance(partial, RawContractResult):
            raise NativeAxleError(str(error), status=3) from error
        manifest = partial.document.get("manifest", {})
        index = int(manifest.get("failed_sample_index", 0))
        partial_vehicle = None
        try:
            partial_vehicle = decode_result(
                partial,
                assembly="vehicle",
                family="vehicle_dynamic",
                prepared=prepared,
                stop=index,
            )
            partial_vehicle = replace(
                partial_vehicle,
                static_wheel_loads=static_wheel_loads,
                metrics=compute_case_metrics("vehicle_dynamic", partial_vehicle),
            )
        except Exception:
            partial_vehicle = None
        raise NativeAxleError(
            str(error),
            status=int(manifest.get("failed_status", 3) or 3),
            partial_result=partial_vehicle,
            failure_diagnostics=safe_failure_row(partial, index),
            failed_sample_index=index,
            failed_time_s=float(manifest.get("failed_time_s") or 0.0),
        ) from error

    result = simulation_run.result
    if not isinstance(result, VehicleDynamicsResult):
        raise TypeError("vehicle simulation did not produce VehicleDynamicsResult")
    completed = replace(
        result,
        native_kernel_wall_time_s=perf_counter() - started,
        static_wheel_loads=static_wheel_loads,
    )
    completed = replace(
        completed,
        metrics=compute_case_metrics("vehicle_dynamic", completed),
    )
    return completed


__all__ = ["run_vehicle_dynamics"]
