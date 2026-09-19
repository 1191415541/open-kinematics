"""Unified single-axle result adapters."""

from __future__ import annotations

from typing import Any

from ..axle_dynamics.result import AxleDynamicsResult

AxleResult = AxleDynamicsResult


def decode_axle_result(
    model: Any,
    case: Any,
    run: Any,
    *,
    stop: int | None = None,
) -> AxleDynamicsResult:
    """Decode a raw contract run through the existing axle result builder."""
    if isinstance(run, AxleDynamicsResult):
        return run
    from ..axle_dynamics.contract_run import build_result

    return build_result(model, case, run, stop=stop)


def axle_result_from_run(
    model: Any,
    case: Any,
    run: Any,
    *,
    stop: int | None = None,
) -> AxleDynamicsResult:
    """Named alias for callers that prefer result-oriented terminology."""
    return decode_axle_result(model, case, run, stop=stop)


__all__ = ["AxleDynamicsResult", "AxleResult", "axle_result_from_run", "decode_axle_result"]
