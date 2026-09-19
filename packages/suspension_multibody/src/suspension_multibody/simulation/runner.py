"""Common compile/submit orchestration for native contract simulations."""

from __future__ import annotations

from dataclasses import dataclass

from ..kernel import KernelContractError
from ..results.decoder import decode_result
from ..results.raw import RawContractResult
from .backend import NativeContractBackend, SimulationBackend
from .compiler import CompilerRegistry, compile_request
from .request import CompiledSimulation, SimulationRequest


@dataclass(frozen=True)
class SimulationRun:
    """Compiled request, neutral raw result, and optional typed result."""

    compiled: CompiledSimulation
    raw: RawContractResult
    result: object | None = None

    @property
    def request(self) -> SimulationRequest:
        return self.compiled.request

    @property
    def status(self) -> str:
        return self.raw.status


def run_compiled(
    compiled: CompiledSimulation,
    *,
    backend: SimulationBackend | None = None,
) -> SimulationRun:
    """Submit an already-compiled request and normalize its raw result."""
    active_backend = NativeContractBackend() if backend is None else backend
    try:
        result = active_backend.run(compiled)
    except KernelContractError as error:
        partial = error.partial_run
        if partial is not None:
            error.partial_raw_result = decode_result(
                partial,
                assembly=compiled.assembly,
                family=compiled.family,
            )
        raise

    raw = decode_result(
        result,
        assembly=compiled.assembly,
        family=compiled.family,
    )
    if not isinstance(raw, RawContractResult):
        raise TypeError("simulation backend raw decode must return RawContractResult")

    request = compiled.request
    typed = decode_result(
        raw,
        assembly=compiled.assembly,
        family=compiled.family,
        model=request.model,
        case=request.case,
        prepared=request.context.get("prepared"),
    )
    if isinstance(typed, RawContractResult):
        typed = None
    return SimulationRun(compiled=compiled, raw=raw, result=typed)


def run_request(
    request: SimulationRequest,
    *,
    registry: CompilerRegistry | None = None,
    backend: SimulationBackend | None = None,
) -> SimulationRun:
    """Compile and submit one request without interpreting business metrics."""
    return run_compiled(
        compile_request(request, registry=registry),
        backend=backend,
    )


__all__ = ["SimulationRun", "run_compiled", "run_request"]
