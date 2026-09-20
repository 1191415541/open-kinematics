"""Common compile/submit orchestration for native contract simulations."""

from __future__ import annotations

from dataclasses import dataclass

from ..kernel import KernelContractError
from ..results.decoder import decode_result
from ..results.raw import RawContractResult
from .backend import NativeContractBackend, SimulationBackend
from .compiler import CompilerRegistry, compile_request
from .preparation import PreparationRegistry, prepare_request
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
        # The vehicle family publishes its preparation under this key; the
        # legacy ``prepared`` key is a migration input the adapter consumes and
        # never a decoding protocol.
        prepared=request.context.get("vehicle_dynamic_prepared"),
    )
    if isinstance(typed, RawContractResult):
        typed = None
    return SimulationRun(compiled=compiled, raw=raw, result=typed)


def run_request(
    request: SimulationRequest | CompiledSimulation,
    *,
    preparation_registry: PreparationRegistry | None = None,
    registry: CompilerRegistry | None = None,
    backend: SimulationBackend | None = None,
) -> SimulationRun:
    """Prepare and compile a domain request, or submit an already compiled one."""
    if isinstance(request, CompiledSimulation):
        return run_compiled(request, backend=backend)
    prepared = prepare_request(request, registry=preparation_registry)
    return run_compiled(
        compile_request(prepared.request, registry=registry),
        backend=backend,
    )


__all__ = ["SimulationRun", "run_compiled", "run_request"]
