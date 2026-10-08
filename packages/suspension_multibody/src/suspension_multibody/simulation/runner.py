"""Submit one resolved native contract and expose its uniform result."""

from __future__ import annotations

from dataclasses import dataclass

from ..kernel import KernelContractError
from ..modeling.resolved import ResolvedModel
from ..results.envelope import ResultEnvelope
from ..results.raw import RawContractResult, _decode_contract_run
from .backend import NativeContractBackend, SimulationBackend
from .request import CompiledSimulation, SimulationRequest


@dataclass(frozen=True)
class SimulationRun:
    """The compiled submission and the native channels produced by that submission."""

    compiled: CompiledSimulation
    raw: RawContractResult
    result: ResultEnvelope

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
    """Submit a resolved model; decoding is independent of study and subsystem roles."""
    model = compiled.request.model
    if not isinstance(model, ResolvedModel):
        raise TypeError("native submission requires a ResolvedModel")
    active_backend = NativeContractBackend() if backend is None else backend
    try:
        native = active_backend.run(compiled)
    except KernelContractError as error:
        if error.partial_run is not None:
            error.partial_raw_result = _decode_contract_run(error.partial_run)
        raise
    raw = _decode_contract_run(native)
    return SimulationRun(compiled, raw, ResultEnvelope(raw, model))


__all__ = ["SimulationRun", "run_compiled"]
