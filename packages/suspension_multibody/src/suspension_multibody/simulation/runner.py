"""Common compile/submit orchestration for native contract simulations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

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


def compiled_from_plan(
    plan: Any,
    model: Any,
    *,
    request: SimulationRequest | None = None,
    registry: Any = None,
) -> CompiledSimulation:
    """
    Compile a solve plan and a model into one submission.

    This is the orchestration the unified pipeline runs on: the *plan* says what
    the run is and the *model* says what it is run on, and neither of them has to
    be a family name for the compilation to resolve.  The request is kept beside
    the documents so every downstream reader -- the result decoder included --
    asks the same object for the routing facts.

    Returning a :class:`CompiledSimulation` rather than documents is what lets
    the existing runner and backend submit it unchanged: there is one submission
    type and one place it is built.
    """
    from ..compilation.compile import compile_plan

    model_document, case_document, model_payload, case_payload, metadata = compile_plan(
        plan, model, registry=registry
    )
    active = request
    if active is None:
        active = SimulationRequest(
            assembly=_assembly_of(model),
            rig=plan.rig,
            family=plan.family,
            study=plan.study,
            model=model,
            name=plan.inputs.name,
            outputs=plan.outputs,
        )
    return CompiledSimulation(
        request=active,
        model_document=model_document,
        case_document=case_document,
        model_payload=model_payload,
        case_payload=case_payload,
        layout={"document_order": ["model", "case"], "payload_order": ["model", "case"]},
        metadata=metadata,
    )


def run_plan(
    plan: Any,
    model: Any,
    *,
    request: SimulationRequest | None = None,
    registry: Any = None,
    backend: SimulationBackend | None = None,
) -> SimulationRun:
    """Compile a plan and a model, then submit it through the common backend."""
    return run_compiled(
        compiled_from_plan(plan, model, request=request, registry=registry),
        backend=backend,
    )


def _assembly_of(model: Any) -> str:
    """Return the assembly a model belongs to, for a request built from a plan."""
    from ..modeling.assembly import SimulationAssembly

    if isinstance(model, SimulationAssembly):
        return model.root_kind or "axle"
    root_kind = getattr(model, "root_kind", "")
    if root_kind:
        return str(root_kind)
    return "vehicle" if hasattr(model, "wheels") else "axle"


__all__ = [
    "SimulationRun",
    "compiled_from_plan",
    "run_compiled",
    "run_plan",
    "run_request",
]
