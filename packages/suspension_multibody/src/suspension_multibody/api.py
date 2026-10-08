"""Public document API for the single resolved multibody pipeline."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Mapping

from .authoring.documents import AssemblyDocument, SimulationAssembly
from .authoring.loader import CaseDocument, DocumentLoader
from .compilation.resolved import compile_resolved, plan_from_case
from .modeling.resolved import ResolvedSolvePlan
from .simulation.request import CompiledSimulation
from .simulation.runner import SimulationRun, run_compiled


def validate(
    assembly_document: str | Path | AssemblyDocument | SimulationAssembly,
    case_document: str | Path | Mapping[str, Any] | CaseDocument,
    *,
    case_payload: bytes = b"",
) -> CompiledSimulation:
    """Load and compile the same declarations accepted by every simulation."""
    source = assembly_document.assembly if isinstance(assembly_document, SimulationAssembly) else assembly_document
    bundle = DocumentLoader().load(source, case_document)
    case = bundle.case.to_payload()
    plan = (plan_from_case(case, case_payload) if case.get("contract") == "multibody-case"
        else ResolvedSolvePlan(case, case_payload))
    return compile_resolved(bundle.resolve(), plan)


def simulate(
    assembly_document: str | Path | AssemblyDocument | SimulationAssembly,
    case_document: str | Path | Mapping[str, Any] | CaseDocument,
) -> SimulationRun:
    """Submit one resolved assembly and case through the native backend."""
    return run_compiled(validate(assembly_document, case_document))


__all__ = ["simulate", "validate"]
