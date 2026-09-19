"""Neutral native backends for compiled contract submissions."""

from __future__ import annotations

from typing import Protocol, runtime_checkable

from ..kernel import ContractRun, run_contract
from .request import CompiledSimulation


@runtime_checkable
class SimulationBackend(Protocol):
    """Backend protocol independent of case-family business semantics."""

    def run(self, compiled: CompiledSimulation) -> ContractRun:
        """Submit a compiled model/case pair and return the parsed raw run."""
        ...


class NativeContractBackend:
    """Submit compiled documents through the existing shared kernel runtime."""

    def run(self, compiled: CompiledSimulation) -> ContractRun:
        return run_contract(
            compiled.model_document,
            compiled.case_document,
            model_payload=compiled.model_payload,
            case_payload=compiled.case_payload,
        )


__all__ = ["NativeContractBackend", "SimulationBackend"]
