"""Submission metadata and a fully materialized native contract pair."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

__all__ = ["CompiledSimulation", "SimulationRequest"]


@dataclass(frozen=True)
class SimulationRequest:
    """Explicit metadata for one resolved model and solve plan."""

    assembly: str
    rig: str = ""
    family: str = ""
    study: str = ""
    model: Any = None
    case: Any = None
    outputs: tuple[str, ...] = ()
    name: str | None = None
    request_kind: str | None = None
    context: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for key in ("assembly", "rig", "family", "study"):
            object.__setattr__(self, key, str(getattr(self, key)).strip().lower())
        if not self.assembly:
            raise ValueError("simulation request assembly must not be empty")
        if not self.family:
            raise ValueError("simulation request requires an explicit protocol family")
        kind = self.family if self.request_kind is None else str(self.request_kind).strip().lower()
        if not kind:
            raise ValueError("simulation request kind must not be empty")
        object.__setattr__(self, "request_kind", kind)
        object.__setattr__(self, "outputs", tuple(self.outputs))
        object.__setattr__(self, "context", dict(self.context))

    @property
    def kind(self) -> str:
        assert self.request_kind is not None
        return self.request_kind


@dataclass(frozen=True)
class CompiledSimulation:
    """A fully materialised native contract submission."""

    request: SimulationRequest
    model_document: dict[str, Any]
    case_document: dict[str, Any]
    model_payload: bytes
    case_payload: bytes
    layout: dict[str, Any] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)

    @property
    def assembly(self) -> str:
        return self.request.assembly

    @property
    def rig(self) -> str:
        return self.request.rig

    @property
    def family(self) -> str:
        return self.request.family

    @property
    def request_kind(self) -> str:
        return self.request.kind

    @property
    def name(self) -> str | None:
        return self.request.name

    def documents(self) -> tuple[dict[str, Any], dict[str, Any]]:
        """Return model and case documents in native submission order."""
        return self.model_document, self.case_document

    def payloads(self) -> tuple[bytes, bytes]:
        """Return packed model and case payloads in native submission order."""
        return self.model_payload, self.case_payload
