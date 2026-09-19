"""Stable request and compiled-contract value objects."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass(frozen=True)
class SimulationRequest:
    """
    One user-level simulation request before contract compilation.

    ``model`` and ``case`` deliberately remain family-owned objects.  The
    request standardises orchestration metadata without flattening each case's
    physical vocabulary into one mega-schema.
    """

    assembly: str
    family: str
    model: Any = None
    case: Any = None
    name: str | None = None
    request_kind: str | None = None
    context: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        assembly = str(self.assembly).strip().lower()
        family = str(self.family).strip().lower()
        request_kind = family if self.request_kind is None else str(self.request_kind).strip().lower()
        if not assembly:
            raise ValueError("simulation request assembly must not be empty")
        if not family:
            raise ValueError("simulation request family must not be empty")
        if not request_kind:
            raise ValueError("simulation request kind must not be empty")
        object.__setattr__(self, "assembly", assembly)
        object.__setattr__(self, "family", family)
        object.__setattr__(self, "request_kind", request_kind)
        object.__setattr__(self, "context", dict(self.context))

    @property
    def kind(self) -> str:
        """Return the normalized request kind used for compiler metadata."""
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
