"""
Unified preparation protocol, registry and lifecycle for simulations.

This module owns the single default preparation registry, the document-request
bypass, the legacy ``prepared`` migration adapter call and the reuse rule for a
``prepared_simulation`` context value.  Family modules own their physical
assembly; nothing here submits native, decodes results or computes metrics.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from importlib import import_module
from typing import Any, Mapping, Protocol, runtime_checkable

from .request import SimulationRequest


@dataclass(frozen=True)
class PreparedSimulation:
    """Preparation result carried through the staged simulation lifecycle."""

    request: SimulationRequest
    value: Any = None
    context: Mapping[str, Any] = field(default_factory=dict)
    metadata: Mapping[str, Any] = field(default_factory=dict)


@runtime_checkable
class Preparation(Protocol):
    """Prepare one assembly/family request without submitting it."""

    assembly: str
    family: str

    def prepare(self, request: SimulationRequest) -> PreparedSimulation:
        """Return the prepared request, value, context and metadata."""
        ...


def _normalized_key(assembly: Any, family: Any) -> tuple[str, str]:
    """Normalize one registry key and reject empty dimensions."""
    if assembly is None or family is None:
        raise ValueError("preparation key assembly and family must not be empty")
    key = (str(assembly).strip().casefold(), str(family).strip().casefold())
    if not key[0] or not key[1]:
        raise ValueError(f"preparation key must not be empty, got {key!r}")
    return key


@dataclass(frozen=True)
class _LazyFamilyPreparation:
    """Preparation entry that imports its family module on first preparation."""

    assembly: str
    family: str
    module: str
    function: str = "prepare_request"

    def prepare(self, request: SimulationRequest) -> PreparedSimulation:
        """Import the family module and run its ``prepare_request`` function."""
        family_module = import_module(self.module)
        return getattr(family_module, self.function)(request)


#: The fixed lazy-import table for the seven prepared families.  Key
#: enumeration, document bypass and prepared-simulation reuse never import
#: these modules; only executing one family preparation does.
_FAMILY_PREPARATIONS: tuple[tuple[str, str, str], ...] = (
    ("axle", "axle_dynamic", "suspension_multibody.preparation.axle_dynamic"),
    ("axle", "kc_quasi_static", "suspension_multibody.preparation.kc_quasi_static"),
    ("vehicle", "vehicle_kc", "suspension_multibody.preparation.vehicle_kc"),
    ("vehicle", "handling", "suspension_multibody.preparation.handling"),
    ("vehicle", "ride_four_post", "suspension_multibody.preparation.ride_four_post"),
    ("vehicle", "ride_random_road", "suspension_multibody.preparation.ride_random_road"),
    ("vehicle", "vehicle_dynamic", "suspension_multibody.preparation.vehicle_dynamic"),
)

_VEHICLE_DYNAMIC_KEY = ("vehicle", "vehicle_dynamic")
_LEGACY_PREPARED_KEY = "prepared"
_PREPARED_SIMULATION_KEY = "prepared_simulation"
_LEGACY_ADAPTER_MODULE = "suspension_multibody.preparation.vehicle_dynamic"
_LEGACY_ADAPTER_FUNCTION = "adapt_legacy_prepared_request"

_DOCUMENT_CONTEXT_KEYS = (
    "model_document",
    "case_document",
    "model_document_pair",
    "model_payload",
    "case_payload",
)


class PreparationRegistry:
    """Mutable registry keyed by the orthogonal assembly/family dimensions."""

    def __init__(self) -> None:
        self._preparations: dict[tuple[str, str], Preparation] = {}

    def register(
        self, preparation: Preparation, *, replace: bool = False
    ) -> Preparation:
        """Register one preparation, rejecting accidental duplicate keys."""
        key = _normalized_key(preparation.assembly, preparation.family)
        if key in self._preparations and not replace:
            raise KeyError(f"preparation already registered for {key[0]}/{key[1]}")
        self._preparations[key] = preparation
        return preparation

    def resolve(self, assembly: str, family: str) -> Preparation:
        """Return the preparation registered for one assembly/family pair."""
        key = _normalized_key(assembly, family)
        try:
            return self._preparations[key]
        except KeyError as error:
            available = ", ".join(f"{a}/{f}" for a, f in sorted(self._preparations))
            raise KeyError(
                f"no preparation registered for {key!r}; available: {available}"
            ) from error

    def keys(self) -> tuple[tuple[str, str], ...]:
        """Return registered keys in stable order."""
        return tuple(sorted(self._preparations))

    def __contains__(self, key: object) -> bool:
        return key in self._preparations


def default_preparation_registry() -> PreparationRegistry:
    """Return the registry that lazily registers all seven prepared families."""
    registry = PreparationRegistry()
    for assembly, family, module in _FAMILY_PREPARATIONS:
        registry.register(
            _LazyFamilyPreparation(assembly=assembly, family=family, module=module)
        )
    return registry


def _is_document_value(value: Any) -> bool:
    """Return whether one model/case value is already a contract document."""
    if isinstance(value, dict):
        return True
    return (
        isinstance(value, tuple)
        and len(value) == 2
        and isinstance(value[0], dict)
        and isinstance(value[1], bytes)
    )


def _is_document_request(request: SimulationRequest) -> bool:
    """Return whether a request already carries authored contract documents."""
    if any(key in request.context for key in _DOCUMENT_CONTEXT_KEYS):
        return True
    return _is_document_value(request.model) or _is_document_value(request.case)


def _has_legacy_prepared(request: SimulationRequest) -> bool:
    """Return whether a request carries the migrated vehicle-dynamic key."""
    if _normalized_key(request.assembly, request.family) != _VEHICLE_DYNAMIC_KEY:
        return False
    return request.context.get(_LEGACY_PREPARED_KEY) is not None


def _adapt_legacy_prepared_request(request: SimulationRequest) -> SimulationRequest:
    """Wrap a legacy ``prepared`` request through the family migration adapter."""
    family_module = import_module(_LEGACY_ADAPTER_MODULE)
    adapter = getattr(family_module, _LEGACY_ADAPTER_FUNCTION)
    return adapter(request)


def _reusable_prepared_simulation(
    request: SimulationRequest,
) -> PreparedSimulation | None:
    """Return the matching prepared result carried by the request, if any."""
    candidate = request.context.get(_PREPARED_SIMULATION_KEY)
    if not isinstance(candidate, PreparedSimulation):
        return None
    if _normalized_key(candidate.request.assembly, candidate.request.family) != (
        _normalized_key(request.assembly, request.family)
    ):
        return None
    if (
        candidate.request.model is not request.model
        or candidate.request.case is not request.case
    ):
        return None
    return candidate


def _compose_prepared(
    request: SimulationRequest, prepared: PreparedSimulation
) -> PreparedSimulation:
    """
    Merge a preparation context into the request and record the reuse handle.

    The injected ``prepared_simulation`` value is the preparation result handed
    in here, never the composed result, so the value graph stays acyclic and
    frozen-dataclass equality cannot recurse through the request context.
    """
    context = dict(request.context)
    context.update(dict(prepared.context))
    context[_PREPARED_SIMULATION_KEY] = prepared
    return PreparedSimulation(
        request=replace(request, context=context),
        value=prepared.value,
        context=context,
        metadata=dict(prepared.metadata),
    )


def prepare_request(
    request: SimulationRequest,
    *,
    registry: PreparationRegistry | None = None,
) -> PreparedSimulation:
    """Prepare one domain request, or pass a document request straight through."""
    if _is_document_request(request):
        return PreparedSimulation(request=request, context=dict(request.context))

    adapted = (
        _adapt_legacy_prepared_request(request)
        if _has_legacy_prepared(request)
        else request
    )
    reusable = _reusable_prepared_simulation(adapted)
    if reusable is not None:
        return _compose_prepared(adapted, reusable)

    active = default_preparation_registry() if registry is None else registry
    preparation = active.resolve(adapted.assembly, adapted.family)
    prepared = preparation.prepare(adapted)
    if not isinstance(prepared, PreparedSimulation):
        raise TypeError(
            f"preparation for {adapted.assembly}/{adapted.family} must return "
            f"PreparedSimulation, got {type(prepared).__name__}"
        )
    return _compose_prepared(adapted, prepared)


__all__ = [
    "Preparation",
    "PreparationRegistry",
    "PreparedSimulation",
    "default_preparation_registry",
    "prepare_request",
]
