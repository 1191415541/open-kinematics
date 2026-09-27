"""
Contract compilation: one plan and one model, one submission.

This is the layer the EPIC's G4 asks for.  A compiled submission used to be
produced by a family-specific class that also decided *what the run was* -- its
bench, its reading, its driven coordinates and its tire activation all came from
which class you picked.  Here the run is a :class:`~.plan.SolvePlan` and the
model is a :class:`~.model_view.ModelView`, and an emitter turns the two into
documents.  Nothing below reads a template name, a suspension topology or a
family name to decide what to emit: a family key selects an *emitter*, and the
emitter reads the model's own entities.

Two things the emitters deliberately do not do:

* they do not re-derive the model.  The documents come from the model the caller
  handed in, so a run cannot quietly solve a model other than the one the caller
  built;
* they do not decide the solve.  Time semantics, tire activation and the solver
  block come from the plan, so the same model compiled under two studies differs
  only where a study is allowed to differ.

``metadata`` records the whole plan, which is what A6 reads: two studies of one
assembly must produce the same model document and the same fingerprint, and
metadata is where the study is named without the model being touched.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Protocol, runtime_checkable

from suspension_contracts import CONTRACT_VERSION, pack_container

from .model_view import ModelView, view_of
from .plan import SolvePlan

__all__ = [
    "CompilationError",
    "Emitter",
    "EmitterRegistry",
    "compile_documents",
    "compile_plan",
    "default_emitters",
]

#: The one payload layout every emitter produces: a model document, then a case
#: document, each framed with its own blob section.
PAYLOAD_SCHEMA = "contract_document_pair"


class CompilationError(ValueError):
    """A plan and a model cannot be compiled into a submission."""


@runtime_checkable
class Emitter(Protocol):
    """Author the two contract documents for one family."""

    family: str
    payload_schema: str

    def emit(
        self, plan: SolvePlan, view: ModelView
    ) -> tuple[dict[str, Any], dict[str, Any], bytes, bytes]:
        """Return ``(model_document, case_document, model_blob, case_blob)``."""
        ...


def _document(value: Any, *, label: str) -> tuple[dict[str, Any], bytes]:
    """Normalize an emitter result that may carry an unstamped blob."""
    if isinstance(value, tuple) and len(value) == 2:
        return value[0], value[1]
    raise CompilationError(f"{label} must be a (document, blob) pair")


@dataclass(frozen=True)
class _KcQuasiStaticEmitter:
    """
    The K/C emitter: the existing family authoring, driven by a plan.

    It calls the family's own document functions rather than reimplementing them.
    That is the point: there is one authoring implementation and this layer
    decides *when* it runs and with what plan, so a K/C document cannot drift
    between the historical entry point and the compiled one.
    """

    family: str = "kc_quasi_static"
    payload_schema: str = "kc_quasi_static_contract_documents"

    def emit(
        self, plan: SolvePlan, view: ModelView
    ) -> tuple[dict[str, Any], dict[str, Any], bytes, bytes]:
        from ..cases.kc_quasi_static import case_document, model_document

        assembly = view.physical
        name = plan.inputs.name
        inputs = plan.inputs
        model_doc = model_document(
            assembly,
            name=name,
            drive_wheels=plan.drive_wheels,
            drive_mode=plan.drive_mode,
        )
        case_doc = case_document(
            assembly,
            family=self.family,
            name=name,
            wheel_values_mm=inputs.wheel_values_mm,
            rack_values_mm=inputs.rack_values_mm,
            drive=inputs.drive,
            left_right_mode=inputs.left_right_mode,
            paths=inputs.paths,
            levels=inputs.levels,
            maximum=inputs.maximum,
            side_mode=inputs.side_mode,
            times_s=plan.times_s,
            settings=plan.solver,
            drive_wheels=plan.drive_wheels,
        )
        return model_doc, case_doc, b"", b""


@dataclass(frozen=True)
class _AxleDynamicEmitter:
    """The axle dynamics emitter: the family's own authoring, framed by the plan."""

    family: str = "axle_dynamic"
    payload_schema: str = "axle_dynamic_contract_documents"

    def emit(
        self, plan: SolvePlan, view: ModelView
    ) -> tuple[dict[str, Any], dict[str, Any], bytes, bytes]:
        from ..cases.axle_dynamic import case_document, model_document

        model = plan.dynamic_model
        if model is None:
            raise CompilationError(
                "the axle_dynamic emitter needs an SI model on the plan; a "
                "compile view carries an assembly, and converting one into the "
                "other is what the plan's `dynamic_model` states explicitly"
            )
        model_doc, model_blob = model_document(model, name=plan.inputs.name)
        case_doc, case_blob = case_document(model, plan.dynamic_case, name=plan.inputs.name)
        return model_doc, case_doc, model_blob, case_blob


@dataclass(frozen=True)
class _DocumentPairEmitter:
    """
    A family whose documents the caller already authored.

    Vehicle K/C, handling, four-post and random-road prepare their documents
    through their own family modules and hand them over; this emitter frames
    them.  It exists so those four families keep one code path rather than
    growing a class each: what differs between them is *their* authoring, and
    that already lives in `preparation/`.
    """

    family: str
    payload_schema: str = PAYLOAD_SCHEMA

    def emit(
        self, plan: SolvePlan, view: ModelView
    ) -> tuple[dict[str, Any], dict[str, Any], bytes, bytes]:
        raise CompilationError(
            f"the {self.family!r} emitter is fed authored documents through the "
            "request context; use `compile_documents`"
        )


@dataclass(frozen=True)
class EmitterRegistry:
    """Mutable registry of emitters, keyed by the family they author."""

    _emitters: dict[str, Emitter] = field(default_factory=dict)

    def register(self, emitter: Emitter, *, replace: bool = False) -> Emitter:
        """Register one emitter, rejecting an accidental duplicate key."""
        key = str(emitter.family).strip().lower()
        if key in self._emitters and not replace:
            raise CompilationError(f"an emitter is already registered for {key!r}")
        self._emitters[key] = emitter
        return emitter

    def resolve(self, family: str) -> Emitter:
        """Return the emitter registered for one family, naming the unknown one."""
        key = str(family).strip().lower()
        try:
            return self._emitters[key]
        except KeyError as error:
            known = ", ".join(sorted(self._emitters))
            raise CompilationError(
                f"no emitter is registered for family {family!r}; the registered "
                f"families are {known}"
            ) from error

    def keys(self) -> tuple[str, ...]:
        """Return the registered families in a stable order."""
        return tuple(sorted(self._emitters))

    def __contains__(self, key: object) -> bool:
        return str(key).strip().lower() in self._emitters


#: The families this module can author from a plan and a model.
_PLANNED_FAMILIES: tuple[tuple[str, Emitter], ...] = (
    ("kc_quasi_static", _KcQuasiStaticEmitter()),
    ("axle_dynamic", _AxleDynamicEmitter()),
)

#: The families whose documents their own preparation authors and this layer
#: frames unchanged.  Listing them is what keeps "the compiler knows every
#: family" checkable rather than implied by whichever ones happen to be reached.
_AUTHORED_FAMILIES: tuple[tuple[str, str], ...] = (
    ("vehicle_kc", "vehicle_kc_contract_documents"),
    ("handling", "handling_contract_documents"),
    ("ride_four_post", "ride_four_post_contract_documents"),
    ("ride_random_road", "ride_random_road_contract_documents"),
    ("vehicle_dynamic", "vehicle_dynamic_contract_documents"),
)


def default_emitters() -> EmitterRegistry:
    """Return the registry of every family this build can compile."""
    registry = EmitterRegistry()
    for emitter in (*_PLANNED_FAMILIES,):
        registry.register(emitter[1])
    for family, payload_schema in _AUTHORED_FAMILIES:
        registry.register(_DocumentPairEmitter(family=family, payload_schema=payload_schema))
    return registry


def compile_plan(
    plan: SolvePlan,
    model: Any,
    *,
    registry: EmitterRegistry | None = None,
) -> tuple[dict[str, Any], dict[str, Any], bytes, bytes, dict[str, Any]]:
    """
    Compile one plan and one model into documents, payloads and metadata.

    The returned metadata carries the plan's routing facts alongside the
    documents, so a caller holding only the compiled submission can still say
    which bench it was on, which study read it and how it was read.
    """
    view = model if isinstance(model, ModelView) else view_of(model)
    active = default_emitters() if registry is None else registry
    emitter = active.resolve(plan.family)
    model_doc, case_doc, model_blob, case_blob = emitter.emit(plan, view)
    metadata: dict[str, Any] = {
        "emitter": type(emitter).__name__,
        "payload_schema": emitter.payload_schema,
        **plan.describe(),
    }
    if view.fingerprint:
        metadata["fingerprint"] = view.fingerprint
    if plan.note:
        metadata["note"] = plan.note
    return model_doc, case_doc, model_blob, case_blob, metadata


def compile_documents(
    plan: SolvePlan,
    *,
    model_document: Any,
    case_document: Any,
    model_payload: bytes | None = None,
    case_payload: bytes | None = None,
    validate_contract_family: bool = False,
) -> tuple[dict[str, Any], dict[str, Any], bytes, bytes, dict[str, Any]]:
    """
    Frame documents a family already authored.

    Four families prepare their own documents; this is how they reach the same
    submission shape without growing a compiler class each.  The documents are
    checked against the contract identity when asked, because a family that hands
    over a document for another family is the mistake this boundary can catch.
    """
    model, embedded_model = _pair(model_document, label="model_document")
    case, embedded_case = _pair(case_document, label="case_document")
    if validate_contract_family:
        _validate_identity(case, family=plan.family)
    metadata: dict[str, Any] = {
        "emitter": "authored_documents",
        "payload_schema": PAYLOAD_SCHEMA,
        **plan.describe(),
    }
    return (
        model,
        case,
        model_payload if model_payload is not None else embedded_model or pack_container(model),
        case_payload if case_payload is not None else embedded_case or pack_container(case),
        metadata,
    )


def _pair(value: Any, *, label: str) -> tuple[dict[str, Any], bytes | None]:
    """Return one document and its embedded blob, if the caller supplied a pair."""
    if isinstance(value, tuple) and len(value) == 2:
        return value[0], value[1]
    if isinstance(value, dict):
        return value, None
    raise CompilationError(f"{label} must be a document or (document, blob) pair")


def _validate_identity(case: dict[str, Any], *, family: str) -> None:
    """Refuse a case document that belongs to another family or version."""
    if case.get("contract") != "multibody-case":
        raise CompilationError("a case document must declare the multibody-case contract")
    if case.get("contract_version") != CONTRACT_VERSION:
        raise CompilationError(
            f"a case document must declare contract_version {CONTRACT_VERSION}, got "
            f"{case.get('contract_version')!r}"
        )
    if case.get("kind") != "case":
        raise CompilationError("a case document must declare kind 'case'")
    if case.get("family") != family:
        raise CompilationError(
            f"the {family!r} emitter was handed a case for {case.get('family')!r}"
        )
