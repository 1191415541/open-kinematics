"""Compiler protocols and adapters for existing family compilers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Protocol, runtime_checkable

from suspension_contracts import CONTRACT_VERSION, pack_container

from .request import CompiledSimulation, SimulationRequest


@runtime_checkable
class CaseCompiler(Protocol):
    """Compile one family-owned request into a native contract submission."""

    assembly: str
    family: str

    def compile(self, request: SimulationRequest) -> CompiledSimulation:
        """Compile a request without submitting it to native."""
        ...


def _document_and_payload(
    value: Any, *, label: str
) -> tuple[dict[str, Any], bytes | None]:
    """Normalize a document or ``(document, payload)`` pair."""
    if isinstance(value, tuple) and len(value) == 2:
        document, payload = value
        if isinstance(document, dict) and isinstance(payload, bytes):
            return document, payload
        raise TypeError(f"{label} must contain a dict document and bytes payload")
    if isinstance(value, dict):
        return value, None
    raise TypeError(f"{label} must be a document or (document, payload) pair")


def _validate_request_identity(
    request: SimulationRequest,
    *,
    compiler_name: str,
    assembly: str,
    family: str,
    expected_request_kind: str | None = None,
) -> None:
    """Validate routing dimensions before family-specific authoring."""
    if request.assembly != assembly:
        raise ValueError(
            f"{compiler_name} expects assembly {assembly!r}, got {request.assembly!r}"
        )
    if request.family != family:
        raise ValueError(
            f"{compiler_name} expects family {family!r}, got {request.family!r}"
        )
    if expected_request_kind is not None and request.kind != expected_request_kind:
        raise ValueError(
            f"{compiler_name} expects request kind {expected_request_kind!r}, "
            f"got {request.kind!r}"
        )


def compile_document_pair(
    request: SimulationRequest,
    *,
    model_document: Any,
    case_document: Any,
    model_payload: bytes | None = None,
    case_payload: bytes | None = None,
    compiler_name: str | None = None,
    payload_schema: str = "contract_document_pair",
    layout: dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
) -> CompiledSimulation:
    """Build a compiled submission from already-authored contract documents."""
    model, embedded_model_payload = _document_and_payload(
        model_document, label="model_document"
    )
    case, embedded_case_payload = _document_and_payload(
        case_document, label="case_document"
    )
    if model_payload is not None and not isinstance(model_payload, bytes):
        raise TypeError("model_payload must be bytes")
    if case_payload is not None and not isinstance(case_payload, bytes):
        raise TypeError("case_payload must be bytes")

    compiled_layout: dict[str, Any] = {
        "document_order": ["model", "case"],
        "payload_order": ["model", "case"],
    }
    if layout:
        compiled_layout.update(layout)

    compiled_metadata: dict[str, Any] = {
        "compiler": compiler_name or request.family,
        "assembly": request.assembly,
        "family": request.family,
        "request_kind": request.kind,
        "payload_schema": payload_schema,
    }
    if metadata:
        compiled_metadata.update(metadata)

    return CompiledSimulation(
        request=request,
        model_document=model,
        case_document=case,
        model_payload=(
            model_payload
            if model_payload is not None
            else embedded_model_payload
            if embedded_model_payload is not None
            else pack_container(model)
        ),
        case_payload=(
            case_payload
            if case_payload is not None
            else embedded_case_payload
            if embedded_case_payload is not None
            else pack_container(case)
        ),
        layout=compiled_layout,
        metadata=compiled_metadata,
    )


@dataclass(frozen=True)
class DocumentPairCompiler:
    """Compatibility compiler for callers that already own documents."""

    assembly: str
    family: str
    compiler_name: str | None = None
    payload_schema: str = "contract_document_pair"
    expected_request_kind: str | None = None
    validate_contract_family: bool = False

    def _validate_request(self, request: SimulationRequest) -> None:
        _validate_request_identity(
            request,
            compiler_name=type(self).__name__,
            assembly=self.assembly,
            family=self.family,
            expected_request_kind=self.expected_request_kind,
        )

    def _validate_documents(
        self,
        model_document: dict[str, Any],
        case_document: dict[str, Any],
    ) -> None:
        """Validate the stable identity fields of native contract documents."""
        if not self.validate_contract_family:
            return

        expected = CONTRACT_VERSION
        if model_document.get("contract") != "multibody-model":
            raise ValueError(
                f"{type(self).__name__} requires a multibody-model document"
            )
        if model_document.get("contract_version") != expected:
            raise ValueError(
                f"{type(self).__name__} requires model contract_version "
                f"{expected}, got {model_document.get('contract_version')!r}"
            )
        if model_document.get("kind") != "model":
            raise ValueError(f"{type(self).__name__} requires model kind")

        if case_document.get("contract") != "multibody-case":
            raise ValueError(
                f"{type(self).__name__} requires a multibody-case document"
            )
        if case_document.get("contract_version") != expected:
            raise ValueError(
                f"{type(self).__name__} requires case contract_version "
                f"{expected}, got {case_document.get('contract_version')!r}"
            )
        if case_document.get("kind") != "case":
            raise ValueError(f"{type(self).__name__} requires case kind")
        document_family = case_document.get("family")
        if document_family != self.family:
            raise ValueError(
                f"{type(self).__name__} requires case family {self.family!r}, "
                f"got {document_family!r}"
            )

    def compile(self, request: SimulationRequest) -> CompiledSimulation:
        """Compile existing documents without submitting them to native."""
        self._validate_request(request)
        model_document = request.context.get("model_document", request.model)
        case_document = request.context.get("case_document", request.case)
        model_payload = request.context.get("model_payload")
        case_payload = request.context.get("case_payload")
        model, _ = _document_and_payload(model_document, label="model_document")
        case, _ = _document_and_payload(case_document, label="case_document")
        self._validate_documents(model, case)
        return compile_document_pair(
            request,
            model_document=model_document,
            case_document=case_document,
            model_payload=model_payload,
            case_payload=case_payload,
            compiler_name=self.compiler_name or type(self).__name__,
            payload_schema=self.payload_schema,
        )


class KcQuasiStaticCompiler(DocumentPairCompiler):
    """Compile axle K/C documents without executing or decoding them."""

    def __init__(self) -> None:
        super().__init__(
            assembly="axle",
            family="kc_quasi_static",
            compiler_name="KcQuasiStaticCompiler",
            payload_schema="kc_quasi_static_contract_documents",
            expected_request_kind="kc_quasi_static",
            validate_contract_family=True,
        )


class VehicleKcCompiler(DocumentPairCompiler):
    """Compile vehicle K/C documents, including driven-joint authoring."""

    def __init__(self) -> None:
        super().__init__(
            assembly="vehicle",
            family="vehicle_kc",
            compiler_name="VehicleKcCompiler",
            payload_schema="vehicle_kc_contract_documents",
            expected_request_kind="vehicle_kc",
            validate_contract_family=True,
        )

    def compile(self, request: SimulationRequest) -> CompiledSimulation:
        self._validate_request(request)
        from ..cases.vehicle_kc import model_document as vehicle_kc_model_document

        model_pair = request.context.get("model_document_pair", request.model)
        vehicle_assembly = request.context.get("vehicle_assembly")
        wheels = tuple(request.context.get("wheels", ()))
        if vehicle_assembly is None:
            if isinstance(model_pair, dict):
                return super().compile(request)
            raise ValueError("vehicle_kc requests require vehicle_assembly context")
        if not isinstance(model_pair, tuple) or len(model_pair) != 2:
            raise TypeError("vehicle_kc model must be a (document, payload) pair")

        model, model_blob = vehicle_kc_model_document(
            model_pair,
            wheels=wheels,
            assembly=vehicle_assembly,
        )
        case_document = request.context.get("case_document", request.case)
        case, _ = _document_and_payload(case_document, label="case_document")
        self._validate_documents(model, case)
        return compile_document_pair(
            request,
            model_document=model,
            case_document=case,
            model_payload=pack_container(model, model_blob),
            case_payload=request.context.get("case_payload"),
            compiler_name=self.compiler_name,
            payload_schema=self.payload_schema,
            metadata={
                "derived_model": True,
                "driven_joint_count": sum(
                    1
                    for joint in model.get("joints", ())
                    if joint.get("type") == "driven_translation"
                ),
            },
        )


class HandlingCompiler(DocumentPairCompiler):
    """Compile handling documents without executing the manoeuvre."""

    def __init__(self) -> None:
        super().__init__(
            assembly="vehicle",
            family="handling",
            compiler_name="HandlingCompiler",
            payload_schema="handling_contract_documents",
            expected_request_kind="handling",
            validate_contract_family=True,
        )


class RideFourPostCompiler(DocumentPairCompiler):
    """Compile four-post ride documents without interpreting results."""

    def __init__(self) -> None:
        super().__init__(
            assembly="vehicle",
            family="ride_four_post",
            compiler_name="RideFourPostCompiler",
            payload_schema="ride_four_post_contract_documents",
            expected_request_kind="ride_four_post",
            validate_contract_family=True,
        )


class RideRandomRoadCompiler(DocumentPairCompiler):
    """Compile random-road ride documents without interpreting results."""

    def __init__(self) -> None:
        super().__init__(
            assembly="vehicle",
            family="ride_random_road",
            compiler_name="RideRandomRoadCompiler",
            payload_schema="ride_random_road_contract_documents",
            expected_request_kind="ride_random_road",
            validate_contract_family=True,
        )


@dataclass(frozen=True)
class AxleDynamicCompiler:
    """Adapter over the existing axle dynamic document authoring functions."""

    assembly: str = "axle"
    family: str = "axle_dynamic"
    compiler_name: str = "AxleDynamicCompiler"
    payload_schema: str = "axle_dynamic_contract_documents"
    expected_request_kind: str = "axle_dynamic"

    def compile(self, request: SimulationRequest) -> CompiledSimulation:
        _validate_request_identity(
            request,
            compiler_name=self.compiler_name,
            assembly=self.assembly,
            family=self.family,
            expected_request_kind=self.expected_request_kind,
        )
        from ..cases.axle_dynamic import case_document, model_document

        if request.model is None or request.case is None:
            raise ValueError("axle_dynamic requests require model and case objects")
        model, model_blob = model_document(request.model, name=request.name)
        case, case_blob = case_document(request.model, request.case, name=request.name)
        return compile_document_pair(
            request,
            model_document=model,
            case_document=case,
            model_payload=pack_container(model, model_blob),
            case_payload=pack_container(case, case_blob),
            compiler_name=self.compiler_name,
            payload_schema=self.payload_schema,
        )


@dataclass(frozen=True)
class VehicleDynamicCompiler:
    """Adapter over the existing vehicle dynamic document authoring functions."""

    assembly: str = "vehicle"
    family: str = "vehicle_dynamic"
    compiler_name: str = "VehicleDynamicCompiler"
    payload_schema: str = "vehicle_dynamic_contract_documents"
    expected_request_kind: str = "vehicle_dynamic"

    def compile(self, request: SimulationRequest) -> CompiledSimulation:
        _validate_request_identity(
            request,
            compiler_name=self.compiler_name,
            assembly=self.assembly,
            family=self.family,
            expected_request_kind=self.expected_request_kind,
        )
        from ..cases.vehicle_dynamic import (
            case_document,
            model_document,
            prepare_vehicle_run,
        )

        if request.model is None or request.case is None:
            raise ValueError("vehicle_dynamic requests require model and case objects")
        prepared = request.context.get("prepared")
        prepared_was_supplied = prepared is not None
        if prepared is None:
            prepared = prepare_vehicle_run(request.model, request.case)
        model, model_blob = model_document(request.model, prepared, name=request.name)
        case, case_blob = case_document(
            request.model, request.case, prepared, name=request.name
        )
        if model.get("contract") != "multibody-model":
            raise ValueError("VehicleDynamicCompiler produced an invalid model contract")
        if model.get("contract_version") != CONTRACT_VERSION:
            raise ValueError("VehicleDynamicCompiler produced an invalid model version")
        if model.get("kind") != "model":
            raise ValueError("VehicleDynamicCompiler produced an invalid model kind")
        if case.get("contract") != "multibody-case":
            raise ValueError("VehicleDynamicCompiler produced an invalid case contract")
        if case.get("contract_version") != CONTRACT_VERSION:
            raise ValueError("VehicleDynamicCompiler produced an invalid case version")
        if case.get("kind") != "case" or case.get("family") != self.family:
            raise ValueError("VehicleDynamicCompiler produced an invalid case document")
        return compile_document_pair(
            request,
            model_document=model,
            case_document=case,
            model_payload=pack_container(model, model_blob),
            case_payload=pack_container(case, case_blob),
            compiler_name=self.compiler_name,
            payload_schema=self.payload_schema,
            metadata={"prepared": prepared_was_supplied},
        )


class CompilerRegistry:
    """Mutable registry keyed by the orthogonal assembly/family dimensions."""

    def __init__(self) -> None:
        self._compilers: dict[tuple[str, str], CaseCompiler] = {}

    def register(self, compiler: CaseCompiler, *, replace: bool = False) -> CaseCompiler:
        """Register one compiler, rejecting accidental duplicate keys."""
        key = (str(compiler.assembly).lower(), str(compiler.family).lower())
        if key in self._compilers and not replace:
            raise KeyError(f"compiler already registered for {key[0]}/{key[1]}")
        self._compilers[key] = compiler
        return compiler

    def resolve(self, assembly: str, family: str) -> CaseCompiler:
        """Return the compiler registered for one assembly/family pair."""
        key = (str(assembly).strip().lower(), str(family).strip().lower())
        try:
            return self._compilers[key]
        except KeyError as error:
            available = ", ".join(f"{a}/{f}" for a, f in sorted(self._compilers))
            raise KeyError(
                f"no compiler registered for {key[0]}/{key[1]}; available: {available}"
            ) from error

    def keys(self) -> tuple[tuple[str, str], ...]:
        """Return registered keys in stable order."""
        return tuple(sorted(self._compilers))

    def __contains__(self, key: object) -> bool:
        return key in self._compilers


def compile_request(
    request: SimulationRequest, *, registry: CompilerRegistry | None = None
) -> CompiledSimulation:
    """Compile a request using a supplied or default registry."""
    from .dispatch import default_registry

    active = default_registry() if registry is None else registry
    return active.resolve(request.assembly, request.family).compile(request)


__all__ = [
    "AxleDynamicCompiler",
    "CaseCompiler",
    "CompiledSimulation",
    "CompilerRegistry",
    "DocumentPairCompiler",
    "HandlingCompiler",
    "KcQuasiStaticCompiler",
    "RideFourPostCompiler",
    "RideRandomRoadCompiler",
    "VehicleDynamicCompiler",
    "VehicleKcCompiler",
    "compile_document_pair",
    "compile_request",
]
