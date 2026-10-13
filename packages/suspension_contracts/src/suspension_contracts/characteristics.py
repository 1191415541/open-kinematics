"""
Pure data declarations for a characteristics output request.

A *characteristics request* asks the existing single physical path for extra
derived numbers.  It is **not** a study, not a case family and not a second
entry point: the request travels inside the one ``ResolvedModel`` ->
``ResolvedSolvePlan`` -> ``compile_resolved`` -> ``run_compiled`` ->
``ResultEnvelope`` chain, and everything here is inert data.

Three rules shape this module:

* the encoding is pure data, so the same document validates in Python and in
  the native reader;
* an unknown field, version, boundary, unit or domain is rejected layer by
  layer -- never silently ignored, because a reader that drops what it does not
  know turns a typo into a different physical question;
* every declared item carries its own version so a definition can be revised
  without overwriting the older meaning.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import Any

from .multibody import ContractError

REQUEST_VERSION = 1
"""Version of the request envelope itself."""

DOMAINS = ("kinematic", "equilibrium", "trajectory", "dynamic")
"""The four request domains, matching the design's suspension/vehicle x
quasi-static/dynamic combinations."""

DYNAMIC_DEFINITION_KINDS = (
    "frozen_force_tangent",
    "frozen_load_compliance",
    "continuous_descriptor",
)
"""Dynamic definitions are named separately and accepted separately.  A frozen
tangent is not a continuous descriptor, and neither is a substitute for the
other."""

SAMPLE_STATUSES = (
    "valid",
    "unidentifiable",
    "singular",
    "nonsmooth",
    "not_equilibrium",
    "inconsistent_sample",
    "undefined_geometry",
)
"""Per-sample outcome vocabulary.  A missing runtime channel is a query error,
not one of these statuses."""

MEASUREMENT_ID_CHARS = frozenset(
    "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-*{};"
)
"""A measurement ID is checked for shape only; it is never parsed for meaning.

The design forbids recovering a metric from an entity *name*, so no prefix or
suffix rule exists here.  ``*`` and ``{}`` appear because a declared family may
be written as a pattern (``front.{}``).
"""

_REQUEST_FIELDS = frozenset({
    "schema_version",
    "characteristics",
})
_CHARACTERISTICS_FIELDS = frozenset({
    "request_id",
    "domain",
    "samples",
    "measurements",
    "perturbations",
    "boundary",
    "definition",
    "frozen_conditions",
    "response_operator",
    "wheel_end_response",
})
_MEASUREMENT_FIELDS = frozenset({
    "id",
    "measurement_id",
    "definition_version",
    "units",
    "response_kind",
    "response_version",
    "binding",
})
_PERTURBATION_FIELDS = frozenset({
    "id",
    "measurement_id",
    "definition_version",
    "units",
    "response_kind",
    "response_version",
    "mode",
    "binding",
})

MEASUREMENT_COMPONENTS = ("x", "y", "z", "rx", "ry", "rz")
"""The six world-axis blades a measurement may name.  The first three are
translations, the last three are the relative attitude's small rotation about
that world axis (design §434), which is deliberately *not* an Euler angle."""

WRENCH_COMPONENTS = ("Fx", "Fy", "Fz", "Mx", "My", "Mz")
"""The wrench order ``body_wrench`` itself uses, so a perturbation that names a
component and the column the kernel assembles cannot disagree about which
physical quantity a unit means."""

_MEASUREMENT_BINDING_FIELDS = frozenset({"marker", "component", "reference"})
_PERTURBATION_BINDING_FIELDS = frozenset({"terms"})
_PERTURBATION_TERM_FIELDS = frozenset({"body", "wrench", "amplitude"})
_DEFINITION_FIELDS = frozenset({"kind", "version"})
_FROZEN_FIELDS = frozenset({"protocol", "version"})
_OPERATOR_FIELDS = frozenset({"kind", "version"})
_BOUNDARY_FIELDS = frozenset({"profile", "version", "released", "held"})


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ContractError(f"characteristics: {label} must be an object")
    return value


def _reject_unknown(row: Mapping[str, Any], allowed: frozenset[str], label: str) -> None:
    unknown = sorted(key for key in row if key not in allowed)
    if unknown:
        raise ContractError(f"characteristics: {label} has unknown field(s) {unknown}")


def _require_text(row: Mapping[str, Any], key: str, label: str) -> str:
    value = row.get(key)
    if not isinstance(value, str) or not value:
        raise ContractError(f"characteristics: {label} requires a non-empty {key!r}")
    return value


def _require_version(row: Mapping[str, Any], label: str) -> int:
    version = row.get("definition_version")
    if not isinstance(version, int) or isinstance(version, bool) or version < 1:
        raise ContractError(f"characteristics: {label} requires a positive integer definition_version")
    return version


def _check_measurement_id(value: str, label: str) -> None:
    if not set(value) <= MEASUREMENT_ID_CHARS:
        raise ContractError(f"characteristics: {label} id {value!r} has an unsupported character")
    # A wildcard is a declared family, not a way to name a metric by pattern
    # matching at runtime; it must be the whole final segment when present.
    if "*" in value and not value.endswith(("*", "}")):
        raise ContractError(
            f"characteristics: {label} id {value!r} uses a wildcard outside a final segment"
        )


def _check_measurement_binding(row: Mapping[str, Any], label: str) -> None:
    """
    Check one measurement's explicit entity binding.

    A measurement must say *which* marker, *which* component and *which*
    reference body it reads.  The contract bans recovering a metric or a frame
    from an entity name (``mb_cases/functions.hpp``), so the binding is a
    declaration, never an inference: nothing here parses a marker name for a
    prefix or a suffix.
    """
    _reject_unknown(row, _MEASUREMENT_BINDING_FIELDS, label)
    _require_text(row, "marker", label)
    component = _require_text(row, "component", label)
    if component not in MEASUREMENT_COMPONENTS:
        raise ContractError(
            f"characteristics: {label} declares unknown component {component!r}; "
            f"expected one of {list(MEASUREMENT_COMPONENTS)}"
        )
    reference = row.get("reference")
    if reference is not None and not isinstance(reference, str):
        raise ContractError(f"characteristics: {label} reference must be a string")


def _check_perturbation_binding(row: Mapping[str, Any], label: str) -> None:
    """
    Check one perturbation's explicit load-mode binding.

    The binding is a list of *terms*, each naming a body, a wrench component and
    an amplitude.  A named mode (the design's ``heave_load``) is exactly the list
    with two terms, which is why the shape is a list rather than one entry: a
    single-component representation could not express the in-phase pair.
    """
    _reject_unknown(row, _PERTURBATION_BINDING_FIELDS, label)
    terms = row.get("terms")
    if not isinstance(terms, Sequence) or isinstance(terms, (str, bytes)):
        raise ContractError(f"characteristics: {label} terms must be a list")
    if not terms:
        raise ContractError(f"characteristics: {label} terms must not be empty")
    for entry in terms:
        term = _require_mapping(entry, f"{label} term")
        _reject_unknown(term, _PERTURBATION_TERM_FIELDS, f"{label} term")
        _require_text(term, "body", f"{label} term")
        wrench = _require_text(term, "wrench", f"{label} term")
        if wrench not in WRENCH_COMPONENTS:
            raise ContractError(
                f"characteristics: {label} term declares unknown wrench {wrench!r}; "
                f"expected one of {list(WRENCH_COMPONENTS)}"
            )
        amplitude = term.get("amplitude")
        if isinstance(amplitude, bool) or not isinstance(amplitude, (int, float)):
            raise ContractError(
                f"characteristics: {label} term requires a numeric amplitude"
            )


def _check_declaration(row: Mapping[str, Any], label: str, *, kind: str) -> None:
    _reject_unknown(row, _MEASUREMENT_FIELDS if kind == "measurement" else _PERTURBATION_FIELDS, label)
    identifier = _require_text(row, "measurement_id", label)
    _check_measurement_id(identifier, label)
    _require_version(row, label)
    units = _require_text(row, "units", label)
    if units not in _UNIT_TOKENS:
        raise ContractError(f"characteristics: {label} declares unsupported unit {units!r}")
    response_kind = _require_text(row, "response_kind", label)
    if response_kind not in _RESPONSE_KINDS:
        raise ContractError(f"characteristics: {label} declares unknown response_kind {response_kind!r}")
    version = row.get("response_version")
    if not isinstance(version, int) or isinstance(version, bool) or version < 1:
        raise ContractError(f"characteristics: {label} requires a positive integer response_version")
    if "id" in row:
        _require_text(row, "id", label)
    binding = row.get("binding")
    if binding is None:
        return
    binding_row = _require_mapping(binding, f"{label}.binding")
    if kind == "measurement":
        _check_measurement_binding(binding_row, f"{label}.binding")
    else:
        _check_perturbation_binding(binding_row, f"{label}.binding")


_UNIT_TOKENS = frozenset({
    "1", "rad", "deg", "m", "mm", "s", "kg", "N", "N*m", "N/m", "N*m/rad",
    "N*m/m", "N*m*s", "rad/m", "rad/s", "rad/N", "rad/(N*m)", "m/m", "m/s",
    "m/N", "m/(N*m)", "N*m*s/rad", "percent",
})
"""Units are compared as opaque tokens.  The list is deliberately explicit: a
free-form unit string would let two incompatible requests look compatible."""

_RESPONSE_KINDS = frozenset({
    "measurement_value",
    "measurement_derivative",
    "time_derivative",
    "frozen_force_tangent",
    "frozen_load_compliance",
    "local_response",
    "matrix_view",
    "load_compliance",
    "wheel_rate",
    "ride_rate",
    "roll_rate",
    "component_rate",
    "grouped_rate",
    "roll_center",
    "instant_center",
    "swing_arm",
    "pitch_center",
    "screw_axis",
    "ackermann",
})
"""Named response kinds.  A kind is a separate axis from the response version:
``roll_center`` exists at version 1 (legacy) and version 2 (force-equivalent)."""


def validate_characteristics_request(payload: Any, label: str = "characteristics") -> None:
    """
    Reject an invalid characteristics request layer by layer.

    The function never mutates and never defaults a missing field: an absent
    piece of the question is a different question, not an implied one.
    """
    row = _require_mapping(payload, label)
    _reject_unknown(row, _REQUEST_FIELDS, label)
    if row.get("schema_version") != REQUEST_VERSION:
        raise ContractError(
            f"characteristics: {label} schema_version must be {REQUEST_VERSION}"
        )
    body = _require_mapping(row.get("characteristics"), "characteristics")
    _reject_unknown(body, _CHARACTERISTICS_FIELDS, "characteristics")

    _require_text(body, "request_id", "characteristics")

    domain = _require_text(body, "domain", "characteristics")
    if domain not in DOMAINS:
        raise ContractError(f"characteristics: unknown domain {domain!r}")

    # ``samples`` is an explicit selection.  ``all`` would make the request
    # depend on whatever the plan happens to contain, so it is refused.
    samples = body.get("samples")
    if not isinstance(samples, Sequence) or isinstance(samples, (str, bytes)):
        raise ContractError("characteristics: samples must be a list of indices")
    if not samples:
        raise ContractError("characteristics: samples must select at least one index")
    seen: set[int] = set()
    for index in samples:
        if isinstance(index, bool) or not isinstance(index, int) or index < 0:
            raise ContractError("characteristics: samples must be non-negative integers")
        if index in seen:
            raise ContractError(f"characteristics: duplicate sample index {index}")
        seen.add(index)

    for key, label_key in (("measurements", "measurement"), ("perturbations", "perturbation")):
        entries = body.get(key)
        if entries is None:
            if key == "measurements":
                raise ContractError("characteristics: at least one measurement is required")
            continue
        if not isinstance(entries, Sequence) or isinstance(entries, (str, bytes)):
            raise ContractError(f"characteristics: {key} must be a list")
        if not entries:
            raise ContractError(f"characteristics: {key} must not be empty when present")
        for entry in entries:
            item = _require_mapping(entry, label_key)
            _check_declaration(item, label_key, kind=label_key)

    boundary = _require_mapping(body.get("boundary"), "characteristics.boundary")
    _reject_unknown(boundary, _BOUNDARY_FIELDS, "characteristics.boundary")
    _require_text(boundary, "profile", "characteristics.boundary")
    version = boundary.get("version")
    if not isinstance(version, int) or isinstance(version, bool) or version < 1:
        raise ContractError("characteristics: boundary requires a positive integer version")

    definition = _require_mapping(body.get("definition"), "characteristics.definition")
    _reject_unknown(definition, _DEFINITION_FIELDS, "characteristics.definition")
    kind = _require_text(definition, "kind", "characteristics.definition")
    if domain == "dynamic" and kind not in DYNAMIC_DEFINITION_KINDS:
        raise ContractError(
            f"characteristics: dynamic definition kind {kind!r} is not one of "
            f"{list(DYNAMIC_DEFINITION_KINDS)}"
        )
    if kind in DYNAMIC_DEFINITION_KINDS and domain != "dynamic":
        raise ContractError(
            f"characteristics: definition kind {kind!r} is dynamic-only"
        )
    definition_version = definition.get("version")
    if not isinstance(definition_version, int) or isinstance(definition_version, bool) or definition_version < 1:
        raise ContractError("characteristics: definition requires a positive integer version")

    frozen = body.get("frozen_conditions")
    if kind in ("frozen_force_tangent", "frozen_load_compliance"):
        frozen_row = _require_mapping(frozen, "characteristics.frozen_conditions")
        _reject_unknown(frozen_row, _FROZEN_FIELDS, "characteristics.frozen_conditions")
        _require_text(frozen_row, "protocol", "characteristics.frozen_conditions")
        frozen_version = frozen_row.get("version")
        if not isinstance(frozen_version, int) or isinstance(frozen_version, bool) or frozen_version < 1:
            raise ContractError(
                "characteristics: frozen_conditions requires a positive integer version"
            )
    elif frozen is not None:
        frozen_row = _require_mapping(frozen, "characteristics.frozen_conditions")
        _reject_unknown(frozen_row, _FROZEN_FIELDS, "characteristics.frozen_conditions")

    operator = _require_mapping(body.get("response_operator"), "characteristics.response_operator")
    _reject_unknown(operator, _OPERATOR_FIELDS, "characteristics.response_operator")
    _require_text(operator, "kind", "characteristics.response_operator")
    operator_version = operator.get("version")
    if not isinstance(operator_version, int) or isinstance(operator_version, bool) or operator_version < 1:
        raise ContractError("characteristics: response_operator requires a positive integer version")

    wheel_end = body.get("wheel_end_response")
    if wheel_end is not None and not isinstance(wheel_end, Mapping):
        raise ContractError("characteristics: wheel_end_response must be an object")


def request_document(
    request_id: str,
    domain: str,
    samples: Sequence[int],
    measurements: Sequence[Mapping[str, Any]],
    boundary: Mapping[str, Any],
    definition: Mapping[str, Any],
    response_operator: Mapping[str, Any],
    *,
    perturbations: Sequence[Mapping[str, Any]] | None = None,
    frozen_conditions: Mapping[str, Any] | None = None,
    wheel_end_response: Mapping[str, Any] | None = None,
    schema_version: int = REQUEST_VERSION,
) -> dict[str, Any]:
    """Build a request document; the argument order mirrors the spec's field list."""
    body: dict[str, Any] = {
        "request_id": request_id,
        "domain": domain,
        "samples": list(samples),
        "measurements": [dict(entry) for entry in measurements],
        "boundary": dict(boundary),
        "definition": dict(definition),
        "response_operator": dict(response_operator),
    }
    if perturbations is not None:
        body["perturbations"] = [dict(entry) for entry in perturbations]
    if frozen_conditions is not None:
        body["frozen_conditions"] = dict(frozen_conditions)
    if wheel_end_response is not None:
        body["wheel_end_response"] = dict(wheel_end_response)
    return {"schema_version": schema_version, "characteristics": body}


def split_request(document: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    """
    Return ``(document_without_request, request_body)``.

    The plan keeps the request verbatim; this helper exists so a caller can
    inspect either half without re-deriving the split.
    """
    validate_characteristics_request(document)
    remainder = {key: value for key, value in document.items() if key != "characteristics"}
    return remainder, dict(document["characteristics"])
