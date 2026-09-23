"""
Multibody Contract V1: canonical form, container format, and validation.

This module is the Python half of the contract boundary described in
``ARCHITECTURE.md``.  The C++ kernel consumes the same documents, so three
things are nailed down here and nowhere else:

* **canonical form** -- a byte-exact serialisation (sorted keys, no incidental
  whitespace, ``%.17g`` for finite floats, no NaN/Inf) so a document hash is
  reproducible on both sides;
* **container** -- ``magic | version | json_length | blob_length | json | blob``
  with little-endian integers, keeping bulk numeric arrays out of JSON;
* **validation** -- a structural check against the versioned JSON Schema using
  a deliberately small JSON Schema subset, so the package keeps its zero
  runtime dependencies.

The schemas live in ``contracts/`` next to this module and ship in the wheel.
"""

from __future__ import annotations

import hashlib
import json
import math
import struct
from collections.abc import Mapping, Sequence
from importlib import resources
from typing import Any

CONTRACT_MAGIC = b"MBC1"
CONTRACT_VERSION = 1
_HEADER = struct.Struct("<4sIQQ")

SCHEMA_FILES = {
    "model": "multibody_model.schema.json",
    "case": "multibody_case.schema.json",
    "result": "multibody_result.schema.json",
}


class ContractError(ValueError):
    """Raised when a document is not a valid member of the contract."""


def _encode(value: Any, path: str) -> str:
    if value is None:
        return "null"
    if value is True:
        return "true"
    if value is False:
        return "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if math.isnan(value) or math.isinf(value):
            raise ContractError(f"{path}: NaN and infinity are not representable")
        text = "%.17g" % value
        # ``%.17g`` renders -0.0 as "-0" and 1.0 as "1"; JSON then parses those
        # as integers, which loses the sign of negative zero and the float/int
        # distinction.  Force an unambiguous float literal.
        if not any(marker in text for marker in ".eE"):
            text += ".0"
        return text
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, Mapping):
        parts = []
        for key in sorted(value):
            if not isinstance(key, str):
                raise ContractError(f"{path}: object keys must be strings")
            parts.append(
                json.dumps(key, ensure_ascii=False)
                + ":"
                + _encode(value[key], f"{path}/{key}")
            )
        return "{" + ",".join(parts) + "}"
    if isinstance(value, Sequence):
        items = [
            _encode(item, f"{path}[{index}]") for index, item in enumerate(value)
        ]
        return "[" + ",".join(items) + "]"
    raise ContractError(f"{path}: unsupported value type {type(value).__name__}")


def canonical_json(value: Any) -> str:
    """Return the canonical JSON text for ``value``."""
    return _encode(value, "$")


def canonical_json_bytes(value: Any) -> bytes:
    """Return the canonical JSON bytes for ``value``."""
    return canonical_json(value).encode("utf-8")


def contract_hash(value: Any) -> str:
    """Return the SHA-256 of the canonical form, as lowercase hex."""
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def parse_json(payload: bytes | str) -> Any:
    """Parse JSON produced by either side of the boundary."""
    if isinstance(payload, bytes):
        payload = payload.decode("utf-8")
    return json.loads(payload)


def pack_container(document: Mapping[str, Any], blob: bytes = b"") -> bytes:
    """Serialise ``document`` (plus optional blob section) into one payload."""
    body = canonical_json_bytes(document)
    blob = bytes(blob)
    return (
        _HEADER.pack(CONTRACT_MAGIC, CONTRACT_VERSION, len(body), len(blob))
        + body
        + blob
    )


def unpack_container(payload: bytes) -> tuple[dict[str, Any], bytes]:
    """Return ``(document, blob)`` from a payload built by :func:`pack_container`."""
    if len(payload) < _HEADER.size:
        raise ContractError("payload is shorter than the container header")
    magic, version, json_length, blob_length = _HEADER.unpack_from(payload)
    if magic != CONTRACT_MAGIC:
        raise ContractError(f"unexpected container magic {magic!r}")
    if version != CONTRACT_VERSION:
        raise ContractError(
            f"unsupported container version {version} (expected {CONTRACT_VERSION})"
        )
    start = _HEADER.size
    json_end = start + json_length
    blob_end = json_end + blob_length
    if blob_end != len(payload):
        raise ContractError(
            f"container length mismatch: header says {blob_end}, got {len(payload)}"
        )
    document = parse_json(payload[start:json_end])
    if not isinstance(document, dict):
        raise ContractError("container document must be a JSON object")
    return document, payload[json_end:blob_end]


def blob_slice(blob: bytes, descriptor: Mapping[str, Any]) -> bytes:
    """Return the bytes a descriptor references inside the blob section."""
    offset = int(descriptor.get("offset", 0))
    length = int(descriptor.get("length", 0))
    if offset < 0 or length < 0 or offset + length > len(blob):
        raise ContractError(
            f"blob descriptor out of range: offset={offset} length={length} "
            f"blob={len(blob)}"
        )
    return blob[offset : offset + length]


def load_schema(name: str) -> dict[str, Any]:
    """Load a bundled schema by short name (``model``/``case``/``result``)."""
    try:
        filename = SCHEMA_FILES[name]
    except KeyError:
        raise ContractError(f"unknown schema {name!r}") from None
    text = (
        resources.files("suspension_contracts.contracts")
        .joinpath(filename)
        .read_text(encoding="utf-8")
    )
    return json.loads(text)


def _type_matches(value: Any, name: str) -> bool:
    if name == "object":
        return isinstance(value, dict)
    if name == "array":
        return isinstance(value, list)
    if name == "string":
        return isinstance(value, str)
    if name == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if name == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if name == "boolean":
        return isinstance(value, bool)
    raise ContractError(f"unsupported schema type {name!r}")


def _check(
    value: Any, schema: Mapping[str, Any], path: str, root: Mapping[str, Any]
) -> None:
    ref = schema.get("$ref")
    if isinstance(ref, str):
        if not ref.startswith("#/"):
            raise ContractError(f"{path}: unsupported $ref {ref!r}")
        target: Any = root
        for token in ref[2:].split("/"):
            target = target[token]
        _check(value, target, path, root)
        return

    if "const" in schema and value != schema["const"]:
        raise ContractError(f"{path}: expected {schema['const']!r}, got {value!r}")
    if "enum" in schema and value not in schema["enum"]:
        raise ContractError(f"{path}: {value!r} is not one of {schema['enum']}")

    declared = schema.get("type")
    if isinstance(declared, str) and not _type_matches(value, declared):
        raise ContractError(f"{path}: expected {declared}, got {type(value).__name__}")

    if isinstance(value, dict):
        for key in schema.get("required", []):
            if key not in value:
                raise ContractError(f"{path}: missing required field {key!r}")
        properties = schema.get("properties", {})
        if schema.get("additionalProperties") is False:
            extra = sorted(set(value) - set(properties))
            if extra:
                raise ContractError(f"{path}: unexpected fields {extra}")
        for key, sub in properties.items():
            if key in value:
                _check(value[key], sub, f"{path}/{key}", root)
    elif isinstance(value, list):
        items = schema.get("items")
        if isinstance(items, dict):
            for index, item in enumerate(value):
                _check(item, items, f"{path}[{index}]", root)
        minimum_items = schema.get("minItems")
        if isinstance(minimum_items, int) and len(value) < minimum_items:
            raise ContractError(f"{path}: needs at least {minimum_items} items")
        maximum_items = schema.get("maxItems")
        if isinstance(maximum_items, int) and len(value) > maximum_items:
            raise ContractError(f"{path}: allows at most {maximum_items} items")
    elif isinstance(value, str):
        minimum_length = schema.get("minLength")
        if isinstance(minimum_length, int) and len(value) < minimum_length:
            raise ContractError(f"{path}: needs at least {minimum_length} characters")
        maximum_length = schema.get("maxLength")
        if isinstance(maximum_length, int) and len(value) > maximum_length:
            raise ContractError(f"{path}: allows at most {maximum_length} characters")
    elif isinstance(value, (int, float)) and not isinstance(value, bool):
        if schema.get("finite") is True and not math.isfinite(value):
            raise ContractError(f"{path}: must be finite")
        minimum = schema.get("minimum")
        if isinstance(minimum, (int, float)) and value < minimum:
            raise ContractError(f"{path}: must be >= {minimum}")

def validate(document: Mapping[str, Any], kind: str) -> None:
    """Validate ``document`` against the schema registered for ``kind``."""
    schema = load_schema(kind)
    _check(document, schema, "$", schema)


def validate_model(document: Mapping[str, Any]) -> None:
    """Validate a model document."""
    validate(document, "model")


def validate_case(document: Mapping[str, Any]) -> None:
    """Validate a case document."""
    validate(document, "case")


def validate_result(document: Mapping[str, Any]) -> None:
    """Validate a result document."""
    validate(document, "result")
