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
    "template": "template.schema.json",
    "element_properties": "element_properties.schema.json",
    "subsystem": "subsystem.schema.json",
    "assembly": "assembly.schema.json",
    "rig": "rig.schema.json",
    "resolved_model": "resolved_model.schema.json",
    "solve_plan": "solve_plan.schema.json",
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
    arities = {"constant": 0, "binding": 0, "identity": 1, "neg": 1,
               "add": 2, "sub": 2, "mul": 2, "div": 2, "pow": 2,
               "sin": 1, "cos": 1, "tanh": 1, "exp": 1, "sqrt": 1,
               "step": 5, "curve": 1, "surface": 2}
    for pindex, program in enumerate(document.get("function_programs", ())):
        nodes = program["nodes"]
        for nindex, node in enumerate(nodes):
            if node["op"] not in arities:
                raise ContractError(f"$/function_programs[{pindex}]/nodes[{nindex}]/op: unsupported operation {node['op']!r}")
            args = node.get("args", ())
            if len(args) != arities[node["op"]]:
                raise ContractError(f"function node {nindex}: invalid argument count")
            for arg in args:
                if arg >= nindex:
                    raise ContractError(f"$/function_programs[{pindex}]/nodes[{nindex}]/args: program must be a DAG")
            if node["op"] == "constant" and "value" not in node:
                raise ContractError("function constant requires a finite value")
            if node["op"] == "binding" and not 0 <= node.get("binding", -1) < len(program["bindings"]):
                raise ContractError("function node names an unknown binding")
            if node["op"] == "pow":
                exponent = nodes[args[1]]
                value = exponent.get("value", 0.5)
                if exponent["op"] != "constant" or value != int(value):
                    raise ContractError("function power exponent must be an integer constant")
            if node["op"] in {"curve", "surface"}:
                index = node.get("table", -1)
                tables = program.get("tables", ())
                if not 0 <= index < len(tables) or tables[index].get("dimension") != (1 if node["op"] == "curve" else 2):
                    raise ContractError("function node names an incompatible table")
        output = program["outputs"]["value"]
        if output >= len(nodes):
            raise ContractError(f"$/function_programs[{pindex}]/outputs/value: unknown node")
        for binding in program["bindings"]:
            source = binding.get("source", "constant")
            if source not in {"time", "constant", "property", "signal", "channel", "measurement"}:
                raise ContractError("unsupported function binding source")
            if source in {"constant", "property"} and "value" not in binding:
                raise ContractError("function constant/property binding requires a value")
            for key in ("value", "scale"):
                if key in binding and (not isinstance(binding[key], (int, float)) or not math.isfinite(binding[key])):
                    raise ContractError("function binding must be finite")
            if source in {"signal", "channel"}:
                _function_points(binding.get("samples"), "signal")
            if source == "measurement":
                if binding.get("measurement") not in {"position", "relative_position", "relative_velocity", "relative_angular_velocity"}:
                    raise ContractError("unsupported function measurement")
                for key in ("action", "reaction", "reference"):
                    marker = binding.get(key)
                    if not isinstance(marker, dict) or marker.get("body") not in {body["name"] for body in document["bodies"]}:
                        raise ContractError("function measurement requires explicit marker poses")
        for table in program.get("tables", ()):
            if table.get("extrapolation") not in {"clamp", "linear", "error"}:
                raise ContractError("unsupported function table extrapolation")
            if table.get("dimension") == 1:
                if table.get("interpolation") not in {"piecewise_linear", "akima"}:
                    raise ContractError("unsupported function curve interpolation")
                _function_points(table.get("points"), "curve")
            elif table.get("dimension") == 2:
                if table.get("interpolation") != "bilinear":
                    raise ContractError("unsupported function surface interpolation")
                xs, ys, rows = (table.get(key, ()) for key in ("x_axis", "y_axis", "values"))
                for axis in (xs, ys):
                    if len(axis) < 2 or any(not isinstance(x, (int, float)) or not math.isfinite(x) for x in axis) or any(b <= a for a, b in zip(axis, axis[1:])):
                        raise ContractError("function surface axes must strictly increase")
                if len(rows) != len(xs) or any(not isinstance(row, list) or len(row) != len(ys) for row in rows):
                    raise ContractError("function surface values must match its axes")
                if any(not isinstance(value, (int, float)) or not math.isfinite(value) for row in rows for value in row):
                    raise ContractError("function surface values must be finite")
            else:
                raise ContractError("unsupported function table dimension")
    for index, tire in enumerate(document.get("tires", ())):
        matrix = tire.get("inertia")
        if matrix is None:
            continue
        scale = max(abs(value) for row in matrix for value in row)
        if scale == 0:
            continue
        normalized = [[value / scale for value in row] for row in matrix]
        if any(abs(normalized[i][j] - normalized[j][i]) > 1e-12 for i in range(3) for j in range(3)):
            raise ContractError(f"$/tires[{index}]/inertia: must be symmetric")
        # I = trace(C) identity - C, where C is the positive semidefinite
        # second moment of mass. This includes the principal-inertia triangle
        # inequalities and permits either sign of off-diagonal entries.
        half_trace = sum(normalized[i][i] for i in range(3)) / 2
        second_moment = [[(half_trace if i == j else 0.0) - normalized[i][j] for j in range(3)] for i in range(3)]
        a, b, c = (second_moment[i][i] for i in range(3))
        d, e, f = second_moment[0][1], second_moment[0][2], second_moment[1][2]
        minors = (a, b, c, a*b-d*d, a*c-e*e, b*c-f*f, a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d)
        if min(minors) < -1e-12:
            raise ContractError(f"$/tires[{index}]/inertia: must be a physical positive semidefinite tensor")


def _function_points(points: Any, label: str) -> None:
    if not isinstance(points, list) or len(points) < 2 or any(
        not isinstance(row, list) or len(row) != 2 or any(
            not isinstance(x, (int, float)) or not math.isfinite(x) for x in row
        ) for row in points
    ):
        raise ContractError(f"function {label} requires at least two finite pairs")
    if any(b[0] <= a[0] for a, b in zip(points, points[1:])):
        raise ContractError(f"function {label} abscissas must strictly increase")


def validate_case(document: Mapping[str, Any]) -> None:
    """Validate a case document."""
    validate(document, "case")


def validate_result(document: Mapping[str, Any]) -> None:
    """Validate a result document."""
    validate(document, "result")
def validate_template(document: Mapping[str, Any]) -> None:
    """Validate a declarative subsystem template document."""
    validate(document, "template")
    for element in document["elements"]:
        kind = element["type"]
        if kind in {"force", "torque", "wrench"}:
            fields = ("action", "reaction", "reference", "functions" if kind == "wrench" else "function")
        elif kind in {"aerodynamic_drag", "point_wrench", "gravity", "steering_actuator"}:
            fields = ("parameters",)
        else:
            fields = ("body_a", "body_b", "property_slot")
        for key in fields:
            if key not in element:
                raise ContractError(f"element {element['name']!r}: missing required field {key!r}")


def validate_element_properties(document: Mapping[str, Any]) -> None:
    """Validate a spring, damper, or bump-stop property document."""
    validate(document, "element_properties")


def validate_subsystem(document: Mapping[str, Any]) -> None:
    """Validate a subsystem instance document."""
    validate(document, "subsystem")


def validate_assembly(document: Mapping[str, Any]) -> None:
    """Validate an assembly document."""
    validate(document, "assembly")


def validate_rig(document: Mapping[str, Any]) -> None:
    """Validate a test-rig document."""
    validate(document, "rig")
