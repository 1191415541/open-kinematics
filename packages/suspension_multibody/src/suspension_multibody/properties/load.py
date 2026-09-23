"""
The properties file format, and the loader that refuses everything else.

Frozen here so the rest of the architecture can be written against it::

    {
      "schema_version": 1,
      "name": "baseline_compliance",
      "units": "mm-N-N*mm-kg-deg",
      "properties": {
        "front_spring": {"kind": "spring", "stiffness": 45.0, "free_length": 250.0},
        "front_bushing": {"kind": "bushing6x6", "stiffness": [[...6x6...]]}
      }
    }

Entry fields are the *same names* as the corresponding `schema/elements.py`
classes, so an entry is a drop-in replacement for a template's inline numbers
rather than a second vocabulary with its own rules about what "stiffness" means.
Validation therefore delegates to those classes instead of re-stating their
bounds -- one source of truth for "stiffness must be positive".
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from pydantic import ValidationError

from ..schema.common import StrictModel
from ..schema.elements import (
    BumpStop,
    Bushing6x6,
    LinearSpring,
    StaticDamper,
    VerticalTire,
)

__all__ = [
    "ENTRY_KINDS",
    "PropertiesDocument",
    "PropertiesError",
    "PropertySet",
    "load_properties",
]

#: The entry kinds a properties file may declare, mapped to the schema class
#: whose fields and bounds define the entry.
ENTRY_KINDS: dict[str, type[StrictModel]] = {
    "spring": LinearSpring,
    "damper": StaticDamper,
    "bushing6x6": Bushing6x6,
    "tire": VerticalTire,
    "bump_stop": BumpStop,
}

#: Fields the schema classes require that name the *placement* of an element
#: rather than its properties.  A properties file supplies numbers; the model
#: supplies where the element attaches.  Requiring them here would force every
#: file to repeat geometry that already exists in the model.
_PLACEMENT_FIELDS: dict[str, tuple[str, ...]] = {
    "spring": ("name", "body_a", "body_b", "point_a", "point_b"),
    "damper": ("name", "body_a", "body_b", "point_a", "point_b"),
    "bushing6x6": ("name", "body_a", "body_b"),
    "tire": ("contact_point",),
    "bump_stop": ("name", "body_a", "body_b", "point_a", "point_b"),
}


class PropertiesError(ValueError):
    """A properties file is malformed, or a property is missing or out of range."""


@dataclass(frozen=True)
class PropertySet:
    """One loaded properties file: named, validated entries."""

    path: Path
    name: str
    units: str
    entries: dict[str, dict[str, Any]]

    def __contains__(self, key: str) -> bool:
        return key in self.entries

    def __getitem__(self, key: str) -> dict[str, Any]:
        return self.entries[key]

    def keys(self) -> tuple[str, ...]:
        """Return the entry names, in file order."""
        return tuple(self.entries)

    def kind_of(self, key: str) -> str:
        """Return the declared kind of one entry."""
        return str(self.entries[key]["kind"])

    def as_values(self) -> dict[str, float]:
        """
        Return the scalar view of this file: entry name -> its `stiffness`.

        This is the bridge to `templates.instantiate`, whose property slots hold
        scalars.  A structured entry keeps its full form in `entries`; the scalar
        view is what a slot with a single number reads.
        """
        values: dict[str, float] = {}
        # A damper's scalar is its viscous damping; every other kind's is its
        # stiffness.  Kept in step with `templates.instantiate`'s own mapping.
        for key, entry in self.entries.items():
            field = "viscous_damping" if entry.get("kind") == "damper" else "stiffness"
            stiffness = entry.get(field)
            if isinstance(stiffness, (int, float)):
                values[key] = float(stiffness)
            elif isinstance(stiffness, list):
                # An isotropic 6x6 describes one translational stiffness, which is
                # what a scalar slot reads.  An anisotropic one has no scalar
                # view, so it simply does not appear here -- `resolve_properties`
                # is where refusing it, with a message naming the slot, belongs.
                diagonal = [
                    float(stiffness[index][index])
                    for index in range(min(3, len(stiffness)))
                    if index < len(stiffness[index])
                ]
                if len(diagonal) == 3 and len({round(v, 12) for v in diagonal}) == 1:
                    values[key] = diagonal[0]
        return values


@dataclass(frozen=True)
class PropertiesDocument:
    """The validated document, as written."""

    schema_version: int
    name: str
    units: str
    properties: dict[str, dict[str, Any]]


def load_properties(path: str | Path) -> PropertySet:
    """
    Load and validate a properties file.

    Every failure carries the file path, the property name, the field name and
    the reason.  No failure is a silent fallback to a default: a file that cannot
    be read in full is a wrong vehicle, and a wrong vehicle is not something a
    caller should discover from a number three steps later.
    """
    target = Path(path)
    try:
        text = target.read_text(encoding="utf-8")
    except OSError as exc:
        raise PropertiesError(f"{target}: cannot read properties file: {exc}") from exc
    try:
        payload = json.loads(text)
    except json.JSONDecodeError as exc:
        raise PropertiesError(f"{target}: invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise PropertiesError(
            f"{target}: properties document must be an object, found "
            f"{type(payload).__name__}"
        )
    document = _validate_document(target, payload)
    return PropertySet(
        path=target,
        name=document.name,
        units=document.units,
        entries=document.properties,
    )


def _validate_document(path: Path, payload: dict[str, Any]) -> PropertiesDocument:
    """Check the root object's shape and version."""
    allowed = {"schema_version", "name", "units", "properties"}
    unknown = sorted(set(payload) - allowed)
    if unknown:
        raise PropertiesError(
            f"{path}: unknown root key(s) {unknown}; allowed keys are "
            f"{sorted(allowed)}"
        )
    version = payload.get("schema_version")
    if version != 1:
        raise PropertiesError(f"{path}: schema_version must be 1, found {version!r}")
    if "properties" not in payload:
        raise PropertiesError(f"{path}: document has no 'properties' object")
    properties = payload["properties"]
    if not isinstance(properties, dict):
        raise PropertiesError(
            f"{path}: 'properties' must be an object, found "
            f"{type(properties).__name__}"
        )
    entries: dict[str, dict[str, Any]] = {}
    for name in properties:
        entries[str(name)] = _validate_entry(path, str(name), properties[name])
    return PropertiesDocument(
        schema_version=1,
        name=str(payload.get("name", path.stem)),
        units=str(payload.get("units", "")),
        properties=entries,
    )


def _validate_entry(path: Path, name: str, raw: Any) -> dict[str, Any]:
    """Validate one entry against the schema class its `kind` names."""
    if not isinstance(raw, dict):
        raise PropertiesError(
            f"{path}: property {name!r} must be an object, found {type(raw).__name__}"
        )
    kind = raw.get("kind")
    if kind is None:
        raise PropertiesError(f"{path}: property {name!r} has no 'kind'")
    if kind not in ENTRY_KINDS:
        raise PropertiesError(
            f"{path}: property {name!r} has unknown kind {kind!r}; known kinds "
            f"are {sorted(ENTRY_KINDS)}"
        )
    model = ENTRY_KINDS[kind]
    fields = set(model.model_fields)
    payload = dict(raw)
    unknown = sorted(set(payload) - {"kind"} - fields)
    if unknown:
        raise PropertiesError(
            f"{path}: property {name!r} (kind {kind!r}) has unknown field(s) "
            f"{unknown}; {kind} fields are {sorted(fields)}"
        )
    # Only fields with no default of their own are required: a schema class with
    # a sensible default (a zero damping, an identity attachment frame) does not
    # make a properties file incomplete by omitting it.
    required_fields = {
        field
        for field, definition in model.model_fields.items()
        if definition.is_required()
    }
    missing = sorted(
        field
        for field in required_fields
        if field not in payload and field not in _PLACEMENT_FIELDS[kind]
    )
    if missing:
        raise PropertiesError(
            f"{path}: property {name!r} (kind {kind!r}) is missing required "
            f"field(s) {missing}"
        )
    _check_types(path, name, kind, payload)
    values = {key: value for key, value in payload.items() if key != "kind"}
    try:
        # The schema class is the single source of truth for the bounds: a
        # negative stiffness is refused here for the same reason and by the same
        # rule the model's own inline numbers are.
        model.model_validate({"name": name, **values, **_placement_defaults(kind)})
    except ValidationError as exc:
        raise PropertiesError(
            f"{path}: property {name!r} (kind {kind!r}) failed validation: "
            f"{_first_problem(exc)}"
        ) from exc
    return {"kind": kind, **values}


def _placement_defaults(kind: str) -> dict[str, Any]:
    """Return placeholder values for the placement fields validation needs."""
    defaults: dict[str, Any] = {}
    for field in _PLACEMENT_FIELDS[kind]:
        if field == "name":
            defaults[field] = "validation"
        elif field in {"body_a", "body_b"}:
            defaults[field] = "chassis"
        elif field in {"point_a", "point_b", "contact_point"}:
            defaults[field] = {"x": 0.0, "y": 0.0, "z": 0.0}
    return defaults


def _check_types(path: Path, name: str, kind: str, payload: dict[str, Any]) -> None:
    """
    Reject a field whose JSON type cannot even be interpreted.

    The schema classes catch these too, but their message says "input should be a
    valid number"; naming the property and the field is what makes the message
    usable.
    """
    for field, value in payload.items():
        if field == "kind":
            continue
        if field == "stiffness":
            if isinstance(value, str):
                raise PropertiesError(
                    f"{path}: property {name!r} (kind {kind!r}) field 'stiffness' "
                    f"must be a number, found {value!r}"
                )
            if isinstance(value, list):
                if kind != "bushing6x6":
                    raise PropertiesError(
                        f"{path}: property {name!r} (kind {kind!r}) field "
                        f"'stiffness' must be a number, found a matrix"
                    )
                rows = len(value)
                widths = {len(row) for row in value if isinstance(row, list)}
                if rows != 6 or widths != {6}:
                    raise PropertiesError(
                        f"{path}: property {name!r} (kind {kind!r}) field "
                        f"'stiffness' must be a 6x6 matrix, found "
                        f"{rows}x{sorted(widths)}"
                    )
            continue
        if field in {"free_length", "reference_length", "unloaded_radius"} and isinstance(
            value, str
        ):
            raise PropertiesError(
                f"{path}: property {name!r} (kind {kind!r}) field {field!r} must "
                f"be a number, found {value!r}"
            )


def _first_problem(exc: ValidationError) -> str:
    """Return one problem from a pydantic failure, with its field name."""
    errors = exc.errors()
    if not errors:
        return str(exc)
    first = errors[0]
    location = ".".join(str(part) for part in first.get("loc", ()))
    return f"{location}: {first.get('msg', 'invalid')}"
