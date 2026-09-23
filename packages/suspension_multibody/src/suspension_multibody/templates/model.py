"""
The template data model.

One structure covers every subsystem, of every kind, simplified or detailed.  The
differences between a suspension template and a brake template are *content*: the
parts it lists, the connections it declares, the slots it asks a properties file
to fill, the outputs it contributes.  The shape is identical, which is what makes
them interchangeable within a role and what makes a new role a declaration rather
than a new class.

That is also how Adams Car works: a `.tpl` is one file format for every subsystem,
and `MAJOR_ROLE` is a field inside it rather than a different kind of file.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Literal

from .roles import RoleSpec, get_role

__all__ = [
    "ConnectionDefinition",
    "OutputDeclaration",
    "PartDefinition",
    "PropertySlot",
    "Template",
    "TemplateError",
]


class TemplateError(ValueError):
    """A template is malformed, or violates its role's contract."""


#: Which of a connection's two columns a joint name came from.  The joint column
#: is what a K-mode instance activates; the bushing column is what a C-mode
#: instance activates.  A point may declare either, both, or neither.
Column = Literal["joint", "bushing"]


@dataclass(frozen=True)
class PartDefinition:
    """One body a template declares."""

    name: str
    #: Mass in kg.  A template may leave this to the properties file by
    #: declaring a slot of the same name instead.
    mass: float = 0.0
    #: Whether the assembly holds this body fixed.
    fixed: bool = False


@dataclass(frozen=True)
class ConnectionDefinition:
    """
    One attachment point, carrying a joint column and a bushing column.

    Both columns are optional, and so is both being absent:

    * **both absent** -- the point exists only to locate geometry (the inboard
      rear hardpoint is used to define the arm's rotation axis and carries no
      constraint of its own in K mode);
    * **joint only** -- the point is a joint in both modes (arm outer points, tie
      rod ends, the rack guide);
    * **bushing only** -- the point is compliant in both modes;
    * **both present** -- the K/C choice decides, which is the case the user
      described: the same point is a revolute in K and a bushing in C.

    `mount` names the hardpoint role that locates the point, so a template can be
    read against an existing hardpoint set.
    """

    name: str
    role: str
    #: The joint column: a document joint type name (see `joints.table`).
    joint: str | None = None
    #: The bushing column: the name a properties file supplies stiffness for.
    bushing: str | None = None

    def columns(self) -> tuple[Column, ...]:
        """Return which columns this connection declares, in a stable order."""
        declared: list[Column] = []
        if self.joint is not None:
            declared.append("joint")
        if self.bushing is not None:
            declared.append("bushing")
        return tuple(declared)


@dataclass(frozen=True)
class PropertySlot:
    """
    Something the template asks a properties file to provide.

    A slot carries a name and a unit so a properties file can be checked before
    it is used: a stiffness supplied in N/mm where N/m was declared is a wrong
    model, not a rounding difference.
    """

    name: str
    unit: str
    #: Default value, for templates whose simplified form carries its own numbers
    #: rather than referring to a file.
    default: float | None = None
    #: Connection names that read this slot.
    #:
    #: One slot can feed several connections: a template declares *one* mount
    #: bushing and the left and right arms share it, because the number an author
    #: writes down is one number.  Without this mapping the assembly could only
    #: find a slot by guessing from a connection name, which would make the
    #: template's own naming load-bearing.
    connections: tuple[str, ...] = ()


@dataclass(frozen=True)
class OutputDeclaration:
    """
    One minimum-unit output the template contributes.

    Only the declaration is built here; evaluating an output is a later concern.
    """

    name: str
    unit: str
    #: Where the value comes from, e.g. "kernel" or a derived expression name.
    source: str = "kernel"


@dataclass(frozen=True)
class Template:
    """
    One subsystem template: a role, and everything that implements it.

    The structure does not vary by role.  A brake template and a suspension
    template are the same dataclass with different contents; that is what makes
    them one format rather than two.

    `elastic_slots` names the spring/damper elements the template declares, and
    `suspension_kind` names the category the template belongs to (a double
    wishbone, a MacPherson strut) so the registry can be searched by it.
    """

    name: str
    role: str
    parts: tuple[PartDefinition, ...] = ()
    connections: tuple[ConnectionDefinition, ...] = ()
    elastic_slots: tuple[str, ...] = ()
    property_slots: tuple[PropertySlot, ...] = ()
    outputs: tuple[OutputDeclaration, ...] = ()
    #: The template family, e.g. "double_wishbone".  Distinct from `role`.
    suspension_kind: str = ""
    #: Free-text description for the registry listing.
    description: str = ""

    @property
    def role_spec(self) -> RoleSpec:
        """Return this template's role declaration."""
        return get_role(self.role)

    def check_role_contract(self) -> None:
        """
        Reject a template that does not satisfy its role's contract.

        Called at registration and again at instantiation, because a template can
        be registered with placeholders and only filled in later.  Every failure
        names the specific missing mount or slot: "template is invalid" would not
        tell an author which hardpoint to add.
        """
        spec = self.role_spec
        declared_mounts = {connection.role for connection in self.connections}
        missing_mounts = sorted(set(spec.required_mounts) - declared_mounts)
        if missing_mounts:
            raise TemplateError(
                f"template {self.name!r} does not satisfy role {self.role!r}: "
                f"missing mount(s) {missing_mounts}"
            )
        declared_slots = {slot.name for slot in self.property_slots}
        missing_slots = sorted(set(spec.required_slots) - declared_slots)
        if missing_slots:
            raise TemplateError(
                f"template {self.name!r} does not satisfy role {self.role!r}: "
                f"missing property slot(s) {missing_slots}"
            )
        self._check_uniqueness()

    def check_filled(self, supplied: dict[str, object]) -> None:
        """
        Reject a template whose required slots were not filled.

        `supplied` is what a properties file provided.  A slot with a default of
        its own counts as filled, which is how a simplified template carries its
        own numbers.
        """
        spec = self.role_spec
        missing: list[str] = []
        for slot_name in spec.required_slots:
            if slot_name in supplied:
                continue
            slot = next((s for s in self.property_slots if s.name == slot_name), None)
            if slot is not None and slot.default is not None:
                continue
            missing.append(slot_name)
        if missing:
            raise TemplateError(
                f"template {self.name!r} (role {self.role!r}) has unfilled "
                f"required property slot(s) {sorted(missing)}"
            )

    def _check_uniqueness(self) -> None:
        names = [part.name for part in self.parts]
        if len(set(names)) != len(names):
            raise TemplateError(f"template {self.name!r} declares a part twice")
        connection_names = [connection.name for connection in self.connections]
        if len(set(connection_names)) != len(connection_names):
            raise TemplateError(f"template {self.name!r} declares a connection twice")
        # A role may legitimately appear at several points (an arm has an inner
        # and an outer hardpoint), so roles are not required to be unique -- but
        # property slots are, because a name collision would make the properties
        # file ambiguous.
        slot_names = [slot.name for slot in self.property_slots]
        if len(set(slot_names)) != len(slot_names):
            duplicates = sorted({n for n in slot_names if slot_names.count(n) > 1})
            raise TemplateError(
                f"template {self.name!r} declares duplicate property slot(s) "
                f"{duplicates}"
            )


def _parts_to_json(template: Template) -> list[dict[str, Any]]:
    return [
        {"name": part.name, "mass": part.mass, "fixed": part.fixed}
        for part in template.parts
    ]


def _connections_to_json(template: Template) -> list[dict[str, Any]]:
    return [
        {
            "name": connection.name,
            "role": connection.role,
            "joint": connection.joint,
            "bushing": connection.bushing,
        }
        for connection in template.connections
    ]


def template_to_json(template: Template) -> dict[str, Any]:
    """
    Serialise a template to a plain mapping, ready for JSON.

    Templates are declared by experts and shared, so the round trip has to be
    exact; `template_from_json` is its inverse and the pair is tested together.
    """
    return {
        "name": template.name,
        "role": template.role,
        "parts": _parts_to_json(template),
        "connections": _connections_to_json(template),
        "elastic_slots": list(template.elastic_slots),
        "property_slots": [
            {
                "name": slot.name,
                "unit": slot.unit,
                "default": slot.default,
                "connections": list(slot.connections),
            }
            for slot in template.property_slots
        ],
        "outputs": [
            {"name": output.name, "unit": output.unit, "source": output.source}
            for output in template.outputs
        ],
        "suspension_kind": template.suspension_kind,
        "description": template.description,
    }


def template_from_json(payload: dict[str, Any]) -> Template:
    """Rebuild a template from `template_to_json` output."""
    return Template(
        name=str(payload["name"]),
        role=str(payload["role"]),
        parts=tuple(
            PartDefinition(
                name=str(part["name"]),
                mass=float(part.get("mass", 0.0)),
                fixed=bool(part.get("fixed", False)),
            )
            for part in payload.get("parts", ())
        ),
        connections=tuple(
            ConnectionDefinition(
                name=str(connection["name"]),
                role=str(connection["role"]),
                joint=connection.get("joint"),
                bushing=connection.get("bushing"),
            )
            for connection in payload.get("connections", ())
        ),
        elastic_slots=tuple(str(s) for s in payload.get("elastic_slots", ())),
        property_slots=tuple(
            PropertySlot(
                name=str(slot["name"]),
                unit=str(slot["unit"]),
                default=None if slot.get("default") is None else float(slot["default"]),
                connections=tuple(str(n) for n in slot.get("connections", ())),
            )
            for slot in payload.get("property_slots", ())
        ),
        outputs=tuple(
            OutputDeclaration(
                name=str(output["name"]),
                unit=str(output["unit"]),
                source=str(output.get("source", "kernel")),
            )
            for output in payload.get("outputs", ())
        ),
        suspension_kind=str(payload.get("suspension_kind", "")),
        description=str(payload.get("description", "")),
    )


def template_dumps(template: Template) -> str:
    """Serialise a template to a JSON string."""
    return json.dumps(template_to_json(template), sort_keys=True, indent=2)


def template_loads(text: str) -> Template:
    """Rebuild a template from `template_dumps` output."""
    return template_from_json(json.loads(text))
