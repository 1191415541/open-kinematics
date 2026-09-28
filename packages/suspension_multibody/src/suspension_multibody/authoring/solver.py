"""
From authoring documents to a solved K/C run.

This is the compatibility conversion the plan calls for, and it is deliberately
narrow: it turns a *file* template into the runtime ``Template`` the existing
composition already understands, and a *file* subsystem into a ``FrontAxleModel``.
Everything downstream -- assembly, rig binding, contract authoring, the kernel --
is the code that already exists, unchanged.  Nothing here re-implements a solve.

The mapping between the two template shapes, stated once:

* a file template's **bodies** are the runtime template's parts;
* a file **hardpoint** is a runtime *connection*, because that is the unit the
  runtime resolves: a connection carries the point's owner, its label, the joint
  column that constrains it and the bushing column that makes it compliant;
* a file **joint** supplies the joint column and, through its far endpoint, the
  far body a two-ended mount spans;
* a file **element** of type ``bushing`` supplies the bushing column; the other
  element kinds become the model's force elements and are built by the bridge;
* a file **property slot** becomes a runtime slot, keeping the slot's ``unit``
  and its ``connections`` list, which is what lets one slot feed several points.

What the file format cannot express is also stated, because a conversion that
quietly dropped it would produce a subsystem that differs from the built-in one
without saying so: per-mode joint-kind overrides, per-mode column activation,
port needs and output declarations have no file spelling, so the conversion uses
the runtime defaults for them and refuses rather than guesses when a template
depends on a value it cannot carry.
"""

from __future__ import annotations

import re
from dataclasses import replace
from typing import Any, Literal, Mapping

from ..schema import FrontAxleModel
from ..subsystems.types import AssemblyRequest
from ..templates.instantiate import SubsystemInstance, instantiate
from ..templates.model import (
    ConnectionDefinition,
    OutputDeclaration,
    PartDefinition,
    PropertySlot,
    Template,
    TemplateError,
)
from .bridge import BridgeError, front_axle_model_from
from .documents import EffectiveSubsystem, SubsystemDocument, TemplateDocument

__all__ = [
    "assembly_request_from",
    "front_axle_model_for",
    "runtime_template_from",
    "template_document_from",
]

#: Units for the slots a role requires but a file need not declare.  Stated so a
#: synthesised slot carries the unit its value is in rather than a placeholder.
_SLOT_UNITS: dict[str, str] = {
    "spring": "N/m",
    "damper": "N*s/m",
    "bushing": "N/m",
}

#: The connection column each synthesised slot feeds, where one does.  A slot with
#: no entry feeds nothing directly; ``spring`` and ``damper`` are read by the model
#: through the bridge, not through a connection's column.
#: The slot a mount's compliant column reads.  Named once because the whole layer
#: refers to it: a file writes its name, the conversion binds it, the export writes
#: it back.
_MOUNT_SLOT = "bushing"
_COLUMN_OF: dict[str, str] = {"bushing": "bushing"}
#: Element kinds that become a model force element rather than a template column.
_FORCE_KINDS = frozenset({"spring", "damper", "bump_stop"})

#: The side a file template declares; the other side is mirrored from it.
_MODEL_SIDE = "L"

#: The two modes a file template may activate a column in.
_BOTH_MODES: tuple[str, ...] = ("K", "C")


def _modes_of(
    row: Mapping[str, Any] | None, *, default: tuple[str, ...]
) -> tuple[str, ...]:
    """
    Return the modes a declaration activates in.

    A declaration that says nothing activates in both modes, which is the reading
    that makes "this mount is rigid in K and compliant in C" the *default* rather
    than something every mount has to spell out: both columns declared, and the
    mode choosing between them.  A point that really is inert in one reading says
    so by naming the modes it is live in.
    """
    if row is None:
        return default
    declared = row.get("modes")
    return tuple(str(mode) for mode in declared) if declared else default

def runtime_template_from(document: TemplateDocument) -> Template:
    """
    Convert a file template into the runtime ``Template`` the composition reads.

    The result is an ordinary template, so choosing a file template produces a
    different subsystem through exactly the path a registered Python template
    would -- which is what makes the file format an authoring route *into* the
    model rather than a parallel description of it.

    The side the file declares is expanded to both sides.  A file template writes
    one side, as the model does, while the composition resolves each side against
    parts and connections whose names carry that side's suffix; a template
    declaring only ``_L`` names would contribute nothing on the right.  Mirroring
    here keeps one description of the geometry instead of asking every author to
    write the same axle twice, and it is what makes a file template a *symmetric*
    template rather than half of one.
    """
    payload = document.payload
    parts = _mirrored_parts(payload)
    connections = _mirrored_connections(_connections_from(payload))
    slots = _slots_from(payload, connections)
    outputs = tuple(
        OutputDeclaration(
            name=str(row["name"]),
            unit=str(row["unit"]),
            source=str(row.get("source", "kernel")),
        )
        for row in payload.get("outputs", ())
    )
    try:
        template = Template(
            name=document.name,
            role=document.functional_role,
            parts=parts,
            connections=connections,
            elastic_slots=tuple(
                str(row["name"])
                for row in payload["elements"]
                if str(row["type"]) in _FORCE_KINDS
            ),
            property_slots=slots,
            outputs=outputs,
            suspension_kind=str(payload.get("suspension_kind", "")),
            description=str(payload.get("description", "")),
        )
        template.check_role_contract()
    except TemplateError as exc:
        raise BridgeError(
            f"{document.path}: the file template is not a valid "
            f"{document.functional_role} template for the solver: {exc}"
        ) from exc
    return template


def _connections_from(payload: Mapping[str, Any]) -> tuple[ConnectionDefinition, ...]:
    """
    Build one runtime connection per declared hardpoint.

    A joint acts at **one** location: the hardpoint its ``point_a`` names.  Its two
    bodies share that point, and the connection for the point records the joint
    column plus the far body the joint spans.  A joint whose endpoints name two
    different hardpoints is refused rather than interpreted -- naming two distinct
    points would silently add "these two points coincide" to the model, which is a
    constraint the author did not ask for and which shows up as a rank-deficient
    Jacobian rather than as an authoring mistake.

    Which body is the connection's own end is decided by the hardpoint's **owner**,
    not by the endpoint order the joint happened to use: a rack guide is declared
    between the rack and the chassis, and reading the order instead of the owners
    would make the far body the rack itself -- a joint from a body to itself.
    """
    joints = payload["joints"]
    bushing_by_point: dict[str, str] = {}
    bushing_modes_by_point: dict[str, tuple[str, ...]] = {}
    for element in payload["elements"]:
        if str(element["type"]) != "bushing":
            continue
        point = str(element["point_a"])
        bushing_by_point.setdefault(point, str(element["name"]))
        # A bushing says nothing by default: a mount that declares both columns
        # lets the mode choose (rigid in K, compliant in C), which is exactly how
        # the built-in's inboard mounts behave.  A point that is compliant in C
        # *instead of* constrained there says so by writing its modes.
        bushing_modes_by_point.setdefault(
            point, _modes_of(element, default=_BOTH_MODES)
        )

    by_point: dict[str, Mapping[str, Any]] = {}
    for joint in joints:
        anchor = str(joint["point_a"])
        other = joint.get("point_b")
        if other is not None and str(other) != anchor:
            raise BridgeError(
                f"joint {joint['name']!r} names two hardpoints ({anchor!r} and "
                f"{other!r}); a joint acts at one point, so state the point it "
                "constrains and let the two bodies share it"
            )
        if anchor in by_point:
            raise BridgeError(
                f"two joints are declared at hardpoint {anchor!r}; one point carries "
                "one joint"
            )
        by_point[anchor] = joint

    connections: list[ConnectionDefinition] = []
    for row in payload["hardpoints"]:
        point = str(row["name"])
        owner = str(row["owner"])
        label = str(row.get("label", "")) or point
        joint = by_point.get(point)
        joint_type = str(joint["type"]) if joint is not None else None
        joint_modes = _modes_of(joint, default=_BOTH_MODES) if joint else _BOTH_MODES
        joint_kind_by_mode = (
            tuple(
                (str(pair["mode"]), str(pair["type"]))
                for pair in joint.get("kind_by_mode", ())
            )
            if joint is not None
            else ()
        )
        axis_reference = (
            str(joint["axis_reference"]) if joint is not None and joint.get("axis_reference") else ""
        )
        first_body: Literal["owner", "far"] = "owner"
        far_owner = ""
        far_label = ""
        if joint is not None:
            body_a = str(joint["body_a"])
            body_b = str(joint["body_b"])
            # ``first_body`` says which end the joint records as ``body_a``.  A
            # joint that names this point's body *second* records the far end
            # first, and vice versa; deriving it from the order the file wrote
            # rather than from a guess is what makes export and import inverse to
            # each other, so a template survives a trip through the file format
            # unchanged.
            if body_b == owner and body_a != owner:
                first_body = "owner"
                far_owner = body_a
            elif body_a == owner and body_b != owner:
                first_body = "far"
                far_owner = body_b
            # The far end carries a *label*, not a body name: the far body holds one
            # point per connection, and two mounts whose far labels collided would
            # overwrite each other's point.  A joint that states the far label is
            # taken at its word -- that is how a template keeps the label a frozen
            # contract already records -- and otherwise the point's own name is used.
            far_label = (
                str(joint["far_label"])
                if joint.get("far_label")
                else (f"{point}_{_MODEL_SIDE}" if far_owner else "")
            )
        connections.append(
            ConnectionDefinition(
                # The point's *name* is what the contract records for the constraint
                # or bushing it carries, so it comes from the declaration that owns
                # it; only a point that constrains nothing falls back to its own
                # role-and-side name.  The *role* is separate and deliberately the
                # same on both sides.
                name=_connection_name(point, joint, bushing_by_point),
                role=point,
                joint=joint_type,
                bushing=bushing_by_point.get(point),
                owner=owner,
                label=label,
                joint_modes=joint_modes,
                bushing_modes=bushing_modes_by_point.get(point, _BOTH_MODES),
                joint_kind_by_mode=joint_kind_by_mode,
                far_owner=far_owner,
                far_label=far_label,
                first_body=first_body,
                axis_reference_role=axis_reference,
            )
        )
    return tuple(connections)


def _mirrored_parts(payload: Mapping[str, Any]) -> tuple[PartDefinition, ...]:
    """
    Build the template's parts, mirrored like its connections.

    A part named for the left side gets a right-side twin with the same mass, so
    the mirroring of connections lands on bodies that exist.  An unsided part --
    the chassis, the rack -- is declared once, because it exists once.
    """
    declared = [
        PartDefinition(
            name=str(row["name"]),
            mass=float(row.get("mass", 0.0)),
            fixed=bool(row.get("fixed", False)),
        )
        for row in payload["bodies"]
    ]
    # Two passes, not one: every declared part is listed before its right-side twin.
    # The sequence is a fact about the template -- it is the order the document
    # records its bodies in -- so interleaving the twins would produce a different
    # document from the same file even though both carry the same bodies.
    mirrored = [
        replace(part, name=_side_name(part.name, "R"))
        for part in declared
        if part.name.endswith("_L")
    ]
    return tuple(declared + mirrored)


def _mirrored_connections(
    connections: tuple[ConnectionDefinition, ...],
) -> tuple[ConnectionDefinition, ...]:
    """
    Expand the connections a file declares on one side into both sides.

    A connection whose owner is unsided (the chassis, the rack, a whole-axle part)
    describes one point that exists once and is stated once.  A connection whose
    owner carries ``_L`` is mirrored into an ``_R`` twin with the same role, label
    and columns, and its far end mirrored the same way; a far end that is itself
    unsided stays shared, which is what keeps a rack guide one guide rather than
    two.
    """
    mirrored: list[ConnectionDefinition] = []
    for connection in connections:
        mirrored.append(connection)
        if not connection.owner.endswith("_L"):
            continue
        mirrored.append(
            replace(
                connection,
                name=_mirrored_name(connection.name, "R"),
                owner=_side_name(connection.owner, "R"),
                far_owner=_side_name(connection.far_owner, "R")
                if connection.far_owner
                else "",
                far_label=_mirrored_name(connection.far_label, "R")
                if connection.far_label
                else "",
                bushing=_mirrored_name(connection.bushing, "R")
                if connection.bushing
                else None,
            )
        )
    return tuple(mirrored)


#: A side token inside a generated name: ``_L`` or ``_R`` bounded by a ``_`` or the
#: end of the name.  Rewriting the token rather than the suffix is what lets a name
#: like ``uca_mount_L_inner_front`` mirror to ``uca_mount_R_inner_front`` -- its side
#: is in the middle, and a suffix-only rule would leave both sides named alike.
_SIDE_TOKEN = re.compile(r"_(L|R)(?=_|$)")


def _side_name(name: str, side: str) -> str:
    """
    Rewrite one *body* name's side token, leaving an unsided name alone.

    A body that exists once -- the chassis, the rack -- must stay shared when a
    connection is mirrored, so this function never invents a side for it: a
    ``chassis_R`` is a body no contribution produces.
    """
    return _SIDE_TOKEN.sub(f"_{side}", name, count=1)


def _mirrored_name(name: str, side: str) -> str:
    """
    Return a per-side name for something that exists once per side.

    A connection and the label a far body records both have to be distinct on each
    side, so a name that carries no side token gets one appended rather than being
    left to collide with its twin.  Bodies use ``_side_name`` instead, because a
    body's name is allowed to be shared.
    """
    rewritten = _SIDE_TOKEN.sub(f"_{side}", name, count=1)
    return rewritten if rewritten != name else f"{name}_{side}"


def _connection_name(
    point: str, joint: Mapping[str, Any] | None, bushing_by_point: Mapping[str, str]
) -> str:
    """Return the name a connection records, taken from the declaration that owns it."""
    if joint is not None:
        return str(joint["name"])
    bushing = bushing_by_point.get(point)
    if bushing is not None:
        return bushing
    return f"{point}_{_MODEL_SIDE}"


def _slots_from(
    payload: Mapping[str, Any], connections: tuple[ConnectionDefinition, ...]
) -> tuple[PropertySlot, ...]:
    """
    Build the runtime property slots from the file's declared slots.

    A slot keeps its ``connections`` list because that is how the runtime finds
    the slot feeding a bushing column without matching names; the file states it
    for the same reason.  Every slot the *role* requires and the file does not
    declare is added with the built-in's own zero, because the runtime role
    contract is checked on the converted template: a file template whose spring
    stiffness lives in the model -- as the built-in's does -- is a complete
    template, not an invalid one.
    """
    slots: list[PropertySlot] = [
        PropertySlot(
            name=str(row["name"]),
            unit=str(row.get("unit", "-")),
            default=None if row.get("default") is None else float(row["default"]),
            connections=tuple(str(name) for name in row.get("connections", ())),
        )
        for row in payload["property_slots"]
    ]
    declared = {slot.name for slot in slots}
    role = str(payload["functional_role"])
    for name in _role_required_slots(role):
        if name in declared:
            continue
        column = _COLUMN_OF.get(name)
        connection_names = (
            tuple(
                connection.name
                for connection in connections
                if getattr(connection, column, None) is not None
            )
            if column
            else ()
        )
        slots.append(
            PropertySlot(
                name=name,
                unit=_SLOT_UNITS.get(name, "-"),
                default=0.0,
                connections=connection_names,
            )
        )
    return tuple(slots)


def _role_required_slots(role: str) -> tuple[str, ...]:
    """Return the slots a role requires, or none for a role with no declaration."""
    from ..templates.roles import ROLES

    spec = ROLES.get(role)
    return () if spec is None else tuple(spec.required_slots)


def front_axle_model_for(
    subsystem: SubsystemDocument,
    *,
    name: str | None = None,
    sprung_mass: float = 600.0,
) -> FrontAxleModel:
    """Build the solver's model from one file subsystem, overrides applied."""
    effective = subsystem.effective()
    return bridge_model(
        effective, name=name or f"{subsystem.payload['name']}_axle", sprung_mass=sprung_mass
    )


def bridge_model(
    effective: EffectiveSubsystem,
    *,
    name: str,
    sprung_mass: float = 600.0,
) -> FrontAxleModel:
    """Bridge one effective subsystem to a model, naming the file on failure."""
    return front_axle_model_from(effective, name=name, sprung_mass=sprung_mass)


def assembly_request_from(
    subsystem: SubsystemDocument,
    *,
    mode: Literal["K", "C"] = "K",
    subsystems: frozenset[str] | None = None,
) -> AssemblyRequest:
    """
    Build the assembly request the composition entry accepts.

    The suspension template is the file's, instantiated with the *resolved*
    constitutive values of its property bindings, so the stiffness the C solve
    reads comes from the property file rather than from a number written beside
    the template.  The other subsystem roles keep their existing implementations:
    the plan's subject is the suspension template, and claiming to have migrated
    steering or the wheel as well would be a claim this code does not support.
    """
    template = runtime_template_from(subsystem.template)
    effective = subsystem.effective()
    properties: dict[str, float] = {}
    for slot_name, document in effective.property_bindings.items():
        scalar = _scalar_of(document.resolved)
        if scalar is not None:
            properties[slot_name] = scalar
    instance: SubsystemInstance = instantiate(
        template, mode=mode, properties=properties
    )
    return AssemblyRequest(
        mode=mode,
        subsystems=subsystems or AssemblyRequest().subsystems,
        suspension_template=instance,
    )


def _scalar_of(resolved: Mapping[str, Any]) -> float | None:
    """Return the single scalar a runtime slot reads from a resolved law."""
    element_type = str(resolved.get("element_type", ""))
    key = "viscous_damping" if element_type == "damper" else "stiffness"
    value = resolved.get(key)
    return None if value is None else float(value)


def template_document_from(template: Template) -> dict[str, Any]:
    """
    Export a runtime template to the file format.

    This is the other direction of the conversion, and it is what makes the file
    format a *faithful* spelling of a template rather than a subset of one: the
    built-in double wishbone can be written out, read back, and assembled into the
    same model, which the round-trip test asserts rather than assumes.

    Only one side is written, because that is what the format declares and what the
    conversion mirrors; the left side is the one exported, matching the side the
    model describes.  Per-mode joint kinds, per-mode column activation and slot
    defaults are all carried, because a template that has them and an export that
    dropped them would produce a different model from the same file.
    """
    parts = [part for part in template.parts if not part.name.endswith("_R")]
    connections = [
        connection
        for connection in template.connections
        if not connection.owner.endswith("_R")
    ]
    hardpoints = [
        {
            "name": connection.role,
            "owner": connection.owner,
            "label": connection.label or connection.role,
        }
        for connection in connections
    ]
    joints: list[dict[str, Any]] = []
    elements: list[dict[str, Any]] = []
    for connection in connections:
        if connection.joint is not None:
            body_a, body_b = _joint_bodies(connection)
            joint: dict[str, Any] = {
                "name": connection.name,
                "type": connection.joint,
                "body_a": body_a,
                "body_b": body_b,
                "point_a": connection.role,
            }
            if connection.axis_reference_role:
                joint["axis_reference"] = connection.axis_reference_role
            if connection.far_label:
                joint["far_label"] = connection.far_label
            if tuple(connection.joint_modes) != ("K", "C"):
                joint["modes"] = list(connection.joint_modes)
            if connection.joint_kind_by_mode:
                joint["kind_by_mode"] = [
                    {"mode": mode, "type": kind}
                    for mode, kind in connection.joint_kind_by_mode
                ]
            joints.append(joint)
        if connection.bushing is not None:
            body_a, body_b = _joint_bodies(connection)
            element: dict[str, Any] = {
                "name": connection.bushing,
                "type": "bushing",
                "body_a": body_a,
                "body_b": body_b,
                "point_a": connection.role,
                "property_slot": _MOUNT_SLOT,
            }
            if tuple(connection.bushing_modes) != ("K", "C"):
                element["modes"] = list(connection.bushing_modes)
            elements.append(element)
    slots = [
        {
            "name": slot.name,
            "element_type": _slot_element_type(slot.name),
            "unit": slot.unit,
            # "required" means the *role* needs a value from the properties file, not
            # merely that this slot carries no default: the model-owned slots
            # (masses, inertias, tire data) are filled by the model, and marking them
            # required would demand a property file for geometry.
            "required": slot.name in template.role_spec.required_slots,
            **({"default": float(slot.default)} if slot.default is not None else {}),
            **({"connections": list(slot.connections)} if slot.connections else {}),
        }
        for slot in template.property_slots
    ]
    document: dict[str, Any] = {
        "document": "template",
        "schema_version": 1,
        "name": template.name,
        "functional_role": template.role,
        "allowed_placement_roles": ["any", "front", "rear"],
        "bodies": [
            {"name": part.name, "mass": part.mass, "fixed": part.fixed}
            for part in parts
        ],
        "hardpoints": hardpoints,
        "joints": joints,
        "elements": elements,
        "property_slots": slots,
    }
    if template.outputs:
        document["outputs"] = [
            {"name": output.name, "unit": output.unit, "source": output.source}
            for output in template.outputs
        ]
    if template.ports:
        document["ports"] = [
            {
                "name": port.name,
                "role": port.role,
                **_owner_of(port, parts),
            }
            for port in template.ports
        ]
    if template.suspension_kind:
        document["suspension_kind"] = template.suspension_kind
    if template.description:
        document["description"] = template.description
    return document


def _joint_bodies(connection: ConnectionDefinition) -> tuple[str, str]:
    """
    Return the ``(body_a, body_b)`` a file joint must state for ``connection``.

    The runtime builds a mount with the *far* body first (``chassis`` then the arm),
    so the export states the same order and the conversion reproduces it.  A
    one-ended point has no far body and names its own body twice, which is what
    "these two bodies share this point" means for a joint that reaches only itself.
    """
    own = connection.owner
    far = connection.far_owner or own
    if connection.first_body == "far":
        return own, far
    return far, own


def _slot_element_type(name: str) -> str:
    """Return the element type a slot of this name declares."""
    return name if name in {"spring", "damper", "bump_stop", "bushing", "tire"} else "generic"


def _owner_of(port: Any, parts: list[PartDefinition]) -> dict[str, Any]:
    """Return the ``owner`` a port declaration carries, when it names a real part."""
    owner = str(getattr(port, "owner", ""))
    if owner and owner in {part.name for part in parts}:
        return {"owner": owner}
    return {}
