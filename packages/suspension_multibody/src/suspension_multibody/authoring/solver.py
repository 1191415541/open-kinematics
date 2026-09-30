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

from dataclasses import replace
from typing import Any, Literal, Mapping

from ..schema import FrontAxleModel, VehicleModel
from ..subsystems.types import AssemblyRequest
from ..templates.instantiate import (
    SubsystemInstance,
    instantiate,
    slot_element_type,
)
from ..templates.model import (
    ConnectionDefinition,
    OutputDeclaration,
    PartDefinition,
    PropertySlot,
    SlotValue,
    Template,
    TemplateError,
)
from .bridge import (
    _SIDE_TOKEN,
    BridgeError,
    _mirrored_parts,
    _side_name,
    front_axle_model_from,
)
from .documents import (
    AssemblyDocument,
    EffectiveSubsystem,
    SimulationAssembly,
    SubsystemDocument,
    TemplateDocument,
)
from .errors import AuthoringError
from .properties import ElementPropertyDocument

__all__ = [
    "assembly_request_for",
    "assembly_request_from",
    "file_axles_from",
    "front_axle_model_for",
    "runtime_template_from",
    "template_document_from",
    "vehicle_model_with_file_axles",
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
    parts = _mirrored_parts(payload, mirror=document.mirrors)
    connections = _mirrored_connections(
        _connections_from(payload), mirror=document.mirrors
    )
    if not document.mirrors:
        connections = _per_side_roles(connections)
    slots = _slots_from(payload, connections, _slot_connections(payload, connections))
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
        # The side this point is *on*, taken from its owner rather than assumed:
        # a file that writes both sides declares right-hand points, and naming
        # them after the left side would give one connection two names.
        point_side = "R" if str(row.get("owner", "")).endswith("_R") else _MODEL_SIDE
        # A mount with no owner is one that sits on a body another role declares;
        # the file states it without an owner rather than with an empty one.
        owner = str(row.get("owner", ""))
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
                else (f"{point}_{point_side}" if far_owner else "")
            )
        connections.append(
            ConnectionDefinition(
                # The point's *name* is what the contract records for the constraint
                # or bushing it carries, so it comes from the declaration that owns
                # it; only a point that constrains nothing falls back to its own
                # role-and-side name.  The *role* is separate and deliberately the
                # same on both sides.
                name=_connection_name(point, joint, bushing_by_point, side=point_side),
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



def _mirrored_connections(
    connections: tuple[ConnectionDefinition, ...], *, mirror: bool = True
) -> tuple[ConnectionDefinition, ...]:
    """
    Expand the connections a file declares on one side into both sides.

    A connection whose owner is unsided (the chassis, the rack, a whole-axle part)
    describes one point that exists once and is stated once.  A connection whose
    owner carries ``_L`` is mirrored into an ``_R`` twin with the same role, label
    and columns, and its far end mirrored the same way; a far end that is itself
    unsided stays shared, which is what keeps a rack guide one guide rather than
    two.

    ``mirror`` is the template's own declaration, and a file that writes both sides
    gets its connections back unchanged: the right-hand ones are already there, and
    a twin of a right-hand connection would name a body nobody declared.
    """
    if not mirror:
        return tuple(connections)
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


def _per_side_roles(
    connections: tuple[ConnectionDefinition, ...],
) -> tuple[ConnectionDefinition, ...]:
    """
    Take the side token out of a both-sides file's *role* names.

    A role is side-independent by design: ``upper_outer`` is the same role on both
    sides, and the composition resolves it against that side's hardpoints.  A file
    that writes both sides therefore has to name its right-hand declarations
    somehow -- and the convention is the model's own, ``<role>_R``, which is what
    ``geometry.side_hardpoints`` already reads on the model's side of the same
    question.

    Removing the token here is what makes the two ways of writing one file land on
    the *same* connection: the mirrored route's twin keeps the role and gains a
    side token on its name, and this route's declaration states the token and gets
    it taken off the role.  Without this, a both-sides file would compose a right
    side whose roles nobody can look up.

    Only a connection whose owner carries that same token is rewritten, so a role
    that merely ends in ``_L`` on an unsided body is left alone.
    """
    # A role another connection on the same body already carries: stripping a
    # token must not merge two points into one, so those names are left as written.
    taken = {
        (connection.owner, connection.role)
        for connection in connections
        if not connection.role.endswith(("_L", "_R"))
    }
    rewritten: list[ConnectionDefinition] = []
    for connection in connections:
        token = next(
            (side for side in ("_L", "_R") if connection.owner.endswith(side)), None
        )
        if token is None:
            rewritten.append(connection)
            continue
        role = (
            connection.role[: -len(token)]
            if connection.role.endswith(token)
            else connection.role
        )
        if role != connection.role and (connection.owner, role) in taken:
            role = connection.role
        rewritten.append(
            replace(
                connection,
                role=role,
                # The label carries the same token for the same reason: it names
                # a point that exists once per side.  Only a label ending in the
                # owner's own side token is touched, and a label that stops doing
                # so keeps its spelling.
                label=connection.label[: -len(token)]
                if connection.label.endswith(token)
                else connection.label,
            )
        )
    return tuple(rewritten)


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
    point: str,
    joint: Mapping[str, Any] | None,
    bushing_by_point: Mapping[str, str],
    *,
    side: str = _MODEL_SIDE,
) -> str:
    """
    Return the name a connection records, taken from the declaration that owns it.

    A point that carries no joint of its own and feeds no bushing still needs a
    name the contract can record, and the name is per-side because the point is:
    the fallback takes the side the point is on, so a both-sides file does not name
    its right-hand points after the left one.
    """
    if joint is not None:
        return str(joint["name"])
    bushing = bushing_by_point.get(point)
    if bushing is not None:
        return bushing
    # The point's own name may already spell its side (a both-sides file writes it
    # that way); the token is taken off and the point's *side* appended, so the two
    # routes land on one name: `upper_rear` + R and `upper_rear_R` + R are the same
    # connection.
    base = point[:-2] if point.endswith(("_L", "_R")) else point
    return f"{base}_{side}"


def _slot_connections(
    payload: Mapping[str, Any], connections: tuple[ConnectionDefinition, ...]
) -> dict[str, tuple[str, ...]]:
    """
    Return, per slot, the connections whose bushing column an element feeds.

    The file already says this twice over -- an element names its ``property_slot``
    and the connection it becomes carries that element's ``bushing`` name -- so the
    slot's own ``connections`` list is *derived* rather than asked for.  Requiring
    an author to repeat it would be asking for a name that must match a generated
    one (``mount_upper_front_R``), and getting it subtly wrong fails far away, as a
    bushing column no slot feeds.
    """
    slot_of_element = {
        str(row["name"]): str(row["property_slot"])
        for row in payload["elements"]
        if str(row["type"]) == "bushing"
    }
    derived: dict[str, list[str]] = {}
    for connection in connections:
        slot = slot_of_element.get(connection.bushing or "")
        if slot is not None:
            derived.setdefault(slot, []).append(connection.name)
    return {name: tuple(names) for name, names in derived.items()}


def _slots_from(
    payload: Mapping[str, Any],
    connections: tuple[ConnectionDefinition, ...],
    derived: Mapping[str, tuple[str, ...]],
) -> tuple[PropertySlot, ...]:
    """
    Build the runtime property slots from the file's declared slots.

    A slot keeps its ``connections`` list because that is how the runtime finds the
    slot feeding a bushing column without matching names; the file may state it, and
    the connections its elements imply are added when it does not.  Every slot the
    *role* requires and the file does not declare is added with the built-in's own
    zero, because the runtime role contract is checked on the converted template: a
    file template whose spring stiffness lives in the model -- as the built-in's
    does -- is a complete template, not an invalid one.
    """
    slots: list[PropertySlot] = [
        PropertySlot(
            name=str(row["name"]),
            unit=str(row.get("unit", "-")),
            default=None if row.get("default") is None else float(row["default"]),
            connections=tuple(
                dict.fromkeys(
                    [str(name) for name in row.get("connections", ())]
                    + list(derived.get(str(row["name"]), ()))
                )
            ),
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
    overrides: Mapping[str, Any] | None = None,
) -> FrontAxleModel:
    """
    Build the solver's model from one file subsystem, overrides applied.

    ``overrides`` are an assembly entry's own coverings, passed in rather than read
    here so that the same function serves a standalone subsystem and one an
    assembly has already covered.
    """
    effective = subsystem.effective(overrides=overrides)
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


def role_instance_from(
    subsystem: SubsystemDocument,
    *,
    mode: Literal["K", "C"] = "K",
    overrides: Mapping[str, Any] | None = None,
) -> SubsystemInstance:
    """
    Build one role's runtime instance from one file subsystem.

    This is the conversion every role shares: the template is the *file's*, and it
    is instantiated with the resolved constitutive values of that subsystem's
    property bindings, so whichever role a file describes reaches the composition
    through the same path.  A file that names no property file still gets an
    instance, because the role's own slots carry their defaults.

    ``overrides`` are the assembly document's own copy-on-write coverings, passed
    in rather than read here so that the subsystem file on disk stays the one the
    assembly pointed at.
    """
    template = runtime_template_from(subsystem.template)
    effective = subsystem.effective(overrides=overrides)
    # The resolved law, not a stripped scalar: what the property file declared
    # about the element -- its type, its model, its curve and where it came from --
    # travels with the value, so a consumer that needs the whole law does not have
    # to read the file a second time.
    properties: dict[str, SlotValue] = {
        slot_name: slot_value_of(document)
        for slot_name, document in effective.property_bindings.items()
    }
    return instantiate(template, mode=mode, properties=properties)


def assembly_request_from(
    subsystem: SubsystemDocument,
    *,
    mode: Literal["K", "C"] = "K",
    subsystems: frozenset[str] | None = None,
    overrides: Mapping[str, Any] | None = None,
) -> AssemblyRequest:
    """
    Build the assembly request one *suspension* file subsystem describes.

    The suspension template is the file's, instantiated with the resolved
    constitutive values of its property bindings, so the stiffness the C solve
    reads comes from the property file rather than from a number written beside
    the template.  The other roles a *document* describes -- steering, wheel,
    chassis -- travel through `assembly_request_for`, which is the entry that has
    the whole document in hand.
    """
    instance = role_instance_from(subsystem, mode=mode, overrides=overrides)
    return AssemblyRequest(
        mode=mode,
        subsystems=subsystems or AssemblyRequest().subsystems,
        suspension_template=instance,
    )


def _sides_from_file(assembly: AssemblyDocument) -> tuple[str, ...] | None:
    """
    Return the sides an assembly document declares, or `None` if it declares none.

    A template says which sides it writes and whether the other is mirrored from
    it, so the assembly's sides are the union over the entries that *say*
    something.  Only an explicit `sides` counts as saying something, and that is
    load-bearing rather than tidy: a side-less role's template -- the chassis, a
    simplified brake -- has no sides to declare, so letting it vote would widen a
    one-corner document back to a pair and make the declaration unable to express
    what it exists for.  `None` means no entry said anything, which is the
    ordinary case, and the request's own default is what it means.
    """
    side_of = {"left": "L", "right": "R"}
    declared: list[str] = []
    for entry in assembly.entries:
        template = getattr(entry.subsystem, "template", None)
        payload = getattr(template, "payload", None) or {}
        if template is None or not ({"sides", "mirror"} & set(payload)):
            continue
        if getattr(template, "mirrors", True):
            sides = ("L", "R")
        else:
            sides = tuple(
                side_of[str(name)] for name in getattr(template, "declared_sides", ("left",))
            )
        for side in sides:
            if side not in declared:
                declared.append(side)
    return tuple(declared) or None


def assembly_request_for(
    document: AssemblyDocument | SimulationAssembly,
    *,
    mode: Literal["K", "C"] = "K",
) -> AssemblyRequest:
    """
    Build the composition request from an *assembly document*.

    What the file decides is what the composition carries.  The roles come from the
    document's own assignments rather than from the single-axle default, so an
    assembly file that declares a steering subsystem is composed with steering and
    one that does not is composed without it; the counts and placements were
    already judged against ``connections.policy`` when the document loaded.

    Each *subsystem the document places* also supplies its role's template, through
    the one conversion `role_instance_from` performs: an authored steering, wheel or
    chassis subsystem is the one that gets built, with that entry's overrides
    applied.  A role the document does not place keeps its registered built-in.

    The suspension template is supplied only for an *axle* document.  A vehicle
    document places two suspensions and neither is "the" suspension, so the two
    axles come from `file_axles_from` and the request only names the role set --
    which is also why the vehicle's wheels, steering ratio and driveline stay
    `VehicleModel` data unless the document's own `vehicle` section states them.

    Which roles an assembly may carry stays the policy's decision rather than this
    function's: a vehicle document composed as an axle is translated like any other
    and then refused by ``check_root`` for the roles an axle must not carry.  That
    keeps one home for the rule instead of a second copy here.
    """
    assembly = document.assembly if isinstance(document, SimulationAssembly) else document
    roles = frozenset(entry.functional_role for entry in assembly.entries)
    # `check_assembly_shape` requires exactly one suspension in an axle document,
    # so the entry is taken rather than searched with a fallback that could never
    # run: an assembly with two suspensions is refused while it is being loaded.
    suspensions = [
        entry for entry in assembly.entries if entry.functional_role == "suspension"
    ]
    templates: dict[str, object] = {}
    if assembly.assembly_kind == "suspension_axle":
        templates["suspension_template"] = role_instance_from(
            suspensions[0].subsystem, mode=mode, overrides=suspensions[0].overrides
        )
    for entry in assembly.entries:
        role = entry.functional_role
        if role in _FILE_ROLE_TEMPLATES:
            templates[f"{role}_template"] = role_instance_from(
                entry.subsystem, mode=mode, overrides=entry.overrides
            )
    sides = _sides_from_file(assembly)
    if sides is None:
        return AssemblyRequest(mode=mode, subsystems=roles, **templates)
    return AssemblyRequest(mode=mode, subsystems=roles, sides=sides, **templates)


#: The roles a document may describe whose template reaches the composition
#: directly.  The suspension is not here because an axle document supplies it as
#: `suspension_template` and a vehicle document supplies two of them, which one
#: field cannot hold; the steering, chassis and wheel roles have one entry each.
#: The wheel role used to be absent because its wheel centre is a per-side mount
#: on a body the wheel template did not own -- which stopped being true when the
#: wheel subsystem became the wheel end's producer: a file's wheel subsystem
#: describes its own wheel body, so the mount has somewhere to hang and the
#: template belongs on the request like any other role's.
_FILE_ROLE_TEMPLATES: tuple[str, ...] = ("steering", "chassis", "wheel")


def vehicle_model_with_file_axles(
    template: VehicleModel,
    document: AssemblyDocument | SimulationAssembly,
    *,
    name: str | None = None,
) -> VehicleModel:
    """
    Return a vehicle model whose two axles come from an assembly document.

    This is the compatibility conversion for a caller that already has a
    `VehicleModel` and wants the document's axles in it: the wheels, the steering
    ratio and the driveline stay the template model's own, while the two axles --
    down to every hardpoint and every constitutive law -- are built from the
    document's suspension entries, overrides applied.

    A vehicle whose *whole* description is a file does not need a template at all;
    `authoring.vehicle.vehicle_model_from` reads the document's own `vehicle`
    section for the vehicle-level numbers.  Both paths take their axles from
    `file_axles_from`, so the two cannot disagree about what a file's axle is.
    """
    assembly = document.assembly if isinstance(document, SimulationAssembly) else document
    axles = file_axles_from(assembly)
    return template.model_copy(
        update={
            "name": name or template.name,
            "front_axle": axles["front"],
            "rear_axle": axles["rear"],
        }
    )


def file_axles_from(
    document: AssemblyDocument | SimulationAssembly,
) -> dict[str, FrontAxleModel]:
    """
    Build one model per suspension the document places, keyed by its placement.

    This is the half of the vehicle conversion that both directions share: a
    vehicle model's two axles are subsystems, so whether the vehicle-level numbers
    come from a template or from the document's own `vehicle` section, the axles
    come from here -- down to every hardpoint and every constitutive law, with the
    entry's overrides applied.

    A document that does not place one suspension at `front` and one at `rear` is
    refused, naming the placements it does have: the alternative is a vehicle
    silently carrying one axle twice or once.
    """
    assembly = document.assembly if isinstance(document, SimulationAssembly) else document
    axles: dict[str, FrontAxleModel] = {}
    for entry in assembly.entries:
        if entry.functional_role != "suspension":
            continue
        axles[entry.placement_role] = front_axle_model_for(
            entry.subsystem,
            name=f"{entry.placement_role}_{assembly.name}",
            overrides=entry.overrides,
        )
    missing = sorted({"front", "rear"} - set(axles))
    if missing:
        raise AuthoringError(
            f"{assembly.path}: a vehicle model needs a suspension at {missing}; "
            f"this document places them at {sorted(axles)}"
        )
    return axles

def slot_value_of(document: ElementPropertyDocument) -> SlotValue:
    """
    Return the resolved constitutive value one property file produces.

    The scalar is the one the kernel's parameter block reads -- a stiffness for
    everything but a damper, whose number is its viscous damping -- and the curve
    and the table are carried beside it rather than instead of it: the resolver
    already derived a slope for the kernel's fallback from the curve's first
    interval, so a consumer that only wants a number still gets one, and one that
    has to build a six-axis mount gets the table.  For a table, the scalar view is
    its first entry, which is the translational diagonal a diagonal table states --
    meaningful for exactly the tables where the two agree.
    """
    resolved = document.resolved
    element_type = str(resolved.get("element_type", "generic"))
    key = "viscous_damping" if element_type == "damper" else "stiffness"
    value = resolved.get(key)
    table = value if isinstance(value, tuple) else ()
    if table:
        scalar = float(table[0][0])
    elif value is None:
        scalar = 0.0
    else:
        scalar = float(value)
    return SlotValue(
        element_type=element_type,
        model=str(resolved.get("model", "linear")),
        scalar=scalar,
        curve=tuple(
            (float(independent), float(dependent))
            for independent, dependent in resolved.get("force_curve", ())
        ),
        matrix=table,
        source=str(resolved.get("source", document.path)),
    )


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
    # One hardpoint per *mount role*, not one per connection.  A connection exists
    # per side while the file states a mount once -- that is what the format
    # declares and what the conversion mirrors -- and a role that owns no bodies
    # (the simplified brake and drive) spells its two mounts once each on both
    # sides, so writing them per connection would repeat the name and produce a
    # document the loader has to refuse.
    declared: dict[str, dict[str, Any]] = {}
    for connection in connections:
        row: dict[str, Any] = {
            "name": connection.role,
            "label": connection.label or connection.role,
        }
        # An ownerless mount is stated without an owner rather than with an empty
        # one: the point sits on whichever body carries it, which is the other
        # role's business.
        if connection.owner:
            row["owner"] = connection.owner
        declared.setdefault(connection.role, row)
    hardpoints = list(declared.values())
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
            "element_type": slot_element_type(slot.name),
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


def _owner_of(port: Any, parts: list[PartDefinition]) -> dict[str, Any]:
    """Return the ``owner`` a port declaration carries, when it names a real part."""
    owner = str(getattr(port, "owner", ""))
    if owner and owner in {part.name for part in parts}:
        return {"owner": owner}
    return {}
