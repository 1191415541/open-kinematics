"""
The left/right suspension subsystem: the arms, the uprights and their joints.

This is the largest of the six and the one whose emission order matters most: the
contract document lists bodies and constraints in the order the assembly appends
them, so the hooks below are separated by *what is appended when*, and the
assembly drives them in the original sequence:

1. `side_bodies`   -- upper/lower arm and upright, before any point is resolved;
2. `side_content`  -- the mount-table points, then the inboard joints, the
                      outboard ball joints and their connection rows;
3. `elements`      -- springs, dampers, bump stops, per side;
4. `global_elements` -- the anti-roll bar, which spans both sides, so it can only
                      belong to this subsystem and can only be emitted once;
5. `compliance_elements` -- the user's `model.bushings` in C mode;
6. `placeholder_bushings` -- the C-mode placeholders at the arm inboard points.

The K/C split lives inside `side_content`: K builds a revolute joint at each arm's
inboard front point (and nothing at the rear point, because the front joint's axis
already passes through it), C builds a ball joint at all four inboard points plus a
placeholder bushing.  K/C state here is the *assembly's* declaration, so the
semantics stay exactly where they were.
"""

from __future__ import annotations

import numpy as np

from ..modeling.primitives.joints import (
    BallJoint,
    Constraint,
    CylindricalJoint,
    PrismaticJoint,
    RevoluteJoint,
    RigidBody,
    UniversalJoint,
    WeldJoint,
)
from ..modeling.primitives.spatial import SE3
from ..templates.model import ConnectionDefinition, Template
from .geometry import body_from_spec, body_without_spec, resolve_body
from .types import (
    Connection,
    ResolvedElement,
    Side,
    SubsystemContext,
    SubsystemOutput,
)

__all__ = [
    "compliance_elements",
    "elements",
    "global_elements",
    "placeholder_bushings",
    "role",
    "side_bodies",
    "side_body_order",
    "side_content",
]

#: The role this subsystem implements.
role = "suspension"

#: The arm stems, in the order the recorded document lists them.
#:
#: Used only when a template names no arms: the derived tables below read the
#: template, and this is the fallback for a template that declares no inboard
#: mounts at all.
_ARM_STEMS: tuple[str, ...] = ("upper_arm", "lower_arm")

#: The two inboard labels, in the order the recorded document lists them.
_INBOARD_LABELS: tuple[str, ...] = ("inner_front", "inner_rear")


def _suspension_template(context: SubsystemContext) -> Template:
    """
    Return the suspension template this assembly was asked for.

    The template is the source of the topology: which mounts exist, where each
    one attaches, and which column each activates in each mode.  Reading it here
    is what makes "choose a template, get that subsystem" true, rather than the
    template being a decoration beside a hard-coded build.
    """
    request = getattr(context, "request", None)
    if request is None:
        from ..templates import DOUBLE_WISHBONE

        return DOUBLE_WISHBONE
    return request.instantiated_suspension.template


def _point_rows(context: SubsystemContext, side: Side) -> list[tuple[str, str, str]]:
    """
    Return ``(body stem, label, hardpoint role)`` for every point on one side.

    The rows come from the template's own connections, so a template that adds an
    arm point gets it resolved without this module being edited.  Connections
    whose owner is the chassis or the rack are skipped: those points are placed by
    whoever owns the far end, which is the arm for a mount and the rack for its
    guide.

    The side filter is not optional: one template declares both sides, and this
    function is called once per side, so without it every left row would also be
    produced while building the right one.
    """
    rows: list[tuple[str, str, str]] = []
    for connection in _suspension_template(context).connections:
        owner = connection.owner
        if not owner.endswith(f"_{side}"):
            continue
        stem = owner.rsplit("_", 1)[0]
        label = connection.label or connection.name
        rows.append((stem, label, connection.role))
    return rows


def _inboard_rows(
    context: SubsystemContext, side: Side
) -> list[tuple[str, str, str, ConnectionDefinition]]:
    """
    Return one side's arm inboard mounts, in the recorded order.

    ``(arm stem, label, hardpoint role, connection)`` per row.  Order is the
    document's: arm by arm, and within an arm the front point before the rear one,
    because that is the sequence the constraint rows are recorded in.
    """
    template = _suspension_template(context)
    arms: list[str] = []
    for connection in template.connections:
        owner = connection.owner
        if not owner.endswith(f"_{side}") or not owner.startswith(
            ("upper_arm_", "lower_arm_")
        ):
            continue
        stem = owner.rsplit("_", 1)[0]
        if stem not in arms:
            arms.append(stem)
    rows: list[tuple[str, str, str, ConnectionDefinition]] = []
    for arm in arms or list(_ARM_STEMS):
        for label in _INBOARD_LABELS:
            for connection in template.connections:
                if connection.owner != f"{arm}_{side}":
                    continue
                if (connection.label or "") != label:
                    continue
                rows.append((arm, label, connection.role, connection))
                break
    return rows


def _outer_rows(
    context: SubsystemContext, side: Side
) -> list[tuple[str, str, str, ConnectionDefinition]]:
    """Return one side's arm outer joints, in the recorded order."""
    template = _suspension_template(context)
    rows: list[tuple[str, str, str, ConnectionDefinition]] = []
    for connection in template.connections:
        owner = connection.owner
        if not owner.endswith(f"_{side}") or not owner.startswith(
            ("upper_arm_", "lower_arm_")
        ):
            continue
        if (connection.label or "") != "outer":
            continue
        rows.append((owner.rsplit("_", 1)[0], "outer", connection.role, connection))
    return rows


def side_bodies(context: SubsystemContext, side: Side) -> dict[str, RigidBody]:
    """
    Declare the bodies this template puts on one side.

    The set comes from the **template's parts**, in the template's own order.  It
    used to name three stems unconditionally (`upper_arm`, `lower_arm`, `upright`),
    which meant a template declaring no upper arm still got one: the template drove
    the connections while the bodies were decided for it, so "choose a template"
    could not change the model's entities -- the first thing the flow promises.

    The built-in template declares its parts in exactly the recorded document order
    (`chassis, rack, arm, arm, upright, tie rod` per side), which is what makes this
    derivation reproduce every existing document unchanged; a template that declares
    a different set produces a correspondingly different model, which is the point.

    Only this side's parts are built, and only the ones this subsystem *owns*: the
    template lists every part of the assembly, so a tie rod or a rack appears there
    too, and building those here would make two contributions declare one body --
    refused by name as `duplicate bodies between fragments`, and rightly.  Ownership
    is stated once in :data:`_FOREIGN_STEMS`.  A declared part with no spec gets
    default mass properties, which is how a template says "this part exists and is
    otherwise unspecified".
    """
    specs = context.body_specs
    bodies: dict[str, RigidBody] = {}
    for part in _suspension_template(context).parts:
        name = part.name
        if name == "chassis" or not name.endswith(f"_{side}"):
            continue
        if _stem_of(name) in _FOREIGN_STEMS:
            continue
        bodies[name] = (
            body_from_spec(name, specs[name])
            if name in specs
            else body_without_spec(name)
        )
    return bodies


def side_body_order(request: object) -> tuple[str, ...]:
    """
    Return the document's body sequence, derived from the template's parts.

    ``si_assembly`` used to hard-code this list, which meant the order was a fact
    about that module rather than about the model: a template declaring its parts in
    another sequence would have been reordered to match the list, silently.  The
    built-in template happens to declare exactly the recorded order, so deriving it
    here leaves every existing document bit-identical while making a template's own
    declaration the thing that decides.

    The template lists every part of the assembly, including the ones other
    subsystems *build* -- a tie rod and a rack belong to steering, the chassis to the
    chassis subsystem.  This function therefore returns the whole sequence for the
    subsystems this assembly actually **carries**, while :func:`side_bodies` builds
    only this subsystem's own parts: the order is a fact about the document, ownership
    is a fact about who constructs what, and which subsystems are present is a fact
    about the request.  Conflating them produced `duplicate bodies between fragments`
    when this was first written, and `body_order names bodies no contribution
    produced` when only the first half was fixed.

    ``request`` is the ``AssemblyRequest``: the template and the subsystem set are both
    reachable from it, so the caller does not have to build a context to ask this.
    """
    from ..templates import DOUBLE_WISHBONE

    instance = getattr(request, "instantiated_suspension", None)
    template = instance.template if instance is not None else DOUBLE_WISHBONE
    carries_steering = True
    carries = getattr(request, "carries", None)
    if callable(carries):
        carries_steering = bool(carries("steering"))
    return tuple(
        part.name
        for part in template.parts
        if carries_steering or _stem_of(part.name) not in _STEERING_STEMS
    )


#: The stems this subsystem does **not** build, though the template lists them.
#:
#: ``chassis`` is the chassis subsystem's, ``rack`` and ``tie_rod`` are steering's.
#: Stated here rather than inferred, because ownership is a decision about the design
#: and not something a name can be asked about.
_FOREIGN_STEMS: frozenset[str] = frozenset({"chassis", "rack", "tie_rod"})

#: The stems that exist only when the assembly carries steering.
_STEERING_STEMS: frozenset[str] = frozenset({"rack", "tie_rod"})


def _stem_of(name: str) -> str:
    """Return a body name's stem: ``tie_rod_L`` -> ``tie_rod``."""
    return name.rsplit("_", 1)[0] if name[-1] in ("L", "R") else name


def _build_joint(
    kind: str | None,
    *,
    name: str,
    body_a: str,
    point_a: np.ndarray,
    body_b: str,
    point_b: np.ndarray,
    axis_global: np.ndarray | None,
    bodies: dict[str, RigidBody],
) -> Constraint:
    """
    Build one joint from the template's declared type.

    The type is a template's *declaration*, so the mapping from it to a joint
    class lives here; the geometry conversion (a world axis into each body's local
    frame) is the same for every kind that carries one.  An unknown type is
    refused rather than defaulted: substituting a plausible joint for a declared
    one produces a model that solves and answers a different question.
    """
    if kind in (None, "spherical"):
        return BallJoint(body_a, point_a, body_b, point_b, name=name)
    if kind == "fixed":
        return WeldJoint(body_a, point_a, body_b, point_b, name=name)
    if kind == "revolute":
        axis_a, axis_b = _local_axes(axis_global, body_a, body_b, bodies)
        return RevoluteJoint(
            body_a, point_a, axis_a, body_b, point_b, axis_b, name=name
        )
    if kind == "prismatic":
        axis_a, axis_b = _local_axes(axis_global, body_a, body_b, bodies)
        return PrismaticJoint(
            body_a, point_a, axis_a, body_b, point_b, axis_b, name=name
        )
    if kind == "universal":
        axis_a, axis_b = _local_axes(axis_global, body_a, body_b, bodies)
        return UniversalJoint(
            body_a, point_a, axis_a, body_b, point_b, axis_b, name=name
        )
    if kind == "cylindrical":
        axis_a, axis_b = _local_axes(axis_global, body_a, body_b, bodies)
        return CylindricalJoint(
            body_a, point_a, axis_a, body_b, point_b, axis_b, name=name
        )
    raise ValueError(f"unsupported ideal joint kind {kind!r} in a suspension template")


def _local_axes(
    axis_global: np.ndarray | None,
    body_a: str,
    body_b: str,
    bodies: dict[str, RigidBody],
) -> tuple[np.ndarray, np.ndarray]:
    """Return one world axis expressed in each endpoint body's local frame."""
    if axis_global is None:
        raise ValueError("this joint kind needs an axis, and none was declared")
    return (
        bodies[body_a].pose.rotation.T @ axis_global,
        bodies[body_b].pose.rotation.T @ axis_global,
    )


def side_content(context: SubsystemContext, side: Side) -> SubsystemOutput:
    """
    Declare one side's points, joints and connection rows, as the template says.

    The template is what decides the topology, and the mode decides which column
    of each connection is active.  This function therefore reads *what to build*
    from the declaration and *how* from the mode, instead of carrying both as
    constants -- which is what makes choosing a different template produce a
    different subsystem.

    A row whose body this assembly does not carry is skipped rather than resolved
    against a missing body: with no steering subsystem the tie rod rows have
    nowhere to go, and that is exactly what requirement 15 asks for.
    """
    upright = f"upright_{side}"
    bodies = context.bodies
    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    connections: list[Connection] = []
    placeholders: list[ResolvedElement] = []

    # Every declared point is resolved first: the joints below consume these, and
    # the document records them in this order.
    for stem, label, hardpoint_role in _point_rows(context, side):
        body = f"{stem}_{side}"
        if body not in context.bodies:
            continue
        points[(body, label)] = context.local(body, context.point(side, hardpoint_role))

    for arm, label, hardpoint_role, connection in _inboard_rows(context, side):
        body = f"{arm}_{side}"
        if body not in bodies:
            continue
        global_point = context.point(side, hardpoint_role)
        local = context.local(body, global_point)
        # The support-side label comes from the connection's own declaration; the
        # historical build derives it from the arm's stem, which is the same string
        # and is now stated rather than derived in two places.
        chassis_label = connection.far_label
        points[("chassis", chassis_label)] = global_point.copy()
        column = connection.active_column(context.mode)
        # The connection row is recorded in *every* mode, including a mode where
        # the point constrains nothing: the row is the assembly's accounting of
        # what it attached, and the frozen snapshot carries all sixteen rows in K
        # even though K produces twelve constraints from them.  Only the
        # constraint is conditional on the activation.
        kind = "bushing" if column == "bushing" else "ideal"
        if column == "joint":
            joint_kind = connection.joint_kind(context.mode)
            axis_global = None
            if connection.axis_reference_role:
                axis_global = _inboard_axis(
                    context, side, connection.axis_reference_role, global_point, arm
                )
            joint = _build_joint(
                joint_kind,
                name=connection.name,
                body_a=connection.far_owner or "chassis",
                point_a=global_point,
                body_b=body,
                point_b=local,
                axis_global=axis_global,
                bodies=bodies,
            )
            constraints.append(joint)
            ideal_constraints.append(joint)
        elif column == "bushing":
            ideal_constraints.append(
                BallJoint(
                    "chassis",
                    global_point,
                    body,
                    local,
                    name=connection.name,
                )
            )
            placeholders.append(
                ResolvedElement(
                    kind="bushing",
                    name=f"{connection.bushing or connection.name}",
                    # The stiffness comes from the template's bushing slot via the
                    # assembly context, not from a constant written here.  The
                    # built-in template's own value is zero, so C-mode compliance is
                    # unchanged by default and a template that declares real
                    # stiffness changes it deliberately.
                    spec=context.mount_bushing_stiffness,
                    body_a="chassis",
                    body_b=body,
                    local_pose_a=SE3(
                        translation=global_point,
                        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    ),
                    local_pose_b=SE3(
                        translation=local,
                        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    ),
                )
            )
        connections.append(
            Connection(
                connection.name,
                kind,
                "chassis",
                body,
                chassis_label,
                label,
            )
        )

    for arm, _label, hardpoint_role, connection in _outer_rows(context, side):
        body = f"{arm}_{side}"
        if body not in bodies:
            continue
        global_point = context.point(side, hardpoint_role)
        local = context.local(body, global_point)
        upright_label = connection.far_label
        upright_point = context.local(upright, global_point)
        points[(upright, upright_label)] = upright_point.copy()
        joint_kind = connection.joint_kind(context.mode)
        joint = _build_joint(
            joint_kind,
            name=connection.name,
            body_a=body,
            point_a=local,
            body_b=upright,
            point_b=upright_point,
            axis_global=None,
            bodies=bodies,
        )
        constraints.append(joint)
        ideal_constraints.append(joint)
        connections.append(
            Connection(
                connection.name, "ideal", body, upright, "outer", upright_label
            )
        )

    return SubsystemOutput(
        points=points,
        connections=connections,
        constraints=constraints,
        ideal_constraints=ideal_constraints,
        # Consumed by the assembly's last element phase, not merged with the
        # rest: the original build appends them after everything else.
        bushings=placeholders,
    )


def elements(
    context: SubsystemContext, side: Side, kind: str
) -> list[ResolvedElement]:
    """
    Declare one side's spring, damper or bump stop rows.

    `kind` is the slot the assembly is filling, so each type keeps its original
    position in the element list rather than following subsystem order.
    """
    model = context.model
    if kind == "spring":
        return [
            ResolvedElement(
                kind="spring",
                name=f"{spec.name}_{side}",
                spec=spec,
                body_a=resolve_body(spec.body_a, side, context.bodies),
                point_a=context.local(
                    resolve_body(spec.body_a, side, context.bodies),
                    context.attachment_point(spec.point_a, side),
                ),
                body_b=resolve_body(spec.body_b, side, context.bodies),
                point_b=context.local(
                    resolve_body(spec.body_b, side, context.bodies),
                    context.attachment_point(spec.point_b, side),
                ),
            )
            for spec in model.springs
        ]
    if kind == "damper":
        return [
            ResolvedElement(
                kind="damper",
                name=f"{spec.name}_{side}",
                spec=spec,
                body_a=resolve_body(spec.body_a, side, context.bodies),
                point_a=context.local(
                    resolve_body(spec.body_a, side, context.bodies),
                    context.attachment_point(spec.point_a, side),
                ),
                body_b=resolve_body(spec.body_b, side, context.bodies),
                point_b=context.local(
                    resolve_body(spec.body_b, side, context.bodies),
                    context.attachment_point(spec.point_b, side),
                ),
            )
            for spec in model.dampers
        ]
    if kind == "bump_stop":
        return [
            ResolvedElement(
                kind="bump_stop",
                name=f"{spec.name}_{side}",
                spec=spec,
                body_a=resolve_body(spec.body_a, side, context.bodies),
                point_a=context.local(
                    resolve_body(spec.body_a, side, context.bodies),
                    context.attachment_point(spec.point_a, side),
                ),
                body_b=resolve_body(spec.body_b, side, context.bodies),
                point_b=context.local(
                    resolve_body(spec.body_b, side, context.bodies),
                    context.attachment_point(spec.point_b, side),
                ),
            )
            for spec in model.stops
        ]
    return []


def global_elements(context: SubsystemContext) -> list[ResolvedElement]:
    """Declare the anti-roll bar, which spans both sides and is emitted once."""
    return [
        ResolvedElement(
            kind="anti_roll_bar",
            name=spec.name,
            spec=spec,
            body_a="upright_L",
            point_a=context.local("upright_L", spec.left_link_point.as_array()),
            body_b="upright_R",
            point_b=context.local("upright_R", spec.right_link_point.as_array()),
        )
        for spec in context.model.anti_roll_bars
    ]


def compliance_elements(
    context: SubsystemContext, side: Side
) -> list[ResolvedElement]:
    """Declare the user's C-mode bushings for one side."""
    if context.mode != "C":
        return []
    declared: list[ResolvedElement] = []
    for spec in context.model.bushings:
        body_a = resolve_body(spec.body_a, side, context.bodies)
        body_b = resolve_body(spec.body_b, side, context.bodies)
        declared.append(
            ResolvedElement(
                kind="bushing",
                name=f"{spec.name}_{side}",
                spec=spec,
                body_a=body_a,
                body_b=body_b,
                local_pose_a=context.local_attachment(spec.pose_a, side, body_a),
                local_pose_b=context.local_attachment(spec.pose_b, side, body_b),
            )
        )
    return declared


def placeholder_bushings(
    context: SubsystemContext, side: Side
) -> list[ResolvedElement]:
    """
    Return the C-mode inboard placeholders collected by `side_content`.

    They are zero-stiffness today.  Requirement 7 / subtask 05 replaces them with
    the template's default properties; until then they keep the original position
    at the very end of the element list.
    """
    del context, side
    return []


def _inboard_axis(
    context: SubsystemContext,
    side: Side,
    rear_role: str,
    front_point: np.ndarray,
    arm: str,
) -> np.ndarray:
    """Return the unit inboard pivot axis, as the original build computes it."""
    axis_global = context.point(side, rear_role) - front_point
    norm = np.linalg.norm(axis_global)
    if norm <= 1e-12:
        raise ValueError(f"{arm.replace('_', ' ')} {side} inboard hardpoints must be distinct")
    return axis_global / norm
