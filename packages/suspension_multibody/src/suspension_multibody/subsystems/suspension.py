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

from ..preparation.assembly.types import (
    BallJoint,
    Constraint,
    RevoluteJoint,
    RigidBody,
)
from ..preparation.geometry import SE3
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
    "side_content",
]

#: The role this subsystem implements.
role = "suspension"

#: One row per mount-table point: `(body, label, hardpoint role)`.
_MOUNT_DATA: tuple[tuple[str, str, str], ...] = (
    ("upper_arm", "inner_front", "upper_front"),
    ("upper_arm", "inner_rear", "upper_rear"),
    ("upper_arm", "outer", "upper_outer"),
    ("lower_arm", "inner_front", "lower_front"),
    ("lower_arm", "inner_rear", "lower_rear"),
    ("lower_arm", "outer", "lower_outer"),
    ("tie_rod", "inner", "tie_inner"),
    ("tie_rod", "outer", "tie_outer"),
    ("upright", "wheel_center", "wheel_center"),
)

#: `(arm suffix, inboard point roles)` for the two arms.
_ARMS: tuple[tuple[str, str, str, str], ...] = (
    ("upper_arm", "uca", "upper_front", "upper_rear"),
    ("lower_arm", "lca", "lower_front", "lower_rear"),
)


def side_bodies(context: SubsystemContext, side: Side) -> dict[str, RigidBody]:
    """Declare the arm and upright bodies for one side."""
    specs = context.body_specs
    bodies: dict[str, RigidBody] = {}
    for stem in ("upper_arm", "lower_arm", "upright"):
        name = f"{stem}_{side}"
        bodies[name] = (
            body_from_spec(name, specs[name])
            if name in specs
            else body_without_spec(name)
        )
    return bodies


def side_content(context: SubsystemContext, side: Side) -> SubsystemOutput:
    """Declare one side's points, joints and connection rows."""
    uca = f"upper_arm_{side}"
    lca = f"lower_arm_{side}"
    upright = f"upright_{side}"
    bodies = context.bodies
    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    connections: list[Connection] = []
    placeholders: list[ResolvedElement] = []

    # The mount table resolves every declared point first: the joints below
    # consume these, and the document records them in this order.  A row whose
    # body this assembly does not carry is skipped rather than resolved against a
    # missing body -- with no steering subsystem the tie rod rows have nowhere to
    # go, and that is exactly what requirement 15 asks for.
    for stem, label, hardpoint_role in _MOUNT_DATA:
        body = f"{stem}_{side}"
        if body not in context.bodies:
            continue
        global_point = context.point(side, hardpoint_role)
        points[(body, label)] = context.local(body, global_point)

    for arm, stem, front_role, rear_role in _ARMS:
        body = f"{arm}_{side}"
        for label, hardpoint_role in (
            ("inner_front", front_role),
            ("inner_rear", rear_role),
        ):
            global_point = context.point(side, hardpoint_role)
            local = context.local(body, global_point)
            chassis_label = f"{stem}_{side}_{label}"
            points[("chassis", chassis_label)] = global_point.copy()
            if context.mode == "K":
                if label == "inner_front":
                    axis_global = _inboard_axis(context, side, rear_role, global_point, arm)
                    constraints.append(
                        RevoluteJoint(
                            "chassis",
                            global_point,
                            bodies["chassis"].pose.rotation.T @ axis_global,
                            body,
                            local,
                            bodies[body].pose.rotation.T @ axis_global,
                            name=f"{stem}_mount_{side}_inner_front",
                        )
                    )
                    ideal_constraints.append(constraints[-1])
                kind = "ideal"
            else:
                ideal_constraints.append(
                    BallJoint(
                        "chassis",
                        global_point,
                        body,
                        local,
                        name=f"{stem}_mount_{side}_{label}",
                    )
                )
                placeholders.append(
                    ResolvedElement(
                        kind="bushing",
                        name=f"{stem}_bushing_{side}_{label}",
                        # The stiffness comes from the template's bushing slot via
                        # the assembly context, not from a constant written here.
                        # The built-in template's own value is zero, so C-mode
                        # compliance is unchanged by default and a template that
                        # declares real stiffness changes it deliberately.
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
                kind = "bushing"
            connections.append(
                Connection(
                    f"{stem}_mount_{side}_{label}",
                    kind,
                    "chassis",
                    body,
                    chassis_label,
                    label,
                )
            )

    for arm, hardpoint_role in ((uca, "upper_outer"), (lca, "lower_outer")):
        global_point = context.point(side, hardpoint_role)
        local = context.local(arm, global_point)
        upright_label = f"{arm}_outer"
        upright_point = context.local(upright, global_point)
        points[(upright, upright_label)] = upright_point.copy()
        joint = BallJoint(
            arm, local, upright, upright_point, name=f"{arm}_outer_joint"
        )
        constraints.append(joint)
        ideal_constraints.append(joint)
        connections.append(
            Connection(
                f"{arm}_outer_joint", "ideal", arm, upright, "outer", upright_label
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
