"""
The steering subsystem: the rack, its guide, and the two tie rods.

This is the subsystem that makes requirement 15 real.  On a single axle a
suspension assembly may have **no steering at all**; when the declaration says so,
nothing here is emitted -- no rack body, no rack centre point, no tie rods, no
tie-rod ball joints, no rack guide.  The rest of the axle is untouched.

What it deliberately does *not* do is build a degenerate rack body that carries
no constraint.  That would leave a floating rigid body in an assembly that claims
not to have steering, and it would poison the rig's capability check downstream.

The full-vehicle side is not opened up: `SteeringSystemSpec` stays required and
`_build_steering` stays unconditional there (decision D5).  The asymmetry is
intentional and locked by a test.

The *declaration* is the template's rather than this module's: which bodies the
role has, the two tie-rod joints with their names and endpoints, and the rack
guide's name and kind all come from `STEERING_GUIDED` or `STEERING_FIXED`.  Two
templates are needed rather than one because the guide's kind is decided by
`model.rack_fixed_to_chassis`, which is a model field, and a template has no
spelling for a conditional -- so the two cases are declared separately and the
one the model asks for is instantiated.  What stays here is the *geometry*: the
points, the rack axis (the model's own, expressed in each body's frame) and the
order the assembly consumes the rows in.

`build` is the whole subsystem in one call, which is what makes the six
independently instantiable.  The assembly drives `bodies`, `side_content` and
`guide` separately instead, because the recorded contract interleaves the
steering rows with the suspension's on each side.
"""

from __future__ import annotations

import numpy as np

from ..modeling.primitives.joints import (
    BallJoint,
    Constraint,
    PrismaticJoint,
    RigidBody,
    WeldJoint,
)
from ..schema import Vec3
from ..templates.builtin import STEERING_FIXED, STEERING_GUIDED
from ..templates.instantiate import SubsystemInstance, instantiate
from ..templates.model import ConnectionDefinition, TemplateError
from .geometry import body_from_spec, body_without_spec
from .types import SIDES, Connection, Side, SubsystemContext, SubsystemOutput

__all__ = ["bodies", "build", "guide", "role", "side_content"]

#: The role this subsystem implements.
role = "steering"

#: The joint kinds a rack guide may be declared as, and what each one builds.
#:
#: The template states *which* kind; the module knows how to build the kinds a
#: rack guide can be.  A template that declared something else -- a revolute
#: guide, say -- is refused by name rather than silently built as a prismatic,
#: because the two are different constraints and not two spellings of one.
_GUIDE_JOINTS: dict[str, type] = {"prismatic": PrismaticJoint, "fixed": WeldJoint}


def _instance(context: SubsystemContext) -> SubsystemInstance:
    """
    Return the steering template to read, from the request or the model.

    The model's choice: a rack bolted to the chassis is a weld, a rack that slides
    along the rack axis is a prismatic joint, and the template that states each
    case is declared separately.
    The `rack_housing` exception is *not* a third template -- a model that supplies
    its own rack housing is a model whose guide is defined by the housing, which is
    a fact about the geometry rather than about the topology the template states.

    A *file* subsystem's steering template reaches here through the request and
    decides *how* a steered rack is built.  The model still decides *whether* an
    axle's rack is steered: an axle whose rack is bolted to the chassis keeps the
    built-in fixed template, because a bolted rack has no topology to choose -- and
    that is what lets one file's steering subsystem describe a whole vehicle, whose
    rear axle is bolted down while its front axle steers.
    """
    requested = context.request.role_instance("steering")
    if isinstance(requested, SubsystemInstance) and not (
        context.model.rack_fixed_to_chassis
    ):
        return requested
    template = STEERING_FIXED if context.model.rack_fixed_to_chassis else STEERING_GUIDED
    return instantiate(template, mode=context.mode)


def _body(context: SubsystemContext, name: str) -> RigidBody:
    """Build one steering body from its schema spec, if the model declares one."""
    specs = context.body_specs
    return (
        body_from_spec(name, specs[name])
        if name in specs
        else body_without_spec(name)
    )


def _declared(
    instance: SubsystemInstance, role_name: str, *, owner: str
) -> ConnectionDefinition:
    """
    Return the one connection the template declares for `role_name` on `owner`.

    Found by role and owner rather than by name: the name is the template's, and
    the pair (what the point is, which body carries it) is the interface.  Two
    matches are refused rather than picked between -- two joints at one point is
    a template that says two different things about one attachment.
    """
    matches = [
        connection
        for connection in instance.template.connections
        if connection.role == role_name and connection.owner == owner
    ]
    if len(matches) != 1:
        raise TemplateError(
            f"steering template {instance.template.name!r} declares {len(matches)} "
            f"{role_name!r} connections on {owner!r}; the role needs exactly one"
        )
    return matches[0]


def bodies(context: SubsystemContext) -> dict[str, RigidBody]:
    """
    Declare the rack and the two tie rods, as the template lists them.

    The bodies are registered in the shared context as they are declared, because
    the tie rod points are resolved against the rack and the uprights and those
    have to exist first.  Registration is idempotent, so the assembly can seed
    whatever it needs before calling in and still get the recorded order.
    """
    if not context.request.carries(role):
        return {}
    instance = _instance(context)
    declared: dict[str, RigidBody] = {}
    for part in instance.template.parts:
        declared[part.name] = _body(context, part.name)
        context.bodies[part.name] = declared[part.name]
    return declared


def side_content(context: SubsystemContext, side: Side) -> SubsystemOutput:
    """
    Declare one side's tie rod points, ball joints and connection rows.

    Each row is read from the connection the template declares for it: its name
    is the constraint's name, its `role` locates the hardpoint, its `label` names
    the point on the tie rod and its `far_label` the point on the far body.  Which
    end is recorded first is the template's `first_body`, which is the field that
    exists for exactly this -- the rack-side joint records the rack first and the
    upright-side joint records the tie rod first, and both are real choices rather
    than an accident of one build.
    """
    if not context.request.carries(role):
        return SubsystemOutput()
    instance = _instance(context)
    tie = f"tie_rod_{side}"
    declared = [
        connection
        for connection in instance.template.connections
        if connection.owner == tie
    ]
    if not declared:
        raise TemplateError(
            f"steering template {instance.template.name!r} declares no joint on "
            f"{tie!r}; a tie rod with nothing attached is not a steering subsystem"
        )

    points: dict[tuple[str, str], np.ndarray] = {}
    connections: list[Connection] = []
    constraints: list[Constraint] = []
    for connection in declared:
        far = connection.far_owner or "chassis"
        global_point = context.point(side, connection.role)
        own_local = context.local(tie, global_point)
        far_local = context.local(far, global_point)
        # `first_body` names the *second* end in the model's own vocabulary -- see
        # `solver._joint_bodies`, whose rule this reads rather than restates -- so
        # "far" puts the owner first and "owner" puts the far body first.
        owner_first = connection.first_body == "far"
        ends = (
            ((tie, own_local, connection.label), (far, far_local, connection.far_label))
            if owner_first
            else ((far, far_local, connection.far_label), (tie, own_local, connection.label))
        )
        (first, first_local, first_label), (second, second_local, second_label) = ends
        constraints.append(
            BallJoint(
                first,
                first_local,
                second,
                second_local,
                name=connection.name,
            )
        )
        connections.append(
            Connection(
                connection.name,
                "ideal",
                first,
                second,
                first_label or connection.role,
                second_label or connection.role,
            )
        )
        # Only the far end's point is recorded.  The tie rod's own points are the
        # joints' geometry, not entries the rest of the assembly looks up, and the
        # recorded contract carries exactly the two far-side rows per side.
        points[(far, connection.far_label or connection.role)] = far_local.copy()
    return SubsystemOutput(
        points=points,
        connections=connections,
        constraints=constraints,
        ideal_constraints=list(constraints),
    )


def guide(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the rack's support, the rack centre points, and `RACK_CENTER`.

    The guide's *name* and *kind* are the template's: the fixed and guided cases
    are two templates, and this reads whichever one the model asked for.  The axis
    a guided rack slides along is the model's own `rack_axis` expressed in each
    body's frame, because an axis is geometry rather than topology.

    Split out because the original build order computes the rack centre *after*
    the per-side tie rod loop, and the assembly consumes pieces in the order they
    are emitted: emitting the rack centre first would reorder the point table.
    """
    if not context.request.carries(role):
        return SubsystemOutput()

    model = context.model
    bodies_ = context.bodies
    instance = _instance(context)
    mount = _declared(instance, "rack_center", owner="rack")
    far = mount.far_owner or "chassis"
    rack_point = context.mirror("L", mount.role)
    rack_point_local = context.local(mount.owner, rack_point)
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []

    if "rack_housing" in bodies_:
        # 源模型自带齿条外壳时，齿条支承由外壳定义，不再叠加 chassis-rack 导向。
        rack_guide: Constraint | None = None
    else:
        joint_class = _GUIDE_JOINTS.get(mount.joint or "")
        if joint_class is None:
            raise TemplateError(
                f"steering template {instance.template.name!r} declares the rack "
                f"guide as {mount.joint!r}; a rack guide is "
                f"{sorted(_GUIDE_JOINTS)}"
            )
        if joint_class is WeldJoint:
            rack_guide = WeldJoint(
                far,
                rack_point,
                mount.owner,
                rack_point_local,
                name=mount.name,
            )
        else:
            rack_guide = PrismaticJoint(
                far,
                rack_point,
                _axis_in_body(model, far, bodies_),
                mount.owner,
                rack_point_local,
                _axis_in_body(model, mount.owner, bodies_),
                name=mount.name,
            )

    # 现役实现只把 rack_guide 追加到 constraints/ideal_constraints，不进连接表：
    # 连接表描述的是物理连接点，齿条导轨不算一个。这里保持一致。
    if rack_guide is not None:
        constraints.append(rack_guide)
        ideal_constraints.append(rack_guide)

    return SubsystemOutput(
        points={
            (mount.owner, mount.label or mount.role): rack_point_local,
            (far, mount.far_label or mount.role): rack_point.copy(),
        },
        hardpoints={
            "RACK_CENTER": Vec3(
                x=float(rack_point[0]),
                y=float(rack_point[1]),
                z=float(rack_point[2]),
            )
        },
        constraints=constraints,
        ideal_constraints=ideal_constraints,
    )


def build(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the whole steering subsystem in one call.

    The assembly does not use this: it drives `bodies`, `side_content` and
    `guide` separately so the recorded row order comes out right.  This entry
    point is what makes the subsystem independently instantiable.
    """
    declared = bodies(context)
    if not declared:
        return SubsystemOutput()
    merged = SubsystemOutput(bodies=declared)
    for side in SIDES:
        part = side_content(context, side)
        merged.points.update(part.points)
        merged.connections.extend(part.connections)
        merged.constraints.extend(part.constraints)
        merged.ideal_constraints.extend(part.ideal_constraints)
    tail = guide(context)
    merged.points.update(tail.points)
    merged.hardpoints.update(tail.hardpoints)
    merged.constraints.extend(tail.constraints)
    merged.ideal_constraints.extend(tail.ideal_constraints)
    return merged


def _axis_in_body(
    model: object, body: str, bodies_: dict[str, RigidBody]
) -> np.ndarray:
    """Express the model's vehicle-frame rack axis in `body` coordinates."""
    axis_world = np.asarray(model.rack_axis.as_tuple(), dtype=float)  # type: ignore[attr-defined]
    return bodies_[body].pose.rotation.T @ axis_world
