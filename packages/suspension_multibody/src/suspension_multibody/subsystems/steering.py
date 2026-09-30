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
    Constraint,
    PrismaticJoint,
    RigidBody,
    WeldJoint,
)
from ..schema import Vec3
from ..templates.builtin import STEERING_GUIDED
from ..templates.instantiate import SubsystemInstance, instantiate
from ..templates.model import ConnectionDefinition, TemplateError
from .geometry import body_from_part, body_from_spec
from .types import SIDES, Side, SubsystemContext, SubsystemOutput

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
    """Return the steering template to read, from the request or the model."""
    requested = context.request.role_instance("steering")
    if isinstance(requested, SubsystemInstance):
        return requested
    return instantiate(STEERING_GUIDED, mode=context.mode)


def _body(
    context: SubsystemContext, part: object, instance: SubsystemInstance
) -> RigidBody:
    """
    Build one steering body: the model's spec for it, or the template's.

    The rack is a part a model describes and the model wins there; the housing is
    the template's own support -- the rack slides in it and it mounts to the
    chassis -- so the template is what weighs it (see
    :func:`~.geometry.body_from_part`).  It is placed at the rack centre, the
    hardpoint its own declaration attaches it at: a left-side lookup, because the
    rack centre is one shared point rather than a per-side mount.
    """
    name = str(getattr(part, "name"))
    specs = context.body_specs
    if name in specs:
        return body_from_spec(name, specs[name])
    return body_from_part(
        name,
        part,
        # The rack and its housing are side-less, so which side places them does
        # not matter -- what matters is that the side is one this assembly *has*: a
        # one-sided corner has no left side to ask, and naming one here would make
        # a rack impossible on it.  Module subtask 06's rule is that a side is a
        # declaration, and this is the last consumer that spelt one out.
        center_of_mass=context.part_placement(
            name, instance.template.connections, context.request.sides[0]
        ),
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
    Declare the rack and rack housing, as the template lists them.

    Registration is idempotent, so the assembly can seed whatever it needs
    before calling in and still get the recorded order.
    """
    if not context.request.carries(role):
        return {}
    instance = _instance(context)
    declared: dict[str, RigidBody] = {}
    for part in instance.template.parts:
        declared[part.name] = _body(context, part, instance)
        context.bodies[part.name] = declared[part.name]
    return declared


def side_content(context: SubsystemContext, side: Side) -> SubsystemOutput:
    """
    Declare one side's rack connection point for the suspension tie rod.

    Tie rods and their joints are owned by the suspension subsystem.
    """
    if not context.request.carries(role):
        return SubsystemOutput()
    points: dict[tuple[str, str], np.ndarray] = {}
    if "rack" in context.bodies:
        global_point = context.point(side, "tie_inner")
        points[("rack", f"tie_{side}")] = context.local("rack", global_point)
    return SubsystemOutput(points=points)


def guide(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the rack's support, the rack centre points, and `RACK_CENTER`.

    The rack is guided in `rack_housing`, and the housing mounts to the chassis (or
    to ground when the assembly has none).  Which *kind* of guide the rack gets is
    the model's call rather than the template's -- `model.rack_fixed_to_chassis` --
    and the housing is what it is made against either way: a steered rack slides in
    its housing, and a rack bolted down cannot move at all, which is the same
    statement about where the rack sits with its one degree of freedom removed.  The
    two therefore differ in the joint, not in the bodies, and a rear axle that is
    not steered does not need a second template to say so.
    """
    if not context.request.carries(role):
        return SubsystemOutput()

    model = context.model
    bodies_ = context.bodies
    instance = _instance(context)
    mount = _declared(instance, "rack_center", owner="rack")
    rack_point = context.mirror("L", mount.role)
    rack_point_local = context.local(mount.owner, rack_point)
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    pts: dict[tuple[str, str], np.ndarray] = {
        (mount.owner, mount.label or mount.role): rack_point_local,
    }

    if "rack_housing" in bodies_:
        housing_local = context.local("rack_housing", rack_point)
        far = "chassis" if "chassis" in bodies_ else "ground"
        if far not in bodies_ and far == "ground":
            bodies_["ground"] = RigidBody("ground", fixed=True)
        far_point_local = context.local(far, rack_point)
        # The housing carries the rack's guide and is mounted to the chassis; the
        # rack itself is either guided in that housing or bolted down.  The two are
        # built against *different* bodies on purpose: a weld from the rack to the
        # housing that is itself welded to the chassis would say the same thing
        # twice -- twelve rows of constraint for six degrees of freedom -- and a
        # redundant pair is exactly what a static solve cannot balance.
        if model.rack_fixed_to_chassis:
            rack_guide: Constraint = WeldJoint(
                far,
                far_point_local,
                mount.owner,
                rack_point_local,
                name=RACK_BOLTED_NAME,
            )
        else:
            rack_guide = PrismaticJoint(
                "rack_housing",
                housing_local,
                _axis_in_body(model, "rack_housing", bodies_),
                mount.owner,
                rack_point_local,
                _axis_in_body(model, mount.owner, bodies_),
                name=mount.name,
            )
        constraints.append(rack_guide)
        ideal_constraints.append(rack_guide)

        housing_mount = WeldJoint(
            far,
            far_point_local,
            "rack_housing",
            housing_local,
            name="housing_mount",
        )
        constraints.append(housing_mount)
        ideal_constraints.append(housing_mount)
        pts[(far, mount.far_label or mount.role)] = far_point_local.copy()
    else:
        # A template that declares no housing guides the rack straight against the
        # body the connection names, which is the pre-housing topology.
        far = mount.far_owner or ("chassis" if "chassis" in bodies_ else "ground")
        if far not in bodies_:
            far = "ground"
            if "ground" not in bodies_:
                bodies_["ground"] = RigidBody("ground", fixed=True)
        far_point_local = context.local(far, rack_point)
        if model.rack_fixed_to_chassis:
            rack_guide = WeldJoint(
                far,
                far_point_local,
                mount.owner,
                rack_point_local,
                name=RACK_BOLTED_NAME,
            )
        else:
            rack_guide = PrismaticJoint(
                far,
                far_point_local,
                _axis_in_body(model, far, bodies_),
                mount.owner,
                rack_point_local,
                _axis_in_body(model, mount.owner, bodies_),
                name=mount.name,
            )
        constraints.append(rack_guide)
        ideal_constraints.append(rack_guide)
        pts[(far, mount.far_label or mount.role)] = far_point_local.copy()

    return SubsystemOutput(
        points=pts,
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

#: The name a rack bolted to the chassis records: the constraint the recorded
#: contract has always carried for a rack that cannot move.
RACK_BOLTED_NAME = "rack_fixed_to_chassis"


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
