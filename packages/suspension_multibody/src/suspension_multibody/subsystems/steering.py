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

`build` is the whole subsystem in one call, which is what makes the six
independently instantiable.  The assembly drives `bodies`, `side_content` and
`guide` separately instead, because the recorded contract interleaves the
steering rows with the suspension's on each side.
"""

from __future__ import annotations

import numpy as np

from ..preparation.assembly.types import (
    BallJoint,
    Constraint,
    PrismaticJoint,
    RigidBody,
    WeldJoint,
)
from ..schema import Vec3
from .geometry import body_from_spec, body_without_spec
from .types import SIDES, Connection, Side, SubsystemContext, SubsystemOutput

__all__ = ["bodies", "build", "guide", "role", "side_content"]

#: The role this subsystem implements.
role = "steering"


def _body(context: SubsystemContext, name: str) -> RigidBody:
    """Build one steering body from its schema spec, if the model declares one."""
    specs = context.body_specs
    return (
        body_from_spec(name, specs[name])
        if name in specs
        else body_without_spec(name)
    )


def bodies(context: SubsystemContext) -> dict[str, RigidBody]:
    """
    Declare the rack and the two tie rods.

    The bodies are registered in the shared context as they are declared, because
    the tie rod points are resolved against the rack and the uprights and those
    have to exist first.  Registration is idempotent, so the assembly can seed
    whatever it needs before calling in and still get the recorded order.
    """
    if not context.request.carries(role):
        return {}
    declared = {"rack": _body(context, "rack")}
    context.bodies["rack"] = declared["rack"]
    for side in SIDES:
        tie = f"tie_rod_{side}"
        declared[tie] = _body(context, tie)
        context.bodies[tie] = declared[tie]
    return declared


def side_content(context: SubsystemContext, side: Side) -> SubsystemOutput:
    """Declare one side's tie rod points, ball joints and connection rows."""
    if not context.request.carries(role):
        return SubsystemOutput()
    tie = f"tie_rod_{side}"
    upright = f"upright_{side}"
    tie_inner_global = context.point(side, "tie_inner")
    tie_outer_global = context.point(side, "tie_outer")
    tie_inner = context.local(tie, tie_inner_global)
    tie_outer = context.local(tie, tie_outer_global)
    rack_tie_inner = context.local("rack", tie_inner_global)
    upright_tie_outer = context.local(upright, tie_outer_global)
    rack_joint = BallJoint(
        "rack", rack_tie_inner, tie, tie_inner, name=f"rack_tie_joint_{side}"
    )
    tie_joint = BallJoint(
        tie, tie_outer, upright, upright_tie_outer, name=f"tie_upright_joint_{side}"
    )
    return SubsystemOutput(
        points={
            ("rack", f"tie_{side}"): rack_tie_inner.copy(),
            (upright, "tie_outer"): upright_tie_outer.copy(),
        },
        connections=[
            Connection(
                f"rack_tie_joint_{side}", "ideal", "rack", tie, f"tie_{side}", "inner"
            ),
            Connection(
                f"tie_upright_joint_{side}", "ideal", tie, upright, "outer", "tie_outer"
            ),
        ],
        constraints=[rack_joint, tie_joint],
        ideal_constraints=[rack_joint, tie_joint],
    )


def guide(context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the rack's support, the rack centre points, and `RACK_CENTER`.

    Split out because the original build order computes the rack centre *after*
    the per-side tie rod loop, and the assembly consumes pieces in the order they
    are emitted: emitting the rack centre first would reorder the point table.
    """
    if not context.request.carries(role):
        return SubsystemOutput()

    model = context.model
    bodies_ = context.bodies
    rack_point = context.mirror("L", "rack_center")
    rack_point_local = context.local("rack", rack_point)
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []

    if model.rack_fixed_to_chassis:
        rack_guide: Constraint | None = WeldJoint(
            "chassis",
            rack_point,
            "rack",
            rack_point_local,
            name="rack_fixed_to_chassis",
        )
    elif "rack_housing" not in bodies_:
        # The rack is a guided rigid body: its only ideal degree of freedom is
        # translation along the declared vehicle Y rack axis.
        rack_guide = PrismaticJoint(
            "chassis",
            rack_point,
            _axis_in_body(model, "chassis", bodies_),
            "rack",
            rack_point_local,
            _axis_in_body(model, "rack", bodies_),
            name="rack_guide",
        )
    else:
        # 源模型自带齿条外壳时，齿条支承由外壳定义，不再叠加 chassis-rack 导向。
        rack_guide = None

    # 现役实现只把 rack_guide 追加到 constraints/ideal_constraints，不进连接表：
    # 连接表描述的是物理连接点，齿条导轨不算一个。这里保持一致。
    if rack_guide is not None and "rack_housing" not in bodies_:
        constraints.append(rack_guide)
        ideal_constraints.append(rack_guide)

    return SubsystemOutput(
        points={
            ("rack", "center"): rack_point_local,
            ("chassis", "rack_center"): rack_point.copy(),
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
