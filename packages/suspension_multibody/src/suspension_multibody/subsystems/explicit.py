"""
The explicit topology: a model that states its own parts and joints.

An explicit model is the import path for a source model -- an Adams dataset, a
generated fixture -- whose bodies and joints are already known and must be taken
literally rather than composed from subsystem roles.  It is the second of the two
topologies ``FrontAxleModel.topology`` declares, and it used to live in the axle
assembly, which is why a composition could not produce one.

What an explicit build does **not** do is apply the symmetric-proxy convention:
there is no left-side model to mirror, no subsystem set to expand, and no
template to consult.  Every body is declared, every joint is declared, and the
two rack supports the source format distinguishes (a rack fixed to the chassis,
and a rack guided along its own axis) are chosen from the declaration.

Nothing here solves or submits native.  It builds the runtime face a document is
authored from, exactly like the composed path.
"""

from __future__ import annotations

from typing import Literal

import numpy as np

from ..modeling.primitives import (
    SE3,
    BallJoint,
    BumpStopElement,
    BushingElement,
    ConstantVelocityJoint,
    Constraint,
    CylindricalJoint,
    InPlaneJoint,
    LinearSpringElement,
    PrismaticJoint,
    RevoluteJoint,
    RigidBody,
    StaticDamperElement,
    UniversalJoint,
    WeldJoint,
)
from ..schema import (
    FrontAxleModel,
    IdealJointSpec,
    Pose,
)
from .capabilities import capabilities_for
from .geometry import body_from_spec, local_point, lookup_hardpoint
from .runtime import SubsystemRuntime
from .types import Connection

__all__ = [
    "build_explicit_runtime",
    "explicit_constraint",
]


def build_explicit_runtime(
    model: FrontAxleModel, mode: Literal["K", "C"]
) -> SubsystemRuntime:
    """
    Build the runtime face of an explicit axle.

    The roles are derived from what the build actually produced rather than
    asserted: an explicit source model that declares no rack body has no steering,
    and claiming one made the rig offer a rack coordinate the model does not
    declare -- the run then failed inside the kernel with "unknown coordinate
    rack_drive", which names the symptom and not the cause.
    """
    if mode not in ("K", "C"):
        raise ValueError(f"mode must be K or C, got {mode!r}")

    body_specs = {spec.name: spec for spec in model.bodies}
    bodies: dict[str, RigidBody] = {"chassis": RigidBody("chassis", fixed=True)}
    for name, spec in body_specs.items():
        if name == "chassis":
            raise ValueError("explicit axle body specs must not redefine chassis")
        bodies[name] = body_from_spec(name, spec)

    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Constraint] = []
    connections: list[Connection] = []

    for spec in model.joints:
        if spec.body_a not in bodies or spec.body_b not in bodies:
            raise ValueError(f"explicit joint {spec.name!r} references an unknown body")
        point_a = local_point(bodies, spec.body_a, spec.point_a.as_array())
        point_b = local_point(bodies, spec.body_b, spec.point_b.as_array())
        points[(spec.body_a, f"{spec.name}_a")] = point_a
        points[(spec.body_b, f"{spec.name}_b")] = point_b
        constraint = explicit_constraint(spec, bodies)
        constraints.append(constraint)
        connections.append(
            Connection(
                name=spec.name,
                kind="ideal",
                body_a=spec.body_a,
                body_b=spec.body_b,
                point_a=f"{spec.name}_a",
                point_b=f"{spec.name}_b",
            )
        )

    for side in ("L", "R"):
        upright = f"upright_{side}"
        if upright not in bodies:
            continue
        try:
            if side == "L":
                global_point = lookup_hardpoint(model.hardpoints, "wheel_center").as_array()
            else:
                # A generated right-side hardpoint set is preferred over a mirror
                # of the left one: the source model may state the right side
                # explicitly, and mirroring would then quietly override it.
                right_hardpoints = {
                    key.removesuffix("__R"): value
                    for key, value in model.hardpoints.items()
                    if key.endswith("__R")
                }
                if right_hardpoints:
                    global_point = lookup_hardpoint(right_hardpoints, "wheel_center").as_array()
                else:
                    global_point = (
                        lookup_hardpoint(model.hardpoints, "wheel_center")
                        .mirrored_y()
                        .as_array()
                    )
        except ValueError:
            continue
        points[(upright, "wheel_center")] = local_point(bodies, upright, global_point)

    rack = "rack"
    if rack in bodies:
        rack_center = lookup_hardpoint(model.hardpoints, "rack_center").as_array()
        points[(rack, "center")] = local_point(bodies, rack, rack_center)
        points[("chassis", "rack_center")] = rack_center.copy()
        if model.rack_fixed_to_chassis:
            rack_joint: Constraint = WeldJoint(
                "chassis",
                rack_center,
                rack,
                points[(rack, "center")],
                name="rack_fixed_to_chassis",
            )
            constraints.append(rack_joint)
            connections.append(
                Connection(
                    name=rack_joint.name,
                    kind="ideal",
                    body_a="chassis",
                    body_b=rack,
                    point_a="rack_center",
                    point_b="center",
                )
            )
        elif "rack_housing" not in bodies:
            # A free rack is allowed to translate along its own axis only, which
            # keeps the explicit source topology from introducing an unconstrained
            # rigid-body freedom.  When the source supplies a rack housing the
            # support comes from the source TRANSLATIONAL plus the housing
            # bushings, so a rigid chassis-rack guide must not be added on top.
            rack_axis_world = np.asarray(model.rack_axis.as_tuple(), dtype=float)
            chassis_axis = bodies["chassis"].pose.rotation.T @ rack_axis_world
            rack_axis = bodies[rack].pose.rotation.T @ rack_axis_world
            rack_joint = PrismaticJoint(
                "chassis",
                rack_center,
                chassis_axis,
                rack,
                points[(rack, "center")],
                rack_axis,
                name="rack_guide",
            )
            constraints.append(rack_joint)
            connections.append(
                Connection(
                    name=rack_joint.name,
                    kind="ideal",
                    body_a="chassis",
                    body_b=rack,
                    point_a="rack_center",
                    point_b="center",
                )
            )

    elements = _explicit_elements(model, mode, bodies)
    bushings = (
        tuple(element for element in elements if isinstance(element, BushingElement))
        if mode == "C"
        else ()
    )
    return SubsystemRuntime(
        mode=mode,
        bodies=bodies,
        points=points,
        hardpoints=dict(model.hardpoints),
        connections=tuple(connections),
        constraints=tuple(constraints),
        ideal_constraints=tuple(constraints),
        bushings=bushings,
        elements=elements,
        capabilities=capabilities_for(
            subsystems=explicit_roles(bodies, constraints, points),
            body_names=frozenset(bodies),
        ),
        # The explicit topology reaches the same document author as the composed one, so
        # a surface the model declared has to travel the same way -- otherwise an
        # explicit model's ground would be dropped while a composed model's survived.
        road=model.road,
    )


def explicit_constraint(spec: IdealJointSpec, bodies: dict[str, RigidBody]) -> Constraint:
    """Create one core constraint from an explicit vehicle-frame joint spec."""
    point_a = local_point(bodies, spec.body_a, spec.point_a.as_array())
    point_b = local_point(bodies, spec.body_b, spec.point_b.as_array())
    axis_a = bodies[spec.body_a].pose.rotation.T @ spec.axis_a.as_array()
    axis_b = bodies[spec.body_b].pose.rotation.T @ spec.axis_b.as_array()
    common = {
        "body_a": spec.body_a,
        "point_a": point_a,
        "body_b": spec.body_b,
        "point_b": point_b,
        "name": spec.name,
    }
    if spec.kind == "spherical":
        return BallJoint(**common)  # ty: ignore[missing-argument]
    if spec.kind == "fixed":
        return WeldJoint(**common)  # ty: ignore[missing-argument]
    if spec.kind == "revolute":
        return RevoluteJoint(**common, axis_a=axis_a, axis_b=axis_b)  # ty: ignore[missing-argument]
    if spec.kind == "prismatic":
        return PrismaticJoint(**common, axis_a=axis_a, axis_b=axis_b)  # ty: ignore[missing-argument]
    if spec.kind == "universal":
        return UniversalJoint(**common, axis_a=axis_a, axis_b=axis_b)  # ty: ignore[missing-argument]
    if spec.kind == "constant_velocity":
        secondary_a = bodies[spec.body_a].pose.rotation.T @ spec.axis_a_secondary.as_array()
        secondary_b = bodies[spec.body_b].pose.rotation.T @ spec.axis_b_secondary.as_array()
        return ConstantVelocityJoint(  # ty: ignore[missing-argument]
            **common,
            axis_a=axis_a,
            axis_a_secondary=secondary_a,
            axis_b=axis_b,
            axis_b_secondary=secondary_b,
            angle_target=spec.constant_velocity_angle_target,
        )
    if spec.kind == "cylindrical":
        return CylindricalJoint(**common, axis_a=axis_a, axis_b=axis_b)  # ty: ignore[missing-argument]
    if spec.kind == "inplane":
        return InPlaneJoint(**common, axis_a=axis_a)  # ty: ignore[missing-argument]
    raise ValueError(f"unsupported explicit ideal joint kind {spec.kind!r}")


def explicit_roles(
    bodies: dict[str, RigidBody],
    constraints: list[Constraint] | tuple[Constraint, ...],
    points: dict[tuple[str, str], np.ndarray],
) -> frozenset[str]:
    """
    Return the subsystem roles an explicit build actually produced.

    An explicit source model states its parts and joints directly, so there is no
    request to read the roles off.  They are read off the *result* instead: a rack
    body means steering, a wheel-carrying body means the wheel subsystem, and the
    chassis is the fixed body the rest hangs from.  Suspension is present whenever
    a non-chassis body is connected, which is what distinguishes a suspension
    assembly from a bare chassis.
    """
    roles: set[str] = {"chassis"}
    free = [name for name, body in bodies.items() if name != "chassis" and not body.fixed]
    if free and constraints:
        roles.add("suspension")
    if "rack" in bodies or "rack_housing" in bodies:
        roles.add("steering")
    if any(label == "wheel_center" for _, label in points):
        roles.add("wheel")
    return frozenset(roles)


def _explicit_local_pose(value: Pose, body: str, bodies: dict[str, RigidBody]) -> SE3:
    """Convert an explicit global attachment frame to a body-local frame."""
    global_pose = SE3(
        value.translation.as_array(),
        np.asarray(value.rotation.as_tuple(), dtype=float),
    )
    return bodies[body].pose.inverse().compose(global_pose)


def _explicit_elements(
    model: FrontAxleModel,
    mode: Literal["K", "C"],
    bodies: dict[str, RigidBody],
) -> tuple[object, ...]:
    """Build force elements without applying the symmetric proxy convention."""
    elements: list[object] = []
    for spec in model.springs:
        elements.append(
            LinearSpringElement(
                name=spec.name,
                body_a=spec.body_a,
                point_a=local_point(bodies, spec.body_a, spec.point_a.as_array()),
                body_b=spec.body_b,
                point_b=local_point(bodies, spec.body_b, spec.point_b.as_array()),
                stiffness=spec.stiffness,
                free_length=spec.free_length,
                reference_length=spec.reference_length,
                preload=spec.preload or 0.0,
                force_curve=spec.force_curve,
            )
        )
    for spec in model.dampers:
        elements.append(
            StaticDamperElement(
                name=spec.name,
                body_a=spec.body_a,
                point_a=local_point(bodies, spec.body_a, spec.point_a.as_array()),
                body_b=spec.body_b,
                point_b=local_point(bodies, spec.body_b, spec.point_b.as_array()),
                gas_stiffness=spec.gas_stiffness,
                gas_reference_length=spec.gas_reference_length,
                gas_reference_force=spec.gas_reference_force,
                preload=spec.preload,
                friction=spec.friction,
                viscous_damping=spec.viscous_damping,
                force_curve=spec.force_curve,
            )
        )
    for spec in model.stops:
        elements.append(
            BumpStopElement(
                name=spec.name,
                body_a=spec.body_a,
                point_a=local_point(bodies, spec.body_a, spec.point_a.as_array()),
                body_b=spec.body_b,
                point_b=local_point(bodies, spec.body_b, spec.point_b.as_array()),
                clearance=spec.clearance,
                stiffness=spec.stiffness,
                direction=spec.direction,
                force_curve=spec.force_curve,
            )
        )
    if mode == "C":
        for spec in model.bushings:
            elements.append(
                BushingElement(
                    name=spec.name,
                    body_a=spec.body_a,
                    body_b=spec.body_b,
                    local_pose_a=_explicit_local_pose(spec.pose_a, spec.body_a, bodies),
                    local_pose_b=_explicit_local_pose(spec.pose_b, spec.body_b, bodies),
                    stiffness=np.asarray(spec.stiffness, dtype=float),
                    damping=np.diag(np.asarray(spec.damping, dtype=float)),
                    preload=np.asarray(spec.preload, dtype=float),
                    force_curves=spec.force_curves,
                    force_curve_interpolation=spec.force_curve_interpolation,
                    rotation_coordinates=spec.rotation_coordinates,
                )
            )
    return tuple(elements)
