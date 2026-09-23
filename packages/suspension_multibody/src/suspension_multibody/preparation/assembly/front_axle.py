"""
Symmetric front double-wishbone and rack steering assembly.

The author-side half of the axle: hardpoint mirroring, schema-to-declaration
conversion and the naming the contract needs, and nothing else.  It builds the
declaration data in ``types`` and never solves, submits native or decodes a
result.  Moved here from ``model/front_axle.py``, which 08 deletes.

The axle is now *composed* from the six subsystems in ``subsystems/`` rather than
built inline.  This module keeps three jobs a subsystem must not take:

* it owns the element constructors, because it is one of the two modules the
  legacy surface gate has registered as an ``elements`` importer;
* it owns the emission order, because that order *is* the recorded contract --
  the document lists bodies, constraints and elements in the order they are
  appended, so the order lives here and the subsystems only decide content;
* it re-exports the hardpoint helpers that used to live here, so existing
  importers (``analysis/vehicle_physics.py``, ``adams/strict_c.py``) are
  unchanged.

``build_front_axle(model, mode)`` keeps its signature and ``FrontAxleAssembly``
keeps its fields.  Absence (requirement 15) arrives through ``request``; the
default is the subsystem set the axle has always carried, so every existing
caller builds what it built before.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Literal, cast

import numpy as np

from ...elements import (
    AntiRollBarElement,
    BumpStopElement,
    BushingElement,
    LinearSpringElement,
    StaticDamperElement,
    VerticalTireElement,
)
from ...schema import (
    AntiRollBar,
    BumpStop,
    Bushing6x6,
    FrontAxleModel,
    IdealJointSpec,
    LinearSpring,
    Pose,
    StaticDamper,
    Vec3,
    VerticalTire,
)
from ...subsystems import (
    AssemblyRequest,
    SubsystemContext,
    capabilities_for,
)
from ...subsystems import chassis as chassis_subsystem
from ...subsystems import steering as steering_subsystem
from ...subsystems import suspension as suspension_subsystem
from ...subsystems import wheel as wheel_subsystem
from ...subsystems.capabilities import AssemblyCapabilities
from ...subsystems.geometry import (
    HARDPOINT_ALIASES,
    as_array,
    body_from_spec,
    body_without_spec,
    local_point,
    local_pose,
    lookup_hardpoint,
    mirror_hardpoints,
    mirror_point,
    resolve_body,
    side_hardpoints,
)
from ...subsystems.types import Connection, ResolvedElement
from ..geometry import SE3
from .types import (
    BallJoint,
    ConstantVelocityJoint,
    Constraint,
    CylindricalJoint,
    InPlaneJoint,
    PrismaticJoint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
    UniversalJoint,
    WeldJoint,
)

__all__ = [
    "AssemblyCapabilities",
    "Connection",
    "FrontAxleAssembly",
    "build_front_axle",
    "mirror_hardpoints",
    "side_hardpoints",
]

# Private aliases: existing callers import these names directly
# (`cases/kc_quasi_static/contract.py`, `preparation/assembly/vehicle.py`,
# `tests/model/test_vehicle.py`), so the names have to keep resolving here even
# though the definitions now live in `subsystems/geometry.py`.
_array = as_array
_lookup = lookup_hardpoint
_mirror_point = mirror_point
_local_point = local_point
_local_pose = local_pose
_resolve_body = resolve_body
_body_from_spec = body_from_spec
_body_with_points = body_without_spec
_ALIASES = HARDPOINT_ALIASES


@dataclass
class FrontAxleAssembly:
    """Constructed two-sided front axle model."""

    mode: Literal["K", "C"]
    bodies: dict[str, RigidBody]
    state: RigidBodyState
    points: dict[tuple[str, str], np.ndarray]
    hardpoints: dict[str, Vec3]
    connections: tuple[Connection, ...]
    constraints: tuple[Constraint, ...]
    ideal_constraints: tuple[Constraint, ...] = ()
    bushings: tuple[BushingElement, ...] = ()
    elements: tuple[object, ...] = ()
    #: What this assembly actually carries.  A rig binds to this instead of
    #: probing for body names; immutable, so a binding cannot go stale.
    capabilities: AssemblyCapabilities | None = None

    def point(self, body: str, label: str) -> np.ndarray:
        """Return a body-local point by stable label."""
        return self.points[(body, label)].copy()

    @property
    def component_ids(self) -> tuple[str, ...]:
        return tuple(self.bodies)

    @property
    def element_ids(self) -> tuple[str, ...]:
        """Return stable force-element identifiers for result tables."""
        return tuple(
            getattr(element, "name", f"element_{index}")
            for index, element in enumerate(self.elements)
        )


def _with_body_specs(
    bodies: dict[str, RigidBody], model: FrontAxleModel
) -> dict[str, RigidBody]:
    """
    Apply the model's mass specs to the bodies the subsystems declared.

    A spec for a body this assembly does not carry is skipped rather than
    rejected: with the steering subsystem left out (requirement 15) the model may
    still declare a rack or tie rod spec, and that is not an error -- there is
    simply nothing to apply it to.  The assembly has already refused any role it
    could not build, so a leftover spec is a declaration about a body this
    assembly chose not to carry, not a typo.
    """
    updated = dict(bodies)
    for spec in model.bodies:
        if spec.name not in updated:
            continue
        body = updated[spec.name]
        updated[spec.name] = RigidBody(
            name=body.name,
            pose=body.pose,
            mass=spec.mass,
            inertia=np.asarray(spec.inertia, dtype=float),
            center_of_mass=spec.center_of_mass.as_array(),
            fixed=body.fixed or spec.fixed,
        )
    return updated


def _explicit_constraint(
    spec: IdealJointSpec, bodies: dict[str, RigidBody]
) -> Constraint:
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


def _explicit_local_pose(value: Pose, body: str, bodies: dict[str, RigidBody]) -> SE3:
    """Convert an explicit global attachment frame to a body-local frame."""
    global_pose = SE3(
        value.translation.as_array(),
        np.asarray(value.rotation.as_tuple(), dtype=float),
    )
    return bodies[body].pose.inverse().compose(global_pose)


def _runtime_elements_explicit(
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


def _build_explicit_axle(
    model: FrontAxleModel, mode: Literal["K", "C"]
) -> FrontAxleAssembly:
    """Build an axle from explicit bodies and source-frame joint declarations."""
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
            raise ValueError(
                f"explicit joint {spec.name!r} references an unknown body"
            )
        point_a = local_point(bodies, spec.body_a, spec.point_a.as_array())
        point_b = local_point(bodies, spec.body_b, spec.point_b.as_array())
        points[(spec.body_a, f"{spec.name}_a")] = point_a
        points[(spec.body_b, f"{spec.name}_b")] = point_b
        constraint = _explicit_constraint(spec, bodies)
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
                global_point = lookup_hardpoint(
                    model.hardpoints, "wheel_center"
                ).as_array()
            else:
                right_hardpoints = {
                    key.removesuffix("__R"): value
                    for key, value in model.hardpoints.items()
                    if key.endswith("__R")
                }
                if right_hardpoints:
                    global_point = lookup_hardpoint(
                        right_hardpoints, "wheel_center"
                    ).as_array()
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
            # 自由齿条只允许沿其轴线平移，避免显式源拓扑引入未约束的刚体自由度。
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
        # 当显式源模型提供齿条外壳时，齿条的支承由源 TRANSLATIONAL
        # 和外壳衬套共同定义；不能再叠加一个 chassis-rack 刚性导向。

    runtime_elements = _runtime_elements_explicit(model, mode, bodies)
    explicit_bushings = tuple(
        element for element in runtime_elements if isinstance(element, BushingElement)
    )
    return FrontAxleAssembly(
        mode=mode,
        bodies=bodies,
        state=RigidBodyState(bodies),
        points=points,
        hardpoints=dict(model.hardpoints),
        connections=tuple(connections),
        constraints=tuple(constraints),
        ideal_constraints=tuple(constraints),
        bushings=explicit_bushings if mode == "C" else (),
        elements=runtime_elements,
        capabilities=capabilities_for(
            subsystems=frozenset({"chassis", "suspension", "steering", "wheel"}),
            body_names=frozenset(bodies),
        ),
    )


def _spring(row: ResolvedElement, spec: LinearSpring) -> LinearSpringElement:
    """Build one declared linear spring."""
    return LinearSpringElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        point_a=cast(np.ndarray, row.point_a),
        body_b=cast(str, row.body_b),
        point_b=cast(np.ndarray, row.point_b),
        stiffness=spec.stiffness,
        free_length=spec.free_length,
        reference_length=spec.reference_length,
        preload=spec.preload or 0.0,
        force_curve=spec.force_curve,
    )


def _damper(row: ResolvedElement, spec: StaticDamper) -> StaticDamperElement:
    """Build one declared quasi-static damper."""
    return StaticDamperElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        point_a=cast(np.ndarray, row.point_a),
        body_b=cast(str, row.body_b),
        point_b=cast(np.ndarray, row.point_b),
        gas_stiffness=spec.gas_stiffness,
        gas_reference_length=spec.gas_reference_length,
        gas_reference_force=spec.gas_reference_force,
        preload=spec.preload,
        friction=spec.friction,
        viscous_damping=spec.viscous_damping,
        force_curve=spec.force_curve,
    )


def _bump_stop(row: ResolvedElement, spec: BumpStop) -> BumpStopElement:
    """Build one declared bump stop."""
    return BumpStopElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        point_a=cast(np.ndarray, row.point_a),
        body_b=cast(str, row.body_b),
        point_b=cast(np.ndarray, row.point_b),
        clearance=spec.clearance,
        stiffness=spec.stiffness,
        direction=spec.direction,
        force_curve=spec.force_curve,
    )


def _anti_roll_bar(row: ResolvedElement, spec: AntiRollBar) -> AntiRollBarElement:
    """Build one declared anti-roll bar."""
    return AntiRollBarElement(
        name=row.name,
        left_body=cast(str, row.body_a),
        left_point=cast(np.ndarray, row.point_a),
        right_body=cast(str, row.body_b),
        right_point=cast(np.ndarray, row.point_b),
        stiffness=spec.torsional_stiffness,
    )


def _tire(row: ResolvedElement, spec: VerticalTire) -> VerticalTireElement:
    """Build one declared vertical tire."""
    return VerticalTireElement(
        name=row.name,
        wheel_body=cast(str, row.body_a),
        wheel_center_local=cast(np.ndarray, row.point_a),
        stiffness=spec.stiffness,
        unloaded_radius=spec.unloaded_radius,
    )


def _bushing(row: ResolvedElement) -> BushingElement:
    """
    Build one declared bushing.

    A `spec` of `None` means a C-mode slot placeholder: the original build writes
    those with a zero stiffness and an identity rotation, and they stay that way
    until subtask 05 gives them the template's default properties.
    """
    if row.spec is None:
        return BushingElement(
            name=row.name,
            body_a=cast(str, row.body_a),
            body_b=cast(str, row.body_b),
            local_pose_a=cast(SE3, row.local_pose_a),
            local_pose_b=cast(SE3, row.local_pose_b),
            stiffness=np.zeros((6, 6)),
        )
    spec = cast(Bushing6x6, row.spec)
    return BushingElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        body_b=cast(str, row.body_b),
        local_pose_a=cast(SE3, row.local_pose_a),
        local_pose_b=cast(SE3, row.local_pose_b),
        stiffness=np.asarray(spec.stiffness, dtype=float),
        damping=np.diag(np.asarray(spec.damping, dtype=float)),
        preload=np.asarray(spec.preload, dtype=float),
        force_curves=spec.force_curves,
        force_curve_interpolation=spec.force_curve_interpolation,
        rotation_coordinates=spec.rotation_coordinates,
    )


def _build_element(row: ResolvedElement) -> object:
    """
    Build one declared elastic element.

    A subsystem decides what exists and where, and this module constructs it,
    because it is the registered `elements` importer.
    """
    if row.kind == "spring":
        return _spring(row, cast(LinearSpring, row.spec))
    if row.kind == "damper":
        return _damper(row, cast(StaticDamper, row.spec))
    if row.kind == "bump_stop":
        return _bump_stop(row, cast(BumpStop, row.spec))
    if row.kind == "anti_roll_bar":
        return _anti_roll_bar(row, cast(AntiRollBar, row.spec))
    if row.kind == "tire":
        return _tire(row, cast(VerticalTire, row.spec))
    if row.kind == "bushing":
        return _bushing(row)
    raise ValueError(f"unsupported element kind {row.kind!r}")


def _element_rows(
    model: FrontAxleModel,
    mode: Literal["K", "C"],
    context: SubsystemContext,
    placeholders: Iterable[ResolvedElement],
) -> list[ResolvedElement]:
    """
    Collect the symmetric-proxy element declarations in their recorded order.

    The order is the contract: per side springs, dampers, tires, stops; then the
    anti-roll bar once; then the user's C-mode bushings; then the C-mode slot
    placeholders, which the original build appends last.  Each slot is filled by
    the subsystem that owns that content; the slot positions are fixed here.
    """
    rows: list[ResolvedElement] = []
    for side in ("L", "R"):
        for kind in ("spring", "damper"):
            rows.extend(suspension_subsystem.elements(context, side, kind))
        rows.extend(wheel_subsystem.tires(context, side))
        rows.extend(suspension_subsystem.elements(context, side, "bump_stop"))
    rows.extend(suspension_subsystem.global_elements(context))
    if mode == "C":
        for side in ("L", "R"):
            rows.extend(suspension_subsystem.compliance_elements(context, side))
        rows.extend(placeholders)
    return rows


def build_front_axle(
    model: FrontAxleModel,
    mode: Literal["K", "C"] | None = None,
    request: AssemblyRequest | None = None,
) -> FrontAxleAssembly:
    """
    Build a symmetric front axle from one left-side model definition.

    `mode` and `request` say the same thing twice, so either alone is enough and
    passing both is only accepted when they agree:

    * `build_front_axle(model, "C")` -- the historical call, unchanged;
    * `build_front_axle(model, request=AssemblyRequest(mode="C", subsystems=...))`
      -- the requirement-15 call, where the subsystem set is the point;
    * both, with different modes -- refused, because there is no way to tell which
      one the caller meant.

    Omitting `request` asks for the subsystem set the axle has always carried, so
    no existing caller changes behaviour: requirement 15 adds a way to leave
    steering out, it does not change what the default is.
    """
    if mode is not None and mode not in ("K", "C"):
        raise ValueError("mode must be K or C")
    if request is None:
        request = AssemblyRequest(mode=mode or "K")
    elif mode is not None and request.mode != mode:
        raise ValueError(
            f"mode {mode!r} disagrees with request.mode {request.mode!r}; "
            "pass one or the other, or make them agree"
        )
    mode = request.mode
    if model.topology == "explicit":
        return _build_explicit_axle(model, mode)
    if not request.carries("chassis"):
        raise ValueError("an axle assembly must carry the chassis subsystem")
    if not request.carries("suspension"):
        raise ValueError("an axle assembly must carry the suspension subsystem")
    # A single-axle assembly has no brake and no drive (requirement 17 / D8).
    # The axle cannot build them, so claiming them in `capabilities` would be a
    # lie a rig would then plan against; refuse the request instead.
    for role in ("brake", "drive"):
        if request.carries(role):
            raise ValueError(
                f"single-axle assemblies do not carry the {role} subsystem "
                "(requirement 17); it belongs to the full-vehicle assembly"
            )

    context = SubsystemContext(
        model=model,
        request=request,
        hardpoints=dict(model.hardpoints),
        side_schema={
            side: side_hardpoints(model.hardpoints, side) for side in ("L", "R")
        },
    )
    chassis = chassis_subsystem.build(context)
    context.bodies.update(chassis.bodies)
    # The per-side rigid bodies have to exist before the steering subsystem can
    # resolve its tie rod points against the uprights.
    side_bodies: dict[str, dict[str, RigidBody]] = {}
    for side in ("L", "R"):
        side_bodies[side] = suspension_subsystem.side_bodies(context, side)
        context.bodies.update(side_bodies[side])
    steering = steering_subsystem.bodies(context)

    # The *recorded* body order is not the order the parts are computed in: the
    # contract lists chassis, rack, then per side arm/arm/upright/tie rod, so the
    # bodies are ordered here and `_with_body_specs` keeps that order.  Without a
    # steering subsystem the rack and the tie rods are absent, not degenerate:
    # requirement 15 wants them gone, and a floating rack body would poison the
    # rig's capability check.
    ordered_bodies: dict[str, RigidBody] = dict(chassis.bodies)
    if "rack" in steering:
        ordered_bodies["rack"] = steering["rack"]
    for side in ("L", "R"):
        ordered_bodies.update(side_bodies[side])
        tie = f"tie_rod_{side}"
        if tie in steering:
            ordered_bodies[tie] = steering[tie]
    context.bodies = ordered_bodies

    connections: list[Connection] = list(chassis.connections)
    constraints: list[Constraint] = list(chassis.constraints)
    ideal_constraints: list[Constraint] = list(chassis.ideal_constraints)
    placeholders: list[ResolvedElement] = []

    for side in ("L", "R"):
        for name, point in side_hardpoints(model.hardpoints, side).items():
            values = point.as_array()
            context.hardpoints[f"{name}__{side}"] = Vec3(
                x=values[0], y=values[1], z=values[2]
            )
        for output in (
            suspension_subsystem.side_content(context, side),
            steering_subsystem.side_content(context, side),
        ):
            context.points.update(output.points)
            connections.extend(output.connections)
            constraints.extend(output.constraints)
            ideal_constraints.extend(output.ideal_constraints)
            placeholders.extend(
                cast(Iterable[ResolvedElement], output.bushings)
            )

    guide = steering_subsystem.guide(context)
    context.points.update(guide.points)
    context.hardpoints.update(cast(dict[str, Vec3], guide.hardpoints))
    connections.extend(guide.connections)
    constraints.extend(guide.constraints)
    ideal_constraints.extend(guide.ideal_constraints)

    rows = _element_rows(model, mode, context, placeholders)
    elements = [_build_element(row) for row in rows]
    bodies = _with_body_specs(context.bodies, model)
    explicit_bushings = tuple(
        element for element in elements if isinstance(element, BushingElement)
    )
    return FrontAxleAssembly(
        mode=mode,
        bodies=bodies,
        state=RigidBodyState(bodies),
        points=context.points,
        hardpoints=context.hardpoints,
        connections=tuple(connections),
        constraints=tuple(constraints),
        ideal_constraints=tuple(ideal_constraints),
        bushings=explicit_bushings if mode == "C" else (),
        elements=tuple(elements),
        capabilities=capabilities_for(
            subsystems=frozenset(request.subsystems),
            body_names=frozenset(bodies),
        ),
    )
