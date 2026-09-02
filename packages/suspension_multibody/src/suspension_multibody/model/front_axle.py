"""Symmetric front double-wishbone and rack steering assembly."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

import numpy as np

from ..core import (
    SE3,
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
from ..elements import (
    AntiRollBarElement,
    BumpStopElement,
    BushingElement,
    LinearSpringElement,
    StaticDamperElement,
    VerticalTireElement,
)
from ..schema import (
    FrontAxleModel,
    IdealJointSpec,
    Pose,
    Vec3,
)


def _array(point: Vec3 | np.ndarray | list[float]) -> np.ndarray:
    if isinstance(point, Vec3):
        return np.asarray(point.as_tuple(), dtype=float)
    return np.asarray(point, dtype=float)


def mirror_hardpoints(hardpoints: dict[str, Vec3]) -> dict[str, Vec3]:
    """Return left hardpoints plus deterministic ``__R`` mirrored copies."""
    mirrored = dict(hardpoints)
    for name, point in hardpoints.items():
        mirrored[f"{name}__R"] = point.mirrored_y()
    return mirrored


def side_hardpoints(
    hardpoints: dict[str, Vec3], side: Literal["L", "R"]
) -> dict[str, Vec3]:
    """Return one side's hardpoints with a common, side-independent key set."""
    if side == "L":
        return dict(hardpoints)
    return {name: point.mirrored_y() for name, point in hardpoints.items()}


@dataclass(frozen=True)
class Connection:
    """Stable physical connection identifier."""

    name: str
    kind: Literal["ideal", "bushing"]
    body_a: str
    body_b: str
    point_a: str
    point_b: str


@dataclass(frozen=True)
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


_ALIASES: dict[str, tuple[str, ...]] = {
    "upper_front": (
        "UPPER_INBOARD_FRONT",
        "UPPER_INNER_FRONT",
        "UCA_FRONT",
        "UCA_INNER_FRONT",
    ),
    "upper_rear": (
        "UPPER_INBOARD_REAR",
        "UPPER_INNER_REAR",
        "UCA_REAR",
        "UCA_INNER_REAR",
    ),
    "upper_outer": ("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER"),
    "lower_front": (
        "LOWER_INBOARD_FRONT",
        "LOWER_INNER_FRONT",
        "LCA_FRONT",
        "LCA_INNER_FRONT",
    ),
    "lower_rear": (
        "LOWER_INBOARD_REAR",
        "LOWER_INNER_REAR",
        "LCA_REAR",
        "LCA_INNER_REAR",
    ),
    "lower_outer": ("LOWER_OUTBOARD", "LOWER_OUTER", "LCA_OUTER"),
    "tie_inner": ("TIE_ROD_INBOARD", "TIE_ROD_INNER", "TIEROD_INNER", "RACK_TIE_ROD"),
    "tie_outer": ("TIE_ROD_OUTBOARD", "TIE_ROD_OUTER", "TIEROD_OUTER"),
    "wheel_center": ("WHEEL_CENTER", "WHEEL_CENTRE", "WHEEL_CG"),
    "rack_center": ("RACK_CENTER", "RACK_CENTRE", "RACK_REFERENCE"),
}


def _lookup(hardpoints: dict[str, Vec3], role: str) -> Vec3:
    normalized = {
        key.upper().replace("-", "_"): value for key, value in hardpoints.items()
    }
    for alias in _ALIASES[role]:
        if alias in normalized:
            return normalized[alias]
    raise ValueError(f"missing required front-axle hardpoint for {role}")


def _body_with_points(name: str, points: dict[str, np.ndarray]) -> RigidBody:
    del points
    return RigidBody(name=name, pose=SE3.identity())


def _with_body_specs(
    bodies: dict[str, RigidBody], model: FrontAxleModel
) -> dict[str, RigidBody]:
    updated = dict(bodies)
    for spec in model.bodies:
        if spec.name not in updated:
            raise ValueError(f"unknown runtime body in body spec {spec.name!r}")
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


def _body_from_spec(name: str, spec: object) -> RigidBody:
    """Create a body using the schema reference frame and mass properties."""
    pose_spec = getattr(spec, "pose")
    return RigidBody(
        name=name,
        pose=SE3(
            pose_spec.translation.as_array(),
            np.asarray(pose_spec.rotation.as_tuple(), dtype=float),
        ),
        mass=float(getattr(spec, "mass")),
        inertia=np.asarray(getattr(spec, "inertia"), dtype=float),
        center_of_mass=getattr(spec, "center_of_mass").as_array(),
        fixed=bool(getattr(spec, "fixed")),
    )


def _resolve_body(
    name: str, side: Literal["L", "R"], bodies: dict[str, RigidBody]
) -> str:
    """Resolve a schema body name to a generated side-specific body."""
    if name in bodies:
        return name
    normalized = name.strip().lower().replace("-", "_")
    aliases = {
        "uca": "upper_arm",
        "upper": "upper_arm",
        "lca": "lower_arm",
        "lower": "lower_arm",
        "wheel": "upright",
        "knuckle": "upright",
        "spindle": "upright",
        "tie": "tie_rod",
        "tierod": "tie_rod",
    }
    base = aliases.get(normalized, normalized)
    candidate = f"{base}_{side}"
    if candidate in bodies:
        return candidate
    for suffix in ("_l", "_r", " left", " right"):
        if normalized.endswith(suffix):
            stem = normalized[: -len(suffix)].rstrip()
            candidate = f"{aliases.get(stem, stem)}_{side}"
            if candidate in bodies:
                return candidate
    raise ValueError(f"unknown force-element body {name!r}")


def _mirror_point(value: Vec3, side: Literal["L", "R"]) -> np.ndarray:
    point = value.mirrored_y() if side == "R" else value
    return point.as_array()


def _pose(value: Pose, side: Literal["L", "R"]) -> SE3:
    translation = _mirror_point(value.translation, side)
    return SE3(translation, np.asarray(value.rotation.as_tuple(), dtype=float))


def _local_point(
    bodies: dict[str, RigidBody], body: str, point_global: np.ndarray
) -> np.ndarray:
    """Convert an imported global hardpoint into a body-local point."""
    return bodies[body].pose.inverse().transform_point(point_global)


def _local_pose(
    value: Pose,
    side: Literal["L", "R"],
    body: str,
    bodies: dict[str, RigidBody],
) -> SE3:
    """Convert a schema attachment pose from vehicle to body coordinates."""
    global_translation = _mirror_point(value.translation, side)
    return SE3(
        _local_point(bodies, body, global_translation),
        np.asarray(value.rotation.as_tuple(), dtype=float),
    )


def _local_pose_explicit(
    value: Pose, body: str, bodies: dict[str, RigidBody]
) -> SE3:
    """Convert an explicit global attachment frame to a body-local frame."""
    global_pose = SE3(
        value.translation.as_array(),
        np.asarray(value.rotation.as_tuple(), dtype=float),
    )
    return bodies[body].pose.inverse().compose(global_pose)


def _explicit_constraint(
    spec: IdealJointSpec, bodies: dict[str, RigidBody]
) -> Constraint:
    """Create one core constraint from an explicit vehicle-frame joint spec."""
    point_a = _local_point(bodies, spec.body_a, spec.point_a.as_array())
    point_b = _local_point(bodies, spec.body_b, spec.point_b.as_array())
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
        return BallJoint(**common)
    if spec.kind == "fixed":
        return WeldJoint(**common)
    if spec.kind == "revolute":
        return RevoluteJoint(**common, axis_a=axis_a, axis_b=axis_b)
    if spec.kind == "prismatic":
        return PrismaticJoint(**common, axis_a=axis_a, axis_b=axis_b)
    if spec.kind == "universal":
        return UniversalJoint(**common, axis_a=axis_a, axis_b=axis_b)
    if spec.kind == "constant_velocity":
        secondary_a = bodies[spec.body_a].pose.rotation.T @ spec.axis_a_secondary.as_array()
        secondary_b = bodies[spec.body_b].pose.rotation.T @ spec.axis_b_secondary.as_array()
        return ConstantVelocityJoint(
            **common,
            axis_a=axis_a,
            axis_a_secondary=secondary_a,
            axis_b=axis_b,
            axis_b_secondary=secondary_b,
            angle_target=spec.constant_velocity_angle_target,
        )
    if spec.kind == "cylindrical":
        return CylindricalJoint(**common, axis_a=axis_a, axis_b=axis_b)
    if spec.kind == "inplane":
        return InPlaneJoint(**common, axis_a=axis_a)
    raise ValueError(f"unsupported explicit ideal joint kind {spec.kind!r}")


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
                point_a=_local_point(bodies, spec.body_a, spec.point_a.as_array()),
                body_b=spec.body_b,
                point_b=_local_point(bodies, spec.body_b, spec.point_b.as_array()),
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
                point_a=_local_point(bodies, spec.body_a, spec.point_a.as_array()),
                body_b=spec.body_b,
                point_b=_local_point(bodies, spec.body_b, spec.point_b.as_array()),
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
                point_a=_local_point(bodies, spec.body_a, spec.point_a.as_array()),
                body_b=spec.body_b,
                point_b=_local_point(bodies, spec.body_b, spec.point_b.as_array()),
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
                    local_pose_a=_local_pose_explicit(spec.pose_a, spec.body_a, bodies),
                    local_pose_b=_local_pose_explicit(spec.pose_b, spec.body_b, bodies),
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
        bodies[name] = _body_from_spec(name, spec)
    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Constraint] = []
    connections: list[Connection] = []
    for spec in model.joints:
        if spec.body_a not in bodies or spec.body_b not in bodies:
            raise ValueError(
                f"explicit joint {spec.name!r} references an unknown body"
            )
        point_a = _local_point(bodies, spec.body_a, spec.point_a.as_array())
        point_b = _local_point(bodies, spec.body_b, spec.point_b.as_array())
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
                global_point = _lookup(model.hardpoints, "wheel_center").as_array()
            else:
                right_hardpoints = {
                    key.removesuffix("__R"): value
                    for key, value in model.hardpoints.items()
                    if key.endswith("__R")
                }
                if right_hardpoints:
                    global_point = _lookup(right_hardpoints, "wheel_center").as_array()
                else:
                    global_point = _lookup(model.hardpoints, "wheel_center").mirrored_y().as_array()
        except ValueError:
            continue
        points[(upright, "wheel_center")] = _local_point(
            bodies, upright, global_point
        )

    rack = "rack"
    if rack in bodies:
        rack_center = _lookup(model.hardpoints, "rack_center").as_array()
        points[(rack, "center")] = _local_point(bodies, rack, rack_center)
        points[("chassis", "rack_center")] = rack_center.copy()
        if model.rack_fixed_to_chassis:
            rack_joint = WeldJoint(
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
    )


def _runtime_elements(
    model: FrontAxleModel,
    mode: Literal["K", "C"],
    bodies: dict[str, RigidBody],
) -> tuple[object, ...]:
    """Convert validated schema force elements into runtime evaluators."""
    if model.topology == "explicit":
        return _runtime_elements_explicit(model, mode, bodies)
    elements: list[object] = []
    for side in ("L", "R"):
        for spec in model.springs:
            body_a = _resolve_body(spec.body_a, side, bodies)
            body_b = _resolve_body(spec.body_b, side, bodies)
            elements.append(
                LinearSpringElement(
                    name=f"{spec.name}_{side}",
                    body_a=body_a,
                    point_a=_local_point(bodies, body_a, _mirror_point(spec.point_a, side)),
                    body_b=body_b,
                    point_b=_local_point(bodies, body_b, _mirror_point(spec.point_b, side)),
                    stiffness=spec.stiffness,
                    free_length=spec.free_length,
                    reference_length=spec.reference_length,
                    preload=spec.preload or 0.0,
                    force_curve=spec.force_curve,
                )
            )
        for spec in model.dampers:
            body_a = _resolve_body(spec.body_a, side, bodies)
            body_b = _resolve_body(spec.body_b, side, bodies)
            elements.append(
                StaticDamperElement(
                    name=f"{spec.name}_{side}",
                    body_a=body_a,
                    point_a=_local_point(bodies, body_a, _mirror_point(spec.point_a, side)),
                    body_b=body_b,
                    point_b=_local_point(bodies, body_b, _mirror_point(spec.point_b, side)),
                    gas_stiffness=spec.gas_stiffness,
                    gas_reference_length=spec.gas_reference_length,
                    gas_reference_force=spec.gas_reference_force,
                    preload=spec.preload,
                    friction=spec.friction,
                    viscous_damping=spec.viscous_damping,
                    force_curve=spec.force_curve,
                )
            )
        for spec in model.tires:
            elements.append(
                VerticalTireElement(
                    name=f"tire_{side}",
                    wheel_body=f"upright_{side}",
                    wheel_center_local=_local_point(
                        bodies,
                        f"upright_{side}",
                        _lookup(side_hardpoints(model.hardpoints, side), "wheel_center").as_array(),
                    ),
                    stiffness=spec.stiffness,
                    unloaded_radius=spec.unloaded_radius,
                )
            )
        for spec in model.stops:
            body_a = _resolve_body(spec.body_a, side, bodies)
            body_b = _resolve_body(spec.body_b, side, bodies)
            elements.append(
                BumpStopElement(
                    name=f"{spec.name}_{side}",
                    body_a=body_a,
                    point_a=_local_point(bodies, body_a, _mirror_point(spec.point_a, side)),
                    body_b=body_b,
                    point_b=_local_point(bodies, body_b, _mirror_point(spec.point_b, side)),
                    clearance=spec.clearance,
                    stiffness=spec.stiffness,
                    direction=spec.direction,
                    force_curve=spec.force_curve,
                )
            )
    for spec in model.anti_roll_bars:
        elements.append(
            AntiRollBarElement(
                name=spec.name,
                left_body="upright_L",
                left_point=_local_point(bodies, "upright_L", spec.left_link_point.as_array()),
                right_body="upright_R",
                right_point=_local_point(bodies, "upright_R", spec.right_link_point.as_array()),
                stiffness=spec.torsional_stiffness,
            )
        )
    if mode == "C":
        for spec in model.bushings:
            for side in ("L", "R"):
                body_a = _resolve_body(spec.body_a, side, bodies)
                body_b = _resolve_body(spec.body_b, side, bodies)
                elements.append(
                    BushingElement(
                        name=f"{spec.name}_{side}",
                        body_a=body_a,
                        body_b=body_b,
                        local_pose_a=_local_pose(spec.pose_a, side, body_a, bodies),
                        local_pose_b=_local_pose(spec.pose_b, side, body_b, bodies),
                        stiffness=np.asarray(spec.stiffness, dtype=float),
                        damping=np.diag(np.asarray(spec.damping, dtype=float)),
                        preload=np.asarray(spec.preload, dtype=float),
                        force_curves=spec.force_curves,
                        force_curve_interpolation=spec.force_curve_interpolation,
                        rotation_coordinates=spec.rotation_coordinates,
                    )
                )
    return tuple(elements)


def build_front_axle(
    model: FrontAxleModel, mode: Literal["K", "C"] = "K"
) -> FrontAxleAssembly:
    """Build a symmetric front axle from one left-side model definition."""
    if mode not in ("K", "C"):
        raise ValueError("mode must be K or C")
    if model.topology == "explicit":
        return _build_explicit_axle(model, mode)
    left = {
        name: _array(point)
        for name, point in side_hardpoints(model.hardpoints, "L").items()
    }
    right = {
        name: _array(point)
        for name, point in side_hardpoints(model.hardpoints, "R").items()
    }
    points: dict[tuple[str, str], np.ndarray] = {}
    body_specs = {spec.name: spec for spec in model.bodies}
    bodies: dict[str, RigidBody] = {
        "chassis": RigidBody("chassis", fixed=True),
        "rack": _body_from_spec("rack", body_specs["rack"])
        if "rack" in body_specs
        else RigidBody("rack"),
    }
    connections: list[Connection] = []
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    bushings: list[BushingElement] = []
    all_hardpoints = dict(model.hardpoints)

    for side, side_points in (("L", left), ("R", right)):
        uca = f"upper_arm_{side}"
        lca = f"lower_arm_{side}"
        upright = f"upright_{side}"
        tie = f"tie_rod_{side}"
        for body in (uca, lca, upright, tie):
            bodies[body] = (
                _body_from_spec(body, body_specs[body])
                if body in body_specs
                else _body_with_points(body, {})
            )
        for name, point in side_points.items():
            all_hardpoints[f"{name}__{side}"] = Vec3(x=point[0], y=point[1], z=point[2])
        mount_data = (
            (uca, "inner_front", "upper_front", "upper_front"),
            (uca, "inner_rear", "upper_rear", "upper_rear"),
            (uca, "outer", "upper_outer", "upper_outer"),
            (lca, "inner_front", "lower_front", "lower_front"),
            (lca, "inner_rear", "lower_rear", "lower_rear"),
            (lca, "outer", "lower_outer", "lower_outer"),
            (tie, "inner", "tie_inner", "tie_inner"),
            (tie, "outer", "tie_outer", "tie_outer"),
            (upright, "wheel_center", "wheel_center", "wheel_center"),
        )
        side_schema = {
            key: Vec3(x=value[0], y=value[1], z=value[2])
            for key, value in side_points.items()
        }
        for body, label, role, _ in mount_data:
            global_point = _lookup(side_schema, role).as_array()
            points[(body, label)] = _local_point(bodies, body, global_point)
        for label, role in (
            ("inner_front", "upper_front"),
            ("inner_rear", "upper_rear"),
        ):
            global_point = _lookup(side_schema, role).as_array()
            p = _local_point(bodies, uca, global_point)
            chassis_label = f"uca_{side}_{label}"
            points[("chassis", chassis_label)] = global_point.copy()
            if mode == "K":
                if label == "inner_front":
                    axis_global = (
                        _lookup(side_schema, "upper_rear").as_array()
                        - global_point
                    )
                    axis_norm = np.linalg.norm(axis_global)
                    if axis_norm <= 1e-12:
                        raise ValueError(
                            f"upper arm {side} inboard hardpoints must be distinct"
                        )
                    axis_global /= axis_norm
                    joint = RevoluteJoint(
                        "chassis",
                        global_point,
                        bodies["chassis"].pose.rotation.T @ axis_global,
                        uca,
                        p,
                        bodies[uca].pose.rotation.T @ axis_global,
                        name=f"uca_mount_{side}_inner_front",
                    )
                    constraints.append(joint)
                    ideal_constraints.append(joint)
                kind: Literal["ideal", "bushing"] = "ideal"
            else:
                ideal_constraints.append(
                    BallJoint(
                        "chassis", global_point, uca, p, name=f"uca_mount_{side}_{label}"
                    )
                )
                bushings.append(
                    BushingElement(
                        f"uca_bushing_{side}_{label}",
                        "chassis",
                        uca,
                        local_pose_a=SE3(
                            translation=global_point,
                            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                        ),
                        local_pose_b=SE3(
                            translation=p,
                            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                        ),
                        stiffness=np.zeros((6, 6)),
                    )
                )
                kind = "bushing"
            connections.append(
                Connection(
                    f"uca_mount_{side}_{label}",
                    kind,
                    "chassis",
                    uca,
                    chassis_label,
                    label,
                )
            )
        for label, role in (
            ("inner_front", "lower_front"),
            ("inner_rear", "lower_rear"),
        ):
            global_point = _lookup(side_schema, role).as_array()
            p = _local_point(bodies, lca, global_point)
            chassis_label = f"lca_{side}_{label}"
            points[("chassis", chassis_label)] = global_point.copy()
            if mode == "K":
                if label == "inner_front":
                    axis_global = (
                        _lookup(side_schema, "lower_rear").as_array()
                        - global_point
                    )
                    axis_norm = np.linalg.norm(axis_global)
                    if axis_norm <= 1e-12:
                        raise ValueError(
                            f"lower arm {side} inboard hardpoints must be distinct"
                        )
                    axis_global /= axis_norm
                    joint = RevoluteJoint(
                        "chassis",
                        global_point,
                        bodies["chassis"].pose.rotation.T @ axis_global,
                        lca,
                        p,
                        bodies[lca].pose.rotation.T @ axis_global,
                        name=f"lca_mount_{side}_inner_front",
                    )
                    constraints.append(joint)
                    ideal_constraints.append(joint)
                kind = "ideal"
            else:
                ideal_constraints.append(
                    BallJoint(
                        "chassis", global_point, lca, p, name=f"lca_mount_{side}_{label}"
                    )
                )
                bushings.append(
                    BushingElement(
                        f"lca_bushing_{side}_{label}",
                        "chassis",
                        lca,
                        local_pose_a=SE3(
                            translation=global_point,
                            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                        ),
                        local_pose_b=SE3(
                            translation=p,
                            quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                        ),
                        stiffness=np.zeros((6, 6)),
                    )
                )
                kind = "bushing"
            connections.append(
                Connection(
                    f"lca_mount_{side}_{label}",
                    kind,
                    "chassis",
                    lca,
                    chassis_label,
                    label,
                )
        )
        for arm, outer_label in ((uca, "outer"), (lca, "outer")):
            role = "upper_outer" if arm == uca else "lower_outer"
            global_point = _lookup(side_schema, role).as_array()
            p = _local_point(bodies, arm, global_point)
            upright_label = f"{arm}_outer"
            upright_point = _local_point(bodies, upright, global_point)
            points[(upright, upright_label)] = upright_point.copy()
            joint = BallJoint(
                arm, p, upright, upright_point, name=f"{arm}_outer_joint"
            )
            constraints.append(joint)
            ideal_constraints.append(joint)
            connections.append(
                Connection(
                    f"{arm}_outer_joint",
                    "ideal",
                    arm,
                    upright,
                    outer_label,
                    upright_label,
                )
            )
        tie_inner_global = _lookup(side_schema, "tie_inner").as_array()
        tie_outer_global = _lookup(side_schema, "tie_outer").as_array()
        tie_inner = _local_point(bodies, tie, tie_inner_global)
        tie_outer = _local_point(bodies, tie, tie_outer_global)
        rack_tie_inner = _local_point(bodies, "rack", tie_inner_global)
        upright_tie_outer = _local_point(bodies, upright, tie_outer_global)
        points["rack", f"tie_{side}"] = rack_tie_inner.copy()
        points[(upright, "tie_outer")] = upright_tie_outer.copy()
        rack_joint = BallJoint(
            "rack", rack_tie_inner, tie, tie_inner, name=f"rack_tie_joint_{side}"
        )
        tie_joint = BallJoint(
            tie, tie_outer, upright, upright_tie_outer, name=f"tie_upright_joint_{side}"
        )
        constraints.extend((rack_joint, tie_joint))
        ideal_constraints.extend((rack_joint, tie_joint))
        connections.extend(
            (
                Connection(
                    f"rack_tie_joint_{side}",
                    "ideal",
                    "rack",
                    tie,
                    f"tie_{side}",
                    "inner",
                ),
                Connection(
                    f"tie_upright_joint_{side}",
                    "ideal",
                    tie,
                    upright,
                    "outer",
                    "tie_outer",
                ),
            )
        )
    rack_point = _lookup(
        {key: Vec3(x=value[0], y=value[1], z=value[2]) for key, value in left.items()},
        "rack_center",
    ).as_array()
    rack_point_local = _local_point(bodies, "rack", rack_point)
    points[("rack", "center")] = rack_point_local
    points[("chassis", "rack_center")] = rack_point.copy()
    if model.rack_fixed_to_chassis:
        rack_guide = WeldJoint(
            "chassis",
            rack_point,
            "rack",
            rack_point_local,
            name="rack_fixed_to_chassis",
        )
    elif "rack_housing" not in bodies:
        # The rack is a guided rigid body: its only ideal degree of freedom is
        # translation along the declared vehicle Y rack axis.
        rack_guide = PrismaticJoint(
            "chassis",
            rack_point,
            np.asarray(model.rack_axis.as_tuple(), dtype=float),
            "rack",
            rack_point_local,
            np.asarray(model.rack_axis.as_tuple(), dtype=float),
            name="rack_guide",
        )
    if "rack_housing" not in bodies:
        constraints.append(rack_guide)
        ideal_constraints.append(rack_guide)
    all_hardpoints["RACK_CENTER"] = Vec3(
        x=rack_point[0], y=rack_point[1], z=rack_point[2]
    )
    bodies = _with_body_specs(bodies, model)
    state = RigidBodyState(bodies)
    runtime_elements = list(_runtime_elements(model, mode, bodies))
    if mode == "C":
        runtime_elements.extend(bushings)
    explicit_bushings = tuple(
        element for element in runtime_elements if isinstance(element, BushingElement)
    )
    return FrontAxleAssembly(
        mode=mode,
        bodies=bodies,
        state=state,
        points=points,
        hardpoints=all_hardpoints,
        connections=tuple(connections),
        constraints=tuple(constraints),
        ideal_constraints=tuple(ideal_constraints),
        bushings=explicit_bushings if mode == "C" else (),
        elements=tuple(runtime_elements),
    )
