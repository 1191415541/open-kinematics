"""
The mechanisms a vehicle runtime is built from.

Merging two composed axles under one chassis, attaching the wheel ends, and the
renaming that makes the result readable as a whole vehicle.  Welded bodies stay
separate by default: each weld reaches the kernel as a ``kind="fixed"`` joint, six
rows of point coincidence plus full relative rotation.
``SUSPENSION_MULTIBODY_CONDENSE_WELDS=1`` restores the older fused form.

This module holds *mechanisms* and no entry point of its own.
``subsystems/vehicle_assembly.py`` composes the axles and calls these.  The two used
to be one module whose ``compose_vehicle`` was a hand-written path beside the
composition; that path is gone, and these are the parts of it worth keeping.

The functions here name :class:`VehicleRuntime` in their signatures only, so the
import is under ``TYPE_CHECKING``: ``vehicle_assembly`` imports *this* module, and a
real import back would be a cycle.
"""

from __future__ import annotations

import os
from dataclasses import fields, replace
from typing import TYPE_CHECKING, Any, Mapping

import numpy as np

from ..modeling.primitives import (
    SE3,
    BumpStopElement,
    BushingElement,
    Constraint,
    LinearSpringElement,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
    StaticDamperElement,
    WeldJoint,
)
from ..schema import RigidBodySpec, WheelSpec
from .geometry import local_point
from .types import Connection

if TYPE_CHECKING:  # pragma: no cover - annotations only, keeps the import acyclic
    from .vehicle_assembly import VehicleRuntime

__all__ = [
    "_add_wheel",
    "_body_from_spec",
    "_condense_welded_bodies",
    "_condense_wheel_end",
    "_drop_isolated_bodies",
    "_fuse_welded_bodies",
    "_merge_fixed_wheel",
    "_parallel_axis_inertia",
    "_rename_connections",
    "_rename_dataclasses",
    "_wheel_inertia",
]


def _body_from_spec(spec: RigidBodySpec) -> RigidBody:
    """Convert a schema rigid-body spec into the runtime body."""
    body = spec  # keep this helper narrow so the schema remains immutable
    return RigidBody(
        name=body.name,
        pose=SE3(
            body.pose.translation.as_array(),
            np.asarray(body.pose.rotation.as_tuple(), dtype=float),
        ),
        mass=body.mass,
        inertia=np.asarray(body.inertia, dtype=float),
        center_of_mass=body.center_of_mass.as_array(),
        fixed=body.fixed,
    )


def _rename_dataclasses(
    values: tuple[Any, ...], body_map: dict[str, str], prefix: str
) -> tuple[Any, ...]:
    renamed: list[Any] = []
    body_fields = {"body", "body_a", "body_b", "wheel_body", "left_body", "right_body"}
    for value in values:
        updates: dict[str, object] = {}
        # Renamed from `field`: the module imports `field` from `dataclasses`,
        # and shadowing it here made the import look unused.
        for dataclass_field in fields(value):
            if dataclass_field.name in body_fields:
                updates[dataclass_field.name] = body_map[
                    getattr(value, dataclass_field.name)
                ]
            elif dataclass_field.name == "name":
                updates[dataclass_field.name] = (
                    f"{prefix}{getattr(value, dataclass_field.name)}"
                )
        renamed.append(replace(value, **updates))
    return tuple(renamed)


def _rename_connections(
    values: tuple[Connection, ...], body_map: dict[str, str], prefix: str
) -> tuple[Connection, ...]:
    return tuple(
        replace(
            value,
            name=f"{prefix}{value.name}",
            body_a=body_map[value.body_a],
            body_b=body_map[value.body_b],
        )
        for value in values
    )


def _condense_welded_bodies(assembly: VehicleRuntime) -> VehicleRuntime:
    """
    Return the assembly unchanged: the kernel's ``fixed`` joint carries a weld.

    Condensation used to fuse welded bodies here so the kernel never saw them.
    The kernel has its own ``fixed`` kind for exactly this constraint -- six rows,
    the coincident point plus the full relative rotation -- and taking it means
    the authoring layer no longer answers a *solving* question (which bodies are
    really one).  Bodies stay separate and the weld is sent as a constraint.

    ``SUSPENSION_MULTIBODY_CONDENSE_WELDS=1`` restores the fused form.  It is kept
    because the fusion is still a correct statement of the same physics and the
    two agree in the world frame -- same total mass, same centre of mass -- while
    the fused form is the smaller body set an external caller may still want.
    """
    if os.environ.get("SUSPENSION_MULTIBODY_CONDENSE_WELDS") != "1":
        return assembly
    return _fuse_welded_bodies(assembly)


def _fuse_welded_bodies(assembly: VehicleRuntime) -> VehicleRuntime:
    """Exactly merge bodies connected by WeldJoint constraints."""
    welds = tuple(
        constraint
        for constraint in assembly.constraints
        if isinstance(constraint, WeldJoint)
    )
    if not welds:
        return assembly

    body_order = {name: index for index, name in enumerate(assembly.bodies)}
    parent = {name: name for name in assembly.bodies}

    def find(body: str) -> str:
        root = parent[body]
        while parent[root] != root:
            root = parent[root]
        while parent[body] != body:
            next_body = parent[body]
            parent[body] = root
            body = next_body
        return root

    def union(a: str, b: str) -> None:
        root_a = find(a)
        root_b = find(b)
        if root_a != root_b:
            if body_order[root_a] > body_order[root_b]:
                root_a, root_b = root_b, root_a
            parent[root_b] = root_a

    for weld in welds:
        union(weld.body_a, weld.body_b)

    components: dict[str, list[str]] = {}
    for body in assembly.bodies:
        components.setdefault(find(body), []).append(body)

    def component_root(component: list[str]) -> str:
        # A fused component is named after the vehicle's own chassis when it
        # carries it: the chassis is the body the assembly called the centre, and
        # naming the merged component after it keeps the alias map agreeing with
        # the body list the vehicle already had.
        if assembly.chassis_name in component:
            return assembly.chassis_name
        body_b_candidates = [
            weld.body_b
            for weld in welds
            if weld.body_a in component and weld.body_b in component
        ]
        if body_b_candidates:
            return min(body_b_candidates, key=lambda name: body_order[name])
        return min(component, key=lambda name: body_order[name])

    alias: dict[str, str] = {}
    for component in components.values():
        if len(component) <= 1:
            continue
        root = component_root(component)
        for body in component:
            if body != root:
                alias[body] = root
    if not alias:
        return assembly

    def mapped_body(body: str) -> str:
        return alias.get(body, body)

    def point_to_body(point: np.ndarray, source: str, target: str) -> np.ndarray:
        value = np.asarray(point, dtype=float)
        if source == target:
            return value.copy()
        source_pose = assembly.bodies[source].pose
        target_pose = assembly.bodies[target].pose
        return target_pose.inverse().transform_point(source_pose.transform_point(value))

    def axis_to_body(axis: np.ndarray, source: str, target: str) -> np.ndarray:
        value = np.asarray(axis, dtype=float)
        if source == target:
            return value.copy()
        source_rotation = assembly.bodies[source].pose.rotation
        target_rotation = assembly.bodies[target].pose.rotation
        return target_rotation.T @ source_rotation @ value

    def pose_to_body(pose: SE3, source: str, target: str) -> SE3:
        if source == target:
            return pose
        source_pose = assembly.bodies[source].pose
        target_pose = assembly.bodies[target].pose
        return target_pose.inverse().compose(source_pose.compose(pose))

    def fused_body(root: str, component: list[str]) -> RigidBody:
        root_body = assembly.bodies[root]
        root_pose = root_body.pose
        root_rotation = root_pose.rotation
        root_origin = root_pose.translation
        masses: list[float] = []
        centers: list[np.ndarray] = []
        inertias: list[np.ndarray] = []
        fixed = False
        for body_name in component:
            body = assembly.bodies[body_name]
            fixed = fixed or body.fixed
            mass = float(body.mass)
            body_rotation = body.pose.rotation
            center_world = body.pose.transform_point(body.center_of_mass)
            center_root = root_rotation.T @ (center_world - root_origin)
            inertia_root = (
                root_rotation.T @ body_rotation
                @ np.asarray(body.inertia, dtype=float)
                @ body_rotation.T @ root_rotation
            )
            masses.append(mass)
            centers.append(center_root)
            inertias.append(inertia_root)
        total_mass = float(sum(masses))
        if total_mass <= 0.0:
            return replace(root_body, mass=0.0, fixed=fixed)
        center = sum(
            mass * point for mass, point in zip(masses, centers)
        ) / total_mass
        inertia = np.zeros((3, 3), dtype=float)
        for mass, point, body_inertia in zip(masses, centers, inertias):
            inertia += body_inertia + _parallel_axis_inertia(mass, point - center)
        return replace(
            root_body,
            mass=total_mass,
            inertia=inertia,
            center_of_mass=center,
            fixed=fixed,
        )

    bodies: dict[str, RigidBody] = {}
    for component in components.values():
        root = component_root(component)
        if len(component) == 1:
            bodies[root] = assembly.bodies[root]
        else:
            bodies[root] = fused_body(root, component)

    points: dict[tuple[str, str], np.ndarray] = {}
    for (body, label), point in assembly.points.items():
        target = mapped_body(body)
        key = (target, label)
        converted = point_to_body(point, body, target)
        if key in points and not np.allclose(points[key], converted, atol=1e-9):
            raise ValueError(
                f"weld condensation creates conflicting point {key!r}"
            )
        points[key] = converted

    def transform_constraint(constraint: Constraint) -> Constraint | None:
        if isinstance(constraint, WeldJoint):
            return None
        updates: dict[str, object] = {}
        body_a = getattr(constraint, "body_a", None)
        body_b = getattr(constraint, "body_b", None)
        if isinstance(body_a, str):
            updates["body_a"] = mapped_body(body_a)
        if isinstance(body_b, str):
            updates["body_b"] = mapped_body(body_b)
        if isinstance(body_a, str) and hasattr(constraint, "point_a"):
            updates["point_a"] = point_to_body(
                getattr(constraint, "point_a"), body_a, mapped_body(body_a)
            )
        if isinstance(body_b, str) and hasattr(constraint, "point_b"):
            updates["point_b"] = point_to_body(
                getattr(constraint, "point_b"), body_b, mapped_body(body_b)
            )
        if isinstance(body_a, str) and hasattr(constraint, "axis_a"):
            updates["axis_a"] = axis_to_body(
                getattr(constraint, "axis_a"), body_a, mapped_body(body_a)
            )
        if isinstance(body_b, str) and hasattr(constraint, "axis_b"):
            updates["axis_b"] = axis_to_body(
                getattr(constraint, "axis_b"), body_b, mapped_body(body_b)
            )
        if isinstance(body_a, str) and hasattr(constraint, "axis_a_secondary"):
            updates["axis_a_secondary"] = axis_to_body(
                getattr(constraint, "axis_a_secondary"), body_a, mapped_body(body_a)
            )
        if isinstance(body_b, str) and hasattr(constraint, "axis_b_secondary"):
            updates["axis_b_secondary"] = axis_to_body(
                getattr(constraint, "axis_b_secondary"), body_b, mapped_body(body_b)
            )
        transformed = replace(constraint, **updates)  # ty: ignore[invalid-argument-type]
        if getattr(transformed, "body_a", None) == getattr(transformed, "body_b", None):
            raise ValueError(
                f"weld condensation collapses constraint {constraint.name!r} onto one body"
            )
        return transformed

    def transform_element(element: object) -> object:
        if isinstance(element, (LinearSpringElement, StaticDamperElement, BumpStopElement)):
            body_a = element.body_a
            body_b = element.body_b
            return replace(
                element,
                body_a=mapped_body(body_a),
                body_b=mapped_body(body_b),
                point_a=point_to_body(element.point_a, body_a, mapped_body(body_a)),
                point_b=point_to_body(element.point_b, body_b, mapped_body(body_b)),
            )
        if isinstance(element, BushingElement):
            body_a = element.body_a
            body_b = element.body_b
            return replace(
                element,
                body_a=mapped_body(body_a),
                body_b=mapped_body(body_b),
                local_pose_a=pose_to_body(
                    element.local_pose_a, body_a, mapped_body(body_a)
                ),
                local_pose_b=pose_to_body(
                    element.local_pose_b, body_b, mapped_body(body_b)
                ),
            )
        return element

    constraints = tuple(
        transformed
        for constraint in assembly.constraints
        for transformed in (transform_constraint(constraint),)
        if transformed is not None
    )
    ideal_constraints = tuple(
        transformed
        for constraint in assembly.ideal_constraints
        for transformed in (transform_constraint(constraint),)
        if transformed is not None
    )
    elements = tuple(transform_element(element) for element in assembly.elements)
    connections = tuple(
        replace(
            connection,
            body_a=mapped_body(connection.body_a),
            body_b=mapped_body(connection.body_b),
        )
        for connection in assembly.connections
        if mapped_body(connection.body_a) != mapped_body(connection.body_b)
    )
    wheel_body_names = {
        wheel: mapped_body(body)
        for wheel, body in assembly.wheel_body_names.items()
    }
    wheel_centers = {
        wheel: (
            mapped_body(body),
            point_to_body(point, body, mapped_body(body)),
        )
        for wheel, (body, point) in assembly.wheel_centers.items()
    }
    wheel_rotations_local = {}
    for wheel, rotation in assembly.wheel_rotations_local.items():
        body = assembly.wheel_body_names[wheel]
        target = mapped_body(body)
        wheel_rotations_local[wheel] = (
            axis_to_body(rotation[:, 0], body, target),
            axis_to_body(rotation[:, 1], body, target),
            axis_to_body(rotation[:, 2], body, target),
        )
        wheel_rotations_local[wheel] = np.column_stack(wheel_rotations_local[wheel])

    return replace(
        assembly,
        bodies=bodies,
        state=RigidBodyState(bodies),
        points=points,
        constraints=constraints,
        ideal_constraints=ideal_constraints,
        elements=elements,
        connections=connections,
        wheel_centers=wheel_centers,
        wheel_body_names=wheel_body_names,
        wheel_rotations_local=wheel_rotations_local,
        body_aliases={**assembly.body_aliases, **alias},
    )


def _drop_isolated_bodies(assembly: VehicleRuntime) -> VehicleRuntime:
    """Remove free bodies that have no physical connection to the vehicle."""
    flag = os.environ.get("SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES")
    if flag is not None and flag != "" and flag == "0":
        return assembly

    # The chassis is the assembly's own body, and it is never isolated: it is the
    # body everything else is measured against, so a runtime that named none has
    # nothing to protect here and the set simply starts empty.
    referenced: set[str] = (
        {assembly.chassis_name} if assembly.chassis_name else set()
    )
    for constraint in (*assembly.constraints, *assembly.ideal_constraints):
        for attr in ("body_a", "body_b"):
            body = getattr(constraint, attr, None)
            if isinstance(body, str):
                referenced.add(body)
    for connection in assembly.connections:
        referenced.add(connection.body_a)
        referenced.add(connection.body_b)
    for element in assembly.elements:
        for attr in ("body_a", "body_b"):
            body = getattr(element, attr, None)
            if isinstance(body, str):
                referenced.add(body)
    for body, _point in assembly.wheel_centers.values():
        referenced.add(body)
    referenced.update(assembly.wheel_body_names.values())

    dropped = tuple(
        name
        for name, body in assembly.bodies.items()
        if name not in referenced and not body.fixed
    )
    if not dropped:
        return assembly
    dropped_set = set(dropped)
    bodies = {
        name: body
        for name, body in assembly.bodies.items()
        if name not in dropped_set
    }
    points = {
        key: point
        for key, point in assembly.points.items()
        if key[0] not in dropped_set
    }
    return replace(
        assembly,
        bodies=bodies,
        state=RigidBodyState(bodies),
        points=points,
    )


def _add_wheel(
    wheel: WheelSpec,
    upright: str,
    center: np.ndarray,
    mount_body: str,
    runtime_body: str,
    bodies: dict[str, RigidBody],
    points: dict[tuple[str, str], np.ndarray],
    constraints: list[Constraint],
    connections: list[Connection],
    wheel_centers: dict[str, tuple[str, np.ndarray]],
    wheel_body_names: dict[str, str],
    wheel_rotations_local: dict[str, np.ndarray],
    prefix: str,
) -> None:
    if wheel.body in bodies and runtime_body != wheel.body:
        raise ValueError(f"wheel body {wheel.body!r} collides with suspension body")
    quaternion = np.asarray(wheel.pose.rotation.as_tuple(), dtype=float)
    rotation = SE3(np.zeros(3), quaternion).rotation
    center_local = wheel.center_local.as_array()
    center_global = bodies[upright].pose.transform_point(center)
    origin = center_global - rotation @ center_local
    wheel_inertia = _wheel_inertia(wheel)

    if wheel.mount_joint_kind == "fixed":
        mount = bodies[mount_body]
        if mount_body in wheel_body_names.values():
            raise ValueError(
                f"multiple fixed wheels cannot share runtime body {mount_body!r}"
            )
        wheel_to_mount = mount.pose.rotation.T @ rotation
        center_mount = mount.pose.rotation.T @ (
            center_global - mount.pose.translation
        )
        _merge_fixed_wheel(
            mount,
            bodies,
            mount_body,
            wheel_origin=origin,
            wheel_rotation=rotation,
            wheel_mass=wheel.mass,
            wheel_inertia=wheel_inertia,
        )
        points[(mount_body, "center")] = center_mount.copy()
        points[(mount_body, "contact")] = wheel_to_mount @ np.array(
            [0.0, 0.0, -wheel.tire.unloaded_radius], dtype=float
        )
        wheel_body_names[wheel.name] = mount_body
        wheel_rotations_local[wheel.name] = wheel_to_mount
        wheel_centers[wheel.name] = (upright, center.copy())
        return

    bodies[wheel.body] = RigidBody(
        name=wheel.body,
        pose=SE3(origin, quaternion),
        mass=wheel.mass,
        inertia=wheel_inertia,
    )
    points[(wheel.body, "center")] = center_local.copy()
    points[(wheel.body, "contact")] = np.array(
        [0.0, 0.0, -wheel.tire.unloaded_radius], dtype=float
    )
    if mount_body != upright:
        mount_point = local_point(bodies, mount_body, center_global)
        points[(mount_body, "mount")] = mount_point.copy()
    else:
        mount_point = center
    if "wheel_hub" in mount_body:
        joint: Constraint = WeldJoint(
            mount_body,
            mount_point,
            wheel.body,
            center_local,
            name=f"{prefix}wheel_mount_{wheel.name}",
        )
    else:
        joint = RevoluteJoint(
            mount_body,
            mount_point,
            bodies[mount_body].pose.rotation.T @ wheel.spin_axis.as_array(),
            wheel.body,
            center_local,
            wheel.spin_axis.as_array(),
            name=f"{prefix}wheel_spin_{wheel.name}",
        )
    constraints.append(joint)
    wheel_body_names[wheel.name] = wheel.body
    wheel_rotations_local[wheel.name] = np.eye(3)
    wheel_centers[wheel.name] = (upright, center.copy())
    connections.append(
        Connection(
            name=joint.name,
            kind="ideal",
            body_a=mount_body,
            body_b=wheel.body,
            point_a="wheel_center" if mount_body == upright else "mount",
            point_b="center",
        )
    )


def _wheel_inertia(wheel: WheelSpec) -> np.ndarray:
    """返回当前运行时车轮表示使用的惯量."""
    if wheel.inertia is not None:
        return np.asarray(wheel.inertia, dtype=float)
    return np.eye(3) * wheel.axial_inertia


def _parallel_axis_inertia(mass: float, offset: np.ndarray) -> np.ndarray:
    """返回点质量位于 ``offset`` 时的平行轴惯量修正."""
    offset_squared = float(offset @ offset)
    return mass * (offset_squared * np.eye(3) - np.outer(offset, offset))


def _merge_fixed_wheel(
    mount: RigidBody,
    bodies: dict[str, RigidBody],
    mount_body: str,
    *,
    wheel_origin: np.ndarray,
    wheel_rotation: np.ndarray,
    wheel_mass: float,
    wheel_inertia: np.ndarray,
) -> None:
    """用复合质量属性将固定车轮精确凝聚到安装刚体."""
    mount_rotation = mount.pose.rotation
    wheel_center_local = mount_rotation.T @ (
        wheel_origin - mount.pose.translation
    )
    wheel_inertia_local = (
        mount_rotation.T @ wheel_rotation @ wheel_inertia @ wheel_rotation.T @ mount_rotation
    )
    mount_mass = float(mount.mass)
    total_mass = mount_mass + float(wheel_mass)
    if total_mass <= 0.0:
        bodies[mount_body] = replace(mount, mass=0.0)
        return
    mount_com = np.asarray(mount.center_of_mass, dtype=float)
    composite_com = (
        mount_mass * mount_com + float(wheel_mass) * wheel_center_local
    ) / total_mass
    composite_inertia = (
        np.asarray(mount.inertia, dtype=float)
        + _parallel_axis_inertia(mount_mass, mount_com - composite_com)
        + wheel_inertia_local
        + _parallel_axis_inertia(float(wheel_mass), wheel_center_local - composite_com)
    )
    bodies[mount_body] = replace(
        mount,
        mass=total_mass,
        inertia=composite_inertia,
        center_of_mass=composite_com,
    )


def _condense_wheel_end(
    runtime: Any, mounts: Mapping[str, str]
) -> Any:
    """
    Fold each declared wheel body into the body it mounts to.

    Decision D2's condensation, and it goes through the **existing** mechanism:
    ``_merge_fixed_wheel`` is the composition's own composite-mass routine, so a
    condensed wheel and a fixed vehicle wheel are the same statement about the
    same physics -- the wheel's mass, centre of mass and inertia are folded into
    its mount by the parallel-axis rule, and no independent wheel body is left
    behind.  Nothing here rewrites a number: what changes is which body carries
    it.

    The entity set, the constraint rows and the degrees of freedom are all the
    invariants this has to keep, so:

    * the wheel body disappears, and the *only* rows removed are the ones naming
      it -- a row interior to a body that no longer exists would otherwise
      constrain a body that is not there;
    * its wheel-centre point is carried over to the mount when the mount does not
      declare one, because the reader that drives the wheel centre asks for that
      point by name;
    * the elements it owned follow it to the mount, with their local centre
      re-expressed in the mount's frame.  That is a change of *frame*, not of
      geometry: the world point the tire acts at is the same one.

    ``mounts`` maps each wheel body to its mount, and it is the caller's because
    the caller is the party that knows which bodies the wheel subsystem
    produced.
    """
    pairs = {body: mount for body, mount in mounts.items() if body != mount}
    if not pairs:
        return runtime
    bodies = dict(runtime.bodies)
    wheels = {body: bodies[body] for body in pairs if body in bodies}
    for wheel_body, mount in pairs.items():
        if mount not in bodies:
            raise ValueError(
                f"wheel body {wheel_body!r} mounts to {mount!r}, which this "
                "assembly does not carry"
            )
        if wheel_body not in wheels:
            continue
        wheel = wheels[wheel_body]
        _merge_fixed_wheel(
            bodies[mount],
            bodies,
            mount,
            wheel_origin=np.asarray(wheel.pose.translation, dtype=float),
            wheel_rotation=np.asarray(wheel.pose.rotation, dtype=float),
            wheel_mass=float(wheel.mass),
            wheel_inertia=np.asarray(wheel.inertia, dtype=float),
        )
        del bodies[wheel_body]

    # Every point the wheel end declared follows it to its mount, under its own
    # label: one body's points are the places on it, and a label that disappeared
    # would be a place the reading can no longer name.  `wheel_center` keeps its
    # spelling because that is what the reader looks for; a label the mount already
    # uses is left with the mount's own value, which is the one the assembly
    # declared first.
    points: dict[tuple[str, str], np.ndarray] = {
        key: point for key, point in runtime.points.items() if key[0] not in pairs
    }
    for wheel_body, mount in pairs.items():
        if wheel_body not in wheels:
            continue
        wheel = wheels[wheel_body]
        for (body, label), point in runtime.points.items():
            if body != wheel_body or (mount, label) in points:
                continue
            world = wheel.pose.transform_point(np.asarray(point, dtype=float))
            points[(mount, label)] = local_point(bodies, mount, world)

    elements = tuple(
        _reown_wheel_end_element(element, wheels, bodies, pairs)
        for element in runtime.elements
    )
    constraints = tuple(
        row for row in runtime.constraints if not _names_a_condensed_body(row, pairs)
    )
    ideal_constraints = tuple(
        row
        for row in runtime.ideal_constraints
        if not _names_a_condensed_body(row, pairs)
    )
    connections = tuple(
        replace(
            connection,
            body_a=pairs.get(connection.body_a, connection.body_a),
            body_b=pairs.get(connection.body_b, connection.body_b),
        )
        for connection in runtime.connections
    )
    return replace(
        runtime,
        bodies=bodies,
        state=RigidBodyState(bodies),
        points=points,
        elements=elements,
        constraints=constraints,
        ideal_constraints=ideal_constraints,
        connections=connections,
    )


def _reown_wheel_end_element(
    element: Any,
    wheels: Mapping[str, RigidBody],
    bodies: Mapping[str, RigidBody],
    pairs: Mapping[str, str],
) -> object:
    """Return one element with a condensed owner replaced by the mount."""
    owner = getattr(element, "wheel_body", None)
    if owner not in pairs or owner not in wheels:
        return element
    mount = pairs[owner]
    local_center = getattr(element, "wheel_center_local", None)
    if local_center is None:
        return element
    world = wheels[owner].pose.transform_point(np.asarray(local_center, dtype=float))
    return replace(
        element,
        wheel_body=mount,
        wheel_center_local=local_point(bodies, mount, world),
    )

def _names_a_condensed_body(row: object, pairs: Mapping[str, str]) -> bool:
    """Return whether a constraint row names a body that was condensed away."""
    return bool(
        {getattr(row, "body_a", None), getattr(row, "body_b", None)} & set(pairs)
    )
