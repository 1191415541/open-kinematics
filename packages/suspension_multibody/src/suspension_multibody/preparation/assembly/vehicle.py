"""
Full-vehicle multibody topology and wheel-end assembly.

The author-side half of the vehicle: two axles merged under one chassis, the
wheel ends attached, and the load-bearing names the contract needs.  Welded
bodies stay separate by default (the epic's A3 ruling): each weld goes to the
kernel as a ``kind="fixed"`` joint, six rows of point coincidence plus full
relative rotation.  ``_fuse_welded_bodies`` keeps the Python-side fusion
behind ``SUSPENSION_MULTIBODY_CONDENSE_WELDS=1``.  Moved here from
``model/vehicle.py``, which 08 deletes.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field, fields, replace
from typing import Any, Literal

import numpy as np

from ...elements import (
    BumpStopElement,
    BushingElement,
    LinearSpringElement,
    StaticDamperElement,
    VerticalTireElement,
)
from ...schema import RigidBodySpec, VehicleModel, WheelSpec
from ...subsystems import (
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyCapabilities,
    capabilities_for,
)
from ..geometry import SE3
from .front_axle import (
    Connection,
    FrontAxleAssembly,
    _local_point,
    build_front_axle,
)
from .types import (
    Constraint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
    WeldJoint,
)


@dataclass(frozen=True)
class VehicleAssembly:
    """Merged chassis, suspension and wheel-end runtime representation."""

    mode: Literal["K", "C"]
    bodies: dict[str, RigidBody]
    state: RigidBodyState
    points: dict[tuple[str, str], np.ndarray]
    constraints: tuple[Constraint, ...]
    ideal_constraints: tuple[Constraint, ...]
    elements: tuple[object, ...]
    connections: tuple[Connection, ...]
    wheel_specs: dict[str, WheelSpec]
    wheel_centers: dict[str, tuple[str, np.ndarray]]
    wheel_body_names: dict[str, str]
    wheel_rotations_local: dict[str, np.ndarray]
    axle_assemblies: dict[str, FrontAxleAssembly]
    body_aliases: dict[str, str] = field(default_factory=dict)
    #: What this assembly carries.  A full-vehicle rig binds to this instead of
    #: probing for body names; the full vehicle carries all six roles, brake and
    #: drive included (requirement 17 / D8).  Defaulted so existing constructions
    #: and `replace` calls keep working unchanged.
    capabilities: AssemblyCapabilities | None = None

    @property
    def component_ids(self) -> tuple[str, ...]:
        """Return deterministic body identifiers."""
        return tuple(self.bodies)

    @property
    def wheel_ids(self) -> tuple[str, ...]:
        """Return deterministic corner identifiers."""
        return tuple(self.wheel_specs)

    @property
    def element_ids(self) -> tuple[str, ...]:
        """Return deterministic force-element identifiers."""
        return tuple(
            getattr(element, "name", f"element_{index}")
            for index, element in enumerate(self.elements)
        )

    @property
    def total_mass(self) -> float:
        """Return the sum of all movable and fixed body masses."""
        return float(sum(body.mass for body in self.bodies.values()))

    def wheel_center_local(self, wheel: str) -> np.ndarray:
        """Return the wheel-center point on its upright body."""
        try:
            return self.wheel_centers[wheel][1].copy()
        except KeyError as exc:
            raise KeyError(f"unknown wheel {wheel!r}") from exc


def build_vehicle(model: VehicleModel, mode: Literal["K", "C"] = "K") -> VehicleAssembly:
    """Compose suspension and wheel ends, condensing fixed wheels exactly."""
    if mode not in ("K", "C"):
        raise ValueError("mode must be K or C")
    chassis = _body_from_spec(model.chassis)
    bodies: dict[str, RigidBody] = {chassis.name: chassis}
    points: dict[tuple[str, str], np.ndarray] = {}
    constraints: list[Constraint] = []
    ideal_constraints: list[Constraint] = []
    elements: list[object] = []
    connections: list[Connection] = []
    axle_assemblies: dict[str, FrontAxleAssembly] = {}
    wheel_specs = {wheel.name: wheel for wheel in model.wheels}
    wheel_centers: dict[str, tuple[str, np.ndarray]] = {}
    wheel_body_names: dict[str, str] = {}
    wheel_rotations_local: dict[str, np.ndarray] = {}

    for axle_name, axle_model, prefix in (
        ("front", model.front_axle, "front_"),
        ("rear", model.rear_axle, "rear_"),
    ):
        axle = build_front_axle(axle_model, mode=mode)
        axle_assemblies[axle_name] = axle
        body_map = {
            old: model.chassis.name if old == "chassis" else f"{prefix}{old}"
            for old in axle.bodies
        }
        for old_name, body in axle.bodies.items():
            if old_name == "chassis":
                continue
            new_name = body_map[old_name]
            if new_name in bodies:
                raise ValueError(f"duplicate vehicle body {new_name!r}")
            bodies[new_name] = replace(body, name=new_name)
        points.update(
            {
                (body_map[body], label): np.asarray(point, dtype=float).copy()
                for (body, label), point in axle.points.items()
            }
        )
        constraints.extend(
            _rename_dataclasses(axle.constraints, body_map, prefix)
        )
        ideal_constraints.extend(
            _rename_dataclasses(axle.ideal_constraints, body_map, prefix)
        )
        elements.extend(
            element
            for element in _rename_dataclasses(axle.elements, body_map, prefix)
            if not isinstance(element, VerticalTireElement)
        )
        connections.extend(
            _rename_connections(axle.connections, body_map, prefix)
        )

        for wheel in model.wheels:
            if (wheel.name.startswith(f"{axle_name}_")):
                side = "L" if wheel.name.endswith("left") else "R"
                upright = body_map[f"upright_{side}"]
                center = points[(upright, "wheel_center")]
                mount_body = wheel.mount_body or f"upright_{side}"
                actual_mount_body = body_map.get(mount_body, mount_body)
                if actual_mount_body not in bodies:
                    raise ValueError(
                        f"wheel {wheel.name!r} mount body {mount_body!r} is undefined"
                    )
                runtime_body = (
                    actual_mount_body
                    if wheel.mount_joint_kind == "fixed"
                    else wheel.body
                )
                _add_wheel(
                    wheel,
                    upright,
                    center,
                    actual_mount_body,
                    runtime_body,
                    bodies,
                    points,
                    constraints,
                    connections,
                    wheel_centers,
                    wheel_body_names,
                    wheel_rotations_local,
                    prefix,
                )

    state = RigidBodyState(bodies)
    assembly = VehicleAssembly(
        mode=mode,
        bodies=bodies,
        state=state,
        points=points,
        constraints=tuple(constraints),
        ideal_constraints=tuple(ideal_constraints),
        elements=tuple(elements),
        connections=tuple(connections),
        wheel_specs=wheel_specs,
        wheel_centers=wheel_centers,
        wheel_body_names=wheel_body_names,
        wheel_rotations_local=wheel_rotations_local,
        axle_assemblies=axle_assemblies,
        # The full vehicle carries all six roles, brake and drive included
        # (requirement 17 / D8), so a vehicle rig can ask rather than probe.
        capabilities=capabilities_for(
            subsystems=frozenset(DEFAULT_VEHICLE_SUBSYSTEMS),
            body_names=frozenset(bodies),
        ),
    )
    return _drop_isolated_bodies(_condense_welded_bodies(assembly))


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


def _condense_welded_bodies(assembly: VehicleAssembly) -> VehicleAssembly:
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


def _fuse_welded_bodies(assembly: VehicleAssembly) -> VehicleAssembly:
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
        if "chassis" in component:
            return "chassis"
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


def _drop_isolated_bodies(assembly: VehicleAssembly) -> VehicleAssembly:
    """Remove free bodies that have no physical connection to the vehicle."""
    flag = os.environ.get("SUSPENSION_MULTIBODY_DROP_ISOLATED_BODIES")
    if flag is not None and flag != "" and flag == "0":
        return assembly

    referenced: set[str] = {"chassis"}
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
        mount_point = _local_point(bodies, mount_body, center_global)
        points[(mount_body, "mount")] = mount_point.copy()
    else:
        mount_point = center
    spin_joint = RevoluteJoint(
        mount_body,
        mount_point,
        bodies[mount_body].pose.rotation.T @ wheel.spin_axis.as_array(),
        wheel.body,
        center_local,
        wheel.spin_axis.as_array(),
        name=f"{prefix}wheel_spin_{wheel.name}",
    )
    constraints.append(spin_joint)
    wheel_body_names[wheel.name] = wheel.body
    wheel_rotations_local[wheel.name] = np.eye(3)
    wheel_centers[wheel.name] = (upright, center.copy())
    connections.append(
        Connection(
            name=spin_joint.name,
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
