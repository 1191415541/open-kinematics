"""Full-vehicle multibody topology and wheel-end assembly."""

from __future__ import annotations

from dataclasses import dataclass, fields, replace
from typing import Any, Literal

import numpy as np

from ..core import (
    SE3,
    Constraint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
)
from ..elements import VerticalTireElement
from ..schema import RigidBodySpec, VehicleModel, WheelSpec
from .front_axle import (
    Connection,
    FrontAxleAssembly,
    _local_point,
    build_front_axle,
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
    return VehicleAssembly(
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
    )


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
        for field in fields(value):
            if field.name in body_fields:
                updates[field.name] = body_map[getattr(value, field.name)]
            elif field.name == "name":
                updates[field.name] = f"{prefix}{getattr(value, field.name)}"
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
