"""
Interpret data-only multibody templates and submit their native contracts.

Hardpoints and initial body positions use the template's length unit (mm when
omitted, matching existing authoring documents). Axes are world-frame vectors.
The native model is SI. A test rig is another subsystem entry in this path.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Any, Mapping

import numpy as np
from suspension_contracts import (
    CONTRACT_VERSION,
    pack_container,
    validate_model,
)

from ..connections.matcher import MatchReport, match_requirements
from ..modeling.identity import EntityId
from ..modeling.instance import FragmentProvenance, ModelFragment
from ..modeling.ports import (
    ChannelPort,
    GeometryPort,
    PortRequirement,
    PortSpec,
    RoadPort,
    SpinPort,
)
from ..modeling.primitives.factory import body_from_data, joint_from_data
from ..modeling.primitives.joints import RigidBody
from ..modeling.primitives.spatial import SE3
from ..modeling.resolved import ResolvedModel
from ..properties.tire_native import compile_tires
from .documents import AssemblyDocument, EffectiveSubsystem, SubsystemDocument
from .errors import AuthoringError
from .function_program import (
    compile_curve,
    compile_function,
    compile_surface,
    evaluate_constant_function,
)

__all__ = [
    "GenericMultibodyAssembly",
    "GenericSubsystemAssembler",
    "assemble_generic",
]

_MIRROR = np.diag([1.0, -1.0, 1.0])
_AXIAL_JOINTS = {"revolute", "universal", "cylindrical", "convel", "driven_rotation"}
_AXIS_JOINTS = _AXIAL_JOINTS | {"prismatic", "inplane", "driven_translation"}
_IDENTITY = np.eye(3)
_ZERO = (0.0, 0.0, 0.0)


def _vector(value: Any, label: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise AuthoringError(f"{label}: expected three finite values")
    return vector


def _unit_axis(value: Any, label: str) -> np.ndarray:
    axis = _vector(value, label)
    norm = np.linalg.norm(axis)
    if norm <= 1e-12:
        raise AuthoringError(f"{label}: axis must be nonzero")
    return axis / norm


def _active(row: Mapping[str, Any], mode: str, configuration: str = "") -> bool:
    return mode in row.get("modes", ("K", "C")) and (
        "configurations" not in row or configuration in row["configurations"]
    )


def _side_name(name: str, side: str, sided: set[str]) -> str:
    if not side or name not in sided:
        return name
    return name[:-2] + "_" + side if name.endswith("_L") else name + "_" + side


def _sides(
    row: Mapping[str, Any], mirrors: bool, involved: bool = True
) -> tuple[str, ...]:
    return ("L", "R") if mirrors and row.get("symmetric", involved) else ("",)


def _ref(prefix: str, name: str) -> str:
    return f"{prefix}.{name}" if prefix else name


def _has_tire_mount(owner: str, carrier: str, joints: Any) -> bool:
    def fixed_members(body: str) -> set[str]:
        members = {body}
        pending = [body]
        while pending:
            current = pending.pop()
            for joint in joints:
                if joint["type"] != "fixed" or current not in {joint["body_a"], joint["body_b"]}:
                    continue
                other = joint["body_b"] if current == joint["body_a"] else joint["body_a"]
                if other not in members:
                    members.add(other)
                    pending.append(other)
        return members

    wheel_members, carrier_members = fixed_members(owner), fixed_members(carrier)
    return bool(wheel_members & carrier_members) or any(
        joint["type"] == "revolute" and (
            joint["body_a"] in wheel_members and joint["body_b"] in carrier_members
            or joint["body_b"] in wheel_members and joint["body_a"] in carrier_members
        ) for joint in joints
    )


@dataclass(frozen=True)
class GenericMultibodyAssembly:
    """A materialised native model with its entity and port provenance."""

    name: str
    bodies: Mapping[str, RigidBody]
    joints: tuple[dict[str, Any], ...]
    elements: tuple[dict[str, Any], ...]
    ports: Mapping[str, PortSpec] = field(default_factory=dict)
    fragments: tuple[ModelFragment, ...] = ()
    gravity: tuple[float, ...] = (0.0, 0.0, -9.80665)
    bindings: Mapping[str, MatchReport] = field(default_factory=dict)
    markers: Mapping[str, GeometryPort] = field(default_factory=dict)
    tires: tuple[dict[str, Any], ...] = ()
    function_programs: tuple[dict[str, Any], ...] = ()
    inputs: tuple[dict[str, Any], ...] = ()
    coordinates: tuple[dict[str, Any], ...] = ()
    coordinate_frames: tuple[dict[str, Any], ...] = ()
    measurements: tuple[dict[str, Any], ...] = ()
    couplers: tuple[dict[str, Any], ...] = ()
    gauges: tuple[dict[str, Any], ...] = ()
    capabilities: tuple[str, ...] = ()
    initial_state: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    road: Mapping[str, Any] | None = None

    def resolved_model(self) -> ResolvedModel:
        """Resolve native-ready entities and complete local frames into one SI graph."""
        native = self.model_document()
        frames = {**self.markers, **{
            name: port for name, port in self.ports.items()
            if isinstance(port, GeometryPort)
        }}
        document = {
            "schema_version": 1, "name": self.name, "units": "SI",
            "bodies": native["bodies"], "joints": native["joints"],
            "elements": native["elements"], "tires": native["tires"],
            "frames": [
                {"name": name, "body": port.owner.local,
                 "point": port.pose.translation.tolist(),
                 "quaternion": port.pose.quaternion.tolist()}
                for name, port in frames.items()
            ],
            "ports": [], "gravity": list(self.gravity),
            "coordinates": list(self.coordinates),
            "provenance": {
                name: {"template": fragment.provenance.template,
                       "revision": fragment.provenance.revision,
                       "instance": list(fragment.instance),
                       "properties_fingerprint": fragment.provenance.properties_fingerprint}
                for fragment in self.fragments if fragment.provenance is not None
                for name in fragment.bodies
            },
        }
        for name, port in self.ports.items():
            row = {"name": name, "kind": port.kind, "owner": str(port.owner),
                   "role": port.role, "cardinality": port.cardinality,
                   "capabilities": sorted(port.capabilities), "labels": sorted(port.labels)}
            if isinstance(port, GeometryPort):
                row["frame"] = name
            elif isinstance(port, ChannelPort):
                row["units"] = port.units
                if port.direction is not None:
                    row["direction"] = list(port.direction)
            elif isinstance(port, RoadPort):
                row["resource"] = port.resource
            elif isinstance(port, SpinPort):
                row.update(coordinate=port.coordinate, units=port.units, owner=port.owner.local)
            document["ports"].append(row)
        for key in ("road", "blobs", "function_programs", "couplers", "gauges", "capabilities"):
            if key in native:
                document[key] = native[key]
        document["signals"] = list(self.inputs)
        document["measurements"] = list(self.measurements)
        document["frames"].extend(self.coordinate_frames)
        return ResolvedModel(document, compile_tires(self.tires)[2])

    def model_document(self) -> dict[str, Any]:
        tires, descriptors, _ = compile_tires(self.tires)
        document = {
            "contract": "multibody-model",
            "contract_version": CONTRACT_VERSION,
            "kind": "model",
            "name": self.name,
            "units": {"length": "m", "mass": "kg", "time": "s", "angle": "rad"},
            "gravity": list(self.gravity),
            "bodies": [
                {
                    "name": name,
                    "mass": body.mass,
                    "fixed": body.fixed,
                    "inertia": body.inertia.tolist(),
                    "position": body.pose.translation.tolist(),
                    "quaternion": body.pose.quaternion.tolist(),
                }
                for name, body in self.bodies.items()
            ],
            "joints": list(self.joints),
            "elements": list(self.elements),
            "markers": [
                {
                    "name": name,
                    "body": port.owner.local,
                    "point": port.pose.translation.tolist(),
                }
                for name, port in {**self.markers, **self.ports}.items()
                if isinstance(port, GeometryPort)
            ],
            "tires": tires,
        }
        if descriptors:
            document["blobs"] = descriptors
        if tires or self.road is not None:
            document["road"] = dict(self.road or {"kind": "plane"})
        if self.function_programs:
            document["function_programs"] = list(self.function_programs)
        for key in ("couplers", "gauges", "capabilities"):
            value = getattr(self, key)
            if value:
                document[key] = list(value)
        for row in document["bodies"]:
            row.update(self.initial_state.get(row["name"], {}))
        validate_model(document)
        return document

    def model_container(self) -> bytes:
        """Pack the model and its tire parameter tables into one native container."""
        _, _, blob = compile_tires(self.tires)
        return pack_container(self.model_document(), blob)


class GenericSubsystemAssembler:
    """One interpreter for every data-declared rigid-body graph."""

    @staticmethod
    def materialize_bodies(rows: Any) -> dict[str, RigidBody]:
        """Create bodies from resolved declarations in the caller's units."""
        return {str(row["name"]): body_from_data(row) for row in rows}

    @staticmethod
    def materialize_joint(row: Mapping[str, Any]) -> Any:
        """Create a resolved primitive joint for runtime consumers."""
        return joint_from_data(row)

    def assemble(
        self,
        subsystem: SubsystemDocument | EffectiveSubsystem,
        *,
        mode: str = "K",
        instance: str = "",
        geometry_only: bool = False,
        external_ports: Mapping[str, PortSpec] | None = None,
        external_bodies: Mapping[str, RigidBody] | None = None,
        defer_mount_validation: bool = False,
        placement: SE3 | None = None,
    ) -> GenericMultibodyAssembly:
        if mode not in ("K", "C"):
            raise AuthoringError(f"unknown mode {mode!r}; modes are K and C")
        effective = (
            subsystem.effective()
            if isinstance(subsystem, SubsystemDocument)
            else subsystem
        )
        template = effective.template
        mounting = SE3.identity() if placement is None else placement
        data = template.payload
        configuration = str(effective.parameters.get("configuration", data.get("default_configuration", "")))
        if configuration and configuration not in data.get("configurations", ()):
            raise AuthoringError(f"{template.where}: unknown configuration {configuration!r}")
        def enabled(row: Mapping[str, Any]) -> bool:
            if not _active(row, mode, configuration):
                return False
            if geometry_only:
                return True
            return all(key in (external_ports or {}) for key in row.get("requires", ()))
        scale = 0.001 if data.get("units", {}).get("length", "mm") == "mm" else 1.0
        mirrors = template.mirrors
        body_rows = list(data["bodies"])
        hardpoint_rows = {str(row["name"]): row for row in data["hardpoints"]}
        marker_declarations = {str(row["name"]): row for row in data.get("markers", ())}
        for tire in data.get("tires", ()):
            row = tire["body"]
            if any(body["name"] == row["name"] for body in body_rows):
                continue
            center = marker_declarations[str(row["center_marker"])]
            if center["owner"] != row["name"]:
                raise AuthoringError(f"tire {tire['name']!r}: center_marker must belong to its wheel body")
            if not row.get("mass_slot") or not row.get("inertia_slot"):
                raise AuthoringError(f"tire {tire['name']!r}: mass_slot and inertia_slot are required")
            mass = self._law(effective, str(row["mass_slot"])).get("value")
            inertia_law = self._law(effective, str(row["inertia_slot"]))
            tensor = inertia_law.get("inertia")
            if mass is None or tensor is None:
                raise AuthoringError(f"tire {tire['name']!r}: mass/inertia slots need explicit inertial properties")
            inertia_scale = 0.001 if inertia_law.get("units", {}).get("length", "mm") == "mm" else 1.0
            body_rows.append({
                "name": row["name"], "mass": mass,
                "inertia": (np.asarray(tensor, dtype=float)*(inertia_scale/scale)**2).tolist(),
                "position": effective.hardpoints[str(center["point"])],
                "symmetric": tire.get("symmetric", True),
            })
        sided_bodies = {
            str(row["name"])
            for row in body_rows
            if mirrors and row.get("symmetric", True)
        }
        sided_points = {
            str(row["name"])
            for row in data["hardpoints"]
            if mirrors
            and row.get("symmetric", str(row.get("owner", "")) in sided_bodies)
        }
        bodies: dict[str, RigidBody] = {}
        initial_state: dict[str, dict[str, Any]] = {}
        for row in body_rows:
            local = str(row["name"])
            for side in _sides(row, mirrors):
                name = _ref(instance, _side_name(local, side, sided_bodies))
                mass = row.get("mass", 0.0)
                if row.get("mass_slot"):
                    mass = self._law(effective, str(row["mass_slot"])).get("value")
                    if mass is None:
                        raise AuthoringError(
                            f"{template.where}: mass_slot needs a numeric default"
                        )
                position_value = effective.hardpoints[str(row["position_point"])] if row.get("position_point") else row.get("position", _ZERO)
                position = _vector(position_value, f"body {name}") * scale
                quaternion = np.asarray(
                    row.get("quaternion", (1.0, 0.0, 0.0, 0.0)), dtype=float
                )
                inertia = (
                    np.asarray(row.get("inertia", _IDENTITY), dtype=float) * scale**2
                )
                if row.get("inertia_slot"):
                    law = self._law(effective, str(row["inertia_slot"]))
                    inertia_scale = 0.001 if law.get("units", {}).get("length", "mm") == "mm" else 1.0
                    if law.get("inertia") is None:
                        raise AuthoringError(f"body {name!r}: inertia_slot needs a tensor")
                    inertia = np.asarray(law["inertia"], dtype=float) * inertia_scale**2
                com = (
                    _vector(row.get("center_of_mass", _ZERO), f"body {name} COM")
                    * scale
                )
                if side == "R":
                    position = _MIRROR @ position
                    quaternion = quaternion * (1.0, -1.0, 1.0, -1.0)
                    inertia = _MIRROR @ inertia @ _MIRROR
                    com = _MIRROR @ com
                pose = mounting.compose(SE3(position, quaternion))
                # Native body frames are centred at COM; shift the frame and keep
                # hardpoints in world coordinates until their owning body is known.
                position = pose.transform_point(com)
                if name in bodies:
                    raise AuthoringError(
                        f"{template.where}: mirrored body {name!r} is declared twice"
                    )
                body = self.materialize_bodies([{
                    "name": name, "mass": mass, "inertia": inertia,
                    "fixed": row.get("fixed", False),
                    "position": position, "quaternion": pose.quaternion,
                }])[name]
                if not np.allclose(body.inertia, body.inertia.T, atol=1e-12):
                    raise AuthoringError(f"body {name!r}: inertia must be symmetric")
                if not body.fixed and (
                    body.mass <= 0 or np.linalg.eigvalsh(body.inertia).min() <= 0
                ):
                    raise AuthoringError(
                        f"body {name!r}: free bodies need positive mass and inertia"
                    )
                bodies[name] = body
                state = {}
                for key in ("velocity", "omega"):
                    if key in row:
                        value = _vector(row[key], f"body {name} {key}") * (scale if key == "velocity" else 1)
                        if side == "R":
                            value = (1 if key == "velocity" else -1) * _MIRROR @ value
                        state[key] = (mounting.rotation @ value).tolist()
                if state:
                    initial_state[name] = state

        available_bodies = {**(external_bodies or {}), **bodies}

        def external_port(local: str, side: str) -> PortSpec:
            alias = local.removeprefix("@")
            candidates = external_ports or {}
            port = candidates.get(_side_name(alias, side, {alias}), candidates.get(alias))
            if port is None:
                raise AuthoringError(f"{template.where}: unbound external endpoint {local!r}")
            return port

        def body_name(local: str, side: str) -> str:
            if local.startswith("@"):
                port = external_port(local, side)
                if not isinstance(port, GeometryPort):
                    raise AuthoringError(f"{template.where}: {local!r} must bind a geometry port")
                return port.owner.local
            name = _ref(instance, _side_name(local, side, sided_bodies))
            if name not in bodies:
                raise AuthoringError(
                    f"{template.where}: body {local!r} is not declared here; connect neighbours through ports"
                )
            return name

        def world_point(local: str, side: str) -> np.ndarray:
            if not local or local not in effective.hardpoints:
                raise AuthoringError(f"{template.where}: missing hardpoint {local!r}")
            point = _vector(effective.hardpoints[local], f"hardpoint {local}") * scale
            point = _MIRROR @ point if side == "R" and local in sided_points else point
            return mounting.transform_point(point)

        def local_point(body: str, point: str, side: str) -> list[float]:
            if point.startswith("@"):
                port = external_port(point, side)
                if not isinstance(port, GeometryPort) or port.owner.local != body:
                    raise AuthoringError(f"{template.where}: endpoint {point!r} does not belong to {body!r}")
                return port.pose.translation.tolist()
            declared = hardpoint_rows.get(point, {})
            if declared.get("space") == "body":
                if body_name(str(declared.get("owner", "")), side) != body:
                    raise AuthoringError(f"hardpoint {point!r}: local point owner differs from endpoint")
                value = _vector(effective.hardpoints[point], point)*scale
                return (_MIRROR @ value if side == "R" else value).tolist()
            return (
                available_bodies[body]
                .pose.inverse()
                .transform_point(world_point(point, side))
                .tolist()
            )

        joints: list[dict[str, Any]] = []
        elements: list[dict[str, Any]] = []
        points: dict[tuple[str, str], Any] = {}
        for row in data["hardpoints"]:
            owner = str(row.get("owner", ""))
            if geometry_only and owner.startswith("@"):
                continue
            if owner:
                for side in _sides(row, mirrors, owner in sided_bodies):
                    body = body_name(owner, side)
                    point = str(row["name"])
                    points[(body, _side_name(point, side, sided_points))] = local_point(
                        body, point, side
                    )
        for row in () if geometry_only else data["joints"]:
            if not enabled(row):
                continue
            kind = str(row["type"])
            for override in row.get("kind_by_mode", ()):
                if override["mode"] == mode:
                    kind = str(override["type"])
            for side in _sides(
                row,
                mirrors,
                row["body_a"] in sided_bodies or row["body_b"] in sided_bodies,
            ):
                a, b = (
                    body_name(str(row["body_a"]), side),
                    body_name(str(row["body_b"]), side),
                )
                entry = {
                    "name": _ref(
                        instance, _side_name(str(row["name"]), side, {str(row["name"])})
                    ),
                    "type": kind,
                    "body_a": a,
                    "body_b": b,
                    "point_a": local_point(a, str(row.get("point_a", "")), side),
                    "point_b": local_point(
                        b, str(row.get("point_b", row.get("point_a", ""))), side
                    ),
                }
                if kind in _AXIS_JOINTS:
                    axis = row.get("axis")
                    if row.get("axis_reference"):
                        axis = (_vector(effective.hardpoints[str(row["axis_reference"])], "axis reference")
                                - _vector(effective.hardpoints[str(row["point_a"])], "axis point")) * scale
                    self._axes(entry, row, available_bodies, side=side, axis=axis, rotation=mounting.rotation)
                if kind.startswith("driven_"):
                    target = str(row.get("target", ""))
                    if not target:
                        raise AuthoringError(
                            f"joint {entry['name']!r} needs a target signal"
                        )
                    entry["target"] = _ref(instance, _side_name(target, side, {target}))
                if kind == "convel":
                    for end in ("a", "b"):
                        axis = _unit_axis(row[f"axis_{end}_secondary"], entry["name"])
                        if side == "R":
                            axis = -_MIRROR @ axis
                        entry[f"axis_{end}_secondary"] = (axis if row.get("axis_space") == "body" else available_bodies[entry[f"body_{end}"]].pose.rotation.T @ mounting.rotation @ axis).tolist()
                    entry["convel_angle_target"] = float(row.get("convel_angle_target", 0))
                if "reference_quaternion" in row:
                    reference = np.asarray(row["reference_quaternion"], dtype=float)
                    if side == "R":
                        reference *= (1, -1, 1, -1)
                    entry["reference_quaternion"] = reference.tolist()
                if a == b:
                    raise AuthoringError(
                        f"joint {entry['name']!r} connects a body to itself"
                    )
                joints.append(entry)
        for row in () if geometry_only else data["elements"]:
            if not enabled(row):
                continue
            kind = str(row["type"])
            if kind in {"force", "torque", "wrench"}:
                continue
            if kind in {"aerodynamic_drag", "point_wrench", "gravity", "steering_actuator"}:
                if scale != 1:
                    raise AuthoringError(f"element {row['name']!r}: {kind} declarations require SI template units")
                for side in _sides(row, mirrors):
                    params = dict(row.get("parameters", {}))
                    if row.get("property_slot"):
                        params = {**self._law(effective, row["property_slot"]), **params}
                        for key in ("element_type", "model", "units", "source", "value"):
                            params.pop(key, None)
                    for key in ("body", "reaction_body"):
                        if key in params:
                            params[key] = body_name(str(params[key]), side)
                    entry = {"name": _ref(instance, _side_name(str(row["name"]), side, {str(row["name"])})), "type": kind, "parameters": params}
                    for key in ("body_a", "body_b"):
                        if key in row:
                            entry[key] = body_name(str(row[key]), side)
                    if "target" in row:
                        entry["target"] = _ref(instance, str(row["target"]))
                    elements.append(entry)
                continue
            for side in _sides(
                row,
                mirrors,
                row["body_a"] in sided_bodies or row["body_b"] in sided_bodies,
            ):
                a, b = (
                    body_name(str(row["body_a"]), side),
                    body_name(str(row["body_b"]), side),
                )
                law = self._law(effective, str(row["property_slot"]))
                params = self._element_parameters(
                    kind, law, row.get("parameters", {}), scale
                )
                for key, expression in row.get("parameter_expressions", {}).items():
                    values = {
                        binding: float(self._law(effective, slot)["value"])
                        for binding, slot in expression["slots"].items()
                    }
                    value = evaluate_constant_function(expression["function"], values)
                    unit = expression.get("unit", "SI")
                    if unit not in {"SI", "N*mm", "Nm"}:
                        raise AuthoringError(f"element {row['name']!r}: unsupported parameter unit {unit!r}")
                    params[key] = value * (0.001 if unit == "N*mm" else 1.0)
                params["point_a"] = local_point(a, str(row.get("point_a", "")), side)
                params["point_b"] = local_point(
                    b, str(row.get("point_b", row.get("point_a", ""))), side
                )
                if side == "R":
                    self._mirror_parameters(params)
                elements.append(
                    {
                        "name": _ref(
                            instance,
                            _side_name(str(row["name"]), side, {str(row["name"])}),
                        ),
                        "type": "rotational_torque" if kind == "wheel_torque" else kind,
                        "body_a": a,
                        "body_b": b,
                        "parameters": params,
                    }
                )

        markers: dict[str, GeometryPort] = {}
        marker_rows = {str(row["name"]): row for row in data.get("markers", ())}
        for local, row in marker_rows.items():
            for side in _sides(row, mirrors, str(row["owner"]) in sided_bodies):
                name = _side_name(local, side, {local})
                owner = body_name(str(row["owner"]), side)
                quaternion = np.asarray(row.get("quaternion", (1.0, 0.0, 0.0, 0.0)), dtype=float)
                if side == "R":
                    quaternion = quaternion * (1.0, -1.0, 1.0, -1.0)
                markers[_ref(instance, name)] = GeometryPort(
                    id=EntityId((instance,) if instance else (), name),
                    owner=EntityId((), owner),
                    role="marker",
                    pose=SE3(np.asarray(local_point(owner, str(row["point"]), side)), quaternion),
                )
        ports: dict[str, PortSpec] = {}
        for row in data.get("ports", ()):
            marker_row = marker_rows.get(str(row.get("marker", "")), {})
            declared_owner = str(row.get("owner", marker_row.get("owner", "")))
            for side in _sides(row, mirrors, declared_owner in sided_bodies):
                name = _side_name(str(row["name"]), side, {str(row["name"])})
                kind = str(row.get("kind", "geometry"))
                owner = body_name(declared_owner, side) if declared_owner else (instance or effective.name)
                identity = EntityId((instance,) if instance else (), name)
                common = dict(
                    labels=frozenset(row.get("labels", ()))
                    | (frozenset({side}) if side else frozenset()),
                    capabilities=frozenset(row.get("capabilities", ())),
                    cardinality=row.get("cardinality", "one"),
                )
                if kind == "channel":
                    if not row.get("units"):
                        raise AuthoringError(f"channel port {name!r} needs units")
                    direction = row.get("direction")
                    if direction is not None:
                        direction = _unit_axis(direction, name)
                        if side == "R":
                            direction = _MIRROR @ direction
                        direction = tuple(direction)
                    ports[_ref(instance, name)] = ChannelPort(id=identity, owner=EntityId((), owner), role=str(row["role"]), **common, units=str(row["units"]), direction=direction)
                    continue
                if kind == "road":
                    ports[_ref(instance, name)] = RoadPort(id=identity, owner=EntityId((), owner), role=str(row["role"]), **common, resource=str(row.get("resource", "")))
                    continue
                if kind == "spin":
                    coordinate = _ref(instance, _side_name(str(row["coordinate"]), side, {str(row["coordinate"])}))
                    ports[_ref(instance, name)] = SpinPort(id=identity, owner=EntityId((), owner), role=str(row["role"]), **common, coordinate=coordinate)
                    continue
                if not declared_owner:
                    raise AuthoringError(f"geometry port {name!r} needs an owner or marker")
                point = str(row.get("point", ""))
                marker = str(row.get("marker", ""))
                if marker:
                    resolved = markers[_ref(instance, _side_name(marker, side, {marker}))]
                    if resolved.owner.local != owner:
                        raise AuthoringError(f"port {name!r} owner disagrees with marker")
                    pose = resolved.pose
                elif point:
                    pose = SE3(np.asarray(local_point(owner, point, side)), np.array([1.0, 0.0, 0.0, 0.0]))
                else:
                    raise AuthoringError(f"port {name!r} needs a declared point")
                ports[_ref(instance, name)] = GeometryPort(
                    id=identity, owner=EntityId((), owner), role=str(row["role"]),
                    **common,
                    family=str(row.get("family", "")),
                    pose=pose,
                )
        function_programs: list[dict[str, Any]] = []
        geometries = {**markers, **ports}

        def function_marker(local: str, side: str) -> dict[str, Any]:
            name = _ref(instance, _side_name(local, side, {local}))
            port = external_port(local, side) if local.startswith("@") else geometries.get(name, geometries.get(_ref(instance, local)))
            if not isinstance(port, GeometryPort):
                raise AuthoringError(f"function marker {name!r} must be a declared geometry marker/port")
            return {"body": port.owner.local, "point": port.pose.translation.tolist(),
                    "quaternion": port.pose.quaternion.tolist()}

        for row in () if geometry_only else data["elements"]:
            kind = str(row["type"])
            if kind not in {"force", "torque", "wrench"} or not enabled(row):
                continue
            for side in _sides(row, mirrors):
                name = _ref(instance, _side_name(str(row["name"]), side, {str(row["name"])}))
                params = {key: function_marker(str(row[key]), side) for key in ("action", "reaction", "reference")}
                axis = _unit_axis(row.get("axis", (0, 0, 1)), name)
                if side == "R":
                    axis = (1 if kind == "force" else -1)*_MIRROR @ axis
                params["axis"] = axis.tolist()
                function_bindings = {}
                for key, declared in row.get("bindings", {}).items():
                    binding = dict(declared)
                    source = binding.get("source", "constant" if "value" in binding else "")
                    if source == "measurement":
                        for reference in ("action", "reaction", "reference"):
                            binding[reference] = function_marker(str(binding[reference]), side)
                        measured_axis = _unit_axis(binding.get("axis", (0, 0, 1)), key)
                        if side == "R":
                            measured_axis = (-1 if binding.get("measurement") == "relative_angular_velocity" else 1)*_MIRROR @ measured_axis
                        binding["axis"] = measured_axis.tolist()
                    elif source == "property":
                        slot = str(binding.pop("slot", ""))
                        law = self._law(effective, slot)
                        field_name = str(binding.pop("field", "value"))
                        if field_name not in law:
                            raise AuthoringError(f"function {name!r}: property {slot!r} has no {field_name!r}")
                        binding["value"] = law[field_name]
                    elif source not in {"constant", "signal", "channel"}:
                        raise AuthoringError(f"function {name!r}: binding {key!r} requires an explicit source/value")
                    if source in {"signal", "channel"} and "samples" not in binding:
                        signal = str(binding.pop("signal", key))
                        samples = effective.parameters.get("signals", {}).get(signal)
                        if samples is None:
                            raise AuthoringError(f"function {name!r}: missing declared signal {signal!r}")
                        binding["samples"] = samples
                    binding["source"] = source
                    function_bindings[key] = binding
                tables = {}
                for key, table in row.get("tables", {}).items():
                    table = dict(table)
                    dimension = table.pop("dimension", 1)
                    if dimension == 1:
                        tables[key] = compile_curve(table["points"], independent_unit=str(table["independent_unit"]), dependent_unit=str(table["dependent_unit"]), interpolation=str(table.get("interpolation", "piecewise_linear")), extrapolation=str(table.get("extrapolation", "clamp")))
                    elif dimension == 2:
                        units = table["independent_units"]
                        tables[key] = compile_surface(table["x_axis"], table["y_axis"], table["values"], independent_units=(str(units[0]), str(units[1])), dependent_unit=str(table["dependent_unit"]), extrapolation=str(table.get("extrapolation", "clamp")))
                    else:
                        raise AuthoringError(f"function {name!r}: unsupported table dimension {dimension}")
                formulas = list(row["functions"]) if kind == "wrench" else [str(row["function"])]
                ids = []
                for component, formula in enumerate(formulas):
                    if side == "R" and kind == "wrench" and component in (1, 3, 5):
                        formula = f"-({formula})"
                    program = compile_function(formula, bindings=function_bindings,
                        output_unit="N" if kind == "force" or kind == "wrench" and component < 3 else "Nm",
                        variables=row.get("variables"), tables=tables)
                    ids.append(len(function_programs))
                    function_programs.append(program)
                params["program_ids" if kind == "wrench" else "program_id"] = ids if kind == "wrench" else ids[0]
                elements.append({"name": name, "type": kind, "parameters": params})
        tire_rows: list[dict[str, Any]] = []
        tire_names: set[str] = set()
        for tire in () if geometry_only else data.get("tires", ()):
            tire_body = tire["body"]
            local_body = str(tire_body["name"])
            symmetric = bool(tire.get("symmetric", True))
            for side in _sides(tire, mirrors, symmetric):
                body_local = _side_name(local_body, side, {local_body})
                body_name_value = _ref(instance, body_local)
                model_law = self._law(effective, str(tire["model_slot"]))
                if model_law.get("element_type") != "tire":
                    raise AuthoringError(f"tire {tire['name']!r}: model_slot must bind a tire law")
                if model_law.get("model") == "linear":
                    law_scale = 0.001 if model_law.get("units", {}).get("length", "mm") == "mm" else 1.0
                    model_law = {**model_law, "kind": "native_brush", "maximum_compression": .5*model_law["unloaded_radius"],
                        "vertical_stiffness": model_law["stiffness"], "vertical_damping": 0,
                        "friction_coefficient": 1, "longitudinal_stiffness": 1, "cornering_stiffness": 1,
                        "relaxation_length": 1, "detached_relaxation_s": 1,
                        "parameter_source": "linear", "pac2002_coefficients": {}, "pac2002_tables": {}, "fiala_parameters": {}}
                model_name = _side_name(str(tire["name"]), side, {str(tire["name"])})
                if model_name in tire_names:
                    raise AuthoringError(f"{template.where}: duplicate tire instance {model_name!r}")
                tire_names.add(model_name)
                spin_local = _side_name(str(tire["spin_marker"]), side, {str(tire["spin_marker"])})
                frame_local = _side_name(str(tire["contact_frame"]["port"]), side, {str(tire["contact_frame"]["port"])})
                road_local = _side_name(str(tire["road_port"]), side, {str(tire["road_port"])})
                mount_local = _side_name(str(tire["mount_port"]), side, {str(tire["mount_port"])})
                marker_port = markers.get(_ref(instance, spin_local))
                frame_port = external_port(str(tire["contact_frame"]["port"]), side) if frame_local.startswith("@") else ports.get(_ref(instance, frame_local))
                road_port = external_port(str(tire["road_port"]), side) if road_local.startswith("@") else ports.get(_ref(instance, road_local), ports.get(_ref(instance, str(tire["road_port"]))))
                mount_port = external_port(str(tire["mount_port"]), side) if mount_local.startswith("@") else ports.get(_ref(instance, mount_local))
                if not isinstance(marker_port, GeometryPort) or not isinstance(frame_port, GeometryPort):
                    raise AuthoringError(f"tire {model_name!r}: spin/contact frame must be geometry markers or ports")
                if not isinstance(road_port, RoadPort):
                    raise AuthoringError(f"tire {model_name!r}: road_port must be a road port")
                if not isinstance(mount_port, GeometryPort):
                    raise AuthoringError(f"tire {model_name!r}: mount_port must be a geometry port")
                frame_body = frame_port.owner.local
                frame_center = frame_port.pose.translation.tolist()
                if marker_port.owner.local != body_name_value:
                    raise AuthoringError(f"tire {model_name!r}: spin_marker must belong to its wheel")
                if road_port.resource != "plane":
                    raise AuthoringError(f"tire {model_name!r}: road resource must declare the supported plane road")
                frame_pose = available_bodies[frame_body].pose.compose(frame_port.pose)
                spin_pose = bodies[body_name_value].pose.compose(marker_port.pose)
                if not np.allclose(frame_pose.translation, spin_pose.translation, rtol=0, atol=1e-10):
                    raise AuthoringError(f"tire {model_name!r}: contact_frame must locate the wheel centre")
                if not np.allclose(frame_pose.rotation[:, 1], spin_pose.rotation[:, 1], rtol=0, atol=1e-10):
                    raise AuthoringError(f"tire {model_name!r}: contact_frame spin axis disagrees with spin_marker")
                if not defer_mount_validation and not _has_tire_mount(body_name_value, frame_body, joints):
                    raise AuthoringError(f"tire {model_name!r}: contact_frame requires an explicit fixed or revolute mount")
                if model_law.get("kind") not in {"pac2002", "fiala", "native_brush"}:
                    raise AuthoringError(f"tire {model_name!r}: unsupported native tire model {model_law.get('kind')!r}")
                law_scale = 0.001 if model_law.get("units", {}).get("length", "mm") == "mm" else 1.0
                relaxation = float(model_law["relaxation_length"]) or float(model_law["unloaded_radius"])
                brush_scale = 1 if model_law.get("model") == "linear" else 1/(relaxation*law_scale)
                params = {
                    "unloaded_radius": float(model_law["unloaded_radius"])*law_scale,
                    "maximum_compression": float(model_law["maximum_compression"] or .99*model_law["unloaded_radius"])*law_scale,
                    "vertical_stiffness": float(model_law["vertical_stiffness"])/law_scale,
                    "vertical_damping": float(model_law["vertical_damping"])/law_scale,
                    "longitudinal_friction_coefficient": float(model_law["friction_coefficient"]),
                    "lateral_friction_coefficient": float(model_law["friction_coefficient"]),
                    "longitudinal_brush_stiffness": float(model_law["longitudinal_stiffness"])*brush_scale,
                    "lateral_brush_stiffness": float(model_law["cornering_stiffness"])*brush_scale,
                    "longitudinal_relaxation_length": relaxation*law_scale,
                    "lateral_relaxation_length": relaxation*law_scale,
                    "detached_relaxation_s": float(model_law["detached_relaxation_s"]),
                    "parameter_source": model_law["parameter_source"], "mirror": tire.get("mirror", side == "R"),
                    "pac2002_coefficients": model_law["pac2002_coefficients"],
                    "pac2002_tables": model_law["pac2002_tables"], "fiala_parameters": model_law["fiala_parameters"],
                    "_coefficient_length_scale": law_scale,
                }
                for key, factor in (("longitudinal_friction_coefficient", 1), ("lateral_friction_coefficient", 1),
                    ("longitudinal_brush_stiffness", 1/law_scale), ("lateral_brush_stiffness", 1/law_scale),
                    ("longitudinal_relaxation_length", law_scale), ("lateral_relaxation_length", law_scale)):
                    if key in model_law:
                        params[key] = float(model_law[key])*factor
                params.update({"center_local": marker_port.pose.translation.tolist(), "frame_center_local": frame_center,
                               "spin_axis_local": frame_port.pose.rotation[:, 1].tolist(),
                               "forward_axis_local": frame_port.pose.rotation[:, 0].tolist(),
                               "frame_body": frame_body})
                mass = 0.0
                inertia = np.zeros((3, 3))
                if tire.get("mass_slot"):
                    mass_value = self._law(effective, str(tire["mass_slot"])).get("value")
                    if mass_value is None:
                        raise AuthoringError(f"tire {model_name!r}: mass_slot needs a numeric value")
                    mass = float(mass_value)
                if tire.get("inertia_slot"):
                    inertia_law = self._law(effective, str(tire["inertia_slot"]))
                    if inertia_law.get("inertia") is None:
                        raise AuthoringError(f"tire {model_name!r}: inertia_slot needs a tensor")
                    inertia_scale = 0.001 if inertia_law.get("units", {}).get("length", "mm") == "mm" else 1.0
                    inertia = np.asarray(inertia_law["inertia"], dtype=float) * inertia_scale ** 2
                tire_rows.append({"name": _ref(instance, model_name), "model": str(model_law.get("kind", model_law.get("model"))),
                                  "body": body_name_value, "mass": mass, "inertia": inertia.tolist(), "parameters": params})

        fragment = ModelFragment(
            bodies=bodies,
            points=points,
            joints={r["name"]: r for r in joints},
            forces={r["name"]: r for r in elements},
            ports=ports,
            outputs={str(row["name"]): dict(row) for row in data.get("outputs", ())},
            provenance=FragmentProvenance(
                template=template.name,
                revision=template.topology_hash,
                properties_fingerprint=effective.effective_values_hash,
            ),
        )
        coordinates = []
        coordinate_frames = []
        for row in () if geometry_only else data.get("coordinates", ()):
            sources = [joint for joint in joints if joint["name"] in {
                _ref(instance, _side_name(str(row["joint"]), side, {str(row["joint"])}))
                for side in _sides(row, mirrors)
            }]
            for source in sources:
                suffix = source["name"][len(_ref(instance, str(row["joint"]))):]
                name = _ref(instance, str(row["name"]) + suffix)
                frames = {}
                for end in ("a", "b"):
                    frame = name + "." + end
                    coordinate_frames.append({"name": frame, "body": source["body_"+end], "point": source["point_"+end], "quaternion": [1., 0., 0., 0.]})
                    frames["frame_"+end] = frame
                coordinates.append({"name": name, "source_joint_id": source["name"], "kind": row["kind"],
                    **frames, "axis": source["axis_a"], "units": "rad" if row["kind"] == "rotation" else "m",
                    "reference": float(row.get("reference", 0))})
        couplers = []
        for row in () if geometry_only else data.get("couplers", ()):
            for side in _sides(row, mirrors):
                couplers.append({**row, "name": _ref(instance, _side_name(str(row["name"]), side, {str(row["name"])})),
                    **{f"joint_{end}": _ref(instance, _side_name(str(row[f"joint_{end}"]), side, {str(row[f"joint_{end}"])})) for end in ("a", "b")}})
        gauges = []
        for row in data.get("gauges", ()):
            for side in _sides(row, mirrors, row["body"] in sided_bodies):
                axis = _unit_axis(row["axis_local"], "rotation gauge")
                gauges.append({"body": body_name(row["body"], side), "axis_local": (-_MIRROR @ axis if side == "R" else axis).tolist()})
        return GenericMultibodyAssembly(
            effective.name, bodies, tuple(joints), tuple(elements), ports, (fragment,), markers=markers,
            tires=tuple(tire_rows), function_programs=tuple(function_programs),
            inputs=tuple(dict(row) for row in data.get("inputs", ()) if enabled(row)),
            coordinates=tuple(coordinates), coordinate_frames=tuple(coordinate_frames),
            couplers=tuple(couplers), gauges=tuple(gauges), capabilities=tuple(data.get("capabilities", ())), initial_state=initial_state,
        )

    @staticmethod
    def _law(effective: EffectiveSubsystem, slot: str) -> dict[str, Any]:
        if slot in effective.property_bindings:
            return dict(effective.resolved_property(slot))
        declared = effective.template.property_slots[slot]
        if declared.get("default") is None:
            raise AuthoringError(
                f"subsystem {effective.name!r}: slot {slot!r} has no value"
            )
        kind = str(declared["element_type"])
        scalar = "viscous_damping" if kind == "damper" else "stiffness"
        length = 0.001 if declared.get("unit") in ("N/mm", "N*s/mm") else 1.0
        return {
            scalar: float(declared["default"]) / length,
            "value": declared["default"],
            "units": {"length": "m"},
        }

    @staticmethod
    def _axes(
        entry: dict[str, Any],
        row: Mapping[str, Any],
        bodies: Mapping[str, RigidBody],
        *,
        side: str = "",
        axis: Any = None,
        rotation: np.ndarray | None = None,
    ) -> None:
        if axis is None:
            axis = row.get("axis")
        if axis is None:
            raise AuthoringError(
                f"joint {entry['name']!r} needs axis or axis_reference"
            )
        a = _unit_axis(axis, entry["name"])
        if row.get("axis_space") == "body":
            axes = (("a", axis),) if entry["type"] == "inplane" else (("a", axis), ("b", row.get("axis_b", axis)))
            for end, declared in axes:
                vector = _unit_axis(declared, entry["name"])
                if side == "R":
                    vector = (-1 if entry["type"] in _AXIAL_JOINTS else 1)*_MIRROR @ vector
                entry["axis_"+end] = vector.tolist()
            return
        if entry["type"] == "universal" and "axis_b" not in row:
            raise AuthoringError(f"universal joint {entry['name']!r} needs axis_b")
        b = _unit_axis(row.get("axis_b", axis), entry["name"])
        if side == "R":
            sign = -1.0 if entry["type"] in _AXIAL_JOINTS else 1.0
            a, b = sign * _MIRROR @ a, sign * _MIRROR @ b
        if rotation is not None:
            a, b = rotation @ a, rotation @ b
        entry["axis_a"] = (bodies[entry["body_a"]].pose.rotation.T @ a).tolist()
        if entry["type"] != "inplane":
            entry["axis_b"] = (bodies[entry["body_b"]].pose.rotation.T @ b).tolist()

    @staticmethod
    def _element_parameters(
        kind: str,
        law: Mapping[str, Any],
        declared: Mapping[str, Any],
        geometry_scale: float,
    ) -> dict[str, Any]:
        if kind not in {
            "spring",
            "damper",
            "bump_stop",
            "bushing",
            "anti_roll_bar",
            "wheel_torque",
            "rotational_torque",
        }:
            raise AuthoringError(
                f"generic element {kind!r} has no native property adapter"
            )
        scale = 0.001 if law.get("units", {}).get("length", "mm") == "mm" else 1.0
        params = GenericSubsystemAssembler._convert_parameters(kind, law, scale)
        params.update(
            GenericSubsystemAssembler._convert_parameters(
                kind, declared, geometry_scale
            )
        )
        curve = law.get("force_curve")
        if curve:
            key = {
                "spring": "elastic_curve",
                "damper": "damper_curve",
                "bump_stop": "stop_curve",
            }.get(kind)
            if key is None:
                raise AuthoringError(
                    "generic bushing curves require explicit six-axis parameters"
                )
            params[key] = [[float(x) * scale, float(y)] for x, y in curve]
        if kind == "damper":
            damping = params.pop("viscous_damping", None)
            if damping is not None:
                params.setdefault("compression_damping", damping)
                params.setdefault("rebound_damping", damping)
        if kind == "spring" and "free_length" not in params:
            raise AuthoringError(
                "a generic spring needs free_length in its property or element declaration"
            )
        if kind == "bushing":
            if "stiffness" not in params:
                raise AuthoringError("a generic bushing needs stiffness")
            params.setdefault("damping", np.zeros((6, 6)).tolist())
        if kind in {"anti_roll_bar", "wheel_torque", "rotational_torque"} and "axis_a" not in params:
            raise AuthoringError(f"{kind} needs an explicit local axis_a")
        return params

    @staticmethod
    def _convert_parameters(
        kind: str, source: Mapping[str, Any], scale: float
    ) -> dict[str, Any]:
        params: dict[str, Any] = {}
        for key, value in source.items():
            if key in {
                "element_type",
                "model",
                "units",
                "source",
                "value",
                "force_curve",
            }:
                continue
            if key in {"free_length", "clearance", "gas_reference_length"}:
                params[key] = float(value) * scale
            elif key in {
                "stiffness",
                "viscous_damping",
                "damping",
                "compression_damping",
                "rebound_damping",
                "gas_stiffness",
            }:
                if kind == "bushing":
                    matrix = np.asarray(value, dtype=float)
                    if matrix.ndim == 0:
                        matrix = np.diag([float(matrix)] * 3 + [0.0] * 3)
                    if matrix.shape != (6, 6) or not np.isfinite(matrix).all():
                        raise AuthoringError(
                            "bushing stiffness/damping must be a finite 6x6 matrix"
                        )
                    matrix = matrix.copy()
                    matrix[:3, :3] /= scale
                    matrix[3:, 3:] *= scale
                    params[key] = matrix.tolist()
                else:
                    params[key] = (
                        float(value) * scale
                        if kind in {"anti_roll_bar", "wheel_torque", "rotational_torque"}
                        else float(value) / scale
                    )
            elif key == "max_torque":
                params[key] = float(value) * scale
            elif key == "reference_translation":
                params[key] = (_vector(value, key) * scale).tolist()
            elif key == "preload" and kind == "bushing":
                preload = np.asarray(value, dtype=float).copy()
                if preload.shape != (6,) or not np.isfinite(preload).all():
                    raise AuthoringError(
                        "bushing preload must contain six finite values"
                    )
                preload[3:] *= scale
                params[key] = preload.tolist()
            elif key in {"elastic_curve", "damper_curve", "stop_curve"}:
                params[key] = [[float(x) * scale, float(y)] for x, y in value]
            elif key == "force_curves" and kind == "bushing":
                params[key] = [
                    [[float(x)*(scale if axis < 3 else 1), float(y)*(1 if axis < 3 else scale)] for x, y in curve]
                    for axis, curve in enumerate(value)]
            else:
                params[key] = value
        return params

    @staticmethod
    def _mirror_parameters(params: dict[str, Any]) -> None:
        # Translation is a polar vector; rotation and moment are axial vectors.
        transform = np.diag([1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
        for key in ("stiffness", "damping"):
            value = np.asarray(params.get(key, 0.0))
            if value.shape == (6, 6):
                params[key] = (transform @ value @ transform).tolist()
        if "preload" in params and np.asarray(params["preload"]).shape == (6,):
            params["preload"] = (transform @ np.asarray(params["preload"])).tolist()
        if "reference_translation" in params:
            params["reference_translation"] = (
                _MIRROR @ np.asarray(params["reference_translation"])
            ).tolist()
        for key in ("frame_a_quaternion", "frame_b_quaternion", "reference_quaternion"):
            if key in params:
                params[key] = (
                    np.asarray(params[key]) * (1.0, -1.0, 1.0, -1.0)
                ).tolist()
        for key in ("axis_a", "axis_b"):
            if key in params:
                params[key] = (-_MIRROR @ _unit_axis(params[key], key)).tolist()


def assemble_generic(document: AssemblyDocument) -> GenericMultibodyAssembly:
    """Compose subsystem graphs using explicit, typed port connections."""
    if document.assembly_kind != "generic_multibody":
        raise AuthoringError("assemble_generic requires a generic_multibody assembly")
    mode = str(document.payload.get("mode", "K"))
    assembler = GenericSubsystemAssembler()
    placements = {
        str(row["ref"]): SE3(
            _vector(row.get("placement", {}).get("translation", _ZERO), "placement translation"),
            np.asarray(row.get("placement", {}).get("quaternion", (1, 0, 0, 0)), dtype=float),
        ) for row in document.payload["subsystems"]
    }
    contributions = [
        assembler.assemble(entry.effective(), mode=mode, instance=entry.ref, geometry_only=True, placement=placements[entry.ref])
        for entry in document.entries
    ]
    bodies: dict[str, RigidBody] = {}
    ports: dict[str, PortSpec] = {}
    markers: dict[str, GeometryPort] = {}
    tires: list[dict[str, Any]] = []
    joints: list[dict[str, Any]] = []
    elements: list[dict[str, Any]] = []
    function_programs: list[dict[str, Any]] = []
    inputs: list[dict[str, Any]] = []
    coordinates: list[dict[str, Any]] = []
    coordinate_frames: list[dict[str, Any]] = []
    couplers: list[dict[str, Any]] = []
    gauges: list[dict[str, Any]] = []
    capabilities: set[str] = set()
    initial_state: dict[str, Mapping[str, Any]] = {}
    for contribution in contributions:
        if set(bodies) & set(contribution.bodies) or set(ports) & set(
            contribution.ports
        ):
            raise AuthoringError("generic assembly repeats a subsystem reference")
        bodies.update(contribution.bodies)
        ports.update(contribution.ports)
        markers.update(contribution.markers)
    bindings: dict[str, MatchReport] = {}
    bound_ports: dict[str, dict[str, PortSpec]] = {}
    consumers: dict[str, str] = {}
    fragments: list[ModelFragment] = []
    for entry, contribution in zip(document.entries, contributions):
        needs = tuple(
            PortRequirement(
                role=str(row["role"]),
                count=int(row["count"]),
                required=bool(row["required"]),
                requires_capabilities=frozenset(row.get("requires_capabilities", ())),
                match_labels=frozenset(row.get("match_labels", ())),
                bound_outputs=tuple(row.get("bound_outputs", ())),
                kind=row.get("kind"),
                units=str(row.get("units", "")),
                family=str(row.get("family", "")),
                name=str(row.get("name", "")),
            )
            for row in entry.subsystem.template.payload.get("needs", ())
        )
        candidates = {
            name: port for name, port in ports.items() if name not in contribution.ports
        }
        for requirement in needs:
            chosen = entry.pairings.get(requirement.key)
            if chosen in candidates and not requirement.accepts(candidates[chosen]):
                raise AuthoringError(
                    f"requirement {requirement.role!r}: explicit port {chosen!r} "
                    "has incompatible role, capabilities or labels"
                )
        report = match_requirements(needs, candidates, explicit=entry.pairings)
        bindings[entry.ref] = report
        bound_ports[entry.ref] = {}
        for binding in report.bindings:
            for port_id in binding.port_ids:
                if port_id in consumers and ports[port_id].cardinality == "one":
                    raise AuthoringError(f"port {port_id!r} accepts one connection; already bound by {consumers[port_id]!r}")
                consumers[port_id] = entry.ref + "." + binding.requirement.key
            if len(binding.port_ids) == 1:
                bound_ports[entry.ref][binding.requirement.key] = ports[binding.port_ids[0]]
    contributions = [
        assembler.assemble(
            entry.effective(), mode=mode, instance=entry.ref,
            external_ports=bound_ports[entry.ref], external_bodies=bodies,
            defer_mount_validation=True,
            placement=placements[entry.ref],
        )
        for entry in document.entries
    ]
    for entry, contribution in zip(document.entries, contributions):
        joints.extend(contribution.joints)
        offset = len(function_programs)
        for declared in contribution.elements:
            element = dict(declared)
            if element["type"] in {"force", "torque", "wrench"}:
                params = dict(element["parameters"])
                if "program_id" in params:
                    params["program_id"] += offset
                if "program_ids" in params:
                    params["program_ids"] = [value + offset for value in params["program_ids"]]
                element["parameters"] = params
            elements.append(element)
        function_programs.extend(contribution.function_programs)
        tires.extend(contribution.tires)
        coordinates.extend(contribution.coordinates)
        coordinate_frames.extend(contribution.coordinate_frames)
        couplers.extend(contribution.couplers)
        gauges.extend(contribution.gauges)
        capabilities.update(contribution.capabilities)
        initial_state.update(contribution.initial_state)
        report = bindings[entry.ref]
        for row in contribution.inputs:
            input_row = dict(row)
            input_row["name"] = _ref(entry.ref, str(row["name"]))
            active_inputs = {str(item["name"]) for item in contribution.inputs}
            input_row["coupled_with"] = [_ref(entry.ref, name) for name in row.get("coupled_with", ()) if name in active_inputs]
            reference = str(row.get("port", ""))
            if reference.startswith("@"):
                bound = bound_ports[entry.ref][reference[1:]]
                input_row["port"] = str(bound.id)
                input_row["owner"] = bound.owner.local
            inputs.append(input_row)
        fragments.extend(
            replace(
                fragment,
                outputs={
                    name: output
                    for name, output in fragment.outputs.items()
                    if name not in report.dropped_outputs
                },
                requirements=tuple(binding.requirement for binding in report.bindings),
            )
            for fragment in contribution.fragments
        )
    names = {row["name"] for row in joints + elements}
    for row in document.payload.get("connections", ()):
        if not _active(row, mode):
            continue
        name = str(row["name"])
        if name in names:
            raise AuthoringError(f"duplicate connection name {name!r}")
        names.add(name)
        try:
            a, b = ports[str(row["port_a"])], ports[str(row["port_b"])]
        except KeyError as exc:
            raise AuthoringError(
                f"connection {name!r}: unknown port {exc.args[0]!r}"
            ) from exc
        if not isinstance(a, GeometryPort) or not isinstance(b, GeometryPort):
            raise AuthoringError(f"connection {name!r} requires two geometry ports")
        if a.family and b.family and a.family != b.family:
            raise AuthoringError(f"connection {name!r}: incompatible port families")
        if a.owner.local == b.owner.local:
            raise AuthoringError(f"connection {name!r} joins a body to itself")
        kind = str(row["type"])
        physical = {
            "name": name,
            "type": kind,
            "body_a": a.owner.local,
            "body_b": b.owner.local,
            "point_a": a.pose.translation.tolist(),
            "point_b": b.pose.translation.tolist(),
        }
        if kind == "bushing":
            params = assembler._element_parameters(
                "bushing",
                {
                    **row.get("parameters", {}),
                    "units": row.get("units", {"length": "m"}),
                },
                {},
                1.0,
            )
            params.update(
                point_a=physical.pop("point_a"), point_b=physical.pop("point_b")
            )
            physical["parameters"] = params
            elements.append(physical)
        else:
            if kind in _AXIS_JOINTS:
                assembler._axes(physical, row, bodies)
            if kind.startswith("driven_"):
                if not row.get("target"):
                    raise AuthoringError(f"driven connection {name!r} needs target")
                physical["target"] = str(row["target"])
            joints.append(physical)
    result = GenericMultibodyAssembly(
        document.name,
        bodies,
        tuple(joints),
        tuple(elements),
        ports,
        tuple(fragments),
        tuple(
            _vector(document.payload.get("gravity", (0.0, 0.0, -9.80665)), "gravity")
        ),
        bindings,
        markers,
        tuple(tires),
        tuple(function_programs),
        tuple(inputs),
        tuple(coordinates),
        tuple(coordinate_frames),
        tuple(document.payload.get("measurements", ())),
        tuple(couplers) + tuple(document.payload.get("couplers", ())), tuple(gauges), tuple(sorted(capabilities)), initial_state,
        document.payload.get("road"),
    )
    for tire in tires:
        owner, carrier = tire["body"], tire["parameters"]["frame_body"]
        if not _has_tire_mount(owner, carrier, joints):
            raise AuthoringError(f"tire {tire['name']!r}: contact_frame requires an explicit fixed or revolute mount")
    result.model_document()
    return result
