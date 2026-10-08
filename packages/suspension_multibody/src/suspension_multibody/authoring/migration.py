"""Offline conversion of v1 declarations into ordinary authoring data."""

from __future__ import annotations

import json
from collections import Counter
from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from suspension_contracts import validate_case

from ..axle_dynamics.schema import AxleDynamicsCase, AxleDynamicsModel
from ..modeling.primitives.spatial import SE3
from ..schema.common import UnitSystem, Vec3
from ..schema.model import AxleDeclaration
from ..schema.solver import AxleSolverSettings
from ..schema.vehicle import VehicleDeclaration, VehicleDynamicCase
from ..templates.builtin import (
    BRAKE,
    DOUBLE_WISHBONE,
    DRIVE,
    RACK_HOUSING_MASS,
    TORQUE_PARAMETER_RECIPES,
    WHEEL_HUB_MASS,
)
from .documents import AssemblyDocument, SubsystemDocument, TemplateDocument
from .errors import AuthoringError
from .loader import CaseDocument
from .properties import ElementPropertyDocument

_POINT_ALIASES = {
    "upper_front": ("UPPER_INBOARD_FRONT", "UPPER_INNER_FRONT", "UCA_FRONT", "UCA_INNER_FRONT", "UPPER_FRONT"),
    "upper_rear": ("UPPER_INBOARD_REAR", "UPPER_INNER_REAR", "UCA_REAR", "UCA_INNER_REAR", "UPPER_REAR"),
    "upper_outer": ("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER"),
    "lower_front": ("LOWER_INBOARD_FRONT", "LOWER_INNER_FRONT", "LCA_FRONT", "LCA_INNER_FRONT", "LOWER_FRONT"),
    "lower_rear": ("LOWER_INBOARD_REAR", "LOWER_INNER_REAR", "LCA_REAR", "LCA_INNER_REAR", "LOWER_REAR"),
    "lower_outer": ("LOWER_OUTBOARD", "LOWER_OUTER", "LCA_OUTER"),
    "tie_inner": ("TIE_ROD_INBOARD", "TIE_ROD_INNER", "TIEROD_INNER", "RACK_TIE_ROD", "TIE_INNER"),
    "tie_outer": ("TIE_ROD_OUTBOARD", "TIE_ROD_OUTER", "TIEROD_OUTER", "TIE_OUTER"),
    "wheel_center": ("WHEEL_CENTER", "WHEEL_CENTRE", "WHEEL_CG"),
    "rack_center": ("RACK_CENTER", "RACK_CENTRE", "RACK_REFERENCE"),
}


def _proxy_declaration(model: AxleDeclaration, mode: str) -> AxleDeclaration:
    """Expand the v1 declaration table without calling an assembly builder."""
    normalized = {name.upper().replace("-", "_"): value.as_array() for name, value in model.hardpoints.items()}

    def point(role: str, side: str) -> list[float]:
        aliases = _POINT_ALIASES[role]
        if side == "R":
            explicit = next((normalized[key + "__R"] for key in aliases if key + "__R" in normalized), None)
            if explicit is not None:
                return explicit.tolist()
        value = next((normalized[key] for key in aliases if key in normalized), None)
        if value is None:
            raise MigrationError(f"v1 proxy is missing hardpoint {role!r}")
        return (value * [1, -1 if side == "R" else 1, 1]).tolist()

    source = model.model_dump(mode="json")
    specs = {row["name"]: row for row in source["bodies"]}
    names = {part.name for part in DOUBLE_WISHBONE.parts} | {"chassis", "rack", "rack_housing"}
    bodies = []
    for name in ("rack", *(part.name for part in DOUBLE_WISHBONE.parts), "rack_housing"):
        row = deepcopy(specs.get(name, {"name": name, "mass": 0, "inertia": np.eye(3).tolist()}))
        # v1's native reader substituted 1 kg for an undeclared free-body mass.
        # Make that effective input explicit rather than leaving a runtime fallback.
        if name not in specs and name.startswith("wheel_hub_"):
            row.update(mass=WHEEL_HUB_MASS, inertia=(np.eye(3)*WHEEL_HUB_MASS*64).tolist(),
                center_of_mass=point("wheel_center", name[-1]))
        elif name not in specs and name == "rack_housing":
            row.update(mass=RACK_HOUSING_MASS, inertia=(np.eye(3)*RACK_HOUSING_MASS*64).tolist(),
                center_of_mass=point("rack_center", "L"))
        elif not row.get("fixed") and row.get("mass", 0) == 0:
            row["mass"] = 1
        bodies.append(row)

    joints = []
    placeholders = []
    for connection in DOUBLE_WISHBONE.connections:
        if not connection.owner or not connection.far_owner:
            continue
        side = connection.owner[-1]
        if side not in {"L", "R"}:
            continue
        location = point(connection.role, side)
        if connection.active_column(mode) == "bushing":
            placeholders.append({"name": connection.bushing, "body_a": connection.far_owner,
                "body_b": connection.owner, "pose_a": {"translation": location}, "pose_b": {"translation": location},
                "stiffness": np.zeros((6, 6)).tolist()})
        elif connection.active_column(mode) == "joint":
            a, b = connection.owner, connection.far_owner
            if connection.first_body == "far":
                a, b = b, a
            row = {"name": connection.name, "kind": connection.joint_kind(mode),
                "body_a": a, "body_b": b, "point_a": location, "point_b": location}
            if connection.axis_reference_role:
                axis = np.asarray(point(connection.axis_reference_role, side)) - location
                row.update(axis_a=axis.tolist(), axis_b=axis.tolist())
            elif connection.axis is not None:
                row.update(axis_a=list(connection.axis), axis_b=list(connection.axis))
            joints.append(row)
    center = point("rack_center", "L")
    joints.extend([
        {"name": "rack_fixed_to_chassis" if model.rack_fixed_to_chassis else "rack_guide",
         "kind": "fixed" if model.rack_fixed_to_chassis else "prismatic",
         "body_a": "chassis" if model.rack_fixed_to_chassis else "rack_housing", "body_b": "rack",
         "point_a": center, "point_b": center, "axis_a": model.rack_axis.as_tuple(), "axis_b": model.rack_axis.as_tuple()},
        {"name": "housing_mount", "kind": "fixed", "body_a": "chassis", "body_b": "rack_housing", "point_a": center, "point_b": center},
    ])

    def body(name: str, side: str) -> str:
        if name in names:
            return name
        normalized_name = name.strip().lower().replace("-", "_")
        if normalized_name in {"chassis", "vehicle_body"}:
            return "chassis"
        for suffix in ("_l", "_r", " left", " right"):
            if normalized_name.endswith(suffix):
                normalized_name = normalized_name[:-len(suffix)].rstrip()
                break
        candidate = normalized_name + "_" + side
        if candidate not in names:
            raise MigrationError(f"v1 element names unknown body {name!r}")
        return candidate

    for key in ("springs", "dampers", "stops", "bushings"):
        rows = []
        for side in ("L", "R"):
            for original in source[key]:
                row = deepcopy(original)
                row["name"] += "_" + side
                for end in ("a", "b"):
                    row["body_" + end] = body(row["body_" + end], side)
                    field = "pose_" + end if key == "bushings" else "point_" + end
                    value = Vec3.model_validate(row[field]["translation"] if key == "bushings" else row[field]).as_array().tolist()
                    if side == "R":
                        value[1] *= -1
                    if key == "bushings":
                        row[field]["translation"] = value
                    else:
                        row[field] = value
                rows.append(row)
        source[key] = rows
    source["bushings"].extend(placeholders)
    source.update(topology="explicit", bodies=bodies, joints=joints)
    return AxleDeclaration.model_validate(source)


class MigrationError(AuthoringError):
    """An input cannot be converted without supplying additional physical data."""


def _partition_axle(model: AxleDeclaration, payload: dict[str, Any], hardpoints: dict[str, Any], mode: str) -> AssemblyDocument:
    """Keep wheel and bar ownership explicit in the migrated document set."""
    spin_joints = {row["body_b"]: row for row in payload["joints"] if row["name"].startswith("wheel_spin_joint_")}
    wheel_bodies = set(spin_joints)
    declared_centers = {key.upper() for key in model.hardpoints}.intersection(_POINT_ALIASES["wheel_center"])
    if not wheel_bodies and (model.tires or declared_centers):
        available = {row["name"] for row in payload["bodies"]}
        wheel_bodies = {name for name in ("upright_L", "upright_R") if name in available}
        if len(wheel_bodies) != 2:
            raise MigrationError("v1 wheel centers require declared left/right owners")
    groups: dict[str, dict[str, Any]] = {}
    values: dict[str, dict[str, Any]] = {}
    laws: dict[str, dict[str, ElementPropertyDocument]] = {}
    for role in ("model", "wheel", "anti_roll_bar"):
        groups[role] = {**deepcopy(payload), "name": model.name + "_" + role,
            "functional_role": {"model": "generic", "wheel": "wheel", "anti_roll_bar": "anti_roll_bar"}[role],
            "bodies": [], "joints": [], "elements": [], "markers": [], "ports": [], "needs": [], "coordinates": [], "gauges": []}
        values[role] = deepcopy(hardpoints)
        laws[role] = {}
    owners = {row["name"]: "wheel" if row["name"] in wheel_bodies else "model" for row in payload["bodies"]}
    for gauge in payload.get("gauges", ()):
        groups[owners[gauge["body"]]]["gauges"].append(gauge)
    for row in payload["bodies"]:
        owner = owners[row["name"]]
        groups[owner]["bodies"].append(row)
        key = "body_" + row["name"]
        groups[owner]["ports"].append({"name": key, "role": key, "owner": row["name"], "point": "reference", "cardinality": "many"})
    for role in groups:
        groups[role]["hardpoints"].append({"name": "reference"})
        values[role]["reference"] = [0, 0, 0]

    def bind(role: str, row: dict[str, Any]) -> dict[str, Any]:
        row = deepcopy(row)
        for end in ("a", "b"):
            name = row["body_" + end]
            if owners[name] == role:
                continue
            key = "body_" + name
            if not any(need["name"] == key for need in groups[role]["needs"]):
                groups[role]["needs"].append({"name": key, "role": key, "count": 1, "required": True})
            row["body_" + end] = "@" + key
        return row

    for row in payload["joints"]:
        role = "wheel" if row["body_b"] in spin_joints and row["name"] == spin_joints[row["body_b"]]["name"] else "model"
        groups[role]["joints"].append(bind(role, row))
    groups["model"]["elements"] = [bind("model", row) for row in payload["elements"]]
    for role in groups:
        required = {row[field] for row in (*groups[role]["joints"], *groups[role]["elements"])
            for field in ("point_a", "point_b") if field in row}
        groups[role]["hardpoints"] = [row for row in groups[role]["hardpoints"]
            if row["name"] in required or row["name"] == "reference"]
        for point_row in groups[role]["hardpoints"]:
            owner = point_row.get("owner")
            if owner and owners[owner] != role:
                key = "body_"+owner
                if not any(row["name"] == key for row in groups[role]["needs"]):
                    groups[role]["needs"].append({"name": key, "role": key, "count": 1, "required": True})
                point_row["owner"] = "@"+key
    for name, joint in spin_joints.items():
        side = name[-1]
        coordinate = "spin_" + side
        groups["wheel"]["coordinates"].append({"name": coordinate, "joint": joint["name"], "kind": "rotation"})
        groups["wheel"]["ports"].append({"name": coordinate, "role": "wheel_spin", "kind": "spin", "owner": name,
            "coordinate": coordinate, "labels": [side], "cardinality": "many"})
    for name in sorted(wheel_bodies):
        side = name[-1]
        joint = spin_joints.get(name)
        if joint:
            source_joint = next(row for row in model.joints if row.name == joint["name"])
            center = source_joint.point_b.as_array().tolist()
            carrier = joint["body_a"]
        else:
            aliases = _POINT_ALIASES["wheel_center"]
            declared = {key.upper(): value.as_array() for key, value in model.hardpoints.items()}
            center_value = next((declared[key] for key in aliases if key in declared), None)
            if center_value is None:
                raise MigrationError("vertical tire input requires its wheel-center hardpoint")
            center = (center_value * [1, -1 if side == "R" else 1, 1]).tolist()
            carrier = name
        key = "wheel_center_" + side
        groups["wheel"]["hardpoints"].append({"name": key})
        values["wheel"][key] = center
        groups["wheel"]["markers"].append({"name": key, "owner": name, "point": key})
        groups["wheel"]["ports"].append({"name": key, "role": "wheel_hub", "marker": key, "labels": [side], "cardinality": "many"})
        if not model.tires:
            continue
        port = "contact_frame_" + side
        groups[owners[carrier]]["hardpoints"].append({"name": port})
        values[owners[carrier]][port] = center
        groups[owners[carrier]]["ports"].append({"name": port, "role": port, "owner": carrier, "point": port, "cardinality": "many"})
        contact = port
        if owners[carrier] != "wheel":
            groups["wheel"]["needs"].append({"name": port, "role": port, "count": 1, "required": True})
            contact = "@" + port
        for index, spec in enumerate(model.tires):
            slot = f"tire_{index}"
            if slot not in laws["wheel"]:
                laws["wheel"][slot] = ElementPropertyDocument.from_payload({
                    "document": "element_properties", "schema_version": 1, "name": slot, "element_type": "tire", "model": "linear",
                    "units": {"length": payload["units"]["length"], "force": "N"},
                    "parameters": {"stiffness": spec.stiffness, "unloaded_radius": spec.unloaded_radius}})
                groups["wheel"]["property_slots"].append({"name": slot, "element_type": "tire", "required": True, "allowed_models": ["linear"]})
            groups["wheel"].setdefault("tires", []).append({"name": f"tire_{index}_{side}", "body": {"name": name, "center_marker": key},
                "model_slot": slot, "mount_port": key, "spin_marker": key, "contact_frame": {"port": contact}, "road_port": "road"})
    if model.tires:
        groups["wheel"]["ports"].append({"name": "road", "role": "road", "kind": "road", "resource": "plane", "cardinality": "many"})
    for spec in model.anti_roll_bars:
        row = {"name": spec.name, "type": "bushing", "body_a": "upright_L", "body_b": "upright_R", "property_slot": "parameters",
            "parameters": {"stiffness": np.diag([0, 0, spec.torsional_stiffness, 0, 0, 0]).tolist(), "damping": np.zeros((6, 6)).tolist()}}
        for end, point in (("a", spec.left_link_point), ("b", spec.right_link_point)):
            key = spec.name + "_" + end
            groups["anti_roll_bar"]["hardpoints"].append({"name": key})
            values["anti_roll_bar"][key] = point.as_array().tolist()
            row["point_" + end] = key
        groups["anti_roll_bar"]["elements"].append(bind("anti_roll_bar", row))
    documents = {}
    entries = []
    for role, template_payload in groups.items():
        if not template_payload["bodies"] and not template_payload["elements"]:
            continue
        template = TemplateDocument.from_payload(template_payload)
        values[role] = {key: value for key, value in values[role].items() if key in template.hardpoint_names}
        properties = {slot: slot + ".property.json" for slot in laws[role]}
        ref = role + ".sub.json"
        documents[ref] = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
            "name": template.name, "template": role + ".tpl.json", "functional_role": template.functional_role,
            "placement_role": "any", "hardpoints": values[role], "property_bindings": properties}, template=template, properties=laws[role])
        entries.append({"ref": ref, "functional_role": template.functional_role, "placement_role": "any"})
    return AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1, "name": model.name,
        "assembly_kind": "generic_multibody", "mode": mode, "subsystems": entries}, subsystems=documents)


def migrate_v1_axle(source: str | Path | Mapping[str, Any] | AxleDeclaration, *, mode: str = "K") -> AssemblyDocument:
    """Convert declared geometry and laws without assembling or running the old model."""
    if isinstance(source, (str, Path)):
        path = Path(source)
        text = path.read_text(encoding="utf-8")
        source = json.loads(text) if path.suffix.lower() == ".json" else yaml.safe_load(text)
    model = source if isinstance(source, AxleDeclaration) else AxleDeclaration.model_validate(source)
    if mode not in {"K", "C"}:
        raise MigrationError("migration mode must be K or C")
    if model.topology == "symmetric_proxy":
        variants = {candidate: _migrate_explicit(_proxy_declaration(model, candidate), candidate) for candidate in ("K", "C")}
        documents = {}
        for entry in variants["K"].entries:
            other = next(item for item in variants["C"].entries if item.ref == entry.ref)
            payload = entry.subsystem.template.to_payload()
            other_points = {"compliant_" + key: value for key, value in other.subsystem.payload["hardpoints"].items()}
            points = {row["name"]: row for row in payload["hardpoints"]}
            points.update({"compliant_" + row["name"]: {**row, "name": "compliant_" + row["name"]} for row in other.subsystem.template.payload["hardpoints"]})
            payload["hardpoints"] = list(points.values())
            left_points = {row["name"]: row for row in payload["hardpoints"]}
            right_points = {row["name"]: row for row in other.subsystem.template.payload["hardpoints"]}
            def point_identity(key: str, value: Any, rows: Mapping[str, Any]) -> tuple[Any, ...]:
                row = rows.get(key, {})
                return (row.get("owner"), row.get("space"), *map(float, value))
            canonical = {point_identity(key, value, left_points): key
                for key, value in entry.subsystem.payload["hardpoints"].items() if key in left_points}
            other_template = deepcopy(other.subsystem.template.payload)
            for row in (*other_template["joints"], *other_template["elements"]):
                for field in ("point_a", "point_b"):
                    value = row.get(field)
                    if value in other.subsystem.payload["hardpoints"]:
                        coords = other.subsystem.payload["hardpoints"][value]
                        row[field] = canonical.get(point_identity(value, coords, right_points), "compliant_" + value)
            for row in (*payload["joints"], *payload["elements"]):
                for field in ("point_a", "point_b"):
                    if row.get(field) in entry.subsystem.payload["hardpoints"]:
                        coords = entry.subsystem.payload["hardpoints"][row[field]]
                        row[field] = canonical[point_identity(row[field], coords, left_points)]
            for section in ("joints", "elements"):
                left = {row["name"]: row for row in payload[section]}
                right = {row["name"]: row for row in other_template[section]}
                rows = []
                for name in dict.fromkeys([*left, *right]):
                    if name not in left or name not in right:
                        rows.append({**deepcopy(left.get(name, right.get(name))), "modes": ["K" if name in left else "C"]})
                    elif left[name] == right[name]:
                        rows.append(left[name])
                    else:
                        raise MigrationError(f"proxy row {name!r} changes parameters between modes")
                payload[section] = rows
            subsystem_payload = entry.subsystem.to_payload()
            subsystem_payload["hardpoints"].update(other_points)
            documents[entry.ref] = SubsystemDocument.from_payload(subsystem_payload,
                template=TemplateDocument.from_payload(payload), properties=entry.subsystem.properties)
        assembly = variants[mode].to_payload()
        return _with_road(AssemblyDocument.from_payload(assembly, subsystems=documents), model)
    return _migrate_explicit(model, mode)


def _migrate_explicit(model: AxleDeclaration, mode: str) -> AssemblyDocument:
    specs = {spec.name: spec for spec in model.bodies}
    bodies: list[dict[str, Any]] = [{"name": "chassis", "fixed": True, "mass": 0}]
    poses = {"chassis": SE3.identity()}
    gauges = []
    for name, spec in specs.items():
        if name == "chassis":
            raise MigrationError("explicit v1 axle cannot redefine chassis")
        if not spec.fixed and spec.mass <= 0:
            raise MigrationError(f"body {name!r}: free body has no declared mass")
        poses[name] = SE3(spec.pose.translation.as_array(), np.asarray(spec.pose.rotation.as_tuple()))
        bodies.append({"name": name, "mass": spec.mass, "fixed": spec.fixed,
            "position": spec.pose.translation.as_tuple(), "quaternion": spec.pose.rotation.as_tuple(),
            "center_of_mass": spec.center_of_mass.as_tuple(), "inertia": spec.inertia})
        if spec.static_rotation_axis_local is not None:
            gauges.append({"body": name, "axis_local": spec.static_rotation_axis_local.as_array().tolist()})
    hardpoints: dict[str, list[float]] = {}
    point_rows: list[dict[str, str]] = []

    def point(name: str, value: Any) -> str:
        key = f"attachment_{len(point_rows)}"
        if name not in poses:
            raise MigrationError(f"attachment names unknown body {name!r}")
        local = poses[name].inverse().transform_point(np.asarray(value, dtype=float))
        if name in specs:
            local -= specs[name].center_of_mass.as_array()
        hardpoints[key] = local.tolist()
        point_rows.append({"name": key, "owner": name, "space": "body"})
        return key

    joints = []
    for joint in model.joints:
        entry = {"name": joint.name, "type": "convel" if joint.kind == "constant_velocity" else joint.kind,
            "body_a": joint.body_a, "body_b": joint.body_b,
            "point_a": point(joint.body_a, joint.point_a.as_array()), "point_b": point(joint.body_b, joint.point_b.as_array())}
        if joint.kind not in {"fixed", "spherical"}:
            entry.update(axis=joint.axis_a.as_tuple(), axis_b=joint.axis_b.as_tuple())
        if joint.kind == "constant_velocity":
            entry.update(axis_a_secondary=joint.axis_a_secondary.as_tuple(),
                axis_b_secondary=joint.axis_b_secondary.as_tuple(), convel_angle_target=joint.constant_velocity_angle_target)
        joints.append(entry)
    if "rack" in specs and "rack_housing" not in specs:
        normalized = {name.upper().replace("-", "_"): value for name, value in model.hardpoints.items()}
        rack_center = next((normalized[name] for name in ("RACK_CENTER", "RACK_CENTRE", "RACK_REFERENCE") if name in normalized), None)
        if rack_center is None:
            raise MigrationError("explicit v1 rack requires its declared rack center")
        joint = {"name": "rack_fixed_to_chassis" if model.rack_fixed_to_chassis else "rack_guide",
            "type": "fixed" if model.rack_fixed_to_chassis else "prismatic", "body_a": "chassis", "body_b": "rack",
            "point_a": point("chassis", rack_center.as_array()), "point_b": point("rack", rack_center.as_array())}
        if not model.rack_fixed_to_chassis:
            joint["axis"] = model.rack_axis.as_tuple()
        joints.append(joint)
    elements = []
    for kind, values in (("spring", model.springs), ("damper", model.dampers), ("bump_stop", model.stops)):
        for spec in values:
            params = spec.model_dump(mode="json", exclude={"name", "body_a", "body_b", "point_a", "point_b"})
            params = {key: value for key, value in params.items() if value is not None}
            if kind == "spring":
                if spec.free_length is not None:
                    params["free_length"] = spec.free_length
                else:
                    assert spec.reference_length is not None and spec.preload is not None
                    params["free_length"] = spec.reference_length - spec.preload/spec.stiffness
                params.pop("preload", None)
                params.pop("reference_length", None)
            if kind == "damper":
                params["compression_damping"] = params["rebound_damping"] = params.pop("viscous_damping")
            if kind == "bump_stop":
                params["direction"] = 1 if params["direction"] == "bump" else -1
            curve = params.pop("force_curve", ())
            if curve:
                if kind == "spring":
                    curve = [(-extension, -force) for extension, force in reversed(curve)]
                params[{"spring": "elastic_curve", "damper": "damper_curve", "bump_stop": "stop_curve"}[kind]] = curve
            elements.append({"name": spec.name, "type": kind, "body_a": spec.body_a, "body_b": spec.body_b,
                "point_a": point(spec.body_a, spec.point_a.as_array()), "point_b": point(spec.body_b, spec.point_b.as_array()),
                "property_slot": "parameters", "parameters": params})
    for spec in model.bushings:
        parameters = {"stiffness": spec.stiffness, "damping": np.diag(spec.damping).tolist(), "preload": spec.preload,
            "frame_a_quaternion": spec.pose_a.rotation.as_tuple(), "frame_b_quaternion": spec.pose_b.rotation.as_tuple(),
            "rotation_coordinates": spec.rotation_coordinates}
        if spec.force_curves:
            parameters.update(force_curves=spec.force_curves, force_curve_interpolation=spec.force_curve_interpolation)
        elements.append({"name": spec.name, "type": "bushing", "body_a": spec.body_a, "body_b": spec.body_b,
            "point_a": point(spec.body_a, spec.pose_a.translation.as_array()), "point_b": point(spec.body_b, spec.pose_b.translation.as_array()),
            "property_slot": "parameters", "modes": ["C"],
            "parameters": parameters})
    payload = json.loads(json.dumps({
        "document": "template", "schema_version": 1, "name": model.name, "functional_role": "generic",
        "allowed_placement_roles": ["any"], "symmetry": "asymmetric",
        "units": {"length": "mm" if model.units == UnitSystem.ENGINEERING else "m"},
        "bodies": bodies, "hardpoints": point_rows, "joints": joints, "elements": elements,
        "gauges": gauges,
        "property_slots": [{"name": "parameters", "element_type": "generic", "required": False, "default": 0}],
    }))
    wheel_centers = {key.upper() for key in model.hardpoints}.intersection(_POINT_ALIASES["wheel_center"])
    compound_wheels = bool(wheel_centers) and {"upright_L", "upright_R"} <= specs.keys()
    if model.tires or model.anti_roll_bars or compound_wheels or any(joint.name.startswith("wheel_spin_joint_") for joint in model.joints):
        document = _partition_axle(model, payload, hardpoints, mode)
        return _with_road(document, model)
    template = TemplateDocument.from_payload(payload)
    subsystem = SubsystemDocument.from_payload({
        "document": "subsystem", "schema_version": 1, "name": model.name, "template": "model.tpl.json",
        "functional_role": "generic", "placement_role": "any", "hardpoints": hardpoints, "property_bindings": {},
    }, template=template)
    document = AssemblyDocument.from_payload({
        "document": "assembly", "schema_version": 1, "name": model.name, "assembly_kind": "generic_multibody",
        "mode": mode, "subsystems": [{"ref": "model.sub.json", "functional_role": "generic", "placement_role": "any"}],
    }, subsystems={"model.sub.json": subsystem})
    return _with_road(document, model)


def _with_road(document: AssemblyDocument, model: AxleDeclaration) -> AssemblyDocument:
    if model.road is None:
        return document
    if model.road.corner_height_signals:
        raise MigrationError("sampled road signals must be migrated into a SolvePlan")
    scale = .001 if model.units == UnitSystem.ENGINEERING else 1
    parameters = model.road.model_dump(mode="json", exclude={"kind", "corner_height_signals"})
    for key in ("amplitude", "wavelength", "bump_start", "bump_length"):
        parameters[key] *= scale
    parameters["origin"] = (model.road.origin.as_array()*scale).tolist()
    payload = document.to_payload()
    payload["road"] = {"kind": model.road.kind, "parameters": parameters}
    return AssemblyDocument.from_payload(payload, subsystems={entry.ref: entry.subsystem for entry in document.entries})


def migrate_v1_kc_case(
    model: AxleDeclaration, *, mode: str, name: str = "kc",
    wheel_values_mm: tuple[float, ...] = (), rack_values_mm: tuple[float, ...] = (),
    pad_height_mm: tuple[float, ...] = (),
    paths: tuple[str, ...] = (), levels: int = 11, maximum: float = 1,
    side_mode: str = "single", left_right_mode: str = "symmetric",
    drive: str = "wheel_center", drive_mode: str | None = None,
    times_s: tuple[float, ...] = (0, .001), settings: AxleSolverSettings | None = None,
) -> tuple[AssemblyDocument, CaseDocument]:
    """Declare a v1 K/C fixture as an ordinary subsystem and explicit run data."""
    if not (wheel_values_mm or rack_values_mm or pad_height_mm or paths):
        if drive_mode == "pad":
            pad_height_mm = (0,)
        else:
            wheel_values_mm = (0,)
    document = migrate_v1_axle(model, mode=mode)
    documents = {entry.ref: entry.subsystem for entry in document.entries}
    owners = {row["name"]: entry.ref for entry in document.entries for row in entry.subsystem.template.payload["bodies"]}
    if "chassis" not in owners:
        raise MigrationError("K/C migration requires its declared fixed support")
    wheel = next((entry for entry in document.entries if entry.functional_role == "wheel"), None)
    if wheel is None:
        raise MigrationError("K/C migration requires declared wheel-center ports")
    available = {row["name"] for row in wheel.subsystem.template.payload["ports"]}
    if not {"wheel_center_L", "wheel_center_R"} <= available:
        raise MigrationError("K/C migration requires both wheel-center ports")
    driven = bool(wheel_values_mm)
    reading = drive_mode or ("force_balance" if (driven or paths) else "pad")
    if driven and reading == "pad":
        raise MigrationError("pad heights must be supplied as explicit Study inputs")
    if pad_height_mm and (driven or rack_values_mm or paths or reading != "pad"):
        raise MigrationError("pad heights require a pad study without wheel/rack sweeps or load paths")
    template = {"document": "template", "schema_version": 1, "name": "kc_rig",
        "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "mm" if model.units == UnitSystem.ENGINEERING else "m"},
        "bodies": [], "joints": [], "elements": [], "hardpoints": [
            {"name": "origin", "owner": "@fixture", "space": "body"}],
        "needs": [{"name": "fixture", "role": "body_chassis", "count": 1, "required": True}],
        "ports": [], "property_slots": []}
    values = {"origin": [0, 0, 0]}
    pairings = [{"requirement_role": "fixture", "port": owners["chassis"]+".body_chassis"}]
    for side in ("L", "R"):
        alias = "wheel_"+side
        template["needs"].append({"name": alias, "role": "wheel_hub", "count": 1, "required": True})
        pairings.append({"requirement_role": alias, "port": wheel.ref+".wheel_center_"+side})
        if driven:
            target = "wheel_drive_"+side
            template["joints"].append({"name": target, "type": "driven_translation",
                "body_a": "@"+alias, "body_b": "@fixture", "point_a": "@"+alias,
                "point_b": "origin", "axis_space": "body", "axis": [0, 0, 1], "axis_b": [0, 0, 1],
                "target": target, "reference_quaternion": [1, 0, 0, 0]})
    rack_target = None
    if "rack" in owners:
        normalized = {key.upper(): value.as_array() for key, value in model.hardpoints.items()}
        center = next((normalized[key] for key in _POINT_ALIASES["rack_center"] if key in normalized), None)
        if center is None:
            raise MigrationError("K/C rack requires its declared center")
        template["needs"].append({"name": "rack", "role": "body_rack", "count": 1, "required": True})
        pairings.append({"requirement_role": "rack", "port": owners["rack"]+".body_rack"})
        template["hardpoints"].append({"name": "rack_center", "owner": "@rack"})
        values["rack_center"] = center.tolist()
        rack_target = "rack_drive" if driven else "rack_neutral"
        if not rack_values_mm and not paths and not pad_height_mm:
            rack_values_mm = (0,)
        template["joints"].append({"name": rack_target, "type": "driven_translation",
            "body_a": "@rack", "body_b": "@fixture", "point_a": "rack_center", "point_b": "origin",
            "axis_space": "body", "axis": [0, 1, 0], "axis_b": [0, 1, 0],
            "target": rack_target, "reference_quaternion": [1, 0, 0, 0]})
    ref = "kc_rig.sub.json"
    documents[ref] = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": "kc_rig", "template": "kc_rig.tpl.json", "functional_role": "generic",
        "placement_role": "any", "hardpoints": values, "property_bindings": {}},
        template=TemplateDocument.from_payload(template))
    assembly = document.to_payload()
    assembly["gravity"] = [0, 0, 0]
    assembly["subsystems"].append({"ref": ref, "functional_role": "generic", "placement_role": "any", "pairings": pairings})
    document = AssemblyDocument.from_payload(assembly, subsystems=documents)
    boundaries = [{"name": "hold:"+wheel.ref+"."+row["name"], "coordinate": wheel.ref+"."+row["name"],
        "mode": "locked", "units": "rad", "value": 0} for row in wheel.subsystem.template.payload.get("coordinates", ())]
    excitation = {"drive_mode": reading}
    if pad_height_mm:
        excitation["k"] = {"pad_height_mm": list(pad_height_mm)}
    if wheel_values_mm or rack_values_mm:
        axis_map: dict[str, Any] = {}
        if driven:
            axis_map["wheel"] = [ref+".wheel_drive_"+side for side in ("L", "R")]
        if rack_target:
            axis_map["rack"] = ref+"."+rack_target
        excitation["k"] = {"wheel_values_mm": list(wheel_values_mm), "rack_values_mm": list(rack_values_mm),
            "drive": drive, "left_right_mode": left_right_mode, "axis_map": axis_map}
    if paths:
        excitation["c"] = {"paths": list(paths), "levels": levels, "maximum": maximum,
            "side_mode": side_mode, "load_marker": wheel.ref+".wheel_center_L"}
        if side_mode != "single":
            excitation["c"]["mirror_marker"] = wheel.ref+".wheel_center_R"
    activation = []
    for entry in document.entries:
        for section in ("elements", "tires"):
            if (section == "tires" and reading != "pad") or (section == "elements" and reading == "kinematics"):
                activation.extend({"entity": entry.ref+"."+row["name"], "active": False}
                    for row in entry.subsystem.template.payload.get(section, ())
                    if mode in row.get("modes", ("K", "C")))
    return document, CaseDocument({"schema_version": 1, "name": name, "study": "quasi_static",
        "protocol": "kc_quasi_static", "samples": list(times_s),
        "solver": (settings or AxleSolverSettings()).model_dump(mode="json"),
        "boundaries": boundaries, "element_activation": activation,
        "excitation": excitation, "inputs": [], "outputs": []})


def migrate_v1_vehicle(source: str | Path | Mapping[str, Any] | VehicleDeclaration, *, mode: str = "K") -> AssemblyDocument:
    """Convert vehicle declarations to explicitly bound ordinary subsystems."""
    if isinstance(source, (str, Path)):
        path = Path(source)
        text = path.read_text(encoding="utf-8")
        source = json.loads(text) if path.suffix.lower() == ".json" else yaml.safe_load(text)
    model = source if isinstance(source, VehicleDeclaration) else VehicleDeclaration.model_validate(source)
    documents: dict[str, SubsystemDocument] = {}
    entries = []
    body_ids: dict[str, tuple[str, str]] = {}
    joint_ids = {}

    def add(ref: str, template: dict[str, Any], points: Mapping[str, Any], properties: Mapping[str, ElementPropertyDocument] | None = None) -> None:
        document = TemplateDocument.from_payload(json.loads(json.dumps(template)))
        bindings = {key: ref + "." + key + ".property.json" for key in (properties or {})}
        documents[ref] = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
            "name": document.name, "template": ref + ".tpl.json", "functional_role": document.functional_role,
            "placement_role": "any", "hardpoints": dict(points), "property_bindings": bindings}, template=document, properties=properties)
        entries.append({"ref": ref, "functional_role": document.functional_role, "placement_role": "any"})
        for body in template["bodies"]:
            body_ids[body["name"]] = (ref, body["name"])

    def base(name: str, role: str, units: str = "m") -> dict[str, Any]:
        return {"document": "template", "schema_version": 1, "name": name, "functional_role": role,
            "allowed_placement_roles": ["any"], "symmetry": "asymmetric", "units": {"length": units},
            "bodies": [], "hardpoints": [], "joints": [], "elements": [], "property_slots": [], "ports": [], "needs": []}

    length = .001 if model.units == UnitSystem.ENGINEERING else 1
    chassis = model.chassis
    template = base(model.name + "_body", "chassis", "mm" if length == .001 else "m")
    template["bodies"] = [{"name": chassis.name, "mass": chassis.mass, "inertia": chassis.inertia,
        "position": chassis.pose.translation.as_tuple(), "quaternion": chassis.pose.rotation.as_tuple(),
        "center_of_mass": chassis.center_of_mass.as_tuple(), "fixed": chassis.fixed}]
    template["hardpoints"] = [{"name": "reference"}]
    template["ports"] = [{"name": "support", "role": "body_chassis", "owner": chassis.name, "point": "reference", "cardinality": "many"}]
    if chassis.static_rotation_axis_local:
        template["gauges"] = [{"body": chassis.name, "axis_local": chassis.static_rotation_axis_local.as_tuple()}]
    add("body", template, {"reference": [0, 0, 0]})
    for placement, axle in (("front", model.front_axle), ("rear", model.rear_axle)):
        raw = axle.model_dump(mode="json")
        raw["bodies"] = [row for row in raw["bodies"] if row["name"] not in {chassis.name, "chassis", "ground"}]
        for section in ("joints", "springs", "dampers", "stops", "bushings"):
            for row in raw[section]:
                for field in ("body_a", "body_b"):
                    if row[field] in {chassis.name, "ground"}:
                        row[field] = "chassis"
        axle_document = migrate_v1_axle(raw, mode=mode)
        for entry in axle_document.entries:
            template = entry.subsystem.template.to_payload()
            template["name"] = placement + "_" + template["name"]
            template["bodies"] = [row for row in template["bodies"] if row["name"] != "chassis"]
            template["ports"] = [row for row in template.get("ports", ()) if row.get("owner") != "chassis"]
            needs = template.setdefault("needs", [])
            def need(name: str) -> None:
                if not any(row.get("name", row["role"]) == name for row in needs):
                    needs.append({"name": name, "role": name, "count": 1, "required": True})
            for row in (*template["joints"], *template["elements"]):
                for field in ("body_a", "body_b"):
                    if row.get(field) == "chassis":
                        row[field] = "@body_chassis"
                        need("body_chassis")
            for row in (*template.get("ports", ()), *needs):
                if row["role"] != "body_chassis":
                    row["role"] = placement + "_" + row["role"]
            template["gauges"] = [row for row in template.get("gauges", ()) if row["body"] != "chassis"]
            hardpoints = dict(entry.subsystem.payload["hardpoints"])
            chassis_pose = SE3(chassis.pose.translation.as_array(), np.asarray(chassis.pose.rotation.as_tuple()))
            for point_row in template["hardpoints"]:
                if point_row.get("owner") == "chassis":
                    point_row["owner"] = "@body_chassis"
                    need("body_chassis")
                    key = point_row["name"]
                    hardpoints[key] = (chassis_pose.inverse().transform_point(np.asarray(hardpoints[key]))
                        - chassis.center_of_mass.as_array()).tolist()
            ref = placement + "_" + entry.ref
            add(ref, template, hardpoints, entry.subsystem.properties)
            for body in template["bodies"]:
                body_ids[placement + "_" + body["name"]] = (ref, body["name"])
                body_ids.pop(body["name"], None)
            for row in template["joints"]:
                joint_ids[placement + "_" + row["name"]] = ref + "." + row["name"]

    def reference(name: str, placement: str) -> tuple[str, str]:
        if name in {chassis.name, "chassis", "ground"}:
            return body_ids[chassis.name]
        for key in (name, placement + "_" + name):
            if key in body_ids:
                return body_ids[key]
        raise MigrationError(f"vehicle body {name!r} has no declared owner at {placement!r}")

    wheel_endpoints = {}
    for wheel in model.wheels:
        placement = wheel.name.split("_", 1)[0]
        side = "R" if wheel.name.endswith("right") else "L"
        carrier_ref, carrier_name = reference("upright_" + side, placement)
        carrier = documents[carrier_ref]
        carrier_template = carrier.template.to_payload()
        carrier_points = deepcopy(carrier.payload["hardpoints"])
        carrier_body = next(row for row in carrier_template["bodies"] if row["name"] == carrier_name)
        axial = model.front_axle if placement == "front" else model.rear_axle
        declared = {key.upper(): value.as_array() for key, value in axial.hardpoints.items()}
        global_center = next((declared[key] for key in _POINT_ALIASES["wheel_center"] if key in declared), None)
        if global_center is None:
            raise MigrationError(f"wheel {wheel.name!r} requires its axle wheel-center hardpoint")
        explicit_right = next((declared[key+"__R"] for key in _POINT_ALIASES["wheel_center"] if key+"__R" in declared), None)
        if side == "R":
            global_center = explicit_right if explicit_right is not None else global_center * [1, -1, 1]
        port = "carrier_" + wheel.name
        rotation = SE3(np.zeros(3), np.asarray(wheel.pose.rotation.as_tuple())).rotation
        spin = wheel.spin_axis.as_array()
        spin /= np.linalg.norm(spin)
        forward = wheel.forward_axis.as_array() if wheel.forward_axis else rotation.T @ np.array([1., 0., 0.])
        forward -= spin * np.dot(spin, forward)
        forward /= np.linalg.norm(forward)
        marker_rotation = np.column_stack((forward, spin, np.cross(forward, spin)))
        carrier_rotation = SE3(np.zeros(3), np.asarray(carrier_body.get("quaternion", [1, 0, 0, 0]))).rotation
        contact_rotation = carrier_rotation.T @ rotation @ marker_rotation
        carrier_template.setdefault("markers", []).append({"name": port, "owner": carrier_name,
            "point": port, "quaternion": Rotation.from_matrix(contact_rotation).as_quat(scalar_first=True).tolist()})
        carrier_template["hardpoints"].append({"name": port})
        carrier_points[port] = global_center.tolist()
        carrier_template["ports"].append({"name": port, "role": port, "marker": port,
            "capabilities": ["mount", "contact_frame"], "cardinality": "many"})
        documents[carrier_ref] = SubsystemDocument.from_payload({**carrier.to_payload(), "hardpoints": carrier_points},
            template=TemplateDocument.from_payload(carrier_template), properties=carrier.properties)
        ref = "wheel_" + wheel.name
        wheel_template = base(ref, "wheel", "mm" if length == .001 else "m")
        origin = global_center - rotation @ wheel.center_local.as_array()
        if wheel.mass <= 0:
            raise MigrationError(f"wheel {wheel.name!r}: declare physical mass before migration")
        if wheel.tire_mass and np.count_nonzero(wheel.center_local.as_array()):
            raise MigrationError("v1 tire mass at an offset wheel centre requires unsupported lever-arm physics")
        wheel_template["bodies"] = [{"name": wheel.body, "mass": wheel.mass,
            "inertia": wheel.inertia or (np.eye(3)*wheel.axial_inertia).tolist(),
            "position": origin.tolist(), "quaternion": wheel.pose.rotation.as_tuple()}]
        wheel_template["hardpoints"] = [{"name": "center"}]
        wheel_template["markers"] = [{"name": "center", "owner": wheel.body, "point": "center",
            "quaternion": Rotation.from_matrix(marker_rotation).as_quat(scalar_first=True).tolist()}]
        wheel_template["ports"] = [{"name": "hub", "role": "hub_" + wheel.name, "marker": "center", "cardinality": "many"},
            {"name": "road", "role": "road", "kind": "road", "resource": "plane", "cardinality": "many"}]
        wheel_template["needs"] = [{"name": "carrier", "role": port, "count": 1, "required": True}]
        default_mount = "wheel_hub_" + side if wheel.mount_joint_kind != "fixed" and placement+"_wheel_hub_"+side in body_ids else "upright_"+side
        mount_ref, mount_name = reference(wheel.mount_body or default_mount, placement)
        mount_key = "body_" + mount_name
        mount_role = placement + "_" + mount_key
        # An explicit axle may not already expose a mount port.
        mount_document = documents[mount_ref]
        mount_template = mount_document.template.to_payload()
        if not any(row["role"] == mount_role for row in mount_template["ports"]):
            mount_template["hardpoints"].append({"name": "mount_reference"})
            mount_template["ports"].append({"name": "mount", "role": mount_role, "owner": mount_name, "point": "mount_reference", "cardinality": "many"})
            documents[mount_ref] = SubsystemDocument.from_payload({**mount_document.to_payload(),
                "hardpoints": {**mount_document.payload["hardpoints"], "mount_reference": global_center.tolist()}},
                template=TemplateDocument.from_payload(mount_template), properties=mount_document.properties)
        wheel_template["needs"].append({"name": "mount", "role": mount_role, "count": 1, "required": True})
        kind = "fixed" if wheel.mount_joint_kind == "fixed" or mount_name.startswith("wheel_hub_") else "revolute"
        joint = {"name": "mount", "type": kind, "body_a": "@mount", "body_b": wheel.body, "point_a": "center", "point_b": "center"}
        if kind == "revolute":
            joint.update(axis_space="body", axis=(SE3(np.zeros(3), np.asarray(next(row for row in mount_template["bodies"] if row["name"] == mount_name).get("quaternion", [1, 0, 0, 0]))).rotation.T @ wheel.spin_axis.as_array()).tolist(),
                axis_b=wheel.spin_axis.as_tuple())
            wheel_template["coordinates"] = [{"name": "spin", "joint": "mount", "kind": "rotation"}]
            wheel_template["ports"].append({"name": "spin", "role": "wheel_spin", "kind": "spin", "owner": wheel.body, "coordinate": "spin", "cardinality": "many"})
        wheel_template["joints"] = [joint]
        wheel_template["property_slots"] = [{"name": "tire", "element_type": "tire", "required": True}]
        tire = wheel.tire
        parameters = {key: value for key, value in tire.model_dump(mode="json").items() if isinstance(value, (float, int)) and value is not None}
        parameters.update(tire.fiala_parameters if tire.kind == "fiala" else tire.pac2002_coefficients)
        properties = {}
        if wheel.tire_mass:
            wheel_template["property_slots"].append({"name": "tire_mass", "element_type": "mass", "required": True})
            properties["tire_mass"] = ElementPropertyDocument.from_payload({"document": "element_properties", "schema_version": 1,
                "name": ref + "_tire_mass", "element_type": "mass", "model": "linear",
                "units": {"length": "m", "force": "N", "mass": "kg"}, "parameters": {"value": wheel.tire_mass}})
        law = ElementPropertyDocument.from_payload({"document": "element_properties", "schema_version": 1,
            "name": ref + "_tire", "element_type": "tire", "model": "native_brush" if tire.kind == "vertical_linear" else tire.kind,
            "units": {"length": "mm" if length == .001 else "m", "force": "N"}, "parameters": parameters,
            "tables": json.loads(json.dumps(tire.pac2002_tables)),
            "metadata": {"parameter_source": tire.parameter_source}})
        wheel_template["tires"] = [{"name": wheel.name, "body": {"name": wheel.body, "center_marker": "center"},
            "model_slot": "tire", "mount_port": "hub", "spin_marker": "center", "contact_frame": {"port": "@carrier"}, "road_port": "road",
            "mirror": tire.kind == "pac2002" and tire.parameter_source == "adams_builtin"
                and (wheel.name.endswith("right") or tire.pac2002_coefficients.get("USE_MODE", 14) < 0)}]
        if wheel.static_rotation_axis_local:
            wheel_template["gauges"] = [{"body": wheel.body, "axis_local": wheel.static_rotation_axis_local.as_tuple()}]
        properties["tire"] = law
        if wheel.tire_mass:
            wheel_template["tires"][0]["mass_slot"] = "tire_mass"
        add(ref, wheel_template, {"center": global_center.tolist()}, properties)
        wheel_endpoints[wheel.name] = {"action": (ref, wheel.body), "reaction": (carrier_ref, carrier_name), "axis": wheel.spin_axis.as_tuple()}
    couplers = []
    for spec in model.coordinate_couplers:
        row = spec.model_dump(mode="json")
        for end in ("a", "b"):
            if row["joint_" + end] not in joint_ids:
                raise MigrationError(f"coupler names unmapped joint {row['joint_' + end]!r}")
            row["joint_" + end] = joint_ids[row["joint_" + end]]
            if row["coordinate_" + end] == "translation":
                row["scale_" + end] /= length
        couplers.append(row)
    if model.aerodynamic_drag:
        drag = model.aerodynamic_drag
        template = base("aerodynamic", "generic")
        template["capabilities"] = ["vehicle"]
        template["needs"] = [{"name": "body", "role": "body_chassis", "count": 1, "required": True}]
        template["elements"] = [{"name": "drag", "type": "aerodynamic_drag", "body_a": "@body",
            "parameters": {"application_point": ((drag.application_point.as_array()-chassis.center_of_mass.as_array())*length).tolist(),
                "forward_axis": (drag.forward_axis.as_array()/np.linalg.norm(drag.forward_axis.as_array())).tolist(),
                "coefficient": .5*drag.air_density*drag.drag_coefficient*drag.frontal_area}}]
        add("aerodynamic", template, {})
    def expose(endpoint: tuple[str, str]) -> str:
        ref, name = endpoint
        document = documents[ref]
        template = document.template.to_payload()
        role = ref + ":" + name
        if not any(row["role"] == role for row in template["ports"]):
            key = "endpoint_" + name
            template["hardpoints"].append({"name": key})
            template["ports"].append({"name": key, "role": role, "owner": name, "point": key, "cardinality": "many"})
            documents[ref] = SubsystemDocument.from_payload({**document.to_payload(),
                "hardpoints": {**document.payload["hardpoints"], key: [0, 0, 0]}},
                template=TemplateDocument.from_payload(template), properties=document.properties)
        return role

    demand = model.driveline.torque_demand
    splits = dict(zip(("front_left", "front_right", "rear_left", "rear_right"), model.driveline.drive_split))
    for role, source_template in (("brake", BRAKE), ("drive", DRIVE)):
        if demand not in {role, "both"}:
            continue
        wheel_names = [wheel.name for wheel in model.wheels if wheel.braked] if role == "brake" else list(model.driveline.driven_wheels)
        for name in wheel_names:
            endpoint = wheel_endpoints[name]
            wheel = next(wheel for wheel in model.wheels if wheel.name == name)
            placement = name.split("_", 1)[0]
            action = reference(wheel.drive_torque_body, placement) if wheel.drive_torque_body else endpoint["action"]
            reaction = reference(wheel.drive_torque_reaction_body, placement) if wheel.drive_torque_reaction_body else endpoint["reaction"]
            share = (.6 if placement == "front" else .4)/sum(value.startswith(placement+"_") for value in wheel_names) if role == "brake" else splits[name]
            if share <= 0:
                raise MigrationError(f"drive share for {name!r} must be positive")
            template = base(role + "_" + name, role)
            template["needs"] = [{"name": end, "role": expose(value), "count": 1, "required": True}
                for end, value in (("action", action), ("reaction", reaction))]
            values = {slot.name: slot.default for slot in source_template.property_slots}
            recipe = TORQUE_PARAMETER_RECIPES[role]
            values.update(recipe["constants"], share=share, input=1, amplitude=0)
            template["property_slots"] = [{"name": key, "element_type": "generic", "required": False, "default": value} for key, value in values.items()]
            axis = wheel.drive_torque_axis_local.as_tuple() if wheel.drive_torque_axis_local else endpoint["axis"]
            template["elements"] = [{"name": role, "type": "rotational_torque", "body_a": "@reaction", "body_b": "@action",
                "point_a": "@reaction", "point_b": "@action", "property_slot": "amplitude",
                "parameters": {"axis_a": axis, "demand_source": 2 if role == "brake" else 1,
                    "demand_tire": next(index for index, item in enumerate(model.wheels) if item.name == name)},
                "parameter_expressions": {key: {"function": recipe["expression"], "unit": "N*mm",
                    "slots": {key: key for key in values if key != "amplitude"}} for key in ("stiffness", "max_torque")}}]
            add(role + "_" + name, template, {})
    for channel in (model.steering, *model.steering_channels):
        if not channel.enabled:
            continue
        placement = channel.placement
        if channel.actuator_mode == "prescribed_rotation":
            if channel.actuator_body is None:
                raise MigrationError("prescribed steering requires its actuator_body")
            action = reference(channel.actuator_body, placement)
            reaction = reference(channel.actuator_reaction_body or chassis.name, placement)
            point_action = point_reaction = [0, 0, 0]
            axis = channel.actuator_axis_local.as_tuple()
        else:
            action = reference(channel.rack_body, placement)
            ref, name = action
            document = documents[ref]
            rows = [row for row in document.template.payload["joints"] if row["type"] == "prismatic" and name in {row["body_a"], row["body_b"]}]
            if len(rows) != 1:
                raise MigrationError(f"steering {channel.channel_name!r} requires exactly one declared rack guide")
            row = rows[0]
            end = "a" if row["body_a"] == name else "b"
            far = "b" if end == "a" else "a"
            far_name = row["body_" + far]
            reaction = reference(channel.actuator_reaction_body or (chassis.name if far_name == "@body_chassis" else far_name), placement)
            point_action = (np.asarray(document.payload["hardpoints"][row["point_" + end]])*length).tolist()
            point_reaction = (np.asarray(document.payload["hardpoints"][row["point_" + far]])*length).tolist()
            reaction_document = documents[reaction[0]]
            reaction_body = next(body for body in reaction_document.template.payload["bodies"] if body["name"] == reaction[1])
            rotation = SE3(np.zeros(3), np.asarray(reaction_body.get("quaternion", [1, 0, 0, 0]))).rotation
            axis = (rotation.T @ np.asarray(row["axis"])).tolist()
        template = base("steering_" + channel.channel_name, "steering")
        template["capabilities"] = ["vehicle"]
        template["needs"] = [{"name": end, "role": expose(value), "count": 1, "required": True}
            for end, value in (("action", action), ("reaction", reaction))]
        template["elements"] = [{"name": "actuator", "type": "steering_actuator", "target": channel.channel_name,
            "parameters": {"type": "translation" if channel.actuator_mode == "rack_translation" else channel.actuator_mode,
                "body": "@action", "reaction_body": "@reaction",
                "point_local": point_action, "reaction_point_local": point_reaction, "axis_local": axis,
                "reference_quaternion": channel.actuator_reference_rotation.as_tuple(),
                "stiffness": channel.rack_stiffness/(1 if channel.actuator_mode == "prescribed_rotation" else length),
                "damping": channel.rack_damping/(1 if channel.actuator_mode == "prescribed_rotation" else length)}}]
        add("steering_" + channel.channel_name, template, {})
    return AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1, "name": model.name,
        "assembly_kind": "generic_multibody", "mode": mode, "subsystems": entries, "couplers": couplers}, subsystems=documents)


def save_migrated_assembly(document: AssemblyDocument, directory: str | Path) -> Path:
    """Write an ordinary standalone document set for the common document loader."""
    target = Path(directory).resolve()
    target.mkdir(parents=True, exist_ok=True)
    for entry in document.entries:
        entry.subsystem.template.save(target / entry.subsystem.payload["template"])
        for slot, property_document in entry.subsystem.properties.items():
            property_document.save(target / entry.subsystem.payload["property_bindings"][slot])
        entry.subsystem.save(target / entry.ref)
    return document.save(target / "assembly.json")


def migrate_v1_case(
    source: str | Path | Mapping[str, Any], *, entity_ids: Mapping[str, str],
    input_payload: bytes = b"", length_scale: float = 1,
    coordinate_units: Mapping[str, str] | None = None,
    boundaries: tuple[Mapping[str, Any], ...] = (),
) -> CaseDocument:
    """Convert a native case into standalone SI analysis data with explicit IDs."""
    if isinstance(source, (str, Path)):
        source = json.loads(Path(source).read_text(encoding="utf-8"))
    case = deepcopy(dict(source))
    validate_case(case)
    if length_scale not in {1, .001}:
        raise MigrationError("case length_scale must be 1 (m) or .001 (mm)")

    def reference(name: str, kind: str = "coordinate") -> str:
        key = kind + ":" + name if kind + ":" + name in entity_ids else name
        if key not in entity_ids:
            raise MigrationError(f"case references unmapped entity {name!r}")
        return entity_ids[key]

    def remap(value: Any) -> Any:
        if isinstance(value, list):
            return [remap(item) for item in value]
        if not isinstance(value, dict):
            return value
        return {key: reference(item, key) if key in {
            "body", "tire", "coordinate", "actuator", "load_marker", "mirror_marker"
        } and isinstance(item, str) else remap(item) for key, item in value.items()}

    descriptors = case.pop("blobs", ())
    counts = Counter(row["name"] for row in descriptors)
    occupied = set(counts)
    tables = []
    for index, descriptor in enumerate(descriptors):
        offset, length = descriptor["offset"], descriptor["length"]
        if offset < 0 or length < 0 or offset+length > len(input_payload):
            raise MigrationError(f"case table {descriptor['name']!r} falls outside input payload")
        array = np.frombuffer(input_payload[offset:offset+length], dtype="<f8")
        if array.size != np.prod(descriptor["shape"]):
            raise MigrationError(f"case table {descriptor['name']!r} has inconsistent shape")
        name = descriptor["name"]
        if counts[name] > 1:
            suffix = index
            while name+":"+str(suffix) in occupied:
                suffix += 1
            name += ":"+str(suffix)
            occupied.add(name)
        tables.append((name, descriptor, array.reshape(descriptor["shape"]).copy()))
    time = case.pop("time")
    sample_id = time.get("samples")
    if sample_id:
        matches = [row for row in tables if row[1]["name"] == sample_id]
        if len(matches) != 1 or matches[0][1]["role"] != "sample_times":
            raise MigrationError("case time requires its explicit sample_times table")
        samples = matches[0][2].tolist()
        tables = [row for row in tables if row is not matches[0]]
    else:
        start, end, step = (float(time[key]) for key in ("start_s", "end_s", "step_s"))
        if step <= 0 or end <= start:
            raise MigrationError("case requires a positive time interval and step")
        samples = np.linspace(start, end, int(round((end-start)/step))+1).tolist()
    inputs = []
    for name, descriptor, values in tables:
        role = descriptor["role"]
        if role in {"road_height", "road_velocity", "wheel_torque", "brake_torque"}:
            values *= length_scale
        elif role == "body_wrench":
            values[..., 3:] *= length_scale
        elif role in {"driven_offset", "driven_offset_rate"}:
            unit = (coordinate_units or {}).get(descriptor["coordinate"])
            if unit not in {"rad", "m", "mm"}:
                raise MigrationError("driven tables require explicit coordinate units")
            if unit == "mm":
                values *= .001
        row = {key: value for key, value in descriptor.items() if key in {
            "role", "body", "tire", "coordinate", "actuator", "quantity"
        }}
        inputs.append({"name": name, **remap(row), "values": values.tolist()})
    identity = {"contract", "contract_version", "kind", "family", "name", "solver"}
    excitation = remap({key: value for key, value in case.items() if key not in identity})

    def check_blob_references(value: Any) -> None:
        if isinstance(value, list):
            for item in value:
                check_blob_references(item)
        elif isinstance(value, dict):
            for key, item in value.items():
                if key in {"blob", "rate_blob"} and isinstance(item, str) and counts[item] > 1:
                    raise MigrationError(f"case names ambiguous input table {item!r}")
                check_blob_references(item)

    check_blob_references(excitation)
    axis_map = excitation.get("k", {}).get("axis_map", {})
    for axis, names in tuple(axis_map.items()):
        axis_map[axis] = [reference(name) for name in names] if isinstance(names, list) else reference(names)
    return CaseDocument({"schema_version": 1, "name": case["name"], "protocol": case["family"],
        "study": "quasi_static" if case["family"] in {"kc_quasi_static", "vehicle_kc"} else "dynamic",
        "samples": samples, "solver": case["solver"], "excitation": excitation,
        "boundaries": [dict(row) for row in boundaries], "inputs": inputs, "outputs": []})


def migrate_v1_vehicle_case(case: VehicleDynamicCase) -> tuple[AssemblyDocument, CaseDocument]:
    """Offline migration of effective vehicle physics, initial state and excitation."""
    model, settings = case.vehicle, case.solver
    scale = .001 if model.units == UnitSystem.ENGINEERING else 1
    if model.allocation_law != "direct":
        raise MigrationError("migrate allocated steering as explicit channel target tables")
    if settings.integrator != "generalized_alpha" or settings.global_velocity_damping or settings.velocity_recovery_enabled:
        raise MigrationError("vehicle case requires the native generalized_alpha solver settings")
    if model.units == UnitSystem.SI and "gravity" not in settings.model_fields_set:
        raise MigrationError("SI vehicle case requires gravity to be explicitly declared in m/s^2")
    compliant = bool(model.front_axle.bushings or model.rear_axle.bushings)
    mode = ("C" if compliant else "K") if case.suspension_mode == "auto" else case.suspension_mode
    if (mode == "C") != compliant:
        raise MigrationError("case connection mode disagrees with its declared physical bushings")
    document = migrate_v1_vehicle(model, mode=mode)
    properties = {}
    for entry in document.entries:
        properties[entry.ref] = dict(entry.subsystem.properties)
        for slot, law in entry.subsystem.properties.items():
            if law.payload["element_type"] != "tire":
                continue
            payload = law.to_payload()
            payload["parameters"]["friction_coefficient"] *= case.road.friction_coefficient
            properties[entry.ref][slot] = ElementPropertyDocument.from_payload(payload)
    document = AssemblyDocument.from_payload({**document.to_payload(), "gravity": (settings.gravity.as_array()*scale).tolist()},
        subsystems={entry.ref: entry.subsystem for entry in document.entries}, properties=properties)
    times = [float(settings.start_time)]
    step = settings.output_step or settings.step_size
    while times[-1]+step < settings.end_time-1e-12:
        times.append(times[-1]+step)
    if times[-1] < settings.end_time-1e-12:
        times.append(float(settings.end_time))
    elif abs(times[-1]-settings.end_time) <= 1e-12:
        times[-1] = float(settings.end_time)
    tolerance = settings.integration_error_tolerance or settings.constraint_tolerance
    solver = AxleSolverSettings(integrator="ggl_generalized_alpha", rho_inf=settings.generalized_alpha_rho_inf,
        initialization_mode="static_equilibrium" if case.static_equilibrium else "provided_consistent_state",
        adaptive_step=settings.adaptive_substepping, internal_step_s=settings.internal_step_size,
        minimum_step_s=settings.min_internal_step_size, maximum_step_s=max(settings.step_size, settings.internal_step_size),
        local_relative_tolerance=tolerance, local_position_tolerance_m=tolerance*scale,
        local_angle_tolerance_rad=tolerance, local_velocity_tolerance_m_per_s=tolerance*scale,
        local_angular_velocity_tolerance_rad_per_s=tolerance, local_brush_tolerance_m=tolerance*scale,
        contact_event_tolerance_s=settings.event_tolerance, max_newton_iterations=settings.projection_max_iterations,
        max_line_search_iterations=settings.projection_backtracking, position_tolerance_m=settings.constraint_tolerance*scale,
        velocity_tolerance_m_per_s=settings.velocity_tolerance*scale, dynamics_tolerance=settings.constraint_tolerance,
        increment_tolerance=settings.constraint_tolerance*scale).model_dump(mode="json")
    state, aliases, centers, providers = {}, {}, {}, {}
    for entry in document.entries:
        template = entry.subsystem.template.payload
        unit = .001 if template["units"]["length"] == "mm" else 1
        for row in template["bodies"]:
            name = entry.ref+"."+row["name"]
            pose = SE3(np.asarray(row.get("position", [0, 0, 0]), dtype=float)*unit, np.asarray(row.get("quaternion", [1, 0, 0, 0])))
            com = np.asarray(row.get("center_of_mass", [0, 0, 0]), dtype=float)*unit
            state[name] = {"position": (pose.translation+pose.rotation@com).tolist(), "quaternion": pose.quaternion.tolist(),
                "velocity": [case.initial_velocity_sign*case.initial_forward_speed_mps, 0, 0], "omega": [0, 0, 0]}
            centers[name] = com
            alias = entry.ref.split("_", 1)[0]+"_"+row["name"] if entry.ref.startswith(("front_", "rear_")) else row["name"]
            aliases[alias] = name
        for port in template["ports"]:
            if port.get("owner"):
                providers[port["role"]] = entry.ref+"."+port["owner"]

    def body_id(alias: str) -> str:
        if alias in aliases:
            return aliases[alias]
        matches = [value for key, value in aliases.items() if key in {"front_"+alias, "rear_"+alias}]
        if len(matches) != 1:
            raise MigrationError(f"initial state body {alias!r} is missing or ambiguous")
        return matches[0]

    explicit = set()
    for initial in case.initial_states:
        name = body_id(initial.body)
        pose = SE3(initial.pose.translation.as_array()*scale, np.asarray(initial.pose.rotation.as_tuple()))
        velocity = initial.velocity.as_array()
        state[name] = {"position": (pose.translation+pose.rotation@centers[name]).tolist(), "quaternion": pose.quaternion.tolist(),
            "velocity": (velocity[:3]*scale+np.cross(velocity[3:], pose.rotation@centers[name])).tolist(), "omega": velocity[3:].tolist()}
        explicit.add(name)
    rates = dict(case.initial_wheel_speeds)
    for wheel in model.wheels:
        name = body_id(wheel.body)
        if name in explicit:
            continue
        rotation = SE3(np.zeros(3), np.asarray(state[name]["quaternion"])).rotation
        spin = rotation @ (wheel.spin_axis.as_array()/np.linalg.norm(wheel.spin_axis.as_array()))
        rate = rates.get(wheel.name)
        if rate is None and case.initial_forward_speed_mps:
            forward = wheel.forward_axis.as_array() if wheel.forward_axis else rotation.T@np.array([1., 0, 0])
            forward -= wheel.spin_axis.as_array()*np.dot(wheel.spin_axis.as_array(), forward)/np.dot(wheel.spin_axis.as_array(), wheel.spin_axis.as_array())
            forward = rotation@(forward/np.linalg.norm(forward))
            coefficient = np.dot(np.cross(spin, [0, 0, -1]), forward)*wheel.tire.unloaded_radius*scale
            if abs(coefficient) <= 1e-12:
                raise MigrationError(f"wheel {wheel.name!r} has no valid rolling axis")
            rate = -case.initial_velocity_sign*case.initial_forward_speed_mps/coefficient
        if rate is not None:
            state[name]["omega"] = (np.asarray(state[name]["omega"])+spin*rate).tolist()
            entry = next(item for item in document.entries if item.ref == "wheel_"+wheel.name)
            mount = entry.subsystem.template.payload["joints"][0]
            if mount["type"] == "fixed":
                need = next(row for row in entry.subsystem.template.payload["needs"] if row["name"] == "mount")
                state[providers[need["role"]]]["omega"] = state[name]["omega"]
    inputs = []

    def signal(role: str, target: str, source: Any, factor: float = 1, *, derivative: bool = False, quantity: str = "") -> None:
        row = {"name": role+":"+target, "role": role,
            "values": [factor*(source.derivative_at(time) if derivative else source.value_at(time)) for time in times]}
        row["actuator" if role.startswith("steering_") else "tire"] = target
        if quantity:
            row["quantity"] = quantity
        inputs.append(row)

    for channel in (model.steering, *model.steering_channels):
        if not channel.enabled:
            continue
        target = "steering_"+channel.channel_name+"."+channel.channel_name
        rotation = channel.actuator_mode == "prescribed_rotation"
        factor = 1 if rotation else scale*(1 if channel.input == "rack_displacement" else channel.rack_displacement_per_steering_wheel_angle or channel.ratio)
        limit = channel.max_steering_angle if rotation else channel.max_rack_displacement*scale
        if any(abs(factor*case.steering_input.value_at(time)) > limit+1e-12 for time in times):
            raise MigrationError("steering input exceeds its declared limit")
        signal("steering_target", target, case.steering_input, factor, quantity="rotation" if rotation else "translation")
        signal("steering_rate", target, case.steering_input, factor, derivative=True, quantity="rotation" if rotation else "translation")
    demand = model.driveline.torque_demand
    if demand != "none" and (case.wheel_drive_torque or case.wheel_brake_torque):
        fields = [name for name in ("wheel_drive_torque", "wheel_brake_torque") if getattr(case, name)]
        raise MigrationError(f"{', '.join(fields)} conflicts with declared demand elements")
    drive, brake = dict(case.wheel_drive_torque), dict(case.wheel_brake_torque)
    splits = dict(zip(("front_left", "front_right", "rear_left", "rear_right"), model.driveline.drive_split))
    for wheel in model.wheels:
        target = "wheel_"+wheel.name+"."+wheel.name
        if demand == "none":
            from ..schema.dynamic import TimeSignal

            placement = wheel.name.split("_", 1)[0]
            count = sum(item.braked and item.name.startswith(placement+"_") for item in model.wheels)
            brake_share = (model.driveline.front_brake_bias if placement == "front" else 1-model.driveline.front_brake_bias)/count if wheel.braked else 0
            signal("wheel_torque", target, drive.get(wheel.name, TimeSignal(constant=0)) if drive else case.drive_input,
                scale if drive else scale*model.driveline.maximum_drive_torque*splits[wheel.name] if wheel.name in model.driveline.driven_wheels else 0)
            signal("brake_torque", target, brake.get(wheel.name, TimeSignal(constant=0)) if brake else case.brake_input,
                scale if brake else scale*model.driveline.maximum_brake_torque*brake_share)
        else:
            if demand in {"drive", "both"} and wheel.name in model.driveline.driven_wheels:
                signal("throttle_demand", target, case.drive_input)
            if demand in {"brake", "both"} and wheel.braked:
                signal("brake_pressure", target, case.brake_input)
    road = case.road
    if not np.allclose(road.normal.as_array()/np.linalg.norm(road.normal.as_array()), [0, 0, 1], rtol=0, atol=1e-12) or abs(road.origin.y) > 1e-12:
        raise MigrationError("native road requires a horizontal surface and zero origin y")
    excitation = {"inputs": {"initial_state_angle_tolerance": settings.initial_state_angle_tolerance_rad or settings.constraint_tolerance,
        "road": {"kind": "plane" if road.corner_height_signals else road.kind, "parameters": {
            "origin_x": road.origin.x*scale, "origin_z": 0 if road.corner_height_signals else road.origin.z*scale,
            "amplitude": road.amplitude*scale, "wavelength": road.wavelength*scale, "phase": road.phase,
            "bump_start": road.bump_start*scale, "bump_length": road.bump_length*scale, "corner_scale": road.corner_scales}}}}
    if road.corner_height_signals:
        for wheel, height in zip(model.wheels, road.corner_height_signals):
            target = "wheel_"+wheel.name+"."+wheel.name
            signal("road_height", target, height, scale)
            inputs[-1]["values"] = [value+road.origin.z*scale for value in inputs[-1]["values"]]
            signal("road_velocity", target, height, scale, derivative=True)
    if case.static_equilibrium and not model.chassis.fixed and (road.kind == "plane" or road.corner_height_signals):
        excitation["inputs"]["static_gauge"] = {"body": "body."+model.chassis.name, "dof_mask": 35,
            "trim_then_release": case.initial_forward_speed_mps > 0}
    boundaries = [{"name": "free:"+entry.ref+"."+row["name"], "coordinate": entry.ref+"."+row["name"], "mode": "free"}
        for entry in document.entries for row in entry.subsystem.template.payload.get("coordinates", ())]
    return document, CaseDocument({"schema_version": 1, "name": case.name, "study": "dynamic", "protocol": "vehicle_dynamic",
        "samples": times, "solver": solver, "initial_state": state, "excitation": excitation,
        "boundaries": boundaries, "inputs": inputs, "outputs": []})


def migrate_v1_vehicle_kc_case(
    source: VehicleDynamicCase, *, name: str = "vehicle-kc",
    wheel_values_mm: tuple[float, ...], rack_values_mm: tuple[float, ...],
    times_s: tuple[float, ...], settings: AxleSolverSettings | None = None,
    left_right_mode: str = "symmetric",
) -> tuple[AssemblyDocument, CaseDocument]:
    """Connect a declared vehicle to an ordinary K/C fixture through its ports."""
    document, initial = migrate_v1_vehicle_case(source)
    documents = {entry.ref: entry.subsystem for entry in document.entries}
    scale = .001 if source.vehicle.units == UnitSystem.ENGINEERING else 1
    ref = "kc_rig.sub.json"
    template = {"document": "template", "schema_version": 1, "name": "vehicle_kc_rig",
        "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"}, "bodies": [], "elements": [],
        "hardpoints": [{"name": "origin", "owner": "@fixture", "space": "body"}],
        "needs": [{"name": "fixture", "role": "body_chassis", "count": 1, "required": True}],
        "ports": [], "property_slots": [], "joints": []}
    points = {"origin": (source.vehicle.chassis.center_of_mass.as_array()*scale).tolist()}
    pairings = [{"requirement_role": "fixture", "port": "body.support"}]
    targets = []
    for wheel in source.vehicle.wheels:
        target = "wheel_drive_"+wheel.name
        alias = "wheel_"+wheel.name
        template["needs"].append({"name": alias, "role": "hub_"+wheel.name, "count": 1, "required": True})
        pairings.append({"requirement_role": alias, "port": alias+".hub"})
        template["joints"].append({"name": target, "type": "driven_translation",
            "body_a": "@"+alias, "body_b": "@fixture", "point_a": "@"+alias, "point_b": "origin",
            "axis_space": "body", "axis": [0, 0, 1], "axis_b": [0, 0, 1],
            "reference_quaternion": [1, 0, 0, 0], "target": target})
        targets.append(ref+"."+target)
    if not targets:
        raise MigrationError("vehicle K/C requires declared wheel hub ports")
    front = source.vehicle.front_axle
    normalized = {key.upper(): value.as_array() for key, value in front.hardpoints.items()}
    center = next((normalized[key] for key in _POINT_ALIASES["rack_center"] if key in normalized), None)
    rack = next((entry for entry in document.entries if entry.ref.startswith("front_")
        and any(row["name"] == "body_rack" for row in entry.subsystem.template.payload["ports"])), None)
    if rack is None or center is None:
        raise MigrationError("vehicle K/C requires its explicit rack center and port")
    template["needs"].append({"name": "rack", "role": "front_body_rack", "count": 1, "required": True})
    pairings.append({"requirement_role": "rack", "port": rack.ref+".body_rack"})
    template["hardpoints"].append({"name": "rack_center", "owner": "@rack"})
    points["rack_center"] = (center*scale).tolist()
    template["joints"].append({"name": "rack_drive", "type": "driven_translation",
        "body_a": "@rack", "body_b": "@fixture", "point_a": "rack_center", "point_b": "origin",
        "axis_space": "body", "axis": [0, 1, 0], "axis_b": [0, 1, 0],
        "reference_quaternion": [1, 0, 0, 0], "target": "rack_drive"})
    documents[ref] = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": "vehicle_kc_rig", "template": "vehicle_kc_rig.tpl.json", "functional_role": "generic",
        "placement_role": "any", "hardpoints": points, "property_bindings": {}},
        template=TemplateDocument.from_payload(template))
    payload = document.to_payload()
    payload["subsystems"].append({"ref": ref, "functional_role": "generic", "placement_role": "any", "pairings": pairings})
    document = AssemblyDocument.from_payload(payload, subsystems=documents)
    boundaries = [{"name": "hold:"+entry.ref+"."+row["name"], "coordinate": entry.ref+"."+row["name"],
        "mode": "locked", "units": "rad", "value": 0} for entry in document.entries
        for row in entry.subsystem.template.payload.get("coordinates", ())]
    inactive = [{"entity": entry.ref+"."+row["name"], "active": False} for entry in document.entries
        for row in entry.subsystem.template.payload["elements"] if row["type"] == "steering_actuator"]
    return document, CaseDocument({"schema_version": 1, "name": name, "study": "quasi_static", "protocol": "vehicle_kc",
        "samples": list(times_s), "solver": (settings or AxleSolverSettings()).model_dump(mode="json"),
        "boundaries": boundaries, "element_activation": inactive, "initial_state": initial.to_payload()["initial_state"],
        "excitation": {"k": {"wheel_values_mm": list(wheel_values_mm), "rack_values_mm": list(rack_values_mm),
            "ramp_s": times_s[-1]-times_s[0], "drive": "wheel_center", "left_right_mode": left_right_mode,
            "axis_map": {"wheel": targets, "rack": ref+".rack_drive"}}}, "inputs": [], "outputs": []})


def migrate_v1_dynamic_axle(model: AxleDynamicsModel, case: AxleDynamicsCase) -> tuple[AssemblyDocument, CaseDocument]:
    """Convert the SI declaration directly to ordinary subsystem and run data."""
    templates, points, properties = {}, {}, {}
    tire_bodies = {tire.body for tire in model.tires}
    poses = {body.name: SE3(np.asarray(body.position_m), np.asarray(body.quaternion_body_to_world)) for body in model.bodies}
    ids = {}
    for body in model.bodies:
        role = "wheel" if body.name in tire_bodies else "generic"
        templates[body.name] = {"document": "template", "schema_version": 1, "name": body.name,
            "functional_role": role, "allowed_placement_roles": ["any"], "symmetry": "asymmetric", "units": {"length": "m"},
            "bodies": [{"name": body.name, "mass": body.mass_kg, "inertia": np.asarray(body.inertia_kg_m2).tolist(),
                "position": body.position_m, "quaternion": body.quaternion_body_to_world, "fixed": body.fixed,
                "velocity": body.linear_velocity_m_per_s, "omega": body.angular_velocity_rad_per_s}],
            "hardpoints": [{"name": "reference", "owner": body.name, "space": "body"}], "joints": [], "elements": [],
            "markers": [], "ports": [{"name": "body", "role": "body:"+body.name, "owner": body.name,
                "point": "reference", "cardinality": "many"}], "needs": [],
            "property_slots": [{"name": "parameters", "element_type": "generic", "required": False, "default": 0}]}
        points[body.name], properties[body.name] = {"reference": [0, 0, 0]}, {}
        ids["body:"+body.name] = body.name+"."+body.name

    def endpoint(owner: str, body: str) -> str:
        if owner == body:
            return body
        alias = "body:"+body
        if not any(row["name"] == alias for row in templates[owner]["needs"]):
            templates[owner]["needs"].append({"name": alias, "role": alias, "count": 1, "required": True})
        return "@"+alias

    def point(owner: str, body: str, value: Any) -> str:
        key = "point_"+str(len(points[owner]))
        templates[owner]["hardpoints"].append({"name": key, "owner": endpoint(owner, body), "space": "body"})
        points[owner][key] = np.asarray(value, dtype=float).tolist()
        return key

    for joint in model.joints:
        owner = joint.body_b
        templates[owner]["joints"].append({"name": joint.name, "type": "convel" if joint.kind == "constant_velocity" else joint.kind,
            "body_a": endpoint(owner, joint.body_a), "body_b": endpoint(owner, joint.body_b),
            "point_a": point(owner, joint.body_a, joint.point_a_m), "point_b": point(owner, joint.body_b, joint.point_b_m),
            "axis_space": "body", "axis": joint.axis_a, "axis_b": joint.axis_b})
        ids["joint:"+joint.name] = owner+"."+joint.name
        if joint.kind == "revolute" and owner in tire_bodies:
            templates[owner].setdefault("coordinates", []).append({"name": joint.name+"_coordinate", "joint": joint.name, "kind": "rotation"})
            templates[owner]["ports"].append({"name": joint.name+"_spin", "role": "wheel_spin", "kind": "spin", "owner": owner,
                "coordinate": joint.name+"_coordinate", "cardinality": "many"})
    for driven in model.driven_coordinates:
        owner = driven.body
        templates[owner]["joints"].append({"name": driven.name, "type": "driven_"+driven.kind,
            "body_a": owner, "body_b": endpoint(owner, driven.reaction_body),
            "point_a": point(owner, owner, driven.point_local_m), "point_b": point(owner, driven.reaction_body, driven.reaction_point_local_m),
            "axis_space": "body", "axis": driven.axis_local, "axis_b": driven.axis_local,
            "reference_quaternion": driven.reference_quaternion, "target": driven.name})
        ids["coordinate:"+driven.name] = owner+"."+driven.name

    def element(spec: Any, kind: str, parameters: dict[str, Any]) -> None:
        owner = spec.body_b
        slot = "parameters_"+kind
        if not any(row["name"] == slot for row in templates[owner]["property_slots"]):
            property_kind = kind if kind in {"spring", "damper", "bump_stop", "bushing"} else "generic"
            templates[owner]["property_slots"].append({"name": slot, "element_type": property_kind, "required": False, "default": 0})
        row = {"name": spec.name, "type": kind, "body_a": endpoint(owner, spec.body_a), "body_b": owner,
            "point_a": point(owner, spec.body_a, getattr(spec, "point_a_m", [0, 0, 0])),
            "point_b": point(owner, owner, getattr(spec, "point_b_m", [0, 0, 0])),
            "property_slot": slot, "parameters": parameters}
        templates[owner]["elements"].append(row)
        ids["element:"+spec.name] = owner+"."+spec.name

    for spec in model.springs:
        parameters = {"stiffness": spec.stiffness_n_per_m, "free_length": spec.free_length_m, "preload": spec.preload_n}
        if spec.elastic_curve_deflection_m:
            parameters["elastic_curve"] = list(zip(spec.elastic_curve_deflection_m, spec.elastic_curve_force_n))
        element(spec, "spring", parameters)
    for spec in model.dampers:
        parameters = {"compression_damping": spec.compression_damping_n_s_per_m, "rebound_damping": spec.rebound_damping_n_s_per_m,
            "gas_stiffness": spec.gas_stiffness_n_per_m, "gas_reference_force": spec.gas_reference_force_n,
            "preload": spec.preload_n, "friction": spec.friction_n, "extension_sign": spec.extension_sign}
        if spec.gas_reference_length_m is not None:
            parameters["gas_reference_length"] = spec.gas_reference_length_m
        if spec.damper_curve_velocity_m_per_s:
            parameters["damper_curve"] = list(zip(spec.damper_curve_velocity_m_per_s, spec.damper_curve_force_n))
        element(spec, "damper", parameters)
    for spec in model.bump_stops:
        parameters = {"clearance": spec.clearance_m, "stiffness": spec.stiffness_n_per_m, "direction": spec.direction,
            "damping": spec.damping_n_s_per_m}
        if spec.stop_curve_penetration_m:
            parameters["stop_curve"] = list(zip(spec.stop_curve_penetration_m, spec.stop_curve_force_n))
        element(spec, "bump_stop", parameters)
    for spec in model.bushings:
        element(spec, "bushing", {"stiffness": spec.stiffness, "damping": spec.damping,
            "preload": spec.preload_in_frame_a_n_n_m, "frame_a_quaternion": spec.frame_a_to_body_quaternion,
            "frame_b_quaternion": spec.frame_b_to_body_quaternion, "reference_translation": spec.reference_translation_in_frame_a_m,
            "reference_quaternion": spec.reference_quaternion_a_to_b})
    for spec in model.anti_roll_bars:
        element(spec, "anti_roll_bar", {"axis_a": spec.axis_a, "reference_quaternion": spec.reference_quaternion_a_to_b,
            "stiffness": spec.stiffness_n_m_per_rad, "damping": spec.damping_n_m_s_per_rad})
    for index, spec in enumerate(model.aerodynamic_drags):
        templates[spec.body]["capabilities"] = ["vehicle"]
        templates[spec.body]["elements"].append({"name": "aero_"+str(index), "type": "aerodynamic_drag", "body_a": spec.body,
            "parameters": {"application_point": spec.application_point_m, "forward_axis": spec.forward_axis_local,
                "coefficient": spec.coefficient_n_s2_per_m2}})
    for tire in model.tires:
        owner = tire.body
        carrier = tire.frame_body
        if carrier is None:
            joints = [row for row in model.joints if row.kind == "revolute" and owner in {row.body_a, row.body_b}]
            if len(joints) > 1:
                raise MigrationError(f"tire {tire.name!r}: declare its non-spinning contact frame")
            carrier = (joints[0].body_a if joints[0].body_b == owner else joints[0].body_b) if joints else owner
        if tire.drive_torque_body is not None:
            raise MigrationError("migrate routed tire torques as explicit Drive/Brake subsystem elements")
        if tire.mass_kg and np.count_nonzero(tire.center_local_m):
            raise MigrationError("offset tire mass requires unsupported lever-arm physics")
        spin = np.asarray(tire.spin_axis_local, dtype=float)
        spin /= np.linalg.norm(spin)
        forward = np.asarray(tire.forward_axis_local, dtype=float)
        forward -= spin*np.dot(spin, forward)
        forward /= np.linalg.norm(forward)
        rotation = np.column_stack((forward, spin, np.cross(forward, spin)))
        wheel_rotation = poses[owner].rotation.T@poses[carrier].rotation@rotation if tire.frame_body else rotation
        contact_rotation = rotation if tire.frame_body else poses[carrier].rotation.T@poses[owner].rotation@rotation
        center = poses[owner].transform_point(np.asarray(tire.center_local_m))
        contact_point = tire.frame_center_local_m if tire.frame_body else poses[carrier].inverse().transform_point(center)
        if contact_point is None:
            raise MigrationError(f"tire {tire.name!r}: contact frame requires its local center")
        contact_name = "contact:"+tire.name
        templates[carrier]["markers"].append({"name": contact_name, "owner": carrier,
            "point": point(carrier, carrier, contact_point), "quaternion": Rotation.from_matrix(contact_rotation).as_quat(scalar_first=True).tolist()})
        templates[carrier]["ports"].append({"name": contact_name, "role": contact_name, "marker": contact_name, "cardinality": "many"})
        if carrier != owner:
            templates[owner]["needs"].append({"name": contact_name, "role": contact_name, "count": 1, "required": True})
        center_name = "center:"+tire.name
        templates[owner]["markers"].append({"name": center_name, "owner": owner,
            "point": point(owner, owner, tire.center_local_m), "quaternion": Rotation.from_matrix(wheel_rotation).as_quat(scalar_first=True).tolist()})
        templates[owner]["ports"].extend([
            {"name": center_name, "role": "mount:"+tire.name, "marker": center_name, "cardinality": "many"},
            {"name": "road:"+tire.name, "role": "road", "kind": "road", "resource": "plane", "cardinality": "many"}])
        kind = "pac2002" if tire.model_kind == "pac2002_pure_slip" else tire.model_kind
        parameters = {"unloaded_radius": tire.unloaded_radius_m, "maximum_compression": tire.maximum_compression_m,
            "vertical_stiffness": tire.vertical_stiffness_n_per_m, "vertical_damping": tire.vertical_damping_n_s_per_m,
            "longitudinal_stiffness": tire.longitudinal_brush_stiffness_n_per_m*tire.longitudinal_relaxation_length_m,
            "cornering_stiffness": tire.lateral_brush_stiffness_n_per_m*tire.lateral_relaxation_length_m,
            "friction_coefficient": tire.longitudinal_friction_coefficient, "relaxation_length": tire.longitudinal_relaxation_length_m,
            "detached_relaxation_s": tire.detached_relaxation_s}
        if kind == "native_brush":
            parameters.update(longitudinal_friction_coefficient=tire.longitudinal_friction_coefficient,
                lateral_friction_coefficient=tire.lateral_friction_coefficient,
                longitudinal_brush_stiffness=tire.longitudinal_brush_stiffness_n_per_m,
                lateral_brush_stiffness=tire.lateral_brush_stiffness_n_per_m,
                longitudinal_relaxation_length=tire.longitudinal_relaxation_length_m,
                lateral_relaxation_length=tire.lateral_relaxation_length_m)
        parameters.update(tire.fiala_parameters if kind == "fiala" else tire.pac2002_coefficients)
        properties[owner][tire.name] = ElementPropertyDocument.from_payload({"document": "element_properties", "schema_version": 1,
            "name": tire.name, "element_type": "tire", "model": kind, "units": {"length": "m", "force": "N"},
            "parameters": parameters, "tables": json.loads(json.dumps(tire.pac2002_tables)),
            "metadata": {"parameter_source": tire.pac2002_parameter_source}})
        templates[owner]["property_slots"].append({"name": tire.name, "element_type": "tire", "required": True})
        row = {"name": tire.name, "body": {"name": owner, "center_marker": center_name}, "model_slot": tire.name,
            "mount_port": center_name, "spin_marker": center_name, "contact_frame": {"port": ("@" if carrier != owner else "")+contact_name},
            "road_port": "road:"+tire.name, "mirror": bool(tire.pac2002_mirror)}
        for slot, value in (("mass", tire.mass_kg), ("inertia", tire.inertia_kg_m2)):
            if value is None or not np.any(np.asarray(value)):
                continue
            key = tire.name+":"+slot
            properties[owner][key] = ElementPropertyDocument.from_payload({"document": "element_properties", "schema_version": 1,
                "name": key, "element_type": slot, "model": "linear", "units": {"length": "m", "force": "N"},
                **({"parameters": {"value": value}} if slot == "mass" else {"matrix": {"name": "inertia", "rows": np.asarray(value).tolist()}})})
            templates[owner]["property_slots"].append({"name": key, "element_type": slot, "required": True})
            row[slot+"_slot"] = key
        templates[owner].setdefault("tires", []).append(row)
        ids["tire:"+tire.name] = owner+"."+tire.name
    documents = {}
    for ref, template in templates.items():
        documents[ref] = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1, "name": ref,
            "template": ref+".tpl.json", "functional_role": template["functional_role"], "placement_role": "any",
            "hardpoints": points[ref], "property_bindings": {key: ref+"."+key+".property.json" for key in properties[ref]}},
            template=TemplateDocument.from_payload(json.loads(json.dumps(template))), properties=properties[ref])
    couplers = []
    for spec in model.coordinate_couplers:
        row = spec.model_dump(mode="json")
        for end in ("a", "b"):
            row["joint_"+end] = ids["joint:"+row["joint_"+end]]
        couplers.append(row)
    document = AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1, "name": model.name,
        "assembly_kind": "generic_multibody", "mode": "C", "gravity": list(model.gravity_m_per_s2), "couplers": couplers,
        "subsystems": [{"ref": ref, "functional_role": value.functional_role, "placement_role": "any"} for ref, value in documents.items()]},
        subsystems=documents)
    inputs = []
    for field, role, target_kind, target_field in (("road_height_m", "road_height", "tire", "tire"),
        ("road_velocity_m_per_s", "road_velocity", "tire", "tire"), ("wheel_torque_n_m", "wheel_torque", "tire", "tire"),
        ("body_wrench_n_n_m", "body_wrench", "body", "body")):
        for target, values in getattr(case, field).items():
            key = target_kind+":"+target
            if key not in ids:
                raise MigrationError(f"case references unmapped entity {key!r}")
            inputs.append({"name": role+":"+target, "role": role, target_field: ids[key], "values": values})
    times = np.asarray(case.times_s)
    for driven in model.driven_coordinates:
        target, rate = case.driven_target_m.get(driven.name), case.driven_target_rate.get(driven.name)
        if target is None and rate is None:
            raise MigrationError(f"coordinate {driven.name!r} has neither a target nor a rate")
        if target is None:
            values = np.asarray(rate)
            target = np.concatenate(([0.], np.cumsum(.5*(values[1:]+values[:-1])*np.diff(times)))).tolist()
        if rate is None:
            rate = np.gradient(target, times).tolist()
        for role, values in (("driven_offset", target), ("driven_offset_rate", rate)):
            inputs.append({"name": role+":"+driven.name, "role": role, "coordinate": ids["coordinate:"+driven.name], "values": values})
    boundaries = [{"name": "free:"+ref+"."+row["name"], "coordinate": ref+"."+row["name"], "mode": "free"}
        for ref, template in templates.items() for row in template.get("coordinates", ())]
    return document, CaseDocument({"schema_version": 1, "name": case.name, "study": "dynamic", "protocol": "axle_dynamic",
        "samples": case.times_s, "solver": case.solver.model_dump(mode="json"), "boundaries": boundaries,
        "inputs": inputs, "outputs": []})
