"""Offline export of declarative template records to editable documents."""

from __future__ import annotations

from typing import Any

from .instantiate import slot_element_type
from .model import ConnectionDefinition, Template


def _joint_bodies(connection: ConnectionDefinition) -> tuple[str, str]:
    own, far = connection.owner, connection.far_owner or connection.owner
    return (own, far) if connection.first_body == "far" else (far, own)


def template_document_from(template: Template) -> dict[str, Any]:
    """Export topology and mode tables without constructing runtime entities."""
    mirrors = template.symmetry == "mirrored_xz"
    parts = [part for part in template.parts if not mirrors or not part.name.endswith("_R")]
    connections = [row for row in template.connections if not mirrors or not row.owner.endswith("_R")]
    points: dict[str, dict[str, Any]] = {}
    joints, elements = [], []
    for row in connections:
        points.setdefault(row.role, {"name": row.role, "label": row.label or row.role,
            **({"owner": row.owner} if row.owner else {})})
        body_a, body_b = _joint_bodies(row)
        if row.joint is not None:
            joint: dict[str, Any] = {"name": row.name, "type": row.joint, "body_a": body_a,
                "body_b": body_b, "point_a": row.role}
            if row.axis_reference_role:
                joint["axis_reference"] = row.axis_reference_role
            if row.axis is not None:
                joint["axis"] = list(row.axis)
            if row.far_label:
                joint["far_label"] = row.far_label
            if tuple(row.joint_modes) != ("K", "C"):
                joint["modes"] = list(row.joint_modes)
            if row.joint_kind_by_mode:
                joint["kind_by_mode"] = [{"mode": mode, "type": kind} for mode, kind in row.joint_kind_by_mode]
            joints.append(joint)
        if row.bushing is not None:
            element: dict[str, Any] = {"name": row.bushing, "type": "bushing", "body_a": body_a,
                "body_b": body_b, "point_a": row.role, "property_slot": "bushing"}
            if tuple(row.bushing_modes) != ("K", "C"):
                element["modes"] = list(row.bushing_modes)
            elements.append(element)
    document: dict[str, Any] = {"document": "template", "schema_version": 1, "name": template.name,
        "functional_role": template.role, "allowed_placement_roles": ["any", "front", "rear"],
        "symmetry": template.symmetry, "sides": ["left"] if mirrors else ["left", "right"],
        "bodies": [{"name": part.name, "mass": part.mass, "fixed": part.fixed} for part in parts],
        "hardpoints": list(points.values()), "joints": joints, "elements": elements,
        "property_slots": [{"name": slot.name, "element_type": slot_element_type(slot.name), "unit": slot.unit,
            "required": slot.name in template.role_spec.required_slots,
            **({"default": float(slot.default)} if slot.default is not None else {}),
            **({"connections": list(slot.connections)} if slot.connections else {})} for slot in template.property_slots]}
    if template.outputs:
        document["outputs"] = [{"name": row.name, "unit": row.unit, "source": row.source} for row in template.outputs]
    if template.ports:
        names = {part.name for part in parts}
        document["ports"] = [{"name": row.name, "role": row.role,
            **({"owner": row.owner} if row.owner and row.owner in names else {})} for row in template.ports]
    if template.suspension_kind:
        document["suspension_kind"] = template.suspension_kind
    if template.description:
        document["description"] = template.description
    return document
