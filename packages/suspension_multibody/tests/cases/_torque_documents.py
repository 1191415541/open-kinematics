"""Ordinary template declarations for the two-body torque reader fixture."""

from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.migration import migrate_v1_case


def torque_documents(native, case):
    points, hardpoints, joints = {}, [], []
    for row in native["joints"]:
        converted = dict(row)
        for end in ("a", "b"):
            key = row["name"]+":"+end
            hardpoints.append({"name": key, "owner": row["body_"+end], "space": "body"})
            points[key] = row["point_"+end]
            converted["point_"+end] = key
        converted["axis"], converted["axis_space"] = converted.pop("axis_a"), "body"
        joints.append(converted)
    elements = []
    for row in native["elements"]:
        converted = {**row, "property_slot": "parameters"}
        for end in ("a", "b"):
            key = row["name"]+":"+end
            hardpoints.append({"name": key, "owner": row["body_"+end], "space": "body"})
            points[key] = [0., 0., 0.]
            converted["point_"+end] = key
        elements.append(converted)
    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": native["name"], "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"}, "bodies": native["bodies"],
        "hardpoints": hardpoints, "joints": joints,
        "elements": elements,
        "property_slots": [{"name": "parameters", "element_type": "generic", "required": False, "default": 0}]})
    subsystem = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": native["name"], "template": "mechanism.tpl.json", "functional_role": "generic",
        "placement_role": "any", "hardpoints": points, "property_bindings": {}}, template=template)
    assembly = AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1,
        "name": native["name"], "assembly_kind": "generic_multibody", "mode": "C",
        "gravity": native["gravity"], "subsystems": [{"ref": "mechanism", "functional_role": "generic", "placement_role": "any"}]},
        subsystems={"mechanism": subsystem})
    plan = migrate_v1_case(case, entity_ids={row["name"]: "mechanism."+row["name"]
        for key in ("bodies", "joints", "elements") for row in native[key]}).to_payload()
    plan["initial_state"] = {"mechanism."+row["name"]: {key: row[key] for key in ("position", "quaternion", "velocity", "omega")}
        for row in native["bodies"]}
    return assembly, plan
