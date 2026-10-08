"""Explicit fixture interfaces shared by rig declaration tests."""

from suspension_multibody.authoring import TemplateDocument, assemble_generic
from suspension_multibody.presets import generic_template
from suspension_multibody.rigs import RIGS, generic_rig_subsystem
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)


def rig_source(name="kc_quasi_static", *, offered=None, required=False, body="member"):
    drives = tuple(d.coordinate for d in RIGS[name].drives if d.from_assembly)
    offered = drives if offered is None else tuple(offered)
    ports = [{"name": key, "owner": body, "point": "origin", "role": key} for key in offered]
    specimen = subsystem(TemplateDocument.from_payload({
        "document": "template", "schema_version": 1, "name": "specimen_tpl",
        "functional_role": "generic", "allowed_placement_roles": ["any"],
        "units": {"length": "m"}, "symmetry": "asymmetric", "bodies": [{"name": body, "fixed": True}],
        "hardpoints": [{"name": "origin", "owner": body}], "ports": ports,
        "joints": [], "elements": [], "property_slots": [],
    }), {"origin": [.1, -.7, .3]})
    rig = generic_rig_subsystem(name, interfaces={key: {"role": key, "kind": "geometry", "required": required} for key in drives})
    return assembly({"specimen": specimen, "rig": rig}, pairings={"rig": {key: "specimen." + key for key in offered}})


def rig_graph(name="kc_quasi_static", **kwargs):
    return assemble_generic(rig_source(name, **kwargs))


def wheel_specimen(name=None):
    points = {key: [0, .7, .334] for key in generic_template("suspension").hardpoint_names}
    points.update(upper_rear=[1, .7, .334], lower_rear=[1, .7, .334])
    support_data = carrier_subsystem().template.to_payload()
    support_data["ports"][-1]["cardinality"] = "many"
    subs = {"support": subsystem(TemplateDocument.from_payload(support_data), {"center": [0, 0, .334]}),
            "suspension": subsystem(generic_template("suspension"), points),
            "left": subsystem(generic_template("wheel"), {"center": [0, .7, .334]}),
            "right": subsystem(generic_template("wheel"), {"center": [0, -.7, .334]})}
    pairings = {"left": {"carrier": "suspension.carrier_L", "road": "support.road"},
                "right": {"carrier": "suspension.carrier_R", "road": "support.road"}}
    if name:
        interfaces = {drive.coordinate: {"role": "steering_axis" if drive.coordinate == "rack_drive" else "wheel_hub",
                      "kind": "geometry", "required": False}
                      for drive in RIGS[name].drives if drive.from_assembly}
        subs["rig"] = generic_rig_subsystem(name, interfaces=interfaces)
        pairings["rig"] = {key: "left.hub" if key.endswith("_L") else "right.hub"
                           for key in interfaces if key != "rack_drive"}
    return assembly(subs, pairings=pairings)
