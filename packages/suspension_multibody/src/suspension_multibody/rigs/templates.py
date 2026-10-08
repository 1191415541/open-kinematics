"""Ordinary authoring documents for the registered test benches."""

from __future__ import annotations

from typing import Any, Mapping

from ..authoring.documents import SubsystemDocument, TemplateDocument
from .rig import RigSpec, get_rig


def generic_rig_template(
    rig: str | RigSpec, *, interfaces: Mapping[str, Mapping[str, Any]],
) -> TemplateDocument:
    """Declare a bench from explicit interfaces, without adding wheel entities."""
    spec = get_rig(rig) if isinstance(rig, str) else rig
    unknown = set(interfaces) - set(spec.coordinate_names())
    if unknown:
        raise ValueError(f"rig {spec.name!r}: unknown input interfaces {sorted(unknown)}")
    needs = []
    inputs = []
    for drive in spec.drives:
        row = {"name": drive.coordinate, "kind": drive.kind,
               "coupled_with": list(drive.coupled_with)}
        if drive.from_assembly:
            if drive.coordinate not in interfaces:
                raise ValueError(f"rig {spec.name!r}: missing interface {drive.coordinate!r}")
            interface = dict(interfaces[drive.coordinate])
            needs.append({"name": drive.coordinate, "count": 1, "required": True,
                          **interface, "bound_outputs": list(spec.outputs)})
            row.update(port="@" + drive.coordinate, requires=[drive.coordinate])
        inputs.append(row)
    return TemplateDocument.from_payload({
        "document": "template", "schema_version": 1, "name": spec.name + "_rig",
        "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"},
        "bodies": [{"name": "fixture", "fixed": True}],
        "hardpoints": [{"name": "origin", "owner": "fixture"}],
        "joints": [], "elements": [], "property_slots": [],
        "ports": [{"name": "road", "role": "road", "kind": "road", "resource": "plane", "cardinality": "many"}],
        "needs": needs, "inputs": inputs,
        "outputs": [{"name": name, "unit": "1", "source": "rig"} for name in spec.outputs],
    })


def generic_rig_subsystem(
    rig: str | RigSpec, *, interfaces: Mapping[str, Mapping[str, Any]],
) -> SubsystemDocument:
    """Bind the ordinary rig template to its support origin."""
    template = generic_rig_template(rig, interfaces=interfaces)
    return SubsystemDocument.from_payload({
        "document": "subsystem", "schema_version": 1, "name": template.name,
        "template": template.name, "functional_role": "generic", "placement_role": "any",
        "hardpoints": {"origin": [0, 0, 0]}, "property_bindings": {},
    }, template=template)
