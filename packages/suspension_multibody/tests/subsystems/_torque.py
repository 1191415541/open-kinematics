"""Ordinary wheel and force subsystem declarations used by torque tests."""

from suspension_multibody.authoring import TemplateDocument, assemble_generic
from suspension_multibody.presets import generic_template
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)


def torque_assembly(role, *, values=None, demand=1, payload=None, reaction="support"):
    data = payload or generic_template(role).to_payload()
    for slot in data["property_slots"]:
        if slot["name"] in (values or {}):
            slot["default"] = values[slot["name"]]
    if role == "brake":
        next(slot for slot in data["property_slots"] if slot["name"] == "input")["default"] = demand
    force = subsystem(TemplateDocument.from_payload(data), signals={"drive_input": [[0, demand], [1, demand]]})
    return assembly({reaction: carrier_subsystem(),
        "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}), "unit": force})


def torque_graph(role, **kwargs):
    return assemble_generic(torque_assembly(role, **kwargs))


def locked_case():
    return {"schema_version": 1, "name": "torque", "study": "dynamic", "samples": [0, .001, .002],
        "solver": {}, "inputs": [], "outputs": [],
        "boundaries": [{"name": "hold", "coordinate": "wheel.spin", "mode": "locked", "value": 0, "units": "rad"}]}
