"""A saved Wheel subsystem carries its body, TIR law and spin into native input."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import AssemblyDocument
from suspension_multibody.authoring.migration import save_migrated_assembly
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.presets import generic_template
from tests.subsystems._torque import locked_case

from .test_unified_subsystem_templates import assembly, carrier_subsystem, subsystem


def test_file_wheel_mass_and_properties_reach_the_same_graph(tmp_path):
    mass = ElementPropertyDocument.from_payload({"document": "element_properties", "schema_version": 1,
        "name": "wheel_mass", "element_type": "mass", "model": "linear", "units": {"mass": "kg", "force": "N", "length": "m"}, "parameters": {"value": 12}})
    support = carrier_subsystem()
    wheel = subsystem(generic_template("wheel"), {"center": [0, 0, .334]}, properties={"mass": mass})
    documents = {}
    for name, doc in (("support", support), ("wheel", wheel)):
        payload = doc.to_payload()
        payload["template"] = name + ".tpl.json"
        payload["property_bindings"] = {slot: name + "_" + slot + ".json" for slot in doc.properties}
        documents[name] = type(doc).from_payload(payload, template=doc.template, properties=doc.properties)
    source = assembly(documents)
    loaded = AssemblyDocument.load(save_migrated_assembly(source, tmp_path))
    original, file = [validate(doc, locked_case()) for doc in (source, loaded)]
    assert original.model_document == file.model_document
    assert original.model_payload == file.model_payload
    wheel_body = next(row for row in file.model_document["bodies"] if row["name"] == "wheel.wheel")
    assert wheel_body["mass"] == pytest.approx(12, rel=1e-12)
    assert sum(row["mass"] for row in file.model_document["bodies"]) == 12
    np.testing.assert_array_equal(wheel_body["inertia"], np.eye(3))
    assert file.model_document["tires"][0]["parameters"]["frame_body"] == "support.carrier"
    run = simulate(loaded, locked_case())
    assert run.status == "success"
    assert run.result.tire_ids == ("wheel.tire",)
    assert "wheel.wheel" in run.result.body_ids
