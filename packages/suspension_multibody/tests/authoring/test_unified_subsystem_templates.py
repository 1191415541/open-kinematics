"""The same document interpreter resolves automotive and arbitrary topologies."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.authoring import (
    AssemblyDocument,
    GenericSubsystemAssembler,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.presets import generic_template

from .test_tir_properties import TIR


def subsystem(template, coordinates=None, *, signals=None, properties=None):
    laws = dict(properties or {})
    for name, slot in template.property_slots.items():
        kind = slot["element_type"]
        if name in laws or kind not in {"mass", "inertia", "tire", "spring", "damper"}:
            continue
        if kind == "tire":
            laws[name] = ElementPropertyDocument.from_tir_text(TIR, name=name)
            continue
        if kind in {"spring", "damper"}:
            laws[name] = ElementPropertyDocument.from_payload({
                "document": "element_properties", "schema_version": 1, "name": name,
                "element_type": kind, "model": "linear", "units": {"force": "N", "length": "m", "time": "s"},
                "parameters": {"stiffness": 10, "free_length": .3} if kind == "spring" else {"viscous_damping": 1},
            })
            continue
        laws[name] = ElementPropertyDocument.from_payload({
            "document": "element_properties", "schema_version": 1,
            "name": name, "element_type": kind, "model": "linear",
            "units": {"mass": "kg", "length": "m", "force": "N"},
            **({"parameters": {"value": 20}} if kind == "mass" else
               {"matrix": {"name": "inertia", "rows": np.eye(3).tolist()}}),
        })
    return SubsystemDocument.from_payload({
        "document": "subsystem", "schema_version": 1, "name": template.name,
        "template": template.name, "functional_role": template.functional_role,
        "placement_role": "any", "hardpoints": coordinates or {},
        "property_bindings": {name: name for name in laws},
        "parameters": {"signals": signals or {}},
    }, template=template, properties=laws)


def assembly(subsystems, *, mode="K", pairings=None):
    return AssemblyDocument.from_payload({
        "document": "assembly", "schema_version": 1, "name": "fixture",
        "assembly_kind": "generic_multibody", "mode": mode,
        "subsystems": [
            {"ref": name, "functional_role": sub.functional_role, "placement_role": "any",
             "pairings": [{"requirement_role": role, "port": port} for role, port in (pairings or {}).get(name, {}).items()]}
            for name, sub in subsystems.items()
        ],
    }, subsystems=subsystems)


def carrier_subsystem():
    return subsystem(TemplateDocument.from_payload({
        "document": "template", "schema_version": 1, "name": "test_support", "functional_role": "generic",
        "allowed_placement_roles": ["any"], "symmetry": "asymmetric", "units": {"length": "m"},
        "bodies": [{"name": "carrier", "fixed": True, "position": [0, 0, .334]}],
        "hardpoints": [{"name": "center", "owner": "carrier"}], "joints": [], "elements": [], "property_slots": [],
        "ports": [{"name": "carrier", "role": "wheel_carrier", "owner": "carrier", "point": "center",
                   "cardinality": "many", "capabilities": ["mount", "contact_frame", "torque_reaction"]},
                  {"name": "support", "role": "support", "owner": "carrier", "point": "center", "cardinality": "many"},
                  {"name": "rack", "role": "rack", "owner": "carrier", "point": "center", "cardinality": "many"},
                  {"name": "road", "role": "road", "kind": "road", "resource": "plane"}],
    }), {"center": [0, 0, .334]})


@pytest.mark.parametrize("name", ["suspension", "steering", "body", "wheel", "brake", "drive", "anti_roll_bar"])
def test_all_presets_are_ordinary_serializable_templates(name, tmp_path):
    original = generic_template(name)
    assert type(original) is TemplateDocument
    assert TemplateDocument.load(original.save(tmp_path / "template.json")).payload == original.payload


@pytest.mark.parametrize("name", ["suspension", "steering", "body", "anti_roll_bar"])
def test_automotive_entities_come_only_from_template_rows(name):
    template = generic_template(name)
    coordinates = {point: [0, 0, .334] for point in template.hardpoint_names}
    if name == "suspension":
        coordinates["upper_rear"] = [1, 0, .334]
        coordinates["lower_rear"] = [1, 0, .334]
    built = assemble_generic(assembly({"support": carrier_subsystem(), "unit": subsystem(template, coordinates)}))
    units = [key for key in built.bodies if key.startswith("unit.")]
    assert len(units) == len(template.payload["bodies"]) * (2 if template.mirrors else 1)
    if name == "suspension":
        assert all("wheel" not in key for key in units)
        assert {tuple(row["axis_a"]) for row in built.joints if row["type"] == "revolute"} == {(1, 0, 0), (-1, 0, 0)}


def test_mode_selection_changes_only_declared_mounts():
    template = generic_template("suspension")
    coordinates = {point: [0, 0, .334] for point in template.hardpoint_names}
    coordinates.update(upper_rear=[1, 0, .334], lower_rear=[1, 0, .334])
    subs = {"support": carrier_subsystem(), "unit": subsystem(template, coordinates)}
    rigid, compliant = [assemble_generic(assembly(subs, mode=mode)) for mode in ("K", "C")]
    assert rigid.bodies.keys() == compliant.bodies.keys()
    assert len(rigid.joints) == 12
    assert len(compliant.joints) == 8
    assert len(compliant.elements) == 12
    assert len(rigid.elements) == 4


def test_wheel_owns_its_only_inertia_and_tire_and_bearing():
    built = assemble_generic(assembly({"support": carrier_subsystem(),
                                      "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]})}))
    assert list(built.bodies) == ["support.carrier", "wheel.wheel"]
    assert built.bodies["wheel.wheel"].mass == 20
    assert len(built.joints) == 1
    assert built.tires[0]["mass"] == 0
    assert built.tires[0]["parameters"]["frame_body"] == "support.carrier"
    assert built.resolved_model().to_document()["tires"][0]["body"] == "wheel.wheel"


def test_zero_body_drive_resolves_endpoints_after_assembly():
    wheel = subsystem(generic_template("wheel"), {"center": [0, 0, .334]})
    drive = subsystem(generic_template("drive"), signals={"drive_input": [[0, 1], [1, 1]]})
    built = assemble_generic(assembly({"support": carrier_subsystem(), "wheel": wheel, "drive": drive}))
    assert not any(name.startswith("drive.") for name in built.bodies)
    assert built.elements[0]["parameters"]["action"]["body"] == "wheel.wheel"
    assert built.elements[0]["parameters"]["reaction"]["body"] == "support.carrier"
    assert built.function_programs[0]["bindings"][-1]["source"] in {"property", "signal"}


def test_brake_keeps_existing_resistance_law_and_input_scaling():
    built = assemble_generic(assembly({"support": carrier_subsystem(),
        "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}),
        "brake": subsystem(generic_template("brake"))}))
    element = built.elements[0]
    assert element["type"] == "rotational_torque"
    assert element["parameters"]["stiffness"] == 29
    assert element["parameters"]["max_torque"] == 29
    assert element["body_a"] == "support.carrier"
    assert element["body_b"] == "wheel.wheel"


def test_undeclared_external_endpoint_and_duplicate_inertia_rejected():
    payload = generic_template("drive").to_payload()
    payload["elements"][0]["action"] = "@unknown"
    with pytest.raises(AuthoringError, match="unknown action"):
        TemplateDocument.from_payload(payload)
    payload = generic_template("wheel").to_payload()
    payload["tires"][0]["body"]["mass_slot"] = "mass"
    with pytest.raises(AuthoringError, match="also declared"):
        TemplateDocument.from_payload(payload)


def test_detailed_drive_uses_the_same_local_and_external_endpoints():
    payload = generic_template("drive").to_payload()
    payload["bodies"] = [{"name": "shaft", "mass": 3, "inertia": np.eye(3).tolist(), "position": [0, 0, .334]}]
    payload["hardpoints"] = [{"name": "center", "owner": "shaft"}]
    payload["markers"] = [{"name": "shaft", "owner": "shaft", "point": "center"}]
    payload["joints"] = [{"name": "shaft_mount", "type": "fixed", "body_a": "@action", "body_b": "shaft",
                          "point_a": "@action", "point_b": "center"}]
    payload["elements"][0]["action"] = "shaft"
    drive = subsystem(TemplateDocument.from_payload(payload), {"center": [0, 0, .334]}, signals={"drive_input": [[0, 1], [1, 1]]})
    built = assemble_generic(assembly({"support": carrier_subsystem(), "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}), "drive": drive}))
    assert built.bodies["drive.shaft"].mass == 3
    assert sum(body.mass for body in built.bodies.values()) == 23
    assert built.elements[0]["parameters"]["action"]["body"] == "drive.shaft"


def test_function_requires_explicit_control_signal():
    with pytest.raises(AuthoringError, match="unbound external endpoint"):
        GenericSubsystemAssembler().assemble(subsystem(generic_template("drive")))


def test_steering_configuration_is_data_and_keeps_same_bodies():
    steering = subsystem(generic_template("steering"), {"center": [0, 0, .334], "housing_center": [0, 0, .334]})
    payload = steering.to_payload()
    payload["parameters"]["configuration"] = "fixed"
    fixed = SubsystemDocument.from_payload(payload, template=steering.template, properties=steering.properties)
    guided_model, fixed_model = [assemble_generic(assembly({"support": carrier_subsystem(), "steering": model})) for model in (steering, fixed)]
    assert guided_model.bodies.keys() == fixed_model.bodies.keys()
    assert [row["type"] for row in guided_model.joints] == ["prismatic", "fixed"]
    assert [row["type"] for row in fixed_model.joints] == ["fixed", "fixed"]


def test_detailed_brake_keeps_rotor_caliper_and_wheel_mass_separate():
    payload = generic_template("brake").to_payload()
    payload["bodies"] = [{"name": name, "mass": mass, "position": [0, 0, .334], "inertia": np.eye(3).tolist()}
                         for name, mass in (("rotor", 2), ("caliper", 3))]
    payload["hardpoints"] = [{"name": name, "owner": name} for name in ("rotor", "caliper")]
    payload["joints"] = [{"name": name + "_mount", "type": "fixed", "body_a": endpoint, "body_b": name,
                          "point_a": endpoint, "point_b": name}
                         for name, endpoint in (("rotor", "@action"), ("caliper", "@reaction"))]
    payload["elements"][0].update(body_a="caliper", body_b="rotor", point_a="caliper", point_b="rotor")
    built = assemble_generic(assembly({"support": carrier_subsystem(),
        "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}),
        "brake": subsystem(TemplateDocument.from_payload(payload), {"rotor": [0, 0, .334], "caliper": [0, 0, .334]})}))
    assert sum(body.mass for body in built.bodies.values()) == 25
    assert built.bodies["wheel.wheel"].mass == 20
    assert built.tires[0]["mass"] == 0


@pytest.mark.parametrize("role", ["brake", "drive"])
def test_peer_force_subsystems_reach_real_native(role):
    from suspension_multibody.api import validate
    from suspension_multibody.simulation.runner import run_compiled

    declaration = assembly({"support": carrier_subsystem(),
        "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}),
        "actuator": subsystem(generic_template(role), signals={"drive_input": [[0, 1], [1, 1]]})})
    run = run_compiled(validate(declaration, {"schema_version": 1, "name": "actuator", "study": "dynamic",
        "samples": [0, .001, .002], "solver": {},
        "boundaries": [{"name": "hold", "coordinate": "wheel.spin", "mode": "locked", "value": 0, "units": "rad"}],
        "inputs": [], "outputs": []}))
    assert run.status == "success"
    assert run.raw.body_names == ("support.carrier", "wheel.wheel")
