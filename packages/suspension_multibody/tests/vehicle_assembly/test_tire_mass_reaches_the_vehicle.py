"""Vehicle tire inertia reaches the same compiler as an axle tire."""

from __future__ import annotations

from suspension_multibody.api import validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    ElementPropertyDocument,
    SubsystemDocument,
    TemplateDocument,
    migrate_v1_vehicle_case,
)
from tests.vehicle.vehicle_fixtures import _case, _vehicle

_INERTIA = ((0.7, 0.0, 0.0), (0.0, 1.1, 0.0), (0.0, 0.0, 0.7))


def tire_document(mass=0.0, inertia=None, friction=1.0):
    vehicle = _vehicle()
    vehicle = vehicle.model_copy(update={"wheels": tuple(
        wheel.model_copy(update={"tire_mass": mass}) if wheel.name == "front_left" else wheel
        for wheel in vehicle.wheels)})
    source = _case(vehicle)
    source = source.model_copy(update={"road": source.road.model_copy(update={"friction_coefficient": friction})})
    assembly, case = migrate_v1_vehicle_case(source)
    if inertia is not None:
        subsystems = {entry.ref: entry.subsystem for entry in assembly.entries}
        owner = next(entry for entry in assembly.entries
            if any(row["name"] == "front_left" for row in entry.subsystem.template.payload.get("tires", ())))
        template = owner.subsystem.template.to_payload()
        template["property_slots"].append({"name": "tire_inertia", "element_type": "inertia", "required": True})
        template["tires"][0]["inertia_slot"] = "tire_inertia"
        properties = {**owner.properties, "tire_inertia": ElementPropertyDocument.from_payload({
            "document": "element_properties", "schema_version": 1, "name": "tire_inertia",
            "element_type": "inertia", "model": "linear", "units": {"length": "m", "force": "N"},
            "matrix": {"name": "inertia", "rows": [list(row) for row in inertia]}})}
        payload = owner.subsystem.to_payload()
        payload["property_bindings"]["tire_inertia"] = "tire_inertia.property.json"
        subsystems[owner.ref] = SubsystemDocument.from_payload(payload,
            template=TemplateDocument.from_payload(template), properties=properties)
        assembly = AssemblyDocument.from_payload(assembly.to_payload(), subsystems=subsystems,
            properties={entry.ref: properties if entry.ref == owner.ref else entry.properties for entry in assembly.entries})
    compiled = validate(assembly, case)
    assert len(compiled.model_document["tires"]) == 4
    assert len(compiled.model_document["bodies"]) == 29
    assert all(row["mass"] == 0 for row in compiled.model_document["tires"] if not row["name"].endswith(".front_left"))
    return next(row for row in compiled.model_document["tires"] if row["name"].endswith(".front_left"))


def test_a_vehicle_tire_without_mass_has_no_inertia_contribution() -> None:
    entry = tire_document()
    assert entry["mass"] == 0
    assert entry["inertia"] == [[0, 0, 0]] * 3


def test_a_vehicle_tire_with_mass_emits_it() -> None:
    entry = tire_document(mass=5.0)
    assert entry["mass"] == 5.0


def test_a_vehicle_tire_inertia_is_emitted_as_a_matrix() -> None:
    entry = tire_document(mass=5.0, inertia=_INERTIA)
    assert entry["inertia"] == [list(row) for row in _INERTIA]


def test_road_friction_scaling_preserves_the_tire_mass() -> None:
    entry = tire_document(mass=5.0, friction=0.5)
    assert entry["mass"] == 5.0
    assert entry["parameters"]["longitudinal_friction_coefficient"] == 0.5
    assert entry["parameters"]["lateral_friction_coefficient"] == 0.5


def test_the_vehicle_entry_keeps_the_keys_the_contract_requires() -> None:
    entry = tire_document(mass=5.0)
    for key in ("name", "model", "body", "parameters"):
        assert key in entry
