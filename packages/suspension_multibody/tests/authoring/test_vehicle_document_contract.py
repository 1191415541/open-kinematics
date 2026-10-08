"""
04: a full-vehicle assembly document is a checked declaration, not a free payload.

The vehicle-level numbers a full-vehicle assembly needs -- the chassis, the four
wheel ends, the steering system and the driveline -- live in the document's
``vehicle`` section.  These tests hold the two properties that make that section
worth having:

* a part that is wrong is refused **by name**, so the report says which part and
  which field rather than losing both in a nested validation error, and
* the model built from the document is the model the file describes -- the axles
  come from the document's own suspension subsystems, down to every body.

They also hold the registry property the routing depends on: each of the five
vehicle benches declares its own drives, so a document does not have to know how
a bench is spelled to be driven by it.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring import AssemblyDocument
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.vehicle import vehicle_declaration_from
from suspension_multibody.rigs.rig import RIGS

from .fixtures import write_vehicle_project


@pytest.fixture
def vehicle_paths(tmp_path: Path) -> dict[str, Path]:
    return write_vehicle_project(tmp_path)


def _section(paths: dict[str, Path]) -> dict:
    return json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))["vehicle"]


def _load(tmp_path: Path, payload: dict) -> AssemblyDocument:
    """Write a whole project with one document replaced, then load it."""
    paths = write_vehicle_project(tmp_path)
    paths["vehicle_assembly"].write_text(
        json.dumps(payload, indent=2), encoding="utf-8"
    )
    return AssemblyDocument.load(paths["vehicle_assembly"])


def _payload(paths: dict[str, Path]) -> dict:
    return json.loads(paths["vehicle_assembly"].read_text(encoding="utf-8"))


def test_the_vehicle_section_is_read_as_a_declaration(vehicle_paths):
    """The document's own section is the model's vehicle-level numbers."""
    document = AssemblyDocument.load(vehicle_paths["vehicle_assembly"])
    assert document.assembly_kind == "full_vehicle"
    section = document.payload["vehicle"]
    assert sorted(section) == ["chassis", "driveline", "steering", "wheels"]

    model = vehicle_declaration_from(document)
    assert model.chassis.mass == pytest.approx(section["chassis"]["mass"])
    assert [wheel.name for wheel in model.wheels] == [
        row["name"] for row in section["wheels"]
    ]
    assert model.steering.ratio == pytest.approx(section["steering"]["ratio"])


def test_the_axles_come_from_the_documents_own_subsystems(vehicle_paths):
    """
    The two axles are the document's suspension subsystems, not a re-declaration.

    A vehicle that carried its axles as numbers *and* placed subsystems would be
    two descriptions of one axle, and the two would drift the moment either moved.
    """
    document = AssemblyDocument.load(vehicle_paths["vehicle_assembly"])
    model = vehicle_declaration_from(document)
    assert model.front_axle.bodies
    assert model.rear_axle.bodies
    # The placed subsystems are what the axles were built from, so a body the
    # document's front subsystem declares is a body of the front axle.
    front = next(
        entry for entry in document.entries if entry.placement_role == "front"
    )
    declared = {
        str(row["name"]) for row in front.subsystem.template.payload["bodies"]
    }
    named = {body.name for body in model.front_axle.bodies}
    assert declared & named, "the axle shares no body with the subsystem it came from"


def test_a_missing_section_is_refused_naming_the_section(tmp_path):
    paths = write_vehicle_project(tmp_path)
    payload = _payload(paths)
    payload.pop("vehicle")
    with pytest.raises(AuthoringError, match="'vehicle' section"):
        vehicle_declaration_from(_load(tmp_path, payload))


def test_a_part_that_is_wrong_is_refused_naming_the_part(tmp_path):
    """
    The report names the part, not only the field.

    A nested ``model_validate`` reports the field and loses which object it was
    in; "ratio must be greater than 0" is a worse report than
    "vehicle.steering: ratio must be greater than 0".
    """
    paths = write_vehicle_project(tmp_path)
    payload = _payload(paths)
    payload["vehicle"]["steering"] = {"ratio": -1.0}
    with pytest.raises(AuthoringError, match="vehicle.steering"):
        vehicle_declaration_from(_load(tmp_path, payload))


def test_a_missing_wheel_is_refused_by_the_vehicle_rules(tmp_path):
    """Four corners is the vehicle's own rule, checked where the model is built."""
    paths = write_vehicle_project(tmp_path)
    payload = _payload(paths)
    payload["vehicle"]["wheels"] = payload["vehicle"]["wheels"][:3]
    with pytest.raises(AuthoringError):
        vehicle_declaration_from(_load(tmp_path, payload))


def test_a_driveline_naming_an_undefined_wheel_is_refused(tmp_path):
    paths = write_vehicle_project(tmp_path)
    payload = _payload(paths)
    payload["vehicle"]["driveline"] = {
        "driven_wheels": ["nowhere_left"],
        "drive_split": [1.0],
    }
    with pytest.raises(AuthoringError, match="driveline"):
        vehicle_declaration_from(_load(tmp_path, payload))


def test_every_vehicle_bench_declares_its_own_drives():
    """
    The five vehicle benches are registrations, each stating what it drives.

    A document is driven by the bench it names, so "which inputs this run has" is
    a property of the registration rather than something an entry has to know.
    """
    expected = {
        "vehicle_kc": {"wheel_drive_L", "wheel_drive_R", "rack_drive"},
        "vehicle_dynamic": {"road_height"},
        "handling": {"steering_wheel_angle", "road_height"},
        "ride_four_post": {
            "road_height_fl",
            "road_height_fr",
            "road_height_rl",
            "road_height_rr",
        },
        "ride_random_road": {"road_height"},
    }
    for bench, drives in expected.items():
        assert set(RIGS[bench].coordinate_names()) == drives


def test_the_four_post_bench_declares_one_input_per_corner():
    """The corner inputs are the bench's own declaration, not derived per run."""
    names = RIGS["ride_four_post"].coordinate_names()
    assert len(names) == 4
    assert len(set(names)) == 4
