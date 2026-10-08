"""Rigs declare input bindings and never create replacement wheel/tire entities."""

from __future__ import annotations

import pytest

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.presets import generic_template
from suspension_multibody.rigs import RIGS
from suspension_multibody.rigs.templates import generic_rig_subsystem
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)


@pytest.mark.parametrize("name", tuple(RIGS))
def test_every_bench_uses_one_ordinary_subsystem(name):
    interfaces = {drive.coordinate: {"role": "wheel_hub", "kind": "geometry", "required": False}
                  for drive in RIGS[name].drives if drive.from_assembly}
    rig = generic_rig_subsystem(name, interfaces=interfaces)
    assert [row["name"] for row in rig.template.payload["bodies"]] == ["fixture"]
    assert not rig.template.payload.get("tires")
    assert {row["name"] for row in rig.template.payload["inputs"]} == set(RIGS[name].coordinate_names())


def test_optional_rig_inputs_and_outputs_disappear_together():
    rig = generic_rig_subsystem("kc_quasi_static", interfaces={
        drive.coordinate: {"role": "wheel_hub", "kind": "geometry", "required": False}
        for drive in RIGS["kc_quasi_static"].drives if drive.from_assembly
    })
    built = assemble_generic(assembly({"rig": rig}))
    assert not built.inputs
    assert not built.fragments[0].outputs
    assert list(built.bodies) == ["rig.fixture"]


def test_rig_binds_named_wheel_without_creating_a_second_one():
    interfaces = {drive.coordinate: {"role": "wheel_hub", "kind": "geometry", "required": False}
                  for drive in RIGS["kc_quasi_static"].drives if drive.from_assembly}
    rig = generic_rig_subsystem("kc_quasi_static", interfaces=interfaces)
    wheel = subsystem(generic_template("wheel"), {"center": [0, 0, .334]})
    built = assemble_generic(assembly({"fixture": carrier_subsystem(), "arbitrary": wheel, "rig": rig},
        pairings={"arbitrary": {"carrier": "fixture.carrier", "road": "fixture.road"}}))
    assert len(built.tires) == 1
    assert list(built.bodies).count("arbitrary.wheel") == 1
    assert all(row["owner"] == "arbitrary.wheel" for row in built.inputs)
    assert all(row["port"] == "arbitrary/hub" for row in built.inputs)


def test_required_input_is_not_silently_deleted():
    rig = generic_rig_subsystem("kc_quasi_static", interfaces={
        drive.coordinate: {"role": "wheel_hub", "kind": "geometry"}
        for drive in RIGS["kc_quasi_static"].drives if drive.from_assembly
    })
    with pytest.raises(ValueError, match="required port"):
        assemble_generic(assembly({"rig": rig}))


def test_four_post_remains_ideal_road_input_data():
    rig = generic_rig_subsystem("ride_four_post", interfaces={})
    built = assemble_generic(assembly({"rig": rig}))
    assert list(built.bodies) == ["rig.fixture"]
    assert not built.joints
    assert {row["name"] for row in built.inputs} == {"rig.road_height_fl", "rig.road_height_fr", "rig.road_height_rl", "rig.road_height_rr"}
