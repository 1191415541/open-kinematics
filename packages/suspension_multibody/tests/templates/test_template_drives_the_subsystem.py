"""Template selection changes topology and native input, with no business builder."""

import copy

import numpy as np
import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring import TemplateDocument, assemble_generic
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.presets import generic_template
from tests.authoring.test_generic_multibody import _case
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)


def declared_suspension(payload=None, mode="K"):
    template = TemplateDocument.from_payload(payload) if payload is not None else generic_template("suspension")
    points = {name: [0, 0, .334] for name in template.hardpoint_names}
    points.update({name: [1, 0, .334] for name in ("upper_rear", "lower_rear") if name in points})
    return assembly({"support": carrier_subsystem(), "unit": subsystem(template, points)}, mode=mode)


def test_an_alternate_topology_changes_exactly_the_declared_joints():
    original = generic_template("suspension").to_payload()
    changed = copy.deepcopy(original)
    removed = [row for row in changed["joints"] if row["body_b"] == "upper_arm"]
    changed["joints"] = [row for row in changed["joints"] if row not in removed]
    before, after = [validate(declared_suspension(payload), _case()).model_document for payload in (original, changed)]
    assert before["bodies"] == after["bodies"]
    removed_ids = {"unit." + row["name"] + "_" + side for row in removed for side in ("L", "R")}
    assert {row["name"] for row in before["joints"]} - {row["name"] for row in after["joints"]} == removed_ids
    assert before["elements"] == after["elements"]


def test_a_template_omitting_a_part_creates_no_hidden_body():
    payload = generic_template("suspension").to_payload()
    payload["bodies"] = [row for row in payload["bodies"] if row["name"] != "upper_arm"]
    payload["hardpoints"] = [row for row in payload["hardpoints"] if row["owner"] != "upper_arm"]
    payload["joints"] = [row for row in payload["joints"] if "upper_arm" not in (row["body_a"], row["body_b"])]
    payload["elements"] = [row for row in payload["elements"] if "upper_arm" not in (row["body_a"], row["body_b"])]
    payload["ports"] = [row for row in payload["ports"] if row.get("owner") != "upper_arm"]
    built = assemble_generic(declared_suspension(payload))
    assert "unit.lower_arm_L" in built.bodies and "unit.upright_R" in built.bodies
    assert not any("upper_arm" in name for name in built.bodies)


def test_file_template_selection_is_the_same_interpreter(tmp_path):
    template = generic_template("steering")
    loaded = TemplateDocument.load(template.save(tmp_path / "selected.json"))
    def build(value):
        return assemble_generic(assembly({"support": carrier_subsystem(), "steering": subsystem(value, {"center": [0, 0, .334], "housing_center": [0, 0, .334]})})).model_document()
    assert build(template) == build(loaded)


def test_renaming_the_steering_guide_changes_the_native_row():
    payload = generic_template("steering").to_payload()
    for row in payload["joints"]:
        if row["type"] == "prismatic":
            row["name"] = "alternate_guide"
    built = assemble_generic(assembly({"support": carrier_subsystem(), "steering": subsystem(TemplateDocument.from_payload(payload), {"center": [0, 0, .334], "housing_center": [0, 0, .334]})}))
    assert "steering.alternate_guide" in {row["name"] for row in built.joints}


def test_body_template_controls_mass_and_fixedness():
    payload = generic_template("body").to_payload()
    payload["bodies"] = [{"name": "support", "fixed": True}, {"name": "frame", "fixed": False, "mass": 5, "inertia": np.eye(3).tolist()}]
    payload["hardpoints"] = []
    payload["ports"] = []
    built = assemble_generic(assembly({"unit": subsystem(TemplateDocument.from_payload(payload))}))
    assert built.bodies["unit.support"].fixed
    assert not built.bodies["unit.frame"].fixed and built.bodies["unit.frame"].mass == 5


def test_unknown_joint_type_is_rejected_by_name():
    payload = generic_template("suspension").to_payload()
    payload["joints"][0]["type"] = "no_such_joint"
    with pytest.raises(AuthoringError, match="no_such_joint"):
        TemplateDocument.from_payload(payload)
