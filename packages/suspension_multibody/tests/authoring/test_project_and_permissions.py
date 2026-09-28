"""
The project directory and the two editing surfaces.

These tests are about what the *files* guarantee: a project loads whole or not at
all, an edit changes exactly the hashes it should, and the user-facing calls cannot
write a topology change no matter how they are called.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring import (
    EXPERT_FIELDS,
    USER_ASSEMBLY_FIELDS,
    USER_FIELDS,
    AuthoringPermissionError,
    ExpertAuthoring,
    Project,
    ProjectError,
    UserAuthoring,
)

from .fixtures import COORDINATES, ROLES, write_axle_project

_TEMPLATE_BODY = {
    "functional_role": "suspension",
    "allowed_placement_roles": ["front", "rear"],
    "bodies": [
        {"name": "chassis", "fixed": True},
        {"name": "rack"},
        {"name": "upper_arm_L"},
        {"name": "lower_arm_L"},
        {"name": "upright_L"},
        {"name": "tie_rod_L"},
    ],
    "hardpoints": [
        {"name": role, "owner": owner, "label": label}
        for role, (owner, label) in ROLES.items()
    ],
    "joints": [
        {
            "name": "upper_pivot",
            "type": "revolute",
            "body_a": "chassis",
            "body_b": "upper_arm_L",
            "point_a": "upper_front",
            "axis_reference": "upper_rear",
        },
        {
            "name": "lower_pivot",
            "type": "revolute",
            "body_a": "chassis",
            "body_b": "lower_arm_L",
            "point_a": "lower_front",
            "axis_reference": "lower_rear",
        },
    ],
    "elements": [
        {
            "name": "spring",
            "type": "spring",
            "body_a": "chassis",
            "body_b": "lower_arm_L",
            "point_a": "upper_front",
            "point_b": "lower_outer",
            "property_slot": "spring",
        }
    ],
    "property_slots": [
        {"name": "spring", "element_type": "spring", "required": True},
    ],
}


def _hardpoints() -> dict[str, list[float]]:
    return {role: list(point) for role, point in COORDINATES.items()}


def test_project_loads_every_kind_it_declares(tmp_path: Path) -> None:
    write_axle_project(tmp_path)
    project = Project.load(tmp_path)
    assert project.names("template") == ("file_chassis", "file_double_wishbone")
    assert project.names("subsystem") == ("chassis", "front_suspension")
    assert project.names("assembly") == ("front_axle",)
    assert project.names("rig") == ("kc_bench",)
    assert project.names("element_properties") == ("axle_spring",)
    # The whole project resolves: an assembly can be loaded with its rig.
    assert project.simulation("front_axle").rig.name == "kc_bench"


def test_project_hashes_separate_the_file_from_the_model(tmp_path: Path) -> None:
    """
    A file hash follows the bytes; a model hash follows the model.

    Rewriting a subsystem with different whitespace changes the file the run read
    and not the model it built, which is why both are recorded rather than one.
    """
    paths = write_axle_project(tmp_path)
    project = Project.load(tmp_path)
    before_files = project.hashes()
    before_model = project.provenance("front_axle")["model"]

    payload = json.loads(paths["subsystem"].read_text(encoding="utf-8"))
    paths["subsystem"].write_text(json.dumps(payload, sort_keys=True), encoding="utf-8")

    after = Project.load(tmp_path)
    assert after.hashes()["front.sub.json"] != before_files["front.sub.json"]
    assert after.provenance("front_axle")["model"]["subsystems"]["front.sub.json"][
        "topology"
    ] == before_model["subsystems"]["front.sub.json"]["topology"]


def test_project_refuses_a_reference_that_does_not_land(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["subsystem"].read_text(encoding="utf-8"))
    payload["template"] = "nowhere.tpl.json"
    paths["subsystem"].write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ProjectError, match="does not exist"):
        Project.load(tmp_path)


def test_project_refuses_a_reference_of_the_wrong_kind(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["subsystem"].read_text(encoding="utf-8"))
    payload["template"] = "rig.json"
    paths["subsystem"].write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ProjectError, match="declares itself a rig"):
        Project.load(tmp_path)


def test_project_refuses_two_documents_of_one_name(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    duplicate = tmp_path / "nested"
    duplicate.mkdir()
    payload = json.loads(paths["rig"].read_text(encoding="utf-8"))
    (duplicate / "rig2.json").write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ProjectError, match="already declared"):
        Project.load(tmp_path)


def test_project_refuses_an_unknown_schema_version(tmp_path: Path) -> None:
    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["rig"].read_text(encoding="utf-8"))
    payload["schema_version"] = 2
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ProjectError, match="schema_version"):
        Project.load(tmp_path)


def test_expert_creates_a_template_and_a_user_places_it(tmp_path: Path) -> None:
    """The two surfaces together produce a project that loads and resolves."""
    write_axle_project(tmp_path)
    expert = ExpertAuthoring(tmp_path)
    expert.create_template("expert_axle", **_TEMPLATE_BODY)

    user = UserAuthoring(tmp_path)
    revision = user.create_subsystem(
        "expert_front",
        template="expert_axle.tpl.json",
        functional_role="suspension",
        placement_role="front",
        hardpoints=_hardpoints(),
        property_bindings={"spring": "spring.json"},
    )
    project = Project.load(tmp_path)
    assert "expert_front" in project.names("subsystem")
    effective = project.subsystem("expert_front").effective()
    assert effective.hardpoints["wheel_center"] == COORDINATES["wheel_center"]
    # The user moved a declared point, and the template's topology hash held.
    moved = user.set_hardpoints(revision, {"wheel_center": [0.0, -710.0, 300.0]})
    assert moved.content_hash != revision.content_hash
    reloaded = Project.load(tmp_path)
    assert reloaded.subsystem("expert_front").effective().hardpoints["wheel_center"] == (
        0.0,
        -710.0,
        300.0,
    )
    assert (
        reloaded.subsystem("expert_front").topology_hash
        == reloaded.template("expert_axle").topology_hash
    )


def test_user_cannot_state_topology(tmp_path: Path) -> None:
    """Every route a user could take to a topology field is refused by name."""
    write_axle_project(tmp_path)
    ExpertAuthoring(tmp_path).create_template("expert_axle", **_TEMPLATE_BODY)
    user = UserAuthoring(tmp_path)
    with pytest.raises(AuthoringPermissionError, match="bodies"):
        user.create_subsystem(
            "cheating",
            template="expert_axle.tpl.json",
            functional_role="suspension",
            placement_role="front",
            hardpoints=_hardpoints(),
            property_bindings={"spring": "spring.json"},
            bodies=[{"name": "illegal"}],
        )
    with pytest.raises(AuthoringPermissionError, match="joints"):
        user.create_assembly(
            "cheating",
            assembly_kind="suspension_axle",
            subsystems=[
                {
                    "ref": "front.sub.json",
                    "functional_role": "suspension",
                    "placement_role": "front",
                    "overrides": {"joints": [{"name": "illegal"}]},
                }
            ],
        )


def test_the_two_surfaces_do_not_share_the_topology_fields() -> None:
    """
    The expert's reach is strictly wider, and the difference *is* the topology.

    Stated as a test because it is the property the whole split rests on: if a
    topology field leaked into the user's list, the boundary would be a naming
    convention rather than a rule.
    """
    assert {"bodies", "joints", "elements", "property_slots", "ports", "outputs"} <= (
        EXPERT_FIELDS - USER_FIELDS
    )
    assert "hardpoints" in USER_FIELDS  # a *value* in a subsystem ...
    assert "hardpoints" in EXPERT_FIELDS  # ... and topology in a template
    assert USER_ASSEMBLY_FIELDS.isdisjoint({"bodies", "joints", "elements"})


def test_assembly_override_leaves_the_subsystem_file_alone(tmp_path: Path) -> None:
    """An override lives in the assembly; the document it names is untouched."""
    paths = write_axle_project(tmp_path)
    before = paths["subsystem"].read_bytes()
    user = UserAuthoring(tmp_path)
    revision = user.create_assembly(
        "axle_with_override",
        assembly_kind="suspension_axle",
        subsystems=[
            {"ref": "front.sub.json", "functional_role": "suspension", "placement_role": "front"},
            {"ref": "chassis.sub.json", "functional_role": "chassis", "placement_role": "any"},
        ],
        rig="rig.json",
    )
    user.override(revision, "front.sub.json", hardpoints={"wheel_center": [0.0, -705.0, 300.0]})
    assert paths["subsystem"].read_bytes() == before

    project = Project.load(tmp_path)
    assembly = project.assembly("axle_with_override")
    assert assembly.entries[0].effective().hardpoints["wheel_center"] == (0.0, -705.0, 300.0)
    assert project.subsystem("front_suspension").effective().hardpoints["wheel_center"] == (
        COORDINATES["wheel_center"]
    )


def test_expert_replacement_moves_the_topology_hash(tmp_path: Path) -> None:
    write_axle_project(tmp_path)
    expert = ExpertAuthoring(tmp_path)
    revision = expert.create_template("expert_axle", **_TEMPLATE_BODY)
    before = revision.content_hash
    changed = dict(_TEMPLATE_BODY)
    changed["bodies"] = [*_TEMPLATE_BODY["bodies"], {"name": "extra_arm_L"}]
    after = expert.replace_template(revision, **changed)
    assert after.content_hash != before
    loaded = Project.load(tmp_path).template("expert_axle")
    assert "extra_arm_L" in loaded.body_names
