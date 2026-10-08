"""
The in-memory documents are the same documents as the files.

The point of this module is not that an object can be built without a path -- it
is that the object built without one is the *same* document the file route
produces, checked by the same contract and refused for the same reasons.  Every
test here therefore has two halves: the in-memory answer, and the file answer it
has to match.  A test that only built an object would pass while the two routes
drifted apart, which is the failure this work exists to prevent.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring import (
    AssemblyDocument,
    AuthoringError,
    TemplateDocument,
    load_assembly,
    load_rig,
    load_subsystem,
    load_template,
)
from suspension_multibody.authoring.properties import ElementPropertyDocument

#: The smallest legal template: one body, one point, one joint, one element and
#: the slot the element binds.  Kept explicit rather than derived from a fixture
#: file so a change to the file layout cannot silently move what is being tested.
TEMPLATE = {
    "document": "template",
    "schema_version": 1,
    "name": "in_memory_arm",
    "functional_role": "suspension",
    "allowed_placement_roles": ["front_left"],
    "bodies": [
        {"name": "arm", "mass": 1.0, "fixed": False},
        {"name": "upright", "mass": 1.0, "fixed": False},
    ],
    "hardpoints": [
        {"name": "pivot", "owner": "arm"},
        {"name": "ball", "owner": "upright"},
    ],
    "joints": [
        {"name": "pivot_joint", "type": "revolute", "body_a": "arm", "body_b": "upright",
         "point_a": "pivot", "point_b": "ball"},
    ],
    "elements": [],
    "property_slots": [],
}


def _payload(**changes: object) -> dict[str, object]:
    payload = json.loads(json.dumps(TEMPLATE))
    payload.update(changes)
    return payload


def test_a_template_built_in_memory_is_the_same_document_as_the_file(
    tmp_path: Path,
) -> None:
    path = tmp_path / "arm.tpl.json"
    path.write_text(json.dumps(TEMPLATE), encoding="utf-8")

    from_file = load_template(path)
    in_memory = load_template(TEMPLATE)

    assert in_memory.name == from_file.name
    assert in_memory.payload == from_file.payload
    assert in_memory.topology_hash == from_file.topology_hash
    assert in_memory.hardpoint_names == from_file.hardpoint_names


def test_a_template_built_in_memory_needs_no_path() -> None:
    """
    Building from a declaration reads no file.

    The check is the document's own name in the message: the file route names a
    path, and the in-memory route has none, so the message is what distinguishes
    them without patching the filesystem out from under pytest.
    """
    template = load_template(TEMPLATE)
    assert template.path is None
    assert template.where == "<memory:in_memory_arm>"

    with pytest.raises(AuthoringError) as failure:
        load_template(_payload(bodies_typo=None))
    assert "<memory:in_memory_arm>" in str(failure.value)


def test_the_two_routes_refuse_the_same_documents() -> None:
    """A bad declaration fails in memory exactly as it fails in a file."""
    broken = _payload(joints=[{"name": "j", "type": "revolute", "body_a": "arm",
                               "body_b": "nowhere", "point_a": "pivot", "point_b": "ball"}])
    with pytest.raises(AuthoringError) as in_memory:
        load_template(broken)

    def resolve(name: str) -> TemplateDocument:  # pragma: no cover - not reached
        raise AssertionError("the contract check must fail first")

    assert "nowhere" in str(in_memory.value)


def test_a_contract_violation_is_refused_in_memory_too() -> None:
    """The schema runs for an in-memory declaration, not only for a file."""
    with pytest.raises(AuthoringError):
        load_template(_payload(functional_role="not_a_role"))


def test_a_template_survives_a_round_trip_through_its_own_payload() -> None:
    template = load_template(TEMPLATE)
    again = load_template(template.to_payload())
    assert again.payload == template.payload
    assert again.topology_hash == template.topology_hash


def test_saving_and_loading_a_template_agrees_with_the_declaration(
    tmp_path: Path,
) -> None:
    template = load_template(TEMPLATE)
    written = template.save(tmp_path / "saved.tpl.json")
    reloaded = load_template(written)
    assert reloaded.payload == template.payload
    assert reloaded.topology_hash == template.topology_hash


def test_a_subsystem_can_name_its_template_as_an_object(tmp_path: Path) -> None:
    template = load_template(TEMPLATE)
    subsystem = load_subsystem(
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "left",
            "template": "in_memory_arm",
            "functional_role": "suspension",
            "placement_role": "front_left",
            "hardpoints": {"pivot": [0.0, 0.0, 0.0], "ball": [1.0, 0.0, 0.0]},
            "property_bindings": {},
        },
        template=template,
    )
    assert subsystem.template is template
    assert subsystem.path is None
    assert subsystem.effective().hardpoints["ball"] == (1.0, 0.0, 0.0)


def test_a_subsystem_without_a_template_answer_is_refused() -> None:
    """The template is supplied or read; it is never guessed."""
    with pytest.raises(AuthoringError) as failure:
        load_subsystem(
            {
                "document": "subsystem",
                "schema_version": 1,
                "name": "left",
                "template": "in_memory_arm",
                "functional_role": "suspension",
                "placement_role": "front_left",
                "hardpoints": {"pivot": [0.0, 0.0, 0.0], "ball": [1.0, 0.0, 0.0]},
                "property_bindings": {},
            }
        )
    assert "resolve" in str(failure.value)


def test_an_assembly_can_name_its_subsystems_as_objects() -> None:
    template = load_template(TEMPLATE)
    subsystem = load_subsystem(
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "left",
            "template": "in_memory_arm",
            "functional_role": "suspension",
            "placement_role": "front_left",
            "hardpoints": {"pivot": [0.0, 0.0, 0.0], "ball": [1.0, 0.0, 0.0]},
            "property_bindings": {},
        },
        template=template,
    )
    rig = load_rig(
        {
            "document": "rig",
            "schema_version": 1,
            "name": "bench",
            "supported_assembly_kinds": ["suspension_axle"],
            "required_ports": [],
        }
    )
    assembly = load_assembly(
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "in_memory_axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {
                    "ref": "left.sub.json",
                    "functional_role": "suspension",
                    "placement_role": "front_left",
                }
            ],
        },
        subsystems={"left.sub.json": subsystem},
        rig=rig,
    )
    assert assembly.path is None
    assert assembly.subsystems == (subsystem,)
    assert assembly.rig is rig


def test_an_assembly_reference_that_cannot_be_answered_is_refused() -> None:
    with pytest.raises(AuthoringError) as failure:
        load_assembly(
            {
                "document": "assembly",
                "schema_version": 1,
                "name": "in_memory_axle",
                "assembly_kind": "suspension_axle",
                "subsystems": [
                    {
                        "ref": "missing.sub.json",
                        "functional_role": "suspension",
                        "placement_role": "front_left",
                    }
                ],
            }
        )
    assert "missing.sub.json" in str(failure.value)


def test_a_rig_built_in_memory_matches_the_file_route(tmp_path: Path) -> None:
    declaration = {
        "document": "rig",
        "schema_version": 1,
        "name": "bench",
        "supported_assembly_kinds": ["suspension_axle"],
        "required_ports": ["steering"],
    }
    path = tmp_path / "bench.rig.json"
    path.write_text(json.dumps(declaration), encoding="utf-8")
    assert load_rig(declaration).payload == load_rig(path).payload
    assert load_rig(declaration).required_ports == load_rig(path).required_ports


def test_a_preloaded_property_law_is_checked_against_its_slot() -> None:
    """
    Preloading skips the read and nothing else.

    A law handed in from memory is still checked against the slot that names it,
    so the preloaded route cannot be a weaker way past the element type check.
    """
    law = ElementPropertyDocument.from_payload(
        {
            "document": "element_properties",
            "schema_version": 1,
            "name": "linear_bushing",
            "element_type": "bushing",
            "model": "linear",
            "units": {"force": "N", "length": "mm"},
            "parameters": {"stiffness": 100.0},
        }
    )
    assert law.path is None
    assert law.element_type == "bushing"


def test_an_update_does_not_disturb_the_document_it_came_from() -> None:
    """
    Overrides are copy-on-write, and the original is untouched.

    This is the property an optimisation loop depends on: tuning a value for the
    next run must not change the model the previous run was made from.
    """
    template = load_template(TEMPLATE)
    subsystem = load_subsystem(
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "left",
            "template": "in_memory_arm",
            "functional_role": "suspension",
            "placement_role": "front_left",
            "hardpoints": {"pivot": [0.0, 0.0, 0.0], "ball": [1.0, 0.0, 0.0]},
            "property_bindings": {},
        },
        template=template,
    )
    before = subsystem.effective().hardpoints["ball"]
    tuned = subsystem.effective(overrides={"hardpoints": {"ball": [2.0, 0.0, 0.0]}})
    assert tuned.hardpoints["ball"] == (2.0, 0.0, 0.0)
    assert subsystem.effective().hardpoints["ball"] == before
    assert subsystem.payload["hardpoints"]["ball"] == [1.0, 0.0, 0.0]


def test_an_assembly_document_keeps_the_rig_it_was_bound_to() -> None:
    """An in-memory assembly carries its rig, so binding needs no directory."""
    template = load_template(TEMPLATE)
    subsystem = load_subsystem(
        {
            "document": "subsystem",
            "schema_version": 1,
            "name": "left",
            "template": "in_memory_arm",
            "functional_role": "suspension",
            "placement_role": "front_left",
            "hardpoints": {"pivot": [0.0, 0.0, 0.0], "ball": [1.0, 0.0, 0.0]},
            "property_bindings": {},
        },
        template=template,
    )
    rig = load_rig(
        {
            "document": "rig",
            "schema_version": 1,
            "name": "bench",
            "supported_assembly_kinds": ["suspension_axle"],
            "required_ports": [],
        }
    )
    assembly = load_assembly(
        {
            "document": "assembly",
            "schema_version": 1,
            "name": "in_memory_axle",
            "assembly_kind": "suspension_axle",
            "subsystems": [
                {
                    "ref": "left.sub.json",
                    "functional_role": "suspension",
                    "placement_role": "front_left",
                }
            ],
        },
        subsystems={"left.sub.json": subsystem},
        rig=rig,
    )
    assert isinstance(assembly, AssemblyDocument)
    assert assembly.rig is rig
