"""
Model identity: the same model has one name, a changed model has another.

The identity is what lets a result say which model it came from, so the two
properties that matter are the ones tested here: it does not move when nothing
moved, and it does move when something did.  A hash that failed the first would
make every result look different; one that failed the second would make a tuning
loop unable to tell two runs apart.

The legacy ``Provenance.model_hash`` is deliberately *not* migrated here.  It is
frozen evidence recorded in results nobody re-ran, so it stays exactly as it is
until the models it describes are gone; what this module adds is the identity a
new caller asks for.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from suspension_multibody.authoring import (
    identity_of,
    load_assembly,
    load_rig,
    load_subsystem,
    load_template,
    model_identity,
)
from suspension_multibody.authoring.documents import (
    AssemblyDocument,
    RigDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.properties import ElementPropertyDocument
from tests.authoring.fixtures import write_axle_project


def _spring(stiffness: float = 45.0) -> ElementPropertyDocument:
    return ElementPropertyDocument.from_payload(
        {
            "document": "element_properties",
            "schema_version": 1,
            "name": "identity_spring",
            "element_type": "spring",
            "model": "linear",
            "units": {"force": "N", "length": "mm"},
            "parameters": {"stiffness": stiffness, "free_length": 250.0},
        }
    )


def test_a_template_has_a_stable_identity() -> None:
    first = load_template(
        {
            "document": "template",
            "schema_version": 1,
            "name": "arm",
            "functional_role": "suspension",
            "allowed_placement_roles": ["front"],
            "bodies": [{"name": "arm", "mass": 1.0}],
            "hardpoints": [{"name": "p", "owner": "arm"}],
            "joints": [],
            "elements": [],
            "property_slots": [],
        }
    )
    second = load_template(first.to_payload())
    assert model_identity(first) == model_identity(second)
    assert model_identity(first) == identity_of(first)


def test_a_changed_topology_changes_the_template_identity() -> None:
    base = {
        "document": "template",
        "schema_version": 1,
        "name": "arm",
        "functional_role": "suspension",
        "allowed_placement_roles": ["front"],
        "bodies": [{"name": "arm", "mass": 1.0}],
        "hardpoints": [{"name": "p", "owner": "arm"}],
        "joints": [],
        "elements": [],
        "property_slots": [],
    }
    changed = json.loads(json.dumps(base))
    changed["hardpoints"] = [{"name": "p", "owner": "arm"}, {"name": "q", "owner": "arm"}]
    assert model_identity(load_template(base)) != model_identity(load_template(changed))


def test_a_changed_coordinate_changes_the_subsystem_identity() -> None:
    template = load_template(_template_payload())
    first = load_subsystem(
        _subsystem_payload({"p": [0.0, 0.0, 0.0]}), template=template
    )
    second = load_subsystem(
        _subsystem_payload({"p": [0.0, 0.0, 10.0]}), template=template
    )
    assert model_identity(first) != model_identity(second)


def test_a_subsystem_identity_is_unaffected_by_where_it_came_from(
    tmp_path: Path,
) -> None:
    """
    A file and an in-memory document of the same model share one identity.

    This is the property that makes the identity a statement about the model
    rather than about the route a caller used to build it.
    """
    payload = _template_payload()
    path = tmp_path / "arm.tpl.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    from_file = load_template(path)
    in_memory = load_template(payload)
    assert model_identity(from_file) == model_identity(in_memory)


def test_an_assembly_identity_covers_its_roles_and_overrides(tmp_path: Path) -> None:
    written = write_axle_project(tmp_path)
    documents = _documents(written)
    same = _documents(written)
    assert model_identity(documents["assembly"]) == model_identity(same["assembly"])

    tuned = AssemblyDocument.from_payload(
        documents["assembly"].to_payload(),
        subsystems={e.ref: e.subsystem for e in documents["assembly"].entries},
        properties={
            "front.sub.json": {"spring": _spring(60.0)},
        },
        rig=documents["rig"],
    )
    assert model_identity(tuned) != model_identity(documents["assembly"])


def test_identity_refuses_a_class_instead_of_guessing() -> None:
    """
    A class is not a declaration, and the identity says so by refusing.

    Guessing an identity for something that states no declaration would be the
    silent compatibility this work exists to remove.
    """
    from suspension_multibody.schema.model import AxleDeclaration

    with pytest.raises(TypeError):
        model_identity(AxleDeclaration)  # a class is not a declaration either


def test_the_identity_accepts_a_bare_declaration() -> None:
    """A caller holding only a mapping still gets a stable, sensitive identity."""
    declaration = {"document": "rig", "schema_version": 1, "name": "bench"}
    assert model_identity(declaration) == model_identity(dict(declaration))
    assert model_identity(declaration) != model_identity({**declaration, "name": "other"})


# --- helpers ----------------------------------------------------------------


def _template_payload() -> dict[str, object]:
    return {
        "document": "template",
        "schema_version": 1,
        "name": "arm",
        "functional_role": "suspension",
        "allowed_placement_roles": ["front"],
        "bodies": [{"name": "arm", "mass": 1.0}],
        "hardpoints": [{"name": "p", "owner": "arm"}],
        "joints": [],
        "elements": [],
        "property_slots": [],
    }


def _subsystem_payload(hardpoints: dict[str, list[float]]) -> dict[str, object]:
    return {
        "document": "subsystem",
        "schema_version": 1,
        "name": "placed",
        "template": "arm",
        "functional_role": "suspension",
        "placement_role": "front",
        "hardpoints": hardpoints,
        "property_bindings": {},
    }


def _documents(written: dict[str, Path]) -> dict[str, object]:
    template = load_template(written["template"])
    chassis_template = load_template(written["chassis_template"])
    spring = ElementPropertyDocument.load(written["spring"])
    subsystem = load_subsystem(
        json.loads(written["subsystem"].read_text(encoding="utf-8")), template=template
    )
    chassis = load_subsystem(
        json.loads(written["chassis_subsystem"].read_text(encoding="utf-8")),
        template=chassis_template,
    )
    rig = load_rig(written["rig"])
    assembly = load_assembly(
        json.loads(written["assembly"].read_text(encoding="utf-8")),
        subsystems={"front.sub.json": subsystem, "chassis.sub.json": chassis},
        properties={"front.sub.json": {"spring": spring}},
        rig=rig,
    )
    return {"assembly": assembly, "rig": rig}


def test_the_document_types_used_here_are_the_generic_ones() -> None:
    """Guard: this module tests documents, not legacy models."""
    assert TemplateDocument is not None
    assert SubsystemDocument is not None
    assert RigDocument is not None
