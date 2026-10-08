"""Matched endpoints become physical rows through the common interpreter."""

import numpy as np
import pytest

from suspension_multibody.authoring import TemplateDocument, assemble_generic
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.connections.matcher import AmbiguousBindingError, BindingError
from tests.authoring.test_unified_subsystem_templates import assembly, subsystem


def declared_links(*, kind="fixed", ports=("mount",), pairings=None, recipe=True):
    common = {"document": "template", "schema_version": 1, "functional_role": "generic",
        "allowed_placement_roles": ["any"], "symmetry": "asymmetric", "units": {"length": "m"},
        "joints": [], "elements": [], "property_slots": []}
    provider = subsystem(TemplateDocument.from_payload({**common, "name": "provider",
        "bodies": [{"name": "subframe", "fixed": True}],
        "hardpoints": [{"name": "mount", "owner": "subframe"}],
        "ports": [{"name": name, "role": "mount", "owner": "subframe", "point": "mount"} for name in ports]}),
        {"mount": [.1, -.7, .3]})
    row = {"name": "attachment", "type": kind, "body_a": "member", "body_b": "@mount",
        "point_a": "mount", "point_b": "@mount"}
    if kind == "revolute":
        row["axis"] = [0, 1, 0]
    if kind == "bushing":
        row.update(property_slot="law", parameters={"stiffness": np.eye(6).tolist()})
    consumer = subsystem(TemplateDocument.from_payload({**common, "name": "consumer",
        "bodies": [{"name": "member", "mass": 1}], "hardpoints": [{"name": "mount", "owner": "member"}],
        "needs": [{"name": "mount", "role": "mount", "count": 1, "required": True}],
        "joints": [row] if recipe and kind != "bushing" else [],
        "elements": [row] if recipe and kind == "bushing" else [],
        "property_slots": [{"name": "law", "element_type": "generic", "required": False, "default": 0}] if kind == "bushing" else [],
        "ports": []}), {"mount": [.1, -.7, .3]})
    documents = {}
    for name, document in (("provider", provider), ("consumer", consumer)):
        payload = document.to_payload()
        payload["template"] = name + ".tpl.json"
        documents[name + ".sub.json"] = type(document).from_payload(payload, template=document.template, properties=document.properties)
    translated = {name + ".sub.json": {role: port.replace("provider.", "provider.sub.json.") for role, port in pairs.items()}
                  for name, pairs in (pairings or {}).items()}
    return assembly(documents, pairings=translated)


def test_binding_without_a_physical_recipe_creates_no_constraint():
    graph = assemble_generic(declared_links(recipe=False))
    assert graph.joints == () and graph.elements == ()
    assert len(graph.bodies) == 2


@pytest.mark.parametrize("kind", ["fixed", "revolute", "bushing"])
def test_physical_rows_have_declared_endpoints_and_geometry(kind):
    built = assemble_generic(declared_links(kind=kind))
    row = (built.elements if kind == "bushing" else built.joints)[0]
    assert row["name"] == "consumer.sub.json.attachment"
    assert row["type"] == kind
    assert (row["body_a"], row["body_b"]) == ("consumer.sub.json.member", "provider.sub.json.subframe")
    points = row.get("parameters", row)
    np.testing.assert_array_equal(points["point_a"], [.1, -.7, .3])
    np.testing.assert_array_equal(points["point_b"], [.1, -.7, .3])
    if kind == "revolute":
        np.testing.assert_array_equal(row["axis_a"], [0, 1, 0])
        np.testing.assert_array_equal(row["axis_b"], [0, 1, 0])
    if kind == "bushing":
        assert not built.joints
        np.testing.assert_array_equal(row["parameters"]["stiffness"], np.eye(6))


def test_explicit_pairing_and_unique_inference_resolve_the_same_graph():
    inferred = assemble_generic(declared_links()).resolved_model()
    explicit = assemble_generic(declared_links(pairings={"consumer": {"mount": "provider.mount"}})).resolved_model()
    assert inferred.fingerprint == explicit.fingerprint
    assert inferred.to_document() == explicit.to_document()


def test_ambiguity_names_candidates_and_explicit_pairing_resolves_it():
    with pytest.raises(AmbiguousBindingError) as failure:
        assemble_generic(declared_links(ports=("left", "right")))
    assert "left" in str(failure.value) and "right" in str(failure.value)
    graph = assemble_generic(declared_links(ports=("left", "right"), pairings={"consumer": {"mount": "provider.right"}}))
    assert graph.joints[0]["body_b"] == "provider.sub.json.subframe"


def test_missing_and_misspelled_ports_are_named():
    with pytest.raises(BindingError, match="mount"):
        assemble_generic(declared_links(ports=()))
    with pytest.raises((BindingError, AuthoringError), match="missing"):
        assemble_generic(declared_links(pairings={"consumer": {"mount": "provider.missing"}}))
