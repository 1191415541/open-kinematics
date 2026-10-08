"""Two ordinary body subsystems and an explicit articulation reach one solve."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)


def _tow(*joints):
    templates = {}
    for name, position, mass, fixed in (("ground", [0, 0, 0], 0, True),
        ("tractor", [0, 0, 0], 1200, False), ("trailer", [3, 0, 0], 800, False)):
        templates[name] = {"document": "template", "schema_version": 1, "name": name,
            "functional_role": "generic", "allowed_placement_roles": ["any"], "symmetry": "asymmetric",
            "units": {"length": "m"}, "property_slots": [], "elements": [],
            "bodies": [{"name": name, "position": position, "mass": mass, "inertia": (np.eye(3)*max(mass, 1)).tolist(), "fixed": fixed}],
            "hardpoints": [{"name": "origin", "owner": name, "space": "body"}], "joints": [],
            "needs": [], "ports": [{"name": "mount", "role": name, "owner": name, "point": "origin", "cardinality": "many"}]}
    coordinates = {name: {"origin": [0, 0, 0]} for name in templates}
    for row in joints:
        owner, reaction = row["body_b"], row["body_a"]
        template = templates[owner]
        template["needs"].append({"name": row["name"], "role": reaction, "count": 1, "required": True})
        for end, body in (("a", "@"+row["name"]), ("b", owner)):
            point = row["name"]+"."+end
            template["hardpoints"].append({"name": point, "owner": body, "space": "body"})
            coordinates[owner][point] = list(row.get("point_"+end, [0, 0, 0]))
        template["joints"].append({"name": row["name"], "type": row["type"], "body_a": "@"+row["name"],
            "body_b": owner, "point_a": row["name"]+".a", "point_b": row["name"]+".b",
            **({"axis": [0, 0, 1]} if row["type"] == "revolute" else {})})
    subsystems = {name: SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": name, "functional_role": "generic", "placement_role": "any", "template": name+".tpl.json",
        "hardpoints": coordinates[name], "property_bindings": {}}, template=TemplateDocument.from_payload(template))
        for name, template in templates.items()}
    return AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1, "name": "tow",
        "assembly_kind": "generic_multibody", "mode": "K", "gravity": [0, 0, 0],
        "subsystems": [{"ref": name, "functional_role": "generic", "placement_role": "any"} for name in templates]}, subsystems=subsystems)


def _joints():
    return ({"name": "anchor", "type": "fixed", "body_a": "ground", "body_b": "tractor"},
        {"name": "hitch", "type": "revolute", "body_a": "tractor", "body_b": "trailer", "point_a": [3, 0, .5], "point_b": [0, 0, .5]})


def _case():
    return {"schema_version": 1, "name": "tow", "study": "dynamic", "protocol": "axle_dynamic",
        "samples": [0, .0005, .001], "boundaries": [], "inputs": [], "outputs": [],
        "solver": {"initialization_mode": "provided_consistent_state"}}


def test_two_body_sides_assemble_under_one_articulation():
    source = _tow(*_joints())
    graphs = [assemble_generic(source).resolved_model().to_document() for _ in ("K", "C")]
    assert {row["name"] for row in graphs[0]["bodies"]} == {"ground.ground", "tractor.tractor", "trailer.trailer"}
    assert [row["name"] for row in graphs[0]["joints"]] == ["tractor.anchor", "trailer.hitch"]
    assert graphs[0]["joints"] == graphs[1]["joints"]
    hitch = graphs[0]["joints"][1]
    assert {hitch["body_a"], hitch["body_b"]} == {"tractor.tractor", "trailer.trailer"}


def test_an_articulation_naming_a_body_nobody_produced_is_refused():
    source = _tow({"name": "hitch", "type": "fixed", "body_a": "missing", "body_b": "trailer"})
    with pytest.raises(ValueError, match="missing|candidate"):
        validate(source, _case())


def test_an_unknown_joint_kind_is_refused_rather_than_invented():
    with pytest.raises(ValueError):
        _tow({"name": "hitch", "type": "ball", "body_a": "tractor", "body_b": "trailer"})


def test_a_joint_between_a_body_and_itself_is_refused():
    source = _tow({"name": "self", "type": "fixed", "body_a": "tractor", "body_b": "tractor"})
    with pytest.raises(ValueError, match="candidate|self|body"):
        validate(source, _case())


def test_an_articulation_is_the_only_thing_that_changes_the_bodies():
    assert assemble_generic(_tow()).resolved_model().to_document()["joints"] == []


def test_the_towed_unit_carries_two_body_sides_and_runs():
    source = _tow(*_joints())
    compiled = validate(source, _case())
    assert next(row for row in compiled.model_document["joints"] if row["name"] == "trailer.hitch")["type"] == "revolute"
    result = simulate(source, _case()).result
    tractor, trailer = result.body_state("tractor.tractor"), result.body_state("trailer.trailer")
    assert tractor.shape == trailer.shape == (3, 19)
    assert np.all(np.isfinite(trailer))
    np.testing.assert_allclose(tractor[:, :3], 0, atol=1e-9)
    np.testing.assert_allclose(trailer[:, :3], [[3, 0, 0]]*3, atol=1e-9)
