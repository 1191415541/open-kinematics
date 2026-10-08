"""Physical identity does not depend on business roles or mutable input aliases."""

import copy

import pytest
from suspension_contracts import ContractError

from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan


def _document():
    return {
        "schema_version": 1, "name": "assembly", "units": "SI",
        "bodies": [
            {"name": "component/body", "mass": 2., "inertia": [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], "position": [1., 2., 3.], "quaternion": [1., 0., 0., 0.]},
        ],
        "frames": [{"name": "component/mount", "body": "component/body", "point": [0.1, 0., 0.], "quaternion": [1., 0., 0., 0.]}],
        "joints": [], "elements": [],
        "provenance": {"component/body": {"template": "mechanism", "source": "first.json"}},
    }


def test_model_owns_immutable_data():
    document = _document()
    model = ResolvedModel(document)
    document["bodies"][0]["mass"] = 100.
    detached = model.to_document()
    detached["frames"].clear()
    assert model.to_document()["bodies"][0]["mass"] == 2.
    assert len(model.to_document()["frames"]) == 1


def test_placement_changes_global_but_not_component_identity():
    first = ResolvedModel(_document())
    moved = _document()
    moved["bodies"][0]["position"] = [8., 4., -3.]
    moved["bodies"][0]["quaternion"] = [0., 1., 0., 0.]
    second = ResolvedModel(moved)
    assert first.fingerprint != second.fingerprint
    assert first.subgraph_fingerprint(("component/body",)) == second.subgraph_fingerprint(("component/body",))


def test_source_location_is_not_physical_identity():
    first = ResolvedModel(_document())
    changed = _document()
    changed["provenance"]["component/body"]["source"] = "other.json"
    second = ResolvedModel(changed)
    assert first.fingerprint == second.fingerprint
    assert first.provenance_fingerprint != second.provenance_fingerprint


def test_inertia_change_affects_component_fingerprint():
    first = ResolvedModel(_document())
    changed = _document()
    changed["bodies"][0]["inertia"][0][0] = 1.5
    assert first.subgraph_fingerprint(("component/body",)) != ResolvedModel(changed).subgraph_fingerprint(("component/body",))


def test_entity_order_does_not_change_physical_identity():
    document = _document()
    second = copy.deepcopy(document["bodies"][0])
    second["name"] = "other/body"
    document["bodies"].append(second)
    first = ResolvedModel(document)
    document["bodies"].reverse()
    assert first.fingerprint == ResolvedModel(document).fingerprint


def test_run_plan_has_no_model_or_role():
    document = {"schema_version": 1, "name": "run", "study": "dynamic", "samples": [0., 1.], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}
    plan = ResolvedSolvePlan(document)
    assert plan.to_document() == document
    document["dynamic_model"] = _document()
    with pytest.raises(ContractError):
        ResolvedSolvePlan(document)


def test_unknown_body_in_component_fingerprint_is_rejected():
    with pytest.raises(ValueError, match="unknown body"):
        ResolvedModel(_document()).subgraph_fingerprint(("missing",))


def test_generic_file_and_memory_build_the_same_si_graph():
    from pathlib import Path

    from suspension_multibody.authoring import AssemblyDocument, assemble_generic

    path = Path(__file__).resolve().parents[2] / "examples/generic_multibody/assembly.json"
    loaded = AssemblyDocument.load(path)
    first = assemble_generic(loaded).resolved_model()
    memory = AssemblyDocument.from_payload(loaded.to_payload(), subsystems={entry.ref: entry.subsystem for entry in loaded.entries}, rig=loaded.rig)
    second = assemble_generic(memory).resolved_model()
    assert isinstance(first, ResolvedModel)
    assert isinstance(second, ResolvedModel)
    assert first.fingerprint == second.fingerprint
    assert first.to_document() == second.to_document()
    assert first.to_document()["units"] == "SI"
    assert first.to_document()["elements"]


def test_component_identity_preserves_relative_body_positions():
    document = _document()
    other = copy.deepcopy(document["bodies"][0])
    other["name"] = "component/second"
    other["position"] = [2., 2., 3.]
    document["bodies"].append(other)
    first = ResolvedModel(document)
    document["bodies"][1]["position"] = [3., 2., 3.]
    second = ResolvedModel(document)
    ids = ("component/body", "component/second")
    assert first.subgraph_fingerprint(ids) != second.subgraph_fingerprint(ids)
