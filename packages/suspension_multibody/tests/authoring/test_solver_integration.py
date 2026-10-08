"""File declarations, properties and topology reach the common native solver."""

from __future__ import annotations

import json

import numpy as np
import pytest

from suspension_multibody import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    GenericSubsystemAssembler,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.authoring.bridge import canonical_hardpoint
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.authoring.migration import save_migrated_assembly
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.presets import generic_template

from .fixtures import write_axle_project
from .test_generic_multibody import _assembly, _case, _subsystem, _template


def _project(root):
    assembly = _assembly()
    path = save_migrated_assembly(assembly, root)
    return assembly, path


def _spring(root, *, stiffness=10, curve=None):
    payload = {"document": "element_properties", "schema_version": 1, "name": "spring",
        "element_type": "spring", "model": "piecewise" if curve else "linear",
        "units": {"force": "N", "length": "mm"},
        "parameters": {"stiffness": stiffness, "free_length": 250}}
    if curve:
        payload["curve"] = {"independent": "deflection", "dependent": "force", "points": curve}
    law = ElementPropertyDocument.from_payload(payload)
    law_path = root / "stiff.json"
    law_path.write_text(json.dumps(payload), encoding="utf-8")
    return law, law_path


def test_hardpoint_roles_map_onto_the_model_lookup():
    assert canonical_hardpoint("tie_inner") == "TIE_ROD_INBOARD"
    assert canonical_hardpoint("wheel_center") == "WHEEL_CENTER"
    assert canonical_hardpoint("upper_front") == "UPPER_INBOARD_FRONT"
    assert canonical_hardpoint("some_marker") == "some_marker"


def test_file_template_becomes_the_same_resolved_graph(tmp_path):
    assembly, path = _project(tmp_path)
    reloaded = AssemblyDocument.load(path)
    assert reloaded.entries[0].subsystem.template.topology_hash == assembly.entries[0].subsystem.template.topology_hash
    assert assemble_generic(reloaded).resolved_model().fingerprint == assemble_generic(assembly).resolved_model().fingerprint
    graph = assemble_generic(reloaded).resolved_model().to_document()
    assert len(graph["bodies"]) == 2 and len(graph["joints"]) == 1
    assert [row["type"] for row in graph["elements"]] == ["spring", "damper"]


def test_file_subsystem_supplies_geometry_and_property_values(tmp_path):
    _, path = _project(tmp_path)
    graph = assemble_generic(AssemblyDocument.load(path)).resolved_model().to_document()
    assert graph["bodies"][1]["mass"] == 10
    assert graph["elements"][0]["parameters"]["stiffness"] == 10000
    assert graph["elements"][0]["parameters"]["free_length"] == .25


def test_moving_a_hardpoint_moves_the_model_but_not_the_template(tmp_path):
    assembly, path = _project(tmp_path)
    subsystem = assembly.entries[0].subsystem
    changed = _subsystem(coordinates={"base": [0, 0, 0], "mount": [0, 0, .3]})
    before = assemble_generic(assembly).resolved_model().to_document()
    after = assemble_generic(_assembly({"mechanism": changed})).resolved_model().to_document()
    assert changed.template.topology_hash == subsystem.template.topology_hash
    assert before["elements"][0]["parameters"]["point_b"] != after["elements"][0]["parameters"]["point_b"]
    assert AssemblyDocument.load(path).entries[0].subsystem.payload == subsystem.payload


def test_swapping_the_spring_law_changes_input_and_keeps_topology(tmp_path):
    _, path = _project(tmp_path)
    source = AssemblyDocument.load(path)
    entry = source.entries[0]
    linear = entry.effective()
    _, law_path = _spring(tmp_path, curve=[[0, 0], [10, 100], [20, 400]])
    covered = entry.subsystem.effective(overrides={"property_bindings": {"spring": law_path.name}})
    assert covered.topology_hash == linear.topology_hash
    assert covered.effective_values_hash != linear.effective_values_hash
    assert covered.resolved_property("spring")["force_curve"] == ((0., 0.), (10., 100.), (20., 400.))


def test_file_authored_model_reaches_the_native_solver(tmp_path):
    _, path = _project(tmp_path)
    run = simulate(path, _case())
    assert run.status == "success"
    assert run.result.body_ids == ("mechanism.support", "mechanism.carriage")
    np.testing.assert_allclose(run.result.body_state("mechanism.carriage")[:, 2], .25-10*9.80665/10000, atol=1e-9, rtol=0)
    for index in range(len(run.raw.cases)):
        constraint, dynamics, _ = run.raw.case_residuals(index)
        assert abs(constraint) < 1e-6
        assert abs(dynamics) < 1e-6


def test_overriding_a_property_binding_changes_the_solved_law(tmp_path):
    _, path = _project(tmp_path)
    original = AssemblyDocument.load(path)
    _spring(tmp_path, stiffness=120)
    payload = original.to_payload()
    payload["subsystems"][0]["overrides"] = {"property_bindings": {"spring": "stiff.json"}}
    overridden = AssemblyDocument.from_payload(payload,
        subsystems={entry.ref: entry.subsystem for entry in original.entries})
    compiled = validate(overridden, _case())
    assert compiled.model_document["elements"][0]["parameters"]["stiffness"] == 120000
    assert original.entries[0].effective().resolved_property("spring")["stiffness"] == 10


def test_assembly_carries_every_consumed_file_hash_as_provenance(tmp_path):
    _, path = _project(tmp_path)
    case = tmp_path / "case.json"
    case.write_text(json.dumps(_case()), encoding="utf-8")
    loaded = DocumentLoader().load(path, case)
    assert {row.kind for row in loaded.manifest} == {"assembly", "subsystem", "template", "property", "case"}
    assert all(len(row.file_sha256) == 64 for row in loaded.manifest)
    compiled = validate(path, case)
    assert compiled.request.model.to_document()["resource_manifest"]


def test_incompatible_subsystem_is_refused_by_the_assembly(tmp_path):
    paths = write_axle_project(tmp_path)
    payload = json.loads(paths["assembly"].read_text())
    payload["subsystems"][0]["functional_role"] = "chassis"
    paths["assembly"].write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(AuthoringError, match="functional_role"):
        AssemblyDocument.load(paths["assembly"])


def test_property_file_hash_changes_when_the_law_changes(tmp_path):
    first, path = _spring(tmp_path)
    _spring(tmp_path, stiffness=99)
    assert ElementPropertyDocument.load(path).content_hash != first.content_hash


def test_explicit_missing_port_is_refused():
    with pytest.raises(AuthoringError, match="unknown port"):
        assemble_generic(_assembly(connections=[{"name": "missing", "port_a": "mechanism.absent",
            "port_b": "mechanism.input", "type": "fixed"}]))


@pytest.mark.parametrize("mode", ["K", "C"])
def test_exported_generic_template_is_equivalent_in_both_modes(tmp_path, mode):
    payload = _template()
    payload["joints"][0]["modes"] = ["K"]
    payload["elements"][0]["modes"] = ["C"]
    source = _subsystem(payload)
    path = source.template.save(tmp_path / "slider.json")
    loaded = SubsystemDocument.from_payload(source.to_payload(), template=TemplateDocument.load(path), properties=source.properties)
    one, two = (GenericSubsystemAssembler().assemble(item, mode=mode).resolved_model() for item in (source, loaded))
    assert one.fingerprint == two.fingerprint
    graph = one.to_document()
    assert len(graph["joints"]) == (1 if mode == "K" else 0)
    assert len(graph["elements"]) == (1 if mode == "K" else 2)


def test_the_property_curve_is_the_law_the_kernel_applies(tmp_path):
    template = _template()
    template["coordinates"] = [{"name": "height", "joint": "guide", "kind": "translation"}]
    linear = _subsystem(template)
    curve, _ = _spring(tmp_path, curve=[[-400, -2000], [0, 0], [400, 2000]])
    curved = SubsystemDocument.from_payload(linear.to_payload(), template=linear.template,
        properties={**linear.properties, "spring": curve})
    case = {"schema_version": 1, "name": "hold", "study": "dynamic", "samples": [0, .001],
        "solver": {"initialization_mode": "provided_consistent_state"},
        "boundaries": [{"name": "hold", "coordinate": "mechanism.height", "mode": "locked", "value": 0, "units": "m"}],
        "inputs": [], "outputs": []}
    runs = [simulate(_assembly({"mechanism": item}), case) for item in (linear, curved)]
    force = [np.max(np.abs(run.raw.block("element_wrench")[:, 0, :3])) for run in runs]
    assert force[0] > 0
    assert force[1] == pytest.approx(force[0] / 2, rel=1e-6)


def test_an_assembly_decides_which_subsystems_exist():
    single = assemble_generic(_assembly()).resolved_model().to_document()
    double = assemble_generic(_assembly({"one": _subsystem(), "two": _subsystem()})).resolved_model().to_document()
    assert len(single["bodies"]) == 2
    assert len(double["bodies"]) == 4
    assert all(row["name"].startswith(("one.", "two.")) for row in double["bodies"])


@pytest.mark.parametrize("role", ["suspension", "steering", "body", "wheel", "brake", "drive", "anti_roll_bar"])
def test_every_defined_template_exports_and_reads_back(tmp_path, role):
    source = generic_template(role)
    loaded = TemplateDocument.load(source.save(tmp_path / (role+".json")))
    assert loaded.topology_hash == source.topology_hash
    assert loaded.payload == source.payload
