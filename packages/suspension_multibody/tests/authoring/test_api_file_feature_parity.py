"""File and Python documents expose identical modeling and execution features."""

import json
import shutil

import numpy as np
import pytest

from suspension_multibody import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    TemplateDocument,
    assemble_generic,
    model_identity,
)
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.authoring.migration import (
    migrate_v1_kc_case,
    save_migrated_assembly,
)
from suspension_multibody.presets import generic_template
from tests.benchmark_fixture import benchmark_model

from .test_generic_multibody import _assembly, _case, _subsystem


def test_simulate_accepts_a_path_a_document_and_a_memory_assembly(tmp_path):
    source = _assembly()
    path = save_migrated_assembly(source, tmp_path)
    runs = [simulate(item, _case()) for item in (path, AssemblyDocument.load(path), source)]
    for run in runs:
        assert run.status == "success"
        assert run.compiled.model_payload == runs[0].compiled.model_payload
        assert run.compiled.case_payload == runs[0].compiled.case_payload
        np.testing.assert_array_equal(run.raw.states, runs[0].raw.states)


def test_simulate_refuses_a_bare_contract_model_document():
    with pytest.raises(AuthoringError, match="AssemblyDocument"):
        simulate({"contract": "multibody-model"}, _case())


def test_the_memory_route_reads_the_project_without_the_directory(tmp_path):
    project = tmp_path / "project"
    path = save_migrated_assembly(_assembly(), project)
    bundle = DocumentLoader().load(path, _case())
    before = validate(bundle.assembly, bundle.case)
    shutil.rmtree(project)
    after = simulate(bundle.assembly, bundle.case)
    assert after.status == "success"
    assert before.model_payload == after.compiled.model_payload


def test_the_reading_comes_from_the_case_document_not_a_parameter():
    source, case = migrate_v1_kc_case(benchmark_model(), mode="K")
    compiled = validate(source, case)
    both = compiled.case_document
    both["c"] = {"paths": ["fz"], "maximum": 1, "levels": 3}
    with pytest.raises(ValueError):
        validate(source, both)


def test_a_case_with_neither_reading_is_refused():
    source, case = migrate_v1_kc_case(benchmark_model(), mode="K")
    empty = validate(source, case).case_document
    empty.pop("k")
    with pytest.raises(ValueError):
        validate(source, empty)


def test_a_preset_subsystem_describes_the_same_graph_as_its_file(tmp_path):
    template = generic_template("suspension")
    loaded = TemplateDocument.load(template.save(tmp_path / "template.json"))
    assert loaded.topology_hash == template.topology_hash
    assert loaded.payload == template.payload


def test_a_preset_returns_a_document_a_caller_can_edit_and_save(tmp_path):
    template = generic_template("suspension")
    payload = template.to_payload()
    payload["name"] = "renamed"
    loaded = TemplateDocument.load(TemplateDocument.from_payload(payload).save(tmp_path / "edited.json"))
    assert loaded.name == "renamed"
    assert loaded.payload["bodies"] == template.payload["bodies"]
    assert template.name != loaded.name


def test_updating_a_declaration_moves_one_field_and_nothing_else():
    source = _assembly()
    moved = _assembly({"mechanism": _subsystem(coordinates={"base": [0, 0, 0], "mount": [0, 0, .3]})})
    before, after = [assemble_generic(item).resolved_model().to_document() for item in (source, moved)]
    assert before["bodies"] == after["bodies"]
    assert before["elements"][0]["parameters"]["stiffness"] == after["elements"][0]["parameters"]["stiffness"]
    assert before["elements"][0]["parameters"]["point_b"] != after["elements"][0]["parameters"]["point_b"]
    assert source.entries[0].subsystem.template.topology_hash == moved.entries[0].subsystem.template.topology_hash


def test_the_declaration_carries_the_mirrored_bodies():
    from suspension_multibody.authoring.migration import migrate_v1_axle
    graph = assemble_generic(migrate_v1_axle(benchmark_model())).resolved_model().to_document()
    names = {row["name"] for row in graph["bodies"]}
    for side in ("L", "R"):
        assert any(name.endswith("upper_arm_"+side) for name in names)


def test_the_wheel_role_owns_the_wheel_and_tire():
    template = generic_template("wheel")
    assert template.payload["bodies"][0]["name"] == "wheel"
    assert template.payload["tires"][0]["body"]["name"] == "wheel"
    assert template.payload["joints"][0]["body_b"] == "wheel"
    assert not any("wheel" in row["name"] for row in generic_template("suspension").payload["bodies"])


def test_the_two_routes_agree_on_identity(tmp_path):
    source = _assembly()
    loaded = AssemblyDocument.load(save_migrated_assembly(source, tmp_path))
    assert model_identity(source) != model_identity(loaded)
    assert assemble_generic(source).resolved_model().fingerprint == assemble_generic(loaded).resolved_model().fingerprint


def test_a_runtime_value_is_not_a_document_for_identity():
    with pytest.raises(TypeError, match="model_identity takes"):
        model_identity(object())


def test_a_historical_result_bundle_still_reads(tmp_path):
    from suspension_multibody.schema import load_dynamic_result
    payload = {"manifest": {"schema_version": 1, "format_version": "1.0", "run_id": "historical",
        "mode": "axle_dynamic", "sample_count": 2, "provenance": {"package_version": "0.1.0"}},
        "samples": [{"time": 0., "body": "axle", "metrics": {"heave_mm": 0.}},
                    {"time": .001, "body": "axle", "metrics": {"heave_mm": 1.}}], "diagnostics": []}
    path = tmp_path / "bundle.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    loaded = load_dynamic_result(path)
    assert loaded.manifest.mode == "axle_dynamic"
    assert [row.time for row in loaded.samples] == [0., .001]
