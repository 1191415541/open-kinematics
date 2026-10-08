"""Path and memory sources produce the same pinned document bundle."""

import json
import shutil
from pathlib import Path

import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    assemble_generic,
)
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.loader import CaseDocument, DocumentLoader

EXAMPLES = Path(__file__).resolve().parents[2] / "examples/generic_multibody"


def test_file_and_memory_resolve_identical_model_and_case():
    loader = DocumentLoader(resource_root=EXAMPLES)
    file = loader.load("assembly.json", "dynamic.case.json")
    memory = loader.load(file.assembly, file.case)
    first = assemble_generic(file.assembly).resolved_model()
    second = assemble_generic(memory.assembly).resolved_model()
    assert first.fingerprint == second.fingerprint
    assert file.resolve().fingerprint == memory.resolve().fingerprint
    assert file.case.to_payload() == memory.case.to_payload()
    assert file.resource_fingerprint == memory.resource_fingerprint
    assert validate(file.assembly, file.case.to_payload()).model_payload == validate(memory.assembly, memory.case.to_payload()).model_payload
    assert all(entry.subsystem.path is None for entry in file.assembly.entries)


def test_fully_bound_bundle_runs_without_original_directory(tmp_path):
    from suspension_multibody.simulation.runner import run_compiled

    project = tmp_path / "project"
    shutil.copytree(EXAMPLES, project)
    bundle = DocumentLoader(resource_root=project).load("tire.assembly.json", "dynamic.case.json")
    before = validate(bundle.assembly, bundle.case.to_payload())
    shutil.rmtree(project)
    after = validate(bundle.assembly, bundle.case.to_payload())
    assert before.model_payload == after.model_payload
    assert before.case_payload == after.case_payload
    assert run_compiled(after).status == "success"


def test_manifest_only_contains_consumed_resources():
    bundle = DocumentLoader(resource_root=EXAMPLES).load("assembly.json", "dynamic.case.json")
    assert {row.kind for row in bundle.manifest} == {"assembly", "subsystem", "template", "property", "case"}
    assert all(row.file_sha256 for row in bundle.manifest)
    assert not any("rotor" in row.source for row in bundle.manifest)


def test_tir_and_inertia_properties_are_pinned_in_manifest():
    loader = DocumentLoader(resource_root=EXAMPLES)
    file = loader.load("tire.assembly.json", "dynamic.case.json")
    memory = loader.load(file.assembly, file.case)
    assert file.resource_fingerprint == memory.resource_fingerprint
    assert any(row.source.endswith("vertical.tir") and row.file_sha256 for row in file.manifest)
    assert {row.reference.rsplit(":", 1)[-1] for row in file.manifest if row.kind == "property"} == {"mass", "inertia", "tire"}
    assert file.resolve().fingerprint == memory.resolve().fingerprint


def test_invalid_case_has_the_same_error_for_file_and_memory(tmp_path):
    payload = {"schema_version": 1, "name": "broken", "study": "unknown"}
    path = tmp_path / "case.json"
    path.write_text(json.dumps(payload), encoding="utf-8")
    assembly = AssemblyDocument.load(EXAMPLES / "assembly.json")
    errors = []
    for case in (payload, path):
        with pytest.raises(AuthoringError) as error:
            DocumentLoader().load(assembly, case)
        errors.append(str(error.value))
    assert errors[0] == errors[1]


def test_case_object_has_no_mutable_alias():
    payload = {"schema_version": 1, "name": "run", "study": "dynamic", "samples": [0., 1.], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}
    case = CaseDocument(payload)
    payload["samples"].clear()
    assert case.to_payload()["samples"] == [0., 1.]
