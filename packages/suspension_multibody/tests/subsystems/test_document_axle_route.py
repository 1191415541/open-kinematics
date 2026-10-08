"""Saved declarations resolve and run through exactly the object input route."""

import shutil

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import AssemblyDocument, migrate_v1_kc_case
from suspension_multibody.authoring.migration import save_migrated_assembly
from tests.benchmark_fixture import benchmark_model


@pytest.mark.parametrize("mode", ["K", "C"])
def test_file_and_object_declarations_have_identical_entities_and_submission(tmp_path, mode):
    documents = migrate_v1_kc_case(benchmark_model(), mode=mode, wheel_values_mm=(0,), rack_values_mm=(0,), paths=("fz",), levels=3)
    path = save_migrated_assembly(documents[0], tmp_path / "project")
    compiled = [validate(source, documents[1]) for source in (path, AssemblyDocument.load(path), documents[0])]
    assert all(row.model_document == compiled[0].model_document and row.model_payload == compiled[0].model_payload
               and row.case_document == compiled[0].case_document and row.case_payload == compiled[0].case_payload for row in compiled)
    run = simulate(path, documents[1])
    assert run.status == "success"
    assert np.isfinite(run.raw.states).all()


def test_loaded_documents_run_after_the_project_is_removed(tmp_path):
    source, case = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(0,), rack_values_mm=(0,))
    directory = tmp_path / "project"
    loaded = AssemblyDocument.load(save_migrated_assembly(source, directory))
    expected = validate(loaded, case)
    shutil.rmtree(directory)
    actual = simulate(loaded, case)
    assert actual.compiled.model_document == expected.model_document
    assert actual.compiled.model_payload == expected.model_payload
    assert actual.compiled.case_document == expected.case_document
    assert actual.compiled.case_payload == expected.case_payload
    assert actual.status == "success"
