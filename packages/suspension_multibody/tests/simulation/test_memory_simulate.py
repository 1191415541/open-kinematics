"""Pinned memory documents run identically without their source directory."""

import shutil

import numpy as np

from suspension_multibody import simulate
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.authoring.migration import save_migrated_assembly

from ..authoring.test_generic_multibody import _assembly, _case


def test_an_in_memory_assembly_runs_through_simulate():
    run = simulate(_assembly(), _case())
    assert run.status == "success"
    assert run.compiled.assembly == "generic"


def test_the_memory_assembly_is_the_same_run_as_the_file(tmp_path):
    source = _assembly()
    path = save_migrated_assembly(source, tmp_path)
    first, second = [simulate(item, _case()) for item in (source, path)]
    assert first.compiled.model_payload == second.compiled.model_payload
    assert first.compiled.case_payload == second.compiled.case_payload
    np.testing.assert_array_equal(first.raw.states, second.raw.states)


def test_the_memory_assembly_runs_with_the_project_files_gone(tmp_path):
    directory = tmp_path / "project"
    path = save_migrated_assembly(_assembly(), directory)
    bundle = DocumentLoader().load(path, _case())
    expected = simulate(bundle.assembly, bundle.case)
    shutil.rmtree(directory)
    actual = simulate(bundle.assembly, bundle.case)
    np.testing.assert_array_equal(actual.raw.states, expected.raw.states)


def test_a_memory_assembly_is_not_rebound_from_a_path(monkeypatch):
    from suspension_multibody.authoring import AssemblyDocument

    source = _assembly()
    assert source.path is None
    def fail(*args, **kwargs):
        raise AssertionError("memory submission tried to load an assembly path")
    monkeypatch.setattr(AssemblyDocument, "load", fail)
    assert simulate(source, _case()).status == "success"
