"""Registered protocols all run ordinary assemblies through one public entry."""
import shutil

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.migration import save_migrated_assembly
from suspension_multibody.kernel.capabilities import kernel_capability_document
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.simulation.backend import NativeContractBackend

from ._documents import PROTOCOLS, documents


def test_every_protocol_is_registered_by_the_loaded_native_kernel():
    assert set(PROTOCOLS) <= set(kernel_capability_document()["case_families"])


@pytest.mark.parametrize("protocol", PROTOCOLS)
def test_every_protocol_runs_the_identical_ordinary_assembly(protocol):
    run = simulate(*documents(protocol))
    assert run.status == "success"
    assert isinstance(run.result, ResultEnvelope)
    assert run.result.body_ids == ("support.carrier", "wheel.wheel")
    assert np.isfinite(run.result.body_state("wheel.wheel")).all()
    assert len(run.result.cases) >= 1


def test_unknown_protocol_is_refused_by_name():
    source, case = documents()
    with pytest.raises(ValueError, match="no_such_family"):
        validate(source, {**case, "family": "no_such_family"})


def test_case_references_missing_physical_entities_are_refused():
    source, case = documents("handling")
    case["handling"]["steering"][0]["actuator"] = "missing"
    with pytest.raises(ValueError, match="steering actuators"):
        validate(source, case)


def test_one_simulate_call_submits_exactly_once(monkeypatch):
    calls = []
    original = NativeContractBackend.run

    def counted(self, compiled):
        calls.append(compiled)
        return original(self, compiled)

    monkeypatch.setattr(NativeContractBackend, "run", counted)
    run = simulate(*documents())
    assert run.status == "success"
    assert calls == [run.compiled]


def test_file_and_memory_submit_the_same_compiled_payloads(tmp_path):
    source, case = documents()
    path = save_migrated_assembly(source, tmp_path / "project")
    one, two = validate(source, case), validate(path, case)
    assert one.model_payload == two.model_payload
    assert one.case_payload == two.case_payload
    for key in ("bodies", "joints", "elements", "tires"):
        assert one.model_document[key] == two.model_document[key]


def test_memory_route_runs_after_the_project_files_are_removed(tmp_path):
    from suspension_multibody.authoring.loader import DocumentLoader

    source, case = documents()
    directory = tmp_path / "project"
    path = save_migrated_assembly(source, directory)
    loaded = DocumentLoader().load(path, case)
    shutil.rmtree(directory)
    run = simulate(loaded.assembly, loaded.case)
    assert run.status == "success"
    assert isinstance(run.result, ResultEnvelope)
    assert run.result.body_state("wheel.wheel").shape == (3, 19)
