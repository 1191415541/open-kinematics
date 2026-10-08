"""Vehicle declarations use the same compiler, submission and result as every model."""
import importlib.util
from dataclasses import replace

import numpy as np
import pytest

from suspension_multibody import api
from suspension_multibody.authoring.loader import DocumentLoader, LoadedDocuments
from suspension_multibody.authoring.migration import migrate_v1_vehicle_case
from suspension_multibody.io import read_artifact, write_artifact
from suspension_multibody.kernel import KernelContractError
from suspension_multibody.report.metrics.case_specific import compute_case_metrics
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.simulation import NativeContractBackend, run_compiled, runner

from .vehicle_fixtures import _case, _positioned_vehicle, _vehicle


@pytest.fixture(scope="module")
def declarations():
    model = _positioned_vehicle(_vehicle())
    return migrate_v1_vehicle_case(_case(model))


@pytest.fixture(scope="module")
def submitted(declarations):
    return api.validate(*declarations)


@pytest.fixture(scope="module")
def complete(submitted):
    return NativeContractBackend().run(submitted)


@pytest.mark.parametrize("module", [
    "vehicle.service", "vehicle_dynamics", "results.vehicle", "results.axle",
    "results.decoder", "axle_dynamics.contract_run",
])
def test_business_execution_and_result_adapters_are_retired(module):
    assert importlib.util.find_spec("suspension_multibody." + module) is None


def test_vehicle_has_one_loaded_model_and_no_prepared_context(declarations):
    source, case = declarations
    bundle = DocumentLoader().load(source, case)
    graph = bundle.resolve()
    compiled = api.validate(source, case)
    assert compiled.request.context == {}
    assert compiled.request.model.to_document() == graph.to_document()
    assert compiled.metadata["model_fingerprint"] == graph.fingerprint
    assert compiled.metadata["compiler"] == "ResolvedModelCompiler"
    assert len(compiled.model_document["tires"]) == 4


def test_vehicle_facade_resolves_and_submits_once(declarations, monkeypatch, complete):
    counts = {"resolve": 0, "submit": 0}
    original = LoadedDocuments.resolve

    def resolve(self):
        counts["resolve"] += 1
        return original(self)

    class Backend:
        def run(self, compiled):
            counts["submit"] += 1
            return complete

    monkeypatch.setattr(LoadedDocuments, "resolve", resolve)
    monkeypatch.setattr(runner, "NativeContractBackend", Backend)
    result = api.simulate(*declarations)
    assert counts == {"resolve": 1, "submit": 1}
    assert isinstance(result.result, ResultEnvelope)
    assert result.result.raw is result.raw


def test_vehicle_result_uses_stable_entity_channels(submitted, complete):
    class Backend:
        def run(self, compiled):
            return complete

    result = run_compiled(submitted, backend=Backend()).result
    assert result.status == "success"
    assert result.times_s.size >= 2
    assert len(result.tire_ids) == 4
    assert "body.chassis" in result.body_ids
    assert result.body_state("body.chassis").shape == (len(result.times_s), 19)
    assert np.isfinite(result.body_state("body.chassis")).all()
    for entity in result.tire_ids:
        assert result.tire_state(entity).shape[0] == len(result.times_s)
    assert result.diagnostics is not None
    assert result.performance


def test_vehicle_metrics_and_artifact_consume_the_same_envelope(submitted, complete, tmp_path):
    class Backend:
        def run(self, compiled):
            return complete

    run = run_compiled(submitted, backend=Backend())
    assert compute_case_metrics("vehicle_dynamic", run.result)
    loaded = read_artifact(write_artifact(run.result, tmp_path / "vehicle"))
    assert loaded["manifest"]["artifact_type"] == "multibody_result"
    assert loaded["manifest"]["status"] == "success"
    for name, values in run.result.named_blocks.items():
        np.testing.assert_array_equal(loaded["arrays"][name], values)


def test_vehicle_native_failure_preserves_original_partial_evidence(submitted, complete):
    document = dict(complete.document)
    document["status"] = "partial"
    document["manifest"] = {**document.get("manifest", {}),
        "failed_sample_index": 1, "failed_status": 3, "failed_time_s": .001}
    partial = replace(complete, document=document)
    error = KernelContractError("solver stopped", partial_run=partial)

    class Backend:
        def run(self, compiled):
            raise error

    with pytest.raises(KernelContractError) as caught:
        run_compiled(submitted, backend=Backend())
    assert caught.value is error
    assert error.partial_raw_result.status == "partial"
    assert error.partial_raw_result.failure_evidence["failed_sample_index"] == 1
    assert error.partial_raw_result.failure_evidence["failed_status"] == 3
    assert error.partial_raw_result.failure_evidence["failed_time_s"] == pytest.approx(.001)
    assert error.partial_raw_result.block("body_state").shape == complete.blocks["body_state"].shape
