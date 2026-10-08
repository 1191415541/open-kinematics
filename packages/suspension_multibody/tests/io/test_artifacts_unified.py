"""The artifact writer persists every native channel through one envelope."""

import json
import zipfile
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.io import read_artifact, write_artifact
from suspension_multibody.results import ResultEnvelope

from ..authoring.test_generic_multibody import _assembly, _case


def _result(protocol="axle_dynamic"):
    return simulate(_assembly(), {**_case(), "family": protocol}).result


@pytest.mark.parametrize("protocol", ["axle_dynamic", "vehicle_dynamic"])
def test_native_artifacts_have_the_identical_layout_and_all_channels(tmp_path, protocol):
    result = _result(protocol)
    loaded = read_artifact(write_artifact(result, tmp_path, metrics={"peak_force_n": 12}))
    assert loaded["manifest"]["artifact_type"] == "multibody_result"
    assert loaded["manifest"]["status"] == "success"
    assert loaded["metrics"]["peak_force_n"] == 12
    assert loaded["arrays"]["body_state"].shape == (3, 2, 19)
    assert loaded["manifest"]["model_fingerprint"] == result.model_fingerprint
    for name, channel in result.named_blocks.items():
        np.testing.assert_array_equal(loaded["arrays"][name], channel)
    with zipfile.ZipFile(tmp_path / "arrays.npz") as archive:
        assert archive.namelist() == ["times_s.npy", *[name + ".npy" for name in result.named_blocks]]


def test_times_diagnostics_performance_and_provenance_survive_roundtrip(tmp_path):
    result = _result()
    loaded = read_artifact(write_artifact(result, tmp_path))
    assert loaded["arrays"]["times_s"].tolist() == result.times_s.tolist()
    np.testing.assert_array_equal(loaded["diagnostics"], result.diagnostics)
    assert loaded["performance"] == dict(result.performance)
    assert loaded["manifest"]["model"]["name"] == result.model.name


def test_failed_partial_artifact_preserves_channels_and_evidence(tmp_path):
    result = _result()
    partial = ResultEnvelope(replace(result.raw, document={**result.raw.document, "status": "partial"}), result.model)
    failure = RuntimeError("solver stopped at sample 2")
    loaded = read_artifact(write_artifact(None, tmp_path, partial=partial, status="failed", failure=failure))
    assert loaded["manifest"]["artifact_type"] == "multibody_result"
    assert loaded["manifest"]["status"] == "failed"
    assert loaded["failure_evidence"] == {"type": "RuntimeError", "message": "solver stopped at sample 2"}
    assert loaded["partial_evidence"]["completed_sample_count"] == len(result.times_s)
    np.testing.assert_array_equal(loaded["arrays"]["body_state"], result.named_blocks["body_state"])


def test_failure_without_channels_does_not_fabricate_samples(tmp_path):
    loaded = read_artifact(write_artifact(None, tmp_path, failure=RuntimeError("native solver failed")))
    assert loaded["manifest"]["status"] == "failed"
    assert loaded["arrays"] == {}
    assert loaded["failure_evidence"]["message"] == "native solver failed"


def test_empty_optional_fields_remain_absent(tmp_path):
    result = _result()
    loaded = read_artifact(write_artifact(result, tmp_path))
    assert loaded["metrics"] == {}
    assert loaded["failure_evidence"] == {}
    assert "tire_output" not in loaded["arrays"] or loaded["arrays"]["tire_output"].shape[1] == 0


def test_historical_tables_are_still_read_only(tmp_path: Path):
    (tmp_path / "states.csv").write_text("state_id,converged\ns0,True\n", encoding="utf-8")
    (tmp_path / "manifest.json").write_text(json.dumps({"artifact_type": "result_bundle", "tables": ["states"]}), encoding="utf-8")
    assert read_artifact(tmp_path)["tables"]["states"] == [{"state_id": "s0", "converged": "True"}]
    with pytest.raises(TypeError, match="ResultEnvelope"):
        write_artifact(object(), tmp_path / "invalid")
