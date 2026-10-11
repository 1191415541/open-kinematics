"""The artifact writer persists every native channel through one envelope."""

import json
import zipfile
from dataclasses import replace
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.io import read_artifact, write_artifact
from suspension_multibody.io.artifacts import _channel_table
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


def test_requested_tables_are_real_and_keep_output_identity(tmp_path):
    result = _result()
    write_artifact(result, tmp_path, formats=("csv", "parquet"))
    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    assert set(manifest["table_formats"]) == {"csv", "parquet"}
    assert "body_state" in manifest["table_formats"]["csv"]
    assert (tmp_path / "body_state.csv").is_file()
    assert (tmp_path / "body_state.parquet").is_file()
    assert manifest["table_layouts"]["body_state"] == [
        "sample_index", "case_index", "case_name", "output_sample_id",
        "time_s", "sample_role", "index_0", "index_1", "value", "unit",
        "frame_id", "reason",
    ]
    loaded = read_artifact(tmp_path)
    rows = loaded["tables"]["body_state"]
    assert rows[0]["case_name"] == result.raw.cases[0]["name"]
    assert rows[0]["output_sample_id"] == 0
    assert rows[0]["sample_role"] == "output"
    assert float(rows[0]["time_s"]) == pytest.approx(float(result.times_s[0]))
    assert rows[0]["value"] == pytest.approx(float(result.named_blocks["body_state"][0, 0, 0]))
    assert rows[0]["unit"] == "m"
    assert rows[0]["frame_id"] == "world"
    assert rows[0]["reason"] == ""


@pytest.mark.parametrize("format_name", ["csv", "parquet"])
def test_table_format_selection_writes_only_requested_format(tmp_path, format_name):
    result = _result()
    write_artifact(result, tmp_path, formats=(format_name,))
    assert (tmp_path / f"body_state.{format_name}").is_file()
    other = "parquet" if format_name == "csv" else "csv"
    assert not (tmp_path / f"body_state.{other}").exists()
    if format_name == "parquet":
        table = pq.read_table(tmp_path / "body_state.parquet")
        values = table.column("value").to_pylist()
    else:
        import csv

        with (tmp_path / "body_state.csv").open(newline="", encoding="utf-8") as stream:
            values = [float(row["value"]) for row in csv.DictReader(stream)]
    np.testing.assert_array_equal(
        np.asarray(values), result.named_blocks["body_state"].reshape(-1)
    )


def test_unknown_table_format_fails_before_creating_artifacts(tmp_path):
    with pytest.raises(ValueError, match="unsupported artifact table format"):
        write_artifact(_result(), tmp_path, formats=("xls",))
    assert list(tmp_path.iterdir()) == []

    nested = tmp_path / "not-created" / "artifact"
    with pytest.raises(ValueError, match="unsupported artifact table format"):
        write_artifact(_result(), nested, formats=("xls",))
    assert not nested.exists()
    assert not nested.parent.exists()


def test_table_annotations_keep_component_units_and_refuse_unknowns():
    identities = [{
        "sample_index": 0,
        "case_index": 0,
        "case_name": "case",
        "output_sample_id": 0,
        "time_s": 0.0,
        "sample_role": "output",
    }]
    bushing, _ = _channel_table(
        np.zeros((1, 1, 12)),
        identities=identities,
        channel_name="bushing_output",
        case_spans=(),
    )
    assert (bushing[0]["unit"], bushing[0]["frame_id"]) == ("m", "local")
    assert (bushing[3]["unit"], bushing[3]["frame_id"]) == ("rad", "local")
    assert (bushing[6]["unit"], bushing[6]["frame_id"]) == ("N", "local")
    assert (bushing[9]["unit"], bushing[9]["frame_id"]) == ("N*m", "local")
    bump_stop, _ = _channel_table(
        np.zeros((1, 1, 5)),
        identities=identities,
        channel_name="bump_stop_output",
        case_spans=(),
    )
    assert bump_stop[1]["unit"] == "m/s"

    unknown, _ = _channel_table(
        np.array([[np.nan]]),
        identities=identities,
        channel_name="unmapped_channel",
        case_spans=(),
    )
    assert unknown[0]["unit"] is None
    assert unknown[0]["frame_id"] is None
    assert unknown[0]["reason"] == "non_finite_value"


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
