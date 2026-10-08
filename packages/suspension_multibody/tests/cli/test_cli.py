"""CLI command tests."""

import json
from pathlib import Path

import numpy as np
from typer.testing import CliRunner

from suspension_multibody.cli import app


def test_help_lists_validate_and_run() -> None:
    result = CliRunner().invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "validate" in result.stdout
    assert "run" in result.stdout
    assert "validate-dynamic" in result.stdout
    assert "run-dynamic" in result.stdout


def test_validate_rejects_missing_schema_version(tmp_path: Path) -> None:
    model = tmp_path / "model.yaml"
    case = tmp_path / "case.yaml"
    model.write_text("hardpoints: {}\nmass: {sprung_mass: 1}\n", encoding="utf-8")
    case.write_text("mode: K\n", encoding="utf-8")
    result = CliRunner().invoke(
        app, ["validate", "--model", str(model), "--case", str(case)]
    )
    assert result.exit_code != 0


def test_validate_adams_rejects_smoke_and_full_together() -> None:
    result = CliRunner().invoke(app, ["validate-adams", "--smoke", "--full"])

    assert result.exit_code == 1
    assert "mutually exclusive" in result.output


def test_validate_adams_rejects_full_and_strict_k_together() -> None:
    result = CliRunner().invoke(app, ["validate-adams", "--full", "--strict-k"])

    assert result.exit_code == 1
    assert "mutually exclusive" in result.output


def test_validate_adams_rejects_strict_k_and_strict_c_together() -> None:
    result = CliRunner().invoke(app, ["validate-adams", "--strict-k", "--strict-c"])

    assert result.exit_code == 1
    assert "mutually exclusive" in result.output


def test_generic_validate_compiles_without_submitting(monkeypatch) -> None:
    import suspension_multibody.cli as cli

    examples = Path(__file__).parents[2]/"examples/generic_multibody"
    def refused(*args, **kwargs):
        raise AssertionError("validate must not execute the solver")
    monkeypatch.setattr(cli, "run_compiled", refused)
    result = CliRunner().invoke(app, ["validate", "--assembly", str(examples/"bench.assembly.json"),
        "--case", str(examples/"dynamic.case.json")])
    assert result.exit_code == 0, result.output
    assert "valid" in result.output


def test_generic_run_writes_native_channels_for_any_subsystem(tmp_path: Path) -> None:
    examples = Path(__file__).parents[2]/"examples/generic_multibody"
    result = CliRunner().invoke(app, ["run", "--assembly", str(examples/"rotor.assembly.json"),
        "--case", str(examples/"rotor.case.json"), "--out", str(tmp_path)])
    assert result.exit_code == 0, result.output
    manifest = json.loads((tmp_path/"manifest.json").read_text(encoding="utf-8"))
    assert manifest["artifact_type"] == "multibody_result"
    assert manifest["case"]["protocol"] == "axle_dynamic"
    with np.load(tmp_path/manifest["arrays_file"], allow_pickle=False) as arrays:
        assert arrays["body_state"].shape[0] == len(arrays["times_s"])


def test_generic_cli_failed_run_retains_partial_native_evidence(tmp_path: Path) -> None:
    examples = Path(__file__).parents[2]/"examples/generic_multibody"
    result = CliRunner().invoke(app, ["run-vehicle-dynamics", "--model", str(examples/"bench.assembly.json"),
        "--case", str(examples/"dynamic.case.json"), "--out", str(tmp_path)])
    assert result.exit_code == 1
    manifest = json.loads((tmp_path/"manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["artifact_type"] == "multibody_result"
    assert manifest["failure_evidence"]["type"] == "KernelContractError"
    assert manifest["partial_evidence"]["available"]
    with np.load(tmp_path/manifest["arrays_file"], allow_pickle=False) as arrays:
        assert "diagnostics" in arrays
