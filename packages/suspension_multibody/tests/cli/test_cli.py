"""CLI command tests."""

from pathlib import Path

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
