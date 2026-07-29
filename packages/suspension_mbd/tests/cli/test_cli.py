"""CLI command tests."""

from pathlib import Path

from typer.testing import CliRunner

from suspension_mbd.cli import app


def test_help_lists_validate_and_run() -> None:
    result = CliRunner().invoke(app, ["--help"])
    assert result.exit_code == 0
    assert "validate" in result.stdout
    assert "run" in result.stdout


def test_validate_rejects_missing_schema_version(tmp_path: Path) -> None:
    model = tmp_path / "model.yaml"
    case = tmp_path / "case.yaml"
    model.write_text("hardpoints: {}\nmass: {sprung_mass: 1}\n", encoding="utf-8")
    case.write_text("mode: K\n", encoding="utf-8")
    result = CliRunner().invoke(
        app, ["validate", "--model", str(model), "--case", str(case)]
    )
    assert result.exit_code != 0
