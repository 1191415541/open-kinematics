"""CLI dispatch tests for the time-domain Adams gates."""

from __future__ import annotations

from typer.testing import CliRunner

from suspension_multibody.cli import app


def test_cli_lists_time_domain_adams_gate_flags() -> None:
    result = CliRunner().invoke(app, ["validate-adams", "--help"])

    assert result.exit_code == 0
    assert "--axle-time-domain" in result.stdout
    assert "--vehicle-kc" in result.stdout
    assert "--handling" in result.stdout
    assert "--ride" in result.stdout


def test_cli_rejects_multiple_time_domain_gates() -> None:
    result = CliRunner().invoke(app, ["validate-adams", "--handling", "--ride"])

    assert result.exit_code == 1
    assert "mutually exclusive" in result.output


def test_cli_requires_an_explicit_external_axle_runner() -> None:
    result = CliRunner().invoke(app, ["validate-adams", "--axle-time-domain"])

    assert result.exit_code == 1
    assert "dynamic-model" in result.output
