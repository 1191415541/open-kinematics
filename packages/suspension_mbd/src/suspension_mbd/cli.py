"""Command-line entry point for suspension-mbd."""

from __future__ import annotations

from pathlib import Path

import typer

from . import __version__
from .api import run_case
from .schema import load_case, load_model

app = typer.Typer(add_completion=False, no_args_is_help=True)


@app.callback()
def main(
    version: bool = typer.Option(False, "--version", help="Show package version."),
) -> None:
    """Handle global CLI options."""
    if version:
        typer.echo(__version__)


@app.command("validate")
def validate(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Model YAML/JSON."
    ),
    case: Path = typer.Option(..., exists=True, readable=True, help="Case YAML/JSON."),
) -> None:
    """Validate v1 model and case files."""
    load_model(model)
    load_case(case)
    typer.echo("valid")


@app.command("run")
def run(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Model YAML/JSON."
    ),
    case: Path = typer.Option(..., exists=True, readable=True, help="Case YAML/JSON."),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run one K/C case and write structured results."""
    bundle = run_case(load_model(model), load_case(case), out)
    typer.echo(f"{bundle.manifest.run_id}: {bundle.manifest.state_count} state(s)")


@app.command("validate-adams")
def validate_adams(
    profile: str = typer.Option("adams-car-2024.1"),
    smoke: bool = typer.Option(False),
    full: bool = typer.Option(False),
    require_installed: bool = typer.Option(False, "--require-installed"),
    reference: Path | None = typer.Option(
        None,
        "--reference",
        exists=True,
        readable=True,
        help="Non-proprietary JSON/CSV Adams reference results for --full.",
    ),
    runner: str | None = typer.Option(
        None,
        "--runner",
        help="External Adams runner command; request and output paths are appended.",
    ),
) -> None:
    """Validate the local Adams/Car profile and optional full contract."""
    from .adams import validate_profile

    result = validate_profile(
        profile,
        smoke=smoke,
        full=full,
        require_installed=require_installed,
        reference=reference,
        runner=runner,
    )
    if not result.ok:
        typer.echo(result.message, err=True)
        if result.output_path:
            typer.echo(f"report: {result.output_path}", err=True)
        raise typer.Exit(code=1)
    typer.echo(result.message)


if __name__ == "__main__":
    app()
