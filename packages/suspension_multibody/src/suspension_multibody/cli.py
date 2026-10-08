"""Command-line entry point for suspension-multibody."""

from __future__ import annotations

from pathlib import Path
from typing import cast

import typer

from . import __version__
from .adams.probe import DEFAULT_PROFILE
from .api import validate as compile_documents
from .io.artifacts import write_artifact
from .simulation.runner import run_compiled

app = typer.Typer(add_completion=False, no_args_is_help=True)


def _validate_documents(assembly: Path, case: Path) -> None:
    """Resolve and compile the same documents submitted by every run command."""
    try:
        compile_documents(assembly, case)
    except Exception as exc:  # noqa: BLE001 - the loader's message is the report
        raise typer.BadParameter(str(exc)) from exc


def _declaration_and_dynamic_case(model: Path, case: Path):
    """
    Read an axle declaration and a dynamic case, for the Adams gates.

    The Adams time-domain gates compare a *replay* to Adams, and a replay takes a
    declaration and a dynamic case rather than a document pair.  The v1 *loaders*
    are retired with the modelling route, so this reads the two files straight
    into their schema types: these gates are an internal comparison whose inputs
    are declared in tests, not a user-facing modelling door.
    """
    import json

    import yaml
    from pydantic import ValidationError

    from .schema.dynamic import DynamicCaseSpec
    from .schema.model import AxleDeclaration

    def _read(path: Path) -> dict:
        text = path.read_text(encoding="utf-8")
        payload = (
            json.loads(text) if path.suffix.lower() == ".json" else yaml.safe_load(text)
        )
        if not isinstance(payload, dict):
            raise typer.BadParameter(f"{path} must contain an object")
        return payload

    try:
        return (
            AxleDeclaration.model_validate(_read(model)),
            DynamicCaseSpec.model_validate(_read(case)),
        )
    except (ValueError, ValidationError, OSError) as exc:
        raise typer.BadParameter(str(exc)) from exc


def _simulate_documents(assembly: Path, case: Path):
    """Submit one document pair through the unified entry."""
    return run_compiled(compile_documents(assembly, case))


def _write_run_artifact(run, out: Path) -> Path:
    return write_artifact(run.result, out, model=run.compiled.request.model, case=run.compiled.request.case)


@app.callback()
def main(
    version: bool = typer.Option(False, "--version", help="Show package version."),
) -> None:
    """Handle global CLI options."""
    if version:
        typer.echo(__version__)


@app.command("validate")
def validate(
    assembly: Path = typer.Option(
        ..., exists=True, readable=True, help="Assembly document JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Case contract document JSON."
    ),
) -> None:
    """Validate an assembly document and a case contract document."""
    _validate_documents(assembly, case)
    typer.echo("valid")


@app.command("run")
def run(
    assembly: Path = typer.Option(
        ..., exists=True, readable=True, help="Assembly document JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Case contract document JSON."
    ),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run one document pair and write its native channels."""
    run_envelope = _simulate_documents(assembly, case)
    manifest = _write_run_artifact(run_envelope, out)
    typer.echo(f"{run_envelope.status}: {manifest}")


@app.command("validate-dynamic")
def validate_dynamic(
    assembly: Path = typer.Option(
        ..., exists=True, readable=True, help="Assembly document JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Dynamic case contract document JSON."
    ),
) -> None:
    """Validate a dynamic assembly document and its case contract document."""
    _validate_documents(assembly, case)
    typer.echo("valid")


@app.command("run-dynamic")
def run_dynamic(
    assembly: Path = typer.Option(
        ..., exists=True, readable=True, help="Assembly document JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Dynamic case contract document JSON."
    ),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run one dynamic document pair and write structured results."""
    run_envelope = _simulate_documents(assembly, case)
    manifest = _write_run_artifact(run_envelope, out)
    typer.echo(f"{run_envelope.status}: {manifest}")


@app.command("validate-vehicle-dynamics")
def validate_vehicle_dynamics(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Full-vehicle assembly document JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Vehicle dynamic case document JSON."
    ),
) -> None:
    """Validate a full-vehicle assembly document and its case document."""
    _validate_documents(model, case)
    typer.echo("valid")


@app.command("run-vehicle-dynamics")
def run_vehicle_dynamics_command(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Full-vehicle assembly document JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Vehicle dynamic case document JSON."
    ),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run a full-vehicle document pair and retain raw result evidence."""
    from .kernel import KernelContractError

    compiled = compile_documents(model, case)
    try:
        run_envelope = run_compiled(compiled)
    except KernelContractError as exc:
        from .results.envelope import ResultEnvelope
        from .results.raw import RawContractResult

        raw = exc.partial_raw_result
        partial = None if raw is None else ResultEnvelope(cast(RawContractResult, raw), compiled.request.model)
        artifact = write_artifact(None, out, partial=partial, status="failed", failure=exc)
        typer.echo(str(exc), err=True)
        typer.echo(f"artifact: {artifact}", err=True)
        raise typer.Exit(code=1) from exc
    artifact = _write_run_artifact(run_envelope, out)
    typer.echo(f"{run_envelope.status}: {artifact}")


@app.command("run-axle-dynamics")
def run_axle_dynamics_command(
    assembly: Path = typer.Option(
        ..., exists=True, readable=True, help="Assembly document JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Case contract document JSON."
    ),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run one axle document pair through the unified entry."""
    run_envelope = _simulate_documents(assembly, case)
    manifest = _write_run_artifact(run_envelope, out)
    typer.echo(f"{run_envelope.status}: {manifest}")


@app.command("run-native-axle-evidence")
def run_native_axle_evidence_command(
    manifest: Path = typer.Option(
        ..., exists=True, readable=True, help="Dynamic axle manifest JSON."
    ),
    out: Path = typer.Option(..., help="Independent native evidence directory."),
    producer_id: str = typer.Option(
        "open-kinematics.native",
        help="Unique runner identity recorded in evidence.",
    ),
) -> None:
    """Run the native solver plus step-halving and write strict evidence."""
    from .adams import run_native_axle_manifest

    path = run_native_axle_manifest(
        manifest,
        out,
        producer_id=producer_id,
    )
    typer.echo(str(path))


@app.command("compare-axle-adams")
def compare_axle_adams_command(
    manifest: Path = typer.Option(
        ..., exists=True, readable=True, help="Dynamic axle manifest JSON."
    ),
    adams_evidence: Path = typer.Option(
        ..., exists=True, readable=True, help="Independent Adams evidence JSON."
    ),
    native_evidence: Path = typer.Option(
        ..., exists=True, readable=True, help="Independent native evidence JSON."
    ),
    out: Path = typer.Option(..., help="Comparison report JSON."),
) -> None:
    """Compare independently generated evidence without interpolation."""
    from .adams import compare_axle_evidence

    report = compare_axle_evidence(
        manifest_path=manifest,
        adams_evidence_path=adams_evidence,
        native_evidence_path=native_evidence,
        output_path=out,
    )
    if not report["passed"]:
        typer.echo(f"{report['status']}: {out}", err=True)
        raise typer.Exit(code=1)
    typer.echo(f"PASS: {out}")


@app.command("validate-adams")
def validate_adams(
    profile: str = typer.Option(
        DEFAULT_PROFILE,
        help=(
            "Adams profile: 'adams-car' for the installed release, or "
            "'adams-car-<release>' to require one exact release."
        ),
    ),
    smoke: bool = typer.Option(False),
    full: bool = typer.Option(False),
    strict_k: bool = typer.Option(
        False,
        "--strict-k",
        help="Run the fixed 9-state Adams pure-kinematic equivalence gate.",
    ),
    strict_c: bool = typer.Option(
        False,
        "--strict-c",
        help="Run the fixed 66-state native-Adams compliant equivalence gate.",
    ),
    axle_time_domain: bool = typer.Option(
        False,
        "--axle-time-domain",
        help="Run an axle time-domain gate through an explicit external Adams runner.",
    ),
    vehicle_kc: bool = typer.Option(
        False,
        "--vehicle-kc",
        help="Run the native-Adams prescribed body-roll KC dynamic gate.",
    ),
    handling: bool = typer.Option(
        False,
        "--handling",
        help="Run full-vehicle Adams/Car handling maneuver execution gates.",
    ),
    ride: bool = typer.Option(
        False,
        "--ride",
        help="Run full-vehicle Adams/Car ride maneuver execution gates.",
    ),
    dynamic_model: Path | None = typer.Option(
        None,
        "--dynamic-model",
        exists=True,
        readable=True,
        help="Model document required by --axle-time-domain or --vehicle-kc.",
    ),
    dynamic_case: Path | None = typer.Option(
        None,
        "--dynamic-case",
        exists=True,
        readable=True,
        help="Case document required by --axle-time-domain or --vehicle-kc.",
    ),
    time_runner: str | None = typer.Option(
        None,
        "--time-runner",
        help="External Adams axle time-domain runner command.",
    ),
    require_installed: bool = typer.Option(False, "--require-installed"),
    reference: Path | None = typer.Option(
        None,
        "--reference",
        exists=True,
        readable=True,
        help="Override the built-in suspension_multibody reference for --full.",
    ),
    runner: str | None = typer.Option(
        None,
        "--runner",
        help="Override the built-in Adams/Car batch runner.",
    ),
    evidence_dir: Path | None = typer.Option(
        None,
        "--evidence-dir",
        help="Directory for non-proprietary strict K/C evidence.",
    ),
) -> None:
    """Validate the local Adams/Car profile and optional full contract."""
    selected = sum(
        (
            smoke,
            full,
            strict_k,
            strict_c,
            axle_time_domain,
            vehicle_kc,
            handling,
            ride,
        )
    )
    if selected > 1:
        typer.echo("Adams validation gates are mutually exclusive", err=True)
        raise typer.Exit(code=1)
    if (axle_time_domain or vehicle_kc) and (
        dynamic_model is None or dynamic_case is None
    ):
        typer.echo(
            "--axle-time-domain and --vehicle-kc require --dynamic-model and --dynamic-case",
            err=True,
        )
        raise typer.Exit(code=1)
    if axle_time_domain and time_runner is None:
        typer.echo("--axle-time-domain requires --time-runner", err=True)
        raise typer.Exit(code=1)

    if axle_time_domain:
        from .adams import (
            command_time_domain_runner,
            discover_profile,
            validate_axle_time_domain,
        )

        assert dynamic_model is not None
        assert dynamic_case is not None
        assert time_runner is not None
        # The gate compares a *replay* against Adams.  The replay's two inputs are
        # the axle declaration and the dynamic case; the retired v1 loaders are not
        # a door any more, so the two files are read into their schema types here.
        declaration, dynamic_inputs = _declaration_and_dynamic_case(
            dynamic_model, dynamic_case
        )
        result = validate_axle_time_domain(
            discover_profile(profile),
            declaration,
            dynamic_inputs,
            runner=command_time_domain_runner(time_runner),
            output_dir=evidence_dir,
        )
        _echo_time_domain_result(result.ok, result.message, result.output_path)
        return
    if vehicle_kc:
        from .adams import discover_profile, validate_vehicle_kc_time_domain

        assert dynamic_model is not None
        assert dynamic_case is not None
        result = validate_vehicle_kc_time_domain(
            discover_profile(profile),
            *_declaration_and_dynamic_case(dynamic_model, dynamic_case),
            output_dir=evidence_dir,
        )
        _echo_time_domain_result(result.ok, result.message, result.output_path)
        return
    if handling:
        from .adams import discover_profile, validate_handling_execution

        result = validate_handling_execution(
            discover_profile(profile), output_dir=evidence_dir
        )
        _echo_time_domain_result(
            result.ok, "Adams/Car handling execution gate", result.output_path
        )
        return
    if ride:
        from .adams import discover_profile, validate_ride_execution

        result = validate_ride_execution(discover_profile(profile), output_dir=evidence_dir)
        _echo_time_domain_result(
            result.ok, "Adams/Car ride execution gate", result.output_path
        )
        return

    from .adams import validate_profile

    result = validate_profile(
        profile,
        smoke=smoke,
        full=full,
        require_installed=require_installed,
        reference=reference,
        runner=runner,
        strict_k=strict_k,
        strict_c=strict_c,
        evidence_dir=evidence_dir,
    )
    if not result.ok:
        typer.echo(result.message, err=True)
        if result.output_path:
            typer.echo(f"report: {result.output_path}", err=True)
        raise typer.Exit(code=1)
    typer.echo(result.message)


def _echo_time_domain_result(ok: bool, message: str, output_path: str) -> None:
    if not ok:
        typer.echo(message, err=True)
        typer.echo(f"report: {output_path}", err=True)
        raise typer.Exit(code=1)
    typer.echo(f"{message}: {output_path}")


if __name__ == "__main__":
    app()
