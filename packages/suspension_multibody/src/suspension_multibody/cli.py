"""Command-line entry point for suspension-multibody."""

from __future__ import annotations

from pathlib import Path

import typer

from . import __version__
from .api import run_case, run_dynamic_case
from .schema import (
    load_case,
    load_dynamic_case,
    load_model,
    load_vehicle_dynamic_case,
    load_vehicle_model,
)

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


@app.command("validate-dynamic")
def validate_dynamic(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Model YAML/JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Dynamic case YAML/JSON."
    ),
) -> None:
    """Validate a v1 model and dynamic case file."""
    load_model(model)
    load_dynamic_case(case)
    typer.echo("valid")


@app.command("run-dynamic")
def run_dynamic(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Model YAML/JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Dynamic case YAML/JSON."
    ),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run one time-domain case and write structured results."""
    bundle = run_dynamic_case(load_model(model), load_dynamic_case(case), out)
    typer.echo(f"{bundle.manifest.run_id}: {bundle.manifest.sample_count} sample(s)")


@app.command("validate-vehicle-dynamics")
def validate_vehicle_dynamics(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Full-vehicle model YAML/JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Full-vehicle dynamic case YAML/JSON."
    ),
) -> None:
    """Validate a full-vehicle native dynamics model and case."""
    vehicle_model = load_vehicle_model(model)
    vehicle_case = load_vehicle_dynamic_case(case)
    if vehicle_case.vehicle.model_dump() != vehicle_model.model_dump():
        raise typer.BadParameter("the case vehicle does not match the model file")
    typer.echo("valid")


@app.command("run-vehicle-dynamics")
def run_vehicle_dynamics_command(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Full-vehicle model YAML/JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Full-vehicle dynamic case YAML/JSON."
    ),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run native full-vehicle dynamics and retain raw result evidence."""
    from .axle_dynamics import NativeAxleError, NativeKernelUnavailableError
    from .vehicle_dynamics import (
        VehicleDynamicsResult,
        run_vehicle_dynamics,
        write_vehicle_dynamics_artifact,
    )

    vehicle_model = load_vehicle_model(model)
    vehicle_case = load_vehicle_dynamic_case(case)
    try:
        result = run_vehicle_dynamics(vehicle_model, vehicle_case)
    except (NativeAxleError, NativeKernelUnavailableError) as exc:
        partial_result = getattr(exc, "partial_result", None)
        artifact = write_vehicle_dynamics_artifact(
            None if partial_result is None else VehicleDynamicsResult(
                axle=partial_result
            ),
            vehicle_model,
            vehicle_case,
            out,
            failure=exc,
        )
        typer.echo(str(exc), err=True)
        typer.echo(f"artifact: {artifact}", err=True)
        raise typer.Exit(code=1) from exc
    artifact = write_vehicle_dynamics_artifact(
        result, vehicle_model, vehicle_case, out
    )
    typer.echo(f"{len(result.times_s)} sample(s): {artifact}")


@app.command("validate-axle-dynamics")
def validate_axle_dynamics(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Axle model YAML/JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Axle case YAML/JSON."
    ),
) -> None:
    """Validate closed SI axle-dynamics inputs."""
    from .axle_dynamics import (
        load_axle_dynamics_case,
        load_axle_dynamics_model,
    )

    load_axle_dynamics_model(model)
    load_axle_dynamics_case(case)
    typer.echo("valid")


@app.command("run-axle-dynamics")
def run_axle_dynamics_command(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Axle model YAML/JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Axle case YAML/JSON."
    ),
    out: Path = typer.Option(..., help="Output directory."),
) -> None:
    """Run the native axle solver and retain success or failure evidence."""
    from .axle_dynamics import (
        NativeAxleError,
        NativeKernelUnavailableError,
        load_axle_dynamics_case,
        load_axle_dynamics_model,
        run_axle_dynamics,
        write_axle_dynamics_artifact,
    )

    axle_model = load_axle_dynamics_model(model)
    axle_case = load_axle_dynamics_case(case)
    try:
        result = run_axle_dynamics(axle_model, axle_case)
    except (NativeAxleError, NativeKernelUnavailableError) as exc:
        partial_result = getattr(exc, "partial_result", None)
        manifest = write_axle_dynamics_artifact(
            partial_result,
            axle_model,
            axle_case,
            out,
            failure=exc,
        )
        typer.echo(str(exc), err=True)
        typer.echo(f"artifact: {manifest}", err=True)
        raise typer.Exit(code=1) from exc
    manifest = write_axle_dynamics_artifact(
        result, axle_model, axle_case, out
    )
    typer.echo(f"{len(result.times_s)} sample(s): {manifest}")


@app.command("create-axle-manifest")
def create_axle_manifest_command(
    model: Path = typer.Option(
        ..., exists=True, readable=True, help="Axle model YAML/JSON."
    ),
    case: Path = typer.Option(
        ..., exists=True, readable=True, help="Axle case YAML/JSON."
    ),
    settings: Path = typer.Option(
        ...,
        exists=True,
        readable=True,
        help="Role bindings and Adams execution settings YAML/JSON.",
    ),
    out: Path = typer.Option(..., help="Dynamic axle manifest JSON."),
) -> None:
    """Freeze the sole shared input for independent native and Adams runners."""
    from .adams import (
        create_dynamic_axle_manifest,
        load_dynamic_axle_manifest_settings,
        write_dynamic_axle_manifest,
    )
    from .axle_dynamics import (
        load_axle_dynamics_case,
        load_axle_dynamics_model,
    )

    manifest_settings = load_dynamic_axle_manifest_settings(settings)
    manifest = create_dynamic_axle_manifest(
        load_axle_dynamics_model(model),
        load_axle_dynamics_case(case),
        manifest_settings.role_bindings,
        adams_solver=manifest_settings.adams_solver,
        execution_environment=manifest_settings.execution_environment,
        case_metadata=manifest_settings.case_metadata,
    )
    path = write_dynamic_axle_manifest(manifest, out)
    typer.echo(f"{manifest.sha256}: {path}")


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
    profile: str = typer.Option("adams-car-2024.1"),
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
        help="Model YAML/JSON required by --axle-time-domain or --vehicle-kc.",
    ),
    dynamic_case: Path | None = typer.Option(
        None,
        "--dynamic-case",
        exists=True,
        readable=True,
        help="Dynamic case YAML/JSON required by --axle-time-domain or --vehicle-kc.",
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
        result = validate_axle_time_domain(
            discover_profile(profile),
            load_model(dynamic_model),
            load_dynamic_case(dynamic_case),
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
            load_model(dynamic_model),
            load_dynamic_case(dynamic_case),
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
