from pathlib import Path

import typer

from suspension_kinematics.core.enums import PointID
from suspension_kinematics.io.coupled_loader import parse_coupled_sweep_file
from suspension_kinematics.io.geometry_loader import load_geometry
from suspension_kinematics.io.results_writer import (
    SolutionFrame,
    create_writer_for_path,
)
from suspension_kinematics.io.sweep_loader import parse_sweep_file
from suspension_kinematics.main import solve_sweep
from suspension_kinematics.metrics import compute_metrics_for_state
from suspension_kinematics.steering import load_two_segment_steering_hardpoints_csv
from suspension_kinematics.vehicle import CoupledSweepResult, solve_coupled_sweep

app = typer.Typer(add_completion=False, no_args_is_help=True)


def _prefixed_positions(
    result: CoupledSweepResult,
    output_points: tuple[PointID, ...],
) -> dict[str, tuple[float, float, float]]:
    positions: dict[str, tuple[float, float, float]] = {}
    for prefix, state in (
        ("left", result.left_state),
        ("right", result.right_state),
    ):
        for point_id in output_points:
            pos = state.positions.get(point_id)
            if pos is not None:
                positions[f"{prefix}_{point_id.name}"] = (
                    float(pos[0]),
                    float(pos[1]),
                    float(pos[2]),
                )
    return positions


@app.command()
def sweep(
    geometry: Path = typer.Option(..., exists=True, help="Path to geometry YAML"),
    sweep: Path = typer.Option(..., exists=True, help="Path to sweep YAML"),
    out: Path = typer.Option(..., help="Output path (.parquet or .csv)"),
    animation_out: Path | None = typer.Option(
        None, help="Optional animation output path (.mp4, .gif, etc.)"
    ),
):
    """
    Run a sweep from file and write results to Parquet or CSV format.

    Example:
        suspension-kinematics sweep --geometry=geo.yaml --sweep=sweep.yaml --out=out.parquet
        suspension-kinematics sweep --geometry=geo.yaml --sweep=sweep.yaml --out=out.csv
    """
    suspension = load_geometry(geometry)
    sweep_config = parse_sweep_file(sweep)

    solution_states, solver_stats = solve_sweep(suspension, sweep_config)

    # Write out in wide format.
    writer = create_writer_for_path(
        out, geometry_path=str(geometry), sweep_path=str(sweep)
    )
    output_points = suspension.OUTPUT_POINTS
    config = suspension.config
    for idx, (st, solver_info) in enumerate(zip(solution_states, solver_stats)):
        # Filter to the suspension type's declared output points, in order.
        positions = {
            pid.name: (float(pos[0]), float(pos[1]), float(pos[2]))
            for pid in output_points
            if (pos := st.positions.get(pid)) is not None
        }

        # Compute post-solve metrics for this state.
        metrics: dict[str, float | None] = {}
        if config is not None:
            metrics = compute_metrics_for_state(st, suspension, config)

        frame = SolutionFrame(
            positions=positions,
            solver_info=solver_info,
            metrics=metrics,
        )

        writer.add_frame(idx, frame)
    writer.write()

    typer.echo(f"wrote {out}")

    # Generate animation if requested.
    if animation_out:
        try:
            from suspension_kinematics.visualization.api import (
                visualize_suspension_sweep,
            )

            # Get wheel parameters from suspension configuration.
            if suspension.config is None:
                typer.echo("Error: No config in suspension", err=True)
                raise typer.Exit(1)

            wheel_cfg = suspension.config.wheel

            # Create animation.
            visualize_suspension_sweep(
                suspension=suspension,
                solution_states=solution_states,
                output_path=animation_out,
                wheel_diameter=wheel_cfg.tire.nominal_radius * 2,
                wheel_width=wheel_cfg.tire.section_width,
                fps=20,
                show_live=False,
            )

            typer.echo(f"Wrote animation: {animation_out}")

        except ImportError as e:
            typer.echo(
                f"Error: Visualization dependencies not installed.\n"
                f'Install with: pip install "suspension-kinematics[viz]"\n'
                f"Details: {e}",
                err=True,
            )
            typer.Exit(1)


@app.command("coupled-sweep")
def coupled_sweep(
    geometry: Path = typer.Option(
        ...,
        exists=True,
        help="Path to one-side suspension geometry YAML",
    ),
    steering: Path = typer.Option(
        ...,
        exists=True,
        help="Path to two-segment steering hardpoint CSV",
    ),
    coupled_sweep: Path = typer.Option(
        ...,
        exists=True,
        help="Path to coupled sweep YAML",
    ),
    out: Path = typer.Option(..., help="Output path (.parquet or .csv)"),
    animation_out: Path | None = typer.Option(
        None,
        help="Optional vehicle animation output path (.gif)",
    ),
):
    """
    Run a weakly coupled left/right suspension and steering sweep.
    """
    source_suspension = load_geometry(geometry)
    steering_geometry = load_two_segment_steering_hardpoints_csv(steering)
    coupled_config = parse_coupled_sweep_file(coupled_sweep)

    results = solve_coupled_sweep(
        source_suspension=source_suspension,
        steering_geometry=steering_geometry,
        wheel_travel_values=coupled_config.wheel_travel_values,
        pitman_angle_values=coupled_config.pitman_angle_values,
    )

    writer = create_writer_for_path(
        out,
        geometry_path=str(geometry),
        sweep_path=str(coupled_sweep),
        steering_path=str(steering),
    )
    output_points = source_suspension.OUTPUT_POINTS
    for result in results:
        writer.add_frame(
            result.step_index,
            SolutionFrame(
                positions=_prefixed_positions(result, output_points),
                solver_info=result.solver_info,
                metrics=result.metrics,
            ),
        )
    writer.write()

    typer.echo(f"wrote {out}")

    if not isinstance(animation_out, Path):
        animation_out = None

    if animation_out:
        try:
            from suspension_kinematics.visualization.coupled import (
                create_coupled_animation,
            )

            create_coupled_animation(
                source_suspension=source_suspension,
                steering_geometry=steering_geometry,
                results=results,
                output_path=animation_out,
                fps=12,
            )
            typer.echo(f"Wrote animation: {animation_out}")
        except ImportError as e:
            typer.echo(
                f"Error: Visualization dependencies not installed.\n"
                f'Install with: pip install "suspension-kinematics[viz]"\n'
                f"Details: {e}",
                err=True,
            )
            raise typer.Exit(1)


@app.command()
def visualize(
    geometry: Path = typer.Option(..., exists=True, help="Path to geometry YAML."),
    output: Path = typer.Option(
        ..., help="Output path for the plot image (.png, .jpg)."
    ),
):
    """
    Visualize a suspension geometry at its design condition.

    This command loads a single geometry file, calculates its initial state, and
    generates a debug plot. It also reports whether the contact patch approximation
    (minimum Z position on wheel center plane) is tangent to the ground plane (Z=0).

    Example:
    uv run suspension-kinematics visualize --geometry=tests/data/geometry.yaml --output=plot.png
    """
    try:
        from suspension_kinematics.visualization.api import visualize_geometry
    except ImportError as e:
        typer.echo(
            f"Error: Visualization dependencies not installed.\n"
            f'Install with: pip install "suspension-kinematics[viz]"\n'
            f"Details: {e}",
            err=True,
        )
        raise typer.Exit(1)

    suspension = load_geometry(geometry)

    visualize_geometry(
        suspension=suspension,
        output_path=output,
    )


@app.command("gui")
def gui():
    """
    Launch the kinematics workbench GUI.
    """
    try:
        from suspension_kinematics.gui.app import main as run_gui
    except ImportError as e:
        typer.echo(
            f"Error: GUI dependencies not installed.\n"
            f"Install/run with: uv run --extra viz suspension-kinematics gui\n"
            f"Details: {e}",
            err=True,
        )
        raise typer.Exit(1)

    run_gui()


@app.command("steering-gui")
def steering_gui():
    """
    Launch the steering workbench GUI.
    """
    try:
        from suspension_kinematics.gui.steering import main as run_steering_gui
    except ImportError as e:
        typer.echo(
            f"Error: GUI dependencies not installed.\n"
            f"Install/run with: uv run --extra viz suspension-kinematics steering-gui\n"
            f"Details: {e}",
            err=True,
        )
        raise typer.Exit(1)

    run_steering_gui()


if __name__ == "__main__":
    app()
