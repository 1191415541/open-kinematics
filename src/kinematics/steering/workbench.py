"""
Workbench model for the two-segment steering GUI.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares

from kinematics.steering.csv_loader import load_two_segment_steering_hardpoints_rows
from kinematics.steering.geometry import (
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
)
from kinematics.steering.limits import (
    estimate_two_segment_steering_limits,
    steering_limit_outputs,
)
from kinematics.steering.outputs import (
    available_steering_outputs as _available_steering_outputs,
)
from kinematics.steering.outputs import (
    outputs_from_solution,
)
from kinematics.steering.two_segment import (
    solve_two_segment_from_left_wheel_angle,
    solve_two_segment_from_right_wheel_angle,
    solve_two_segment_steering,
)

INPUT_MODES = ("pitman_angle", "left_wheel_angle", "right_wheel_angle")
PARTIAL_FLOAT_TEXT = frozenset({"", "+", "-", ".", "+.", "-."})
UNREACHABLE_SOLVE_PREFIXES = (
    "No valid steering arm position",
    "No valid pitman arm position",
)
OPTIMIZATION_VARIABLES = (
    "pitman_x",
    "pitman_arm_x_length",
    "tie_rod_outer_x",
    "tie_rod_outer_y",
    "tie_rod_inner_x",
    "tie_rod_inner_y",
)


@dataclass(frozen=True)
class ParsedFloatEntry:
    """Result of parsing a live GUI numeric entry."""

    value: float
    is_valid: bool
    is_complete: bool


@dataclass(frozen=True)
class SliderLimits:
    """Numeric range for a GUI slider."""

    minimum: float
    maximum: float


@dataclass(frozen=True)
class SteeringOptimizationResult:
    """Result of one steering hardpoint optimization."""

    hardpoints: list["SteeringHardpointRow"]
    initial_error_deg: float
    final_error_deg: float
    actual_left_minus_right_deg: float
    success: bool
    message: str
    applied_values: dict[str, float]


def parse_float_entry(text: str, previous: float) -> ParsedFloatEntry:
    """Parse a live numeric entry without rejecting partial edits."""
    stripped = text.strip()
    if stripped in PARTIAL_FLOAT_TEXT:
        return ParsedFloatEntry(previous, is_valid=True, is_complete=False)
    try:
        return ParsedFloatEntry(float(stripped), is_valid=True, is_complete=True)
    except ValueError:
        return ParsedFloatEntry(previous, is_valid=False, is_complete=False)


def _is_unreachable_solve_error(exc: ValueError) -> bool:
    return str(exc).startswith(UNREACHABLE_SOLVE_PREFIXES)


def _row_snapshot(
    rows: list["SteeringHardpointRow"],
) -> list[tuple[float, float, float]]:
    return [(row.x, row.y, row.z) for row in rows]


def _restore_row_snapshot(
    rows: list["SteeringHardpointRow"],
    snapshot: list[tuple[float, float, float]],
) -> None:
    for row, (x, y, z) in zip(rows, snapshot):
        row.x = x
        row.y = y
        row.z = z


@dataclass
class SteeringHardpointRow:
    """Editable CSV-style steering hardpoint row."""

    category: str
    name: str
    x: float
    y: float
    z: float

    def as_loader_row(self) -> dict[str, Any]:
        """Return a row compatible with the steering CSV loader."""
        return asdict(self)


@dataclass
class SteeringCurve:
    """Curve definition for plotting one output against another."""

    x_output: str
    y_output: str
    label: str = ""


def default_hardpoint_rows() -> list[SteeringHardpointRow]:
    """Return a practical symmetric two-segment steering hardpoint set."""
    return [
        SteeringHardpointRow("symmetric", "wheel_kingpin_lower", 0.0, -500.0, 280.0),
        SteeringHardpointRow("symmetric", "wheel_kingpin_upper", 0.0, -500.0, 340.0),
        SteeringHardpointRow("symmetric", "wheel_center", 60.0, -520.0, 320.0),
        SteeringHardpointRow(
            "symmetric",
            "wheel_tie_rod_pickup",
            -180.0,
            -420.0,
            280.0,
        ),
        SteeringHardpointRow("symmetric", "pitman_output", -350.0, -120.0, 285.0),
        SteeringHardpointRow("center", "pitman_pivot", -350.0, 0.0, 300.0),
    ]


@dataclass
class SteeringProject:
    """Persisted steering GUI project state."""

    name: str = "Untitled steering project"
    hardpoints: list[SteeringHardpointRow] = field(
        default_factory=default_hardpoint_rows
    )
    input_mode: str = "pitman_angle"
    input_value: float = 0.0
    sweep_min: float = -20.0
    sweep_max: float = 20.0
    sweep_step: float = 2.0
    wheel_radius: float = 180.0
    wheel_width: float = 120.0
    wheelbase: float = 2800.0
    curves: list[SteeringCurve] = field(default_factory=list)


def default_steering_project() -> SteeringProject:
    """Create a default steering project."""
    return SteeringProject()


def _required_hardpoint_row(
    rows: list[SteeringHardpointRow],
    name: str,
) -> SteeringHardpointRow:
    for row in rows:
        if row.name == name:
            return row
    raise ValueError(f"Missing steering hardpoint row '{name}'")


def _pitman_rows(
    rows: list[SteeringHardpointRow],
) -> tuple[SteeringHardpointRow, SteeringHardpointRow]:
    return (
        _required_hardpoint_row(rows, "pitman_pivot"),
        _required_hardpoint_row(rows, "pitman_output"),
    )


def _copy_hardpoint_rows(
    rows: list[SteeringHardpointRow],
) -> list[SteeringHardpointRow]:
    return [SteeringHardpointRow(**asdict(row)) for row in rows]


def copy_hardpoint_rows(
    rows: list[SteeringHardpointRow],
) -> list[SteeringHardpointRow]:
    """Return an independent copy of editable steering hardpoint rows."""
    return _copy_hardpoint_rows(rows)


def _get_optimization_variable(
    rows: list[SteeringHardpointRow],
    variable_name: str,
) -> float:
    pivot, pitman_output = _pitman_rows(rows)
    tie_rod_outer = _required_hardpoint_row(rows, "wheel_tie_rod_pickup")
    if variable_name == "pitman_x":
        return pivot.x
    if variable_name == "pitman_arm_x_length":
        return pitman_output.x - pivot.x
    if variable_name == "tie_rod_outer_x":
        return tie_rod_outer.x
    if variable_name == "tie_rod_outer_y":
        return tie_rod_outer.y
    if variable_name == "tie_rod_inner_x":
        return pitman_output.x
    if variable_name == "tie_rod_inner_y":
        return pitman_output.y
    raise ValueError(f"Unknown steering optimization variable '{variable_name}'")


def _set_optimization_variable(
    rows: list[SteeringHardpointRow],
    variable_name: str,
    value: float,
) -> None:
    pivot, pitman_output = _pitman_rows(rows)
    tie_rod_outer = _required_hardpoint_row(rows, "wheel_tie_rod_pickup")
    if variable_name == "pitman_x":
        delta = float(value) - pivot.x
        pivot.x += delta
        pitman_output.x += delta
    elif variable_name == "pitman_arm_x_length":
        pitman_output.x = pivot.x + float(value)
    elif variable_name == "tie_rod_outer_x":
        tie_rod_outer.x = float(value)
    elif variable_name == "tie_rod_outer_y":
        tie_rod_outer.y = float(value)
    elif variable_name == "tie_rod_inner_x":
        pitman_output.x = float(value)
    elif variable_name == "tie_rod_inner_y":
        pitman_output.y = float(value)
    else:
        raise ValueError(f"Unknown steering optimization variable '{variable_name}'")


def _apply_optimization_values(
    rows: list[SteeringHardpointRow],
    variable_names: tuple[str, ...],
    values: np.ndarray,
) -> None:
    for variable_name, value in zip(variable_names, values):
        _set_optimization_variable(rows, variable_name, float(value))


def _solve_target_inner_wheel_state(
    rows: list[SteeringHardpointRow],
    inner_wheel: str,
    inner_wheel_angle_deg: float,
) -> TwoSegmentSteeringSolution:
    hardpoints = hardpoints_from_rows(rows)
    if inner_wheel == "left":
        return solve_two_segment_from_left_wheel_angle(
            hardpoints,
            inner_wheel_angle_deg,
        )
    if inner_wheel == "right":
        return solve_two_segment_from_right_wheel_angle(
            hardpoints,
            inner_wheel_angle_deg,
        )
    raise ValueError("inner_wheel must be 'left' or 'right'")


def _left_minus_right_at_inner_wheel_angle(
    rows: list[SteeringHardpointRow],
    inner_wheel: str,
    inner_wheel_angle_deg: float,
) -> float:
    solution = _solve_target_inner_wheel_state(
        rows,
        inner_wheel,
        inner_wheel_angle_deg,
    )
    return solution.left_wheel_angle_deg - solution.right_wheel_angle_deg


def pitman_x_position(rows: list[SteeringHardpointRow]) -> float:
    """Return the current center pitman pivot X position."""
    pivot, _output = _pitman_rows(rows)
    return pivot.x


def pitman_arm_x_length(rows: list[SteeringHardpointRow]) -> float:
    """Return signed pitman output X offset from the pivot."""
    pivot, output = _pitman_rows(rows)
    return output.x - pivot.x


def pitman_angle_slider_limits(rows: list[SteeringHardpointRow]) -> SliderLimits:
    """Return pitman-angle slider limits from current reachable geometry."""
    hardpoints = hardpoints_from_rows(rows)
    limits = estimate_two_segment_steering_limits(hardpoints)
    low = min(limits.left_turn.pitman_angle_deg, limits.right_turn.pitman_angle_deg)
    high = max(limits.left_turn.pitman_angle_deg, limits.right_turn.pitman_angle_deg)
    return SliderLimits(minimum=low, maximum=high)


def input_angle_slider_limits(
    rows: list[SteeringHardpointRow],
    input_mode: str,
) -> SliderLimits:
    """Return slider limits for the selected steering input mode."""
    hardpoints = hardpoints_from_rows(rows)
    limits = estimate_two_segment_steering_limits(hardpoints)
    if input_mode == "pitman_angle":
        low = limits.right_turn.pitman_angle_deg
        high = limits.left_turn.pitman_angle_deg
    elif input_mode == "left_wheel_angle":
        low = limits.right_turn.left_wheel_angle_deg
        high = limits.left_turn.left_wheel_angle_deg
    elif input_mode == "right_wheel_angle":
        low = limits.right_turn.right_wheel_angle_deg
        high = limits.left_turn.right_wheel_angle_deg
    else:
        raise ValueError(f"Unknown steering input mode '{input_mode}'")
    return SliderLimits(minimum=min(low, high), maximum=max(low, high))


def optimize_steering_hardpoints(
    rows: list[SteeringHardpointRow],
    *,
    inner_wheel: str,
    inner_wheel_angle_deg: float,
    target_left_minus_right_deg: float,
    variable_names: tuple[str, ...],
    variable_delta_limit: float,
) -> SteeringOptimizationResult:
    """Optimize selected steering hardpoint variables to match wheel angle delta."""
    if not variable_names:
        raise ValueError("At least one steering optimization variable is required")
    if variable_delta_limit <= 0.0:
        raise ValueError("variable_delta_limit must be positive")
    variable_names = tuple(variable_names)
    start_rows = _copy_hardpoint_rows(rows)
    x0 = np.array(
        [_get_optimization_variable(start_rows, name) for name in variable_names],
        dtype=np.float64,
    )
    lower = x0 - variable_delta_limit
    upper = x0 + variable_delta_limit

    def residual(values: np.ndarray) -> np.ndarray:
        trial_rows = _copy_hardpoint_rows(start_rows)
        _apply_optimization_values(trial_rows, variable_names, values)
        try:
            actual = _left_minus_right_at_inner_wheel_angle(
                trial_rows,
                inner_wheel,
                inner_wheel_angle_deg,
            )
        except ValueError:
            return np.array([1e6], dtype=np.float64)
        return np.array([actual - target_left_minus_right_deg], dtype=np.float64)

    initial_error = float(abs(residual(x0)[0]))
    result = least_squares(residual, x0, bounds=(lower, upper), method="trf")
    optimized_rows = _copy_hardpoint_rows(start_rows)
    _apply_optimization_values(optimized_rows, variable_names, result.x)
    actual_delta = _left_minus_right_at_inner_wheel_angle(
        optimized_rows,
        inner_wheel,
        inner_wheel_angle_deg,
    )
    final_error = float(abs(actual_delta - target_left_minus_right_deg))
    return SteeringOptimizationResult(
        hardpoints=optimized_rows,
        initial_error_deg=initial_error,
        final_error_deg=final_error,
        actual_left_minus_right_deg=float(actual_delta),
        success=bool(result.success),
        message=str(result.message),
        applied_values={
            name: _get_optimization_variable(optimized_rows, name)
            for name in variable_names
        },
    )


def set_pitman_x_position(rows: list[SteeringHardpointRow], x_position: float) -> None:
    """Move pitman pivot and outputs together along vehicle X."""
    pivot, output = _pitman_rows(rows)
    snapshot = _row_snapshot(rows)
    delta = float(x_position) - pivot.x
    pivot.x += delta
    output.x += delta
    try:
        hardpoints_from_rows(rows)
    except Exception:
        _restore_row_snapshot(rows, snapshot)
        raise


def set_pitman_arm_x_length(
    rows: list[SteeringHardpointRow],
    x_length: float,
) -> None:
    """Set signed pitman output X offset from the pivot."""
    pivot, output = _pitman_rows(rows)
    snapshot = _row_snapshot(rows)
    output.x = pivot.x + float(x_length)
    try:
        hardpoints_from_rows(rows)
    except Exception:
        _restore_row_snapshot(rows, snapshot)
        raise


def hardpoints_from_rows(
    rows: list[SteeringHardpointRow],
) -> TwoSegmentSteeringHardpoints3D:
    """Build 3D hardpoints from editable project rows."""
    return load_two_segment_steering_hardpoints_rows(
        [row.as_loader_row() for row in rows]
    )


def hardpoint_rows_from_csv(path: str | Path) -> list[SteeringHardpointRow]:
    """Load editable hardpoint rows from a symmetric/center steering CSV."""
    with Path(path).open(newline="", encoding="utf-8") as csv_file:
        rows = [
            SteeringHardpointRow(
                category=row["category"].strip().lower(),
                name=row["name"].strip().lower(),
                x=float(row["x"]),
                y=float(row["y"]),
                z=float(row["z"]),
            )
            for row in csv.DictReader(csv_file)
        ]
    hardpoints_from_rows(rows)
    return rows


def save_hardpoint_rows_csv(
    rows: list[SteeringHardpointRow],
    path: str | Path,
) -> None:
    """Save editable hardpoint rows to the steering CSV format."""
    hardpoints_from_rows(rows)
    with Path(path).open("w", newline="", encoding="utf-8") as csv_file:
        fieldnames = ("category", "name", "x", "y", "z")
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row.as_loader_row())


def available_steering_outputs() -> tuple[str, ...]:
    """Return all scalar outputs available to tables and curve plots."""
    return _available_steering_outputs()


def curve_specs_for_plot(
    curves: list[SteeringCurve],
    selected_x_output: str,
    selected_y_output: str,
    selected_label: str,
) -> list[tuple[str, str, str]]:
    """Return saved curve specs, or a live preview spec if none are saved."""
    if curves:
        return [(curve.x_output, curve.y_output, curve.label) for curve in curves]
    label = selected_label.strip() or f"{selected_y_output} preview"
    return [(selected_x_output, selected_y_output, label)]


def solve_steering_project(
    project: SteeringProject,
    *,
    include_limits: bool = True,
) -> tuple[TwoSegmentSteeringSolution, dict[str, float]]:
    """Solve the current project state."""
    hardpoints = hardpoints_from_rows(project.hardpoints)
    if project.input_mode == "pitman_angle":
        solution = solve_two_segment_steering(hardpoints, project.input_value)
    elif project.input_mode == "left_wheel_angle":
        solution = solve_two_segment_from_left_wheel_angle(
            hardpoints,
            project.input_value,
        )
    elif project.input_mode == "right_wheel_angle":
        solution = solve_two_segment_from_right_wheel_angle(
            hardpoints,
            project.input_value,
        )
    else:
        raise ValueError(f"Unknown steering input mode '{project.input_mode}'")
    limit_outputs = steering_limit_outputs(hardpoints) if include_limits else None
    return solution, outputs_from_solution(
        solution,
        project.input_value,
        limit_outputs,
        wheelbase=project.wheelbase,
    )


def sweep_steering_project(
    project: SteeringProject,
    *,
    skip_unreachable: bool = False,
) -> list[dict[str, float]]:
    """Sweep the selected input mode over the project sweep range."""
    if project.sweep_step <= 0.0:
        raise ValueError("sweep_step must be positive")
    values = []
    current = project.sweep_min
    while current <= project.sweep_max + project.sweep_step * 1e-9:
        values.append(current)
        current += project.sweep_step
    rows = []
    limit_outputs = steering_limit_outputs(hardpoints_from_rows(project.hardpoints))
    for value in values:
        try:
            _, outputs = solve_steering_project(
                replace(project, input_value=value),
                include_limits=False,
            )
        except ValueError as exc:
            if skip_unreachable and _is_unreachable_solve_error(exc):
                continue
            raise
        outputs.update(limit_outputs)
        rows.append(outputs)
    return rows


def project_to_dict(project: SteeringProject) -> dict[str, Any]:
    """Convert a project to JSON-serializable data."""
    return {
        "name": project.name,
        "hardpoints": [asdict(row) for row in project.hardpoints],
        "input_mode": project.input_mode,
        "input_value": project.input_value,
        "sweep_min": project.sweep_min,
        "sweep_max": project.sweep_max,
        "sweep_step": project.sweep_step,
        "wheel_radius": project.wheel_radius,
        "wheel_width": project.wheel_width,
        "wheelbase": project.wheelbase,
        "curves": [asdict(curve) for curve in project.curves],
    }


def project_from_dict(data: dict[str, Any]) -> SteeringProject:
    """Create a project from JSON data."""
    return SteeringProject(
        name=str(data.get("name", "Untitled steering project")),
        hardpoints=[SteeringHardpointRow(**row) for row in data["hardpoints"]],
        input_mode=str(data.get("input_mode", "pitman_angle")),
        input_value=float(data.get("input_value", 0.0)),
        sweep_min=float(data.get("sweep_min", -20.0)),
        sweep_max=float(data.get("sweep_max", 20.0)),
        sweep_step=float(data.get("sweep_step", 2.0)),
        wheel_radius=float(data.get("wheel_radius", 180.0)),
        wheel_width=float(data.get("wheel_width", 120.0)),
        wheelbase=float(data.get("wheelbase", 2800.0)),
        curves=[SteeringCurve(**curve) for curve in data.get("curves", [])],
    )


def save_steering_project(project: SteeringProject, path: str | Path) -> None:
    """Save a steering project JSON file."""
    Path(path).write_text(
        json.dumps(project_to_dict(project), indent=2),
        encoding="utf-8",
    )


def load_steering_project(path: str | Path) -> SteeringProject:
    """Load a steering project JSON file."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return project_from_dict(data)
