"""
Workbench model for the two-segment steering GUI.
"""

from __future__ import annotations

import csv
import json
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any

from kinematics.steering.csv_loader import load_two_segment_steering_hardpoints_rows
from kinematics.steering.geometry import (
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
)
from kinematics.steering.limits import steering_limit_outputs
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


@dataclass(frozen=True)
class ParsedFloatEntry:
    """Result of parsing a live GUI numeric entry."""

    value: float
    is_valid: bool
    is_complete: bool


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


def pitman_x_position(rows: list[SteeringHardpointRow]) -> float:
    """Return the current center pitman pivot X position."""
    pivot, _output = _pitman_rows(rows)
    return pivot.x


def pitman_arm_x_length(rows: list[SteeringHardpointRow]) -> float:
    """Return signed pitman output X offset from the pivot."""
    pivot, output = _pitman_rows(rows)
    return output.x - pivot.x


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
    return solution, outputs_from_solution(solution, project.input_value, limit_outputs)


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
