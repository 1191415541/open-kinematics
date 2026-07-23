"""
Workbench model for the two-segment steering GUI.
"""

from __future__ import annotations

import csv
import json
import threading
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np
from scipy.optimize import least_squares

from kinematics.gui.common import (
    OptimizationCancelledError,
    parse_float_entry,
    raise_if_cancelled,
)
from kinematics.gui.project import build_project_document, write_project_document
from kinematics.steering.csv_loader import load_two_segment_steering_hardpoints_rows
from kinematics.steering.geometry import (
    BellcrankGeometry2D,
    BellcrankHardpoints3D,
    ThreeSegmentSteeringGeometry,
    ThreeSegmentSteeringHardpoints3D,
    ThreeSegmentSteeringSolution,
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
    WheelSteeringGeometry2D,
    WheelSteeringHardpoints3D,
)
from kinematics.steering.limits import (
    estimate_rack_and_pinion_steering_limits,
    estimate_two_segment_steering_limits,
    rack_and_pinion_steering_limit_outputs,
    steering_limit_outputs,
    three_segment_steering_limit_outputs,
)
from kinematics.steering.outputs import (
    available_steering_outputs as _available_steering_outputs,
)
from kinematics.steering.outputs import (
    outputs_from_solution,
    outputs_from_three_segment_solution,
)
from kinematics.steering.three_segment import (
    solve_three_segment_from_left_wheel_angle,
    solve_three_segment_from_right_bellcrank_angle,
    solve_three_segment_from_right_wheel_angle,
    solve_three_segment_steering,
    solve_three_segment_steering_3d_analytic,
)
from kinematics.steering.two_segment import (
    pinion_angle_from_rack_displacement,
    rack_displacement_from_pinion_angle,
    solve_two_segment_from_left_wheel_angle_3d_analytic,
    solve_two_segment_from_right_wheel_angle_3d_analytic,
    solve_two_segment_rack_and_pinion_3d_analytic,
    solve_two_segment_steering_3d_analytic,
)

RACK_AND_PINION_INPUT_MODES = ("pinion_angle", "rack_displacement")
TWO_SEGMENT_INPUT_MODES = (
    "pitman_angle",
    "left_wheel_angle",
    "right_wheel_angle",
    *RACK_AND_PINION_INPUT_MODES,
)
THREE_SEGMENT_INPUT_MODES = (
    "left_bellcrank_angle",
    "right_bellcrank_angle",
    "left_wheel_angle",
    "right_wheel_angle",
)
INPUT_MODES = TWO_SEGMENT_INPUT_MODES
LINKAGE_TYPES = ("two_segment", "three_segment")
ThreeSegmentGeometryInput = (
    ThreeSegmentSteeringGeometry | ThreeSegmentSteeringHardpoints3D
)
UNREACHABLE_SOLVE_PREFIXES = (
    "No valid steering arm position",
    "No valid pitman arm position",
    "No valid three-segment state",
)
OPTIMIZATION_VARIABLES = (
    "pitman_x",
    "pitman_arm_x_length",
    "tie_rod_outer_x",
    "tie_rod_outer_y",
    "tie_rod_inner_x",
    "tie_rod_inner_y",
)

__all__ = [
    "INPUT_MODES",
    "LINKAGE_TYPES",
    "OPTIMIZATION_VARIABLES",
    "OptimizationCancelledError",
    "RACK_AND_PINION_INPUT_MODES",
    "SliderLimits",
    "SteeringCurve",
    "SteeringHardpointRow",
    "SteeringOptimizationResult",
    "SteeringProject",
    "THREE_SEGMENT_INPUT_MODES",
    "TWO_SEGMENT_INPUT_MODES",
    "available_steering_outputs",
    "copy_hardpoint_rows",
    "curve_specs_for_plot",
    "default_hardpoint_rows",
    "default_steering_project",
    "hardpoint_rows_from_csv",
    "hardpoints_from_rows",
    "input_angle_slider_limits",
    "load_steering_project",
    "optimize_steering_hardpoints",
    "parse_float_entry",
    "pitman_angle_slider_limits",
    "pitman_arm_x_length",
    "pitman_x_position",
    "save_hardpoint_rows_csv",
    "save_steering_project",
    "set_pitman_arm_x_length",
    "set_pitman_x_position",
    "solve_steering_project",
    "steering_project_limit_outputs",
    "sweep_steering_project",
    "three_segment_hardpoints_from_rows",
    "three_segment_geometry_from_rows",
]


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


def default_hardpoint_rows(
    linkage_type: str = "two_segment",
) -> list[SteeringHardpointRow]:
    """Return a practical symmetric steering hardpoint set."""
    if linkage_type == "three_segment":
        return [
            SteeringHardpointRow(
                "symmetric",
                "wheel_kingpin_lower",
                0.0,
                -500.0,
                280.0,
            ),
            SteeringHardpointRow(
                "symmetric",
                "wheel_kingpin_upper",
                0.0,
                -500.0,
                340.0,
            ),
            SteeringHardpointRow("symmetric", "wheel_center", 60.0, -520.0, 320.0),
            SteeringHardpointRow(
                "symmetric",
                "wheel_tie_rod_pickup",
                -180.0,
                -420.0,
                280.0,
            ),
            SteeringHardpointRow(
                "symmetric",
                "bellcrank_pivot",
                -260.0,
                -320.0,
                300.0,
            ),
            SteeringHardpointRow(
                "symmetric",
                "bellcrank_center_link_pickup",
                -460.0,
                -300.0,
                300.0,
            ),
            SteeringHardpointRow(
                "symmetric",
                "bellcrank_tie_rod_pickup",
                -300.0,
                -300.0,
                300.0,
            ),
        ]
    if linkage_type != "two_segment":
        raise ValueError(f"Unknown steering linkage type '{linkage_type}'")
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
    linkage_type: str = "two_segment"
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
    pinion_pitch_radius_mm: float = 15.0
    curves: list[SteeringCurve] = field(default_factory=list)


def default_steering_project(linkage_type: str = "two_segment") -> SteeringProject:
    """Create a default steering project."""
    if linkage_type == "two_segment":
        return SteeringProject()
    if linkage_type == "three_segment":
        return SteeringProject(
            linkage_type="three_segment",
            hardpoints=default_hardpoint_rows("three_segment"),
            input_mode="left_bellcrank_angle",
        )
    raise ValueError(f"Unknown steering linkage type '{linkage_type}'")


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
        return solve_two_segment_from_left_wheel_angle_3d_analytic(
            hardpoints,
            inner_wheel_angle_deg,
        )
    if inner_wheel == "right":
        return solve_two_segment_from_right_wheel_angle_3d_analytic(
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
    linkage_type: str = "two_segment",
    pinion_pitch_radius_mm: float = 15.0,
) -> SliderLimits:
    """Return slider limits for the selected steering input mode."""
    if linkage_type == "three_segment":
        return _three_segment_input_angle_slider_limits(rows, input_mode)
    hardpoints = hardpoints_from_rows(rows)
    if input_mode in RACK_AND_PINION_INPUT_MODES:
        limits = estimate_rack_and_pinion_steering_limits(hardpoints)
        if input_mode == "rack_displacement":
            return SliderLimits(
                minimum=limits.minimum_displacement_mm,
                maximum=limits.maximum_displacement_mm,
            )
        return SliderLimits(
            minimum=pinion_angle_from_rack_displacement(
                limits.minimum_displacement_mm,
                pinion_pitch_radius_mm,
            ),
            maximum=pinion_angle_from_rack_displacement(
                limits.maximum_displacement_mm,
                pinion_pitch_radius_mm,
            ),
        )
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


def _three_segment_input_angle_slider_limits(
    rows: list[SteeringHardpointRow],
    input_mode: str,
) -> SliderLimits:
    hardpoints = three_segment_hardpoints_from_rows(rows)
    if input_mode not in THREE_SEGMENT_INPUT_MODES:
        raise ValueError(f"Unknown steering input mode '{input_mode}'")
    output_name = _three_segment_output_for_input_mode(input_mode)
    if input_mode in {"left_wheel_angle", "right_wheel_angle"}:
        values = _three_segment_monotonic_output_values(hardpoints, output_name)
    else:
        samples = _three_segment_current_branch_samples(hardpoints)
        values = [float(getattr(solution, output_name)) for solution in samples]
    return SliderLimits(minimum=min(values), maximum=max(values))


def _three_segment_monotonic_output_values(
    geometry: ThreeSegmentGeometryInput,
    output_name: str,
) -> list[float]:
    zero = _solve_three_segment_forward(
        geometry,
        0.0,
        (0.0, 0.0, 0.0),
    )
    values = [float(getattr(zero, output_name))]
    negative_values = _walk_three_segment_monotonic_output(
        geometry,
        zero,
        output_name=output_name,
        direction=-1.0,
    )
    positive_values = _walk_three_segment_monotonic_output(
        geometry,
        zero,
        output_name=output_name,
        direction=1.0,
    )
    values[:0] = list(reversed(negative_values))
    values.extend(positive_values)
    return values


def _walk_three_segment_monotonic_output(
    geometry: ThreeSegmentGeometryInput,
    start: ThreeSegmentSteeringSolution,
    *,
    output_name: str,
    direction: float,
    step_deg: float = 1.0,
    max_abs_left_bellcrank_angle_deg: float = 180.0,
) -> list[float]:
    values: list[float] = []
    last = start
    last_output = float(getattr(start, output_name))
    trend = 0.0
    while True:
        angle = last.left_bellcrank_angle_deg + direction * step_deg
        if abs(angle) > max_abs_left_bellcrank_angle_deg:
            break
        solution = _solve_three_segment_forward(
            geometry,
            angle,
            (
                last.right_bellcrank_angle_deg,
                last.left_wheel_angle_deg,
                last.right_wheel_angle_deg,
            ),
        )
        if not solution.converged:
            break
        output = float(getattr(solution, output_name))
        delta = output - last_output
        if abs(delta) <= 1e-9:
            break
        current_trend = 1.0 if delta > 0.0 else -1.0
        if trend != 0.0 and current_trend != trend:
            break
        trend = current_trend
        values.append(output)
        last = solution
        last_output = output
    return values


def _three_segment_current_branch_samples(
    geometry: ThreeSegmentGeometryInput,
    *,
    step_deg: float = 1.0,
    max_abs_left_bellcrank_angle_deg: float = 180.0,
) -> list[ThreeSegmentSteeringSolution]:
    zero = _solve_three_segment_forward(
        geometry,
        0.0,
        (0.0, 0.0, 0.0),
    )
    samples = [zero]
    samples[:0] = list(
        reversed(
            _walk_three_segment_branch(
                geometry,
                zero,
                direction=-1.0,
                step_deg=step_deg,
                max_abs_left_bellcrank_angle_deg=max_abs_left_bellcrank_angle_deg,
            )
        )
    )
    samples.extend(
        _walk_three_segment_branch(
            geometry,
            zero,
            direction=1.0,
            step_deg=step_deg,
            max_abs_left_bellcrank_angle_deg=max_abs_left_bellcrank_angle_deg,
        )
    )
    return samples


def _walk_three_segment_branch(
    geometry: ThreeSegmentGeometryInput,
    start: ThreeSegmentSteeringSolution,
    *,
    direction: float,
    step_deg: float,
    max_abs_left_bellcrank_angle_deg: float,
) -> list[ThreeSegmentSteeringSolution]:
    samples: list[ThreeSegmentSteeringSolution] = []
    last = start
    while True:
        angle = last.left_bellcrank_angle_deg + direction * step_deg
        if abs(angle) > max_abs_left_bellcrank_angle_deg:
            break
        solution = _solve_three_segment_forward(
            geometry,
            angle,
            (
                last.right_bellcrank_angle_deg,
                last.left_wheel_angle_deg,
                last.right_wheel_angle_deg,
            ),
        )
        if not solution.converged:
            break
        samples.append(solution)
        last = solution
    return samples


def _solve_three_segment_forward(
    geometry: ThreeSegmentGeometryInput,
    left_bellcrank_angle_deg: float,
    guess: tuple[float, float, float],
) -> ThreeSegmentSteeringSolution:
    if isinstance(geometry, ThreeSegmentSteeringHardpoints3D):
        return solve_three_segment_steering_3d_analytic(
            geometry,
            left_bellcrank_angle_deg,
            guess,
        )
    return solve_three_segment_steering(
        geometry,
        left_bellcrank_angle_deg,
        guess,
    )


def optimize_steering_hardpoints(
    rows: list[SteeringHardpointRow],
    *,
    inner_wheel: str,
    inner_wheel_angle_deg: float,
    target_left_minus_right_deg: float,
    variable_names: tuple[str, ...],
    variable_delta_limit: float,
    cancel_event: threading.Event | None = None,
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
        raise_if_cancelled(cancel_event)
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

    raise_if_cancelled(cancel_event)
    initial_error = float(abs(residual(x0)[0]))
    result = least_squares(residual, x0, bounds=(lower, upper), method="trf")
    raise_if_cancelled(cancel_event)
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


def _row_vec2(rows: list[SteeringHardpointRow], name: str) -> np.ndarray:
    row = _required_hardpoint_row(rows, name)
    return np.array([row.x, row.y], dtype=np.float64)


def _row_vec3(rows: list[SteeringHardpointRow], name: str) -> np.ndarray:
    row = _required_hardpoint_row(rows, name)
    return np.array([row.x, row.y, row.z], dtype=np.float64)


def _mirror_vec2(point: np.ndarray) -> np.ndarray:
    mirrored = point.copy()
    mirrored[1] *= -1.0
    return mirrored


def _mirror_vec3(point: np.ndarray) -> np.ndarray:
    mirrored = point.copy()
    mirrored[1] *= -1.0
    return mirrored


def _mirror_axis_vec3(axis: np.ndarray) -> np.ndarray:
    mirrored = axis.copy()
    mirrored[1] *= -1.0
    return mirrored


def three_segment_geometry_from_rows(
    rows: list[SteeringHardpointRow],
) -> ThreeSegmentSteeringGeometry:
    """Build three-segment 2D geometry from editable project rows."""
    left_kingpin = _row_vec2(rows, "wheel_kingpin_lower")
    left_wheel_center = _row_vec2(rows, "wheel_center")
    left_wheel_tie = _row_vec2(rows, "wheel_tie_rod_pickup")
    left_bell_pivot = _row_vec2(rows, "bellcrank_pivot")
    left_center_link = _row_vec2(rows, "bellcrank_center_link_pickup")
    left_bell_tie = _row_vec2(rows, "bellcrank_tie_rod_pickup")
    return ThreeSegmentSteeringGeometry(
        left_wheel=WheelSteeringGeometry2D(
            kingpin=left_kingpin,
            wheel_center=left_wheel_center,
            tie_rod_pickup=left_wheel_tie,
        ),
        right_wheel=WheelSteeringGeometry2D(
            kingpin=_mirror_vec2(left_kingpin),
            wheel_center=_mirror_vec2(left_wheel_center),
            tie_rod_pickup=_mirror_vec2(left_wheel_tie),
        ),
        left_bellcrank=BellcrankGeometry2D(
            pivot=left_bell_pivot,
            center_link_pickup=left_center_link,
            tie_rod_pickup=left_bell_tie,
        ),
        right_bellcrank=BellcrankGeometry2D(
            pivot=_mirror_vec2(left_bell_pivot),
            center_link_pickup=_mirror_vec2(left_center_link),
            tie_rod_pickup=_mirror_vec2(left_bell_tie),
        ),
    )


def three_segment_hardpoints_from_rows(
    rows: list[SteeringHardpointRow],
) -> ThreeSegmentSteeringHardpoints3D:
    """Build 3D three-segment hardpoints from editable project rows."""
    left_kingpin_lower = _row_vec3(rows, "wheel_kingpin_lower")
    left_kingpin_upper = _row_vec3(rows, "wheel_kingpin_upper")
    left_wheel_center = _row_vec3(rows, "wheel_center")
    left_wheel_tie = _row_vec3(rows, "wheel_tie_rod_pickup")
    left_bell_pivot = _row_vec3(rows, "bellcrank_pivot")
    left_center_link = _row_vec3(rows, "bellcrank_center_link_pickup")
    left_bell_tie = _row_vec3(rows, "bellcrank_tie_rod_pickup")
    left_axis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    return ThreeSegmentSteeringHardpoints3D(
        left_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=left_kingpin_lower,
            kingpin_upper=left_kingpin_upper,
            wheel_center=left_wheel_center,
            tie_rod_pickup=left_wheel_tie,
        ),
        right_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=_mirror_vec3(left_kingpin_lower),
            kingpin_upper=_mirror_vec3(left_kingpin_upper),
            wheel_center=_mirror_vec3(left_wheel_center),
            tie_rod_pickup=_mirror_vec3(left_wheel_tie),
        ),
        left_bellcrank=BellcrankHardpoints3D(
            pivot=left_bell_pivot,
            center_link_pickup=left_center_link,
            tie_rod_pickup=left_bell_tie,
            axis=left_axis,
        ),
        right_bellcrank=BellcrankHardpoints3D(
            pivot=_mirror_vec3(left_bell_pivot),
            center_link_pickup=_mirror_vec3(left_center_link),
            tie_rod_pickup=_mirror_vec3(left_bell_tie),
            axis=_mirror_axis_vec3(left_axis),
        ),
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


def _three_segment_output_for_input_mode(input_mode: str) -> str:
    if input_mode == "left_bellcrank_angle":
        return "left_bellcrank_angle_deg"
    if input_mode == "right_bellcrank_angle":
        return "right_bellcrank_angle_deg"
    if input_mode == "left_wheel_angle":
        return "left_wheel_angle_deg"
    if input_mode == "right_wheel_angle":
        return "right_wheel_angle_deg"
    raise ValueError(f"Unknown steering input mode '{input_mode}'")


def solve_three_segment_project(
    project: SteeringProject,
    previous_state: ThreeSegmentSteeringSolution | None = None,
) -> ThreeSegmentSteeringSolution:
    """Solve a three-segment steering project state."""
    hardpoints = three_segment_hardpoints_from_rows(project.hardpoints)
    initial_left_bellcrank_guess = (
        0.0 if previous_state is None else previous_state.left_bellcrank_angle_deg
    )
    initial_guess = (
        (0.0, 0.0, 0.0)
        if previous_state is None
        else (
            previous_state.right_bellcrank_angle_deg,
            previous_state.left_wheel_angle_deg,
            previous_state.right_wheel_angle_deg,
        )
    )
    if project.input_mode == "left_bellcrank_angle":
        return solve_three_segment_steering_3d_analytic(
            hardpoints,
            project.input_value,
            initial_guess,
        )
    if project.input_mode == "right_bellcrank_angle":
        return solve_three_segment_from_right_bellcrank_angle(
            hardpoints,
            project.input_value,
            initial_left_bellcrank_guess,
        )
    if project.input_mode == "left_wheel_angle":
        return solve_three_segment_from_left_wheel_angle(
            hardpoints,
            project.input_value,
            initial_left_bellcrank_guess,
        )
    if project.input_mode == "right_wheel_angle":
        return solve_three_segment_from_right_wheel_angle(
            hardpoints,
            project.input_value,
            initial_left_bellcrank_guess,
        )
    raise ValueError(f"Unknown steering input mode '{project.input_mode}'")


def steering_project_limit_outputs(project: SteeringProject) -> dict[str, float]:
    """Return steering travel limits for the project's selected actuator."""
    if project.linkage_type == "three_segment":
        return three_segment_steering_limit_outputs(
            three_segment_hardpoints_from_rows(project.hardpoints)
        )
    hardpoints = hardpoints_from_rows(project.hardpoints)
    if project.input_mode in RACK_AND_PINION_INPUT_MODES:
        return rack_and_pinion_steering_limit_outputs(hardpoints)
    return steering_limit_outputs(hardpoints)


def solve_steering_project(
    project: SteeringProject,
    *,
    include_limits: bool = True,
    previous_state: ThreeSegmentSteeringSolution | None = None,
) -> tuple[TwoSegmentSteeringSolution | ThreeSegmentSteeringSolution, dict[str, float]]:
    """Solve the current project state."""
    if project.linkage_type == "three_segment":
        hardpoints = three_segment_hardpoints_from_rows(project.hardpoints)
        solution = solve_three_segment_project(project, previous_state)
        limit_outputs = (
            steering_project_limit_outputs(project) if include_limits else None
        )
        return solution, outputs_from_three_segment_solution(
            solution,
            project.input_value,
            limit_outputs,
            wheelbase=project.wheelbase,
        )
    hardpoints = hardpoints_from_rows(project.hardpoints)
    actuator_outputs: dict[str, float] = {}
    if project.input_mode == "pitman_angle":
        solution = solve_two_segment_steering_3d_analytic(
            hardpoints,
            project.input_value,
        )
    elif project.input_mode == "left_wheel_angle":
        solution = solve_two_segment_from_left_wheel_angle_3d_analytic(
            hardpoints,
            project.input_value,
        )
    elif project.input_mode == "right_wheel_angle":
        solution = solve_two_segment_from_right_wheel_angle_3d_analytic(
            hardpoints,
            project.input_value,
        )
    elif project.input_mode == "pinion_angle":
        rack_displacement = rack_displacement_from_pinion_angle(
            project.input_value,
            project.pinion_pitch_radius_mm,
        )
        solution = solve_two_segment_rack_and_pinion_3d_analytic(
            hardpoints,
            rack_displacement,
        )
        actuator_outputs = {
            "pinion_angle_deg": float(project.input_value),
            "rack_displacement_mm": rack_displacement,
        }
    elif project.input_mode == "rack_displacement":
        pinion_angle = pinion_angle_from_rack_displacement(
            project.input_value,
            project.pinion_pitch_radius_mm,
        )
        solution = solve_two_segment_rack_and_pinion_3d_analytic(
            hardpoints,
            project.input_value,
        )
        actuator_outputs = {
            "pinion_angle_deg": pinion_angle,
            "rack_displacement_mm": float(project.input_value),
        }
    else:
        raise ValueError(f"Unknown steering input mode '{project.input_mode}'")
    limit_outputs = steering_project_limit_outputs(project) if include_limits else None
    extra_outputs = dict(actuator_outputs)
    if limit_outputs is not None:
        extra_outputs.update(limit_outputs)
    return solution, outputs_from_solution(
        solution,
        project.input_value,
        extra_outputs or None,
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
    limit_outputs = steering_project_limit_outputs(project)
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
    """Convert a project to the shared GUI project JSON format."""
    return build_project_document(
        module="steering",
        system_type=project.linkage_type,
        name=project.name,
        hardpoints=[asdict(row) for row in project.hardpoints],
        parameters={
            "wheel_radius": project.wheel_radius,
            "wheel_width": project.wheel_width,
            "wheelbase": project.wheelbase,
            "pinion_pitch_radius_mm": project.pinion_pitch_radius_mm,
        },
        simulation={
            "input_mode": project.input_mode,
            "input_value": project.input_value,
            "sweep_min": project.sweep_min,
            "sweep_max": project.sweep_max,
            "sweep_step": project.sweep_step,
        },
        curves=[asdict(curve) for curve in project.curves],
    )


def project_from_dict(data: dict[str, Any]) -> SteeringProject:
    """Create a project from JSON data."""
    if data.get("module") == "steering":
        return _project_from_unified_dict(data)

    linkage_type = str(data.get("linkage_type", "two_segment"))
    return SteeringProject(
        name=str(data.get("name", "Untitled steering project")),
        linkage_type=linkage_type,
        hardpoints=[
            SteeringHardpointRow(**row)
            for row in data.get("hardpoints", default_hardpoint_rows(linkage_type))
        ],
        input_mode=str(
            data.get(
                "input_mode",
                "left_bellcrank_angle"
                if linkage_type == "three_segment"
                else "pitman_angle",
            )
        ),
        input_value=float(data.get("input_value", 0.0)),
        sweep_min=float(data.get("sweep_min", -20.0)),
        sweep_max=float(data.get("sweep_max", 20.0)),
        sweep_step=float(data.get("sweep_step", 2.0)),
        wheel_radius=float(data.get("wheel_radius", 180.0)),
        wheel_width=float(data.get("wheel_width", 120.0)),
        wheelbase=float(data.get("wheelbase", 2800.0)),
        pinion_pitch_radius_mm=float(data.get("pinion_pitch_radius_mm", 15.0)),
        curves=[SteeringCurve(**curve) for curve in data.get("curves", [])],
    )


def _project_from_unified_dict(data: dict[str, Any]) -> SteeringProject:
    linkage_type = str(data.get("system_type", "two_segment"))
    parameters = data.get("parameters", {})
    simulation = data.get("simulation", {})
    return SteeringProject(
        name=str(data.get("name", "Untitled steering project")),
        linkage_type=linkage_type,
        hardpoints=[
            SteeringHardpointRow(**row)
            for row in data.get("hardpoints", default_hardpoint_rows(linkage_type))
        ],
        input_mode=str(
            simulation.get(
                "input_mode",
                "left_bellcrank_angle"
                if linkage_type == "three_segment"
                else "pitman_angle",
            )
        ),
        input_value=float(simulation.get("input_value", 0.0)),
        sweep_min=float(simulation.get("sweep_min", -20.0)),
        sweep_max=float(simulation.get("sweep_max", 20.0)),
        sweep_step=float(simulation.get("sweep_step", 2.0)),
        wheel_radius=float(parameters.get("wheel_radius", 180.0)),
        wheel_width=float(parameters.get("wheel_width", 120.0)),
        wheelbase=float(parameters.get("wheelbase", 2800.0)),
        pinion_pitch_radius_mm=float(parameters.get("pinion_pitch_radius_mm", 15.0)),
        curves=[SteeringCurve(**curve) for curve in data.get("curves", [])],
    )


def save_steering_project(project: SteeringProject, path: str | Path) -> None:
    """Save a steering project JSON file."""
    write_project_document(project_to_dict(project), path)


def load_steering_project(path: str | Path) -> SteeringProject:
    """Load a steering project JSON file."""
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    return project_from_dict(data)
