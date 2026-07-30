"""
Scalar output helpers for the steering workbench.
"""

from __future__ import annotations

import math

from suspension_kinematics.steering.geometry import (
    ThreeSegmentSteeringSolution,
    TwoSegmentSteeringSolution,
    Vec2,
    Vec3,
)

STEERING_OUTPUT_NAMES = (
    "input_value",
    "pitman_angle_deg",
    "pinion_angle_deg",
    "rack_displacement_mm",
    "left_bellcrank_angle_deg",
    "right_bellcrank_angle_deg",
    "left_wheel_angle_deg",
    "right_wheel_angle_deg",
    "left_minus_right_deg",
    "ackermann_rate_pct",
    "max_left_turn_left_wheel_angle_deg",
    "max_left_turn_right_wheel_angle_deg",
    "max_right_turn_left_wheel_angle_deg",
    "max_right_turn_right_wheel_angle_deg",
    "left_wheel_center_x",
    "left_wheel_center_y",
    "left_wheel_center_z",
    "right_wheel_center_x",
    "right_wheel_center_y",
    "right_wheel_center_z",
    "left_tie_rod_pickup_x",
    "left_tie_rod_pickup_y",
    "left_tie_rod_pickup_z",
    "right_tie_rod_pickup_x",
    "right_tie_rod_pickup_y",
    "right_tie_rod_pickup_z",
    "pitman_left_output_x",
    "pitman_left_output_y",
    "pitman_left_output_z",
    "pitman_right_output_x",
    "pitman_right_output_y",
    "pitman_right_output_z",
    "center_link_residual",
    "left_tie_rod_residual",
    "right_tie_rod_residual",
    "max_abs_tie_rod_residual",
    "converged",
    "nfev",
)


def _optional_point_component(
    point_2d: Vec2,
    point_3d: Vec3 | None,
    index: int,
) -> float:
    if point_3d is not None:
        return float(point_3d[index])
    if index < 2:
        return float(point_2d[index])
    return 0.0


def _ackermann_rate_pct(
    solution: TwoSegmentSteeringSolution | ThreeSegmentSteeringSolution,
    wheelbase: float | None,
) -> float:
    if wheelbase is None or wheelbase <= 0.0:
        return 0.0
    actual_ackerman = solution.right_wheel_angle_deg - solution.left_wheel_angle_deg
    if abs(actual_ackerman) <= 1e-9:
        return 0.0
    inner_angle_deg = max(
        abs(solution.left_wheel_angle_deg),
        abs(solution.right_wheel_angle_deg),
    )
    if inner_angle_deg <= 1e-9:
        return 0.0
    track = abs(float(solution.right_wheel_center[1] - solution.left_wheel_center[1]))
    if track <= 0.0:
        return 0.0
    inner_rad = math.radians(inner_angle_deg)
    radius_to_inner = wheelbase / math.tan(inner_rad)
    ideal_outer_rad = math.atan2(wheelbase, radius_to_inner + track)
    ideal_outer_angle_deg = abs(math.degrees(ideal_outer_rad))
    ideal_ackerman = math.copysign(
        inner_angle_deg - ideal_outer_angle_deg,
        actual_ackerman,
    )
    if abs(ideal_ackerman) <= 1e-9:
        return 0.0
    return 100.0 * actual_ackerman / ideal_ackerman


def available_steering_outputs() -> tuple[str, ...]:
    """Return all scalar outputs available to tables and curve plots."""
    return STEERING_OUTPUT_NAMES


def outputs_from_solution(
    solution: TwoSegmentSteeringSolution,
    input_value: float,
    extra_outputs: dict[str, float] | None = None,
    wheelbase: float | None = None,
) -> dict[str, float]:
    """Flatten a steering solution into scalar GUI outputs."""
    outputs = {
        "input_value": float(input_value),
        "pitman_angle_deg": solution.pitman_angle_deg,
        "pinion_angle_deg": 0.0,
        "rack_displacement_mm": 0.0,
        "left_bellcrank_angle_deg": 0.0,
        "right_bellcrank_angle_deg": 0.0,
        "left_wheel_angle_deg": solution.left_wheel_angle_deg,
        "right_wheel_angle_deg": solution.right_wheel_angle_deg,
        "left_minus_right_deg": (
            solution.left_wheel_angle_deg - solution.right_wheel_angle_deg
        ),
        "ackermann_rate_pct": _ackermann_rate_pct(solution, wheelbase),
        "left_wheel_center_x": _optional_point_component(
            solution.left_wheel_center,
            solution.left_wheel_center_3d,
            0,
        ),
        "left_wheel_center_y": _optional_point_component(
            solution.left_wheel_center,
            solution.left_wheel_center_3d,
            1,
        ),
        "left_wheel_center_z": _optional_point_component(
            solution.left_wheel_center,
            solution.left_wheel_center_3d,
            2,
        ),
        "right_wheel_center_x": _optional_point_component(
            solution.right_wheel_center,
            solution.right_wheel_center_3d,
            0,
        ),
        "right_wheel_center_y": _optional_point_component(
            solution.right_wheel_center,
            solution.right_wheel_center_3d,
            1,
        ),
        "right_wheel_center_z": _optional_point_component(
            solution.right_wheel_center,
            solution.right_wheel_center_3d,
            2,
        ),
        "left_tie_rod_pickup_x": _optional_point_component(
            solution.left_tie_rod_pickup,
            solution.left_tie_rod_pickup_3d,
            0,
        ),
        "left_tie_rod_pickup_y": _optional_point_component(
            solution.left_tie_rod_pickup,
            solution.left_tie_rod_pickup_3d,
            1,
        ),
        "left_tie_rod_pickup_z": _optional_point_component(
            solution.left_tie_rod_pickup,
            solution.left_tie_rod_pickup_3d,
            2,
        ),
        "right_tie_rod_pickup_x": _optional_point_component(
            solution.right_tie_rod_pickup,
            solution.right_tie_rod_pickup_3d,
            0,
        ),
        "right_tie_rod_pickup_y": _optional_point_component(
            solution.right_tie_rod_pickup,
            solution.right_tie_rod_pickup_3d,
            1,
        ),
        "right_tie_rod_pickup_z": _optional_point_component(
            solution.right_tie_rod_pickup,
            solution.right_tie_rod_pickup_3d,
            2,
        ),
        "pitman_left_output_x": _optional_point_component(
            solution.pitman_left_output,
            solution.pitman_left_output_3d,
            0,
        ),
        "pitman_left_output_y": _optional_point_component(
            solution.pitman_left_output,
            solution.pitman_left_output_3d,
            1,
        ),
        "pitman_left_output_z": _optional_point_component(
            solution.pitman_left_output,
            solution.pitman_left_output_3d,
            2,
        ),
        "pitman_right_output_x": _optional_point_component(
            solution.pitman_right_output,
            solution.pitman_right_output_3d,
            0,
        ),
        "pitman_right_output_y": _optional_point_component(
            solution.pitman_right_output,
            solution.pitman_right_output_3d,
            1,
        ),
        "pitman_right_output_z": _optional_point_component(
            solution.pitman_right_output,
            solution.pitman_right_output_3d,
            2,
        ),
        "center_link_residual": 0.0,
        "left_tie_rod_residual": solution.left_tie_rod_residual,
        "right_tie_rod_residual": solution.right_tie_rod_residual,
        "max_abs_tie_rod_residual": solution.max_abs_tie_rod_residual,
        "converged": float(solution.converged),
        "nfev": float(solution.nfev),
    }
    if extra_outputs is not None:
        outputs.update(extra_outputs)
    return outputs


def outputs_from_three_segment_solution(
    solution: ThreeSegmentSteeringSolution,
    input_value: float,
    extra_outputs: dict[str, float] | None = None,
    wheelbase: float | None = None,
) -> dict[str, float]:
    """Flatten a three-segment steering solution into scalar GUI outputs."""
    outputs = {
        "input_value": float(input_value),
        "pitman_angle_deg": 0.0,
        "pinion_angle_deg": 0.0,
        "rack_displacement_mm": 0.0,
        "left_bellcrank_angle_deg": solution.left_bellcrank_angle_deg,
        "right_bellcrank_angle_deg": solution.right_bellcrank_angle_deg,
        "left_wheel_angle_deg": solution.left_wheel_angle_deg,
        "right_wheel_angle_deg": solution.right_wheel_angle_deg,
        "left_minus_right_deg": (
            solution.left_wheel_angle_deg - solution.right_wheel_angle_deg
        ),
        "ackermann_rate_pct": _ackermann_rate_pct(solution, wheelbase),
        "left_wheel_center_x": _optional_point_component(
            solution.left_wheel_center,
            solution.left_wheel_center_3d,
            0,
        ),
        "left_wheel_center_y": _optional_point_component(
            solution.left_wheel_center,
            solution.left_wheel_center_3d,
            1,
        ),
        "left_wheel_center_z": _optional_point_component(
            solution.left_wheel_center,
            solution.left_wheel_center_3d,
            2,
        ),
        "right_wheel_center_x": _optional_point_component(
            solution.right_wheel_center,
            solution.right_wheel_center_3d,
            0,
        ),
        "right_wheel_center_y": _optional_point_component(
            solution.right_wheel_center,
            solution.right_wheel_center_3d,
            1,
        ),
        "right_wheel_center_z": _optional_point_component(
            solution.right_wheel_center,
            solution.right_wheel_center_3d,
            2,
        ),
        "left_tie_rod_pickup_x": _optional_point_component(
            solution.left_tie_rod_pickup,
            solution.left_tie_rod_pickup_3d,
            0,
        ),
        "left_tie_rod_pickup_y": _optional_point_component(
            solution.left_tie_rod_pickup,
            solution.left_tie_rod_pickup_3d,
            1,
        ),
        "left_tie_rod_pickup_z": _optional_point_component(
            solution.left_tie_rod_pickup,
            solution.left_tie_rod_pickup_3d,
            2,
        ),
        "right_tie_rod_pickup_x": _optional_point_component(
            solution.right_tie_rod_pickup,
            solution.right_tie_rod_pickup_3d,
            0,
        ),
        "right_tie_rod_pickup_y": _optional_point_component(
            solution.right_tie_rod_pickup,
            solution.right_tie_rod_pickup_3d,
            1,
        ),
        "right_tie_rod_pickup_z": _optional_point_component(
            solution.right_tie_rod_pickup,
            solution.right_tie_rod_pickup_3d,
            2,
        ),
        "pitman_left_output_x": 0.0,
        "pitman_left_output_y": 0.0,
        "pitman_left_output_z": 0.0,
        "pitman_right_output_x": 0.0,
        "pitman_right_output_y": 0.0,
        "pitman_right_output_z": 0.0,
        "center_link_residual": solution.center_link_residual,
        "left_tie_rod_residual": solution.left_tie_rod_residual,
        "right_tie_rod_residual": solution.right_tie_rod_residual,
        "max_abs_tie_rod_residual": solution.max_abs_link_residual,
        "converged": float(solution.converged),
        "nfev": float(solution.nfev),
    }
    if extra_outputs is not None:
        outputs.update(extra_outputs)
    return outputs
