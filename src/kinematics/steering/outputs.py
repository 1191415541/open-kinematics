"""
Scalar output helpers for the steering workbench.
"""

from __future__ import annotations

from kinematics.steering.geometry import TwoSegmentSteeringSolution

STEERING_OUTPUT_NAMES = (
    "input_value",
    "pitman_angle_deg",
    "left_wheel_angle_deg",
    "right_wheel_angle_deg",
    "left_minus_right_deg",
    "max_left_turn_left_wheel_angle_deg",
    "max_left_turn_right_wheel_angle_deg",
    "max_right_turn_left_wheel_angle_deg",
    "max_right_turn_right_wheel_angle_deg",
    "left_wheel_center_x",
    "left_wheel_center_y",
    "right_wheel_center_x",
    "right_wheel_center_y",
    "left_tie_rod_pickup_x",
    "left_tie_rod_pickup_y",
    "right_tie_rod_pickup_x",
    "right_tie_rod_pickup_y",
    "left_tie_rod_residual",
    "right_tie_rod_residual",
)


def available_steering_outputs() -> tuple[str, ...]:
    """Return all scalar outputs available to tables and curve plots."""
    return STEERING_OUTPUT_NAMES


def outputs_from_solution(
    solution: TwoSegmentSteeringSolution,
    input_value: float,
    extra_outputs: dict[str, float] | None = None,
) -> dict[str, float]:
    """Flatten a steering solution into scalar GUI outputs."""
    outputs = {
        "input_value": float(input_value),
        "pitman_angle_deg": solution.pitman_angle_deg,
        "left_wheel_angle_deg": solution.left_wheel_angle_deg,
        "right_wheel_angle_deg": solution.right_wheel_angle_deg,
        "left_minus_right_deg": (
            solution.left_wheel_angle_deg - solution.right_wheel_angle_deg
        ),
        "left_wheel_center_x": float(solution.left_wheel_center[0]),
        "left_wheel_center_y": float(solution.left_wheel_center[1]),
        "right_wheel_center_x": float(solution.right_wheel_center[0]),
        "right_wheel_center_y": float(solution.right_wheel_center[1]),
        "left_tie_rod_pickup_x": float(solution.left_tie_rod_pickup[0]),
        "left_tie_rod_pickup_y": float(solution.left_tie_rod_pickup[1]),
        "right_tie_rod_pickup_x": float(solution.right_tie_rod_pickup[0]),
        "right_tie_rod_pickup_y": float(solution.right_tie_rod_pickup[1]),
        "left_tie_rod_residual": solution.left_tie_rod_residual,
        "right_tie_rod_residual": solution.right_tie_rod_residual,
    }
    if extra_outputs is not None:
        outputs.update(extra_outputs)
    return outputs
