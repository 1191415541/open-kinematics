"""
Three-segment steering linkage solver for top-view 2D geometry.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
from scipy.optimize import OptimizeResult, least_squares

from kinematics.core.constants import EPS_GEOMETRIC
from kinematics.steering.geometry import (
    ThreeSegmentSteeringGeometry,
    ThreeSegmentSteeringSolution,
    Vec2,
    WheelSteeringGeometry2D,
    distance_2d,
    rotate_point_2d,
)


def _rotated_wheel_pickups(
    geometry: ThreeSegmentSteeringGeometry,
    left_wheel_angle_rad: float,
    right_wheel_angle_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    return (
        rotate_point_2d(
            geometry.left_wheel.tie_rod_pickup,
            geometry.left_wheel.kingpin,
            left_wheel_angle_rad,
        ),
        rotate_point_2d(
            geometry.right_wheel.tie_rod_pickup,
            geometry.right_wheel.kingpin,
            right_wheel_angle_rad,
        ),
    )


def _rotated_wheel_centers(
    geometry: ThreeSegmentSteeringGeometry,
    left_wheel_angle_rad: float,
    right_wheel_angle_rad: float,
) -> tuple[np.ndarray, np.ndarray]:
    return (
        rotate_point_2d(
            geometry.left_wheel.wheel_center,
            geometry.left_wheel.kingpin,
            left_wheel_angle_rad,
        ),
        rotate_point_2d(
            geometry.right_wheel.wheel_center,
            geometry.right_wheel.kingpin,
            right_wheel_angle_rad,
        ),
    )


def _normalize_angle_rad(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _normalize_angle_deg(angle: float) -> float:
    return float((angle + 180.0) % 360.0 - 180.0)


def _nearest_equivalent_angle_deg(angle: float, reference: float) -> float:
    return float(reference + _normalize_angle_deg(angle - reference))


def _circle_intersections(
    center_a: Vec2,
    radius_a: float,
    center_b: Vec2,
    radius_b: float,
) -> tuple[Vec2, ...]:
    delta = center_b - center_a
    center_distance = distance_2d(center_a, center_b)
    if center_distance <= EPS_GEOMETRIC:
        return ()

    too_far = center_distance > radius_a + radius_b + EPS_GEOMETRIC
    contained = center_distance < abs(radius_a - radius_b) - EPS_GEOMETRIC
    if too_far or contained:
        return ()

    axis = delta / center_distance
    along = (
        radius_a * radius_a
        - radius_b * radius_b
        + center_distance * center_distance
    ) / (2.0 * center_distance)
    height_sq = radius_a * radius_a - along * along
    height = float(np.sqrt(max(0.0, height_sq)))
    base = center_a + along * axis
    perpendicular = np.array([-axis[1], axis[0]], dtype=np.float64)

    first = base + height * perpendicular
    if height <= EPS_GEOMETRIC:
        return (first,)
    return (first, base - height * perpendicular)


def _point_angle_about_pivot(point: Vec2, pivot: Vec2) -> float:
    return float(np.arctan2(point[1] - pivot[1], point[0] - pivot[0]))


def _candidate_rotation_angles(
    *,
    pivot: Vec2,
    design_point: Vec2,
    target_point: Vec2,
    link_length: float,
    initial_guess_rad: float,
) -> tuple[tuple[float, Vec2], ...]:
    radius = distance_2d(pivot, design_point)
    intersections = _circle_intersections(pivot, radius, target_point, link_length)
    design_angle = _point_angle_about_pivot(design_point, pivot)
    candidates = []
    for point in intersections:
        current_angle = _point_angle_about_pivot(point, pivot)
        angle = _normalize_angle_rad(current_angle - design_angle)
        guess_error = abs(_normalize_angle_rad(angle - initial_guess_rad))
        candidates.append((guess_error, angle, point))
    return tuple((angle, point) for _, angle, point in sorted(candidates))


def _select_rotation_angle(
    *,
    pivot: Vec2,
    design_point: Vec2,
    target_point: Vec2,
    link_length: float,
    initial_guess_rad: float,
) -> tuple[float, Vec2] | None:
    candidates = _candidate_rotation_angles(
        pivot=pivot,
        design_point=design_point,
        target_point=target_point,
        link_length=link_length,
        initial_guess_rad=initial_guess_rad,
    )
    if not candidates:
        return None
    return candidates[0]


def _select_wheel_angle(
    *,
    wheel: WheelSteeringGeometry2D,
    target_point: Vec2,
    link_length: float,
    initial_guess_rad: float,
) -> tuple[float, Vec2] | None:
    return _select_rotation_angle(
        pivot=wheel.kingpin,
        design_point=wheel.tie_rod_pickup,
        target_point=target_point,
        link_length=link_length,
        initial_guess_rad=initial_guess_rad,
    )


def _length_residuals(
    geometry: ThreeSegmentSteeringGeometry,
    *,
    left_bellcrank_angle_rad: float,
    right_bellcrank_angle_rad: float,
    left_wheel_angle_rad: float,
    right_wheel_angle_rad: float,
) -> tuple[float, float, float]:
    left_center_link, left_bell_tie = geometry.left_bellcrank.rotate(
        left_bellcrank_angle_rad
    )
    right_center_link, right_bell_tie = geometry.right_bellcrank.rotate(
        right_bellcrank_angle_rad
    )
    left_wheel_tie, right_wheel_tie = _rotated_wheel_pickups(
        geometry,
        left_wheel_angle_rad,
        right_wheel_angle_rad,
    )
    return (
        distance_2d(left_center_link, right_center_link)
        - geometry.center_link_length,
        distance_2d(left_wheel_tie, left_bell_tie) - geometry.left_tie_rod_length,
        distance_2d(right_wheel_tie, right_bell_tie) - geometry.right_tie_rod_length,
    )


def solve_three_segment_steering(
    geometry: ThreeSegmentSteeringGeometry,
    left_bellcrank_angle_deg: float,
    initial_guess_deg: Sequence[float] = (0.0, 0.0, 0.0),
    residual_tolerance: float = 1e-6,
) -> ThreeSegmentSteeringSolution:
    """
    Solve roadwheel angles for one driven left relay bellcrank angle.
    """
    initial_guess = np.deg2rad(np.asarray(initial_guess_deg, dtype=np.float64))
    if initial_guess.shape != (3,):
        raise ValueError(
            "initial_guess_deg must contain right bellcrank, left wheel, "
            "and right wheel angles"
        )
    left_bellcrank_angle_rad = float(np.deg2rad(left_bellcrank_angle_deg))
    left_center_link, left_bell_tie = geometry.left_bellcrank.rotate(
        left_bellcrank_angle_rad
    )
    right_bellcrank = _select_rotation_angle(
        pivot=geometry.right_bellcrank.pivot,
        design_point=geometry.right_bellcrank.center_link_pickup,
        target_point=left_center_link,
        link_length=geometry.center_link_length,
        initial_guess_rad=float(initial_guess[0]),
    )
    left_wheel = _select_wheel_angle(
        wheel=geometry.left_wheel,
        target_point=left_bell_tie,
        link_length=geometry.left_tie_rod_length,
        initial_guess_rad=float(initial_guess[1]),
    )
    if right_bellcrank is None or left_wheel is None:
        right_bellcrank_angle_rad = float(initial_guess[0])
        left_wheel_angle_rad = float(initial_guess[1])
        right_wheel_angle_rad = float(initial_guess[2])
    else:
        right_bellcrank_angle_rad = right_bellcrank[0]
        _, right_bell_tie = geometry.right_bellcrank.rotate(right_bellcrank_angle_rad)
        right_wheel = _select_wheel_angle(
            wheel=geometry.right_wheel,
            target_point=right_bell_tie,
            link_length=geometry.right_tie_rod_length,
            initial_guess_rad=float(initial_guess[2]),
        )
        if right_wheel is None:
            right_wheel_angle_rad = float(initial_guess[2])
        else:
            right_wheel_angle_rad = right_wheel[0]
        left_wheel_angle_rad = left_wheel[0]
    right_center_link, right_bell_tie = geometry.right_bellcrank.rotate(
        right_bellcrank_angle_rad
    )
    left_wheel_tie, right_wheel_tie = _rotated_wheel_pickups(
        geometry, left_wheel_angle_rad, right_wheel_angle_rad
    )
    left_wheel_center, right_wheel_center = _rotated_wheel_centers(
        geometry,
        left_wheel_angle_rad,
        right_wheel_angle_rad,
    )
    center_residual, left_residual, right_residual = _length_residuals(
        geometry,
        left_bellcrank_angle_rad=left_bellcrank_angle_rad,
        right_bellcrank_angle_rad=right_bellcrank_angle_rad,
        left_wheel_angle_rad=left_wheel_angle_rad,
        right_wheel_angle_rad=right_wheel_angle_rad,
    )
    max_residual = max(
        abs(center_residual),
        abs(left_residual),
        abs(right_residual),
    )
    return ThreeSegmentSteeringSolution(
        left_bellcrank_angle_deg=float(left_bellcrank_angle_deg),
        right_bellcrank_angle_deg=float(np.rad2deg(right_bellcrank_angle_rad)),
        left_wheel_angle_deg=float(np.rad2deg(left_wheel_angle_rad)),
        right_wheel_angle_deg=float(np.rad2deg(right_wheel_angle_rad)),
        left_wheel_center=left_wheel_center,
        right_wheel_center=right_wheel_center,
        left_tie_rod_pickup=left_wheel_tie,
        right_tie_rod_pickup=right_wheel_tie,
        left_bellcrank_center_link_pickup=left_center_link,
        right_bellcrank_center_link_pickup=right_center_link,
        left_bellcrank_tie_rod_pickup=left_bell_tie,
        right_bellcrank_tie_rod_pickup=right_bell_tie,
        center_link_residual=float(center_residual),
        left_tie_rod_residual=float(left_residual),
        right_tie_rod_residual=float(right_residual),
        converged=bool(max_residual <= residual_tolerance),
        nfev=0,
    )


def sweep_three_segment_steering(
    geometry: ThreeSegmentSteeringGeometry,
    left_bellcrank_angles_deg: Sequence[float],
) -> list[ThreeSegmentSteeringSolution]:
    """
    Solve a sequence of left bellcrank input angles using continuation.
    """
    solutions: list[ThreeSegmentSteeringSolution] = []
    guess = (0.0, 0.0, 0.0)
    for left_bellcrank_angle in left_bellcrank_angles_deg:
        solution = solve_three_segment_steering(
            geometry,
            left_bellcrank_angle_deg=float(left_bellcrank_angle),
            initial_guess_deg=guess,
        )
        solutions.append(solution)
        guess = (
            solution.right_bellcrank_angle_deg,
            solution.left_wheel_angle_deg,
            solution.right_wheel_angle_deg,
        )
    return solutions


def solve_three_segment_from_right_bellcrank_angle(
    geometry: ThreeSegmentSteeringGeometry,
    right_bellcrank_angle_deg: float,
    initial_left_bellcrank_guess_deg: float = 0.0,
) -> ThreeSegmentSteeringSolution:
    """
    Solve steering state from a driven right relay bellcrank angle.
    """
    return _solve_three_segment_from_output(
        geometry,
        target_value_deg=right_bellcrank_angle_deg,
        output_name="right_bellcrank_angle_deg",
        initial_left_bellcrank_guess_deg=initial_left_bellcrank_guess_deg,
    )


def solve_three_segment_from_left_wheel_angle(
    geometry: ThreeSegmentSteeringGeometry,
    left_wheel_angle_deg: float,
    initial_left_bellcrank_guess_deg: float = 0.0,
) -> ThreeSegmentSteeringSolution:
    """
    Solve steering state from a driven left roadwheel angle.
    """
    return _solve_three_segment_from_output(
        geometry,
        target_value_deg=left_wheel_angle_deg,
        output_name="left_wheel_angle_deg",
        initial_left_bellcrank_guess_deg=initial_left_bellcrank_guess_deg,
    )


def solve_three_segment_from_right_wheel_angle(
    geometry: ThreeSegmentSteeringGeometry,
    right_wheel_angle_deg: float,
    initial_left_bellcrank_guess_deg: float = 0.0,
) -> ThreeSegmentSteeringSolution:
    """
    Solve steering state from a driven right roadwheel angle.
    """
    return _solve_three_segment_from_output(
        geometry,
        target_value_deg=right_wheel_angle_deg,
        output_name="right_wheel_angle_deg",
        initial_left_bellcrank_guess_deg=initial_left_bellcrank_guess_deg,
    )


def _solve_three_segment_from_output(
    geometry: ThreeSegmentSteeringGeometry,
    *,
    target_value_deg: float,
    output_name: str,
    initial_left_bellcrank_guess_deg: float,
) -> ThreeSegmentSteeringSolution:
    def residual(values: np.ndarray) -> np.ndarray:
        solution = solve_three_segment_steering(
            geometry,
            left_bellcrank_angle_deg=float(values[0]),
        )
        return np.array(
            [float(getattr(solution, output_name)) - target_value_deg],
            dtype=np.float64,
        )

    guesses = (
        np.array([initial_left_bellcrank_guess_deg], dtype=np.float64),
        np.array([target_value_deg], dtype=np.float64),
        np.array([0.0], dtype=np.float64),
        np.array([-20.0], dtype=np.float64),
        np.array([20.0], dtype=np.float64),
    )
    solutions = [
        solve_three_segment_steering(
            geometry,
            left_bellcrank_angle_deg=float(result.x[0]),
        )
        for result in _least_squares_results(residual, guesses)
    ]
    solution = _select_inverse_solution(
        solutions,
        output_name=output_name,
        target_value_deg=target_value_deg,
        initial_left_bellcrank_guess_deg=initial_left_bellcrank_guess_deg,
    )
    if abs(float(getattr(solution, output_name)) - target_value_deg) > 1e-6:
        raise ValueError(f"No valid three-segment state for {output_name}")
    return solution


def _select_inverse_solution(
    solutions: list[ThreeSegmentSteeringSolution],
    *,
    output_name: str,
    target_value_deg: float,
    initial_left_bellcrank_guess_deg: float,
) -> ThreeSegmentSteeringSolution:
    ranked = []
    for solution in solutions:
        target_error = abs(float(getattr(solution, output_name)) - target_value_deg)
        branch_error = abs(
            _normalize_angle_deg(
                solution.left_bellcrank_angle_deg
                - initial_left_bellcrank_guess_deg
            )
        )
        ranked.append((target_error, branch_error, solution))
    reachable = [item for item in ranked if item[0] <= 1e-6 and item[2].converged]
    if reachable:
        selected = min(reachable, key=lambda item: item[1])[2]
    else:
        selected = min(ranked, key=lambda item: (item[0], item[1]))[2]
    return _with_left_bellcrank_angle_near(
        selected,
        initial_left_bellcrank_guess_deg,
    )


def _with_left_bellcrank_angle_near(
    solution: ThreeSegmentSteeringSolution,
    reference_deg: float,
) -> ThreeSegmentSteeringSolution:
    return ThreeSegmentSteeringSolution(
        left_bellcrank_angle_deg=_nearest_equivalent_angle_deg(
            solution.left_bellcrank_angle_deg,
            reference_deg,
        ),
        right_bellcrank_angle_deg=solution.right_bellcrank_angle_deg,
        left_wheel_angle_deg=solution.left_wheel_angle_deg,
        right_wheel_angle_deg=solution.right_wheel_angle_deg,
        left_wheel_center=solution.left_wheel_center,
        right_wheel_center=solution.right_wheel_center,
        left_tie_rod_pickup=solution.left_tie_rod_pickup,
        right_tie_rod_pickup=solution.right_tie_rod_pickup,
        left_bellcrank_center_link_pickup=solution.left_bellcrank_center_link_pickup,
        right_bellcrank_center_link_pickup=solution.right_bellcrank_center_link_pickup,
        left_bellcrank_tie_rod_pickup=solution.left_bellcrank_tie_rod_pickup,
        right_bellcrank_tie_rod_pickup=solution.right_bellcrank_tie_rod_pickup,
        center_link_residual=solution.center_link_residual,
        left_tie_rod_residual=solution.left_tie_rod_residual,
        right_tie_rod_residual=solution.right_tie_rod_residual,
        converged=solution.converged,
        nfev=solution.nfev,
    )


def _least_squares_results(
    residual: object,
    guesses: tuple[np.ndarray, ...],
) -> list[OptimizeResult]:
    return [
        least_squares(
            residual,
            guess,
            method="lm",
            xtol=1e-12,
            ftol=1e-12,
            gtol=1e-12,
            max_nfev=200,
        )
        for guess in guesses
    ]


def _best_least_squares_result(
    residual: object,
    guesses: tuple[np.ndarray, ...],
) -> OptimizeResult:
    results = _least_squares_results(residual, guesses)
    return min(results, key=lambda result: float(np.linalg.norm(result.fun)))
