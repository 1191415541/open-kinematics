"""
Two-segment steering linkage solver for top-view 2D geometry.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np

from kinematics.core.constants import EPS_GEOMETRIC
from kinematics.steering.geometry import (
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
    Vec2,
    WheelSteeringGeometry2D,
    distance_2d,
    rotate_point_2d,
)

SteeringInputGeometry = TwoSegmentSteeringGeometry | TwoSegmentSteeringHardpoints3D


def _normalize_angle_rad(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _as_2d_geometry(geometry: SteeringInputGeometry) -> TwoSegmentSteeringGeometry:
    if isinstance(geometry, TwoSegmentSteeringHardpoints3D):
        return geometry.to_2d_geometry()
    return geometry


def _circle_intersections(
    center_a: Vec2,
    radius_a: float,
    center_b: Vec2,
    radius_b: float,
) -> tuple[Vec2, ...]:
    delta = center_b - center_a
    center_distance = distance_2d(center_a, center_b)
    if center_distance <= EPS_GEOMETRIC:
        raise ValueError("Circle centers must be distinct for steering solve")

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


def _solve_wheel_pickup(
    *,
    wheel: WheelSteeringGeometry2D,
    pitman_output: Vec2,
    tie_rod_length: float,
    initial_guess_rad: float,
) -> tuple[float, Vec2]:
    pickup_radius = distance_2d(wheel.kingpin, wheel.tie_rod_pickup)
    intersections = _circle_intersections(
        wheel.kingpin,
        pickup_radius,
        pitman_output,
        tie_rod_length,
    )
    if not intersections:
        raise ValueError("No valid steering arm position for this pitman angle")

    design_angle = _point_angle_about_pivot(wheel.tie_rod_pickup, wheel.kingpin)
    candidates = []
    for point in intersections:
        current_angle = _point_angle_about_pivot(point, wheel.kingpin)
        wheel_angle = _normalize_angle_rad(current_angle - design_angle)
        guess_error = abs(_normalize_angle_rad(wheel_angle - initial_guess_rad))
        candidates.append((guess_error, wheel_angle, point))

    _, wheel_angle, point = min(candidates, key=lambda item: item[0])
    return wheel_angle, point


def _solve_pitman_angle_from_wheel(
    *,
    wheel: WheelSteeringGeometry2D,
    wheel_angle_deg: float,
    pitman_pivot: Vec2,
    pitman_output: Vec2,
    tie_rod_length: float,
    initial_guess_deg: float,
) -> float:
    wheel_angle_rad = float(np.deg2rad(wheel_angle_deg))
    pickup = rotate_point_2d(wheel.tie_rod_pickup, wheel.kingpin, wheel_angle_rad)
    output_radius = distance_2d(pitman_pivot, pitman_output)
    intersections = _circle_intersections(
        pitman_pivot,
        output_radius,
        pickup,
        tie_rod_length,
    )
    if not intersections:
        raise ValueError("No valid pitman arm position for this wheel angle")

    design_angle = _point_angle_about_pivot(pitman_output, pitman_pivot)
    initial_guess_rad = float(np.deg2rad(initial_guess_deg))
    candidates = []
    for point in intersections:
        current_angle = _point_angle_about_pivot(point, pitman_pivot)
        pitman_angle = _normalize_angle_rad(current_angle - design_angle)
        guess_error = abs(_normalize_angle_rad(pitman_angle - initial_guess_rad))
        candidates.append((guess_error, pitman_angle))

    _, pitman_angle_rad = min(candidates, key=lambda item: item[0])
    return float(np.rad2deg(pitman_angle_rad))


def _tie_rod_residuals(
    geometry: TwoSegmentSteeringGeometry,
    pitman_outputs: tuple[Vec2, Vec2],
    left_pickup: Vec2,
    right_pickup: Vec2,
) -> tuple[float, float]:
    left = distance_2d(left_pickup, pitman_outputs[0]) - geometry.left_tie_rod_length
    right = distance_2d(right_pickup, pitman_outputs[1]) - geometry.right_tie_rod_length
    return float(left), float(right)


def _wheel_centers(
    geometry: TwoSegmentSteeringGeometry,
    left_angle_rad: float,
    right_angle_rad: float,
) -> tuple[Vec2, Vec2]:
    left = rotate_point_2d(
        geometry.left_wheel.wheel_center,
        geometry.left_wheel.kingpin,
        left_angle_rad,
    )
    right = rotate_point_2d(
        geometry.right_wheel.wheel_center,
        geometry.right_wheel.kingpin,
        right_angle_rad,
    )
    return left, right


def solve_two_segment_steering(
    geometry: SteeringInputGeometry,
    pitman_angle_deg: float,
    initial_guess_deg: Sequence[float] = (0.0, 0.0),
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """
    Solve roadwheel angles for one driven pitman arm angle.
    """
    initial_guess = np.deg2rad(np.asarray(initial_guess_deg, dtype=np.float64))
    if initial_guess.shape != (2,):
        raise ValueError("initial_guess_deg must contain left and right angles")

    geometry_2d = _as_2d_geometry(geometry)
    pitman_outputs = geometry_2d.pitman.rotate(float(np.deg2rad(pitman_angle_deg)))
    left_angle_rad, left_pickup = _solve_wheel_pickup(
        wheel=geometry_2d.left_wheel,
        pitman_output=pitman_outputs[0],
        tie_rod_length=geometry_2d.left_tie_rod_length,
        initial_guess_rad=float(initial_guess[0]),
    )
    right_angle_rad, right_pickup = _solve_wheel_pickup(
        wheel=geometry_2d.right_wheel,
        pitman_output=pitman_outputs[1],
        tie_rod_length=geometry_2d.right_tie_rod_length,
        initial_guess_rad=float(initial_guess[1]),
    )
    left_residual, right_residual = _tie_rod_residuals(
        geometry_2d,
        pitman_outputs,
        left_pickup,
        right_pickup,
    )
    max_residual = max(abs(left_residual), abs(right_residual))
    left_center, right_center = _wheel_centers(
        geometry_2d,
        left_angle_rad,
        right_angle_rad,
    )

    return TwoSegmentSteeringSolution(
        pitman_angle_deg=float(pitman_angle_deg),
        left_wheel_angle_deg=float(np.rad2deg(left_angle_rad)),
        right_wheel_angle_deg=float(np.rad2deg(right_angle_rad)),
        left_wheel_center=left_center,
        right_wheel_center=right_center,
        left_tie_rod_pickup=left_pickup,
        right_tie_rod_pickup=right_pickup,
        pitman_left_output=pitman_outputs[0],
        pitman_right_output=pitman_outputs[1],
        left_tie_rod_residual=float(left_residual),
        right_tie_rod_residual=float(right_residual),
        converged=bool(max_residual <= residual_tolerance),
        nfev=0,
    )


def solve_two_segment_from_left_wheel_angle(
    geometry: SteeringInputGeometry,
    left_wheel_angle_deg: float,
    initial_pitman_guess_deg: float = 0.0,
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """
    Solve steering state from a driven left roadwheel angle.
    """
    geometry_2d = _as_2d_geometry(geometry)
    pitman_angle = _solve_pitman_angle_from_wheel(
        wheel=geometry_2d.left_wheel,
        wheel_angle_deg=left_wheel_angle_deg,
        pitman_pivot=geometry_2d.pitman.pivot,
        pitman_output=geometry_2d.pitman.left_output,
        tie_rod_length=geometry_2d.left_tie_rod_length,
        initial_guess_deg=initial_pitman_guess_deg,
    )
    return solve_two_segment_steering(
        geometry_2d,
        pitman_angle_deg=pitman_angle,
        initial_guess_deg=(left_wheel_angle_deg, 0.0),
        residual_tolerance=residual_tolerance,
    )


def solve_two_segment_from_right_wheel_angle(
    geometry: SteeringInputGeometry,
    right_wheel_angle_deg: float,
    initial_pitman_guess_deg: float = 0.0,
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """
    Solve steering state from a driven right roadwheel angle.
    """
    geometry_2d = _as_2d_geometry(geometry)
    pitman_angle = _solve_pitman_angle_from_wheel(
        wheel=geometry_2d.right_wheel,
        wheel_angle_deg=right_wheel_angle_deg,
        pitman_pivot=geometry_2d.pitman.pivot,
        pitman_output=geometry_2d.pitman.right_output,
        tie_rod_length=geometry_2d.right_tie_rod_length,
        initial_guess_deg=initial_pitman_guess_deg,
    )
    return solve_two_segment_steering(
        geometry_2d,
        pitman_angle_deg=pitman_angle,
        initial_guess_deg=(0.0, right_wheel_angle_deg),
        residual_tolerance=residual_tolerance,
    )


def sweep_two_segment_steering(
    geometry: SteeringInputGeometry,
    pitman_angles_deg: Sequence[float],
) -> list[TwoSegmentSteeringSolution]:
    """
    Solve a sequence of pitman arm angles using continuation.
    """
    geometry_2d = _as_2d_geometry(geometry)
    solutions: list[TwoSegmentSteeringSolution] = []
    guess = (0.0, 0.0)
    for pitman_angle in pitman_angles_deg:
        solution = solve_two_segment_steering(
            geometry_2d,
            pitman_angle_deg=float(pitman_angle),
            initial_guess_deg=guess,
        )
        solutions.append(solution)
        guess = (solution.left_wheel_angle_deg, solution.right_wheel_angle_deg)
    return solutions
