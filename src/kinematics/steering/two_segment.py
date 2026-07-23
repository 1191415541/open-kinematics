"""
Two-segment steering linkage solver for top-view 2D geometry.
"""

from __future__ import annotations

from typing import Sequence

import numpy as np
from scipy.optimize import least_squares

from kinematics.core.constants import EPS_GEOMETRIC
from kinematics.core.vector_utils.geometric import rotate_point_about_axis
from kinematics.steering.geometry import (
    SteeringCoordinateSystem,
    TwoSegmentSteeringAnalyticComparison,
    TwoSegmentSteeringComparison,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
    Vec2,
    Vec3,
    WheelSteeringGeometry2D,
    distance_2d,
    make_vec2,
    rotate_point_2d,
)

SteeringInputGeometry = TwoSegmentSteeringGeometry | TwoSegmentSteeringHardpoints3D


def _normalize_angle_rad(angle: float) -> float:
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def _normalize_angle_deg(angle: float) -> float:
    return float((angle + 180.0) % 360.0 - 180.0)


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
        radius_a * radius_a - radius_b * radius_b + center_distance * center_distance
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


def _rotate_pitman_outputs_3d(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    pitman_angle_rad: float,
) -> tuple[Vec3, Vec3]:
    pivot = hardpoints.pitman.pivot
    axis = SteeringCoordinateSystem.Z_UP
    return (
        rotate_point_about_axis(
            hardpoints.pitman.left_output,
            pivot,
            axis,
            pitman_angle_rad,
        ),
        rotate_point_about_axis(
            hardpoints.pitman.right_output,
            pivot,
            axis,
            pitman_angle_rad,
        ),
    )


def _rotate_wheel_state_3d(
    wheel,
    wheel_angle_rad: float,
) -> tuple[Vec3, Vec3]:
    axis = wheel.kingpin_upper - wheel.kingpin_lower
    return (
        rotate_point_about_axis(
            wheel.wheel_center,
            wheel.kingpin_lower,
            axis,
            wheel_angle_rad,
        ),
        rotate_point_about_axis(
            wheel.tie_rod_pickup,
            wheel.kingpin_lower,
            axis,
            wheel_angle_rad,
        ),
    )


def _tie_rod_residual_3d(
    pickup: Vec3,
    pitman_output: Vec3,
    design_length: float,
) -> float:
    return float(np.linalg.norm(pickup - pitman_output) - design_length)


def _rotation_basis(
    point: Vec3,
    pivot: Vec3,
    axis: Vec3,
) -> tuple[Vec3, Vec3, Vec3]:
    axis_unit = np.asarray(axis, dtype=np.float64)
    axis_norm = float(np.linalg.norm(axis_unit))
    if axis_norm <= EPS_GEOMETRIC:
        raise ValueError("Rotation axis must be non-zero")
    axis_unit = axis_unit / axis_norm
    radial = point - pivot
    axial = float(np.dot(radial, axis_unit))
    axial_component = axial * axis_unit
    u = radial - axial_component
    u_norm = float(np.linalg.norm(u))
    if u_norm <= EPS_GEOMETRIC:
        raise ValueError("Rotated point must not lie on the rotation axis")
    u = u / u_norm
    v = np.cross(axis_unit, u)
    return axis_unit, u, v


def _angle_candidates_for_distance(
    point: Vec3,
    pivot: Vec3,
    axis: Vec3,
    target: Vec3,
    distance: float,
) -> tuple[float, ...]:
    axis_unit, u, v = _rotation_basis(point, pivot, axis)
    radial = point - pivot
    axial = float(np.dot(radial, axis_unit))
    axial_component = axial * axis_unit
    orbit_radius = float(np.linalg.norm(radial - axial_component))
    center = pivot + axial_component
    delta = target - center
    alpha = float(np.dot(delta, u))
    beta = float(np.dot(delta, v))
    gamma = float(np.dot(delta, axis_unit))
    rhs = (
        orbit_radius * orbit_radius
        + alpha * alpha
        + beta * beta
        + gamma * gamma
        - distance * distance
    ) / (2.0 * orbit_radius)
    amplitude = float(np.hypot(alpha, beta))
    if amplitude <= EPS_GEOMETRIC:
        if abs(rhs) <= EPS_GEOMETRIC:
            return (0.0,)
        return ()
    normalized = rhs / amplitude
    if normalized < -1.0 - EPS_GEOMETRIC or normalized > 1.0 + EPS_GEOMETRIC:
        return ()
    normalized = float(np.clip(normalized, -1.0, 1.0))
    phase = float(np.arctan2(beta, alpha))
    offset = float(np.arccos(normalized))
    first = _normalize_angle_rad(phase + offset)
    second = _normalize_angle_rad(phase - offset)
    if abs(_normalize_angle_rad(first - second)) <= EPS_GEOMETRIC:
        return (first,)
    return (first, second)


def _pick_angle_candidate(
    candidates: tuple[float, ...],
    initial_guess_rad: float,
) -> float:
    if not candidates:
        raise ValueError("No valid steering arm position for this pitman angle")
    return min(
        candidates,
        key=lambda candidate: abs(_normalize_angle_rad(candidate - initial_guess_rad)),
    )


def _solve_wheel_pickup_3d(
    *,
    wheel,
    pitman_output: Vec3,
    tie_rod_length: float,
    initial_guess_rad: float,
) -> tuple[float, Vec3, Vec3]:
    def residual(values: np.ndarray) -> np.ndarray:
        angle_rad = float(values[0])
        wheel_center, pickup = _rotate_wheel_state_3d(wheel, angle_rad)
        return np.array(
            [_tie_rod_residual_3d(pickup, pitman_output, tie_rod_length)],
            dtype=np.float64,
        )

    guesses = (
        np.array([initial_guess_rad], dtype=np.float64),
        np.array([initial_guess_rad + np.deg2rad(12.0)], dtype=np.float64),
        np.array([initial_guess_rad - np.deg2rad(12.0)], dtype=np.float64),
    )
    results = [
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
    best = min(results, key=lambda result: float(np.linalg.norm(result.fun)))
    angle_rad = _normalize_angle_rad(float(best.x[0]))
    wheel_center, pickup = _rotate_wheel_state_3d(wheel, angle_rad)
    return angle_rad, wheel_center, pickup


def _solve_wheel_pickup_3d_analytic(
    *,
    wheel,
    pitman_output: Vec3,
    tie_rod_length: float,
    initial_guess_rad: float,
) -> tuple[float, Vec3, Vec3]:
    candidates = _angle_candidates_for_distance(
        wheel.tie_rod_pickup,
        wheel.kingpin_lower,
        wheel.kingpin_upper - wheel.kingpin_lower,
        pitman_output,
        tie_rod_length,
    )
    angle_rad = _pick_angle_candidate(candidates, initial_guess_rad)
    wheel_center, pickup = _rotate_wheel_state_3d(wheel, angle_rad)
    return angle_rad, wheel_center, pickup


def solve_two_segment_steering_3d(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    pitman_angle_deg: float,
    initial_guess_deg: Sequence[float] = (0.0, 0.0),
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """Solve one two-segment steering state using 3D hardpoints directly."""
    initial_guess = np.deg2rad(np.asarray(initial_guess_deg, dtype=np.float64))
    if initial_guess.shape != (2,):
        raise ValueError("initial_guess_deg must contain left and right angles")

    pitman_angle_rad = float(np.deg2rad(pitman_angle_deg))
    pitman_outputs_3d = _rotate_pitman_outputs_3d(hardpoints, pitman_angle_rad)
    left_design_length = float(
        np.linalg.norm(
            hardpoints.left_wheel.tie_rod_pickup - hardpoints.pitman.left_output
        )
    )
    right_design_length = float(
        np.linalg.norm(
            hardpoints.right_wheel.tie_rod_pickup - hardpoints.pitman.right_output
        )
    )

    left_angle_rad, left_center_3d, left_pickup_3d = _solve_wheel_pickup_3d(
        wheel=hardpoints.left_wheel,
        pitman_output=pitman_outputs_3d[0],
        tie_rod_length=left_design_length,
        initial_guess_rad=float(initial_guess[0]),
    )
    right_angle_rad, right_center_3d, right_pickup_3d = _solve_wheel_pickup_3d(
        wheel=hardpoints.right_wheel,
        pitman_output=pitman_outputs_3d[1],
        tie_rod_length=right_design_length,
        initial_guess_rad=float(initial_guess[1]),
    )

    left_residual = _tie_rod_residual_3d(
        left_pickup_3d,
        pitman_outputs_3d[0],
        left_design_length,
    )
    right_residual = _tie_rod_residual_3d(
        right_pickup_3d,
        pitman_outputs_3d[1],
        right_design_length,
    )
    max_residual = max(abs(left_residual), abs(right_residual))

    return TwoSegmentSteeringSolution(
        pitman_angle_deg=float(pitman_angle_deg),
        left_wheel_angle_deg=float(np.rad2deg(left_angle_rad)),
        right_wheel_angle_deg=float(np.rad2deg(right_angle_rad)),
        left_wheel_center=left_center_3d[:2].copy(),
        right_wheel_center=right_center_3d[:2].copy(),
        left_tie_rod_pickup=left_pickup_3d[:2].copy(),
        right_tie_rod_pickup=right_pickup_3d[:2].copy(),
        pitman_left_output=pitman_outputs_3d[0][:2].copy(),
        pitman_right_output=pitman_outputs_3d[1][:2].copy(),
        left_tie_rod_residual=float(left_residual),
        right_tie_rod_residual=float(right_residual),
        converged=bool(max_residual <= residual_tolerance),
        nfev=0,
        left_wheel_center_3d=left_center_3d,
        right_wheel_center_3d=right_center_3d,
        left_tie_rod_pickup_3d=left_pickup_3d,
        right_tie_rod_pickup_3d=right_pickup_3d,
        pitman_left_output_3d=pitman_outputs_3d[0],
        pitman_right_output_3d=pitman_outputs_3d[1],
    )


def solve_two_segment_steering_3d_analytic(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    pitman_angle_deg: float,
    initial_guess_deg: Sequence[float] = (0.0, 0.0),
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """Solve one two-segment steering state using analytic 3D geometry."""
    initial_guess = np.deg2rad(np.asarray(initial_guess_deg, dtype=np.float64))
    if initial_guess.shape != (2,):
        raise ValueError("initial_guess_deg must contain left and right angles")

    pitman_angle_rad = float(np.deg2rad(pitman_angle_deg))
    pitman_outputs_3d = _rotate_pitman_outputs_3d(hardpoints, pitman_angle_rad)
    left_design_length = float(
        np.linalg.norm(
            hardpoints.left_wheel.tie_rod_pickup - hardpoints.pitman.left_output
        )
    )
    right_design_length = float(
        np.linalg.norm(
            hardpoints.right_wheel.tie_rod_pickup - hardpoints.pitman.right_output
        )
    )

    left_angle_rad, left_center_3d, left_pickup_3d = _solve_wheel_pickup_3d_analytic(
        wheel=hardpoints.left_wheel,
        pitman_output=pitman_outputs_3d[0],
        tie_rod_length=left_design_length,
        initial_guess_rad=float(initial_guess[0]),
    )
    right_angle_rad, right_center_3d, right_pickup_3d = _solve_wheel_pickup_3d_analytic(
        wheel=hardpoints.right_wheel,
        pitman_output=pitman_outputs_3d[1],
        tie_rod_length=right_design_length,
        initial_guess_rad=float(initial_guess[1]),
    )

    left_residual = _tie_rod_residual_3d(
        left_pickup_3d,
        pitman_outputs_3d[0],
        left_design_length,
    )
    right_residual = _tie_rod_residual_3d(
        right_pickup_3d,
        pitman_outputs_3d[1],
        right_design_length,
    )
    max_residual = max(abs(left_residual), abs(right_residual))

    return TwoSegmentSteeringSolution(
        pitman_angle_deg=float(pitman_angle_deg),
        left_wheel_angle_deg=float(np.rad2deg(left_angle_rad)),
        right_wheel_angle_deg=float(np.rad2deg(right_angle_rad)),
        left_wheel_center=left_center_3d[:2].copy(),
        right_wheel_center=right_center_3d[:2].copy(),
        left_tie_rod_pickup=left_pickup_3d[:2].copy(),
        right_tie_rod_pickup=right_pickup_3d[:2].copy(),
        pitman_left_output=pitman_outputs_3d[0][:2].copy(),
        pitman_right_output=pitman_outputs_3d[1][:2].copy(),
        left_tie_rod_residual=float(left_residual),
        right_tie_rod_residual=float(right_residual),
        converged=bool(max_residual <= residual_tolerance),
        nfev=0,
        left_wheel_center_3d=left_center_3d,
        right_wheel_center_3d=right_center_3d,
        left_tie_rod_pickup_3d=left_pickup_3d,
        right_tie_rod_pickup_3d=right_pickup_3d,
        pitman_left_output_3d=pitman_outputs_3d[0],
        pitman_right_output_3d=pitman_outputs_3d[1],
    )


def rack_displacement_from_pinion_angle(
    pinion_angle_deg: float,
    pinion_pitch_radius_mm: float,
) -> float:
    """Return rack travel for a no-slip pinion rotation."""
    if pinion_pitch_radius_mm <= EPS_GEOMETRIC:
        raise ValueError("Pinion pitch radius must be positive")
    return float(np.deg2rad(pinion_angle_deg) * pinion_pitch_radius_mm)


def pinion_angle_from_rack_displacement(
    rack_displacement_mm: float,
    pinion_pitch_radius_mm: float,
) -> float:
    """Return the no-slip pinion rotation needed for a rack displacement."""
    if pinion_pitch_radius_mm <= EPS_GEOMETRIC:
        raise ValueError("Pinion pitch radius must be positive")
    return float(np.rad2deg(rack_displacement_mm / pinion_pitch_radius_mm))


def solve_two_segment_rack_and_pinion_3d_analytic(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    rack_displacement_mm: float,
    initial_guess_deg: Sequence[float] = (0.0, 0.0),
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """Solve a two-segment steering system driven by lateral rack travel.

    The existing left/right pitman-output hardpoints define the two inner tie-rod
    joints at the design position. A rack displacement translates both joints
    along the vehicle-right Y axis without rotating them.
    """
    initial_guess = np.deg2rad(np.asarray(initial_guess_deg, dtype=np.float64))
    if initial_guess.shape != (2,):
        raise ValueError("initial_guess_deg must contain left and right angles")

    rack_translation = float(rack_displacement_mm) * SteeringCoordinateSystem.Y_RIGHT
    rack_outputs_3d = (
        hardpoints.pitman.left_output + rack_translation,
        hardpoints.pitman.right_output + rack_translation,
    )
    left_design_length = float(
        np.linalg.norm(
            hardpoints.left_wheel.tie_rod_pickup - hardpoints.pitman.left_output
        )
    )
    right_design_length = float(
        np.linalg.norm(
            hardpoints.right_wheel.tie_rod_pickup - hardpoints.pitman.right_output
        )
    )

    left_angle_rad, left_center_3d, left_pickup_3d = _solve_wheel_pickup_3d_analytic(
        wheel=hardpoints.left_wheel,
        pitman_output=rack_outputs_3d[0],
        tie_rod_length=left_design_length,
        initial_guess_rad=float(initial_guess[0]),
    )
    right_angle_rad, right_center_3d, right_pickup_3d = _solve_wheel_pickup_3d_analytic(
        wheel=hardpoints.right_wheel,
        pitman_output=rack_outputs_3d[1],
        tie_rod_length=right_design_length,
        initial_guess_rad=float(initial_guess[1]),
    )

    left_residual = _tie_rod_residual_3d(
        left_pickup_3d,
        rack_outputs_3d[0],
        left_design_length,
    )
    right_residual = _tie_rod_residual_3d(
        right_pickup_3d,
        rack_outputs_3d[1],
        right_design_length,
    )
    max_residual = max(abs(left_residual), abs(right_residual))

    return TwoSegmentSteeringSolution(
        pitman_angle_deg=0.0,
        left_wheel_angle_deg=float(np.rad2deg(left_angle_rad)),
        right_wheel_angle_deg=float(np.rad2deg(right_angle_rad)),
        left_wheel_center=left_center_3d[:2].copy(),
        right_wheel_center=right_center_3d[:2].copy(),
        left_tie_rod_pickup=left_pickup_3d[:2].copy(),
        right_tie_rod_pickup=right_pickup_3d[:2].copy(),
        pitman_left_output=rack_outputs_3d[0][:2].copy(),
        pitman_right_output=rack_outputs_3d[1][:2].copy(),
        left_tie_rod_residual=float(left_residual),
        right_tie_rod_residual=float(right_residual),
        converged=bool(max_residual <= residual_tolerance),
        nfev=0,
        left_wheel_center_3d=left_center_3d,
        right_wheel_center_3d=right_center_3d,
        left_tie_rod_pickup_3d=left_pickup_3d,
        right_tie_rod_pickup_3d=right_pickup_3d,
        pitman_left_output_3d=rack_outputs_3d[0],
        pitman_right_output_3d=rack_outputs_3d[1],
    )


def _best_pitman_solution_3d(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    target_angle_deg: float,
    *,
    side: str,
    initial_pitman_guess_deg: float,
    residual_tolerance: float,
) -> TwoSegmentSteeringSolution:
    def residual(values: np.ndarray) -> np.ndarray:
        pitman_angle_deg = float(values[0])
        solution = solve_two_segment_steering_3d(
            hardpoints,
            pitman_angle_deg=pitman_angle_deg,
            residual_tolerance=residual_tolerance,
        )
        wheel_angle_deg = (
            solution.left_wheel_angle_deg
            if side == "left"
            else solution.right_wheel_angle_deg
        )
        return np.array([wheel_angle_deg - target_angle_deg], dtype=np.float64)

    guesses = (
        np.array([initial_pitman_guess_deg], dtype=np.float64),
        np.array([initial_pitman_guess_deg + 12.0], dtype=np.float64),
        np.array([initial_pitman_guess_deg - 12.0], dtype=np.float64),
    )
    results = [
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
    best = min(results, key=lambda result: float(np.linalg.norm(result.fun)))
    solution = solve_two_segment_steering_3d(
        hardpoints,
        pitman_angle_deg=float(best.x[0]),
        residual_tolerance=residual_tolerance,
    )
    achieved_angle_deg = (
        solution.left_wheel_angle_deg
        if side == "left"
        else solution.right_wheel_angle_deg
    )
    if abs(achieved_angle_deg - target_angle_deg) > residual_tolerance:
        raise ValueError("No valid pitman arm position for this wheel angle")
    return solution


def _best_pitman_solution_3d_analytic(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    target_angle_deg: float,
    *,
    side: str,
    initial_pitman_guess_deg: float,
    residual_tolerance: float,
) -> TwoSegmentSteeringSolution:
    wheel = hardpoints.left_wheel if side == "left" else hardpoints.right_wheel
    wheel_angle_rad = float(np.deg2rad(target_angle_deg))
    _, pickup = _rotate_wheel_state_3d(wheel, wheel_angle_rad)
    output = (
        hardpoints.pitman.left_output
        if side == "left"
        else hardpoints.pitman.right_output
    )
    candidates = _angle_candidates_for_distance(
        output,
        hardpoints.pitman.pivot,
        SteeringCoordinateSystem.Z_UP,
        pickup,
        float(np.linalg.norm(wheel.tie_rod_pickup - output)),
    )
    if not candidates:
        raise ValueError("No valid pitman arm position for this wheel angle")
    pitman_angle_rad = _pick_angle_candidate(
        candidates,
        float(np.deg2rad(initial_pitman_guess_deg)),
    )
    solution = solve_two_segment_steering_3d_analytic(
        hardpoints,
        pitman_angle_deg=float(np.rad2deg(pitman_angle_rad)),
        initial_guess_deg=(
            (target_angle_deg, 0.0) if side == "left" else (0.0, target_angle_deg)
        ),
        residual_tolerance=residual_tolerance,
    )
    achieved_angle_deg = (
        solution.left_wheel_angle_deg
        if side == "left"
        else solution.right_wheel_angle_deg
    )
    if abs(achieved_angle_deg - target_angle_deg) > residual_tolerance:
        raise ValueError("No valid pitman arm position for this wheel angle")
    return solution


def compare_two_segment_2d_and_3d(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    pitman_angle_deg: float,
    initial_guess_deg: Sequence[float] = (0.0, 0.0),
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringComparison:
    """Solve the same hardpoints in projected 2D and direct 3D, then compare them."""
    solve_2d = solve_two_segment_steering(
        hardpoints,
        pitman_angle_deg=pitman_angle_deg,
        initial_guess_deg=initial_guess_deg,
        residual_tolerance=residual_tolerance,
    )
    solve_3d = solve_two_segment_steering_3d(
        hardpoints,
        pitman_angle_deg=pitman_angle_deg,
        initial_guess_deg=initial_guess_deg,
        residual_tolerance=residual_tolerance,
    )
    return TwoSegmentSteeringComparison(
        solve_2d=solve_2d,
        solve_3d=solve_3d,
        left_wheel_angle_delta_deg=_normalize_angle_deg(
            solve_3d.left_wheel_angle_deg - solve_2d.left_wheel_angle_deg
        ),
        right_wheel_angle_delta_deg=_normalize_angle_deg(
            solve_3d.right_wheel_angle_deg - solve_2d.right_wheel_angle_deg
        ),
        left_wheel_center_delta_2d=make_vec2(
            solve_3d.left_wheel_center - solve_2d.left_wheel_center
        ),
        right_wheel_center_delta_2d=make_vec2(
            solve_3d.right_wheel_center - solve_2d.right_wheel_center
        ),
        left_tie_rod_pickup_delta_2d=make_vec2(
            solve_3d.left_tie_rod_pickup - solve_2d.left_tie_rod_pickup
        ),
        right_tie_rod_pickup_delta_2d=make_vec2(
            solve_3d.right_tie_rod_pickup - solve_2d.right_tie_rod_pickup
        ),
        pitman_left_output_delta_2d=make_vec2(
            solve_3d.pitman_left_output - solve_2d.pitman_left_output
        ),
        pitman_right_output_delta_2d=make_vec2(
            solve_3d.pitman_right_output - solve_2d.pitman_right_output
        ),
    )


def compare_two_segment_3d_analytic_and_numeric(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    pitman_angle_deg: float,
    initial_guess_deg: Sequence[float] = (0.0, 0.0),
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringAnalyticComparison:
    """Compare analytic and numeric 3D steering solves for one pitman angle."""
    solve_numeric = solve_two_segment_steering_3d(
        hardpoints,
        pitman_angle_deg=pitman_angle_deg,
        initial_guess_deg=initial_guess_deg,
        residual_tolerance=residual_tolerance,
    )
    solve_analytic = solve_two_segment_steering_3d_analytic(
        hardpoints,
        pitman_angle_deg=pitman_angle_deg,
        initial_guess_deg=initial_guess_deg,
        residual_tolerance=residual_tolerance,
    )
    return TwoSegmentSteeringAnalyticComparison(
        solve_numeric=solve_numeric,
        solve_analytic=solve_analytic,
        left_wheel_angle_delta_deg=_normalize_angle_deg(
            solve_analytic.left_wheel_angle_deg - solve_numeric.left_wheel_angle_deg
        ),
        right_wheel_angle_delta_deg=_normalize_angle_deg(
            solve_analytic.right_wheel_angle_deg - solve_numeric.right_wheel_angle_deg
        ),
        pitman_angle_delta_deg=_normalize_angle_deg(
            solve_analytic.pitman_angle_deg - solve_numeric.pitman_angle_deg
        ),
        left_tie_rod_residual_delta=(
            solve_analytic.left_tie_rod_residual - solve_numeric.left_tie_rod_residual
        ),
        right_tie_rod_residual_delta=(
            solve_analytic.right_tie_rod_residual - solve_numeric.right_tie_rod_residual
        ),
    )


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


def solve_two_segment_from_left_wheel_angle_3d(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    left_wheel_angle_deg: float,
    initial_pitman_guess_deg: float = 0.0,
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """Solve 3D steering state from a driven left roadwheel angle."""
    return _best_pitman_solution_3d(
        hardpoints,
        left_wheel_angle_deg,
        side="left",
        initial_pitman_guess_deg=initial_pitman_guess_deg,
        residual_tolerance=residual_tolerance,
    )


def solve_two_segment_from_left_wheel_angle_3d_analytic(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    left_wheel_angle_deg: float,
    initial_pitman_guess_deg: float = 0.0,
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """Solve 3D steering state from a driven left roadwheel angle analytically."""
    return _best_pitman_solution_3d_analytic(
        hardpoints,
        left_wheel_angle_deg,
        side="left",
        initial_pitman_guess_deg=initial_pitman_guess_deg,
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


def solve_two_segment_from_right_wheel_angle_3d(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    right_wheel_angle_deg: float,
    initial_pitman_guess_deg: float = 0.0,
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """Solve 3D steering state from a driven right roadwheel angle."""
    return _best_pitman_solution_3d(
        hardpoints,
        right_wheel_angle_deg,
        side="right",
        initial_pitman_guess_deg=initial_pitman_guess_deg,
        residual_tolerance=residual_tolerance,
    )


def solve_two_segment_from_right_wheel_angle_3d_analytic(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    right_wheel_angle_deg: float,
    initial_pitman_guess_deg: float = 0.0,
    residual_tolerance: float = 1e-6,
) -> TwoSegmentSteeringSolution:
    """Solve 3D steering state from a driven right roadwheel angle analytically."""
    return _best_pitman_solution_3d_analytic(
        hardpoints,
        right_wheel_angle_deg,
        side="right",
        initial_pitman_guess_deg=initial_pitman_guess_deg,
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
