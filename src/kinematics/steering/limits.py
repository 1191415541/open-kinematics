"""
Travel limit estimation for two-segment steering geometry.
"""

from __future__ import annotations

from dataclasses import dataclass

from kinematics.steering.geometry import (
    ThreeSegmentSteeringGeometry,
    ThreeSegmentSteeringHardpoints3D,
    ThreeSegmentSteeringSolution,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
)
from kinematics.steering.three_segment import (
    solve_three_segment_steering,
    solve_three_segment_steering_3d_analytic,
)
from kinematics.steering.two_segment import (
    solve_two_segment_steering,
    solve_two_segment_steering_3d_analytic,
)

SteeringInputGeometry = TwoSegmentSteeringGeometry | TwoSegmentSteeringHardpoints3D
ThreeSegmentInputGeometry = (
    ThreeSegmentSteeringGeometry | ThreeSegmentSteeringHardpoints3D
)
SteeringLimitSolution = TwoSegmentSteeringSolution | ThreeSegmentSteeringSolution
UNREACHABLE_PREFIXES = (
    "No valid steering arm position",
    "No valid pitman arm position",
    "No valid steering arm position for this bellcrank angle",
)


@dataclass(frozen=True)
class SteeringTravelLimits:
    """Left-turn and right-turn travel limit states."""

    left_turn: SteeringLimitSolution
    right_turn: SteeringLimitSolution


def _average_wheel_angle(solution: SteeringLimitSolution) -> float:
    return 0.5 * (solution.left_wheel_angle_deg + solution.right_wheel_angle_deg)


def _is_unreachable_error(exc: ValueError) -> bool:
    return str(exc).startswith(UNREACHABLE_PREFIXES)


def _try_solve(
    geometry: SteeringInputGeometry,
    pitman_angle_deg: float,
    guess: tuple[float, float],
) -> TwoSegmentSteeringSolution | None:
    try:
        if isinstance(geometry, TwoSegmentSteeringHardpoints3D):
            return solve_two_segment_steering_3d_analytic(
                geometry,
                pitman_angle_deg,
                guess,
            )
        return solve_two_segment_steering(geometry, pitman_angle_deg, guess)
    except ValueError as exc:
        if _is_unreachable_error(exc):
            return None
        raise


def _refine_limit(
    geometry: SteeringInputGeometry,
    low_good: TwoSegmentSteeringSolution,
    high_bad_angle: float,
    iterations: int,
) -> TwoSegmentSteeringSolution:
    good = low_good
    low_angle = low_good.pitman_angle_deg
    high_angle = high_bad_angle
    for _ in range(iterations):
        mid_angle = 0.5 * (low_angle + high_angle)
        guess = (good.left_wheel_angle_deg, good.right_wheel_angle_deg)
        mid = _try_solve(geometry, mid_angle, guess)
        if mid is None:
            high_angle = mid_angle
        else:
            good = mid
            low_angle = mid_angle
    return good


def _walk_pitman_direction(
    geometry: SteeringInputGeometry,
    direction: float,
    step_deg: float,
    max_abs_angle_deg: float,
    refinement_steps: int,
) -> list[TwoSegmentSteeringSolution]:
    if isinstance(geometry, TwoSegmentSteeringHardpoints3D):
        zero = solve_two_segment_steering_3d_analytic(geometry, 0.0)
    else:
        zero = solve_two_segment_steering(geometry, 0.0)
    states: list[TwoSegmentSteeringSolution] = []
    last = zero
    while abs(last.pitman_angle_deg + direction * step_deg) <= max_abs_angle_deg:
        angle = last.pitman_angle_deg + direction * step_deg
        guess = (last.left_wheel_angle_deg, last.right_wheel_angle_deg)
        solution = _try_solve(geometry, angle, guess)
        if solution is None:
            states.append(_refine_limit(geometry, last, angle, refinement_steps))
            break
        states.append(solution)
        last = solution
    return states


def _try_solve_three_segment(
    geometry: ThreeSegmentInputGeometry,
    left_bellcrank_angle_deg: float,
    guess: tuple[float, float, float],
) -> ThreeSegmentSteeringSolution | None:
    if isinstance(geometry, ThreeSegmentSteeringHardpoints3D):
        solution = solve_three_segment_steering_3d_analytic(
            geometry,
            left_bellcrank_angle_deg,
            guess,
        )
    else:
        solution = solve_three_segment_steering(
            geometry,
            left_bellcrank_angle_deg,
            guess,
        )
    if not solution.converged:
        return None
    return solution


def _refine_three_segment_limit(
    geometry: ThreeSegmentInputGeometry,
    low_good: ThreeSegmentSteeringSolution,
    high_bad_angle: float,
    iterations: int,
) -> ThreeSegmentSteeringSolution:
    good = low_good
    low_angle = low_good.left_bellcrank_angle_deg
    high_angle = high_bad_angle
    for _ in range(iterations):
        mid_angle = 0.5 * (low_angle + high_angle)
        guess = (
            good.right_bellcrank_angle_deg,
            good.left_wheel_angle_deg,
            good.right_wheel_angle_deg,
        )
        mid = _try_solve_three_segment(geometry, mid_angle, guess)
        if mid is None:
            high_angle = mid_angle
        else:
            good = mid
            low_angle = mid_angle
    return good


def _walk_left_bellcrank_direction(
    geometry: ThreeSegmentInputGeometry,
    direction: float,
    step_deg: float,
    max_abs_angle_deg: float,
    refinement_steps: int,
) -> list[ThreeSegmentSteeringSolution]:
    if isinstance(geometry, ThreeSegmentSteeringHardpoints3D):
        zero = solve_three_segment_steering_3d_analytic(geometry, 0.0)
    else:
        zero = solve_three_segment_steering(geometry, 0.0)
    states: list[ThreeSegmentSteeringSolution] = []
    last = zero
    while (
        abs(last.left_bellcrank_angle_deg + direction * step_deg)
        <= max_abs_angle_deg
    ):
        angle = last.left_bellcrank_angle_deg + direction * step_deg
        guess = (
            last.right_bellcrank_angle_deg,
            last.left_wheel_angle_deg,
            last.right_wheel_angle_deg,
        )
        solution = _try_solve_three_segment(geometry, angle, guess)
        if solution is None:
            states.append(
                _refine_three_segment_limit(
                    geometry,
                    last,
                    angle,
                    refinement_steps,
                )
            )
            break
        states.append(solution)
        last = solution
    return states


def estimate_two_segment_steering_limits(
    geometry: SteeringInputGeometry,
    *,
    step_deg: float = 1.0,
    max_abs_pitman_angle_deg: float = 180.0,
    refinement_steps: int = 24,
) -> SteeringTravelLimits:
    """Estimate current geometry left/right steering travel limits."""
    if isinstance(geometry, TwoSegmentSteeringHardpoints3D):
        zero = solve_two_segment_steering_3d_analytic(geometry, 0.0)
    else:
        zero = solve_two_segment_steering(geometry, 0.0)
    states = [zero]
    states.extend(
        _walk_pitman_direction(
            geometry,
            direction=-1.0,
            step_deg=step_deg,
            max_abs_angle_deg=max_abs_pitman_angle_deg,
            refinement_steps=refinement_steps,
        )
    )
    states.extend(
        _walk_pitman_direction(
            geometry,
            direction=1.0,
            step_deg=step_deg,
            max_abs_angle_deg=max_abs_pitman_angle_deg,
            refinement_steps=refinement_steps,
        )
    )
    return SteeringTravelLimits(
        left_turn=max(states, key=_average_wheel_angle),
        right_turn=min(states, key=_average_wheel_angle),
    )


def estimate_three_segment_steering_limits(
    geometry: ThreeSegmentInputGeometry,
    *,
    step_deg: float = 1.0,
    max_abs_left_bellcrank_angle_deg: float = 180.0,
    refinement_steps: int = 24,
) -> SteeringTravelLimits:
    """Estimate current three-segment geometry left/right steering travel limits."""
    if isinstance(geometry, ThreeSegmentSteeringHardpoints3D):
        zero = solve_three_segment_steering_3d_analytic(geometry, 0.0)
    else:
        zero = solve_three_segment_steering(geometry, 0.0)
    states = [zero]
    states.extend(
        _walk_left_bellcrank_direction(
            geometry,
            direction=-1.0,
            step_deg=step_deg,
            max_abs_angle_deg=max_abs_left_bellcrank_angle_deg,
            refinement_steps=refinement_steps,
        )
    )
    states.extend(
        _walk_left_bellcrank_direction(
            geometry,
            direction=1.0,
            step_deg=step_deg,
            max_abs_angle_deg=max_abs_left_bellcrank_angle_deg,
            refinement_steps=refinement_steps,
        )
    )
    return SteeringTravelLimits(
        left_turn=max(states, key=_average_wheel_angle),
        right_turn=min(states, key=_average_wheel_angle),
    )


def _outputs_from_limits(limits: SteeringTravelLimits) -> dict[str, float]:
    return {
        "max_left_turn_left_wheel_angle_deg": limits.left_turn.left_wheel_angle_deg,
        "max_left_turn_right_wheel_angle_deg": limits.left_turn.right_wheel_angle_deg,
        "max_right_turn_left_wheel_angle_deg": limits.right_turn.left_wheel_angle_deg,
        "max_right_turn_right_wheel_angle_deg": (
            limits.right_turn.right_wheel_angle_deg
        ),
    }


def steering_limit_outputs(geometry: SteeringInputGeometry) -> dict[str, float]:
    """Return scalar output rows for current two-segment geometry steering limits."""
    return _outputs_from_limits(estimate_two_segment_steering_limits(geometry))


def three_segment_steering_limit_outputs(
    geometry: ThreeSegmentInputGeometry,
) -> dict[str, float]:
    """Return scalar output rows for current three-segment steering limits."""
    return _outputs_from_limits(estimate_three_segment_steering_limits(geometry))
