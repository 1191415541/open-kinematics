"""
Travel limit estimation for two-segment steering geometry.
"""

from __future__ import annotations

from dataclasses import dataclass

from suspension_kinematics.steering.geometry import (
    ThreeSegmentSteeringGeometry,
    ThreeSegmentSteeringHardpoints3D,
    ThreeSegmentSteeringSolution,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
)
from suspension_kinematics.steering.three_segment import (
    solve_three_segment_steering,
    solve_three_segment_steering_3d_analytic,
)
from suspension_kinematics.steering.two_segment import (
    solve_two_segment_rack_and_pinion_3d_analytic,
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


@dataclass(frozen=True)
class RackSteeringTravelLimits:
    """Reachable rack-travel endpoints and their steering states."""

    minimum_displacement_mm: float
    maximum_displacement_mm: float
    minimum_state: TwoSegmentSteeringSolution
    maximum_state: TwoSegmentSteeringSolution

    def as_steering_limits(self) -> SteeringTravelLimits:
        """Classify the two rack endpoints by average roadwheel turn direction."""
        states = (self.minimum_state, self.maximum_state)
        return SteeringTravelLimits(
            left_turn=max(states, key=_average_wheel_angle),
            right_turn=min(states, key=_average_wheel_angle),
        )


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


def _try_solve_rack(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    rack_displacement_mm: float,
    guess: tuple[float, float],
) -> TwoSegmentSteeringSolution | None:
    try:
        return solve_two_segment_rack_and_pinion_3d_analytic(
            hardpoints,
            rack_displacement_mm,
            guess,
        )
    except ValueError as exc:
        if _is_unreachable_error(exc):
            return None
        raise


def _refine_rack_limit(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    good_displacement_mm: float,
    good_state: TwoSegmentSteeringSolution,
    bad_displacement_mm: float,
    iterations: int,
) -> tuple[float, TwoSegmentSteeringSolution]:
    good_displacement = good_displacement_mm
    bad_displacement = bad_displacement_mm
    state = good_state
    for _ in range(iterations):
        candidate_displacement = 0.5 * (good_displacement + bad_displacement)
        candidate = _try_solve_rack(
            hardpoints,
            candidate_displacement,
            (state.left_wheel_angle_deg, state.right_wheel_angle_deg),
        )
        if candidate is None:
            bad_displacement = candidate_displacement
        else:
            good_displacement = candidate_displacement
            state = candidate
    return good_displacement, state


def _walk_rack_direction(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    direction: float,
    step_mm: float,
    max_abs_displacement_mm: float,
    refinement_steps: int,
) -> tuple[float, TwoSegmentSteeringSolution]:
    displacement = 0.0
    state = solve_two_segment_rack_and_pinion_3d_analytic(hardpoints, displacement)
    while abs(displacement + direction * step_mm) <= max_abs_displacement_mm:
        candidate_displacement = displacement + direction * step_mm
        candidate = _try_solve_rack(
            hardpoints,
            candidate_displacement,
            (state.left_wheel_angle_deg, state.right_wheel_angle_deg),
        )
        if candidate is None:
            return _refine_rack_limit(
                hardpoints,
                displacement,
                state,
                candidate_displacement,
                refinement_steps,
            )
        displacement = candidate_displacement
        state = candidate
    return displacement, state


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
        abs(last.left_bellcrank_angle_deg + direction * step_deg) <= max_abs_angle_deg
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


def estimate_rack_and_pinion_steering_limits(
    hardpoints: TwoSegmentSteeringHardpoints3D,
    *,
    step_mm: float = 1.0,
    max_abs_displacement_mm: float = 250.0,
    refinement_steps: int = 24,
) -> RackSteeringTravelLimits:
    """Estimate the continuous reachable rack-travel range for current geometry."""
    if step_mm <= 0.0:
        raise ValueError("rack limit step_mm must be positive")
    if max_abs_displacement_mm <= 0.0:
        raise ValueError("rack max_abs_displacement_mm must be positive")
    negative_displacement, negative_state = _walk_rack_direction(
        hardpoints,
        direction=-1.0,
        step_mm=step_mm,
        max_abs_displacement_mm=max_abs_displacement_mm,
        refinement_steps=refinement_steps,
    )
    positive_displacement, positive_state = _walk_rack_direction(
        hardpoints,
        direction=1.0,
        step_mm=step_mm,
        max_abs_displacement_mm=max_abs_displacement_mm,
        refinement_steps=refinement_steps,
    )
    return RackSteeringTravelLimits(
        minimum_displacement_mm=negative_displacement,
        maximum_displacement_mm=positive_displacement,
        minimum_state=negative_state,
        maximum_state=positive_state,
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


def rack_and_pinion_steering_limit_outputs(
    hardpoints: TwoSegmentSteeringHardpoints3D,
) -> dict[str, float]:
    """Return roadwheel travel-limit outputs for a rack-and-pinion system."""
    limits = estimate_rack_and_pinion_steering_limits(hardpoints)
    return _outputs_from_limits(limits.as_steering_limits())
