"""Shared quasi-static KKT equilibrium solver."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Iterable, cast

import numpy as np

from ..core.constraints import Constraint, ConstraintSystem
from ..core.rigid_body import RigidBodyState
from ..core.spatial import wrench_global_to_local
from ..elements import ForceEvaluation


@dataclass(frozen=True)
class EquilibriumResult:
    """Static solve result and residual diagnostics."""

    state: RigidBodyState
    converged: bool
    iterations: int
    constraint_residual: float
    force_residual: float
    moment_residual: float
    multipliers: np.ndarray
    active_events: tuple[str, ...] = ()
    diagnostics: tuple[str, ...] = ()


@dataclass(frozen=True)
class EquilibriumSettings:
    """Newton/KKT tolerances and retry settings."""

    max_iterations: int = 40
    constraint_tolerance: float = 1e-8
    force_tolerance: float = 1e-6
    increment_tolerance: float = 1e-10
    finite_difference_step: float = 1e-6
    regularization: float = 1e-9
    line_search_steps: int = 8
    moment_scale: float = 1000.0
    # Local rotational increments are radians while translations are mm.  The
    # KKT solve uses these scales to keep the two coordinate groups balanced.
    rotation_coordinate_scale: float = 1.0e-3
    translation_finite_difference_multiplier: float = 1000.0
    max_translation_increment: float = 100.0
    max_rotation_increment: float = 0.25


def evaluate_generalized_forces(
    state: RigidBodyState,
    elements: Iterable[object],
    external_wrenches_global: dict[str, np.ndarray] | None = None,
    body_order: tuple[str, ...] | None = None,
) -> tuple[np.ndarray, tuple[ForceEvaluation, ...]]:
    """Assemble local generalized forces for all movable bodies."""
    order = body_order or tuple(
        name for name, body in state.bodies.items() if not body.fixed
    )
    global_wrenches = {name: np.zeros(6) for name in order}
    evaluations: list[ForceEvaluation] = []
    for element in elements:
        evaluator = getattr(element, "evaluate", None)
        if not callable(evaluator):
            continue
        evaluation = cast(Callable[[RigidBodyState], object], evaluator)(state)
        if not isinstance(evaluation, ForceEvaluation):
            raise TypeError(f"element {element!r} did not return ForceEvaluation")
        evaluations.append(evaluation)
        for body, wrench in evaluation.body_wrenches_global.items():
            if body in global_wrenches:
                global_wrenches[body] += wrench
    for body, wrench in (external_wrenches_global or {}).items():
        if body in global_wrenches:
            global_wrenches[body] += np.asarray(wrench, dtype=float)
    local = [
        wrench_global_to_local(state.pose(body), global_wrenches[body])
        for body in order
    ]
    return np.concatenate(local) if local else np.zeros(0), tuple(evaluations)


class EquilibriumSolver:
    """Damped Newton solver for mixed ideal-constraint and force balance."""

    def __init__(self, settings: EquilibriumSettings | None = None) -> None:
        self.settings = settings or EquilibriumSettings()

    def solve(
        self,
        initial_state: RigidBodyState,
        constraints: Iterable[Constraint] = (),
        elements: Iterable[object] = (),
        external_wrenches_global: dict[str, np.ndarray] | None = None,
    ) -> EquilibriumResult:
        system = ConstraintSystem(tuple(constraints))
        element_tuple = tuple(elements)
        body_order = tuple(
            name for name, body in initial_state.bodies.items() if not body.fixed
        )
        state = initial_state
        multipliers = np.zeros(len(system.residual(state)))
        diagnostics: list[str] = []
        active_events: set[str] = set()
        for iteration in range(1, self.settings.max_iterations + 1):
            constraint = system.residual(state)
            force, evaluations = evaluate_generalized_forces(
                state, element_tuple, external_wrenches_global, body_order
            )
            active_events.update(
                event for evaluation in evaluations if (event := evaluation.event)
            )
            constraint_norm = (
                float(np.max(np.abs(constraint))) if constraint.size else 0.0
            )
            jacobian = system.jacobian(state, body_order)
            stationarity = force + jacobian.T @ multipliers
            force_norm, moment_norm = _force_moment_norms(
                stationarity, self.settings.moment_scale
            )
            if (
                constraint_norm <= self.settings.constraint_tolerance
                and max(force_norm, moment_norm) <= self.settings.force_tolerance
            ):
                return EquilibriumResult(
                    state,
                    True,
                    iteration,
                    constraint_norm,
                    force_norm,
                    moment_norm,
                    multipliers,
                    tuple(sorted(active_events)),
                    tuple(diagnostics),
                )
            force_tangent = self._force_tangent(
                state, element_tuple, external_wrenches_global, body_order
            )
            size = force_tangent.shape[0]
            coordinate_scale = _coordinate_scale(
                len(body_order), self.settings.rotation_coordinate_scale
            )
            wrench_scale = _wrench_scale(len(body_order), self.settings.moment_scale)
            constraint_scale = _constraint_scale(jacobian)
            # Solve in length-equivalent rotational coordinates and normalize
            # force/moment and constraint rows.  This avoids the raw KKT matrix
            # mixing N, N-mm, mm and rad by six orders of magnitude.
            scaled_tangent = (
                wrench_scale[:, None]
                * (force_tangent + self.settings.regularization * np.eye(size))
                * coordinate_scale[None, :]
            )
            scaled_jacobian_transpose = wrench_scale[:, None] * jacobian.T
            scaled_jacobian = (
                constraint_scale[:, None] * jacobian * coordinate_scale[None, :]
            )
            kkt = np.block(
                [
                    [scaled_tangent, scaled_jacobian_transpose],
                    [scaled_jacobian, np.zeros((jacobian.shape[0], jacobian.shape[0]))],
                ]
            )
            rhs = -np.concatenate(
                (wrench_scale * stationarity, constraint_scale * constraint)
            )
            try:
                solution = np.linalg.solve(kkt, rhs)
            except np.linalg.LinAlgError:
                solution = np.linalg.lstsq(kkt, rhs, rcond=1e-12)[0]
                diagnostics.append("kkt_lstsq_fallback")
            increment = coordinate_scale * solution[:size]
            increment = _limit_equilibrium_increment(
                increment,
                body_order,
                self.settings.max_translation_increment,
                self.settings.max_rotation_increment,
            )
            multiplier_increment = solution[size:]
            trial_multipliers = multipliers + multiplier_increment
            new_stationarity = force + jacobian.T @ trial_multipliers
            if np.linalg.norm(increment) <= self.settings.increment_tolerance:
                new_force_norm, new_moment_norm = _force_moment_norms(
                    new_stationarity, self.settings.moment_scale
                )
                if (
                    constraint_norm <= self.settings.constraint_tolerance
                    and max(new_force_norm, new_moment_norm)
                    <= self.settings.force_tolerance
                ):
                    return EquilibriumResult(
                        state,
                        True,
                        iteration,
                        constraint_norm,
                        new_force_norm,
                        new_moment_norm,
                        trial_multipliers,
                        tuple(sorted(active_events)),
                        tuple(diagnostics),
                    )
                diagnostics.append("increment_stalled")
                break
            increments = {
                body: increment[index * 6 : (index + 1) * 6]
                for index, body in enumerate(body_order)
            }
            state, multipliers, accepted = self._line_search(
                state,
                increments,
                multipliers,
                multiplier_increment,
                system,
                element_tuple,
                external_wrenches_global,
                body_order,
            )
            if not accepted:
                diagnostics.append("line_search_failed")
                break
        constraint = system.residual(state)
        force, evaluations = evaluate_generalized_forces(
            state, element_tuple, external_wrenches_global, body_order
        )
        active_events.update(
            event for evaluation in evaluations if (event := evaluation.event)
        )
        jacobian = system.jacobian(state, body_order)
        stationarity = force + jacobian.T @ multipliers
        force_norm, moment_norm = _force_moment_norms(
            stationarity, self.settings.moment_scale
        )
        return EquilibriumResult(
            state,
            False,
            self.settings.max_iterations,
            float(np.max(np.abs(constraint))) if constraint.size else 0.0,
            force_norm,
            moment_norm,
            multipliers,
            tuple(sorted(active_events)),
            tuple(diagnostics),
        )

    def _force_tangent(
        self,
        state: RigidBodyState,
        elements: tuple[object, ...],
        external: dict[str, np.ndarray] | None,
        body_order: tuple[str, ...],
    ) -> np.ndarray:
        """Use central differences for the assembled force path."""
        size = 6 * len(body_order)
        # K-mode ideal-constraint assemblies commonly have no elastic force
        # elements.  Their force Jacobian is exactly zero; avoiding the full
        # 2*N central-difference sweep is both exact and important for the
        # 100/6600 state batch gates.
        if not elements and not external:
            return np.zeros((size, size))
        tangent = np.zeros((size, size))
        step = self.settings.finite_difference_step
        for column in range(size):
            body = body_order[column // 6]
            local_index = column % 6
            delta = np.zeros(6)
            delta[local_index] = step * (
                self.settings.translation_finite_difference_multiplier
                if local_index < 3
                else 1.0
            )
            plus = state.retract({body: delta})
            minus = state.retract({body: -delta})
            plus_force, _ = evaluate_generalized_forces(
                plus, elements, external, body_order
            )
            minus_force, _ = evaluate_generalized_forces(
                minus, elements, external, body_order
            )
            # Divide by the actual perturbation.  Translational coordinates
            # intentionally use a larger finite-difference step than angular
            # coordinates; using the base ``step`` here scales translational
            # stiffness by the multiplier and corrupts the KKT Newton step.
            tangent[:, column] = (plus_force - minus_force) / (2 * abs(delta[local_index]))
        return tangent

    def _line_search(
        self,
        state: RigidBodyState,
        increments: dict[str, np.ndarray],
        multipliers: np.ndarray,
        multiplier_increment: np.ndarray,
        system: ConstraintSystem,
        elements: tuple[object, ...],
        external: dict[str, np.ndarray] | None,
        body_order: tuple[str, ...],
    ) -> tuple[RigidBodyState, np.ndarray, bool]:
        """Accept a step against the full constrained KKT residual."""
        current_force, _ = evaluate_generalized_forces(
            state, elements, external, body_order
        )
        current_jacobian = system.jacobian(state, body_order)
        current_force_norm, current_moment_norm = _force_moment_norms(
            current_force + current_jacobian.T @ multipliers,
            self.settings.moment_scale,
        )
        current_constraint = system.residual(state)
        current_constraint_scale = _constraint_scale(current_jacobian)
        current_norm = max(
            float(np.max(np.abs(current_constraint) * current_constraint_scale))
            if current_constraint.size
            else 0.0,
            current_force_norm,
            current_moment_norm,
        )
        for exponent in range(self.settings.line_search_steps):
            factor = 0.5**exponent
            candidate = state.retract(
                {body: factor * delta for body, delta in increments.items()}
            )
            candidate_multipliers = multipliers + factor * multiplier_increment
            force, _ = evaluate_generalized_forces(
                candidate, elements, external, body_order
            )
            residual = system.residual(candidate)
            jacobian = system.jacobian(candidate, body_order)
            force_norm, moment_norm = _force_moment_norms(
                force + jacobian.T @ candidate_multipliers,
                self.settings.moment_scale,
            )
            candidate_constraint_scale = _constraint_scale(jacobian)
            candidate_norm = max(
                float(np.max(np.abs(residual) * candidate_constraint_scale))
                if residual.size
                else 0.0,
                force_norm,
                moment_norm,
            )
            if (
                candidate_norm < current_norm
                or candidate_norm < self.settings.force_tolerance
            ):
                return candidate, candidate_multipliers, True
            if (
                np.linalg.norm(np.concatenate(tuple(increments.values())))
                <= self.settings.increment_tolerance
            ):
                return candidate, candidate_multipliers, True
        return state, multipliers, False


def _force_moment_norms(
    vector: np.ndarray, moment_scale: float = 1.0
) -> tuple[float, float]:
    """Return separate force and moment infinity norms from stacked wrenches."""
    if not vector.size:
        return 0.0, 0.0
    blocks = vector.reshape((-1, 6))
    if moment_scale <= 0 or not np.isfinite(moment_scale):
        raise ValueError("moment_scale must be finite and positive")
    return float(np.max(np.abs(blocks[:, :3]))), float(
        np.max(np.abs(blocks[:, 3:])) / moment_scale
    )


def _coordinate_scale(body_count: int, rotation_scale: float) -> np.ndarray:
    """Return q = S z, with radians expressed as length-equivalent units."""
    if rotation_scale <= 0.0 or not np.isfinite(rotation_scale):
        raise ValueError("rotation_coordinate_scale must be finite and positive")
    return np.tile(np.array([1.0, 1.0, 1.0, rotation_scale, rotation_scale, rotation_scale]), body_count)


def _wrench_scale(body_count: int, moment_scale: float) -> np.ndarray:
    """Normalize stationarity rows from N/N-mm to force-equivalent rows."""
    if moment_scale <= 0.0 or not np.isfinite(moment_scale):
        raise ValueError("moment_scale must be finite and positive")
    return np.tile(np.array([1.0, 1.0, 1.0, 1.0 / moment_scale, 1.0 / moment_scale, 1.0 / moment_scale]), body_count)


def _constraint_scale(jacobian: np.ndarray) -> np.ndarray:
    """Normalize heterogeneous constraint rows by their local Jacobian size."""
    if jacobian.size == 0:
        return np.zeros(jacobian.shape[0])
    return 1.0 / np.maximum(np.linalg.norm(jacobian, axis=1), 1.0)


def _limit_equilibrium_increment(
    increment: np.ndarray,
    body_order: tuple[str, ...],
    translation_limit: float,
    rotation_limit: float,
) -> np.ndarray:
    """Apply a per-body trust region before the equilibrium line search."""
    result = np.asarray(increment, dtype=float).copy()
    for index, _body in enumerate(body_order):
        block = result[index * 6 : (index + 1) * 6]
        translation_norm = float(np.linalg.norm(block[:3]))
        rotation_norm = float(np.linalg.norm(block[3:]))
        factor = 1.0
        if translation_norm > translation_limit:
            factor = min(factor, translation_limit / translation_norm)
        if rotation_norm > rotation_limit:
            factor = min(factor, rotation_limit / rotation_norm)
        result[index * 6 : (index + 1) * 6] = factor * block
    return result
