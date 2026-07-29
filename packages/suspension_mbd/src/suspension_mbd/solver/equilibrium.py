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
            kkt = np.block(
                [
                    [
                        force_tangent + self.settings.regularization * np.eye(size),
                        jacobian.T,
                    ],
                    [jacobian, np.zeros((jacobian.shape[0], jacobian.shape[0]))],
                ]
            )
            rhs = -np.concatenate((stationarity, constraint))
            try:
                solution = np.linalg.solve(kkt, rhs)
            except np.linalg.LinAlgError:
                solution = np.linalg.lstsq(kkt, rhs, rcond=1e-12)[0]
                diagnostics.append("kkt_lstsq_fallback")
            increment = solution[:size]
            multipliers = solution[size:]
            new_stationarity = force + jacobian.T @ multipliers
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
                        multipliers,
                        tuple(sorted(active_events)),
                        tuple(diagnostics),
                    )
                diagnostics.append("increment_stalled")
                break
            increments = {
                body: increment[index * 6 : (index + 1) * 6]
                for index, body in enumerate(body_order)
            }
            state, accepted = self._line_search(
                state,
                increments,
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
            delta[local_index] = step
            plus = state.retract({body: delta})
            minus = state.retract({body: -delta})
            plus_force, _ = evaluate_generalized_forces(
                plus, elements, external, body_order
            )
            minus_force, _ = evaluate_generalized_forces(
                minus, elements, external, body_order
            )
            tangent[:, column] = (plus_force - minus_force) / (2 * step)
        return tangent

    def _line_search(
        self,
        state: RigidBodyState,
        increments: dict[str, np.ndarray],
        system: ConstraintSystem,
        elements: tuple[object, ...],
        external: dict[str, np.ndarray] | None,
        body_order: tuple[str, ...],
    ) -> tuple[RigidBodyState, bool]:
        current_force, _ = evaluate_generalized_forces(
            state, elements, external, body_order
        )
        current_norm = max(
            float(np.max(np.abs(system.residual(state))))
            if system.constraints
            else 0.0,
            float(np.max(np.abs(current_force))) if current_force.size else 0.0,
        )
        for exponent in range(self.settings.line_search_steps):
            factor = 0.5**exponent
            candidate = state.retract(
                {body: factor * delta for body, delta in increments.items()}
            )
            force, _ = evaluate_generalized_forces(
                candidate, elements, external, body_order
            )
            residual = system.residual(candidate)
            candidate_norm = max(
                float(np.max(np.abs(residual))) if residual.size else 0.0,
                float(np.max(np.abs(force))) if force.size else 0.0,
            )
            if (
                candidate_norm < current_norm
                or candidate_norm < self.settings.force_tolerance
            ):
                return candidate, True
            if (
                np.linalg.norm(np.concatenate(tuple(increments.values())))
                <= self.settings.increment_tolerance
            ):
                return candidate, True
        return state, False


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
