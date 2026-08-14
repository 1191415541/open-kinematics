"""First-pass constrained time integration."""

from __future__ import annotations

from collections.abc import Callable, Iterable
from dataclasses import dataclass

import numpy as np

from ..core import Constraint, ConstraintSystem, wrench_global_to_local
from ..elements.elastic import _point_wrench
from ..model import body_mass_properties
from ..schema import DynamicSolverSettings
from .forces import DynamicContext, evaluate_dynamic_element, sum_dynamic_wrenches
from .state import DynamicRigidBodyState

ExternalWrenchFunction = Callable[[float, DynamicRigidBodyState], dict[str, np.ndarray]]


@dataclass(frozen=True)
class DynamicStepResult:
    """One integration step result."""

    time: float
    state: DynamicRigidBodyState
    constraint_residual: float
    velocity_residual: float
    events: tuple[str, ...] = ()


class DynamicIntegrator:
    """Semi-implicit constrained integrator for first-pass dynamic analysis."""

    def __init__(self, settings: DynamicSolverSettings) -> None:
        self.settings = settings

    def integrate(
        self,
        initial_state: DynamicRigidBodyState,
        *,
        elements: Iterable[object] = (),
        constraints: Iterable[Constraint] = (),
        external_wrenches: ExternalWrenchFunction | None = None,
    ) -> tuple[DynamicStepResult, ...]:
        """Integrate from ``settings.start_time`` through ``settings.end_time``."""
        element_tuple = tuple(elements)
        constraint_tuple = tuple(constraints)
        state = self._project_position(initial_state, constraint_tuple)
        state = self._project_velocity(state, constraint_tuple)
        times = self._time_grid()
        results = [
            DynamicStepResult(
                times[0],
                state,
                self._constraint_norm(state, constraint_tuple),
                self._velocity_norm(state, constraint_tuple),
            )
        ]
        for previous_time, time in zip(times, times[1:]):
            step = time - previous_time
            accelerations, events = self._accelerations(
                state, previous_time, element_tuple, external_wrenches
            )
            velocities = {
                body: state.velocities[body] + accelerations[body] * step
                for body in state.body_order()
            }
            increments = {body: velocities[body] * step for body in state.body_order()}
            state = state.retract(
                increments,
                velocity_updates=velocities,
                acceleration_updates=accelerations,
            )
            state = self._project_position(state, constraint_tuple)
            state = self._project_velocity(state, constraint_tuple)
            results.append(
                DynamicStepResult(
                    time,
                    state,
                    self._constraint_norm(state, constraint_tuple),
                    self._velocity_norm(state, constraint_tuple),
                    events,
                )
            )
        return tuple(results)

    def _time_grid(self) -> tuple[float, ...]:
        start = self.settings.start_time
        end = self.settings.end_time
        step = self.settings.step_size
        count = int(np.floor((end - start) / step + 1e-12))
        values = [start + index * step for index in range(count + 1)]
        if values[-1] < end - 1e-12:
            values.append(end)
        return tuple(float(value) for value in values)

    def _accelerations(
        self,
        state: DynamicRigidBodyState,
        time: float,
        elements: tuple[object, ...],
        external_wrenches: ExternalWrenchFunction | None,
    ) -> tuple[dict[str, np.ndarray], tuple[str, ...]]:
        totals = {body: np.zeros(6) for body in state.body_order()}
        events: list[str] = []
        context = DynamicContext(self.settings.allow_static_element_downgrade)
        evaluations = tuple(
            evaluate_dynamic_element(element, state, time, context) for element in elements
        )
        for evaluation in evaluations:
            events.extend(evaluation.events)
        for body, wrench in sum_dynamic_wrenches(evaluations).items():
            if body in totals:
                totals[body] += wrench
        if external_wrenches is not None:
            for body, wrench in external_wrenches(time, state).items():
                if body in totals:
                    totals[body] += np.asarray(wrench, dtype=float)
        for body in state.body_order():
            runtime = state.pose_state.bodies[body]
            center = state.pose_state.pose(body).transform_point(runtime.center_of_mass)
            gravity_force = (
                runtime.mass
                * self.settings.gravity.as_array()
                / self.settings.mass_matrix_scale
            )
            totals[body] += _point_wrench(center, gravity_force)
            if self.settings.global_velocity_damping > 0.0:
                velocity_global = state.pose_state.pose(body).rotation @ state.velocities[body][:3]
                totals[body][:3] -= self.settings.global_velocity_damping * velocity_global
        accelerations: dict[str, np.ndarray] = {}
        for body in state.body_order():
            runtime = state.pose_state.bodies[body]
            local_wrench = wrench_global_to_local(state.pose_state.pose(body), totals[body])
            accelerations[body] = np.linalg.solve(
                body_mass_properties(runtime).spatial_inertia
                / self.settings.mass_matrix_scale,
                local_wrench,
            )
        return accelerations, tuple(event for event in events if event)

    def _project_position(
        self,
        state: DynamicRigidBodyState,
        constraints: tuple[Constraint, ...],
    ) -> DynamicRigidBodyState:
        if not constraints:
            return state
        pose_state = state.pose_state
        system = ConstraintSystem(constraints)
        order = state.body_order()
        for _ in range(self.settings.projection_max_iterations):
            residual = system.residual(pose_state)
            if residual.size == 0:
                break
            jacobian = system.jacobian(pose_state, order)
            row_scale = np.maximum(np.linalg.norm(jacobian, axis=1), 1.0)
            residual_norm = float(np.max(np.abs(residual) / row_scale))
            if residual_norm <= self.settings.constraint_tolerance:
                break
            increment = np.linalg.lstsq(
                jacobian / row_scale[:, None],
                -residual / row_scale,
                rcond=1e-12,
            )[0]
            increments = _limit_pose_increments(
                increment,
                order,
                self.settings.projection_translation_limit,
                self.settings.projection_rotation_limit,
            )
            if not np.any(increments):
                break
            accepted = False
            for backtrack in range(self.settings.projection_backtracking):
                factor = 0.5**backtrack
                candidate = pose_state.retract(
                    {
                        body: factor * increments[index * 6 : (index + 1) * 6]
                        for index, body in enumerate(order)
                    }
                )
                candidate_residual = system.residual(candidate)
                candidate_norm = float(np.max(np.abs(candidate_residual) / row_scale))
                if candidate_norm < residual_norm:
                    pose_state = candidate
                    accepted = True
                    break
            if not accepted:
                break
        return DynamicRigidBodyState(
            pose_state,
            state.velocities,
            state.accelerations,
            state.multipliers,
            state.internal_states,
        )

    def _project_velocity(
        self,
        state: DynamicRigidBodyState,
        constraints: tuple[Constraint, ...],
    ) -> DynamicRigidBodyState:
        if not constraints:
            return state
        order = state.body_order()
        system = ConstraintSystem(constraints)
        jacobian = system.jacobian(state.pose_state, order)
        if jacobian.size == 0:
            return state
        velocity = np.concatenate([state.velocities[body] for body in order])
        residual = jacobian @ velocity
        if float(np.max(np.abs(residual))) <= self.settings.velocity_tolerance:
            return state
        row_scale = np.maximum(np.linalg.norm(jacobian, axis=1), 1.0)
        correction = np.linalg.lstsq(
            jacobian / row_scale[:, None],
            -residual / row_scale,
            rcond=1e-12,
        )[0]
        corrected = velocity + correction
        return state.retract(
            {},
            velocity_updates={
                body: corrected[index * 6 : (index + 1) * 6]
                for index, body in enumerate(order)
            },
        )

    def _constraint_norm(
        self, state: DynamicRigidBodyState, constraints: tuple[Constraint, ...]
    ) -> float:
        if not constraints:
            return 0.0
        residual = ConstraintSystem(constraints).residual(state.pose_state)
        return float(np.max(np.abs(residual))) if residual.size else 0.0

    def _velocity_norm(
        self, state: DynamicRigidBodyState, constraints: tuple[Constraint, ...]
    ) -> float:
        if not constraints:
            return 0.0
        order = state.body_order()
        jacobian = ConstraintSystem(constraints).jacobian(state.pose_state, order)
        if jacobian.size == 0:
            return 0.0
        velocity = np.concatenate([state.velocities[body] for body in order])
        residual = jacobian @ velocity
        return float(np.max(np.abs(residual))) if residual.size else 0.0


def _limit_pose_increments(
    increment: np.ndarray,
    order: tuple[str, ...],
    translation_limit: float,
    rotation_limit: float,
) -> np.ndarray:
    """Apply a trust-region limit per body without changing the correction direction."""
    result = np.asarray(increment, dtype=float).copy()
    for index, _body in enumerate(order):
        block = result[index * 6 : (index + 1) * 6]
        translation_norm = float(np.linalg.norm(block[:3]))
        rotation_norm = float(np.linalg.norm(block[3:]))
        scale = 1.0
        if translation_norm > translation_limit:
            scale = min(scale, translation_limit / translation_norm)
        if rotation_norm > rotation_limit:
            scale = min(scale, rotation_limit / rotation_norm)
        result[index * 6 : (index + 1) * 6] = scale * block
    return result
