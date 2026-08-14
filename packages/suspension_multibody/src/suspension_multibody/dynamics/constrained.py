"""Coupled constrained rigid-body dynamics with Lagrange multipliers."""

from __future__ import annotations

from collections.abc import Callable, Iterable

import numpy as np

from ..core import Constraint, ConstraintSystem, wrench_global_to_local
from ..elements.elastic import _point_wrench
from ..model import BodyMassProperties, body_mass_properties, spatial_bias_wrench
from .forces import DynamicContext, evaluate_dynamic_element, sum_dynamic_wrenches
from .integrator import DynamicIntegrator, DynamicStepResult
from .state import DynamicRigidBodyState

ExternalWrenchFunction = Callable[[float, DynamicRigidBodyState], dict[str, np.ndarray]]


class ConstrainedDynamicIntegrator(DynamicIntegrator):
    """Integrate coupled body accelerations and ideal-constraint reactions."""

    def __init__(self, settings) -> None:
        super().__init__(settings)
        self._runtime_signature: tuple[str, ...] | None = None
        self._runtime_body_order: tuple[str, ...] = ()
        self._runtime_mass_properties: dict[str, BodyMassProperties] = {}
        self._runtime_mass = np.zeros((0, 0))
        self._runtime_mass_inverse = np.zeros((0, 0))
        self._runtime_mass_inverse_sqrt = np.zeros((0, 0))
        self._runtime_scaled_mass = np.zeros((0, 0))
        self._runtime_acceleration_scale = np.zeros(0)
        self._runtime_full_row_rank: bool | None = None
        self._runtime_constraint_system: ConstraintSystem | None = None
        self._reuse_constraint_linearization = settings.reuse_constraint_linearization
        # Populated only when a run cannot maintain the position manifold.
        # Keeping the last accepted state makes failures diagnosable without
        # weakening the acceptance tolerance or swallowing the exception.
        self.last_failure: dict[str, object] | None = None

    def integrate(
        self,
        initial_state: DynamicRigidBodyState,
        *,
        elements: Iterable[object] = (),
        constraints: Iterable[Constraint] = (),
        external_wrenches: ExternalWrenchFunction | None = None,
    ) -> tuple[DynamicStepResult, ...]:
        element_tuple = tuple(elements)
        constraint_tuple = tuple(constraints)
        self._runtime_constraint_system = ConstraintSystem(constraint_tuple)
        self._prepare_runtime(initial_state)
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
            current_time = previous_time
            sample_events: list[str] = []
            remaining = step
            trial_step = step if not constraint_tuple else min(
                step, self.settings.internal_step_size
            )
            while remaining > 1e-12:
                trial_step = min(trial_step, remaining)
                accepted = False
                last_error: Exception | None = None
                first_candidate_context: dict[str, object] | None = None
                last_candidate_context: dict[str, object] | None = None
                for attempt in range(12):
                    try:
                        candidate, accelerations, velocities, multipliers, events = (
                            self._advance_trial(
                                state,
                                current_time,
                                trial_step,
                                element_tuple,
                                constraint_tuple,
                                external_wrenches,
                            )
                        )
                        residual = self._constraint_norm(candidate, constraint_tuple)
                        finite = all(
                            np.all(np.isfinite(value))
                            for value in candidate.velocities.values()
                        )
                        if not finite or residual > self.settings.projection_failure_tolerance:
                            context = _candidate_failure_context(
                                candidate,
                                constraint_tuple,
                                accelerations,
                                velocities,
                                multipliers,
                                events,
                                finite,
                                residual,
                                self._velocity_norm(candidate, constraint_tuple),
                                current_time,
                                trial_step,
                                attempt + 1,
                            )
                            if first_candidate_context is None:
                                first_candidate_context = context
                            last_candidate_context = context
                        if finite and residual <= self.settings.projection_failure_tolerance:
                            accepted = True
                            break
                    except (FloatingPointError, OverflowError, ValueError) as error:
                        last_error = error
                    trial_step *= 0.5
                    if trial_step < self.settings.min_internal_step_size:
                        break
                if not accepted:
                    detail = f"; last error: {last_error}" if last_error else ""
                    self.last_failure = {
                        "time": float(current_time),
                        "attempted_step": float(trial_step),
                        "position_residual": float(
                            self._constraint_norm(state, constraint_tuple)
                        ),
                        "velocity_residual": float(
                            self._velocity_norm(state, constraint_tuple)
                        ),
                        "state": state,
                    }
                    if first_candidate_context is not None:
                        self.last_failure["first_candidate"] = first_candidate_context
                    if last_candidate_context is not None:
                        self.last_failure["last_candidate"] = last_candidate_context
                    raise RuntimeError(
                        "constrained integration cannot maintain the position manifold "
                        f"at t={current_time:.9g}, attempted step={trial_step:.9g}{detail}"
                    )
                state = candidate
                sample_events.extend(events)
                sample_events.extend(
                    _limit_events(
                        accelerations,
                        self.settings.max_linear_acceleration,
                        self.settings.max_angular_acceleration,
                        "acceleration",
                    )
                )
                sample_events.extend(
                    _limit_events(
                        velocities,
                        self.settings.max_linear_velocity,
                        self.settings.max_angular_velocity,
                        "velocity",
                    )
                )
                current_time += trial_step
                remaining -= trial_step
                if self.settings.adaptive_substepping and residual < 0.1 * self.settings.projection_failure_tolerance:
                    trial_step = min(
                        self.settings.internal_step_size,
                        trial_step * 1.5,
                    )
            events = tuple(dict.fromkeys(sample_events))
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

    def _prepare_runtime(self, state: DynamicRigidBodyState) -> None:
        """Cache state-invariant body mass data for one integration run."""
        order = state.body_order()
        properties = {
            body: body_mass_properties(state.pose_state.bodies[body]) for body in order
        }
        mass_blocks = [
            properties[body].spatial_inertia / self.settings.mass_matrix_scale
            for body in order
        ]
        scaled_mass = _block_diagonal(mass_blocks)
        mass_inverse = _block_diagonal(
            [np.linalg.inv(block) for block in mass_blocks]
        )
        inverse_sqrt_blocks = []
        for block in mass_blocks:
            eigenvalues, eigenvectors = np.linalg.eigh(block)
            if np.any(eigenvalues <= 0.0):
                raise ValueError("body spatial inertia must be positive definite")
            inverse_sqrt_blocks.append(
                eigenvectors
                @ np.diag(1.0 / np.sqrt(eigenvalues))
                @ eigenvectors.T
            )
        mass_diagonal = np.maximum(np.diag(scaled_mass), 1e-12)
        acceleration_scale = 1.0 / np.sqrt(mass_diagonal)
        self._runtime_signature = order
        self._runtime_body_order = order
        self._runtime_mass_properties = properties
        self._runtime_mass = scaled_mass
        self._runtime_mass_inverse = mass_inverse
        self._runtime_mass_inverse_sqrt = _block_diagonal(inverse_sqrt_blocks)
        self._runtime_acceleration_scale = acceleration_scale
        self._runtime_scaled_mass = (
            acceleration_scale[:, None]
            * scaled_mass
            * acceleration_scale[None, :]
        )
        self._runtime_full_row_rank = None

    def _ensure_runtime(self, state: DynamicRigidBodyState) -> None:
        order = state.body_order()
        if self._runtime_signature != order:
            self._prepare_runtime(state)

    def _advance_trial(
        self,
        state: DynamicRigidBodyState,
        time: float,
        step: float,
        elements: tuple[object, ...],
        constraints: tuple[Constraint, ...],
        external_wrenches: ExternalWrenchFunction | None,
    ) -> tuple[
        DynamicRigidBodyState,
        dict[str, np.ndarray],
        dict[str, np.ndarray],
        np.ndarray,
        tuple[str, ...],
    ]:
        accelerations, multipliers, events = self._coupled_accelerations(
            state, time, elements, constraints, external_wrenches
        )
        if self.settings.integrator == "semi_implicit_euler":
            velocities = {
                body: state.velocities[body] + accelerations[body] * step
                for body in state.body_order()
            }
            increments = {body: velocities[body] * step for body in state.body_order()}
        else:
            accelerations, velocities, increments, multipliers, corrector_events = (
                self._implicit_step(
                    state,
                    time,
                    step,
                    accelerations,
                    multipliers,
                    elements,
                    constraints,
                    external_wrenches,
                )
            )
            events = tuple(dict.fromkeys((*events, *corrector_events)))
        candidate = state.retract_unchecked(
            increments,
            velocity_updates=velocities,
            acceleration_updates=accelerations,
            multipliers=multipliers,
        )
        candidate = self._project_position(candidate, constraints)
        candidate = self._project_velocity(candidate, constraints)
        candidate, velocities, recovery_events = self._recover_velocity(candidate, constraints)
        events = tuple(dict.fromkeys((*events, *recovery_events)))
        return candidate, accelerations, velocities, multipliers, events

    def _recover_velocity(
        self,
        state: DynamicRigidBodyState,
        constraints: tuple[Constraint, ...],
    ) -> tuple[DynamicRigidBodyState, dict[str, np.ndarray], tuple[str, ...]]:
        """
        Bound only clearly divergent velocities and re-project the result.

        This is an explicit numerical recovery path, disabled by default.  It
        is used by long Adams runs as a circuit breaker: normal road speed and
        wheel spin remain untouched, while an unstable unsprung mode cannot
        grow until the pose projection itself fails.
        """
        velocities = {body: state.velocities[body].copy() for body in state.body_order()}
        if not self.settings.velocity_recovery_enabled:
            return state, velocities, ()
        events: list[str] = []
        linear_limit = self.settings.velocity_recovery_linear_limit
        angular_limit = self.settings.velocity_recovery_angular_limit
        for body, value in velocities.items():
            linear_norm = float(np.linalg.norm(value[:3]))
            angular_norm = float(np.linalg.norm(value[3:]))
            scale = 1.0
            if linear_norm > linear_limit:
                scale = min(scale, linear_limit / linear_norm)
                events.append(f"velocity_recovery_linear:{body}")
            if angular_norm > angular_limit:
                scale = min(scale, angular_limit / angular_norm)
                events.append(f"velocity_recovery_angular:{body}")
            if scale < 1.0:
                velocities[body] *= scale
        if events:
            state = state.retract({}, velocity_updates=velocities)
            state = self._project_velocity(state, constraints)
            velocities = {body: state.velocities[body].copy() for body in state.body_order()}
        return state, velocities, tuple(events)

    def _implicit_step(
        self,
        state: DynamicRigidBodyState,
        time: float,
        step: float,
        initial_acceleration: dict[str, np.ndarray],
        initial_multipliers: np.ndarray,
        elements: tuple[object, ...],
        constraints: tuple[Constraint, ...],
        external_wrenches: ExternalWrenchFunction | None,
        linearization_cache: dict[str, np.ndarray] | None = None,
    ) -> tuple[
        dict[str, np.ndarray],
        dict[str, np.ndarray],
        dict[str, np.ndarray],
        np.ndarray,
        tuple[str, ...],
    ]:
        """Perform a Newmark average-acceleration predictor/corrector step."""
        beta, gamma, alpha_m, alpha_f = _integration_parameters(self.settings)
        order = state.body_order()
        old_velocity = {body: state.velocities[body] for body in order}
        acceleration = {body: value.copy() for body, value in initial_acceleration.items()}
        multipliers = initial_multipliers
        events: list[str] = []
        evaluation_time = time + (1.0 - alpha_f) * step
        if self._reuse_constraint_linearization:
            linearization_cache = {} if linearization_cache is None else linearization_cache
        else:
            linearization_cache = None
        for _ in range(self.settings.max_corrector_iterations):
            increments = {
                body: step * old_velocity[body]
                + step**2 * ((0.5 - beta) * initial_acceleration[body] + beta * acceleration[body])
                for body in order
            }
            velocities = {
                body: old_velocity[body]
                + step * ((1.0 - gamma) * initial_acceleration[body] + gamma * acceleration[body])
                for body in order
            }
            if alpha_m == 0.0 and alpha_f == 0.0:
                candidate = state.retract_unchecked(
                    increments,
                    velocity_updates=velocities,
                    acceleration_updates=acceleration,
                    multipliers=multipliers,
                )
            else:
                # Generalized-alpha evaluates forces at the interpolated
                # configuration while retaining the full Newmark update for
                # the accepted state.
                candidate = state.retract_unchecked(
                    {
                        body: (1.0 - alpha_f) * increments[body]
                        for body in order
                    },
                    velocity_updates={
                        body: alpha_f * old_velocity[body]
                        + (1.0 - alpha_f) * velocities[body]
                        for body in order
                    },
                    acceleration_updates={
                        body: alpha_m * initial_acceleration[body]
                        + (1.0 - alpha_m) * acceleration[body]
                        for body in order
                    },
                    multipliers=multipliers,
                )
            evaluated, evaluated_multipliers, evaluated_events = self._coupled_accelerations(
                candidate,
                evaluation_time,
                elements,
                constraints,
                external_wrenches,
                linearization_cache=linearization_cache,
            )
            events.extend(evaluated_events)
            if alpha_m == 0.0:
                updated_acceleration = evaluated
            else:
                updated_acceleration = {
                    body: (evaluated[body] - alpha_m * initial_acceleration[body])
                    / (1.0 - alpha_m)
                    for body in order
                }
            delta = max(
                (
                    float(
                        np.max(
                            np.abs(updated_acceleration[body] - acceleration[body])
                        )
                    )
                    for body in order
                ),
                default=0.0,
            )
            acceleration = updated_acceleration
            multipliers = evaluated_multipliers
            if delta <= self.settings.event_tolerance:
                break
        increments = {
            body: step * old_velocity[body]
            + step**2 * ((0.5 - beta) * initial_acceleration[body] + beta * acceleration[body])
            for body in order
        }
        velocities = {
            body: old_velocity[body]
            + step * ((1.0 - gamma) * initial_acceleration[body] + gamma * acceleration[body])
            for body in order
        }
        return acceleration, velocities, increments, multipliers, tuple(events)

    def _coupled_accelerations(
        self,
        state: DynamicRigidBodyState,
        time: float,
        elements: tuple[object, ...],
        constraints: tuple[Constraint, ...],
        external_wrenches: ExternalWrenchFunction | None,
        linearization_cache: dict[str, np.ndarray] | None = None,
    ) -> tuple[dict[str, np.ndarray], np.ndarray, tuple[str, ...]]:
        self._ensure_runtime(state)
        order = self._runtime_body_order
        properties = self._runtime_mass_properties
        totals = {body: np.zeros(6) for body in order}
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
        for body in order:
            runtime = state.pose_state.bodies[body]
            center = state.pose_state.pose(body).transform_point(runtime.center_of_mass)
            totals[body] += _point_wrench(
                center,
                runtime.mass
                * self.settings.gravity.as_array()
                / self.settings.mass_matrix_scale,
            )
            if self.settings.global_velocity_damping > 0.0:
                velocity_global = state.pose_state.pose(body).rotation @ state.velocities[body][:3]
                totals[body][:3] -= self.settings.global_velocity_damping * velocity_global

        ramp_time = self.settings.initial_force_ramp_time
        if ramp_time > 0.0:
            load_scale = float(np.clip(time / ramp_time, 0.0, 1.0))
            for body in order:
                totals[body] *= load_scale

        bias = np.concatenate(
            [
                spatial_bias_wrench(
                    properties[body].spatial_inertia,
                    state.velocities[body],
                )
                / self.settings.mass_matrix_scale
                for body in order
            ]
        )
        force = np.concatenate(
            [
                wrench_global_to_local(state.pose_state.pose(body), totals[body])
                for body in order
            ]
        ) / self.settings.mass_matrix_scale
        system = self._runtime_constraint_system or ConstraintSystem(constraints)
        mass_inverse = self._runtime_mass_inverse
        cached_linearization = (
            linearization_cache
            if linearization_cache is not None and linearization_cache
            else None
        )
        if not constraints:
            acceleration_vector = mass_inverse @ (force - bias)
            multipliers = np.zeros(0)
        else:
            if cached_linearization is None:
                jacobian = system.jacobian(state.pose_state, order)
                row_scale = np.maximum(np.linalg.norm(jacobian, axis=1), 1.0)
                derivative_step = self.settings.constraint_derivative_step
                future_state = state.retract_unchecked(
                    {body: derivative_step * state.velocities[body] for body in order}
                )
                future_jacobian = system.jacobian(future_state.pose_state, order)
                if linearization_cache is not None:
                    linearization_cache.update(
                        {
                            "jacobian": jacobian,
                            "future_jacobian": future_jacobian,
                            "row_scale": row_scale,
                        }
                    )
            else:
                jacobian = cached_linearization["jacobian"]
                future_jacobian = cached_linearization["future_jacobian"]
                row_scale = cached_linearization["row_scale"]
            scaled_jacobian = jacobian / row_scale[:, None]
            velocity_vector = np.concatenate([state.velocities[body] for body in order])
            residual = system.residual(state.pose_state)
            velocity_residual = scaled_jacobian @ velocity_vector
            derivative_step = self.settings.constraint_derivative_step
            jdot_velocity = (
                ((future_jacobian - jacobian) / derivative_step) @ velocity_vector
            ) / row_scale
            stabilization = (
                jdot_velocity
                + 2.0 * self.settings.constraint_stabilization_alpha * velocity_residual
                + self.settings.constraint_stabilization_beta**2 * (residual / row_scale)
            )
            # Full-row-rank assemblies use the Schur complement directly.  It
            # has the same mass-weighted projection as the SVD formulation but
            # avoids a dense SVD at every corrector iteration.  Rank-deficient
            # assemblies retain the SVD fallback so redundant ideal-joint rows
            # are still handled without dropping independent physical rows.
            mass_inverse_sqrt_jacobian = (
                scaled_jacobian @ self._runtime_mass_inverse_sqrt
            )
            free_acceleration = mass_inverse @ (force - bias)
            scaled_rhs = scaled_jacobian @ free_acceleration + stabilization
            if self._runtime_full_row_rank is None:
                singular_values = np.linalg.svd(
                    mass_inverse_sqrt_jacobian, compute_uv=False
                )
                rank_tolerance = max(mass_inverse_sqrt_jacobian.shape) * np.finfo(float).eps * (
                    singular_values[0] if singular_values.size else 1.0
                )
                self._runtime_full_row_rank = bool(
                    singular_values.size == scaled_jacobian.shape[0]
                    and np.count_nonzero(singular_values > rank_tolerance)
                    == scaled_jacobian.shape[0]
                )
            if self._runtime_full_row_rank:
                schur = scaled_jacobian @ mass_inverse @ scaled_jacobian.T
                try:
                    multipliers_scaled = np.linalg.solve(schur, scaled_rhs)
                    correction = mass_inverse @ (scaled_jacobian.T @ multipliers_scaled)
                    acceleration_vector = free_acceleration - correction
                except np.linalg.LinAlgError:
                    # A geometry singularity can invalidate the initial rank
                    # decision.  Fall through to the robust SVD projection.
                    self._runtime_full_row_rank = False
            if not self._runtime_full_row_rank:
                row_basis, singular_values, right_basis = np.linalg.svd(
                    mass_inverse_sqrt_jacobian, full_matrices=False
                )
                rank_tolerance = max(mass_inverse_sqrt_jacobian.shape) * np.finfo(float).eps * (
                    singular_values[0] if singular_values.size else 1.0
                )
                rank = int(np.count_nonzero(singular_values > rank_tolerance))
                active_basis = row_basis[:, :rank]
                reciprocal = 1.0 / singular_values[:rank]
                row_coordinates = active_basis.T @ scaled_rhs
                correction = self._runtime_mass_inverse_sqrt @ (
                    right_basis[:rank, :].T @ (reciprocal * row_coordinates)
                )
                acceleration_vector = free_acceleration - correction
                multipliers_scaled = active_basis @ (
                    (reciprocal**2) * row_coordinates
                )
            # Convert multipliers for the scaled rows back to the original
            # constraint-row convention used by diagnostics and callers.
            multipliers = row_scale * multipliers_scaled
        accelerations = {
            body: acceleration_vector[index * 6 : (index + 1) * 6]
            for index, body in enumerate(order)
        }
        return accelerations, multipliers, tuple(event for event in events if event)


def _block_diagonal(blocks: list[np.ndarray]) -> np.ndarray:
    size = sum(block.shape[0] for block in blocks)
    result = np.zeros((size, size))
    offset = 0
    for block in blocks:
        width = block.shape[0]
        result[offset : offset + width, offset : offset + width] = block
        offset += width
    return result


def _candidate_failure_context(
    candidate: DynamicRigidBodyState,
    constraints: tuple[Constraint, ...],
    accelerations: dict[str, np.ndarray],
    velocities: dict[str, np.ndarray],
    multipliers: np.ndarray,
    events: tuple[str, ...],
    finite: bool,
    position_residual: float,
    velocity_residual: float,
    time: float,
    trial_step: float,
    attempt: int,
) -> dict[str, object]:
    """Capture discarded trial data without adding work to accepted steps."""
    residual_parts = [
        np.asarray(constraint.residual(candidate.pose_state), dtype=float).reshape(-1)
        for constraint in constraints
    ]
    residual_vector = (
        np.concatenate(residual_parts) if residual_parts else np.zeros(0, dtype=float)
    )
    rows: list[dict[str, object]] = []
    offset = 0
    for index, (constraint, values) in enumerate(zip(constraints, residual_parts)):
        stop = offset + values.size
        rows.append(
            {
                "constraint_index": index,
                "name": constraint.name,
                "start": offset,
                "stop": stop,
                "max_abs": float(np.max(np.abs(values))) if values.size else 0.0,
            }
        )
        offset = stop
    return {
        "time": float(time),
        "trial_step": float(trial_step),
        "attempt": attempt,
        "finite": finite,
        "position_residual": float(position_residual),
        "velocity_residual": float(velocity_residual),
        "constraint_residual": residual_vector.tolist(),
        "constraint_rows": rows,
        "accelerations": {body: value.tolist() for body, value in accelerations.items()},
        "velocities": {body: value.tolist() for body, value in velocities.items()},
        "multiplier_norm": float(np.linalg.norm(multipliers)),
        "events": tuple(events),
        "state": candidate,
    }


def _limit_events(
    values: dict[str, np.ndarray], linear_limit: float, angular_limit: float, kind: str
) -> tuple[str, ...]:
    events: list[str] = []
    for body, value in values.items():
        linear_norm = float(np.linalg.norm(value[:3]))
        angular_norm = float(np.linalg.norm(value[3:]))
        if linear_norm > linear_limit:
            events.append(f"{kind}_linear_limit:{body}")
        if angular_norm > angular_limit:
            events.append(f"{kind}_angular_limit:{body}")
    return tuple(events)


def _integration_parameters(settings) -> tuple[float, float, float, float]:
    if settings.integrator == "newmark":
        return settings.newmark_beta, settings.newmark_gamma, 0.0, 0.0
    if settings.integrator == "generalized_alpha":
        rho = settings.generalized_alpha_rho_inf
        alpha_m = (2.0 * rho - 1.0) / (rho + 1.0)
        alpha_f = rho / (rho + 1.0)
        gamma = 0.5 - alpha_m + alpha_f
        beta = 0.25 * (1.0 - alpha_m + alpha_f) ** 2
        return beta, gamma, alpha_m, alpha_f
    raise ValueError(f"unsupported implicit integrator: {settings.integrator}")


def _constraint_nullspace(jacobian: np.ndarray) -> np.ndarray:
    """Return a stable basis for the independent constrained accelerations."""
    _, singular_values, vh = np.linalg.svd(jacobian, full_matrices=True)
    tolerance = max(jacobian.shape) * np.finfo(float).eps * (
        singular_values[0] if singular_values.size else 1.0
    )
    rank = int(np.count_nonzero(singular_values > tolerance))
    return vh[rank:].T
