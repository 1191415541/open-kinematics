"""Constrained global-sensitivity analysis for suspension optimization."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations

import numpy as np


@dataclass(frozen=True)
class LinearConstraintParameterization:
    """Null-space parameterization of linear equality constraints."""

    null_basis: np.ndarray
    lower_bounds: np.ndarray
    upper_bounds: np.ndarray
    anchor: np.ndarray
    constraint_rank: int

    @property
    def direction_count(self) -> int:
        return int(self.null_basis.shape[1])

    @property
    def variable_count(self) -> int:
        return int(self.null_basis.shape[0])

    def map_to_variables(self, coordinates: np.ndarray) -> np.ndarray:
        coords = np.asarray(coordinates, dtype=np.float64)
        return self.anchor + self.null_basis @ coords

    def project_direction_to_variables(self, direction: np.ndarray) -> np.ndarray:
        return self.null_basis @ np.asarray(direction, dtype=np.float64)


@dataclass(frozen=True)
class MorrisVariableStat:
    variable_index: int
    mu_star: float
    sigma: float


@dataclass(frozen=True)
class SobolVariableStat:
    variable_index: int
    first_order: float
    total_order: float


def build_linear_constraint_parameterization(
    *,
    anchor: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    constraint_matrix: np.ndarray | None,
) -> LinearConstraintParameterization:
    """Build a null-space parameterization x = anchor + N z."""
    anchor = np.asarray(anchor, dtype=np.float64)
    lower = np.asarray(lower, dtype=np.float64)
    upper = np.asarray(upper, dtype=np.float64)
    if constraint_matrix is None or constraint_matrix.size == 0:
        null_basis = np.eye(anchor.size, dtype=np.float64)
        return LinearConstraintParameterization(
            null_basis=null_basis,
            lower_bounds=lower - anchor,
            upper_bounds=upper - anchor,
            anchor=anchor,
            constraint_rank=0,
        )

    matrix = np.asarray(constraint_matrix, dtype=np.float64)
    _u, singular_values, vh = np.linalg.svd(matrix, full_matrices=True)
    tol = max(matrix.shape) * np.max(singular_values, initial=0.0) * np.finfo(float).eps
    rank = int(np.sum(singular_values > tol))
    null_basis = vh[rank:].T
    if null_basis.size == 0:
        null_basis = np.zeros((anchor.size, 0), dtype=np.float64)
    return LinearConstraintParameterization(
        null_basis=null_basis,
        lower_bounds=lower - anchor,
        upper_bounds=upper - anchor,
        anchor=anchor,
        constraint_rank=rank,
    )


def feasible_direction_scales(
    parameterization: LinearConstraintParameterization,
    *,
    direction: np.ndarray,
) -> tuple[float, float]:
    """Return feasible scalar interval for anchor + t * direction."""
    direction_x = parameterization.project_direction_to_variables(direction)
    lower_t = -np.inf
    upper_t = np.inf
    for component, lower, upper in zip(
        direction_x,
        parameterization.lower_bounds,
        parameterization.upper_bounds,
        strict=True,
    ):
        if abs(component) <= 1e-12:
            continue
        first = lower / component
        second = upper / component
        lower_t = max(lower_t, min(first, second))
        upper_t = min(upper_t, max(first, second))
    return float(lower_t), float(upper_t)


def feasible_direction_step(
    parameterization: LinearConstraintParameterization,
    *,
    direction: np.ndarray,
    step_fraction: float = 0.3,
) -> float:
    """Choose one feasible perturbation size along a reduced direction."""
    lower_t, upper_t = feasible_direction_scales(parameterization, direction=direction)
    positive = max(0.0, upper_t)
    negative = max(0.0, -lower_t)
    span = max(positive, negative)
    if span <= 1e-9:
        return 0.0
    return float(max(span * step_fraction, min(span, 1e-3)))


def run_morris_screening(
    *,
    parameterization: LinearConstraintParameterization,
    evaluate_objective,
    trajectories: int = 6,
    step_fraction: float = 0.3,
) -> tuple[list[MorrisVariableStat], np.ndarray]:
    """Run a reduced-space Morris screening and map it to original variables."""
    direction_count = parameterization.direction_count
    variable_count = parameterization.variable_count
    if direction_count == 0:
        return (
            [
                MorrisVariableStat(variable_index=index, mu_star=0.0, sigma=0.0)
                for index in range(variable_count)
            ],
            np.zeros(variable_count, dtype=np.float64),
        )

    elementary_effects: list[list[float]] = [[] for _ in range(variable_count)]
    reduced_basis = np.eye(direction_count, dtype=np.float64)

    for trajectory in range(max(1, trajectories)):
        order = np.roll(np.arange(direction_count), trajectory % direction_count)
        for reduced_index in order:
            direction = reduced_basis[reduced_index]
            step = feasible_direction_step(
                parameterization,
                direction=direction,
                step_fraction=step_fraction,
            )
            if step <= 0.0:
                continue
            plus = parameterization.map_to_variables(direction * step)
            minus = parameterization.map_to_variables(-direction * step)
            effect = (float(evaluate_objective(plus)) - float(evaluate_objective(minus))) / (
                2.0 * step
            )
            projected = np.abs(parameterization.project_direction_to_variables(direction))
            if float(np.sum(projected)) <= 1e-12:
                continue
            weights = projected / float(np.sum(projected))
            for variable_index in range(variable_count):
                if weights[variable_index] <= 1e-12:
                    continue
                elementary_effects[variable_index].append(float(effect * weights[variable_index]))

    stats: list[MorrisVariableStat] = []
    mu_stars = np.zeros(variable_count, dtype=np.float64)
    for variable_index, effects in enumerate(elementary_effects):
        if effects:
            effect_values = np.asarray(effects, dtype=np.float64)
            mu_star = float(np.mean(np.abs(effect_values)))
            sigma = float(np.std(effect_values))
        else:
            mu_star = 0.0
            sigma = 0.0
        mu_stars[variable_index] = mu_star
        stats.append(
            MorrisVariableStat(
                variable_index=variable_index,
                mu_star=mu_star,
                sigma=sigma,
            )
        )
    return stats, mu_stars


def run_pairwise_sobol_screening(
    *,
    parameterization: LinearConstraintParameterization,
    evaluate_objective,
    direction_indices: tuple[int, ...],
    base_samples: int = 8,
    rng_seed: int = 0,
) -> list[SobolVariableStat]:
    """Approximate first/total Sobol indices on selected reduced directions."""
    if not direction_indices:
        return []
    rng = np.random.default_rng(rng_seed)
    direction_count = parameterization.direction_count
    reduced_basis = np.eye(direction_count, dtype=np.float64)
    selected = tuple(direction_indices)
    values_by_direction: dict[int, list[float]] = {index: [] for index in selected}
    total_values: list[float] = []

    for _sample in range(max(1, base_samples)):
        active_values = np.zeros(direction_count, dtype=np.float64)
        for index in selected:
            direction = reduced_basis[index]
            lower_t, upper_t = feasible_direction_scales(
                parameterization,
                direction=direction,
            )
            low = max(lower_t, -1.0)
            high = min(upper_t, 1.0)
            if high <= low:
                value = 0.0
            else:
                value = float(rng.uniform(low, high))
            active_values[index] = value
        point = parameterization.map_to_variables(active_values)
        total_value = float(evaluate_objective(point))
        total_values.append(total_value)
        for index in selected:
            isolated = np.zeros(direction_count, dtype=np.float64)
            isolated[index] = active_values[index]
            isolated_point = parameterization.map_to_variables(isolated)
            values_by_direction[index].append(float(evaluate_objective(isolated_point)))

    total_variance = float(np.var(np.asarray(total_values, dtype=np.float64)))
    if total_variance <= 1e-12:
        total_variance = 1.0

    stats: list[SobolVariableStat] = []
    for index in selected:
        isolated_values = np.asarray(values_by_direction[index], dtype=np.float64)
        first = float(np.var(isolated_values) / total_variance)
        remainder_values: list[float] = []
        for _sample in range(max(1, base_samples)):
            point = np.zeros(direction_count, dtype=np.float64)
            for other in selected:
                if other == index:
                    continue
                direction = reduced_basis[other]
                lower_t, upper_t = feasible_direction_scales(
                    parameterization,
                    direction=direction,
                )
                low = max(lower_t, -1.0)
                high = min(upper_t, 1.0)
                if high <= low:
                    value = 0.0
                else:
                    value = float(rng.uniform(low, high))
                point[other] = value
            remainder_values.append(float(evaluate_objective(parameterization.map_to_variables(point))))
        conditional_variance = float(np.var(np.asarray(remainder_values, dtype=np.float64)))
        total = float(max(0.0, 1.0 - conditional_variance / total_variance))
        stats.append(
            SobolVariableStat(
                variable_index=index,
                first_order=max(0.0, first),
                total_order=max(first, total),
            )
        )
    return stats


def pick_reduced_directions_from_morris(
    *,
    parameterization: LinearConstraintParameterization,
    morris_mu_stars: np.ndarray,
    max_directions: int = 3,
) -> tuple[int, ...]:
    """Pick reduced directions whose projected loadings best explain Morris scores."""
    direction_count = parameterization.direction_count
    if direction_count == 0:
        return ()
    direction_scores: list[tuple[float, int]] = []
    for direction_index in range(direction_count):
        projected = np.abs(parameterization.null_basis[:, direction_index])
        score = float(np.dot(projected, morris_mu_stars))
        direction_scores.append((score, direction_index))
    direction_scores.sort(reverse=True)
    selected = [
        direction_index
        for score, direction_index in direction_scores
        if score > 1e-12
    ]
    if not selected:
        return tuple(range(min(max_directions, direction_count)))
    return tuple(selected[: max(1, min(max_directions, len(selected)))])
