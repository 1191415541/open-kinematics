"""Forward and inverse static trim helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Literal

from ..schema import MassSpec
from .equilibrium import EquilibriumResult, EquilibriumSolver


@dataclass(frozen=True)
class TrimTarget:
    """One inverse-trim target."""

    kind: Literal["ride_height", "axle_load", "wheel_load"]
    value: float


@dataclass(frozen=True)
class TrimResult:
    """Trim solution and target residual."""

    equilibrium: EquilibriumResult | None
    preloads: dict[str, float]
    target: TrimTarget | None
    target_error: float
    converged: bool


def target_wheel_load(mass: MassSpec, *, gravity: float = 9810.0) -> float:
    """Compute a default front-wheel load from sprung mass and ratio."""
    axle_mass = mass.axle_sprung_mass or mass.sprung_mass
    unsprung = axle_mass * mass.front_unsprung_ratio
    return (axle_mass + unsprung) * gravity / 2.0


class TrimSolver:
    """Forward equilibrium and scalar inverse preload solver."""

    def __init__(self, equilibrium_solver: EquilibriumSolver | None = None) -> None:
        self.equilibrium_solver = equilibrium_solver or EquilibriumSolver()

    def forward(
        self,
        state,
        constraints=(),
        elements=(),
        external_wrenches_global=None,
        preloads: dict[str, float] | None = None,
    ) -> TrimResult:
        """Solve static equilibrium using the supplied spring preload state."""
        result = self.equilibrium_solver.solve(
            state, constraints, elements, external_wrenches_global
        )
        return TrimResult(result, preloads or {}, None, 0.0, result.converged)

    def inverse_scalar(
        self,
        response: Callable[[float], float],
        target: TrimTarget,
        initial_preload: float = 0.0,
        *,
        stiffness_hint: float = 1.0,
        tolerance: float = 1e-8,
        max_iterations: int = 30,
    ) -> TrimResult:
        """Solve one preload from a scalar ride-height/axle/wheel response."""
        preload = float(initial_preload)
        for _ in range(max_iterations):
            error = response(preload) - target.value
            if abs(error) <= tolerance:
                return TrimResult(None, {"preload": preload}, target, error, True)
            step = max(abs(preload) * 1e-5, 1e-4)
            slope = (response(preload + step) - response(preload - step)) / (2 * step)
            if abs(slope) < 1e-12:
                slope = stiffness_hint
            preload -= error / slope
        error = response(preload) - target.value
        return TrimResult(
            None, {"preload": preload}, target, error, abs(error) <= tolerance
        )
