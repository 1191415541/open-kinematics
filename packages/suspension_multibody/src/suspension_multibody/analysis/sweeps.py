"""Deterministic K-mode sweep plans."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np

from ..model import FrontAxleAssembly
from ..schema import SixVector
from .c_mode import CModeSolver, CState, LeftRightMode, LoadPath
from .k_mode import KModeSolver, KState
from .k_reference import KReferenceCache


@dataclass(frozen=True)
class KGrid:
    """Two-axis wheel-travel/rack grid."""

    wheel_values: tuple[float, ...] = tuple(
        float(v) for v in np.linspace(-40.0, 40.0, 10)
    )
    rack_values: tuple[float, ...] = tuple(
        float(v) for v in np.linspace(-10.0, 10.0, 10)
    )
    pattern: Literal["parallel", "opposite", "single_left", "single_right"] = "parallel"

    @property
    def state_count(self) -> int:
        return len(self.wheel_values) * len(self.rack_values)


@dataclass(frozen=True)
class CGrid:
    """C-mode paths evaluated at every K-grid work point."""

    k_grid: KGrid = field(default_factory=KGrid)
    paths: tuple[LoadPath, ...] = field(default_factory=LoadPath.standard)
    side_mode: LeftRightMode = "symmetric"
    drive: Literal["contact_point", "wheel_center"] = "wheel_center"

    @property
    def state_count(self) -> int:
        return self.k_grid.state_count * sum(path.levels for path in self.paths)


def run_k_grid(
    assembly: FrontAxleAssembly,
    grid: KGrid | None = None,
    *,
    drive: Literal["contact_point", "wheel_center"] = "wheel_center",
    solver: KModeSolver | None = None,
) -> tuple[KState, ...]:
    """Run a deterministic row-major K grid with within-row continuation."""
    plan = grid or KGrid()
    engine = solver or KModeSolver()
    states: list[KState] = []
    for row, wheel in enumerate(plan.wheel_values):
        previous = None
        for column, rack in enumerate(plan.rack_values):
            left = wheel
            right = {
                "parallel": wheel,
                "opposite": -wheel,
                "single_left": 0.0,
                "single_right": wheel,
            }[plan.pattern]
            result = engine.solve(
                assembly,
                wheel_travel_left=left,
                wheel_travel_right=right,
                rack_displacement=rack,
                drive=drive,
                case_id=f"k-{row:02d}-{column:02d}",
                initial_state=previous,
            )
            # Continuation is the fast path, but a previous branch can become
            # ill-conditioned at a steering/jounce corner.  Retry that point
            # from the validated neutral pose before exposing a failed state;
            # this keeps the sweep deterministic and provides the requested
            # failure recovery without changing the case definition.
            if not result.equilibrium.converged and previous is not None:
                retry = engine.solve(
                    assembly,
                    wheel_travel_left=left,
                    wheel_travel_right=right,
                    rack_displacement=rack,
                    drive=drive,
                    case_id=result.case_id,
                    initial_state=assembly.state,
                )
                if retry.equilibrium.converged:
                    result = retry
            states.append(result)
            if result.equilibrium.converged:
                previous = result.equilibrium.state
    return tuple(states)


def run_c_grid(
    assembly: FrontAxleAssembly,
    grid: KGrid | None = None,
    *,
    paths: tuple[LoadPath, ...] | None = None,
    side_mode: LeftRightMode = "symmetric",
    drive: Literal["contact_point", "wheel_center"] = "wheel_center",
    k_solver: KModeSolver | None = None,
    c_solver: CModeSolver | None = None,
) -> tuple[CState, ...]:
    """Run standard C paths over a K grid with one cached reference per point."""
    plan = grid or KGrid()
    k_states = run_k_grid(assembly, plan, drive=drive, solver=k_solver)
    cache = KReferenceCache()
    for state in k_states:
        cache.store(state)
    engine = c_solver or CModeSolver()
    load_paths = paths or LoadPath.standard()
    results: list[CState] = []
    for point_index, reference in enumerate(k_states):
        for path in load_paths:
            for level_index, value in enumerate(path.values()):
                load = SixVector(**{path.axis: value})
                state = engine.solve(
                    assembly,
                    load,
                    side_mode=side_mode,
                    k_cache=cache,
                    wheel_left=reference.wheel_travel_left,
                    wheel_right=reference.wheel_travel_right,
                    rack=reference.rack_displacement,
                    case_id=f"c-{point_index:03d}-{path.name}-{level_index:02d}",
                )
                results.append(replace(state, path=path.name, level=float(value)))
    return tuple(results)
