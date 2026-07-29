"""Lagrange multiplier reaction recovery and equilibrium checks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .constraints import ConstraintSystem
from .rigid_body import RigidBodyState
from .spatial import wrench_local_to_global


@dataclass(frozen=True)
class ReactionResult:
    """Constraint generalized forces and body-origin global wrenches."""

    multipliers: np.ndarray
    body_local_wrenches: dict[str, np.ndarray]
    body_global_wrenches: dict[str, np.ndarray]


def recover_reactions(
    system: ConstraintSystem,
    state: RigidBodyState,
    multipliers: np.ndarray,
    *,
    body_order: tuple[str, ...] | None = None,
) -> ReactionResult:
    """Recover generalized reaction wrenches from KKT multipliers."""
    values = np.asarray(multipliers, dtype=float)
    residual_size = sum(len(constraint.residual(state)) for constraint in system.constraints)
    if values.shape != (residual_size,):
        raise ValueError(f"expected {residual_size} multipliers, got {values.size}")
    order = body_order or tuple(name for name, body in state.bodies.items() if not body.fixed)
    local_wrenches: dict[str, np.ndarray] = {}
    cursor = 0
    for constraint in system.constraints:
        size = len(constraint.residual(state))
        blocks = constraint.jacobian(state)
        for body in order:
            block = blocks.get(body)
            if block is None:
                continue
            local_wrenches[body] = local_wrenches.get(body, np.zeros(6)) + block.T @ values[cursor : cursor + size]
        cursor += size
    global_wrenches = {
        body: wrench_local_to_global(state.pose(body), wrench)
        for body, wrench in local_wrenches.items()
    }
    return ReactionResult(values, local_wrenches, global_wrenches)


def body_equilibrium_wrench(wrenches: dict[str, np.ndarray]) -> np.ndarray:
    """Sum global force and moment wrenches."""
    return np.sum(np.vstack(tuple(wrenches.values())), axis=0) if wrenches else np.zeros(6)
