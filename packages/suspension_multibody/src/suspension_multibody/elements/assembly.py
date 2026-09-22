"""
Assemble the generalized force vector from a set of evaluated elements.

This is a *reporting* mapping, not a solve: it walks the elements, collects
their wrenches and expresses the total in each body's own frame.  It used to sit
beside the Newton solver in `solver/`, which made "which elements carry what
load" look like a solver responsibility -- it is not, and the reporting layer
needs it long after the Python solver is gone.
"""

from __future__ import annotations

from typing import Callable, Iterable, cast

import numpy as np

from ..preparation.assembly.types import RigidBodyState
from ..preparation.geometry import wrench_global_to_local
from .base import ForceEvaluation

__all__ = ["evaluate_generalized_forces"]


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
