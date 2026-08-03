"""Dynamic force-element interfaces."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..elements import ForceEvaluation
from ..elements.elastic import _point_wrench
from .state import DynamicRigidBodyState


@dataclass(frozen=True)
class DynamicContext:
    """Context passed to dynamic force elements."""

    allow_static_element_downgrade: bool = False


@dataclass(frozen=True)
class DynamicForceEvaluation:
    """Dynamic force-element output at one time sample."""

    name: str
    energy: float
    power: float = 0.0
    body_wrenches_global: dict[str, np.ndarray] = field(default_factory=dict)
    active: bool = True
    events: tuple[str, ...] = ()
    internal_state: object | None = None


class StaticElementInDynamicError(TypeError):
    """Raised when a static-only force element is used in strict dynamic mode."""


def evaluate_dynamic_element(
    element: object,
    state: DynamicRigidBodyState,
    time: float,
    context: DynamicContext | None = None,
) -> DynamicForceEvaluation:
    """Evaluate a dynamic element or explicitly downgrade a static element."""
    dynamic_evaluator = getattr(element, "evaluate_dynamic", None)
    if callable(dynamic_evaluator):
        result = dynamic_evaluator(state, time)
        if not isinstance(result, DynamicForceEvaluation):
            raise TypeError(f"dynamic element {element!r} returned invalid result")
        return result
    settings = context or DynamicContext()
    if not settings.allow_static_element_downgrade:
        raise StaticElementInDynamicError(
            f"element {element!r} has no dynamic force interface"
        )
    evaluator = getattr(element, "evaluate", None)
    if not callable(evaluator):
        raise TypeError(f"element {element!r} has no force interface")
    static = evaluator(state.pose_state)
    if not isinstance(static, ForceEvaluation):
        raise TypeError(f"static element {element!r} returned invalid result")
    return DynamicForceEvaluation(
        name=static.name,
        energy=static.energy,
        body_wrenches_global=static.body_wrenches_global,
        active=static.active,
        events=() if static.event is None else (static.event,),
    )


@dataclass(frozen=True)
class LinearVelocityDamperElement:
    """Two-point viscous damper using point relative velocity."""

    name: str
    body_a: str
    point_a: np.ndarray
    body_b: str
    point_b: np.ndarray
    damping: float

    def __post_init__(self) -> None:
        if self.damping < 0 or not np.isfinite(self.damping):
            raise ValueError("damper damping must be finite and non-negative")
        for attr in ("point_a", "point_b"):
            point = np.asarray(getattr(self, attr), dtype=float)
            if point.shape != (3,) or not np.all(np.isfinite(point)):
                raise ValueError("damper points must contain three finite values")
            object.__setattr__(self, attr, point.copy())

    def evaluate_dynamic(
        self, state: DynamicRigidBodyState, time: float
    ) -> DynamicForceEvaluation:
        del time
        point_a = state.pose_state.point_world(self.body_a, self.point_a)
        point_b = state.pose_state.point_world(self.body_b, self.point_b)
        delta = point_b - point_a
        length = float(np.linalg.norm(delta))
        if length < 1e-12:
            raise ValueError("damper endpoints are coincident")
        axis = delta / length
        velocity_a = state.point_velocity_global(self.body_a, self.point_a)
        velocity_b = state.point_velocity_global(self.body_b, self.point_b)
        relative_speed = float(axis @ (velocity_b - velocity_a))
        scalar = self.damping * relative_speed
        force_b = -scalar * axis
        force_a = -force_b
        power = -self.damping * relative_speed**2
        return DynamicForceEvaluation(
            name=self.name,
            energy=0.0,
            power=power,
            body_wrenches_global={
                self.body_a: _point_wrench(point_a, force_a),
                self.body_b: _point_wrench(point_b, force_b),
            },
        )


def sum_dynamic_wrenches(
    evaluations: tuple[DynamicForceEvaluation, ...]
) -> dict[str, np.ndarray]:
    """Sum dynamic force evaluations by body."""
    totals: dict[str, np.ndarray] = {}
    for evaluation in evaluations:
        for body, wrench in evaluation.body_wrenches_global.items():
            totals[body] = totals.get(body, np.zeros(6)) + np.asarray(wrench, dtype=float)
    return totals
