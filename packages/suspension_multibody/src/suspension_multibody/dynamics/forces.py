"""Dynamic force-element interfaces."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..core.spatial import (
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_rotation_vector,
)
from ..elements import ElementError, ForceEvaluation
from ..elements.elastic import (
    BumpStopElement,
    BushingElement,
    LinearSpringElement,
    _curve_value,
    _point_wrench,
)
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


@dataclass(frozen=True)
class DynamicElementAdapter:
    """Promote a position-based element and optional viscous damper to dynamics."""

    element: object

    def evaluate_dynamic(
        self, state: DynamicRigidBodyState, time: float
    ) -> DynamicForceEvaluation:
        del time
        if isinstance(self.element, BushingElement):
            return _evaluate_bushing_dynamic(self.element, state)
        if isinstance(self.element, LinearSpringElement):
            return _evaluate_spring_dynamic(self.element, state)
        if isinstance(self.element, BumpStopElement):
            return _evaluate_bump_stop_dynamic(self.element, state)
        evaluator = getattr(self.element, "evaluate", None)
        if not callable(evaluator):
            raise TypeError(f"element {self.element!r} has no static evaluator")
        static = evaluator(state.pose_state)
        power = 0.0
        damping = float(getattr(self.element, "viscous_damping", 0.0))
        force_curve = tuple(getattr(self.element, "force_curve", ()))
        has_velocity_force = (
            damping > 0.0
            or (force_curve and hasattr(self.element, "viscous_damping"))
        ) and all(
            hasattr(self.element, name)
            for name in ("body_a", "body_b", "point_a", "point_b")
        )
        if not has_velocity_force:
            return DynamicForceEvaluation(
                name=getattr(static, "name", type(self.element).__name__),
                energy=float(static.energy),
                power=0.0,
                body_wrenches_global=static.body_wrenches_global,
                active=bool(static.active),
                events=() if static.event is None else (static.event,),
            )
        wrenches = {
            body: np.asarray(wrench, dtype=float).copy()
            for body, wrench in static.body_wrenches_global.items()
        }
        if has_velocity_force:
            body_a = self.element.body_a
            body_b = self.element.body_b
            point_a = np.asarray(self.element.point_a, dtype=float)
            point_b = np.asarray(self.element.point_b, dtype=float)
            world_a = state.pose_state.point_world(body_a, point_a)
            world_b = state.pose_state.point_world(body_b, point_b)
            delta = world_b - world_a
            length = float(np.linalg.norm(delta))
            if length > 1e-12:
                axis = delta / length
                relative_speed = float(
                    axis
                    @ (
                        state.point_velocity_global(body_b, point_b)
                        - state.point_velocity_global(body_a, point_a)
                    )
                )
                scalar = (
                    _curve_value(force_curve, relative_speed)
                    if force_curve
                    else damping * relative_speed
                )
                force_b = -scalar * axis
                force_a = -force_b
                wrench_a = _point_wrench(world_a, force_a)
                wrench_b = _point_wrench(world_b, force_b)
                existing = wrenches.get(body_a)
                if existing is None:
                    wrenches[body_a] = wrench_a
                else:
                    existing += wrench_a
                existing = wrenches.get(body_b)
                if existing is None:
                    wrenches[body_b] = wrench_b
                else:
                    existing += wrench_b
                power = -scalar * relative_speed
        return DynamicForceEvaluation(
            name=getattr(static, "name", type(self.element).__name__),
            energy=float(static.energy),
            power=power,
            body_wrenches_global=wrenches,
            active=bool(static.active),
            events=() if static.event is None else (static.event,),
        )


def _evaluate_bushing_dynamic(
    element: BushingElement, state: DynamicRigidBodyState
) -> DynamicForceEvaluation:
    """Evaluate a bushing without constructing an intermediate static result."""
    body_pose_a = state.pose_state.pose(element.body_a)
    body_pose_b = state.pose_state.pose(element.body_b)
    local_pose_a = element.local_pose_a
    local_pose_b = element.local_pose_b
    rotation_a = body_pose_a.rotation @ local_pose_a.rotation
    translation_a = body_pose_a.translation + body_pose_a.rotation @ local_pose_a.translation
    translation_b = body_pose_b.translation + body_pose_b.rotation @ local_pose_b.translation
    quaternion_a = quaternion_multiply(body_pose_a.quaternion, local_pose_a.quaternion)
    quaternion_b = quaternion_multiply(body_pose_b.quaternion, local_pose_b.quaternion)
    relative_translation = rotation_a.T @ (translation_b - translation_a)
    relative_quaternion = quaternion_multiply(
        quaternion_conjugate(quaternion_a), quaternion_b
    )
    deformation = np.concatenate(
        (relative_translation, quaternion_to_rotation_vector(relative_quaternion))
    )
    velocity_a_global = state.point_velocity_global(
        element.body_a, local_pose_a.translation
    )
    velocity_b_global = state.point_velocity_global(
        element.body_b, local_pose_b.translation
    )
    relative_velocity = np.concatenate(
        (
            rotation_a.T @ (velocity_b_global - velocity_a_global),
            rotation_a.T
            @ (
                body_pose_b.rotation @ state.velocities[element.body_b][3:]
                - body_pose_a.rotation @ state.velocities[element.body_a][3:]
            ),
        )
    )
    generalized = (
        -element.stiffness @ deformation
        - element.damping @ relative_velocity
        + element.preload
    )
    force_global = rotation_a @ generalized[:3]
    moment_global = rotation_a @ generalized[3:]
    wrenches = {
        element.body_a: _point_wrench(translation_a, -force_global)
        + np.concatenate((np.zeros(3), -moment_global)),
        element.body_b: _point_wrench(translation_b, force_global)
        + np.concatenate((np.zeros(3), moment_global)),
    }
    return DynamicForceEvaluation(
        name=element.name,
        energy=0.5 * float(deformation @ element.stiffness @ deformation)
        - float(element.preload @ deformation),
        body_wrenches_global=wrenches,
    )


def _evaluate_spring_dynamic(
    element: LinearSpringElement, state: DynamicRigidBodyState
) -> DynamicForceEvaluation:
    """Evaluate a spring without assembling its unused static tangent."""
    point_a = state.pose_state.point_world(element.body_a, element.point_a)
    point_b = state.pose_state.point_world(element.body_b, element.point_b)
    delta = point_b - point_a
    length = float(np.linalg.norm(delta))
    if length < 1e-12:
        raise ElementError("spring endpoints are coincident")
    unit = delta / length
    reference = (
        element.free_length
        if element.free_length is not None
        else element.reference_length
    )
    extension = length - reference  # type: ignore[operator]
    scalar = (
        _curve_value(element.force_curve, extension) + element.preload
        if element.force_curve
        else element.stiffness * extension + element.preload
    )
    force_b = -scalar * unit
    return DynamicForceEvaluation(
        name=element.name,
        energy=0.5 * element.stiffness * extension**2 + element.preload * extension,
        body_wrenches_global={
            element.body_a: _point_wrench(point_a, -force_b),
            element.body_b: _point_wrench(point_b, force_b),
        },
    )


def _evaluate_bump_stop_dynamic(
    element: BumpStopElement, state: DynamicRigidBodyState
) -> DynamicForceEvaluation:
    """Evaluate a bump stop without assembling its unused static tangent."""
    point_a = state.pose_state.point_world(element.body_a, element.point_a)
    point_b = state.pose_state.point_world(element.body_b, element.point_b)
    distance_vector = point_b - point_a
    distance = float(np.linalg.norm(distance_vector))
    compression = max(0.0, element.clearance - distance)
    if compression <= 0.0 or element.stiffness <= 0.0:
        return DynamicForceEvaluation(
            name=element.name,
            energy=0.0,
            active=False,
            events=("stop_clear",),
        )
    if distance < 1e-12:
        raise ElementError("bump stop endpoints are coincident")
    unit = distance_vector / distance
    scalar = (
        _curve_value(element.force_curve, compression)
        if element.force_curve
        else element.stiffness * compression
    )
    force_b = scalar * unit
    return DynamicForceEvaluation(
        name=element.name,
        energy=0.5 * element.stiffness * compression**2,
        body_wrenches_global={
            element.body_a: _point_wrench(point_a, -force_b),
            element.body_b: _point_wrench(point_b, force_b),
        },
        active=True,
        events=("stop_contact",),
    )


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
            current = totals.get(body)
            if current is None:
                totals[body] = np.asarray(wrench, dtype=float).copy()
            else:
                current += wrench
    return totals
