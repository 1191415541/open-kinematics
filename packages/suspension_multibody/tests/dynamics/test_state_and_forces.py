"""Dynamic state and force-interface tests."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.core import SE3, RigidBody, RigidBodyState
from suspension_multibody.dynamics import (
    DynamicContext,
    DynamicElementAdapter,
    DynamicRigidBodyState,
    LinearVelocityDamperElement,
    StaticElementInDynamicError,
    evaluate_dynamic_element,
)
from suspension_multibody.elements import (
    BumpStopElement,
    BushingElement,
    LinearSpringElement,
)


def test_dynamic_state_returns_point_velocity_from_local_twist() -> None:
    state = DynamicRigidBodyState(
        RigidBodyState({"body": RigidBody("body", mass=1.0)}),
        velocities={"body": np.array([1.0, 0.0, 0.0, 0.0, 0.0, 2.0])},
    )

    velocity = state.point_velocity_global("body", np.array([0.0, 1.0, 0.0]))

    assert velocity.tolist() == pytest.approx([-1.0, 0.0, 0.0])


def test_static_element_requires_explicit_downgrade() -> None:
    element = LinearSpringElement(
        "spring",
        "a",
        np.zeros(3),
        "b",
        np.array([1.0, 0.0, 0.0]),
        stiffness=1.0,
        free_length=1.0,
    )
    state = DynamicRigidBodyState(
        RigidBodyState({"a": RigidBody("a", mass=1.0), "b": RigidBody("b", mass=1.0)})
    )

    with pytest.raises(StaticElementInDynamicError):
        evaluate_dynamic_element(element, state, 0.0)

    result = evaluate_dynamic_element(
        element, state, 0.0, DynamicContext(allow_static_element_downgrade=True)
    )
    assert result.name == "spring"


def test_velocity_damper_reports_dissipated_power() -> None:
    element = LinearVelocityDamperElement(
        "damper",
        "a",
        np.zeros(3),
        "b",
        np.array([1.0, 0.0, 0.0]),
        damping=10.0,
    )
    state = DynamicRigidBodyState(
        RigidBodyState({"a": RigidBody("a", mass=1.0), "b": RigidBody("b", mass=1.0)}),
        velocities={"b": np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0])},
    )

    result = element.evaluate_dynamic(state, 0.0)

    assert result.power == pytest.approx(-40.0)
    assert result.body_wrenches_global["b"][0] == pytest.approx(-20.0)


@pytest.mark.parametrize(
    "element",
    (
        LinearSpringElement(
            "spring",
            "a",
            np.zeros(3),
            "b",
            np.zeros(3),
            stiffness=10.0,
            free_length=1.0,
        ),
        BushingElement("bushing", "a", "b", stiffness=np.eye(6) * 10.0),
        BumpStopElement(
            "stop",
            "a",
            np.zeros(3),
            "b",
            np.zeros(3),
            clearance=3.0,
            stiffness=100.0,
        ),
    ),
)
def test_direct_dynamic_element_matches_static_force_result(element: object) -> None:
    state = DynamicRigidBodyState(
        RigidBodyState(
            {
                "a": RigidBody("a", mass=1.0),
                    "b": RigidBody(
                        "b",
                        pose=SE3(
                            np.array([2.0, 0.0, 0.0]),
                            np.array([1.0, 0.0, 0.0, 0.0]),
                        ),
                    mass=1.0,
                ),
            }
        )
    )
    static = element.evaluate(state.pose_state)  # type: ignore[attr-defined]
    dynamic = DynamicElementAdapter(element).evaluate_dynamic(state, 0.0)
    assert dynamic.energy == pytest.approx(static.energy)
    assert dynamic.active is static.active
    assert dynamic.events == (() if static.event is None else (static.event,))
    for body, wrench in static.body_wrenches_global.items():
        np.testing.assert_allclose(dynamic.body_wrenches_global[body], wrench)
