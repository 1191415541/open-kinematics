"""Dynamic state and force-interface tests."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.core import RigidBody, RigidBodyState
from suspension_multibody.dynamics import (
    DynamicContext,
    DynamicRigidBodyState,
    LinearVelocityDamperElement,
    StaticElementInDynamicError,
    evaluate_dynamic_element,
)
from suspension_multibody.elements import LinearSpringElement


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
