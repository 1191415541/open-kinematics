"""Force-element energy, active-set and tangent tests."""

import numpy as np
import pytest

from suspension_mbd.core import SE3, RigidBody, RigidBodyState
from suspension_mbd.elements import (
    AntiRollBarElement,
    BumpStopElement,
    BushingElement,
    ElementError,
    GravityElement,
    LinearSpringElement,
    StaticDamperElement,
    VerticalTireElement,
)


def _state(offset: float = 2.0) -> RigidBodyState:
    return RigidBodyState(
        {
            "a": RigidBody("a", SE3.identity()),
            "b": RigidBody(
                "b", SE3(np.array([offset, 0.0, 0.0]), np.array([1.0, 0, 0, 0]))
            ),
        }
    )


def test_linear_spring_force_and_energy() -> None:
    element = LinearSpringElement(
        "spring", "a", [0, 0, 0], "b", [0, 0, 0], 10, free_length=1
    )
    result = element.evaluate(_state())
    assert np.isclose(result.energy, 5)
    assert np.isclose(result.body_wrenches_global["b"][0], -10)
    assert result.tangent is not None and result.tangent.shape == (6, 6)


def test_spring_rejects_coincident_endpoints() -> None:
    element = LinearSpringElement(
        "spring", "a", [0, 0, 0], "b", [0, 0, 0], 10, free_length=1
    )
    with pytest.raises(ElementError):
        element.evaluate(_state(0.0))


def test_static_damper_and_bushing() -> None:
    damper = StaticDamperElement(
        "damper",
        "a",
        [0, 0, 0],
        "b",
        [0, 0, 0],
        gas_stiffness=20,
        gas_reference_length=1,
    )
    assert np.isclose(damper.evaluate(_state()).body_wrenches_global["b"][0], -20)
    stiffness = np.eye(6) * 10
    bushing = BushingElement("bush", "a", "b", stiffness=stiffness)
    result = bushing.evaluate(_state())
    assert np.isclose(result.energy, 20)
    assert np.isclose(result.body_wrenches_global["b"][0], -20)


def test_tire_unilateral_active_set() -> None:
    active = VerticalTireElement("tire", "b", [0, 0, 0], 100, 1, road_z=0)
    assert active.evaluate(_state(offset=0.5)).active
    unloaded_state = RigidBodyState(
        {
            "a": RigidBody("a", SE3.identity()),
            "b": RigidBody(
                "b", SE3(np.array([0.0, 0.0, 2.0]), np.array([1.0, 0, 0, 0]))
            ),
        }
    )
    unloaded = active.evaluate(unloaded_state)
    assert not unloaded.active
    assert unloaded.event == "tire_unloaded"


def test_antiroll_stop_and_gravity() -> None:
    state = RigidBodyState(
        {
            "a": RigidBody(
                "a", SE3(np.array([0.0, 0.0, 0.0]), np.array([1.0, 0, 0, 0]))
            ),
            "b": RigidBody(
                "b", SE3(np.array([0.0, 0.0, 0.1]), np.array([1.0, 0, 0, 0]))
            ),
        }
    )
    bar = AntiRollBarElement("arb", "a", [0, 0, 0], "b", [0, 0, 0], 100)
    assert np.isclose(bar.evaluate(state).energy, 0.5)
    stop = BumpStopElement("stop", "a", [0, 0, 0], "b", [0, 0, 0], 0.05, 100)
    assert not stop.evaluate(state).active
    gravity = GravityElement("gravity", "b", 2)
    assert np.isclose(gravity.evaluate(state).body_wrenches_global["b"][2], -19620)
