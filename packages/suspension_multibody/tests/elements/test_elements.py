"""Force-element energy, active-set and tangent tests."""

import numpy as np
import pytest

from suspension_multibody.core import (
    SE3,
    RigidBody,
    RigidBodyState,
    quaternion_multiply,
    rotation_vector_to_quaternion,
)
from suspension_multibody.elements import (
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


def test_bushing_akima_curve_controls_force_tangent_and_energy() -> None:
    curve = ((-2.0, -4.0), (-1.0, -1.0), (0.0, 0.0), (1.0, 2.0), (2.0, 8.0))
    bushing = BushingElement(
        "akima_bushing",
        "a",
        "b",
        stiffness=np.zeros((6, 6)),
        force_curves=(curve, (), (), (), (), ()),
        force_curve_interpolation="akima",
    )

    result = bushing.evaluate(_state(0.5))

    assert result.body_wrenches_global["b"][0] == pytest.approx(-0.8166666667)
    assert result.tangent is not None
    assert result.tangent[0, 0] == pytest.approx(-1.9666666667)
    assert result.energy == pytest.approx(0.1909722222)


def test_bushing_uses_xyz_cardan_coordinates_and_rates() -> None:
    angles = np.array((0.07, -0.04, 0.11))
    relative_quaternion = quaternion_multiply(
        quaternion_multiply(
            rotation_vector_to_quaternion(np.array((angles[0], 0.0, 0.0))),
            rotation_vector_to_quaternion(np.array((0.0, angles[1], 0.0))),
        ),
        rotation_vector_to_quaternion(np.array((0.0, 0.0, angles[2]))),
    )
    angular_velocity = np.array((0.3, -0.2, 0.5))
    bushing = BushingElement(
        "cardan_bushing",
        "a",
        "b",
        stiffness=np.eye(6),
        rotation_coordinates="cardan_xyz",
    )

    expected_rate = np.array(
        (
            angular_velocity[0]
            - np.sin(angles[1])
            * (-np.sin(angles[0]) * angular_velocity[1]
               + np.cos(angles[0]) * angular_velocity[2])
            / np.cos(angles[1]),
            np.cos(angles[0]) * angular_velocity[1]
            + np.sin(angles[0]) * angular_velocity[2],
            (-np.sin(angles[0]) * angular_velocity[1]
             + np.cos(angles[0]) * angular_velocity[2])
            / np.cos(angles[1]),
        )
    )

    np.testing.assert_allclose(
        bushing.rotational_deformation(relative_quaternion), angles, atol=1e-12
    )
    np.testing.assert_allclose(
        bushing.rotational_rate(relative_quaternion, angular_velocity),
        expected_rate,
        atol=1e-12,
    )


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
