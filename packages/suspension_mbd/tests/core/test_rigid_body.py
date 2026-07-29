"""Rigid-body state and point Jacobian tests."""

import numpy as np

from suspension_mbd.core import (
    SE3,
    RigidBody,
    RigidBodyState,
    rotation_vector_to_quaternion,
)


def test_point_jacobian_matches_retraction() -> None:
    body = RigidBody(
        "upright",
        SE3(np.array([1.0, 2.0, 3.0]), rotation_vector_to_quaternion([0.2, -0.1, 0.3])),
    )
    state = RigidBodyState({body.name: body})
    point = np.array([0.4, -0.2, 0.7])
    increment = np.array([1e-6, -2e-6, 3e-6, 4e-7, -5e-7, 6e-7])
    numerical = (
        state.retract({"upright": increment}).point_world("upright", point)
        - state.point_world("upright", point)
    ) / 1e-6
    assert np.allclose(
        numerical,
        state.point_jacobian("upright", point) @ (increment / 1e-6),
        atol=1e-6,
    )


def test_fixed_body_ignores_increment() -> None:
    body = RigidBody("chassis", fixed=True)
    state = RigidBodyState({"chassis": body})
    moved = state.retract({"chassis": np.ones(6)})
    assert np.array_equal(moved.pose("chassis").translation, body.pose.translation)
