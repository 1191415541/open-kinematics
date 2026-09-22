"""
Rigid-body state tests: the data a body carries and its retraction.

``RigidBody`` and ``RigidBodyState`` live in ``preparation/assembly/types.py``.
The point-Jacobian assertion that used to sit here covered
``core/rigid_body.point_jacobian``, which subtask 08 deleted with the rest of
``core`` -- a point Jacobian is part of solving, and the native kernel owns it.
The retraction assertions stay, because retraction is state, not solving.
"""

import numpy as np

from suspension_multibody.preparation.assembly.types import (
    RigidBody,
    RigidBodyState,
)
from suspension_multibody.preparation.geometry import (
    SE3,
    rotation_vector_to_quaternion,
)


def test_a_body_carries_its_pose_and_mass_properties() -> None:
    body = RigidBody(
        "upright",
        SE3(np.array([1.0, 2.0, 3.0]), rotation_vector_to_quaternion([0.2, -0.1, 0.3])),
    )

    assert body.name == "upright"
    assert body.fixed is False
    assert np.allclose(body.pose.translation, [1.0, 2.0, 3.0])


def test_a_pure_translation_increment_moves_the_point_by_that_offset() -> None:
    """Retraction takes a *local* increment, so the world offset is rotated."""
    body = RigidBody(
        "upright",
        SE3(np.array([1.0, 2.0, 3.0]), rotation_vector_to_quaternion([0.2, -0.1, 0.3])),
    )
    state = RigidBodyState({body.name: body})
    point = np.array([0.4, -0.2, 0.7])
    increment = np.array([1e-6, -2e-6, 3e-6, 0.0, 0.0, 0.0])

    moved = state.retract({"upright": increment}).point_world("upright", point)
    unmoved = state.point_world("upright", point)
    rotation = state.pose("upright").rotation

    assert np.allclose(moved - unmoved, rotation @ increment[:3], atol=1e-12)


def test_a_pure_rotation_increment_swings_the_point_about_the_body_origin() -> None:
    """A rotational increment leaves the body's own origin where it was."""
    body = RigidBody(
        "upright",
        SE3(np.array([1.0, 2.0, 3.0]), rotation_vector_to_quaternion([0.2, -0.1, 0.3])),
    )
    state = RigidBodyState({body.name: body})
    point = np.array([0.4, -0.2, 0.7])
    increment = np.array([0.0, 0.0, 0.0, 1e-8, 0.0, 0.0])

    retracted = state.retract({"upright": increment})
    origin_before = state.pose("upright").translation
    origin_after = retracted.pose("upright").translation

    # A rotational increment moves the body about its own origin, so the origin
    # itself is untouched and the point swings.  Both tolerances are pinned: the
    # default ``rtol`` is scaled to the point's own magnitude, which is larger
    # than the swing here and would call the two equal.
    assert np.array_equal(origin_after, origin_before)
    assert not np.allclose(
        retracted.point_world("upright", point),
        state.point_world("upright", point),
        rtol=0.0,
        atol=1e-12,
    )


def test_fixed_body_ignores_increment() -> None:
    body = RigidBody("chassis", fixed=True)
    state = RigidBodyState({"chassis": body})
    moved = state.retract({"chassis": np.ones(6)})
    assert np.array_equal(moved.pose("chassis").translation, body.pose.translation)
