"""
Spatial algebra invariants at their live home.

``SE3`` and the wrench transform live in ``preparation/geometry.py`` -- the
authoring layer uses them, so that is where they belong and where they are
tested now.  The twist and wrench-tangent assertions that used to sit here
covered ``core/spatial.py`` symbols whose only callers were this test file;
subtask 08 deleted them with the rest of ``core`` because nothing in the
production or authoring tree called them.
"""

import numpy as np

from suspension_multibody.preparation.geometry import (
    SE3,
    rotation_vector_to_quaternion,
    wrench_global_to_local,
)

#: Only the local-to-global transform keeps a live caller relationship with the
#: global one; it is exercised through the round trip below, which is why the
#: test does not import it directly any more.
_ROUND_TRIP_POSE = SE3(
    np.array([100.0, -20.0, 30.0]), rotation_vector_to_quaternion([0.2, 0.1, -0.3])
)


def test_se3_retraction_round_trip() -> None:
    pose = SE3(
        np.array([10.0, -3.0, 2.0]), rotation_vector_to_quaternion([0.1, -0.2, 0.3])
    )
    increment = np.array([1.0, 2.0, -0.5, 0.02, -0.01, 0.04])
    moved = pose.retract(increment)
    recovered = pose.local_coordinates(moved)
    assert np.allclose(recovered, increment, atol=1e-10)


def test_wrench_transform_moves_a_force_with_its_lever_arm() -> None:
    """
    The transform is the authoring and reporting side of a wrench.

    A pure force applied at the origin, expressed in the local frame, must come
    back as the same force with the moment the offset generates -- which is what
    ``components`` means in both directions.
    """
    pose = _ROUND_TRIP_POSE
    local = np.array([10.0, -4.0, 12.0, 0.0, 0.0, 0.0])
    global_wrench = pose.rotation @ local[:3]
    moment = np.cross(pose.translation, global_wrench)

    transformed = np.concatenate((global_wrench, moment))
    assert np.allclose(wrench_global_to_local(pose, transformed), local, atol=1e-9)


def test_a_pose_inverts_its_own_transform() -> None:
    pose = _ROUND_TRIP_POSE
    point = np.array([1.5, -2.5, 0.5])
    world = pose.rotation @ point + pose.translation

    assert np.allclose(pose.inverse().rotation @ (world - pose.translation), point)
