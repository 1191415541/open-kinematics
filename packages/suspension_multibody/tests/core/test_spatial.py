"""Spatial algebra invariants and tangent tests."""

import numpy as np

from suspension_multibody.core import (
    SE3,
    rotation_vector_to_quaternion,
    twist_local_to_global,
    wrench_global_to_local,
    wrench_local_to_global,
    wrench_translation_tangent,
)


def test_se3_retraction_round_trip() -> None:
    pose = SE3(
        np.array([10.0, -3.0, 2.0]), rotation_vector_to_quaternion([0.1, -0.2, 0.3])
    )
    increment = np.array([1.0, 2.0, -0.5, 0.02, -0.01, 0.04])
    moved = pose.retract(increment)
    recovered = pose.local_coordinates(moved)
    assert np.allclose(recovered, increment, atol=1e-10)


def test_wrench_transform_is_invertible() -> None:
    pose = SE3(
        np.array([100.0, -20.0, 30.0]), rotation_vector_to_quaternion([0.2, 0.1, -0.3])
    )
    local = np.array([10.0, -4.0, 12.0, 100.0, 20.0, -30.0])
    assert np.allclose(
        wrench_global_to_local(pose, wrench_local_to_global(pose, local)), local
    )


def test_wrench_and_twist_preserve_power() -> None:
    pose = SE3(
        np.array([3.0, 4.0, 5.0]), rotation_vector_to_quaternion([0.1, 0.2, 0.3])
    )
    wrench = np.array([10.0, -4.0, 5.0, 2.0, 6.0, -1.0])
    twist = np.array([0.3, -0.2, 0.1, 0.02, 0.04, -0.03])
    assert np.isclose(
        wrench @ twist,
        wrench_local_to_global(pose, wrench) @ twist_local_to_global(pose, twist),
    )


def test_wrench_translation_tangent_matches_finite_difference() -> None:
    force = np.array([10.0, -3.0, 7.0])
    base = np.array([2.0, 4.0, -1.0])
    tangent = wrench_translation_tangent(force)
    eps = 1e-6
    numerical = np.column_stack(
        [
            (
                np.cross(base + eps * np.eye(3)[i], force)
                - np.cross(base - eps * np.eye(3)[i], force)
            )
            / (2 * eps)
            for i in range(3)
        ]
    )
    assert np.allclose(tangent[3:], numerical)
