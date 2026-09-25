"""
Geometric adaptation: the attachment is derived, never cached.

``A4``'s judgement lives here.  A hardpoint moves; the port's *local* offset does
not; so the mounted instance's world pose must equal what an independently
computed transform predicts.  The test computes that expected pose with plain
rotation/translation arithmetic rather than by calling the code under test, so a
bug in the composition cannot satisfy its own check.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from suspension_multibody.connections import (
    GeometryMismatchError,
    port_world_pose,
    solve_mount,
)
from suspension_multibody.modeling import EntityId, GeometryPort
from suspension_multibody.modeling.primitives.spatial import (
    SE3,
    quaternion_to_matrix,
    rotation_vector_to_quaternion,
)


def _port(*, cardinality: str = "one") -> GeometryPort:
    return GeometryPort(
        id=EntityId(("rig",), "wheel_centre"),
        owner=EntityId(("rig",), "carrier"),
        role="wheel_centre",
        cardinality=cardinality,  # type: ignore[arg-type]
    )


def _translate(x: float, y: float, z: float) -> SE3:
    return SE3(np.array([x, y, z], dtype=float), np.array([1.0, 0.0, 0.0, 0.0]))


def test_a_pure_translation_composes_by_addition() -> None:
    port_world = _translate(1.0, 2.0, 3.0)
    installation = _translate(0.5, 0.0, -1.0)
    solution = solve_mount(_port(), port_world=port_world, installation=installation)
    assert np.allclose(solution.pose.translation, [1.5, 2.0, 2.0])


def test_moving_the_owner_moves_the_mount_by_the_same_amount() -> None:
    """
    The adaptive-interface property, stated as a difference.

    The rig definition is unchanged between the two calls; only the port's world
    pose moved.  If the mount did not move with it, the rig would be attached to
    a stale coordinate.
    """
    installation = _translate(0.25, -0.5, 0.0)
    before = solve_mount(
        _port(), port_world=_translate(0.0, 0.0, 0.0), installation=installation
    )
    after = solve_mount(
        _port(), port_world=_translate(0.1, 0.2, -0.3), installation=installation
    )
    delta = np.asarray(after.pose.translation) - np.asarray(before.pose.translation)
    assert np.allclose(delta, [0.1, 0.2, -0.3])


def test_port_world_pose_is_the_owner_pose_composed_with_the_local_offset() -> None:
    """
    Checked against hand arithmetic, not against ``SE3.compose``.

    A pure translation then a rotated offset: the expected point is
    ``owner_t + R_owner @ local_t``, computed here with plain matrix algebra.
    """
    owner = SE3(
        np.array([1.0, -2.0, 0.5]),
        rotation_vector_to_quaternion(np.array([0.0, 0.0, math.pi / 2])),
    )
    local = _translate(1.0, 0.0, 0.0)
    world = port_world_pose(owner, local)
    rotation = quaternion_to_matrix(owner.quaternion)
    expected = np.asarray(owner.translation) + rotation @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(world.translation, expected)


def test_a_rotated_installation_is_composed_not_added() -> None:
    """Adding a rotated offset would be right only when the port is unrotated."""
    port_world = SE3(
        np.array([0.0, 0.0, 0.0]),
        rotation_vector_to_quaternion(np.array([0.0, 0.0, math.pi / 2])),
    )
    installation = _translate(1.0, 0.0, 0.0)
    solution = solve_mount(_port(), port_world=port_world, installation=installation)
    # A 90-degree rotation about z maps (1,0,0) to (0,1,0).
    assert np.allclose(solution.pose.translation, [0.0, 1.0, 0.0], atol=1e-12)


def test_matches_accepts_an_independently_computed_pose() -> None:
    port_world = _translate(1.0, 1.0, 1.0)
    installation = _translate(0.0, 0.0, -1.0)
    solution = solve_mount(_port(), port_world=port_world, installation=installation)
    assert solution.matches(_translate(1.0, 1.0, 0.0), tolerance=1e-12)


def test_matches_rejects_a_pose_outside_tolerance() -> None:
    solution = solve_mount(
        _port(), port_world=_translate(0.0, 0.0, 0.0), installation=_translate(0.0, 0.0, 0.0)
    )
    assert not solution.matches(_translate(0.001, 0.0, 0.0), tolerance=1e-6)


def test_matches_compares_rotation_not_quaternion_sign() -> None:
    """
    ``q`` and ``-q`` are the same rotation; a sign difference is not a mismatch.

    Comparing quaternions directly would fail here, which is why the check goes
    through the principal rotation vector.
    """
    pose = SE3(
        np.zeros(3),
        rotation_vector_to_quaternion(np.array([0.3, 0.0, 0.0])),
    )
    flipped = SE3(np.zeros(3), -pose.quaternion)
    solution = solve_mount(_port(), port_world=pose, installation=SE3.identity())
    assert solution.matches(flipped, tolerance=1e-12)


def test_an_unconnectable_port_is_refused() -> None:
    """A ``none`` port exists to be reported, not mounted on."""
    with pytest.raises(GeometryMismatchError, match="unconnectable"):
        solve_mount(
            _port(cardinality="none"),
            port_world=SE3.identity(),
            installation=SE3.identity(),
        )


def test_the_solution_keeps_its_evidence() -> None:
    """A4 rechecks the arithmetic, so the two inputs have to survive."""
    port_world = _translate(1.0, 0.0, 0.0)
    installation = _translate(0.0, 1.0, 0.0)
    solution = solve_mount(_port(), port_world=port_world, installation=installation)
    assert np.allclose(solution.port_world.translation, port_world.translation)
    assert np.allclose(solution.installation.translation, installation.translation)
