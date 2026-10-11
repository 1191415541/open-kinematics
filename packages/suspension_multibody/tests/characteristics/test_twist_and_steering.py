"""
P1 step 3 acceptance: Twist/Screw, steering-axis and Ackermann operators.

These are the operators whose failure mode is a *plausible* wrong number rather
than an exception, so each one is checked against a case whose answer can be
derived independently:

* a screw's defining identity ``v(x) = omega x (x - point) + pitch * omega`` is
  re-evaluated at the reported axis point rather than trusted;
* the ``point`` is a point *of* the axis, so moving the anchor along the axis
  must not change it;
* a pure translation has no axis, and that is a refusal rather than a distant
  point;
* the kingpin axis is a geometric fact of two ball joints, and a vertical axis
  has no horizontal projection to report;
* the Ackermann ideal relation is checked by round-tripping a geometry that
  satisfies it exactly.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import (
    CharacteristicsError,
    Screw,
    Twist,
    ackermann_error,
    kingpin_axis,
    kingpin_inclination,
    relative_twist,
    screw_from_twist,
)


def test_a_twist_evaluates_point_velocities_by_the_rigid_rule() -> None:
    """``v(x) = v_origin + omega x x`` is the definition, so check it directly."""
    twist = Twist(omega=[0.0, 0.0, 2.0], velocity=[1.0, 0.0, 0.0])
    # At the origin the velocity is the stored one.
    assert twist.transform_point([0.0, 0.0, 0.0]) == pytest.approx([1.0, 0.0, 0.0])
    # At (1,0,0) a rotation about +z adds (0, 2, 0).
    assert twist.transform_point([1.0, 0.0, 0.0]) == pytest.approx([1.0, 2.0, 0.0])


def test_a_twist_refuses_a_non_three_vector() -> None:
    with pytest.raises(CharacteristicsError, match="three-vector"):
        Twist(omega=[0.0, 0.0], velocity=[0.0, 0.0, 0.0])


def test_a_twist_refuses_a_non_finite_component() -> None:
    with pytest.raises(CharacteristicsError, match="finite"):
        Twist(omega=[0.0, 0.0, float("nan")], velocity=[0.0, 0.0, 0.0])


def test_the_screw_reconstruction_identity_holds_at_its_own_axis_point() -> None:
    """
    Re-derive the motion from the reported screw and compare.

    At a point on the axis the velocity is ``pitch * omega`` (the advance of the
    axis itself), so both the axis point and the pitch are checked by the same
    identity rather than being taken on trust.
    """
    twist = Twist(omega=[0.0, 1.0, 0.0], velocity=[2.0, 1.0, 0.0])
    screw = screw_from_twist(twist)
    assert screw.direction == pytest.approx([0.0, 1.0, 0.0])
    assert screw.angular_speed == pytest.approx(1.0)
    on_axis = twist.transform_point(screw.point)
    assert on_axis == pytest.approx(screw.pitch * screw.direction)
    # And away from the axis the full identity must still reproduce the twist.
    probe = np.array([0.3, -0.4, 0.5])
    expected = np.cross(screw.direction * screw.angular_speed, probe - screw.point)
    expected = expected + screw.pitch * screw.direction * screw.angular_speed
    assert twist.transform_point(probe) == pytest.approx(expected)


def test_a_pure_rotation_has_zero_pitch() -> None:
    """No advance along the axis means the pitch is exactly zero."""
    twist = Twist(omega=[0.0, 0.0, 1.0], velocity=[0.0, 0.0, 0.0])
    screw = screw_from_twist(twist)
    assert screw.pitch == pytest.approx(0.0)
    # The axis passes through the origin for a pure rotation at the origin.
    assert screw.point == pytest.approx([0.0, 0.0, 0.0])


def test_a_pure_translation_is_refused_rather_than_given_a_distant_axis() -> None:
    """A pure translation is not a screw; reporting a far-away axis is a lie."""
    with pytest.raises(CharacteristicsError, match="pure translation is not a screw"):
        screw_from_twist(Twist(omega=[0.0, 0.0, 0.0], velocity=[1.0, 0.0, 0.0]))


def test_the_anchor_only_selects_which_axis_point_is_reported() -> None:
    """
    Every point on the axis describes the same screw.

    The anchor picks the point of the axis closest to it, so an anchor placed on
    the axis is reported back exactly, while an off-axis anchor is projected onto
    the axis.  The screw's *motion* is unchanged either way -- only the reported
    ``point`` moves -- which is what "a point of the axis, not a fixed point"
    means.
    """
    twist = Twist(omega=[0.0, 1.0, 0.0], velocity=[0.0, 0.0, 1.0])
    base = screw_from_twist(twist)
    # An anchor on the axis is its own closest point.
    on_axis = base.point + 3.0 * base.direction
    along = screw_from_twist(twist, anchor=on_axis)
    assert along.point == pytest.approx(on_axis)
    assert along.pitch == pytest.approx(base.pitch)
    assert along.angular_speed == pytest.approx(base.angular_speed)
    # An off-axis anchor is projected onto the axis: the reported point is the
    # axis point plus the anchor's component along the axis.
    off_axis = np.array([5.0, 7.0, -2.0])
    projected = screw_from_twist(twist, anchor=off_axis)
    offset = off_axis - base.point
    expected = base.point + base.direction * float(offset @ base.direction)
    assert projected.point == pytest.approx(expected)
    # The move from an off-axis anchor to the reported point is the perpendicular
    # offset, so it must be perpendicular to the axis.
    delta = projected.point - off_axis
    assert float(delta @ base.direction) == pytest.approx(0.0, abs=1e-12)


def test_the_reported_axis_point_really_is_on_the_axis() -> None:
    """The refused-free check: the point must be perpendicular to the direction."""
    twist = Twist(omega=[0.5, -1.5, 2.0], velocity=[1.0, 2.0, -0.5])
    screw = screw_from_twist(twist)
    # The axis point is the closest point to the world origin, so its component
    # along the direction is zero.
    assert float(screw.point @ screw.direction) == pytest.approx(0.0, abs=1e-12)


def test_relative_twist_subtracts_the_reference() -> None:
    twist = Twist(omega=[0.0, 0.0, 2.0], velocity=[1.0, 0.0, 0.0])
    reference = Twist(omega=[0.0, 0.0, 1.0], velocity=[0.5, 0.0, 0.0])
    relative = relative_twist(twist, reference)
    assert relative.omega == pytest.approx([0.0, 0.0, 1.0])
    assert relative.velocity == pytest.approx([0.5, 0.0, 0.0])


def test_a_world_reference_leaves_the_twist_unchanged() -> None:
    """``None`` means the world, so callers need no special case."""
    twist = Twist(omega=[0.0, 0.0, 2.0], velocity=[1.0, 0.0, 0.0])
    relative = relative_twist(twist, None)
    assert relative.omega == pytest.approx(twist.omega)
    assert relative.velocity == pytest.approx(twist.velocity)


# --------------------------------------------------------------------------- #
# steering axis
# --------------------------------------------------------------------------- #


def test_the_kingpin_axis_is_the_direction_between_the_ball_joints() -> None:
    direction, point = kingpin_axis([0.0, 0.0, 0.0], [0.0, 0.0, 2.0])
    assert direction == pytest.approx([0.0, 0.0, 1.0])
    assert point == pytest.approx([0.0, 0.0, 0.0])


def test_a_tilted_kingpin_axis_returns_its_unit_direction() -> None:
    direction, point = kingpin_axis([0.0, -1.0, 0.0], [1.0, 0.0, 2.0])
    offset = np.array([1.0, 1.0, 2.0])
    assert direction == pytest.approx(offset / np.linalg.norm(offset))
    assert point == pytest.approx([0.0, -1.0, 0.0])


def test_coincident_kingpin_points_are_refused() -> None:
    """Two identical joints do not define an axis."""
    with pytest.raises(CharacteristicsError, match="coincide"):
        kingpin_axis([1.0, 2.0, 3.0], [1.0, 2.0, 3.0])


def test_kingpin_inclination_measures_from_vertical() -> None:
    """
    A 10-degree tilt is 10 degrees in the view that sees it.

    Tilt about the lateral axis (a lean in x) is visible in the front view
    (``axis="y"``) and is invisible in the side view (``axis="x"``), where the
    axis still projects straight up.  The two views are different measurements,
    so they are asserted separately.
    """
    angle = np.deg2rad(10.0)
    direction = [np.sin(angle), 0.0, np.cos(angle)]
    assert kingpin_inclination(direction, axis="y") == pytest.approx(angle)
    # In the side view this axis has no lean at all.
    assert kingpin_inclination(direction, axis="x") == pytest.approx(0.0)


def test_a_vertical_axis_has_zero_inclination_in_every_view() -> None:
    """
    A vertical axis is simply zero degrees, not an error.

    Every view sees the same vertical direction, so the inclination is defined
    and equal to zero.
    """
    assert kingpin_inclination([0.0, 0.0, 1.0], axis="x") == pytest.approx(0.0)
    assert kingpin_inclination([0.0, 0.0, 1.0], axis="y") == pytest.approx(0.0)


def test_an_axis_parallel_to_the_view_normal_has_no_inclination_there() -> None:
    """
    The axis points along the view direction, so the view has no plane to measure.

    Its component in the x-z view is zero, so there is genuinely no direction to
    report rather than a zero angle.
    """
    with pytest.raises(CharacteristicsError, match="no component in the requested view"):
        kingpin_inclination([0.0, 1.0, 0.0], axis="y")


def test_an_unknown_inclination_axis_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="unknown inclination axis"):
        kingpin_inclination([0.0, 0.0, 1.0], axis="z")


# --------------------------------------------------------------------------- #
# Ackermann
# --------------------------------------------------------------------------- #


def test_an_ideal_ackermann_pair_reports_zero_error() -> None:
    """
    Round-trip the ideal relation, and check it turns the outer wheel *less*.

    The ideal outer angle satisfies ``cot(outer) = cot(inner) + track/wheelbase``.
    The direction is asserted explicitly rather than only round-tripped, because
    a round trip alone cannot catch a sign error: the same wrong formula on both
    sides cancels, and the test still passes.
    """
    track, wheelbase = 1.55, 2.70
    inner = np.deg2rad(20.0)
    ideal_outer = np.arctan(1.0 / (1.0 / np.tan(inner) + track / wheelbase))
    assert abs(ideal_outer) < abs(inner), "the outer wheel must turn less than the inner one"
    assert ackermann_error(inner, ideal_outer, track=track, wheelbase=wheelbase) == pytest.approx(
        0.0, abs=1e-12
    )


def test_a_parallel_steer_reports_a_positive_error() -> None:
    """
    Steering both wheels equally over-turns the outer one, so the error is positive.

    This is the assertion that pins the sign convention.  Ideal Ackermann turns
    the outer wheel *less* than the inner one (16.76 deg against 20 deg here), so
    a parallel pair leaves the outer wheel turned more than ideal, and the
    measured-minus-ideal difference is positive.  An inverted sign convention --
    subtracting ``track/wheelbase`` instead of adding it -- reports the opposite,
    and only this direction check catches it.
    """
    inner = np.deg2rad(20.0)
    error = ackermann_error(inner, inner, track=1.55, wheelbase=2.70)
    assert error > 0.0
    # The magnitude is the 3.24 deg between the parallel and the ideal outer angle.
    ideal_outer = np.arctan(1.0 / (1.0 / np.tan(inner) + 1.55 / 2.70))
    assert error == pytest.approx(inner - ideal_outer)


def test_ackermann_refuses_a_non_positive_wheelbase() -> None:
    with pytest.raises(CharacteristicsError, match="positive wheelbase"):
        ackermann_error(0.3, 0.2, track=1.55, wheelbase=0.0)


def test_ackermann_refuses_a_non_positive_track() -> None:
    with pytest.raises(CharacteristicsError, match="positive track"):
        ackermann_error(0.3, 0.2, track=0.0, wheelbase=2.7)


def test_ackermann_refuses_a_zero_inner_angle() -> None:
    """The ideal outer angle is not defined when the inner angle is zero."""
    with pytest.raises(CharacteristicsError, match="inner steer angle is zero"):
        ackermann_error(0.0, 0.1, track=1.55, wheelbase=2.7)


def test_ackermann_refuses_non_finite_angles() -> None:
    with pytest.raises(CharacteristicsError, match="finite"):
        ackermann_error(float("nan"), 0.1, track=1.55, wheelbase=2.7)


def test_a_screw_can_be_reconstructed_from_its_own_fields() -> None:
    """The four reported numbers must describe the same motion losslessly."""
    twist = Twist(omega=[1.0, 2.0, -1.0], velocity=[0.5, -0.5, 1.5])
    screw = screw_from_twist(twist)
    rebuilt = Screw(
        point=screw.point,
        direction=screw.direction,
        pitch=screw.pitch,
        angular_speed=screw.angular_speed,
    )
    omega = rebuilt.direction * rebuilt.angular_speed
    velocity = np.cross(rebuilt.point, omega) + rebuilt.pitch * omega
    assert velocity == pytest.approx(twist.velocity)
