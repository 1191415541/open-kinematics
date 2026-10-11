"""
True time derivatives of dynamic geometry, checked against an independent oracle.

The property under test is not "the numbers are close to something".  It is that the
reported rate is the *time derivative of the geometry's own definition*, computed from
stored velocities rather than from the sample grid.  Three consequences are tested
separately, because each fails on its own:

1. the rate equals the analytic derivative of the same formula (checked against a
   central difference taken outside this package, so the two share no code);
2. a moving reference frame is subtracted -- a rate that omits the observer's own
   rotation is exactly the one a stationary fixture cannot distinguish;
3. a stopped body reports an exact zero instead of a missing value, and a stopped
   *input* refuses rather than passing the time derivative off as a partial one.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics.values import CharacteristicsError
from suspension_multibody.results.trajectory import (
    DYNAMIC_GEOMETRIES,
    ZERO_FLOOR,
    geometry_rate_request,
    pose_rates,
)

# --------------------------------------------------------------------------- #
# an independent oracle: poses and formulas written out here, sharing no code with
# the module under test (no import of its helpers, no reuse of its offsets)
# --------------------------------------------------------------------------- #

SPIN = 0.7
VELOCITY = np.array([0.3, -0.2, 0.05])
ORIGIN = np.array([1.0, 2.0, 0.5])
START_ANGLE = 0.4


def _quaternion(axis: np.ndarray, angle: float) -> np.ndarray:
    unit = np.asarray(axis, dtype=float) / np.linalg.norm(axis)
    return np.concatenate(([np.cos(angle / 2)], unit * np.sin(angle / 2)))


def _matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float) / np.linalg.norm(quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _state(position, quaternion, velocity, omega) -> np.ndarray:
    return np.concatenate([position, quaternion, velocity, omega])


def _resolved(axis: np.ndarray, spin_axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the wheel axis in the world frame, computed from the pose alone."""
    return _matrix(_quaternion(spin_axis, angle)) @ np.asarray(axis, dtype=float)


def _oracle_value(kind: str, axis, spin_axis, angle: float) -> float:
    vector = _resolved(axis, spin_axis, angle)
    if kind == "toe":
        return float(np.arctan2(-vector[0], vector[1]))
    return float(np.arctan2(vector[2], np.hypot(vector[0], vector[1])))


def _sequence(spin_axis, *, times=(0.0, 1e-6)) -> np.ndarray:
    spin = np.asarray(spin_axis, dtype=float) * SPIN
    unit = spin / np.linalg.norm(spin)
    rows = []
    for time in times:
        angle = START_ANGLE + np.linalg.norm(spin) * time
        rows.append(
            _state(ORIGIN + VELOCITY * time, _quaternion(unit, angle), VELOCITY, spin)
        )
    return np.array(rows)


def _oracle_rate(kind: str, axis, spin_axis, *, dt: float = 1e-6) -> float:
    """Central difference of the value formula -- the oracle, not the implementation."""
    span = np.linalg.norm(np.asarray(spin_axis, dtype=float) * SPIN)
    angle = START_ANGLE + span * dt
    # The rate is taken about the same absolute angle the module is given, so the two
    # are compared at one point rather than at two different phases of the motion.
    plus = _oracle_value(kind, axis, spin_axis, START_ANGLE + span * dt)
    minus = _oracle_value(kind, axis, spin_axis, START_ANGLE - span * dt)
    del angle
    return (plus - minus) / (2 * dt)


# --------------------------------------------------------------------------- #
# 1. the rate is the derivative of the value's own definition
# --------------------------------------------------------------------------- #

AXES = {
    "toe": np.array([0.0, 1.0, 0.0]),
    "camber": np.array([0.3, 0.9, 0.2]),
}
SPINS = {
    # about z: the wheel steers, so toe changes and camber cannot (the axis keeps its
    # z-component).  About x: camber changes.  Both axes are needed, or one geometry's
    # rate is only ever tested at a constant value.
    "z": np.array([0.0, 0.0, 1.0]),
    "x": np.array([1.0, 0.0, 0.0]),
}


def test_the_geometries_are_declared() -> None:
    assert set(DYNAMIC_GEOMETRIES) == {"toe", "camber", "wheel_center"}


def test_an_unknown_geometry_is_refused_by_name() -> None:
    with pytest.raises(CharacteristicsError, match="unknown dynamic geometry"):
        geometry_rate_request(_sequence(SPINS["z"]), kind="caster")


@pytest.mark.parametrize("kind", ["toe", "camber"])
def test_the_value_matches_the_independent_formula(kind: str) -> None:
    """The module and the oracle must agree on the value before the rate means anything."""
    states = _sequence(SPINS["z"])
    result = geometry_rate_request(states, kind=kind, axis=AXES[kind])
    expected = _oracle_value(kind, AXES[kind], SPINS["z"], START_ANGLE)
    assert float(result.value.values[0]) == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize(
    ("kind", "spin"),
    [("toe", "z"), ("camber", "x")],
)
def test_the_rate_matches_a_central_difference_of_the_formula(
    kind: str, spin: str
) -> None:
    """
    The rate must equal the derivative of the same formula the value uses.

    The oracle differences the *formula*, not the module's output, so a module that
    reported a sample-to-sample quotient would disagree here as soon as the sample
    spacing changed -- which is the defect this asserts against.
    """
    states = _sequence(SPINS[spin])
    result = geometry_rate_request(states, kind=kind, axis=AXES[kind])
    assert float(result.rate.values[0]) == pytest.approx(
        _oracle_rate(kind, AXES[kind], SPINS[spin]), abs=1e-9
    )


def test_the_rate_does_not_come_from_the_sample_spacing() -> None:
    """
    Changing the sample spacing must not change the rate.

    A difference quotient scales with ``1/dt``; the analytic rate does not.  Halving
    the spacing while keeping the same state leaves the answer untouched, so this
    separates a true derivative from a trajectory ratio in one assertion.
    """
    spin = SPINS["x"]
    for dt in (1e-4, 1e-6, 1e-8):
        states = _sequence(spin, times=(0.0, dt))
        coarse = geometry_rate_request(states, kind="camber", axis=AXES["camber"])
        assert float(coarse.rate.values[0]) == pytest.approx(
            float(geometry_rate_request(
                _sequence(spin), kind="camber", axis=AXES["camber"]
            ).rate.values[0]),
            abs=1e-12,
        )


def test_a_stationary_body_reports_an_exact_zero_rate() -> None:
    """
    Zero is a reading, not a missing value.

    A stopped wheel's toe genuinely does not change, so the answer is exactly zero and
    must stay a *valid* sample.  Reporting it as unavailable would drop a real point
    from every subsequent table.
    """
    frozen = np.array(
        [
            _state(
                ORIGIN,
                _quaternion(SPINS["z"], START_ANGLE),
                np.zeros(3),
                np.zeros(3),
            )
        ]
        * 2
    )
    for kind in ("toe", "camber"):
        result = geometry_rate_request(frozen, kind=kind, axis=AXES[kind])
        assert float(result.rate.values[0]) == 0.0
        assert set(result.rate.status) == {"valid"}


# --------------------------------------------------------------------------- #
# 2. the moving reference frame
# --------------------------------------------------------------------------- #


def test_a_reference_moving_with_the_body_leaves_a_zero_relative_rate() -> None:
    """
    Co-rotating observer: the geometry is still, so its rate must be exactly zero.

    This is the decisive moving-frame test.  An implementation that keeps only the
    body's own rotation reports the full absolute rate here (0.7 rad/s), which looks
    entirely plausible; only subtracting the reference's rotation distinguishes them.
    """
    body = _sequence(SPINS["z"])
    result = geometry_rate_request(
        body, kind="toe", axis=AXES["toe"], reference_states=body
    )
    assert float(result.rate.values[0]) == 0.0
    # And the absolute rate really is non-zero, so the assertion above is not vacuous.
    absolute = geometry_rate_request(body, kind="toe", axis=AXES["toe"])
    assert abs(float(absolute.rate.values[0])) > 0.1


def test_a_still_reference_is_equivalent_to_the_world_frame() -> None:
    """
    A reference that is not moving must change nothing.

    This pins the *other* side of the moving-frame subtraction: the reference terms have
    to be genuinely used (see the co-rotating test) and they have to vanish when there is
    nothing to subtract.  An implementation that always removed a fixed rotation, or that
    subtracted the reference's pose rather than its rate, fails here.
    """
    body = _sequence(SPINS["z"])
    still = np.array(
        [
            _state(
                ORIGIN, _quaternion(SPINS["z"], START_ANGLE), np.zeros(3), np.zeros(3)
            )
        ]
        * 2
    )
    with_reference = geometry_rate_request(
        body, kind="toe", axis=AXES["toe"], reference_states=still
    )
    without_reference = geometry_rate_request(body, kind="toe", axis=AXES["toe"])
    assert float(with_reference.rate.values[0]) == pytest.approx(
        float(without_reference.rate.values[0]), abs=1e-15
    )
    assert not with_reference.reference_moving


def test_a_translating_reference_shifts_the_rate_by_its_own_translation() -> None:
    """
    A reference translating (but not rotating) changes a *position* rate and not an angle.

    The two branches must therefore be separated: wheel-centre subtracts the carried
    translation, while toe subtracts only rotation.  Conflating them would corrupt one
    geometry while leaving the other looking correct.
    """
    body = _sequence(SPINS["z"])
    # The reference must be a *separate* state sequence.  Mutating `body` here would
    # move the measured body as well, and the two would co-move -- which is a different
    # test, and one that cannot see a translation-only error.
    #
    # The reference is given zero omega on purpose.  Leaving the body's own rotation in
    # place would make the two co-rotate, and the angle rate would then be zero for a
    # reason that has nothing to do with the translation under test.
    moving = body.copy()
    moving[:, 7:10] = VELOCITY + np.array([1.0, 0.0, 0.0])
    moving[:, 10:13] = 0.0
    # The point is offset from the body origin on purpose.  With `origin=[0,0,0]` the
    # lever arm is zero and the *position* columns of the reference never enter the
    # answer at all, so the relative-position subtraction is untested -- confirmed by
    # sabotage: dropping the reference's local offset survived that version.
    offset = np.array([0.0, 0.4, 0.0])
    centre = geometry_rate_request(
        body,
        kind="wheel_center",
        origin=offset,
        reference_states=moving,
    )
    # Both bodies carry the same pose, so the relative *value* is zero and the relative
    # rate is the difference of the two carried velocities.  The measured body spins, so
    # its own `omega x r` term is present and the reference (omega cleared) contributes
    # only its translation -- which is what makes this a translation-only test.
    rotation = _matrix(_quaternion(SPINS["z"], START_ANGLE))
    body_rate = VELOCITY + np.cross(SPINS["z"] * SPIN, rotation @ offset)
    expected = body_rate - (VELOCITY + np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(centre.value.values[0], [0.0, 0.0, 0.0], atol=1e-12)
    np.testing.assert_allclose(centre.rate.values[0], expected, atol=1e-12)
    # The lever term is large enough that an implementation dropping it fails here.
    assert np.linalg.norm(np.cross(SPINS["z"] * SPIN, rotation @ offset)) > 0.1
    # An angle is unaffected by a reference's linear motion.
    toe = geometry_rate_request(
        body, kind="toe", axis=AXES["toe"], reference_states=moving
    )
    plain = geometry_rate_request(body, kind="toe", axis=AXES["toe"])
    assert float(toe.rate.values[0]) == pytest.approx(
        float(plain.rate.values[0]), abs=1e-15
    )


def test_a_rotating_reference_offset_is_carried_correctly() -> None:
    """
    The reference's own offset must be rotated before it is used.

    This is the case that catches a reference position used *unrotated*: with the
    reference spinning about z, its local offset swings, and both its world position and
    its carried velocity change.  The measured body is held at a different pose so the
    two contributions cannot cancel.
    """
    # Reference: at ORIGIN, spinning about z, no translation.
    reference = np.array(
        [
            _state(
                ORIGIN,
                _quaternion(SPINS["z"], START_ANGLE),
                np.zeros(3),
                SPINS["z"] * SPIN,
            )
        ]
        * 2
    )
    # Measured body: displaced along x, not spinning, with a point offset from its own
    # origin.  Nothing here can cancel against the reference's rotation.
    body = np.array(
        [
            _state(
                ORIGIN + np.array([1.5, 0.0, 0.0]),
                _quaternion(SPINS["z"], START_ANGLE),
                np.zeros(3),
                np.zeros(3),
            )
        ]
        * 2
    )
    local = np.array([0.2, 0.0, 0.0])
    result = geometry_rate_request(
        body, kind="wheel_center", origin=local, reference_states=reference
    )
    rotation = _matrix(_quaternion(SPINS["z"], START_ANGLE))
    # Reference point in world coordinates, with its offset rotated.
    reference_point = ORIGIN + rotation @ local
    reference_rate = np.cross(SPINS["z"] * SPIN, reference_point - ORIGIN)
    expected_value = (ORIGIN + np.array([1.5, 0.0, 0.0]) + rotation @ local) - reference_point
    np.testing.assert_allclose(result.value.values[0], expected_value, atol=1e-12)
    np.testing.assert_allclose(result.rate.values[0], -reference_rate, atol=1e-12)
    # Both terms are non-zero, so the assertions above are not vacuous.
    assert np.linalg.norm(expected_value) > 0.1
    assert np.linalg.norm(reference_rate) > 0.01


def test_a_rotating_reference_changes_the_camber_rate() -> None:
    """
    Camber's moving-reference term must be subtracted too, not only toe's.

    Both geometries share `_axis_rate`, so testing the observer term through toe alone
    cannot see a sign error that is common to the two.  Camber needs its own case, and
    the spin axis must not be aligned with the wheel axis: co-rotation about an aligned
    axis leaves camber's rate at exactly zero for a reason unrelated to the term.
    """
    spin = np.array([0.4, 0.0, 0.6])
    body = _sequence(spin)
    # A stationary reference that is *rotated* away from the body: the axis then has a
    # genuinely different camber in the two frames, so the observer term cannot vanish.
    reference = np.array(
        [
            _state(
                ORIGIN,
                _quaternion(np.array([0.0, 1.0, 0.0]), 0.3),
                np.zeros(3),
                np.zeros(3),
            )
        ]
        * 2
    )
    with_reference = geometry_rate_request(
        body, kind="camber", axis=AXES["camber"], reference_states=reference
    )
    without_reference = geometry_rate_request(
        body, kind="camber", axis=AXES["camber"]
    )
    assert float(with_reference.rate.values[0]) != float(
        without_reference.rate.values[0]
    )
    # A still reference subtracts no *rate*, only re-expresses the axis, so the two must
    # agree once the reference starts spinning -- and then must differ from the world rate.
    co_rotating = body.copy()
    co_rotating[:, 10:13] = spin * SPIN
    relative = geometry_rate_request(
        body, kind="camber", axis=AXES["camber"], reference_states=co_rotating
    )
    assert float(relative.rate.values[0]) == pytest.approx(0.0, abs=1e-15)


def test_a_moving_reference_is_reported_as_such() -> None:
    """The flag must reflect reality, since a consumer uses it to judge the rate."""
    body = _sequence(SPINS["z"])
    moving = geometry_rate_request(
        body, kind="toe", axis=AXES["toe"], reference_states=body
    )
    assert moving.reference_moving
    still = geometry_rate_request(body, kind="toe", axis=AXES["toe"])
    assert not still.reference_moving


def test_a_reference_on_a_different_grid_is_refused() -> None:
    body = _sequence(SPINS["z"])
    with pytest.raises(CharacteristicsError, match="share a sample grid"):
        geometry_rate_request(
            body, kind="toe", axis=AXES["toe"], reference_states=body[:1]
        )


# --------------------------------------------------------------------------- #
# 3. wheel centre, and what must be refused
# --------------------------------------------------------------------------- #


def test_the_wheel_centre_rate_includes_the_lever_arm_rotation() -> None:
    """
    A point offset from the body origin moves with ``v + omega x r``.

    Dropping the ``omega x r`` term gives the body's own velocity, which is right only
    for a point at the origin -- so the origin-offset case is the one that tests it.
    """
    body = _sequence(SPINS["x"])
    result = geometry_rate_request(
        body, kind="wheel_center", origin=np.array([0.0, 0.5, 0.0])
    )
    lever = np.einsum(
        "ij,j->i", _matrix(_quaternion(SPINS["x"], START_ANGLE)), [0.0, 0.5, 0.0]
    )
    expected = VELOCITY + np.cross(SPINS["x"] * SPIN, lever)
    np.testing.assert_allclose(result.rate.values[0], expected, atol=1e-12)
    # And it must differ from the plain body velocity, or the lever term is missing.
    assert not np.allclose(result.rate.values[0], VELOCITY, atol=1e-9)


def test_a_vertical_wheel_axis_refuses_a_toe_rate() -> None:
    """
    A vertical axis has no horizontal projection, so toe has no value to differentiate.

    Reporting a large finite rate here would be a number with no definition behind it.
    """
    vertical = np.array(
        [
            _state(
                ORIGIN, _quaternion(SPINS["z"], START_ANGLE), VELOCITY, SPINS["z"] * SPIN
            )
        ]
        * 2
    )
    with pytest.raises(CharacteristicsError, match="no horizontal projection"):
        geometry_rate_request(vertical, kind="toe", axis=np.array([0.0, 0.0, 1.0]))


def test_a_stopped_input_refuses_a_partial_derivative() -> None:
    """
    The time derivative must not be passed off as an input partial derivative.

    With a stopped input the geometry's dependence on it cannot be separated from the
    rest of the motion, so the contract calls for a refusal by name rather than the
    time derivative under a different label.
    """
    body = _sequence(SPINS["z"])
    with pytest.raises(CharacteristicsError, match="stopped input"):
        geometry_rate_request(
            body,
            kind="toe",
            axis=AXES["toe"],
            input_rate=np.zeros(body.shape[0]),
        )


def test_a_moving_input_is_recorded_in_the_driver() -> None:
    """A rate that used an input rate must say so, or it is indistinguishable."""
    body = _sequence(SPINS["z"])
    plain = geometry_rate_request(body, kind="toe", axis=AXES["toe"])
    assert plain.driver == "pose_rate"
    driven = geometry_rate_request(
        body, kind="toe", axis=AXES["toe"], input_rate=np.array([1.0, 1.0])
    )
    assert driven.driver == "pose_rate+input_rate"


def test_a_scaled_axis_distinguishes_the_squared_projection() -> None:
    """
    Toe's derivative divides by ``h^2``, and ``h^2 != h`` unless the axis is a unit vector.

    Every other fixture here uses a unit axis, where ``h`` and ``h^2`` coincide and a
    reader that divides by the wrong power of the horizontal projection still passes.
    An axis of length 2 is the cheapest way to separate them: the value is unchanged
    (``atan2`` is scale-invariant) but the rate scales by the axis length, so dividing
    by ``h`` instead of ``h^2`` gives an answer off by a factor of ``h``.
    """
    scaled = np.array([0.0, 2.0, 0.0])
    states = _sequence(SPINS["z"])
    result = geometry_rate_request(states, kind="toe", axis=scaled)
    # The value cannot see the scale, which is exactly why the rate is the only witness.
    unit = geometry_rate_request(states, kind="toe", axis=AXES["toe"])
    assert float(result.value.values[0]) == pytest.approx(
        float(unit.value.values[0]), abs=1e-12
    )
    # The rate must equal the derivative of the *scaled* geometry's own definition.
    # Note the true rate is scale-invariant (atan2 ignores the axis length, so its
    # derivative does too) -- which is precisely why this case is decisive: the correct
    # answer here equals the unit-axis answer, while a reader that divides by ``h``
    # instead of ``h^2`` returns twice it.  Comparing against the oracle, not against
    # the unit-axis number, is what makes the distinction visible.
    assert float(result.rate.values[0]) == pytest.approx(
        _oracle_rate("toe", scaled, SPINS["z"]), abs=1e-9
    )
    # Pin the scale-invariance too, so a future change that made the rate depend on the
    # axis length would be caught rather than absorbed into a tolerance.
    assert float(result.rate.values[0]) == pytest.approx(
        float(unit.rate.values[0]), abs=1e-9
    )


def test_a_scaled_wheel_centre_lever_reaches_the_same_point() -> None:
    """
    Scaling a body-local point scales its world offset, and the rate along with it.

    A cheap second use of the same distinction as the toe case above: here the geometry
    is linear, so an implementation that mishandled the local-to-world map would show up
    as a rate that does not double when the lever does.
    """
    states = _sequence(SPINS["x"])
    near = geometry_rate_request(
        states, kind="wheel_center", origin=np.array([0.0, 0.5, 0.0])
    )
    far = geometry_rate_request(
        states, kind="wheel_center", origin=np.array([0.0, 1.0, 0.0])
    )
    lever_rotation = np.cross(SPINS["x"] * SPIN, np.einsum(
        "ij,j->i", _matrix(_quaternion(SPINS["x"], START_ANGLE)), [0.0, 0.5, 0.0]
    ))
    np.testing.assert_allclose(
        far.rate.values[0] - VELOCITY, 2 * (near.rate.values[0] - VELOCITY), atol=1e-12
    )
    np.testing.assert_allclose(near.rate.values[0] - VELOCITY, lever_rotation, atol=1e-12)


def test_the_wheel_centre_branch_also_validates_an_input_rate() -> None:
    """
    The input-rate guard must run on the vector branch too, not only on the scalars.

    The two branches return different value types and the guard sits in each
    separately, so covering it through toe says nothing about wheel centre -- a stopped
    input there would silently pass and the time derivative would stand in for a
    partial one.
    """
    body = _sequence(SPINS["z"])
    stopped = np.zeros(body.shape[0])
    with pytest.raises(CharacteristicsError, match="stopped input"):
        geometry_rate_request(
            body, kind="wheel_center", origin=np.array([0.0, 0.3, 0.0]),
            input_rate=stopped,
        )
    # And a moving input is recorded on this branch as well.
    driven = geometry_rate_request(
        body, kind="wheel_center", origin=np.array([0.0, 0.3, 0.0]),
        input_rate=np.ones(body.shape[0]),
    )
    assert driven.driver == "pose_rate+input_rate"
    # A rate with the wrong sample count is refused rather than broadcast.
    with pytest.raises(CharacteristicsError, match="one value per sample"):
        geometry_rate_request(
            body, kind="wheel_center", origin=np.array([0.0, 0.3, 0.0]),
            input_rate=np.ones(1),
        )


def test_a_pose_without_a_rate_is_refused() -> None:
    """
    A 7-column state is a pose, and a pose carries no rate.

    Accepting it would force the module to manufacture one by differencing, which is
    the fallback this module exists to avoid.
    """
    pose_only = _sequence(SPINS["z"])[:, :7]
    with pytest.raises(CharacteristicsError, match="body state"):
        geometry_rate_request(pose_only, kind="toe", axis=AXES["toe"])


def test_pose_rates_returns_the_stored_velocities() -> None:
    """The stored velocity is read, not reconstructed from positions."""
    body = _sequence(SPINS["z"])
    velocity, omega = pose_rates(body)
    np.testing.assert_allclose(velocity[0], VELOCITY, atol=1e-15)
    np.testing.assert_allclose(omega[0], SPINS["z"] * SPIN, atol=1e-15)


def test_pose_rates_removes_the_reference_carried_term() -> None:
    """
    ``v_rel = v - (v_ref + omega_ref x r)`` -- the lever term is the part that is missed.

    A reference translating at the same velocity but spinning produces a non-zero
    relative velocity for an offset body, which is only true if the lever arm is used.
    """
    body = np.array(
        [
            _state(
                ORIGIN + np.array([1.0, 0.0, 0.0]),
                _quaternion(SPINS["z"], START_ANGLE),
                VELOCITY,
                np.zeros(3),
            )
        ]
        * 2
    )
    reference = np.array(
        [
            _state(ORIGIN, _quaternion(SPINS["z"], START_ANGLE), VELOCITY, SPINS["z"])
        ]
        * 2
    )
    velocity, omega = pose_rates(body, reference_states=reference)
    # Pure linear subtraction would give zero; the lever term makes it non-zero.
    np.testing.assert_allclose(velocity[0], -np.cross(SPINS["z"], [1.0, 0.0, 0.0]))
    np.testing.assert_allclose(omega[0], -SPINS["z"])


def test_the_zero_floor_is_not_a_rounding_tolerance() -> None:
    """Guard the constant's meaning: it separates "stopped" from "cancelled"."""
    assert 0.0 < ZERO_FLOOR < 1e-9
