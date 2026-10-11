"""
P1 step 1 acceptance: the unified value types and measurement operators.

Two properties are tested, and the second is the one that makes the first worth
anything:

* **The value types refuse to invent data.**  An unavailable sample carries a
  contract status and a reason, never a zero and never a repeated previous
  reading -- those are indistinguishable from measurements once written to a
  table.  A valid sample may not claim a reason, and an invalid one may not carry
  a number.

* **The moved formulas agree with the ones they replace.**  R1 retires the
  frame-position/toe/camber arithmetic out of ``ResultEnvelope.measure``.  The
  comparison below recomputes the *old* expressions inline and asserts the new
  operators match them sample for sample, so the claim is "same numbers", not
  "a similar-looking implementation exists".
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import (
    DOMAINS,
    SAMPLE_STATUSES,
    CharacteristicResult,
    CharacteristicsError,
    DerivativeValue,
    MeasurementValue,
    SampleIdentity,
    VectorValue,
    camber_angle,
    frame_position,
    legacy_lateral_axis_y,
    legacy_wheel_camber,
    legacy_wheel_toe,
    statuses_for,
    toe_angle,
    transformed_point,
    wheel_center,
)
from suspension_multibody.modeling.primitives.spatial import quaternion_to_matrix


def _poses(rotations, translations) -> np.ndarray:
    """Build an (samples, 4, 4) pose batch from rotation vectors and origins."""
    out = np.broadcast_to(np.eye(4), (len(translations), 4, 4)).copy()
    for index, (axis, angle) in enumerate(rotations):
        half = 0.5 * angle
        scale = np.sin(half)
        quaternion = np.array([np.cos(half), *(scale * np.asarray(axis, dtype=float))])
        out[index, :3, :3] = quaternion_to_matrix(quaternion)
        out[index, :3, 3] = translations[index]
    return out


def _sample_poses() -> np.ndarray:
    return _poses(
        [((0, 0, 1), 0.0), ((0, 0, 1), 0.1), ((0, 1, 0), -0.05)],
        [(1.0, 2.0, 3.0), (1.1, 2.0, 3.0), (1.2, 2.05, 3.0)],
    )


# --------------------------------------------------------------------------- #
# value types
# --------------------------------------------------------------------------- #


def test_measurement_tracks_validity_per_sample() -> None:
    value = MeasurementValue(
        measurement_id="front.left.camber",
        values=[0.01, np.nan, -0.02],
        units="rad",
        frame_id="world",
        status=("valid", "undefined_geometry", "valid"),
        reason=("", "axis has no length", ""),
    )
    assert len(value) == 3
    assert value.valid_mask.tolist() == [True, False, True]


def test_an_invalid_sample_must_not_carry_a_number() -> None:
    """Filling in a zero would be indistinguishable from a real reading of zero."""
    with pytest.raises(CharacteristicsError, match="must not be filled in"):
        MeasurementValue(
            measurement_id="m",
            values=[0.0],
            units="rad",
            frame_id="world",
            status=("singular",),
            reason=("degenerate",),
        )


def test_an_invalid_sample_must_say_why() -> None:
    with pytest.raises(CharacteristicsError, match="without a reason"):
        MeasurementValue(
            measurement_id="m",
            values=[np.nan],
            units="rad",
            frame_id="world",
            status=("singular",),
            reason=("",),
        )


def test_a_valid_sample_must_not_claim_a_reason() -> None:
    with pytest.raises(CharacteristicsError, match="valid but carries"):
        MeasurementValue(
            measurement_id="m",
            values=[1.0],
            units="rad",
            frame_id="world",
            status=("valid",),
            reason=("somehow invalid",),
        )


def test_status_length_must_match_the_samples() -> None:
    with pytest.raises(CharacteristicsError, match="statuses"):
        MeasurementValue(
            measurement_id="m",
            values=[1.0, 2.0],
            units="rad",
            frame_id="world",
            status=("valid",),
            reason=("", ""),
        )


def test_unknown_status_is_rejected() -> None:
    with pytest.raises(CharacteristicsError, match="unknown status"):
        MeasurementValue(
            measurement_id="m",
            values=[np.nan],
            units="rad",
            frame_id="world",
            status=("probably_fine",),
            reason=("guess",),
        )


def test_values_are_read_only() -> None:
    """A later in-place edit must not look like a change of physics."""
    source = np.array([1.0, 2.0])
    value = MeasurementValue.all_valid("m", source, "rad", "world")
    source[0] = 99.0
    assert value.values[0] == 1.0
    with pytest.raises(ValueError):
        value.values[0] = 5.0


def test_a_six_axis_quantity_has_no_single_unit() -> None:
    with pytest.raises(CharacteristicsError, match="no single unit"):
        VectorValue(
            measurement_id="wrench",
            values=np.zeros((2, 6)),
            units=("N",),
            frame_id="world",
            status=("valid", "valid"),
            reason=("", ""),
        )


def test_derivative_keeps_both_unit_sides() -> None:
    with pytest.raises(CharacteristicsError, match="both sides"):
        DerivativeValue(
            measurement_id="d",
            perturbation_id="p",
            values=[1.0],
            numerator_units="rad",
            denominator_units="",
            method="analytic",
            status=("valid",),
            reason=("",),
        )


def test_a_difference_quotient_cannot_pose_as_a_production_response() -> None:
    """Only the test oracle may take the difference route."""
    with pytest.raises(CharacteristicsError, match="oracle, not a fallback"):
        DerivativeValue(
            measurement_id="d",
            perturbation_id="p",
            values=[1.0],
            numerator_units="rad",
            denominator_units="m",
            method="looks_about_right",
            status=("valid",),
            reason=("",),
        )


def test_statuses_for_refuses_to_swallow_a_nan() -> None:
    """A NaN with no explanation is exactly the silent hole this guards."""
    with pytest.raises(CharacteristicsError, match="no reason"):
        statuses_for([1.0, np.nan])
    status, reason = statuses_for([1.0, np.nan], ["", "no axis"])
    assert status == ("valid", "undefined_geometry")
    assert reason == ("", "no axis")


def test_result_rejects_a_response_length_mismatch() -> None:
    response = MeasurementValue.all_valid("m", [1.0, 2.0], "rad", "world")
    with pytest.raises(CharacteristicsError, match="selected"):
        CharacteristicResult(
            request_id="r",
            domain="kinematic",
            definition_kind="measurement_derivative",
            definition_version=1,
            samples=(SampleIdentity(index=0),),
            responses={"m": response},
            boundary_profile="p",
            boundary_version=1,
        )


def test_result_unknown_measurement_names_what_it_has() -> None:
    response = MeasurementValue.all_valid("m", [1.0], "rad", "world")
    result = CharacteristicResult(
        request_id="r",
        domain="kinematic",
        definition_kind="measurement_derivative",
        definition_version=1,
        samples=(SampleIdentity(index=0),),
        responses={"m": response},
        boundary_profile="p",
        boundary_version=1,
    )
    assert result.measurement("m") is response
    with pytest.raises(CharacteristicsError, match="answered: m"):
        result.measurement("nope")


def test_domains_and_statuses_match_the_contract() -> None:
    assert DOMAINS == ("kinematic", "equilibrium", "trajectory", "dynamic")
    assert "undefined_geometry" in SAMPLE_STATUSES
    assert "not_equilibrium" in SAMPLE_STATUSES


# --------------------------------------------------------------------------- #
# R1: the moved formulas agree with the ones they replace
# --------------------------------------------------------------------------- #


def test_frame_position_matches_the_retired_expression() -> None:
    """Recompute the old inline expression and compare, sample for sample."""
    pose = _sample_poses()
    reference = _poses([((0, 0, 1), 0.0)], [(0.5, -0.25, 0.1)])[0:1]
    reference = np.broadcast_to(reference, pose.shape)

    old = np.einsum(
        "nji,nj->ni", reference[:, :3, :3], pose[:, :3, 3] - reference[:, :3, 3]
    )
    assert np.array_equal(frame_position(pose, reference), old)
    assert np.array_equal(wheel_center(pose, reference), old)


def test_toe_matches_the_retired_expression() -> None:
    pose = _sample_poses()
    reference = _poses([((0, 0, 1), 0.0)], [(0.0, 0.0, 0.0)])[0:1]
    reference = np.broadcast_to(reference, pose.shape)
    axis = np.array([1.0, 0.0, 0.0])

    vector = np.einsum("nij,j->ni", pose[:, :3, :3], axis)
    vector = np.einsum("nji,nj->ni", reference[:, :3, :3], vector)
    old = np.arctan2(-vector[:, 0], vector[:, 1])

    assert np.array_equal(toe_angle(pose, reference, axis), old)


def test_camber_matches_the_retired_expression() -> None:
    pose = _sample_poses()
    reference = _poses([((0, 0, 1), 0.0)], [(0.0, 0.0, 0.0)])[0:1]
    reference = np.broadcast_to(reference, pose.shape)
    axis = np.array([0.0, 1.0, 0.0])

    vector = np.einsum("nij,j->ni", pose[:, :3, :3], axis)
    vector = np.einsum("nji,nj->ni", reference[:, :3, :3], vector)
    old = np.arctan2(vector[:, 2], np.hypot(vector[:, 0], vector[:, 1]))

    assert np.array_equal(camber_angle(pose, reference, axis), old)


def test_sign_convention_flips_the_reported_angle() -> None:
    """The side sign is a declared input, not a property of the geometry."""
    pose = _poses([((0, 0, 1), 0.3)], [(0.0, 0.0, 0.0)])
    axis = np.array([0.0, 1.0, 0.0])
    assert toe_angle(pose, pose, axis, sign=-1) == -toe_angle(pose, pose, axis)


def test_a_rotation_of_the_reference_frame_changes_the_angle() -> None:
    """Toe is frame-relative: rotating the reference must move the reading."""
    pose = _poses([((0, 0, 1), 0.25)], [(0.0, 0.0, 0.0)])
    axis = np.array([0.0, 1.0, 0.0])
    upright = _poses([((0, 0, 1), 0.0)], [(0.0, 0.0, 0.0)])
    rotated = _poses([((0, 0, 1), 0.05)], [(0.0, 0.0, 0.0)])
    assert toe_angle(pose, rotated, axis) != toe_angle(pose, upright, axis)


def test_a_vertical_axis_has_no_toe() -> None:
    """Straight ahead and undefined are different answers."""
    pose = _poses([((0, 0, 1), 0.0)], [(0.0, 0.0, 0.0)])
    with pytest.raises(CharacteristicsError, match="no horizontal projection"):
        toe_angle(pose, pose, np.array([0.0, 0.0, 1.0]))


def test_a_vertical_axis_still_has_a_camber() -> None:
    pose = _poses([((0, 0, 1), 0.0)], [(0.0, 0.0, 0.0)])
    assert pytest.approx(abs(camber_angle(pose, pose, np.array([0.0, 0.0, 1.0])))) == np.pi / 2


def test_a_zero_length_axis_is_refused() -> None:
    pose = _poses([((0, 0, 1), 0.0)], [(0.0, 0.0, 0.0)])
    with pytest.raises(CharacteristicsError, match="no length"):
        camber_angle(pose, pose, np.array([0.0, 0.0, 0.0]))


def test_a_pose_batch_of_the_wrong_shape_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match=r"\(samples, 4, 4\)"):
        frame_position(np.zeros((3, 3)), np.zeros((1, 4, 4)))


def test_position_subtracts_after_both_poses_are_in_world_coordinates() -> None:
    """A pure translation of both frames by the same vector changes nothing."""
    pose = _sample_poses()
    shifted = pose.copy()
    shifted[:, :3, 3] += np.array([10.0, -4.0, 7.0])
    identity = np.broadcast_to(np.eye(4), pose.shape)
    assert np.allclose(frame_position(pose, identity), frame_position(shifted, identity) - np.array([10.0, -4.0, 7.0]))


def _toed_and_cambered_rotation() -> np.ndarray:
    """Build a wheel rotation carrying both toe and camber, so the formulas differ."""
    x, y, z = 0.17, 0.98, 0.10
    rotation = np.eye(3)
    rotation[0, 1], rotation[1, 1], rotation[2, 1] = x, y, z
    return rotation


def _pose_from_rotation(rotation: np.ndarray) -> np.ndarray:
    """Lift a 3x3 rotation into a homogeneous 4x4 pose batch of length one."""
    pose = np.eye(4)
    pose[:3, :3] = rotation
    return pose[None]


def test_the_legacy_kc_camber_is_not_the_unified_camber() -> None:
    """
    The two camber formulas genuinely differ, and both are kept.

    R6's legacy K&C output measured the spin axis against the lateral axis alone,
    ``atan2(axis_z, axis_y)``, while the unified operator measures against the
    whole horizontal plane, ``atan2(axis_z, hypot(axis_x, axis_y))``.  With any
    toe the two disagree -- by 0.085 degrees at 9.8 degrees of toe, far outside
    the K&C tolerances -- so substituting one for the other would silently move
    every stored value.  This test pins that they differ and that each matches its
    own definition.
    """
    rotation = _toed_and_cambered_rotation()
    # ``outward=-1`` makes the legacy sign convention positive here, so the two
    # numbers can be compared directly instead of differing by a sign as well.
    legacy = legacy_wheel_camber(rotation, outward=-1.0)
    unified = float(
        np.degrees(
            camber_angle(
                _pose_from_rotation(rotation), np.eye(4)[None], [0.0, 1.0, 0.0]
            )[0]
        )
    )
    assert legacy == pytest.approx(np.degrees(np.arctan2(0.10, 0.98)))
    assert unified == pytest.approx(np.degrees(np.arctan2(0.10, np.hypot(0.17, 0.98))))
    assert legacy != pytest.approx(unified)
    # The disagreement is the one the toe causes: 0.085 degrees, well outside any
    # K&C tolerance, which is why the legacy formula could not simply be replaced.
    assert abs(legacy - unified) > 0.08


def test_the_legacy_kc_toe_matches_its_own_definition() -> None:
    """The legacy toe is ``atan2(axis_x, axis_y)``, not the unified operator."""
    rotation = _toed_and_cambered_rotation()
    assert legacy_wheel_toe(rotation, outward=1.0) == pytest.approx(
        -np.degrees(np.arctan2(0.17, 0.98))
    )
    assert legacy_lateral_axis_y(rotation) == pytest.approx(0.98)


def test_the_legacy_kc_angles_carry_the_left_right_sign_convention() -> None:
    """``outward`` flips the sign, which is how left and right are distinguished."""
    rotation = _toed_and_cambered_rotation()
    assert legacy_wheel_camber(rotation, outward=1.0) == pytest.approx(
        -legacy_wheel_camber(rotation, outward=-1.0)
    )
    assert legacy_wheel_toe(rotation, outward=1.0) == pytest.approx(
        -legacy_wheel_toe(rotation, outward=-1.0)
    )


def test_a_non_rotation_is_refused_by_the_legacy_angles() -> None:
    with pytest.raises(CharacteristicsError, match="3x3 matrix"):
        legacy_wheel_camber(np.eye(4), outward=1.0)


def test_transformed_point_matches_the_expression_it_replaces() -> None:
    """The wheel-centre output is ``origin + rotation @ local``; check each axis."""
    rotation = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    origin = np.array([1.0, 2.0, 3.0])
    local = np.array([0.5, 0.0, 0.0])
    expected = origin + rotation @ local
    for axis in range(3):
        assert transformed_point(rotation, origin, local, axis=axis) == pytest.approx(
            expected[axis]
        )


def test_transformed_point_refuses_an_unknown_component() -> None:
    with pytest.raises(CharacteristicsError, match="unknown component index"):
        transformed_point(np.eye(3), [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], axis=3)
