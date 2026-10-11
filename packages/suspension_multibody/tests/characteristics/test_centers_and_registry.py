"""
P1 step 3 acceptance: Twist/centre/axis operators and the definition catalogue.

The centre definitions are checked against cases whose answer can be derived by
hand, because a centre formula that is merely self-consistent can still be
measuring the wrong thing.  In particular:

* the vertical forces must make the virtual work vanish on **every** admissible
  contact motion, not just the one column that happens to be convenient;
* scaling every weight together must not move the centre, while changing how the
  load is shared may;
* a pure translation in the side view has no finite instant centre, which is a
  different statement from "a centre very far away";
* the legacy definition keeps its own arithmetic and is refused as a substitute
  for the new one.

The registry tests pin the failure this design most needs to avoid: answering a
request for one definition with a different one.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import (
    PITCH_CENTER_FORCE_VERSION,
    ROLL_CENTER_FORCE_VERSION,
    ROLL_CENTER_LEGACY_VERSION,
    CharacteristicsError,
    contact_motion_basis,
    definition,
    definitions,
    force_equivalent_pitch_center,
    force_equivalent_roll_center,
    legacy_roll_center,
    missing_requirements,
    require_domain,
    require_operator,
    section_intersection,
    side_view_instant_center,
)

# Two contact patches, left and right, at the wheel centres.
POINTS = np.array([[0.0, -0.75, 0.30], [0.0, 0.75, 0.30]])


def _pure_heave(magnitude: float = 1.0) -> np.ndarray:
    """Both patches move straight up: the classic single bump direction."""
    return np.array([[0.0, 0.0, magnitude], [0.0, 0.0, magnitude]])


def _mirrored_roll(left: float = 1.0, right: float = -1.0) -> np.ndarray:
    """Move the patches in opposite vertical directions."""
    return np.array([[0.0, 0.0, left], [0.0, 0.0, right]])


def test_pure_heave_puts_the_roll_centre_at_the_contact_height() -> None:
    """
    With no lateral motion the centre cannot rise above the contact line.

    For a pure vertical motion ``v_y = 0``, so ``f_z = 0`` and the virtual forces
    are purely lateral; their line of action passes through the contact height.
    """
    motions = contact_motion_basis([_pure_heave()])
    line = force_equivalent_roll_center(POINTS, motions, [1.0, 1.0], section_coordinate=0.0)
    assert line.height == pytest.approx(0.30)
    assert line.vertical_forces == pytest.approx([0.0, 0.0])
    # A purely lateral resultant has no vertical component.
    assert line.force[2] == pytest.approx(0.0)


def test_lateral_and_vertical_motion_moves_the_centre_by_the_hand_value() -> None:
    """
    For one direction, ``f_z = -w * v_y / v_z`` and ``z = sum(y * f_z)/F_y``.

    Both patches sweep laterally at 1 m/s while rising at 2 m/s with unit
    weights: ``f_z = -0.5`` each, so ``sum(y*f_z) = 0`` and the centre sits at the
    contact height even though the patches are moving sideways.
    """
    motion = np.array([[0.0, 1.0, 2.0], [0.0, 1.0, 2.0]])
    line = force_equivalent_roll_center(
        POINTS, contact_motion_basis([motion]), [1.0, 1.0], section_coordinate=0.0
    )
    assert line.vertical_forces == pytest.approx([-0.5, -0.5])
    assert line.height == pytest.approx(0.30)
    # Virtual work is per contact point, so the force at each patch is dotted
    # with that patch's own velocity rather than with the total.
    forces = np.column_stack((np.zeros(2), line.weights, line.vertical_forces))
    per_point = np.einsum("ij,ij->i", motion, forces)
    assert per_point == pytest.approx([0.0, 0.0], abs=1e-12)


def test_a_purely_lateral_contact_motion_cannot_be_balanced() -> None:
    """
    No vertical motion means no vertical force can make the work vanish.

    The design asks for this to be *reported*, not fitted: a contact that only
    sweeps sideways carries no vertical load path, so there is no admissible
    distribution and a least-squares answer would invent one.
    """
    motion = np.array([[0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])
    with pytest.raises(CharacteristicsError, match="no admissible vertical motion"):
        force_equivalent_roll_center(
            POINTS, contact_motion_basis([motion]), [3.0, 1.0], section_coordinate=0.0
        )


def test_contradictory_directions_at_one_point_are_reported() -> None:
    """
    Two directions that demand different vertical forces cannot both hold.

    Both points rise by 1 in the first direction and by 2 in the second, so one
    force must satisfy both equations at each point.  Point 0 needs ``f_z = 0``
    in direction 1 but ``f_z = 0`` also holds in direction 2 while point 1 needs
    ``f_z = 0.5`` and ``f_z = 0.5``; the *inconsistent* case is built by making
    the two directions demand different values at the same point.
    """
    # At point 0 the vertical motion is 1 in both directions, but direction 1
    # demands f_z = 0 (no lateral sweep) while direction 2 demands f_z = -1.
    first = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    second = np.array([[0.0, 1.0, 1.0], [0.0, 0.0, 1.0]])
    with pytest.raises(CharacteristicsError, match="virtual work does not vanish"):
        force_equivalent_roll_center(
            POINTS, contact_motion_basis([first, second]), [1.0, 1.0], section_coordinate=0.0
        )


def test_two_consistent_directions_are_both_satisfied() -> None:
    """
    When the directions agree, every one of them must still be satisfied.

    Both points rise while sweeping laterally, with unit weights: point 0 has no
    lateral motion so it needs ``f_z = 0``, and point 1 sweeps at half its rise so
    it needs ``f_z = 0.5``.  Both directions agree, so the distribution is
    admissible and each direction's virtual work must vanish on its own.
    """
    first = np.array([[0.0, 0.0, 1.0], [0.0, -0.5, 1.0]])
    second = np.array([[0.0, 0.0, 2.0], [0.0, -1.0, 2.0]])
    basis = contact_motion_basis([first, second])
    line = force_equivalent_roll_center(POINTS, basis, [1.0, 1.0], section_coordinate=0.0)
    assert line.vertical_forces == pytest.approx([0.0, 0.5])
    # Virtual work is per contact point: sum_i V_i . F_i, not V @ F_total.
    forces = np.column_stack((np.zeros(2), line.weights, line.vertical_forces))
    for direction in range(2):
        velocity = basis[:, :, direction]
        assert np.allclose(np.einsum("ij,ij->i", velocity, forces), 0.0, atol=1e-12)


def test_scaling_every_weight_together_does_not_move_the_centre() -> None:
    """The centre depends on the load *distribution*, not its magnitude."""
    motion = np.array([[1.0, 0.0, 2.0], [0.5, 0.0, 1.5]])
    basis = contact_motion_basis([motion])
    unit = force_equivalent_roll_center(POINTS, basis, [1.0, 1.0], section_coordinate=0.0)
    scaled = force_equivalent_roll_center(POINTS, basis, [7.5, 7.5], section_coordinate=0.0)
    assert scaled.height == pytest.approx(unit.height)
    assert scaled.vertical_forces == pytest.approx(7.5 * np.asarray(unit.vertical_forces))


def test_changing_the_share_between_points_moves_the_centre() -> None:
    """Hence a reported centre is only comparable together with its weights."""
    # Lateral sweep differs between the patches, so how the weight is shared
    # between them changes the height.
    motion = np.array([[0.0, 1.0, 2.0], [0.0, 0.25, 2.0]])
    basis = contact_motion_basis([motion])
    even = force_equivalent_roll_center(POINTS, basis, [1.0, 1.0], section_coordinate=0.0)
    biased = force_equivalent_roll_center(POINTS, basis, [3.0, 1.0], section_coordinate=0.0)
    assert biased.height != pytest.approx(even.height)


def test_a_zero_lateral_resultant_leaves_the_centre_undefined() -> None:
    """Total lateral force zero is not a high centre, it is no centre."""
    motion = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 1.0]])
    with pytest.raises(CharacteristicsError, match="total lateral force is zero"):
        force_equivalent_roll_center(
            POINTS, contact_motion_basis([motion]), [1.0, -1.0], section_coordinate=0.0
        )


def test_a_zero_weight_distribution_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="nonzero load distribution"):
        force_equivalent_roll_center(
            POINTS, contact_motion_basis([_pure_heave()]), [0.0, 0.0], section_coordinate=0.0
        )


def test_an_empty_motion_basis_is_refused() -> None:
    """A centre cannot be verified against an empty motion set."""
    with pytest.raises(CharacteristicsError, match="no admissible directions"):
        force_equivalent_roll_center(
            POINTS, np.zeros((2, 3, 0)), [1.0, 1.0], section_coordinate=0.0
        )


def test_weight_count_must_match_the_contact_points() -> None:
    with pytest.raises(CharacteristicsError, match="weights for"):
        force_equivalent_roll_center(
            POINTS, contact_motion_basis([_pure_heave()]), [1.0], section_coordinate=0.0
        )


def test_pitch_centre_is_a_separate_definition_from_the_roll_centre() -> None:
    """
    The two use different force components and must not be substituted.

    The same geometry and motion give different heights, which is the concrete
    reason the design insists they are named separately.
    """
    motion = np.array([[2.0, 0.0, 1.0], [2.0, 0.0, 1.0]])
    basis = contact_motion_basis([motion])
    roll = force_equivalent_roll_center(POINTS, basis, [1.0, 1.0], section_coordinate=0.0)
    pitch = force_equivalent_pitch_center(POINTS, basis, [1.0, 1.0], section_coordinate=0.0)
    assert roll.version != pitch.version
    assert roll.kind != pitch.kind
    # Pitch uses F_x, which the roll definition leaves at zero.
    assert pitch.force[0] == pytest.approx(2.0)
    assert pitch.force[1] == pytest.approx(0.0)
    assert roll.force[1] == pytest.approx(2.0)


def test_roll_and_pitch_use_different_force_components_per_point() -> None:
    points = POINTS
    motion = np.array([[1.0, 1.0, 2.0], [1.0, 1.0, 2.0]])
    basis = contact_motion_basis([motion])
    roll = force_equivalent_roll_center(points, basis, [1.0, 1.0], section_coordinate=0.0)
    pitch = force_equivalent_pitch_center(points, basis, [1.0, 1.0], section_coordinate=0.0)
    # Roll forbids longitudinal force and pitch forbids lateral force.
    assert roll.force[0] == pytest.approx(0.0)
    assert pitch.force[1] == pytest.approx(0.0)


# --------------------------------------------------------------------------- #
# lines of action and sections
# --------------------------------------------------------------------------- #


def test_section_intersection_finds_the_crossing_point() -> None:
    point = np.array([0.0, 0.0, 0.0])
    direction = np.array([0.0, 1.0, 1.0]) / np.sqrt(2.0)
    crossing = section_intersection(point, direction, axis="z", coordinate=1.0)
    assert crossing[2] == pytest.approx(1.0)
    assert crossing[1] == pytest.approx(1.0)


def test_a_line_parallel_to_the_section_has_no_intersection() -> None:
    with pytest.raises(CharacteristicsError, match="parallel to the section"):
        section_intersection([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], axis="z", coordinate=1.0)


def test_a_line_lying_in_the_section_has_no_unique_intersection() -> None:
    """Every point of such a line is an intersection, so none can be reported."""
    with pytest.raises(CharacteristicsError, match="not unique"):
        section_intersection([1.0, 0.0, 0.0], [1.0, 0.0, 0.0], axis="z", coordinate=0.0)


def test_an_unknown_section_axis_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="unknown section axis"):
        section_intersection([0.0, 0.0, 0.0], [1.0, 1.0, 1.0], axis="w", coordinate=0.0)


# --------------------------------------------------------------------------- #
# side-view instant centre
# --------------------------------------------------------------------------- #


def test_a_pure_translation_has_no_finite_instant_centre() -> None:
    """A centre at a huge distance is a different claim from 'no centre'."""
    with pytest.raises(CharacteristicsError, match="pure translation"):
        side_view_instant_center(
            [0.0, 0.0, 0.3], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], angular_rate=0.0
        )


def test_the_instant_centre_is_the_point_with_zero_relative_velocity() -> None:
    """
    A hand-checkable case: motion at the contact plus a known angular rate.

    With ``v = (1, 0, 0)`` and ``omega_y = 1``, the centre is a distance
    ``|v|/|omega| = 1`` from the contact, perpendicular to ``v`` in the XZ plane.
    Moving from ``(0, 0, 0)`` by ``r = (v_z, -v_x)/omega = (0, -1)`` gives
    ``(0, 0, -1)``, and the velocity there must indeed be zero.
    """
    centre = side_view_instant_center(
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], angular_rate=1.0
    )
    assert centre == pytest.approx([0.0, 0.0, -1.0])
    # Verify the defining property directly rather than trusting the formula.
    radius = centre - np.array([0.0, 0.0, 0.0])
    omega = np.array([0.0, 1.0, 0.0])
    velocity_at_centre = np.array([1.0, 0.0, 0.0]) + np.cross(omega, radius)
    assert velocity_at_centre == pytest.approx([0.0, 0.0, 0.0])


# --------------------------------------------------------------------------- #
# the legacy definition and the catalogue
# --------------------------------------------------------------------------- #


def test_the_legacy_centre_keeps_its_own_arithmetic() -> None:
    """
    ``roll_center_legacy_v1`` must reproduce the historical numbers exactly.

    The original was the mean lateral patch position and
    ``-sum(y*slope)/lateral_force``, with the default lateral force of ``-2``.
    """
    patches = np.array([[0.0, -0.75, 0.30], [0.0, 0.75, 0.30]])
    slopes = np.array([0.1, -0.1])
    legacy = legacy_roll_center(patches, slopes)
    assert legacy[0] == pytest.approx(0.0)
    # sum(y*slope) = -0.075 - 0.075 = -0.15; -(-0.15)/-2 = -0.075
    assert legacy[1] == pytest.approx(-0.075)


def test_the_legacy_centre_is_not_the_new_one() -> None:
    """
    They must not be interchangeable.

    A caller naming the legacy version gets the legacy arithmetic; naming the new
    version gets the audited construction.  Silently returning one for the other
    would change every stored value.
    """
    patches = np.array([[0.0, -0.75, 0.30], [0.0, 0.75, 0.30]])
    slopes = np.array([0.1, -0.1])
    legacy = legacy_roll_center(patches, slopes)
    modern = force_equivalent_roll_center(
        patches, contact_motion_basis([_pure_heave()]), [1.0, 1.0], section_coordinate=0.0
    )
    assert modern.kind == "roll_center"
    assert modern.version == ROLL_CENTER_FORCE_VERSION
    assert legacy[1] != pytest.approx(modern.height)


def test_an_unknown_definition_version_is_refused_by_name() -> None:
    """The version field exists to prevent substituting another formula."""
    with pytest.raises(CharacteristicsError, match="has no version 9"):
        definition(ROLL_CENTER_FORCE_VERSION, 9)


def test_an_unknown_definition_kind_lists_what_exists() -> None:
    with pytest.raises(CharacteristicsError, match="implemented kinds"):
        definition("roll_center_vibes", 1)


def test_a_known_definition_in_the_wrong_domain_is_refused() -> None:
    """A frozen-state definition has no quasi-static meaning."""
    with pytest.raises(CharacteristicsError, match="cannot answer a 'kinematic' request"):
        require_domain(ROLL_CENTER_FORCE_VERSION, 2, "kinematic")


def test_the_catalogue_names_every_version_it_implements() -> None:
    kinds = {item.kind: item.version for item in definitions()}
    assert kinds[ROLL_CENTER_FORCE_VERSION] == 2
    assert kinds[PITCH_CENTER_FORCE_VERSION] == 1
    assert kinds[ROLL_CENTER_LEGACY_VERSION] == 1


def test_require_operator_refuses_an_unknown_version() -> None:
    with pytest.raises(CharacteristicsError, match="unknown centre definition version"):
        require_operator("no_such_version")
    assert callable(require_operator(ROLL_CENTER_FORCE_VERSION))


def test_missing_requirements_are_named_not_discovered_at_call_time() -> None:
    """A boundary that declared no contact motions is a named gap."""
    found = definition(ROLL_CENTER_FORCE_VERSION, 2)
    assert missing_requirements(found, {"contact_points": None}) == (
        "contact_motions",
        "load_weights",
    )
    assert missing_requirements(found, dict.fromkeys(found.requires)) == ()


def test_contact_motion_basis_rejects_an_empty_basis() -> None:
    with pytest.raises(CharacteristicsError, match="at least one direction"):
        contact_motion_basis([])
