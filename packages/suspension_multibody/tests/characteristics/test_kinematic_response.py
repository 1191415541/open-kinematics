"""
P1 step 2 acceptance: the kinematic rank/identifiability audit.

The audit exists because "we solved a linear system" and "the mechanism's motion
is determined" are different claims.  A least-squares solve always returns
*something*; the tests below pin down that the audit refuses to let that
something be reported as a measurement when the boundary does not determine it.

The Jacobians used here are small enough to reason about by hand, so the expected
rank and nullity are computed independently rather than read back from the code
under test.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import CharacteristicsError
from suspension_multibody.characteristics.motion import (
    audit_boundary,
    is_identifiable,
    measurement_response,
    rank_and_null_space,
    sample_status_for,
    solve_motion,
)


def test_a_full_rank_system_is_unique() -> None:
    jacobian = np.array([[1.0, 0.0], [0.0, 1.0]])
    rank, null = rank_and_null_space(jacobian)
    assert rank == 2
    assert null.shape == (2, 0)


def test_an_underconstrained_system_reports_its_nullity() -> None:
    """One equation, two coordinates: one direction is genuinely free."""
    jacobian = np.array([[1.0, 1.0]])
    rank, null = rank_and_null_space(jacobian)
    assert rank == 1
    assert null.shape == (2, 1)
    # The null vector must satisfy J n = 0, checked here rather than assumed.
    assert np.allclose(jacobian @ null, 0.0, atol=1e-12)


def test_a_redundant_but_consistent_row_is_not_singular() -> None:
    """A repeated constraint does not remove a degree of freedom."""
    jacobian = np.array([[1.0, 0.0], [2.0, 0.0], [0.0, 1.0]])
    rank, null = rank_and_null_space(jacobian)
    assert rank == 2
    assert null.shape[1] == 0


def test_a_direction_inside_the_null_space_is_not_identifiable() -> None:
    """The reading cannot distinguish two motions, so it identifies neither."""
    jacobian = np.array([[1.0, 1.0]])
    assert not is_identifiable(jacobian, np.array([1.0, -1.0]))
    assert is_identifiable(jacobian, np.array([1.0, 1.0]))


def test_solve_refuses_to_invent_a_motion_when_underdetermined() -> None:
    """
    The minimum-norm answer is one arbitrary member of a family.

    Returning it as the mechanism's motion would fabricate the very degree of
    freedom the boundary failed to constrain.
    """
    with pytest.raises(CharacteristicsError, match="unconstrained direction"):
        solve_motion(np.array([[1.0, 1.0]]), np.array([2.0]))


def test_solve_returns_the_unique_motion_and_its_audit() -> None:
    motion, audit = solve_motion(np.array([[1.0, 0.0], [0.0, 2.0]]), np.array([3.0, 4.0]))
    assert np.allclose(motion, [3.0, 2.0])
    assert audit["unique"] is True
    assert audit["rank"] == 2
    assert audit["residual_norm"] == pytest.approx(0.0, abs=1e-12)


def test_inconsistent_targets_are_distinguished_from_underdetermined() -> None:
    """
    An over-constrained, unreachable target is a different defect.

    The two need different fixes, so they must not collapse into one status.
    """
    with pytest.raises(CharacteristicsError, match="inconsistent"):
        solve_motion(np.array([[1.0, 0.0], [0.0, 1.0], [1.0, 1.0]]), np.array([1.0, 1.0, 5.0]))


def test_audit_reports_unidentifiable_with_a_reason() -> None:
    audit = audit_boundary(np.array([[1.0, 1.0]]))
    assert audit["status"] == "unidentifiable"
    assert audit["nullity"] == 1
    assert "unconstrained direction" in audit["reason"]
    status, reason = sample_status_for(audit)
    assert status == "unidentifiable"
    assert reason


def test_audit_of_a_determined_system_is_valid() -> None:
    audit = audit_boundary(np.array([[1.0, 0.0], [0.0, 1.0]]))
    assert audit["status"] == "valid"
    assert audit["unique"] is True
    assert sample_status_for(audit) == ("valid", "")


def test_driven_rows_are_part_of_the_audit() -> None:
    """
    A driven coordinate is an extra row, and it can remove the last freedom.

    Without it the system is underdetermined; adding the drive must be able to
    make it unique, which is exactly what holding a coordinate *means*.
    """
    constraints = np.array([[1.0, 1.0]])
    assert audit_boundary(constraints)["unique"] is False
    driven = audit_boundary(constraints, driven_rows=np.array([[1.0, -1.0]]))
    assert driven["unique"] is True
    assert driven["status"] == "valid"


def test_a_zero_jacobian_leaves_every_direction_free() -> None:
    rank, null = rank_and_null_space(np.zeros((2, 3)))
    assert rank == 0
    assert null.shape == (3, 3)


def test_no_constraints_means_no_rank() -> None:
    rank, null = rank_and_null_space(np.zeros((0, 3)))
    assert rank == 0
    assert null.shape == (3, 3)


def test_a_degenerate_constraint_row_is_singular_not_merely_redundant() -> None:
    """
    A row of exact zeros constrains nothing.

    Counting it as a satisfied constraint would make a degenerate joint look
    like a held one, so the audit must surface it even though the remaining row
    still determines the motion.
    """
    jacobian = np.array([[1.0, 0.0], [0.0, 0.0]])
    audit = audit_boundary(jacobian)
    assert audit["empty_rows"] == 1
    assert audit["status"] == "singular"
    assert "constrain nothing" in audit["reason"]
    with pytest.raises(CharacteristicsError, match="constrain nothing"):
        solve_motion(jacobian, np.array([1.0, 0.0]))


def test_duplicate_rows_are_redundant_but_not_singular() -> None:
    """
    Stating the same constraint twice is not a defect.

    The motion is exactly as determined as with one row, so calling it singular
    would refuse a well-posed boundary.
    """
    jacobian = np.array([[1.0, 0.0], [1.0, 0.0], [0.0, 1.0]])
    audit = audit_boundary(jacobian)
    assert audit["empty_rows"] == 0
    assert audit["redundant_rows"] == 1
    assert audit["unique"] is True
    assert audit["status"] == "valid"
    motion, _ = solve_motion(jacobian, np.array([2.0, 2.0, 3.0]))
    assert np.allclose(motion, [2.0, 3.0])


def test_measurement_response_uses_the_native_jacobian_not_a_difference() -> None:
    """
    A hand-checkable slope: both coordinates are held, so the motion is unique.

    ``J`` is the identity, so driving row ``[1, 0]`` moves q0 by 1 and leaves q1
    at 0; the measurement responds at 3.  The gradient is ``[3, 0]`` rather than
    ``[3, 3]`` because a measurement that also moved with q1 would be
    unidentifiable, not merely larger.
    """
    jacobian = np.eye(2)
    slope, audit = measurement_response(jacobian, np.array([3.0, 0.0]), np.array([1.0, 0.0]))
    assert slope == pytest.approx(3.0)
    assert audit["status"] == "valid"
    assert audit["unique"] is True


def test_a_measurement_reached_only_through_an_unheld_direction_is_refused() -> None:
    """
    Real gradient, real Jacobian, still no answer: the boundary leaves a family.

    Here the constraint only ties q0 to q1, so q0 is not pinned down and the
    measurement gradient ``[1, 0]`` reaches into the free direction.  The
    identifiability guard fires first and names the null space; a least-squares
    solve would instead have returned an arbitrary member of the family.
    """
    jacobian = np.array([[1.0, -1.0]])
    with pytest.raises(CharacteristicsError, match="unconstrained null space"):
        measurement_response(jacobian, np.array([1.0, 0.0]), np.array([1.0]))


def test_measurement_that_only_sees_the_free_direction_is_refused() -> None:
    """
    The measurement is real but the boundary does not pin it down.

    This is the case that a plain least-squares answer would silently fake,
    and it is why the identifiability check precedes the solve.
    """
    jacobian = np.array([[1.0, 1.0]])
    with pytest.raises(CharacteristicsError, match="unconstrained null space"):
        measurement_response(jacobian, np.array([1.0, -1.0]), np.array([1.0, 0.0]))


def test_mismatched_shapes_are_refused_with_the_actual_counts() -> None:
    with pytest.raises(CharacteristicsError, match="components for"):
        is_identifiable(np.array([[1.0, 0.0]]), np.array([1.0, 2.0, 3.0]))
    with pytest.raises(CharacteristicsError, match="targets for"):
        solve_motion(np.array([[1.0, 0.0]]), np.array([1.0, 2.0]))
    with pytest.raises(CharacteristicsError, match="two-dimensional"):
        rank_and_null_space(np.zeros(3))


def test_non_finite_jacobian_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="non-finite"):
        rank_and_null_space(np.array([[1.0, np.nan]]))
