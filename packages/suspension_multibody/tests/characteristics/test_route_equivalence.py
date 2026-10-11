"""
Step 6: the three-route oracle must agree, and must be able to disagree.

SPEC §52 asks for direct/transpose/full-basis/finite-perturbation equivalence, and
§57 makes it an acceptance condition that this holds "within the P0 tolerance,
including non-symmetric and unit/frame" cases.  The subtlety is that an equivalence
test proves nothing unless it *would* fail for a wrong implementation, so half of
what follows sabotages the oracle's inputs and requires the disagreement to appear.

Every tangent fixture is deliberately non-symmetric.  On a symmetric matrix the
transpose route and the direct route coincide trivially, so a transposed solve
would pass; the non-symmetry is what gives the test its teeth, and
`test_the_fixture_is_not_symmetric` asserts it rather than trusting the comment.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import CharacteristicsError
from tests.characteristics.oracle_equivalence import (
    ROUTES,
    compare_routes,
    response_by_finite_perturbation,
    response_operator_by_unit_basis,
)

# A deliberately non-symmetric stiffness-like tangent.  Coupled off-diagonal
# entries that do not mirror each other are what a transposed solve gets wrong.
TANGENT = np.array(
    [
        [4.0, 1.0, 0.5],
        [0.25, 3.0, -1.0],
        [1.5, 0.125, 5.0],
    ]
)
LOAD_MAP = np.array(
    [
        [1.0, 0.0],
        [0.0, 1.0],
        [0.5, -0.5],
    ]
)
DIRECT_TERM = np.zeros((3, 2))
MEASUREMENT = np.eye(3)
# One column per *load mode*: the perturbation route recovers the whole response
# operator, which spans the modes, so a single-column load cannot exercise it.
LOADS = np.array([[2.0, 0.0], [-1.0, 3.0]])


# ---------------------------------------------------------------------------
# The equivalence itself
# ---------------------------------------------------------------------------


def test_the_three_routes_agree_on_a_non_symmetric_system() -> None:
    """The core acceptance condition: three arithmetically distinct routes agree."""
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)
    assert report["agrees"] is True, (
        f"transpose gap {report['transpose_gap']:.3e}, "
        f"perturbation gap {report['perturbation_gap']:.3e}"
    )
    assert report["transpose_gap"] <= 1e-9
    assert report["perturbation_gap"] <= 1e-5


def test_the_fixture_is_not_symmetric() -> None:
    """
    Guard the guard: on a symmetric tangent the transpose route is trivial.

    A transposed solve substituted for the transpose would then agree with the
    direct route, and the whole oracle would be vacuous.  Asserted rather than
    assumed, because a future edit could symmetrise the fixture silently.
    """
    assert not np.array_equal(TANGENT, TANGENT.T)
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)
    assert report["symmetric_tangent"] is False


def test_the_routes_agree_with_several_load_columns() -> None:
    """Equivalence must hold for any load spanning the modes, not one fixed pair."""
    loads = np.array([[2.0, 0.0, -3.0], [-1.0, 4.0, 0.5]])
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, loads, MEASUREMENT)
    # The operator is (measurements, modes) and does not depend on the loads.
    assert report["direct"].shape == (3, 2)
    assert report["response"].shape == (3, 3)
    assert report["agrees"] is True


def test_a_single_load_column_is_refused_by_the_operator_route() -> None:
    """
    One load column cannot recover an operator that spans two modes.

    Refused by name rather than returning a plausible (3, 1) block: the operator's
    width is ``B_w``'s, so a narrower answer would be the wrong object wearing the
    right dtype.
    """
    with pytest.raises(CharacteristicsError, match="one per mode"):
        response_by_finite_perturbation(
            TANGENT, LOAD_MAP, DIRECT_TERM, np.array([[2.0], [-1.0]]), MEASUREMENT
        )


def test_a_nonzero_direct_term_enters_every_route() -> None:
    """
    ``D P`` must appear in all three routes, not just the direct one.

    A route that silently dropped ``D P`` would still agree with itself, and the
    design's ``Y = G X + D P`` explicitly has a feed-through term; a fixture with
    ``D = 0`` cannot detect its loss, so this one is non-zero.
    """
    direct = np.array([[10.0, -5.0], [0.0, 0.0], [1.0, 1.0]])
    report = compare_routes(TANGENT, LOAD_MAP, direct, LOADS, MEASUREMENT)
    assert report["agrees"] is True
    # With D non-zero the answer must differ from the D-free answer.
    plain = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)
    assert not np.allclose(report["direct"], plain["direct"])


def test_the_measurement_map_is_respected() -> None:
    """
    A non-identity ``G`` must project, not merely pass the solved rows through.

    This is the layer the design's ``G`` lives in; using the identity everywhere
    would hide an error in the projection.
    """
    measurement = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    # A reduced measurement set needs its own D: the feed-through term has one row
    # per measurement, so reusing the full-size D here would be a shape error rather
    # than a test of the projection.
    direct = np.zeros((2, 2))
    report = compare_routes(TANGENT, LOAD_MAP, direct, LOADS, measurement)
    assert report["direct"].shape == (2, 2)
    assert report["agrees"] is True


def test_the_finite_perturbation_route_is_reachable_on_its_own() -> None:
    """The third route must be callable directly, not only through the comparison."""
    value = response_by_finite_perturbation(
        TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT
    )
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)
    assert np.allclose(value, report["direct"], rtol=1e-5, atol=1e-8)


def test_the_route_names_are_declared() -> None:
    """
    The four routes the SPEC names are the four the oracle implements.

    SPEC §57 requires "direct mode / transpose / **independent full-unit-basis**
    projection" to be equivalent within the P0 tolerance, so a declaration that
    omitted the unit-basis route would let a literal acceptance condition go
    untested while this assertion still passed.
    """
    assert set(ROUTES) == {
        "direct",
        "transpose",
        "unit_basis",
        "finite_perturbation",
    }


def test_the_unit_basis_route_agrees_with_the_direct_route() -> None:
    """SPEC §57's third named route, exercised on its own."""
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)
    assert report["unit_basis_gap"] <= 1e-9
    assert report["agrees"] is True


def test_the_unit_basis_route_needs_no_load_columns() -> None:
    """
    The unit-basis route is independent of the request's load columns.

    It is the one route that recovers the operator without any load vector, which is
    what makes it a genuinely different check rather than a restatement of route 1.
    """
    direct = np.linalg.solve(TANGENT, LOAD_MAP) + DIRECT_TERM
    unit = response_operator_by_unit_basis(TANGENT, LOAD_MAP, DIRECT_TERM, MEASUREMENT)
    assert np.allclose(unit, direct, rtol=1e-12, atol=1e-12)


def test_the_unit_basis_route_sees_a_transposed_tangent() -> None:
    """A transposed `A` must change this route too, or it is not independent."""
    right = response_operator_by_unit_basis(TANGENT, LOAD_MAP, DIRECT_TERM, MEASUREMENT)
    wrong = response_operator_by_unit_basis(TANGENT.T, LOAD_MAP, DIRECT_TERM, MEASUREMENT)
    scale = max(1.0, float(np.max(np.abs(right))))
    assert float(np.max(np.abs(right - wrong))) / scale > 1e-9


def test_a_mode_with_no_coupling_is_refused_by_the_unit_basis_route() -> None:
    """An uncoupled mode is unidentifiable; the refusal must name it."""
    load_map = LOAD_MAP.copy()
    load_map[:, 1] = 0.0
    with pytest.raises(CharacteristicsError, match="not identifiable"):
        response_operator_by_unit_basis(TANGENT, load_map, DIRECT_TERM, MEASUREMENT)


def test_a_mode_with_no_coupling_is_refused_by_the_perturbation_route() -> None:
    """
    The same unidentifiability in the perturbation route.

    This is the branch the sabotage harness found unexercised.  It matters because the
    failure mode is not a plausible zero: the quotient returns the *untouched* part of
    the response divided by ``h``, so the caller would receive a large number that
    appears to converge as ``h`` shrinks and has nothing to do with the mode asked
    about.  Refusing by name is the only honest outcome.
    """
    load_map = LOAD_MAP.copy()
    load_map[:, 1] = 0.0
    with pytest.raises(CharacteristicsError, match="not identifiable"):
        response_by_finite_perturbation(
            TANGENT, load_map, DIRECT_TERM, LOADS, MEASUREMENT
        )


def test_the_perturbation_route_bumps_the_whole_mode_row() -> None:
    """
    The bump must cover every load column, not just the first.

    Bumping only column 0 leaves the other columns unperturbed, and the error is
    proportional to how much the load columns differ from each other -- so on a
    load matrix with equal columns the defect is invisible while on a
    badly-scaled one it is not.  The assertion is therefore written as the property
    that must hold regardless of the probe, and the probe set below is chosen so the
    defect cannot hide: a partial-row bump leaks the load weights into the answer.
    """
    loads = np.array([[2.0, 5.0], [-1.0, 3.0]])
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, loads, MEASUREMENT)
    assert report["agrees"] is True

    # Independently: the whole-row bump recovers each mode column exactly.
    direct = np.linalg.solve(TANGENT, LOAD_MAP) + DIRECT_TERM
    perturbed = response_by_finite_perturbation(
        TANGENT, LOAD_MAP, DIRECT_TERM, loads, MEASUREMENT
    )
    assert np.allclose(perturbed, direct, rtol=1e-5, atol=1e-8)


def test_the_perturbation_route_is_accurate_across_load_magnitudes() -> None:
    """
    The quotient must stay accurate from tiny to enormous loads.

    Regression: with an absolute ``h = 1e-7`` the route degraded to a 1e-2 relative
    error at ``loads = 1e8``, because the addend was absorbed into the load and the
    quotient then divided by a step that was never added.  Shrinking ``h`` made it
    *worse*.  The fix sizes the step relative to the loads, so the accuracy must now be
    flat in the load magnitude -- which is what this test pins, rather than merely
    checking one comfortable scale.
    """
    exact = np.linalg.solve(TANGENT, LOAD_MAP) + DIRECT_TERM
    worst = 0.0
    for scale in (1e-8, 1.0, 1e3, 1e4, 1e6, 1e8, 1e12):
        loads = np.array([[scale, 0.0], [-1.0, 3.0]])
        got = response_by_finite_perturbation(
            TANGENT, LOAD_MAP, DIRECT_TERM, loads, MEASUREMENT
        )
        gap = float(np.max(np.abs(got - exact))) / max(
            1.0, float(np.max(np.abs(exact)))
        )
        worst = max(worst, gap)
    assert worst <= 1e-8, (
        f"the operator lost accuracy at some load magnitude (worst gap {worst:.3e}); "
        "the perturbation step is probably absolute rather than scaled"
    )


def test_a_small_mode_inside_a_large_problem_stays_accurate() -> None:
    """
    A mode's step must be scaled by the whole problem, not by its own row.

    Regression: sizing the step from the mode's own load row left the modest mode-1 row
    inside a problem dominated by ``1e8`` perturbed by a step that vanished in the
    subtraction -- column 0 was exact while column 1 carried a 6e-4 error.  The failure
    is invisible unless the two modes differ enormously in magnitude, which is why the
    probe set for this test is deliberately lopsided.
    """
    exact = np.linalg.solve(TANGENT, LOAD_MAP) + DIRECT_TERM
    loads = np.array([[1e8, 0.0], [-1.0, 3.0]])
    got = response_by_finite_perturbation(
        TANGENT, LOAD_MAP, DIRECT_TERM, loads, MEASUREMENT
    )
    for mode in range(exact.shape[1]):
        gap = float(np.max(np.abs(got[:, mode] - exact[:, mode]))) / max(
            1.0, float(np.max(np.abs(exact[:, mode])))
        )
        assert gap <= 1e-8, (
            f"mode {mode} lost accuracy inside a large problem (gap {gap:.3e})"
        )


def test_the_perturbation_route_is_independent_of_the_load_weights() -> None:
    """
    The operator must not depend on the loads it was probed with.

    A partial-row bump leaks the probing loads into the answer, so this invariance is
    the property that catches it.  One probe is deliberately badly scaled
    (``1e3`` against ``1e-3``): on well-scaled probes the leaked error sits at the
    1e-10 level, indistinguishable from rounding, so a probe set of only tidy loads
    would report agreement that the code has not earned.
    """
    probes = [
        np.array([[1.0, 0.0], [0.0, 1.0]]),
        np.array([[2.0, 5.0], [-1.0, 3.0]]),
        np.array([[1e3, -7.0], [4.0, 1e-3]]),
        np.array([[1.0, 1.0], [1.0, 1.0]]),  # rank deficient
    ]
    results = [
        response_by_finite_perturbation(
            TANGENT, LOAD_MAP, DIRECT_TERM, probe, MEASUREMENT
        )
        for probe in probes
    ]
    reference = results[0]
    for probe, value in zip(probes[1:], results[1:], strict=True):
        scale = max(1.0, float(np.max(np.abs(reference))))
        gap = float(np.max(np.abs(value - reference))) / scale
        assert gap <= 1e-6, (
            f"the operator changed with the probing loads {probe.ravel().tolist()} "
            f"(gap {gap:.3e}); a partial-row bump leaks the probe into the answer"
        )


# ---------------------------------------------------------------------------
# The oracle must be able to fail -- otherwise equivalence proves nothing
# ---------------------------------------------------------------------------


def test_substituting_the_tangent_for_its_transpose_is_detected() -> None:
    """
    The defect the non-symmetric fixture exists to catch.

    Design §40 requires the transpose path to solve ``A^T`` really, and warns that
    a same-size solve of ``A`` must not be passed off as one.  Here the wrong
    operator is formed deliberately: ``A`` in the place where ``A^T`` belongs.  On
    this fixture the two must differ, which is what gives the transpose route its
    power to detect the substitution.
    """
    measurement = np.eye(3)
    right = measurement @ np.linalg.solve(TANGENT.T, LOAD_MAP) + DIRECT_TERM
    wrong = measurement @ np.linalg.solve(TANGENT, LOAD_MAP) + DIRECT_TERM

    scale = max(1.0, float(np.max(np.abs(right))), float(np.max(np.abs(wrong))))
    gap = float(np.max(np.abs(right - wrong))) / scale
    assert gap > 1e-9, (
        "substituting A for A^T must change the operator; if it does not, the "
        "fixture is symmetric and the transpose route is untested"
    )


def test_a_wrong_load_map_is_detected_by_the_perturbation_route() -> None:
    """
    Why route 3 is needed at all: routes 1 and 2 share ``B_w``.

    Perturbing the *state* instead of the load, or dropping the coupling column,
    leaves both linear routes self-consistent.  The difference quotient does not.
    """
    broken = LOAD_MAP.copy()
    broken[2, 1] = 0.0  # drop a coupling entry both linear routes would share
    good = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)
    bad = response_by_finite_perturbation(
        TANGENT, broken, DIRECT_TERM, LOADS, MEASUREMENT
    )
    scale = max(1.0, float(np.max(np.abs(good["direct"]))))
    gap = float(np.max(np.abs(good["direct"] - bad))) / scale
    assert gap > 1e-6, "the perturbation route must see a changed load map"


def test_a_symmetric_tangent_makes_the_transpose_route_vacuous() -> None:
    """
    Recorded so the non-symmetry requirement is not mistaken for taste.

    On this symmetric fixture the transpose route agrees trivially -- which is
    exactly why it cannot serve as an oracle for the transpose path.
    """
    symmetric = TANGENT + TANGENT.T
    report = compare_routes(symmetric, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)
    assert report["symmetric_tangent"] is True
    assert report["agrees"] is True


# ---------------------------------------------------------------------------
# Refusals: a malformed problem is named, not silently mis-answered
# ---------------------------------------------------------------------------


def test_a_non_square_tangent_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="square"):
        compare_routes(np.ones((2, 3)), LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)


def test_a_mismatched_load_map_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="tangent order"):
        compare_routes(TANGENT, np.ones((2, 2)), DIRECT_TERM, LOADS, MEASUREMENT)


def test_a_mismatched_load_count_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="loads has"):
        compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, np.array([[1.0]]), MEASUREMENT)


def test_a_mismatched_direct_term_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="one per load mode"):
        compare_routes(TANGENT, LOAD_MAP, np.ones((3, 3)), LOADS, MEASUREMENT)


def test_a_mismatched_measurement_map_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="measurement_map has"):
        compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, np.ones((2, 2)))


def test_a_non_finite_input_is_refused() -> None:
    bad = TANGENT.copy()
    bad[0, 0] = np.nan
    with pytest.raises(CharacteristicsError, match="non-finite"):
        compare_routes(bad, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)


def test_a_non_positive_perturbation_step_is_refused() -> None:
    """A zero or negative step is not a difference quotient."""
    for step in (0.0, -1e-7, float("nan")):
        with pytest.raises(CharacteristicsError, match="step must be positive"):
            response_by_finite_perturbation(
                TANGENT, LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT, step=step
            )


def test_a_vector_tangent_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="must be a matrix"):
        compare_routes(np.array([1.0, 2.0]), LOAD_MAP, DIRECT_TERM, LOADS, MEASUREMENT)


def test_a_direct_term_with_too_few_rows_is_refused_not_broadcast() -> None:
    """
    Regression: ``D`` with fewer rows than ``G`` used to broadcast silently.

    ``D @ P`` with a 1-row ``D`` and a 3-row measurement broadcast the single row
    across all three measurements, returning a well-formed operator whose values were
    wrong by exactly that row -- near the right magnitude, with no error raised
    anywhere.  For a layer whose whole job is to report stiffness this is the worst
    possible failure, so it is refused by name.
    """
    with pytest.raises(CharacteristicsError, match="both map into measurement space"):
        response_by_finite_perturbation(
            TANGENT, LOAD_MAP, np.array([[10.0, -5.0]]), LOADS, MEASUREMENT
        )
    with pytest.raises(CharacteristicsError, match="both map into measurement space"):
        compare_routes(
            TANGENT, LOAD_MAP, np.array([[10.0, -5.0]]), LOADS, MEASUREMENT
        )


def test_a_direct_term_with_too_many_rows_is_refused() -> None:
    """The other direction must also be a named refusal, not a bare ValueError."""
    with pytest.raises(CharacteristicsError, match="both map into measurement space"):
        compare_routes(TANGENT, LOAD_MAP, np.ones((4, 2)), LOADS, MEASUREMENT)


def test_an_empty_measurement_set_is_refused() -> None:
    """
    No measurements means every route agrees for free, proving nothing.

    ``gap`` returns 0.0 for an empty array, so ``agrees`` would be True and the
    comparison would be vacuously satisfied -- a test that passes hardest when it
    checks the least.
    """
    with pytest.raises(CharacteristicsError, match="measurement set is empty"):
        compare_routes(TANGENT, LOAD_MAP, np.zeros((0, 2)), LOADS, np.zeros((0, 3)))


def test_a_rank_deficient_load_matrix_is_accepted() -> None:
    """
    The perturbation route imposes no rank condition on the loads.

    Bumping whole mode row ``m`` changes every column by ``h * B_w[:, m]``, so the
    quotient is ``outer(C[:, m], 1)`` and any column is a valid representative.
    Duplicated load columns must therefore still work; asserting otherwise would
    document a condition the mathematics does not have.
    """
    duplicated = np.array([[2.0, 2.0], [-1.0, -1.0]])
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, duplicated, MEASUREMENT)
    assert report["agrees"] is True


def test_a_rectangular_load_matrix_is_accepted() -> None:
    """More load columns than modes is fine; the requirement is one column per mode."""
    loads = np.array([[2.0, 0.0, -3.0], [-1.0, 4.0, 0.5]])
    report = compare_routes(TANGENT, LOAD_MAP, DIRECT_TERM, loads, MEASUREMENT)
    assert report["agrees"] is True
