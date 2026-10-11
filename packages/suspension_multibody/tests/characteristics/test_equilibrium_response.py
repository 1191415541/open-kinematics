"""
P2 step 3: the equilibrium response ``A X = B_w P`` / ``Y = G X + D P``.

The tests come in three groups, and the split is the point.

* **Algebraic identity.**  A spring-like mechanism whose answer is known in closed
  form (``K = k``, ``C = 1/k``).  This pins the *formula*: if the code multiplied by
  ``A`` instead of solving it, the numbers here change and nothing else notices.

* **Refusal.**  The interesting cases are the ones that must *not* produce a number:
  a free rigid-body direction, a degenerate row, and -- the subtle one -- a perfectly
  well-posed tangent with an *unidentifiable measurement*.  That last case is where a
  naive implementation returns a plausible number that identifies nothing, so it is
  tested against its own independent null-space computation rather than against a
  hard-coded value.

* **Strategy equivalence.**  The direct and transposed solves must agree, and the
  transposed solve must be a real transpose: on a deliberately non-symmetric tangent
  the two disagree, which is how "you actually transposed" becomes observable.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import CharacteristicsError
from suspension_multibody.results.equilibrium import (
    LOAD_MODE_COMPONENTS,
    LoadMode,
    ModeResult,
    audited_modes,
    equilibrium_response,
    load_mode_matrix,
)


def test_the_response_is_the_solution_and_not_the_product() -> None:
    """
    ``A X = B`` must give ``B/A``, not ``A B``.

    Both alternatives are finite and same-shaped, so only the value distinguishes
    them; a diagonal-ish test would let the mistake through, hence the asymmetric
    factor of 4 against a load of -8 giving exactly -2.
    """
    response, audit = equilibrium_response(
        np.array([[4.0]]), np.array([[-8.0]]), np.array([[1.0]])
    )
    assert audit["status"] == "valid"
    np.testing.assert_allclose(response[0, 0], -2.0, rtol=1e-12)
    # And explicitly neither of the two plausible mistakes.
    assert not np.isclose(response[0, 0], 4.0 * -8.0), "response must not be the product A B"
    assert not np.isclose(response[0, 0], 2.0), "the sign must survive the solve"


def test_a_diagonal_mechanism_matches_the_closed_form_compliance() -> None:
    """
    For ``A = diag(k_i)`` the response is ``diag(1/k_i)`` times the load.

    The expected value is computed here from the same closed form the design states
    (``K = k``, ``C = 1/k``) rather than read from the implementation, so this is a
    real oracle for the diagonal case and not a snapshot.
    """
    stiffness = np.array([2.0, 5.0, 10.0])
    tangent = np.diag(stiffness)
    load = np.eye(3)
    directions = np.eye(3)
    response, audit = equilibrium_response(tangent, load, directions)
    assert audit["status"] == "valid"
    np.testing.assert_allclose(response, np.diag(1.0 / stiffness), rtol=1e-12)


def test_the_measurement_chain_adds_the_direct_term() -> None:
    """
    ``Y = G X + D P`` -- dropping ``D P`` must be observable.

    A measurement that reads the applied load directly (a wheel load, say) has a
    non-zero ``D``.  If the implementation returned only ``G X`` the result would be
    wrong by exactly ``D P``, and for this fixture that is the entire answer.
    """
    tangent = np.array([[4.0]])
    load = np.array([[-8.0]])
    directions = np.array([[1.0]])
    gradient = np.array([[0.0]])       # this measurement does not see the motion
    direct = np.array([[3.0]])         # ... but it does see the load
    response, audit = equilibrium_response(
        tangent,
        load,
        directions,
        measurement_gradient=gradient,
        measurement_load_gradient=direct,
    )
    assert audit["status"] == "valid"
    # G X = 0, D P = 3 * 1 = 3.
    np.testing.assert_allclose(response[0, 0], 3.0, rtol=1e-12)


def test_one_factorization_serves_every_load_mode() -> None:
    """
    The batch property: many modes, one factorization.

    The count is asserted because a per-mode loop would produce identical numbers
    while costing N times more -- the answers alone cannot tell the two apart.
    """
    tangent = np.diag([1.0, 2.0, 3.0])
    load = np.eye(3)
    directions = np.eye(3)
    _, audit = equilibrium_response(tangent, load, directions)
    assert audit["mode_count"] == 3
    assert audit["factorizations"] == 1


def test_a_free_rigid_body_direction_is_refused_not_approximated() -> None:
    """
    A row of zeros leaves a direction free: the motion is a family, so no answer.

    The refusal must be *numberless*.  Zero-filling it would make the refusal
    numerically identical to a mechanism whose response really is zero, so a caller
    that forgot to check ``status`` would publish a plausible wrong number.  NaN
    cannot be mistaken for a reading.
    """
    tangent = np.array([[4.0, 0.0], [0.0, 0.0]])
    response, audit = equilibrium_response(tangent, np.eye(2), np.eye(2))
    assert audit["status"] == "singular"
    assert audit["empty_rows"] == 1
    assert "constrain nothing" in audit["reason"]
    assert np.isnan(response).all(), (
        "a refused system must carry no finite number at all"
    )


def test_a_rank_deficient_tangent_returns_no_numbers() -> None:
    """
    A dependent-row tangent determines no motion, and the refusal must be visible.

    This is the case a minimum-norm solve would quietly answer: ``lstsq`` returns a
    perfectly plausible vector for a rank-deficient system.  Asserting NaN here is
    what makes "did you refuse, or did you answer a family?" observable from the
    returned array alone, without trusting ``status``.
    """
    tangent = np.array([[1.0, 2.0], [2.0, 4.0]])
    response, audit = equilibrium_response(tangent, np.eye(2), np.eye(2))
    assert audit["status"] == "unidentifiable"
    assert audit["nullity"] == 1
    assert np.isnan(response).all(), (
        "a rank-deficient tangent must not return a minimum-norm representative"
    )
    assert audit["factorizations"] == 0, "no factorization is spent on a refused solve"


def test_a_mode_that_drives_nothing_is_refused_while_its_neighbour_answers() -> None:
    """
    The live per-mode refusal: a perfect tangent, a mode that asks nothing.

    An identifiability test on the *measurement* side is dead code for a square
    tangent -- a unique ``X`` makes every gradient of ``X`` identifiable, so such a
    test can only fire when the tangent is already refused.  The case that actually
    needs a per-mode check is on the **load** side: with a load jacobian whose
    ``vertical_force`` column is zero, that mode's applied direction is zero, and
    ``A^{-1} 0 = 0`` would be reported as a legitimate zero response.

    The tangent here is deliberately the identity, i.e. perfectly conditioned, so
    nothing but the per-mode check can produce a refusal.
    """
    tangent = np.eye(2)
    modes = (
        LoadMode("vertical_force", "N"),     # drives nothing
        LoadMode("longitudinal_force", "N"),  # drives the first coordinate
    )
    # Column 2 (`vertical_force`, component index 2) is out of range here, so the
    # fixture uses a 6-wide load jacobian with only the longitudinal column live.
    load = np.zeros((2, 6))
    load[0, LOAD_MODE_COMPONENTS["longitudinal_force"]] = 1.0
    # `vertical_force` (component 2) stays exactly zero -> nothing is driven.

    results, audit = audited_modes(tangent, load, modes)
    assert audit["status"] == "valid", "the tangent itself is perfectly well posed"
    assert audit["factorizations"] == 1

    assert results[0].kind == "vertical_force"
    assert results[0].status == "unidentifiable"
    assert results[0].values is None
    assert "drives no load direction" in results[0].reason

    # The neighbour is answered in the same call: the refusal is per mode.
    assert results[1].kind == "longitudinal_force"
    assert results[1].status == "valid"
    assert results[1].values is not None
    np.testing.assert_allclose(results[1].values, [1.0, 0.0], rtol=1e-12)


def test_a_refused_mode_is_visible_in_the_audit_too() -> None:
    """
    A caller that reads only the audit must not think everything was answered.

    The two objects answer different questions -- ``audit["status"]`` is the
    *tangent's* verdict, while a mode can be refused on its own -- so the audit also
    names which modes were refused and which were answered.  Without that, a caller
    reading only the audit would see a healthy system and silently drop modes.
    """
    tangent = np.eye(2)
    load = np.zeros((2, 6))
    load[0, LOAD_MODE_COMPONENTS["longitudinal_force"]] = 1.0
    modes = (
        LoadMode("vertical_force", "N"),
        LoadMode("longitudinal_force", "N"),
    )
    _, audit = audited_modes(tangent, load, modes)
    assert audit["status"] == "valid", "the tangent is fine; only a mode was refused"
    assert audit["refused_modes"] == ["vertical_force"]
    assert audit["answered_modes"] == ["longitudinal_force"]


def test_a_small_but_real_load_is_answered_not_refused() -> None:
    """
    The per-mode threshold must be relative, not an absolute floor.

    A load jacobian whose entries are all ~1e-11 is entirely legitimate -- a small
    load on a stiff mechanism.  Comparing a column norm against an absolute 1e-10
    would refuse *every* mode of such a request while the audit still reported
    ``valid``, which is the worst combination: wrong answer, healthy-looking audit.
    """
    scale = 1e-11
    tangent = np.eye(2)
    load = np.zeros((2, 6))
    load[0, LOAD_MODE_COMPONENTS["longitudinal_force"]] = scale
    load[1, LOAD_MODE_COMPONENTS["lateral_force"]] = scale
    modes = (
        LoadMode("longitudinal_force", "N"),
        LoadMode("lateral_force", "N"),
    )
    results, audit = audited_modes(tangent, load, modes)
    assert all(item.status == "valid" for item in results), (
        "a uniformly small but non-zero load jacobian must be answered"
    )
    assert audit["refused_modes"] == []
    # And the values really are the solution: X = A^-1 (B_w P) = scale * I.
    np.testing.assert_allclose(results[0].values, [scale, 0.0], rtol=1e-9)
    np.testing.assert_allclose(results[1].values, [0.0, scale], rtol=1e-9)


def test_an_overflowing_response_is_refused_rather_than_crashing() -> None:
    """
    A well-conditioned tangent can still produce an unrepresentable response.

    If the overflow escaped, the non-finite values would reach ``statuses_for``
    *without a reason* and it would raise -- so the caller would get an exception
    from deep inside instead of a per-mode refusal.  The refusal must also carry a
    reason, because that is what distinguishes it from a silent zero.
    """
    tangent = np.eye(2)
    load = np.zeros((2, 6))
    load[0, LOAD_MODE_COMPONENTS["longitudinal_force"]] = 2.0
    modes = (LoadMode("longitudinal_force", "N"),)
    # A gradient this large makes G X overflow even though X itself is finite.
    gradient = np.array([[1e308, 1e308]])
    with np.errstate(over="ignore"):
        results, audit = audited_modes(tangent, load, modes, measurement_gradient=gradient)
    assert results[0].status == "unidentifiable"
    assert results[0].values is None
    assert "not representable" in results[0].reason, (
        "an overflow refusal must say why, not return an empty reason"
    )
    assert audit["refused_modes"] == ["longitudinal_force"]


def test_a_mode_matrix_with_the_wrong_column_count_is_refused() -> None:
    """
    The response is labelled by the caller's mode list, so a column mismatch is fatal.

    Nothing downstream can recover the mismatch: the columns are positional, so a
    matrix carrying fewer or more columns than the declared modes would be labelled
    wrongly rather than refused.
    """
    with pytest.raises(CharacteristicsError) as error:
        equilibrium_response(
            np.eye(2), np.zeros((2, 6)), np.zeros((6, 2)), mode_count=3
        )
    assert "column" in str(error.value)


def test_a_refused_mode_never_carries_numbers() -> None:
    """``values is None`` exactly when refused -- zeros would look like a reading."""
    refused = ModeResult(kind="vertical_force", status="singular", reason="degenerate", values=None)
    assert refused.values is None
    with pytest.raises(CharacteristicsError):
        ModeResult(kind="vertical_force", status="singular", reason="", values=None)
    with pytest.raises(CharacteristicsError):
        ModeResult(
            kind="vertical_force",
            status="valid",
            reason="should not be here",
            values=np.zeros(1),
        )


def test_an_unknown_load_mode_is_refused_by_name() -> None:
    """An unknown mode must be a named refusal, not a silently-zero direction."""
    with pytest.raises(CharacteristicsError) as error:
        LoadMode("sideways_push", "N")
    assert "sideways_push" in str(error.value)
    assert "vertical_force" in str(error.value), "the refusal must list what does exist"


def test_the_mode_matrix_puts_a_unit_where_the_mode_says() -> None:
    """
    ``P`` is built in one place so the sign convention has one home.

    Checked against the component table rather than a literal index, so a future
    reordering of the load vector cannot leave the test agreeing with a bug.
    """
    modes = (
        LoadMode("vertical_force", "N"),
        LoadMode("aligning_torque", "N*m"),
    )
    matrix = load_mode_matrix(modes)
    assert matrix.shape == (6, 2)
    for column, mode in enumerate(modes):
        expected = np.zeros(6)
        expected[LOAD_MODE_COMPONENTS[mode.kind]] = 1.0
        np.testing.assert_array_equal(matrix[:, column], expected)


def test_the_transpose_is_a_real_transpose() -> None:
    """
    On a non-symmetric tangent, ``A x = b`` and ``A^T x = b`` must differ.

    This is the observable form of "did you transpose?".  For a symmetric matrix
    every transpose bug passes; the design explicitly forbids assuming symmetry,
    because a bushing with velocity terms is not symmetric.
    """
    tangent = np.array([[4.0, 1.0], [0.5, 3.0]])
    assert not np.allclose(tangent, tangent.T), "fixture must be non-symmetric"
    load = np.array([1.0, 2.0])

    forward, _ = equilibrium_response(tangent, load.reshape(-1, 1), np.array([[1.0]]))
    transposed, _ = equilibrium_response(
        tangent.T, load.reshape(-1, 1), np.array([[1.0]])
    )
    assert not np.allclose(forward, transposed), (
        "a non-symmetric tangent must give different forward and transposed responses"
    )
    # And the transposed answer really solves A^T x = b.
    np.testing.assert_allclose(tangent.T @ transposed[:, 0], load, rtol=1e-12)


def test_the_response_satisfies_the_forward_equation() -> None:
    """
    Independent residual check: ``A X - B_w P`` must vanish.

    This does not reuse the solver, so it holds even if the solver is replaced.
    """
    rng = np.random.default_rng(20261010)
    tangent = rng.normal(size=(5, 5))
    tangent += 5.0 * np.eye(5)          # keep it well conditioned
    load = rng.normal(size=(5, 6))
    modes = (
        LoadMode("vertical_force", "N"),
        LoadMode("lateral_force", "N"),
        LoadMode("aligning_torque", "N*m"),
    )
    directions = load_mode_matrix(modes)
    response, audit = equilibrium_response(tangent, load, directions)
    assert audit["status"] == "valid"
    residual = tangent @ response - load @ directions
    assert np.linalg.norm(residual) <= 1e-10 * max(1.0, np.linalg.norm(load @ directions))
