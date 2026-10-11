"""
Independent oracle: three routes to the same equilibrium response must agree.

A single arithmetic route -- however carefully written -- cannot distinguish "the
physics is right" from "the code does what the code does", because a sign or index
error is perfectly self-consistent.  So the tests here compute each quantity three
independent ways and require agreement inside the P0 tolerance:

1. **Direct** -- ``A X = B_w P``, then ``Y = G X + D P``.  Reproduces route 1 *by
   construction*: it re-solves with a fresh right-hand side, so it checks the
   caller's assembly rather than restating it.
2. **Transpose** -- ``A^T Z = G^T``, then ``Y = Z^T B_w P + D P``.  Design §6.6
   states the two are algebraically identical, so this is a genuine oracle: it uses
   ``A^T`` and a different factorization.  Substituting ``A`` for ``A^T`` gives a
   different answer on any non-symmetric matrix, which is why every fixture here is
   deliberately non-symmetric -- a symmetric one would let that error pass.
3. **Finite perturbation** -- bump the load column by ``h``, re-solve, divide by
   ``h``.  This never forms ``A^{-1} B_w`` explicitly, so it catches a wrong
   ``B_w`` or a transposed ``A`` that routes 1 and 2 would agree on, since they
   share the same ``B_w``.

Route 3 perturbs the right-hand side of an already-assembled *linear* system, not
the solver's residual.  That is deliberate: staying inside linear algebra keeps it
free of convergence and step-size effects, so a disagreement is a matrix error and
not a numerical artefact.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from suspension_multibody.characteristics.values import CharacteristicsError

__all__ = [
    "ROUTES",
    "compare_routes",
    "response_by_finite_perturbation",
    "response_operator_by_direct_route",
    "response_operator_by_unit_basis",
]

ROUTES = ("direct", "transpose", "unit_basis", "finite_perturbation")


def _matrix(value: Any, *, name: str) -> np.ndarray:
    """Return ``value`` as a finite 2-D float array, or refuse by name."""
    array = np.asarray(value, dtype=float)
    if array.ndim != 2:
        raise CharacteristicsError(f"{name} must be a matrix, got shape {array.shape}")
    if not np.all(np.isfinite(array)):
        raise CharacteristicsError(f"{name} contains a non-finite entry")
    return array


def _columns(loads: Any) -> np.ndarray:
    """Return the load matrix as columns, accepting a single vector."""
    array = np.asarray(loads, dtype=float)
    if array.ndim == 1:
        array = array.reshape(-1, 1)
    elif array.ndim != 2:
        raise CharacteristicsError(f"loads must be a vector or matrix, got {array.shape}")
    # Checked here rather than only in `_matrix`: a NaN load would otherwise propagate
    # into every route and surface as a silent NaN answer, which is exactly the kind of
    # un-actionable output the rest of this module refuses by name.
    if not np.all(np.isfinite(array)):
        raise CharacteristicsError("loads contains a non-finite entry")
    return array


def _checked(
    tangent: Any, load_jacobian: Any, direct_term: Any, loads: Any
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Validate one problem statement once, so every route judges the same shapes.

    Checking here rather than inside each route is what makes a shape error a
    single refusal instead of three routes disagreeing for the wrong reason.
    """
    matrix = _matrix(tangent, name="tangent")
    load_map = _matrix(load_jacobian, name="load_jacobian")
    direct = _matrix(direct_term, name="direct_term")
    columns = _columns(loads)

    if matrix.shape[0] != matrix.shape[1]:
        raise CharacteristicsError(
            f"the tangent must be square, got {matrix.shape}"
        )
    if load_map.shape[0] != matrix.shape[0]:
        raise CharacteristicsError(
            f"load_jacobian has {load_map.shape[0]} rows but the tangent order is "
            f"{matrix.shape[0]}"
        )
    if columns.shape[0] != load_map.shape[1]:
        raise CharacteristicsError(
            f"loads has {columns.shape[0]} rows but the load map expects "
            f"{load_map.shape[1]}"
        )
    if direct.shape[1] != load_map.shape[1]:
        raise CharacteristicsError(
            f"direct_term must have {load_map.shape[1]} columns (one per load mode), "
            f"got {direct.shape}"
        )
    return matrix, load_map, direct, columns


def _measurement(map_value: Any, *, order: int, direct: np.ndarray) -> np.ndarray:
    """
    Validate the measurement map against the tangent order *and* against ``D``.

    The second check is the one that matters and the one that is easy to omit: ``D``
    maps load modes straight to measurements, so ``D`` and ``G`` must share a row
    count.  Without it, ``D @ P`` with too few rows broadcasts the single row across
    every measurement and returns a well-formed, wrong operator -- a number near the
    right magnitude with no error anywhere, which is precisely the failure a metric
    layer must never produce.
    """
    measurement = _matrix(map_value, name="measurement_map")
    if measurement.shape[1] != order:
        raise CharacteristicsError(
            f"measurement_map has {measurement.shape[1]} columns but the tangent "
            f"order is {order}"
        )
    if measurement.shape[0] != direct.shape[0]:
        raise CharacteristicsError(
            f"measurement_map has {measurement.shape[0]} rows but direct_term has "
            f"{direct.shape[0]}; both map into measurement space and must agree"
        )
    if measurement.shape[0] == 0:
        raise CharacteristicsError(
            "the measurement set is empty, so every route trivially agrees and the "
            "comparison proves nothing"
        )
    return measurement


def _direct_answer(
    matrix: np.ndarray,
    load_map: np.ndarray,
    direct: np.ndarray,
    columns: np.ndarray,
    measurement: np.ndarray,
) -> np.ndarray:
    """Route 1: ``Y = G A^{-1} B_w P + D P``."""
    solution = np.linalg.solve(matrix, load_map @ columns)
    return measurement @ solution + direct @ columns


def response_by_finite_perturbation(
    tangent: Any,
    load_jacobian: Any,
    direct_term: Any,
    loads: Any,
    measurement_map: Any,
    *,
    step: float = 1e-7,
) -> np.ndarray:
    """
    Route 3: recover the **response operator** ``C = G A^{-1} B_w + D`` by differencing.

    ``(Y(P + h e_j) - Y(P)) / h`` approaches column ``j`` of ``C``.  Note carefully
    that this yields ``C`` -- a *derivative* with respect to the load -- and not the
    response ``Y`` itself.  The distinction is the whole point of keeping this route
    separate: ``C`` is what a stiffness/compliance metric is, and it is reachable
    without ever forming ``A^{-1} B_w`` explicitly, so a wrong ``B_w`` or a
    transposed ``A`` shows up here even when the two linear routes agree with each
    other (they share ``B_w``).

    Because the bump is applied to the *load* and not to a state, the quotient
    converges to the exact linear answer as ``h -> 0``.  That convergence is one-sided
    in practice, though, and the step is therefore chosen **relative to the load
    magnitude** rather than taken as an absolute constant:

    * too small, and ``loads[mode] + h`` rounds back to ``loads[mode]`` (absorption),
      so the injected step is not the step divided by -- measured at ``loads = 1e8``
      the actual delta is ``1.043e-07``, not ``1e-07``, and the reported operator is
      wrong by ~1e-2 relative;
    * too large, and the quotient stops approximating the derivative.

    An absolute ``h = 1e-7`` fails the first way as soon as the loads reach ~1e4, and
    *shrinking* it makes things worse rather than better.  Sizing by magnitude keeps
    the step representable at every scale, which is what makes this route a usable
    oracle instead of one that silently degrades with the units of its input.
    """
    matrix, load_map, direct, columns = _checked(
        tangent, load_jacobian, direct_term, loads
    )
    measurement = _measurement(measurement_map, order=matrix.shape[0], direct=direct)
    if not np.isfinite(step) or step <= 0.0:
        raise CharacteristicsError("the perturbation step must be positive and finite")

    base = _direct_answer(matrix, load_map, direct, columns, measurement)
    # The quotient has one column per *load mode*, not per load column in this
    # request: `C` maps modes to measurements, so its width is `B_w`'s width even
    # when the caller asks about a single mode.  Sizing from `base` would silently
    # return a (measurements, 1) block that looks plausible and is the wrong shape.
    #
    # The requirement is only that there be at least one load column per mode.  It is
    # *not* a rank condition on the loads: bumping whole mode row ``m`` changes every
    # column by ``h * B_w[:, m]``, so the quotient is ``outer(C[:, m], 1)`` -- every
    # column carries the same derivative and any one of them is a valid
    # representative.  A rank-deficient or duplicated load matrix is therefore fine,
    # and saying otherwise would send a later reader hunting for a condition the
    # mathematics does not impose.
    if columns.shape[1] < load_map.shape[1]:
        raise CharacteristicsError(
            f"recovering the full response operator needs at least "
            f"{load_map.shape[1]} load columns (one per mode), got {columns.shape[1]}"
        )
    result = np.zeros((measurement.shape[0], load_map.shape[1]))
    for mode in range(load_map.shape[1]):
        if not np.any(load_map[:, mode]):
            # A mode whose `B_w` column is entirely zero carries no information about
            # that load direction: the difference quotient does not vanish, it returns
            # whatever the *untouched* part of the response happens to be, divided by
            # `h`.  So the failure mode is a large garbage number rather than a
            # plausible zero -- and it looks convergent as `h` shrinks.  Refused here
            # so the caller learns the mode is unidentifiable instead of reading a
            # number that has nothing to do with it.
            raise CharacteristicsError(
                f"load mode {mode} has no coupling into the system, so its response "
                "is not identifiable from that load direction"
            )
        # Perturb mode `mode` across *every* load column.  Bumping a single column
        # would move only that column's answer, and the resulting (measurements,
        # columns) difference has no column that isolates the mode -- the operator's
        # mode axis is `B_w`'s, not the request's.
        bumped = columns.copy()
        # Scale the step to the perturbation *problem's* magnitude, then verify the
        # step actually landed.  Sizing by this mode's own row is not enough: `Y` is
        # dominated by the largest load anywhere in the matrix, so a mode whose loads
        # are small is perturbed by a step that vanishes in the subtraction -- measured
        # at loads [[1e8, 0], [-1, 3]] the modest mode-1 row produced a 6e-4 relative
        # error in column 1 while column 0 was exact.
        applied = _applied_step(columns[mode, :], columns, step)
        bumped[mode, :] += applied
        realised = bumped[mode, :] - columns[mode, :]
        if not np.any(realised):
            raise CharacteristicsError(
                f"the perturbation step could not be represented at load magnitude "
                f"{float(np.max(np.abs(columns))):.3e} in mode {mode}; the difference "
                "quotient would divide by a step that was never added"
            )
        quotient = (
            _direct_answer(matrix, load_map, direct, bumped, measurement) - base
        ) / realised
        # Every load column now differs by exactly `h * B_w[:, mode]`, so all of them
        # carry the same derivative column; taking the first states that redundancy
        # rather than picking one arbitrarily.
        result[:, mode] = quotient[:, 0]
    return result


def _applied_step(
    load_row: np.ndarray, all_loads: np.ndarray, step: float
) -> np.ndarray:
    """
    Return the step to add to ``load_row``, scaled so the quotient stays accurate.

    A step that is small *relative to the loads* is the problem, whichever way it is
    reached.  Two distinct errors appear and they pull in opposite directions:

    * **absorption** -- the addend rounds away entirely, so the quotient divides by a
      step that was never added (this is what an absolute ``1e-7`` hits at
      ``loads ~ 1e8``);
    * **cancellation** -- ``Y(bumped) - Y(base)`` subtracts two nearly equal numbers,
      so the difference loses precision in proportion to ``|Y|`` rather than to the
      load being perturbed.

    The scale is therefore taken over **all** the loads and not just this mode's row.
    That distinction is not cosmetic: ``Y`` depends on the whole load vector, so
    perturbing a small mode inside a problem dominated by a large one suffers
    cancellation driven by the large entry.  Sizing by the row alone leaves exactly
    that case broken while looking correct for the dominant mode.

    A relative step of ``1e-6`` is used, with the caller's absolute value as a floor so
    that genuinely small problems keep the accuracy they already have.  Truncation
    error is not a concern at this size: the system is linear, so the quotient is exact
    in exact arithmetic at any step.
    """
    scale = float(np.max(np.abs(all_loads))) if all_loads.size else 0.0
    if scale > 0.0 and np.isfinite(scale):
        return np.full_like(load_row, max(step, scale * 1e-6))
    return np.full_like(load_row, step)


def response_operator_by_unit_basis(
    tangent: Any,
    load_jacobian: Any,
    direct_term: Any,
    measurement_map: Any,
) -> np.ndarray:
    """
    Recover ``C`` one unit basis vector at a time, reassembling the columns.

    SPEC §57 names this route explicitly ("independent full-unit-basis projections")
    alongside the direct and transpose routes, and it is genuinely independent of
    both: it never forms ``B_w`` as a matrix multiply against the whole load set, and
    it never touches ``A^T``.  What it catches is a *column* defect -- a merged or
    mis-ordered ``B_w`` column, or a wrong measurement projection -- because each
    column is obtained from a single unit load rather than from the whole matrix at
    once.

    It does **not** catch a transposed ``A``: it calls ``solve(matrix, ...)`` exactly
    as the direct route does, so that substitution leaves it unchanged.  The
    transpose route is what covers that case; claiming otherwise here would make this
    route look like a broader oracle than it is.

    The per-mode refusal is deliberate: a mode whose ``B_w`` column is entirely zero
    cannot be probed from that load direction at all, so it is reported rather than
    silently returned.
    """
    matrix = _matrix(tangent, name="tangent")
    load_map = _matrix(load_jacobian, name="load_jacobian")
    direct = _matrix(direct_term, name="direct_term")
    if matrix.shape[0] != matrix.shape[1]:
        raise CharacteristicsError("the tangent must be square")
    if load_map.shape[0] != matrix.shape[0]:
        raise CharacteristicsError("load_jacobian rows must match the tangent order")
    if direct.shape[1] != load_map.shape[1]:
        raise CharacteristicsError(
            f"direct_term must have {load_map.shape[1]} columns (one per load mode), "
            f"got {direct.shape}"
        )
    measurement = _measurement(measurement_map, order=matrix.shape[0], direct=direct)

    columns = []
    for mode in range(load_map.shape[1]):
        if not np.any(load_map[:, mode]):
            # Same unidentifiability as the perturbation route, by the same argument:
            # a unit load in an uncoupled direction produces no signal, and dividing
            # the untouched residual by the unit load yields a number unrelated to the
            # requested mode.
            raise CharacteristicsError(
                f"load mode {mode} has no coupling into the system, so its response "
                "is not identifiable from that load direction"
            )
        unit = np.zeros((load_map.shape[1], 1))
        unit[mode, 0] = 1.0
        columns.append(measurement @ np.linalg.solve(matrix, load_map @ unit))
    if not columns:
        raise CharacteristicsError("the load map has no modes to project")
    return np.hstack(columns) + direct


def response_operator_by_direct_route(
    tangent: Any,
    load_jacobian: Any,
    direct_term: Any,
    measurement_map: Any,
) -> np.ndarray:
    """
    Form the exact response operator ``C = G A^{-1} B_w + D`` explicitly.

    This is the quantity route 3 approximates, and the one a stiffness or
    compliance metric is read off.  It is computed here without reference to any
    load vector, which is what lets it be compared against a perturbation of an
    arbitrary load.
    """
    matrix = _matrix(tangent, name="tangent")
    load_map = _matrix(load_jacobian, name="load_jacobian")
    direct = _matrix(direct_term, name="direct_term")
    if matrix.shape[0] != matrix.shape[1]:
        raise CharacteristicsError("the tangent must be square")
    if load_map.shape[0] != matrix.shape[0]:
        raise CharacteristicsError("load_jacobian rows must match the tangent order")
    if direct.shape[1] != load_map.shape[1]:
        raise CharacteristicsError(
            f"direct_term must have {load_map.shape[1]} columns (one per load mode), "
            f"got {direct.shape}"
        )
    measurement = _measurement(measurement_map, order=matrix.shape[0], direct=direct)
    return measurement @ np.linalg.solve(matrix, load_map) + direct


def compare_routes(
    tangent: Any,
    load_jacobian: Any,
    direct_term: Any,
    loads: Any,
    measurement_map: Any,
    *,
    tolerance: float = 1e-9,
    perturbation_tolerance: float = 1e-5,
) -> dict[str, Any]:
    """
    Compute ``Y`` by all three routes and report the largest disagreement.

    Returns the comparison rather than a bare boolean is deliberate: a caller who
    only learns "the routes disagreed" cannot tell a tolerance problem from a wrong
    matrix, and the per-route gaps are what make the failure diagnosable.  Gaps are
    **relative** -- divided by the largest magnitude involved -- because these are
    stiffness-scale quantities where an absolute threshold would be meaningless at
    one scale and unusable at another.

    The three routes are compared on the **response operator** ``C = G A^{-1} B_w + D``
    (SPEC §52's subject), not on a response vector.  That requires care: the
    perturbation route returns ``dY/dP``, which is ``C``, whereas the direct route
    applied to one load returns ``C P``.  Comparing a derivative against a response
    produces a difference of order ``|C P|`` that looks like a real disagreement --
    so both linear routes are reduced to ``C`` here and the load only supplies the
    perturbation direction.
    """
    matrix, load_map, direct, columns = _checked(
        tangent, load_jacobian, direct_term, loads
    )
    measurement = _measurement(measurement_map, order=matrix.shape[0], direct=direct)

    # Route 1: the operator, formed directly.
    c_direct = response_operator_by_direct_route(
        matrix, load_map, direct, measurement
    )

    # Route 2: A^T Z = G^T, then C = Z^T B_w + D.  Algebraically identical to route
    # 1, but it uses A^T and a second factorization.
    z = np.linalg.solve(matrix.T, measurement.T)
    c_transpose = z.T @ load_map + direct

    # Route 3: SPEC §57's independent full-unit-basis projections.
    c_unit = response_operator_by_unit_basis(matrix, load_map, direct, measurement)

    # Route 4: the difference quotient.
    c_perturbed = response_by_finite_perturbation(
        matrix, load_map, direct, columns, measurement
    )

    # The response vector is retained for callers that want it, computed by the
    # direct route only -- it is a *consequence* of C, not a fifth route.
    y_direct = c_direct @ columns

    def gap(a: np.ndarray, b: np.ndarray) -> float:
        scale = max(
            1.0,
            float(np.max(np.abs(a))) if a.size else 0.0,
            float(np.max(np.abs(b))) if b.size else 0.0,
        )
        return float(np.max(np.abs(a - b))) / scale if a.size else 0.0

    transpose_gap = gap(c_direct, c_transpose)
    unit_gap = gap(c_direct, c_unit)
    perturbation_gap = gap(c_direct, c_perturbed)
    return {
        "direct": c_direct,
        "transpose": c_transpose,
        "unit_basis": c_unit,
        "finite_perturbation": c_perturbed,
        "response": y_direct,
        "transpose_gap": transpose_gap,
        "unit_basis_gap": unit_gap,
        "perturbation_gap": perturbation_gap,
        "tolerance": tolerance,
        "perturbation_tolerance": perturbation_tolerance,
        "symmetric_tangent": bool(np.allclose(matrix, matrix.T)),
        "agrees": transpose_gap <= tolerance
        and unit_gap <= tolerance
        and perturbation_gap <= perturbation_tolerance,
    }
