"""
The equilibrium response: `A X = B_w P` and `Y = G X + D P`, audited per mode.

Design §6.6 states the problem.  A quasi-static suspension holds

    R_m^B(x, w) = 0                                    (the mechanism at equilibrium)

and a *load mode* is a direction ``alpha_j`` in which the applied load is varied,
``P[:, j] = d w / d alpha_j``.  Differentiating the equilibrium condition gives the
response in two steps::

    A X = B_w P        A = dR/dx,  B_w = -dR/dw     (what the mechanism does)
    Y = G X + D P      G = dy/dx,  D = dy/dw        (what the measurement reads)

Two properties of that pair dominate the implementation, and both are about *not*
answering:

* **A cannot be inverted in general.**  The equilibrium mechanism is routinely
  singular in the sense that matters: a free body leaves rigid-body directions
  unconstrained, a wheel that has left the ground removes a contact row, and a
  redundant constraint makes rows dependent.  The design splits these into
  ``singular`` (a row constrains nothing) and ``unidentifiable`` (the rows are fine
  but leave a direction free), and they need different fixes -- so they are
  reported separately, and a mode that hits either is *not* answered.

* **A measurement can be unanswerable per mode even when A is perfect.**  For a
  square tangent, a unique ``X`` makes every gradient of ``X`` identifiable, so a
  *measurement*-side identifiability test would be dead code -- it can only fire
  when the tangent is already refused.  The live per-mode case is on the **load**
  side: a mode whose applied direction ``B_w P[:, j]`` is numerically zero asks the
  mechanism nothing, and ``X = A^{-1} 0 = 0`` would report a well-formed zero.  That
  is exactly the wrong answer this module exists to prevent, and the global audit
  cannot see it because the tangent is fine.  ``audited_modes`` refuses such a mode
  by name while answering its neighbours.

The direct and transpose strategies are both here because the design requires the
choice to be made on *independent right-hand sides and output count*, not on matrix
size.  ``solve_transpose`` is a real transposed system: the tangent is not assumed
symmetric, because a bushing with velocity terms is not.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from ..characteristics.motion import audit_boundary, rank_and_null_space
from ..characteristics.values import (
    CharacteristicResult,
    CharacteristicsError,
    statuses_for,
)
from .characteristics import characteristic

#: Load modes are named, and the name is carried through to the answer.
LOAD_MODE_KINDS = (
    "vertical_force",
    "lateral_force",
    "longitudinal_force",
    "aligning_torque",
    "roll_moment",
    "camber_torque",
)

#: Which load vector a mode perturbs.  The six entries are the ordered
#: ``[Fx, Fy, Fz, Mx, My, Mz]`` components of the applied load at the named point,
#: which is the column order the design fixes for the wheel-centre basis
#: (``x,y,z,rx,ry,rz`` rows against ``Fx,Fy,Fz,Mx,My,Mz`` columns).  A mode that is
#: not in this table has no defined sign, so it cannot be answered.
#:
#: The moment names follow the axis each one turns *about*, not the axis each
#: force acts *along*, so the two triples are not in the same order:
#:
#: * ``roll_moment`` is about **x** (indices 3) -- the repo's existing roll
#:   convention is "the roll moment about the x axis", so this must agree with it.
#: * ``camber_torque`` (overturning moment) is about **y**, the wheel's spin axis.
#: * ``aligning_torque`` is about **z**, the steering axis.
#:
#: Getting these three out of order is silent: every one of them would solve a
#: different physical system while producing a well-formed number.
LOAD_MODE_COMPONENTS: dict[str, int] = {
    "longitudinal_force": 0,
    "lateral_force": 1,
    "vertical_force": 2,
    "roll_moment": 3,
    "camber_torque": 4,
    "aligning_torque": 5,
}


@dataclass(frozen=True)
class LoadMode:
    """
    One named load direction, with the sign and unit of its amplitude.

    ``amplitude_units`` is the physical unit of ``alpha_j`` -- newtons for a force
    mode, newton-metres for a moment mode.  It is *not* the unit of the response:
    the response unit comes from the measurement.  Carrying both here is what lets
    a caller check that a mode is being driven in the unit it was declared in
    rather than in whatever the solver's internal scaling happens to be.
    """

    kind: str
    amplitude_units: str
    point: str = "contact_patch"

    def __post_init__(self) -> None:
        if self.kind not in LOAD_MODE_COMPONENTS:
            raise CharacteristicsError(
                f"unknown load mode {self.kind!r}; defined modes: "
                f"{', '.join(sorted(LOAD_MODE_COMPONENTS))}"
            )

    @property
    def component(self) -> int:
        """Index into the ordered ``[Fx, Fy, Fz, Mx, My, Mz]`` load vector."""
        return LOAD_MODE_COMPONENTS[self.kind]


@dataclass(frozen=True)
class ModeResult:
    """
    One answered load mode, or the reason it was refused.

    ``status`` and ``reason`` are per mode rather than per sample because the
    failure is a property of the *direction*: either this load direction produces
    an identifiable reading or it does not.  A different mode over the same sample
    can legitimately succeed, so collapsing the statuses would lose the one fact a
    caller needs.

    ``values`` is ``None`` exactly when the mode was refused.  It is never a
    zero-filled array: a refused mode that returned zeros would be indistinguishable
    from a mode whose response really is zero.
    """

    kind: str
    status: str
    reason: str
    values: np.ndarray | None
    units: str = ""

    def __post_init__(self) -> None:
        if self.values is None and not self.reason:
            raise CharacteristicsError(
                f"refused load mode {self.kind!r} must say why it was refused"
            )
        if self.values is not None and self.reason:
            raise CharacteristicsError(
                f"answered load mode {self.kind!r} must not carry a refusal reason"
            )


def load_mode_matrix(
    modes: Sequence[LoadMode],
    *,
    load_dimension: int = 6,
) -> np.ndarray:
    """
    Build ``P``, the ``(load_dimension, mode_count)`` direction matrix.

    Each column is a unit perturbation along one mode's component.  The matrix is
    built here rather than assembled ad hoc at each call site so the *sign*
    convention has one home: a positive amplitude drives the named component in the
    positive frame direction, and there is no per-call-site opportunity to flip it.
    """
    if load_dimension < 1:
        raise CharacteristicsError("load dimension must be positive")
    columns = len(modes)
    if columns == 0:
        raise CharacteristicsError("a response request must name at least one load mode")
    matrix = np.zeros((load_dimension, columns), dtype=float)
    for index, mode in enumerate(modes):
        if mode.component >= load_dimension:
            raise CharacteristicsError(
                f"load mode {mode.kind!r} needs component {mode.component} but the load "
                f"vector has only {load_dimension} entries"
            )
        matrix[mode.component, index] = 1.0
    return matrix


def equilibrium_response(
    tangent: Any,
    load_jacobian: Any,
    mode_matrix: Any,
    *,
    measurement_gradient: Any | None = None,
    measurement_load_gradient: Any | None = None,
    tangent_tolerance: float = 1e-10,
    mode_count: int | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    """
    Solve ``A X = B_w P`` and, when a measurement is given, ``Y = G X + D P``.

    Both right-hand sides are solved at once through the batched factorization the
    design asks for, so the cost is one factorization for every mode rather than
    one per mode.  That is the whole reason the batch interface exists, and the
    returned audit reports how many factorizations were actually spent.

    The tangent is audited *before* it is used.  A singular or underdetermined
    tangent means the equilibrium condition does not determine a motion, and the
    X that any solver would return is one member of a family; the audit is what
    lets the caller refuse the answer instead of publishing that member.

    Returns ``(Y_or_X, audit)``.  When ``measurement_gradient`` is ``None`` the
    first element is ``X`` -- the mechanism motion -- because a caller that asked
    only what the mechanism does should not be handed a measurement-shaped array.
    """
    matrix = np.asarray(tangent, dtype=float)
    if matrix.ndim != 2 or matrix.shape[0] != matrix.shape[1]:
        raise CharacteristicsError(
            f"the physical tangent must be square, got shape {matrix.shape}"
        )
    dimension = matrix.shape[0]
    applied = np.asarray(load_jacobian, dtype=float)
    if applied.ndim != 2 or applied.shape[0] != dimension:
        raise CharacteristicsError(
            f"the load jacobian must be ({dimension}, n_load), got shape {applied.shape}"
        )
    directions = np.asarray(mode_matrix, dtype=float)
    if directions.ndim != 2 or directions.shape[0] != applied.shape[1]:
        raise CharacteristicsError(
            f"the mode matrix must have {applied.shape[1]} rows to match the load "
            f"jacobian, got shape {directions.shape}"
        )

    audit = audit_boundary(matrix, tolerance=tangent_tolerance)
    audit["mode_count"] = int(directions.shape[1])
    audit["load_dimension"] = int(applied.shape[1])
    # A request whose mode matrix has a different column count than the modes the
    # caller believes it asked for would silently return the wrong number of
    # columns.  This is checked explicitly because the caller's mode list is what
    # labels the columns, and nothing downstream can recover the mismatch.
    if mode_count is not None and directions.shape[1] != mode_count:
        raise CharacteristicsError(
            f"the mode matrix has {directions.shape[1]} column(s) for {mode_count} "
            "declared load mode(s); the columns are what label the response"
        )
    # A refused response is filled with NaN rather than zeros.  This matters more
    # than it looks: a zero-filled refusal is numerically indistinguishable from a
    # mechanism whose response genuinely is zero, so a caller that forgot to read
    # `status` would publish a plausible wrong number.  NaN cannot be mistaken for
    # a reading, and every consumer of this module already treats non-finite as
    # "not available".
    refusal = np.full(
        (dimension, directions.shape[1]), np.nan, dtype=float
    )
    if audit["status"] == "singular":
        # A zero row constrains nothing, so the equilibrium does not hold as
        # stated.  Solving would attach a number to a broken system.
        return refusal, audit

    # `B_w P` is the *applied* load direction: the residual derivative with respect
    # to the load enters with the sign that makes a positive amplitude increase the
    # load, so B_w is used as given and the caller is responsible for it carrying
    # the equilibrium sign convention.  It is not silently negated here, because a
    # second sign flip in this function would be invisible against the first.
    right_hand_sides = applied @ directions

    solution, spent = _solve_block(
        matrix, right_hand_sides, tolerance=tangent_tolerance
    )
    audit["factorizations"] = spent
    if solution is None:
        audit["status"] = "unidentifiable"
        audit["reason"] = (
            f"the equilibrium tangent leaves {audit['nullity']} unconstrained "
            "direction(s), so the mechanism motion is not determined by the load; "
            "no mode-specific response can be reported"
        )
        return refusal, audit
    if not np.isfinite(solution).all():
        # A well-conditioned tangent can still overflow: a large load against a
        # small stiffness produces a mathematically finite but unrepresentable
        # motion.  Reporting `valid` here would push the non-finite values into a
        # consumer, where `statuses_for` would reject them *without a reason* and
        # the caller would get an exception instead of a per-mode refusal.
        audit["status"] = "unidentifiable"
        audit["reason"] = (
            "the solved mechanism motion is not representable in double precision "
            "(the response overflowed); the load is too large for this stiffness"
        )
        return refusal, audit

    if measurement_gradient is None:
        return solution, audit

    gradient = np.asarray(measurement_gradient, dtype=float)
    if gradient.ndim == 1:
        gradient = gradient.reshape(1, -1)
    if gradient.shape[1] != dimension:
        raise CharacteristicsError(
            f"the measurement gradient must have {dimension} columns, got {gradient.shape}"
        )
    if gradient.shape[0] < 1:
        raise CharacteristicsError(
            "the measurement gradient must have at least one row; a measurement "
            "request with no measurement has nothing to answer"
        )
    # No relation between the row count and the mode count is required.  `G` is
    # (measurements, ndof) and `X` is (ndof, modes), so `G X` is
    # (measurements, modes) for any measurement count.  An earlier version required
    # the row count to be 1 or the mode count, which conflated the two counts and
    # refused the design's canonical request: one in-phase mode measuring both wheel
    # centres (design §6.7, the Wheel Rate shape).  The counts are independent.
    audit["measurement_count"] = int(gradient.shape[0])

    # `Y = G X + D P`.  The direct term matters when the measurement reads the load
    # itself (a wheel load, for instance): dropping it because it is usually zero
    # would make those measurements wrong by exactly the amount that matters.
    response = gradient @ solution
    if measurement_load_gradient is not None:
        direct = np.asarray(measurement_load_gradient, dtype=float)
        if direct.ndim == 1:
            direct = direct.reshape(1, -1)
        if direct.shape[1] != applied.shape[1]:
            raise CharacteristicsError(
                f"the direct measurement gradient must have {applied.shape[1]} columns, "
                f"got {direct.shape}"
            )
        if direct.shape[0] not in (1, response.shape[0]):
            raise CharacteristicsError(
                f"the direct gradient has {direct.shape[0]} rows for "
                f"{response.shape[0]} output rows"
            )
        response = response + direct @ directions
    if not np.isfinite(response).all():
        # The `G X + D P` chain can overflow even when X itself did not: a large
        # `D` multiplying a large mode amplitude is enough.  Caught here so the
        # refusal carries the audit's reason rather than surfacing later as a
        # consumer's reasonless "no numeric value" error.
        audit["status"] = "unidentifiable"
        audit["reason"] = (
            "the measurement response is not representable in double precision "
            "(the G X + D P chain overflowed)"
        )
        return refusal, audit
    return response, audit


def _solve_block(
    matrix: np.ndarray,
    right_hand_sides: np.ndarray,
    *,
    tolerance: float,
) -> tuple[np.ndarray | None, int]:
    """
    Solve a square system with several right-hand sides, refusing a family.

    Returns ``(None, 0)`` when the system is rank-deficient: the caller turns that
    into ``unidentifiable`` with a reason, rather than receiving a minimum-norm
    representative that would look like an answer.

    The solve is ``numpy.linalg.solve`` on the whole block, so one factorization
    serves every column -- the property the design wants and the earlier
    ``mb_linear`` batch interface makes explicit at the C++ layer.  ``spent`` is 1
    to say so; a per-column loop would report the mode count.
    """
    rank, null = rank_and_null_space(matrix, tolerance=tolerance)
    if rank < matrix.shape[0] or null.shape[1] != 0:
        return None, 0
    try:
        return np.linalg.solve(matrix, right_hand_sides), 1
    except np.linalg.LinAlgError:
        # The explicit rank test above passed but the factorisation still refused
        # the matrix.  That disagreement is real information -- it means the matrix
        # is numerically at the edge of the tolerance -- so it is reported as a
        # family rather than crashed on.
        return None, 0


def audited_modes(
    tangent: Any,
    load_jacobian: Any,
    modes: Sequence[LoadMode],
    *,
    mode_matrix: Any | None = None,
    measurement_gradient: Any | None = None,
    measurement_units: str = "",
    measurement_load_gradient: Any | None = None,
    tangent_tolerance: float = 1e-10,
) -> tuple[tuple[ModeResult, ...], dict[str, Any]]:
    """
    Answer each load mode that is identifiable, and name each one that is not.

    Two levels of refusal live here, and they are different failures:

    * A globally bad tangent (``singular`` / ``unidentifiable``) refuses every mode,
      because the mechanism motion is not determined at all.
    * A mode whose *applied* direction ``B_w P[:, j]`` is numerically zero is
      refused on its own while its neighbours are answered.  The tangent is fine
      here, so only a per-mode check can find it -- and without it the solver would
      return ``A^{-1} 0 = 0``, a well-formed zero that looks exactly like a
      mechanism which genuinely does not respond to that load.

    Each mode's refusal carries its own reason, and a refused mode carries no
    numbers, so a caller cannot accidentally treat an undefined response as zero.

    ``mode_matrix`` is ``P``, the per-mode amplitude matrix whose column ``j`` is
    ``partial w / partial alpha_j`` for mode ``j`` (design §6.6).  It is optional so
    the common unit-basis request stays a one-argument call, but it is **not** a
    check: when supplied, those numbers are the ones solved for.  The amplitude, the
    wheel end, the axis and the application point all live in ``P``, and re-deriving
    it from ``modes`` would silently ignore every one of them -- solving the unit
    basis and reporting the answer under the caller's mode names.
    """
    matrix = np.asarray(tangent, dtype=float)
    if mode_matrix is None:
        directions = load_mode_matrix(modes)
    else:
        directions = np.asarray(mode_matrix, dtype=float)
        if directions.ndim != 2 or directions.shape[1] != len(modes):
            raise CharacteristicsError(
                f"the mode matrix must be (n_load, {len(modes)}) to match the named "
                f"load modes, got shape {directions.shape}"
            )
    gradient = None if measurement_gradient is None else np.asarray(
        measurement_gradient, dtype=float
    )
    applied = np.asarray(load_jacobian, dtype=float)

    response, audit = equilibrium_response(
        matrix,
        applied,
        directions,
        measurement_gradient=gradient,
        measurement_load_gradient=measurement_load_gradient,
        tangent_tolerance=tangent_tolerance,
        mode_count=len(modes),
    )

    results: list[ModeResult] = []
    # The reference for "is this mode's applied column negligible?" is the largest
    # applied column in *this* request, computed once.  See the per-mode comment
    # below for why an absolute floor is wrong.
    applied_columns = applied @ directions
    reference = max(
        (float(np.linalg.norm(applied_columns[:, other]))
         for other in range(applied_columns.shape[1])),
        default=0.0,
    )
    for index, mode in enumerate(modes):
        if audit["status"] in ("singular", "unidentifiable"):
            results.append(
                ModeResult(kind=mode.kind, status=audit["status"], reason=audit["reason"], values=None)
            )
            continue
        # A mode can be unanswerable on its own even when the tangent is perfect.
        #
        # The applied direction for this mode is the column `B_w P[:, j]`.  If that
        # column is (numerically) zero, the mode asks the mechanism nothing at all:
        # the load vector cannot be driven along it, so the response is not small,
        # it is undefined.  Answering `X = A^{-1} 0 = 0` would report a
        # well-formed zero -- the exact failure mode this module exists to prevent,
        # and one the global audit cannot see because the tangent is fine.
        #
        # The comparison is *relative to this mode's own column scale*, not to a
        # global absolute floor.  A load jacobian whose entries are all ~1e-11 is
        # entirely legitimate (a small load on a stiff mechanism), and an absolute
        # threshold would refuse every one of its modes while the audit still
        # reported `valid`.  What matters is whether the column is negligible
        # *compared to the load directions this request actually expresses*, so the
        # reference is the largest column the request contains.  A request whose
        # every column is zero then has reference 0, and 0 <= 0 correctly refuses
        # all of them.
        column = applied_columns[:, index]
        if float(np.linalg.norm(column)) <= tangent_tolerance * reference:
            results.append(
                ModeResult(
                    kind=mode.kind,
                    status="unidentifiable",
                    reason=(
                        f"load mode {mode.kind!r} drives no load direction that the "
                        "mechanism responds to: its column of B_w P is negligible "
                        "against the other modes in this request, so the response is "
                        "undefined rather than zero"
                    ),
                    values=None,
                )
            )
            continue
        values = response[:, index]
        status, reason = statuses_for(values)
        results.append(
            ModeResult(
                kind=mode.kind,
                status="valid" if all(item == "valid" for item in status) else status[0],
                reason="" if all(item == "valid" for item in status) else reason[0],
                values=np.asarray(values, dtype=float),
                units=measurement_units,
            )
        )
    # Record the per-mode refusals on the audit, not only on the individual
    # `ModeResult`s.  A caller that reads only `audit` would otherwise see
    # `status == "valid"` while some modes were refused, which is the same
    # "looks fine unless you read the other object" trap this module exists to
    # avoid.  `audit["status"]` stays as the *tangent's* verdict -- the two are
    # different questions -- and the refused mode names are listed separately.
    refused = [item.kind for item in results if item.values is None]
    audit["refused_modes"] = refused
    audit["answered_modes"] = [item.kind for item in results if item.values is not None]
    return tuple(results), audit


def response_request(
    result: Any,
    request_id: str,
    *,
    modes: Sequence[LoadMode],
    responses: dict[str, tuple[np.ndarray, str, str]],
    boundary_profile: str,
    boundary_version: int,
    definition_kind: str = "equilibrium_load_response",
    definition_version: int = 1,
    boundary_fingerprint: str = "",
    extra: Any = None,
) -> CharacteristicResult:
    """
    Present per-mode equilibrium responses as one answered request.

    ``responses`` maps a measurement ID to ``(values, units, frame_id)``, where the
    values are indexed by *mode*: entry ``j`` is the response to mode ``j``.  That
    layout is deliberate -- it keeps the answer aligned with the caller's own mode
    list, so a table column can be labelled by the mode name the caller used rather
    than by a position that depends on how the request happened to be ordered.
    """
    if not modes:
        raise CharacteristicsError("an equilibrium request must name at least one load mode")
    channels: dict[str, tuple[Any, Any, str]] = {}
    names = tuple(mode.kind for mode in modes)
    for name, (values, units, frame_id) in responses.items():
        array = np.asarray(values, dtype=float)
        # The response must be exactly one number per mode.  Checking only the first
        # axis would let a 2-D array through, and downstream it would be read as a
        # `(samples, components)` vector -- a completely different quantity wearing
        # the right first dimension.  A scalar also has to be refused with a
        # contract error rather than an `IndexError` from indexing `shape[0]`.
        if array.ndim != 1:
            raise CharacteristicsError(
                f"response {name!r} must be one value per load mode (shape "
                f"({len(modes)},)), got shape {array.shape}"
            )
        if array.shape[0] != len(modes):
            raise CharacteristicsError(
                f"response {name!r} has {array.shape[0]} entries for {len(modes)} load modes"
            )
        if not np.isfinite(array).all():
            # Non-finite entries reach `measurement_from_channel`, which refuses to
            # invent a reason.  A response in this layer is either computed or
            # absent, so a NaN here is a defect and is named as one rather than
            # passed down to become an anonymous `undefined_geometry`.
            bad = int(np.sum(~np.isfinite(array)))
            raise CharacteristicsError(
                f"response {name!r} carries {bad} non-finite entr(ies); a load-mode "
                "response is either answered or refused, not partially numeric"
            )
        channels[name] = (array, units, frame_id)
    return characteristic(
        result,
        request_id,
        domain="equilibrium",
        definition_kind=definition_kind,
        definition_version=definition_version,
        channels=channels,
        boundary_profile=boundary_profile,
        boundary_version=boundary_version,
        boundary_fingerprint=boundary_fingerprint,
        response_operator="equilibrium_response",
        operator_version=1,
        extra={
            "load_modes": list(names),
            "load_mode_units": [mode.amplitude_units for mode in modes],
            **dict(extra or {}),
        },
    )


__all__ = [
    "LOAD_MODE_COMPONENTS",
    "LOAD_MODE_KINDS",
    "LoadMode",
    "ModeResult",
    "audited_modes",
    "equilibrium_response",
    "load_mode_matrix",
    "response_request",
]
