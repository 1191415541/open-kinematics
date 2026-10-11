"""
Kinematic rank, null-space and identifiability audits.

This module answers one question and refuses to answer a different one.  The
question it answers is: *given the constraint derivative the native side already
computed, is the requested quantity actually determined by the boundary?*  The
question it must not answer is "here is a plausible-looking motion", which is
what a least-squares solve returns when it is not checked.

The distinction matters because a minimum-norm solution exists for every
underdetermined system.  Handing that back as if it were the mechanism's motion
would silently invent the very degree of freedom the boundary failed to
constrain.  So the audit runs first and the solve only runs when the audit says
the answer is unique.

Nothing here differentiates numerically.  The derivative is the native
constraint Jacobian; a difference quotient is a test oracle (and is named
``oracle_difference`` where it appears in a value type), never a production
fallback.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from .values import (
    ANGULAR_SPEED_FLOOR,
    RESULTANT_FLOOR,
    SAMPLE_STATUSES,
    CharacteristicsError,
)

#: Below this singular value a direction is treated as unreachable.  It is a
#: judgement about floating-point rank, not a physical tolerance, so it is
#: deliberately much tighter than the P0 measurement tolerances.
RANK_TOLERANCE = 1e-10


def _matrix(jacobian: Any, label: str) -> np.ndarray:
    array = np.asarray(jacobian, dtype=float)
    if array.ndim != 2:
        raise CharacteristicsError(f"{label} must be a two-dimensional matrix")
    if not np.isfinite(array).all():
        raise CharacteristicsError(f"{label} contains non-finite entries")
    return array


def rank_and_null_space(jacobian: Any, *, tolerance: float = RANK_TOLERANCE) -> tuple[int, np.ndarray]:
    """
    Return the numerical rank and an orthonormal basis of the null space.

    The basis is returned with shape ``(dof, dof - rank)``; it is empty (and the
    system is uniquely determined) when the rank equals the column count.
    """
    matrix = _matrix(jacobian, "constraint jacobian")
    if matrix.size == 0:
        # No constraints at all: every direction is free.
        return 0, np.eye(matrix.shape[1])
    left, singular, _ = np.linalg.svd(matrix, full_matrices=True)
    del left
    rank = int(np.sum(singular > tolerance * max(singular[0], 1.0)))
    null = np.linalg.svd(matrix.conj().T, full_matrices=True)[0][:, rank:]
    return rank, null


def is_identifiable(jacobian: Any, direction: Any, *, tolerance: float = RANK_TOLERANCE) -> bool:
    """
    Whether a projection direction is determined by the constraints.

    A direction ``g`` is identifiable exactly when it is orthogonal to the
    null space: ``g N = 0``.  If ``g`` has any component in the null space then
    two different motions produce the same constraint residual and the same
    reading, so the reading does not identify the motion.
    """
    matrix = _matrix(jacobian, "constraint jacobian")
    gradient = np.asarray(direction, dtype=float).ravel()
    if gradient.shape[0] != matrix.shape[1]:
        raise CharacteristicsError(
            f"projection has {gradient.shape[0]} components for {matrix.shape[1]} coordinates"
        )
    _, null = rank_and_null_space(matrix, tolerance=tolerance)
    if null.size == 0:
        return True
    projection = gradient @ null
    scale = max(float(np.linalg.norm(gradient)), 1.0)
    return bool(np.linalg.norm(projection) <= tolerance * scale)


def audit_boundary(
    jacobian: Any,
    *,
    driven_rows: Any | None = None,
    residual: Any | None = None,
    tolerance: float = RANK_TOLERANCE,
) -> dict[str, Any]:
    """
    Audit one sample's boundary and report what can honestly be said about it.

    The returned mapping carries a contract status rather than a bare boolean,
    because the two failures need different fixes:

    * ``singular`` means a constraint row constrains nothing at all -- an
      all-zero row, which happens when a joint's axes degenerate.  That is a
      defect in the model or the state, and it is reported even when the
      remaining rows happen to determine the motion.
    * ``unidentifiable`` means the rows are individually fine but leave a
      direction free, so the requested quantity is not pinned down.
    """
    matrix = _matrix(jacobian, "constraint jacobian")
    if driven_rows is not None:
        rows = np.asarray(driven_rows, dtype=float)
        if rows.ndim == 1:
            rows = rows.reshape(1, -1)
        if rows.shape[1] != matrix.shape[1]:
            raise CharacteristicsError("driven rows disagree with the constraint column count")
        matrix = np.vstack((matrix, rows))
    rank, null = rank_and_null_space(matrix, tolerance=tolerance)
    report: dict[str, Any] = {
        "row_count": int(matrix.shape[0]),
        "dof": int(matrix.shape[1]),
        "rank": rank,
        "nullity": int(null.shape[1]),
        "unique": bool(null.shape[1] == 0),
        "residual_norm": None,
        "redundant_rows": int(matrix.shape[0] - rank),
        "empty_rows": 0,
        "status": "valid",
        "reason": "",
    }
    if residual is not None:
        values = np.asarray(residual, dtype=float).ravel()
        report["residual_norm"] = float(np.linalg.norm(values))
    if matrix.shape[0]:
        # A row of exact zeros constrains nothing.  Silently counting it as a
        # satisfied constraint would make a degenerate joint look like a held
        # one, so it is surfaced before the rank question is asked.
        norms = np.linalg.norm(matrix, axis=1)
        report["empty_rows"] = int(np.sum(norms <= tolerance))
    if report["empty_rows"]:
        report["status"] = "singular"
        report["reason"] = (
            f"{report['empty_rows']} constraint row(s) are identically zero and "
            "constrain nothing; the boundary is degenerate at this sample"
        )
    elif null.shape[1] != 0:
        # The system admits a family of motions.  Whether that is a defect
        # depends on the requested measurement, so the caller decides; the
        # audit reports the fact and its size.
        report["status"] = "unidentifiable"
        report["reason"] = (
            f"constraint set leaves {int(null.shape[1])} unconstrained direction(s); "
            "a minimum-norm motion would not be the mechanism's motion"
        )
    # Redundant rows on their own are not a defect.  Two constraints that state
    # the same thing leave the motion exactly as determined as one of them does,
    # so nullity zero means the boundary is well posed no matter how many
    # dependent rows it carries -- `redundant_rows` records the count for a
    # consumer that cares about the assembly, and that is all it means.
    return report


def solve_motion(
    jacobian: Any,
    targets: Any,
    *,
    tolerance: float = RANK_TOLERANCE,
) -> tuple[np.ndarray, dict[str, Any]]:
    """
    Solve for the motion that meets ``targets``, or refuse to invent one.

    A solution is returned only when it is unique.  When the system is
    underdetermined this raises instead of returning the minimum-norm
    representative, because that representative is one arbitrary member of a
    family and reporting it as *the* motion is the failure this function exists
    to prevent.  The audit is returned alongside a successful solution so the
    rank and residual travel with the numbers.
    """
    matrix = _matrix(jacobian, "constraint jacobian")
    wanted = np.asarray(targets, dtype=float).ravel()
    if wanted.shape[0] != matrix.shape[0]:
        raise CharacteristicsError(
            f"{wanted.shape[0]} targets for {matrix.shape[0]} constraint rows"
        )
    audit = audit_boundary(matrix, residual=None, tolerance=tolerance)
    if audit["status"] == "singular":
        # A degenerate row means the boundary itself is broken at this sample;
        # solving it would produce a number with no physical meaning behind it.
        raise CharacteristicsError(audit["reason"])
    if audit["nullity"] != 0:
        raise CharacteristicsError(audit["reason"])
    motion, *_ = np.linalg.lstsq(matrix, wanted, rcond=None)
    achieved = matrix @ motion
    audit["residual_norm"] = float(np.linalg.norm(achieved - wanted))
    if audit["residual_norm"] > 1e-8 * max(1.0, float(np.linalg.norm(wanted))):
        # A full-rank system that still cannot hit the targets is inconsistent,
        # which is a different defect from being underdetermined.
        audit["status"] = "inconsistent_sample"
        audit["reason"] = (
            f"driving is inconsistent with the constraints (residual "
            f"{audit['residual_norm']:.3e}); the targets are not reachable"
        )
        raise CharacteristicsError(audit["reason"])
    return motion, audit


def measurement_response(
    jacobian: Any,
    measurement_gradient: Any,
    input_row: Any,
    *,
    tolerance: float = RANK_TOLERANCE,
) -> tuple[float, dict[str, Any]]:
    """
    Return the derivative of a measurement with respect to one declared input.

    ``jacobian`` is ``d(constraint)/d(q)``, ``measurement_gradient`` is
    ``d(measure)/d(q)``, and ``input_row`` is the row of ``d(input)/d(q)`` that
    drives the motion.  The result is the change of the measurement per unit of
    that input.

    The identifiability audit runs first and the solve happens only when the
    motion is unique, so an unidentifiable direction is reported as such rather
    than being answered with a projection onto whatever the solver picked.
    """
    matrix = _matrix(jacobian, "constraint jacobian")
    gradient = np.asarray(measurement_gradient, dtype=float).ravel()
    if gradient.shape[0] != matrix.shape[1]:
        raise CharacteristicsError("measurement gradient disagrees with the coordinate count")
    if not is_identifiable(matrix, gradient, tolerance=tolerance):
        raise CharacteristicsError(
            "the requested measurement is not determined by the boundary: its gradient "
            "has a component in the unconstrained null space"
        )
    row = np.asarray(input_row, dtype=float).ravel()
    if row.shape[0] != matrix.shape[0]:
        raise CharacteristicsError(
            f"input row has {row.shape[0]} entries for {matrix.shape[0]} constraint rows"
        )
    motion, audit = solve_motion(matrix, row, tolerance=tolerance)
    return float(gradient @ motion), audit


@dataclass(frozen=True)
class Twist:
    """A rigid velocity: the angular velocity and the velocity of the world origin."""

    omega: np.ndarray
    velocity: np.ndarray

    def __post_init__(self) -> None:
        for name in ("omega", "velocity"):
            value = np.array(getattr(self, name), dtype=float).ravel()
            if value.shape != (3,):
                raise CharacteristicsError(f"a twist's {name} must be a three-vector")
            if not np.isfinite(value).all():
                raise CharacteristicsError(f"a twist's {name} must be finite")
            object.__setattr__(self, name, value)

    def transform_point(self, point: Any) -> np.ndarray:
        """Return the velocity of the world point ``point``."""
        location = np.asarray(point, dtype=float).ravel()
        if location.shape != (3,):
            raise CharacteristicsError("a twist can only be evaluated at a three-vector point")
        return self.velocity + np.cross(self.omega, location)


@dataclass(frozen=True)
class Screw:
    """
    A space instantaneous axis: one point of it, its direction and its pitch.

    ``pitch`` is the advance along ``direction`` per radian of rotation, so a pure
    rotation has ``pitch == 0``.  ``point`` is a point of the axis, not a fixed
    point: any point on the axis describes the same screw, which is why the point
    closest to a caller-chosen anchor is the one reported.
    """

    point: np.ndarray
    direction: np.ndarray
    pitch: float
    angular_speed: float

    def __post_init__(self) -> None:
        for name in ("point", "direction"):
            value = np.array(getattr(self, name), dtype=float).ravel()
            if value.shape != (3,):
                raise CharacteristicsError(f"a screw's {name} must be a three-vector")
            object.__setattr__(self, name, value)


def screw_from_twist(twist: Twist, *, anchor: Any = None) -> Screw:
    """
    Return the space instantaneous axis of ``twist``.

    ``anchor`` picks which point of the axis is reported: the point closest to
    ``anchor``, or to the world origin when it is ``None``.  A motion with no
    rotation has no axis and is refused -- reporting one at a very large distance
    would be a different claim from "this motion has no axis".
    """
    omega = np.asarray(twist.omega, dtype=float)
    speed = float(np.linalg.norm(omega))
    if not np.isfinite(speed) or speed <= ANGULAR_SPEED_FLOOR:
        raise CharacteristicsError(
            "this motion has no angular velocity, so it has no instantaneous axis: a "
            "pure translation is not a screw"
        )
    velocity = np.asarray(twist.velocity, dtype=float)
    point = np.cross(omega, velocity) / (speed * speed)
    direction = omega / speed
    if anchor is not None:
        offset = np.asarray(anchor, dtype=float).ravel() - point
        point = point + direction * float(offset @ direction)
    return Screw(
        point=point,
        direction=direction,
        pitch=float(omega @ velocity) / (speed * speed),
        angular_speed=speed,
    )


def relative_twist(twist: Twist, reference: Twist | None) -> Twist:
    """
    Return ``twist`` measured in a frame moving with ``reference``.

    A ``None`` reference means the world, and the twist is returned unchanged --
    which keeps the caller from having to special-case the world reference at
    every call site.
    """
    if reference is None:
        return twist
    return Twist(
        omega=np.asarray(twist.omega) - np.asarray(reference.omega),
        velocity=np.asarray(twist.velocity) - np.asarray(reference.velocity),
    )


def kingpin_axis(
    lower_point: Any,
    upper_point: Any,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return the steering (kingpin) axis direction and a point on it.

    The axis is defined by the two ball joints, so it is a geometric fact of the
    suspension rather than something inferred from a body name.  A vertical axis
    has no horizontal projection, and the caller that needs one (for caster or
    scrub radius) must be told so instead of silently receiving a zero.
    """
    lower = np.asarray(lower_point, dtype=float).ravel()
    upper = np.asarray(upper_point, dtype=float).ravel()
    if lower.shape != (3,) or upper.shape != (3,):
        raise CharacteristicsError("the kingpin joints must be three-vectors")
    direction = upper - lower
    length = float(np.linalg.norm(direction))
    if length <= RESULTANT_FLOOR:
        raise CharacteristicsError(
            "the two kingpin points coincide, so the steering axis is not defined"
        )
    return direction / length, lower


def kingpin_inclination(axis_direction: Any, *, axis: str = "y") -> float:
    """
    Return the angle between the steering axis and the vertical in a plane.

    ``axis`` names the plane's normal: ``"y"`` looks at the x-z (front) view and
    gives the inclination commonly called KPI; ``"x"`` looks at the y-z (side)
    view and gives the caster.  Both are measured from vertical *within* that
    view, which is the angle a person reads off a front or side elevation.

    The only undefined case is an axis parallel to the plane's normal: it has no
    component in the view at all, so there is no direction to measure.  A
    vertical axis is not that case -- it is simply zero degrees in every view --
    and is returned as zero rather than refused.
    """
    direction = np.asarray(axis_direction, dtype=float).ravel()
    if direction.shape != (3,):
        raise CharacteristicsError("the steering axis direction must be a three-vector")
    normal = {"x": 0, "y": 1}.get(axis)
    if normal is None:
        raise CharacteristicsError(f"unknown inclination axis {axis!r}; use 'x' or 'y'")
    # Within the view, the other horizontal component is the one that leans the
    # axis away from vertical; the remaining component is the vertical one.
    lateral_index = 0 if normal == 1 else 1
    lateral = abs(float(direction[lateral_index]))
    vertical = float(direction[2])
    if lateral <= RESULTANT_FLOOR and abs(vertical) <= RESULTANT_FLOOR:
        raise CharacteristicsError(
            f"the steering axis is parallel to the {axis} axis, so it has no component "
            "in the requested view and its inclination there is not defined"
        )
    return float(np.arctan2(lateral, abs(vertical)))


def ackermann_error(
    inner_angle: float,
    outer_angle: float,
    *,
    track: float,
    wheelbase: float,
) -> float:
    """
    Return the Ackermann error in radians: the difference from the ideal relation.

    The ideal geometry turns the outer wheel **less** than the inner one, and
    satisfies ``cot(outer) - cot(inner) = track / wheelbase``, i.e.
    ``cot(outer) = cot(inner) + track / wheelbase``.  The addition is the part
    that is easy to get backwards: subtracting instead produces an ideal outer
    angle *larger* than the inner one, which is anti-Ackermann, and the sign of
    every reported error flips with it.  A parallel-steer pair then reports a
    negative error and a correctly-steered pair a positive one -- exactly
    inverted, while both remain finite and plausible.

    A non-positive wheelbase or track makes the condition meaningless, so it is
    refused rather than producing an infinite ideal angle.
    """
    if not (np.isfinite(inner_angle) and np.isfinite(outer_angle)):
        raise CharacteristicsError("the steer angles must be finite")
    if track <= 0.0:
        raise CharacteristicsError("Ackermann needs a positive track width")
    if wheelbase <= 0.0:
        raise CharacteristicsError("Ackermann needs a positive wheelbase")
    if abs(np.sin(inner_angle)) <= RESULTANT_FLOOR:
        raise CharacteristicsError(
            "the inner steer angle is zero, so the ideal outer angle is not defined"
        )
    ideal_outer = 1.0 / (1.0 / np.tan(inner_angle) + track / wheelbase)
    return float(outer_angle - np.arctan(ideal_outer))


def sample_status_for(audit: dict[str, Any]) -> tuple[str, str]:
    """Reduce an audit to a contract ``(status, reason)`` pair."""
    status = str(audit.get("status", "valid"))
    if status not in SAMPLE_STATUSES:
        raise CharacteristicsError(f"audit produced an unknown status {status!r}")
    return status, str(audit.get("reason", ""))


__all__ = [
    "RANK_TOLERANCE",
    "audit_boundary",
    "is_identifiable",
    "measurement_response",
    "rank_and_null_space",
    "sample_status_for",
    "solve_motion",
]
