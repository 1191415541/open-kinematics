"""
The single implementation of the angle and frame-position measurements.

R1 retires the formulas that lived inside ``results/envelope.py::measure``
(frame position, toe, camber) into this module.  The envelope keeps its query,
its unit check and its read-only return; the arithmetic moves here so there is
exactly one place where a sign convention is decided.

Two rules this module exists to enforce:

* **Angles are frame-covariant, and the frame is named.**  A toe angle is
  meaningless without saying which frame the axis was resolved in.  Both the
  measurement frame and the reference frame are parameters, never module
  constants, and a rotation of either changes the answer as it should.

* **A degenerate geometry is reported, not approximated.**  A wheel axis with no
  horizontal projection has no toe angle; returning 0 would be indistinguishable
  from a wheel pointing straight ahead.

The functions here take plain arrays.  They do not read a result envelope, do not
import `results`, and do not issue a native request, which is what lets the query
layer delegate to them without inverting a dependency.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from .values import CharacteristicsError

#: Unit tokens the contract accepts for these measurements.  Kept here rather
#: than inline so a new angle cannot be added with a silently different token.
LENGTH_UNITS = "m"
ANGLE_UNITS = "rad"


def _as_pose_batch(poses: Any, label: str) -> np.ndarray:
    """Validate an ``(samples, 4, 4)`` rigid-pose batch."""
    array = np.asarray(poses, dtype=float)
    if array.ndim != 3 or array.shape[1:] != (4, 4):
        raise CharacteristicsError(f"{label} must have shape (samples, 4, 4)")
    if not np.isfinite(array).all():
        raise CharacteristicsError(f"{label} contains non-finite values")
    return array


def _as_vector(vector: Any, label: str) -> np.ndarray:
    array = np.asarray(vector, dtype=float)
    if array.shape != (3,):
        raise CharacteristicsError(f"{label} must be a three-vector")
    if not np.isfinite(array).all():
        raise CharacteristicsError(f"{label} contains non-finite values")
    return array


def frame_position(
    poses: Any,
    reference: Any,
    *,
    origin: Any | None = None,
) -> np.ndarray:
    """
    Express a frame's origin in a reference frame, shape ``(samples, 3)``.

    With no ``origin`` the frame's own point is used.  The subtraction happens
    after both poses are in world coordinates, so the result is a genuine
    relative position rather than a world position relabelled.
    """
    frame_poses = _as_pose_batch(poses, "frame poses")
    reference_poses = _as_pose_batch(reference, "reference poses")
    if frame_poses.shape != reference_poses.shape:
        raise CharacteristicsError("frame and reference must share a sample count")
    point = (
        frame_poses[:, :3, 3]
        if origin is None
        else np.broadcast_to(_as_vector(origin, "origin"), (frame_poses.shape[0], 3))
    )
    # Express the reference basis as a rotation of the world offset.
    return np.einsum("nji,nj->ni", reference_poses[:, :3, :3], point - reference_poses[:, :3, 3])


def _axis_in_reference(poses: Any, reference: Any, axis: Any) -> np.ndarray:
    """Resolve a body-local axis into the reference frame, shape ``(samples, 3)``."""
    frame_poses = _as_pose_batch(poses, "frame poses")
    reference_poses = _as_pose_batch(reference, "reference poses")
    local = _as_vector(axis, "axis")
    resolved = np.einsum("nij,j->ni", frame_poses[:, :3, :3], local)
    return np.einsum("nji,nj->ni", reference_poses[:, :3, :3], resolved)


def toe_angle(poses: Any, reference: Any, axis: Any, *, sign: int = 1) -> np.ndarray:
    """
    Toe angle in radians, shape ``(samples,)``.

    Toe is the rotation of the wheel axis out of the reference frame's
    longitudinal direction, measured in the reference frame's horizontal plane.
    A wheel whose axis has no horizontal projection has no defined toe and is
    refused rather than reported as straight ahead.
    """
    vector = _axis_in_reference(poses, reference, axis)
    horizontal = np.hypot(vector[:, 0], vector[:, 1])
    degenerate = horizontal <= 1e-12
    if degenerate.any():
        raise CharacteristicsError(
            f"toe is undefined at {int(degenerate.sum())} sample(s): the wheel axis has "
            "no horizontal projection"
        )
    return np.arctan2(-vector[:, 0], vector[:, 1]) * sign


def camber_angle(poses: Any, reference: Any, axis: Any, *, sign: int = 1) -> np.ndarray:
    """
    Camber angle in radians, shape ``(samples,)``.

    Camber is the inclination of the wheel axis away from the reference frame's
    horizontal plane.  Unlike toe this stays defined for a vertical axis (it is
    then +/-pi/2), so only a zero-length axis is degenerate.
    """
    vector = _axis_in_reference(poses, reference, axis)
    norm = np.linalg.norm(vector, axis=1)
    degenerate = norm <= 1e-12
    if degenerate.any():
        raise CharacteristicsError(
            f"camber is undefined at {int(degenerate.sum())} sample(s): the wheel axis "
            "has no length"
        )
    return np.arctan2(vector[:, 2], np.hypot(vector[:, 0], vector[:, 1])) * sign


def wheel_center(poses: Any, reference: Any) -> np.ndarray:
    """Return a wheel-centre position in the reference frame; the named alias of `frame_position`."""
    return frame_position(poses, reference)


# --------------------------------------------------------------------------- #
# the legacy K&C angle definitions
# --------------------------------------------------------------------------- #
#
# These are *not* the same formulas as ``camber_angle`` / ``toe_angle`` above, and
# the difference is not a rounding detail.  The unified camber measures the spin
# axis against the whole horizontal plane, ``atan2(z, hypot(x, y))``, while the
# legacy K&C output measured it against the lateral axis alone, ``atan2(z, y)``.
# The two agree only when the wheel has no toe: at 9.8 degrees of toe they differ
# by 0.085 degrees, which is far larger than any tolerance in the K&C tables.
#
# Unifying them by substitution would therefore silently move every stored camber
# and toe value.  Both definitions are kept, named, and separately callable --
# the same treatment ``roll_center_legacy_v1`` gets -- so the formula lives in one
# place while the published numbers keep their meaning.


def legacy_wheel_camber(rotation: Any, *, outward: float) -> float:
    """
    Return the legacy K&C camber in degrees from an upright rotation matrix.

    ``rotation`` is the upright pose's 3x3 basis; the spin axis is its second
    column, and the angle is ``atan2(axis_z, axis_y)``.  ``outward`` carries the
    left/right sign convention.
    """
    matrix = _as_rotation(rotation)
    return -outward * float(np.degrees(np.arctan2(float(matrix[2, 1]), float(matrix[1, 1]))))


def legacy_wheel_toe(rotation: Any, *, outward: float) -> float:
    """
    Return the legacy K&C toe in degrees from an upright rotation matrix.

    Uses ``atan2(axis_x, axis_y)``, matching the historical K&C table rather than
    the unified ``toe_angle`` operator.
    """
    matrix = _as_rotation(rotation)
    return -outward * float(np.degrees(np.arctan2(float(matrix[0, 1]), float(matrix[1, 1]))))


def legacy_lateral_axis_y(rotation: Any) -> float:
    """
    Return ``R[1, 1]``, the shared denominator of both legacy angles.

    Exposed because a caller dividing by it needs to know when it is degenerate;
    ``atan2`` handles a zero denominator gracefully, so this deliberately returns
    the number rather than raising.
    """
    return float(_as_rotation(rotation)[1, 1])


def transformed_point(
    rotation: Any,
    origin: Any,
    local: Any,
    *,
    axis: int,
) -> float:
    """
    Return one axis of ``origin + rotation @ local``.

    The wheel-centre output is this expression, so naming it here removes the
    third copy of the same three-matrix product from the output layer.
    """
    matrix = _as_rotation(rotation)
    centre = np.asarray(origin, dtype=float).ravel()
    offset = np.asarray(local, dtype=float).ravel()
    if centre.shape != (3,) or offset.shape != (3,):
        raise CharacteristicsError("the origin and local offset must be three-vectors")
    if axis not in (0, 1, 2):
        raise CharacteristicsError(f"unknown component index {axis!r}; use 0, 1 or 2")
    return float((centre + matrix @ offset)[axis])


def _as_rotation(rotation: Any) -> np.ndarray:
    """Validate a 3x3 rotation matrix."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3):
        raise CharacteristicsError("a rotation must be a 3x3 matrix")
    if not np.isfinite(matrix).all():
        raise CharacteristicsError("a rotation contains non-finite values")
    return matrix


def axis_direction(poses: Any, axis: Any) -> np.ndarray:
    """Resolve a body-local axis in world coordinates, shape ``(samples, 3)``."""
    frame_poses = _as_pose_batch(poses, "frame poses")
    return np.einsum("nij,j->ni", frame_poses[:, :3, :3], _as_vector(axis, "axis"))


def direction_cosine(poses: Any, reference: Any, axis: Any, other: Any) -> np.ndarray:
    """
    Cosine between two axes, both resolved into the reference frame.

    Useful where a consumer wants an angle that is not toe or camber (a kingpin
    inclination, say) without a new bespoke formula in the consumer.
    """
    first = _axis_in_reference(poses, reference, axis)
    second = np.broadcast_to(
        _axis_in_reference(reference, reference, other), first.shape
    )
    denominator = np.linalg.norm(first, axis=1) * np.linalg.norm(second, axis=1)
    degenerate = denominator <= 1e-12
    if degenerate.any():
        raise CharacteristicsError("direction cosine is undefined for a zero-length axis")
    return np.einsum("ni,ni->n", first, second) / denominator


def measurement_derivative(
    values: Any,
    inputs: Any,
    *,
    numerator_units: str,
    denominator_units: str,
) -> np.ndarray:
    """
    Differentiate per-sample values with respect to per-sample inputs.

    This is a *definition* helper for expressions whose derivative is known in
    closed form, used to keep a chain rule in one place.  It is deliberately not
    a licence to fall back on differences: a solver response is requested from
    the native side, and a finite difference belongs in a test oracle.
    """
    output = np.asarray(values, dtype=float)
    driving = np.asarray(inputs, dtype=float)
    if output.shape != driving.shape:
        raise CharacteristicsError(
            "a derivative needs the measured and driving sequences to share a shape"
        )
    if output.ndim != 1:
        raise CharacteristicsError("measurement derivatives are defined per sample")
    if not numerator_units or not denominator_units:
        raise CharacteristicsError("derivative units require both sides")
    if output.size == 1:
        raise CharacteristicsError(
            "a derivative needs at least two samples; a single point has no slope"
        )
    return np.gradient(output, driving)


__all__ = [
    "ANGLE_UNITS",
    "LENGTH_UNITS",
    "axis_direction",
    "camber_angle",
    "direction_cosine",
    "frame_position",
    "legacy_lateral_axis_y",
    "legacy_wheel_camber",
    "legacy_wheel_toe",
    "measurement_derivative",
    "toe_angle",
    "transformed_point",
    "wheel_center",
]
