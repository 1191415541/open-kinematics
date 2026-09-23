"""
Assembly-time joint availability checks.

A joint that needs an axis but is handed a degenerate one is a *silently* wrong
model, not an error: the kernel treats every axis as optional and substitutes a
default, so a zero-length axis produces a plausible-looking run that answers a
different question.  The check therefore belongs at assembly time, where the
geometry is known and the message can name the joint and the axis.

`joints.table` says which axes each type needs; this module applies that to the
geometry the assembly already carries.
"""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np

from .table import JointTableError, required_axis_fields

__all__ = ["JointAvailabilityError", "validate_joint_axes"]

#: An axis shorter than this is treated as degenerate.  The value is a
#: normalised-vector tolerance, not a length unit: axes arrive normalised, so a
#: well-formed axis has length 1 and a zero vector has length 0.
_AXIS_TOLERANCE = 1e-9


class JointAvailabilityError(ValueError):
    """A joint cannot be encoded: its geometry does not define a needed axis."""


def validate_joint_axes(
    *,
    joint_name: str,
    kernel_name: str,
    axes: Mapping[str, object],
) -> None:
    """
    Reject a joint whose declared axes are missing or degenerate.

    `axes` maps a document axis key (`axis_a`, `axis_a_secondary`, ...) to the
    axis the assembly produced, in any body frame; only its length is inspected.
    Raises `JointAvailabilityError` naming the joint, its type, and the offending
    key -- a bare "unsupported joint" would not tell the author which hardpoint to
    fix.
    """
    try:
        required = required_axis_fields(kernel_name)
    except JointTableError as exc:
        raise JointAvailabilityError(
            f"joint {joint_name!r} has unknown type {kernel_name!r}"
        ) from exc

    for field in required:
        if field not in axes or axes[field] is None:
            raise JointAvailabilityError(
                f"joint {joint_name!r} of type {kernel_name!r} requires {field!r}, "
                "but the assembly produced no axis for it; check the hardpoints "
                "that define this joint's axis"
            )
        values = np.asarray(axes[field], dtype=float).reshape(-1)
        if values.size < 3 or not np.all(np.isfinite(values[:3])):
            raise JointAvailabilityError(
                f"joint {joint_name!r} of type {kernel_name!r} has a non-finite "
                f"{field!r}"
            )
        length = float(np.linalg.norm(values[:3]))
        if length < _AXIS_TOLERANCE:
            raise JointAvailabilityError(
                f"joint {joint_name!r} of type {kernel_name!r} has a degenerate "
                f"{field!r} (zero-length axis); the two hardpoints that define it "
                "coincide, so the joint's direction is undefined"
            )
