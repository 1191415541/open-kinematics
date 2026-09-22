"""
Shared wheel-centre and wheel-angle geometry primitives.

Moved here from ``analysis/_geometry.py``, which 08 deletes.  The caller passes
the pose and the local wheel centre it already has; this is the reporting
convention, not an input transformation, so it does not run preparation.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class _WheelGeometry:
    """Wheel geometry in the caller's position units and degrees."""

    center: np.ndarray
    camber_deg: float
    toe_deg: float


def _wheel_geometry(
    position,
    rotation,
    wheel_center_local,
    *,
    side: str,
) -> _WheelGeometry:
    """
    Compute shared wheel-centre, camber, and toe conventions.

    ``position`` and ``wheel_center_local`` must use the same length unit.  The
    helper intentionally mirrors the established upright-local lateral-axis
    convention: the wheel angles are measured from the rotated local Y axis and
    use the existing left/right outward sign.
    """
    normalized = side.strip().lower()
    if normalized not in {"l", "left", "r", "right"}:
        raise ValueError(f"unknown wheel side {side!r}")
    outward = -1.0 if normalized in {"l", "left"} else 1.0
    matrix = np.asarray(rotation, dtype=float)
    origin = np.asarray(position, dtype=float)
    local = np.asarray(wheel_center_local, dtype=float)
    center = origin + matrix @ local
    lateral_axis_y = float(matrix[1, 1])
    camber_deg = -outward * float(
        np.degrees(np.arctan2(float(matrix[2, 1]), lateral_axis_y))
    )
    toe_deg = -outward * float(
        np.degrees(np.arctan2(float(matrix[0, 1]), lateral_axis_y))
    )
    return _WheelGeometry(
        center=center,
        camber_deg=camber_deg,
        toe_deg=toe_deg,
    )
