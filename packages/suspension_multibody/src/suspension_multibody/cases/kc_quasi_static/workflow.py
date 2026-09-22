"""Native K&C contract workflow."""

from __future__ import annotations

import numpy as np

from ...axle_dynamics import AxleSolverSettings
from ...report.geometry import _wheel_geometry
from .convert import MM, NativeKcError, quaternion_to_rotation

SIDES = ("L", "R")
AXIS_ORDER = ("fx", "fy", "fz", "mx", "my", "mz")
DEFAULT_TIMES = tuple(float(t) for t in np.linspace(0.0, 2e-3, 9))
DEFAULT_SETTINGS = AxleSolverSettings(internal_step_s=2.5e-4)


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two scalar-first quaternions."""
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate (inverse, for unit quaternions) of a scalar-first quaternion."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def wheel_center_world(assembly, side: str, position_m, quaternion) -> np.ndarray:
    """World position of the wheel-centre marker for one side (metres)."""
    rotation = quaternion_to_rotation(quaternion)
    local = np.asarray(assembly.point(f"upright_{side}", "wheel_center"), dtype=float) * MM
    return np.asarray(position_m, dtype=float) + rotation @ local

def _side_fields(assembly, side: str, position_m, quaternion) -> dict[str, float]:
    rotation = quaternion_to_rotation(quaternion)
    local = np.asarray(assembly.point(f"upright_{side}", "wheel_center"), dtype=float)
    geometry = _wheel_geometry(
        np.asarray(position_m, dtype=float) / MM,
        rotation,
        local,
        side=side,
    )
    name = "left" if side == "L" else "right"
    return {
        f"{name}_wheel_center_x_mm": float(geometry.center[0]),
        f"{name}_wheel_center_y_mm": float(geometry.center[1]),
        f"{name}_wheel_center_z_mm": float(geometry.center[2]),
        f"{name}_camber_deg": geometry.camber_deg,
        f"{name}_toe_deg": geometry.toe_deg,
    }



def _assembling_pose(model_document, side):
    """Return the pose the model document declares for one upright."""
    for body in model_document["bodies"]:
        if body["name"] == f"upright_{side}":
            position = np.asarray(body["position"], dtype=float) * MM
            quaternion = np.asarray(body["quaternion"], dtype=float)
            return position, quaternion
    raise NativeKcError(f"the model document has no upright_{side}")
