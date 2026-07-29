"""K&C geometry metrics."""

from __future__ import annotations

import math

from ..core.rigid_body import RigidBodyState
from ..model import FrontAxleAssembly


def _angle_deg(value: float) -> float:
    return math.degrees(math.atan2(value, 1.0))


def wheel_metrics(
    state: RigidBodyState, assembly: FrontAxleAssembly, side: str
) -> dict[str, float]:
    """Compute wheel-center and orientation metrics for one vehicle side."""
    normalized = side.upper()
    side_name = "left" if normalized == "L" else "right"
    body = f"upright_{normalized}"
    rotation = state.pose(body).rotation
    center = state.point_world(body, assembly.point(body, "wheel_center"))
    outward = -1.0 if normalized == "L" else 1.0
    camber = outward * _angle_deg(rotation[1, 2] / max(abs(rotation[2, 2]), 1e-12))
    toe = outward * _angle_deg(rotation[1, 0] / max(abs(rotation[0, 0]), 1e-12))
    return {
        f"{side_name}_wheel_center_x": float(center[0]),
        f"{side_name}_wheel_center_y": float(center[1]),
        f"{side_name}_wheel_center_z": float(center[2]),
        f"{side_name}_camber_deg": float(camber),
        f"{side_name}_toe_deg": float(toe),
    }


def compute_k_metrics(
    state: RigidBodyState, assembly: FrontAxleAssembly
) -> dict[str, float]:
    """Compute symmetric K&C metrics and vehicle-side sign conventions."""
    metrics = {}
    metrics.update(wheel_metrics(state, assembly, "L"))
    metrics.update(wheel_metrics(state, assembly, "R"))
    metrics["track_mm"] = (
        metrics["right_wheel_center_y"] - metrics["left_wheel_center_y"]
    )
    metrics["wheel_center_z_mean"] = 0.5 * (
        metrics["left_wheel_center_z"] + metrics["right_wheel_center_z"]
    )
    metrics["wheel_center_z_difference"] = (
        metrics["right_wheel_center_z"] - metrics["left_wheel_center_z"]
    )
    metrics["camber_deg_difference"] = (
        metrics["right_camber_deg"] - metrics["left_camber_deg"]
    )
    metrics["toe_deg_difference"] = metrics["right_toe_deg"] - metrics["left_toe_deg"]
    return metrics
