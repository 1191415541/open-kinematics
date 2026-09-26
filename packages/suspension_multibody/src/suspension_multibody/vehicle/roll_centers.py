"""
Front-view roll-centre geometry per axle.

A vehicle-level geometric construction, not a solve: it intersects the two
control-arm lines of each side in the front view to get the side's instant
centre, then intersects the contact-patch-to-instant-centre lines of the two
sides.  Nothing here evaluates a force law.

It sits in ``vehicle/`` for the same reason ``static_loads`` does: the
construction reads the assembled vehicle, and ``vehicle/`` is the layer allowed
to import ``preparation``.  ``report/`` is not -- putting it there would add a
``report -> preparation`` edge the report boundary forbids.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..preparation.assembly import build_vehicle
from ..preparation.assembly.front_axle import side_hardpoints
from ..schema import FrontAxleModel, VehicleModel


@dataclass(frozen=True)
class RollCenterResult:
    """Front-view roll-center geometry for one axle."""

    axle: str
    center: np.ndarray
    left_instant_center: np.ndarray
    right_instant_center: np.ndarray


def compute_vehicle_roll_centers(
    vehicle: VehicleModel,
    *,
    road_z: float = 0.0,
) -> dict[str, RollCenterResult]:
    """Compute front-view roll centers from the four double-wishbone arms."""
    assembly = build_vehicle(vehicle, mode="K")
    results: dict[str, RollCenterResult] = {}
    for axle_name, axle in (("front", vehicle.front_axle), ("rear", vehicle.rear_axle)):
        left_ic = _instant_center(axle, "L")
        right_ic = _instant_center(axle, "R")
        left_contact = _contact_front_view(axle, "L", vehicle, axle_name, road_z)
        right_contact = _contact_front_view(axle, "R", vehicle, axle_name, road_z)
        center = _line_intersection(left_contact, left_ic, right_contact, right_ic)
        if center is None:
            raise ValueError(f"{axle_name} roll-center lines are parallel")
        results[axle_name] = RollCenterResult(
            axle=axle_name,
            center=center,
            left_instant_center=left_ic,
            right_instant_center=right_ic,
        )
    del assembly
    return results


_POINT_ALIASES: dict[str, tuple[str, ...]] = {
    "upper_front": ("UPPER_INBOARD_FRONT", "UPPER_INNER_FRONT", "UCA_FRONT"),
    "upper_rear": ("UPPER_INBOARD_REAR", "UPPER_INNER_REAR", "UCA_REAR"),
    "upper_outer": ("UPPER_OUTBOARD", "UPPER_OUTER", "UCA_OUTER"),
    "lower_front": ("LOWER_INBOARD_FRONT", "LOWER_INNER_FRONT", "LCA_FRONT"),
    "lower_rear": ("LOWER_INBOARD_REAR", "LOWER_INNER_REAR", "LCA_REAR"),
    "lower_outer": ("LOWER_OUTBOARD", "LOWER_OUTER", "LCA_OUTER"),
    "wheel_center": ("WHEEL_CENTER", "WHEEL_CENTRE", "WHEEL_CG"),
}


def _hardpoint(axle: FrontAxleModel, role: str, side: str) -> np.ndarray:
    side_points = side_hardpoints(axle.hardpoints, side)  # type: ignore[arg-type]
    normalized = {
        key.upper().replace("-", "_"): value for key, value in side_points.items()
    }
    for alias in _POINT_ALIASES[role]:
        if alias in normalized:
            return normalized[alias].as_array()
    raise ValueError(f"missing hardpoint for roll-center role {role}")


def _instant_center(axle: FrontAxleModel, side: str) -> np.ndarray:
    upper_inner = 0.5 * (
        _hardpoint(axle, "upper_front", side) + _hardpoint(axle, "upper_rear", side)
    )
    lower_inner = 0.5 * (
        _hardpoint(axle, "lower_front", side) + _hardpoint(axle, "lower_rear", side)
    )
    upper_outer = _hardpoint(axle, "upper_outer", side)
    lower_outer = _hardpoint(axle, "lower_outer", side)
    upper_line = np.array(
        [[upper_inner[1], upper_inner[2]], [upper_outer[1], upper_outer[2]]]
    )
    lower_line = np.array(
        [[lower_inner[1], lower_inner[2]], [lower_outer[1], lower_outer[2]]]
    )
    intersection = _line_intersection(*upper_line, *lower_line)
    if intersection is None:
        raise ValueError(f"{side} suspension arm lines are parallel")
    return intersection


def _contact_front_view(
    axle: FrontAxleModel,
    side: str,
    vehicle: VehicleModel,
    axle_name: str,
    road_z: float,
) -> np.ndarray:
    point = _hardpoint(axle, "wheel_center", side)
    # The suspension roll-center construction uses the road contact patch,
    # not the unloaded tire-circle point.  Tire compression changes the wheel
    # center height but does not move the flat road plane used by this geometry.
    del vehicle, axle_name
    return np.array([point[1], road_z], dtype=float)


def _line_intersection(
    point_a: np.ndarray,
    point_b: np.ndarray,
    point_c: np.ndarray,
    point_d: np.ndarray,
) -> np.ndarray | None:
    direction_a = np.asarray(point_b, dtype=float) - np.asarray(point_a, dtype=float)
    direction_b = np.asarray(point_d, dtype=float) - np.asarray(point_c, dtype=float)
    matrix = np.column_stack((direction_a, -direction_b))
    if abs(float(np.linalg.det(matrix))) <= 1e-12:
        return None
    parameters = np.linalg.solve(matrix, np.asarray(point_c) - np.asarray(point_a))
    return np.asarray(point_a, dtype=float) + parameters[0] * direction_a


__all__ = ["RollCenterResult", "compute_vehicle_roll_centers"]
