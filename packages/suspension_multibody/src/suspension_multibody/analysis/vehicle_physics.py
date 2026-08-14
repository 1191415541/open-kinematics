"""Self-consistency diagnostics for full-vehicle physical behavior."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

from ..model import VehicleAssembly, build_vehicle
from ..model.front_axle import side_hardpoints
from ..schema import FrontAxleModel, VehicleModel

_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


@dataclass(frozen=True)
class WheelLoadSummary:
    """Aggregated wheel normal loads with explicit sign conventions."""

    wheel_loads: dict[str, float]
    total: float
    front_axle: float
    rear_axle: float
    left_side: float
    right_side: float
    front_rear_delta: float
    right_left_delta: float


@dataclass(frozen=True)
class StaticWheelLoadResult:
    """Quasi-static vertical support reactions for the four contact points."""

    wheel_loads: dict[str, float]
    total_mass: float
    center_of_mass: np.ndarray
    support_points: dict[str, np.ndarray]
    rank: int
    residual: float

    @property
    def summary(self) -> WheelLoadSummary:
        return summarize_wheel_loads(self.wheel_loads)


@dataclass(frozen=True)
class RollCenterResult:
    """Front-view roll-center geometry for one axle."""

    axle: str
    center: np.ndarray
    left_instant_center: np.ndarray
    right_instant_center: np.ndarray


def summarize_wheel_loads(loads: Mapping[str, float]) -> WheelLoadSummary:
    """Aggregate four wheel loads; ``front_rear_delta`` is front minus rear."""
    missing = set(_WHEELS) - set(loads)
    if missing or set(loads) - set(_WHEELS):
        raise ValueError("wheel loads must contain exactly the four vehicle corners")
    values = {name: float(loads[name]) for name in _WHEELS}
    if any(not np.isfinite(value) for value in values.values()):
        raise ValueError("wheel loads must be finite")
    front = values["front_left"] + values["front_right"]
    rear = values["rear_left"] + values["rear_right"]
    left = values["front_left"] + values["rear_left"]
    right = values["front_right"] + values["rear_right"]
    return WheelLoadSummary(
        wheel_loads=values,
        total=front + rear,
        front_axle=front,
        rear_axle=rear,
        left_side=left,
        right_side=right,
        front_rear_delta=front - rear,
        right_left_delta=right - left,
    )


def wheel_load_metrics(loads: Mapping[str, float]) -> dict[str, float]:
    """Return stable result-channel names for wheel-load diagnostics."""
    summary = summarize_wheel_loads(loads)
    metrics = {f"normal_load_{name}": value for name, value in summary.wheel_loads.items()}
    metrics.update(
        {
            "normal_load_total": summary.total,
            "normal_load_front_axle": summary.front_axle,
            "normal_load_rear_axle": summary.rear_axle,
            "normal_load_left_side": summary.left_side,
            "normal_load_right_side": summary.right_side,
            "load_transfer_front_minus_rear": summary.front_rear_delta,
            "load_transfer_right_minus_left": summary.right_left_delta,
        }
    )
    return metrics


def compute_static_wheel_loads(
    vehicle: VehicleModel,
    *,
    acceleration: np.ndarray | None = None,
    gravity: float = 9810.0,
    road_z: float = 0.0,
) -> StaticWheelLoadResult:
    """
    Solve vertical support reactions from force and moment balance.

    The horizontal acceleration convention is vehicle-frame ``(+x forward,
    +y toward the positive-y wheel side, +z upward)``.  Longitudinal and
    lateral tire forces are assumed to act at the road plane, which gives the
    textbook height-over-wheelbase and height-over-track transfer terms.
    The four vertical reactions are otherwise underdetermined; the minimum
    norm solution is returned and is unique for a symmetric four-corner layout.
    """
    if gravity <= 0.0 or not np.isfinite(gravity):
        raise ValueError("gravity must be finite and positive")
    accel = np.zeros(3) if acceleration is None else np.asarray(acceleration, dtype=float)
    if accel.shape != (3,) or not np.all(np.isfinite(accel)):
        raise ValueError("acceleration must contain three finite values")
    assembly = build_vehicle(vehicle, mode="K")
    support_points = _support_points(vehicle, assembly, road_z)
    total_mass = assembly.total_mass
    center_of_mass = _center_of_mass(assembly, total_mass)
    height = center_of_mass[2] - road_z
    matrix = np.array(
        [
            np.ones(4),
            [support_points[name][0] - center_of_mass[0] for name in _WHEELS],
            [support_points[name][1] - center_of_mass[1] for name in _WHEELS],
        ],
        dtype=float,
    )
    rhs = np.array(
        [
            total_mass * (gravity + accel[2]),
            -total_mass * height * accel[0],
            -total_mass * height * accel[1],
        ],
        dtype=float,
    )
    loads, _, rank, _ = np.linalg.lstsq(matrix, rhs, rcond=1e-12)
    residual = float(np.max(np.abs(matrix @ loads - rhs)))
    if rank < 3:
        raise ValueError("four wheel support points do not span force/moment balance")
    return StaticWheelLoadResult(
        wheel_loads={name: float(loads[index]) for index, name in enumerate(_WHEELS)},
        total_mass=total_mass,
        center_of_mass=center_of_mass,
        support_points=support_points,
        rank=int(rank),
        residual=residual,
    )


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


def _center_of_mass(assembly: VehicleAssembly, total_mass: float) -> np.ndarray:
    weighted = np.zeros(3)
    for name, body in assembly.bodies.items():
        if body.mass <= 0.0:
            continue
        weighted += body.mass * assembly.state.point_world(name, body.center_of_mass)
    return weighted / total_mass


def _support_points(
    vehicle: VehicleModel, assembly: VehicleAssembly, road_z: float
) -> dict[str, np.ndarray]:
    points: dict[str, np.ndarray] = {}
    for wheel in vehicle.wheels:
        upright, local_center = assembly.wheel_centers[wheel.name]
        center = assembly.state.point_world(upright, local_center)
        points[wheel.name] = np.array(
            [center[0], center[1], road_z],
            dtype=float,
        )
    return points


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
