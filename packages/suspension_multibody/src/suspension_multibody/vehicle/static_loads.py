"""
Static vehicle-level wheel loads.

This is a *vehicle-level derived quantity*, not a kernel solve.  It takes the
assembled vehicle's total mass, centre of mass and four support points, writes
the three rigid-body balance equations (vertical force, pitch, roll) and returns
the minimum-norm member of the resulting four-reaction family.  It never touches
the multibody constraint system, so it is not something the native kernel could
answer: the native static solver factors a *square* system, while these four
unknowns against three equations are underdetermined on purpose and need the
minimum-norm convention chosen here.

It lives in ``vehicle/`` because that is the layer that owns whole-vehicle
services and is already allowed to import ``preparation`` -- the assembly is its
only input.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..preparation.assembly import VehicleAssembly, build_vehicle
from ..report.wheel_loads import WheelLoadSummary, summarize_wheel_loads
from ..schema import VehicleModel

_WHEELS = ("front_left", "front_right", "rear_left", "rear_right")


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


__all__ = ["StaticWheelLoadResult", "compute_static_wheel_loads"]
