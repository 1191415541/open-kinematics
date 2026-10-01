"""
Static vehicle-level contact loads.

This is a *vehicle-level derived quantity*, not a kernel solve.  It takes the
assembled vehicle's total mass, centre of mass and its own contact points, writes
the three rigid-body balance equations (vertical force, pitch, roll) and returns
the minimum-norm member of the resulting N-reaction family.  It never touches the
multibody constraint system, so it is not something the native kernel could
answer: the native static solver factors a *square* system, while these N
unknowns against three equations are usually underdetermined and need the
minimum-norm convention chosen here.

The contact points are the *assembly's*: every wheel end the vehicle declares
contributes its wheel centre, brought onto the road plane, and N is however many
wheel ends there are.  Nothing here reads a corner-name constant, so a two-axle
car's four points, a three-axle truck's six and a corner bench's one are one code
path with different data.

Two judgements are kept apart here, because they are different questions:

* **Does a solution exist?**  Three balance equations in N unknowns exist a
  solution exactly when the loads are *compatible* with the contact geometry, and
  that is what the residual ``||A x - b||`` of the least-squares solution measures.
  The residual is compared against a tolerance proportional to the load being
  balanced (see :data:`_RESIDUAL_RELATIVE_TOLERANCE`): a compatible state leaves
  rounding only, an incompatible one leaves a residual of the order of the load
  itself, so the two are many orders of magnitude apart and no
  point-count or rank threshold is involved.
* **Is it unique?**  ``rank(A) == N`` is unique; ``rank(A) < N`` is a family of
  solutions, and the minimum-norm member is returned with ``unique = False``
  marked on the result.  A four-wheel vehicle is the ordinary case of the second
  kind -- three equations, four reactions -- while a single contact point directly
  below the centre of mass with no horizontal acceleration is the first, where
  ``rank(A) = 1 = N``.

``rank(A) < 3`` says only that the three balance equations are not independent,
which is a statement about the constraint capability of the contact set and
nothing about existence or uniqueness.  It is therefore not an error here: a
single contact point has a rank-one matrix and is a perfectly good reading
whenever the loads are compatible.

It lives in ``vehicle/`` because that is the layer that owns whole-vehicle
services and is already allowed to import ``subsystems`` -- the assembly is its
only input.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..report.wheel_loads import WheelLoadSummary, summarize_wheel_loads
from ..schema import VehicleModel
from ..subsystems.vehicle_assembly import VehicleRuntime, compose_vehicle_runtime

#: How large a residual the three balance equations may leave, as a fraction of the
#: vertical load they are balancing.  The equations are stated over force, so their
#: residual is a force: for a 15 kN vehicle the tolerance is 1.5e-5 N, which is six
#: orders above the rounding floor (the loads are O(1e7) N summed over a handful of
#: points, i.e. ~1e-8 N of accumulated double-precision error) and eight orders below
#: the residual an incompatible load state leaves, which is O(1) of the load itself.
#: The scale is the load, not the solution, so the same number means the same thing
#: whatever the vehicle weighs and however many contact points it has.
_RESIDUAL_RELATIVE_TOLERANCE = 1e-9

#: The smallest scale the relative tolerance is applied to, in N.  A load-free model
#: would otherwise get a zero tolerance and judge its own rounding as incompatible.
_MINIMUM_LOAD_SCALE = 1.0


class IncompatibleStaticLoadsError(ValueError):
    """
    The loads cannot be balanced by vertical reactions at the contact points.

    Named, and carrying the measured residual, the tolerance it was judged against
    and the number of contact points, because those three numbers are what tells a
    caller *why* the state is incompatible.  The rank of the balance matrix is
    deliberately not part of this message: a rank says whether the equations are
    independent, never whether the loads can be carried.

    It is a ``ValueError`` so that a caller which treated the old refusal as one
    keeps working unchanged.
    """

    def __init__(self, *, residual: float, tolerance: float, contact_points: int) -> None:
        self.residual = float(residual)
        self.tolerance = float(tolerance)
        self.contact_points = int(contact_points)
        super().__init__(
            f"the static loads are incompatible with the {self.contact_points} contact "
            f"point(s) of this vehicle: the residual of the three equilibrium equations "
            f"is {self.residual!r} and the tolerance is {self.tolerance!r}; no set of "
            "vertical reactions at those points balances the force and the moments"
        )


@dataclass(frozen=True)
class StaticWheelLoadResult:
    """
    Quasi-static vertical support reactions, one per contact point.

    ``wheel_loads`` maps each wheel end's own name to its reaction, ``rank`` and
    ``residual`` are the balance matrix's rank and the residual the returned loads
    leave, ``unique`` records whether ``rank`` reached the number of contact points
    (a four-wheel vehicle does not, and is therefore a marked minimum-norm member of
    a family), and ``residual_tolerance`` is the bound ``residual`` was judged
    against.
    """

    wheel_loads: dict[str, float]
    total_mass: float
    center_of_mass: np.ndarray
    support_points: dict[str, np.ndarray]
    rank: int
    residual: float
    unique: bool
    residual_tolerance: float

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

    The vehicle is composed and its own contact points are read from the result;
    this function is the ``VehicleModel``-shaped door onto
    :func:`compute_static_wheel_loads_for_assembly`, which is where the general
    N-contact-point solve lives.
    """
    _validated_acceleration(acceleration, gravity)
    # The vehicle is built through the composition layer, which is the same
    # construction every other reading uses.  This module reads `total_mass`,
    # `center_of_mass` and the wheel centres, and the composed runtime offers all
    # three -- so there is no reason for it to reach for the historical builder and
    # keep that path alive on its own.
    assembly = compose_vehicle_runtime(vehicle, mode="K")
    return compute_static_wheel_loads_for_assembly(
        assembly,
        acceleration=acceleration,
        gravity=gravity,
        road_z=road_z,
    )


def compute_static_wheel_loads_for_assembly(
    assembly: VehicleRuntime,
    *,
    acceleration: np.ndarray | None = None,
    gravity: float = 9810.0,
    road_z: float = 0.0,
) -> StaticWheelLoadResult:
    """
    Solve the N-contact-point static equilibrium of an assembled vehicle.

    Every wheel end the assembly carries is one unknown, the three balance
    equations are the rows, and the solution is load-compatible (``residual``
    within ``residual_tolerance``) and marked ``unique`` when ``rank(A) == N``.
    Loads no vertical-reaction set can balance raise
    :class:`IncompatibleStaticLoadsError` naming the measured residual, the
    tolerance and the contact-point count.

    An assembly is taken rather than a model because that is the form which
    carries contact points for *any* topology: a ``VehicleModel`` names two axles,
    while an entry-built vehicle names as many as it declares.
    """
    accel = _validated_acceleration(acceleration, gravity)
    support_points = _support_points(assembly, road_z)
    total_mass = assembly.total_mass
    center_of_mass = _center_of_mass(assembly, total_mass)
    names = tuple(support_points)
    count = len(names)
    height = center_of_mass[2] - road_z
    # The three rigid-body balance rows: vertical force, pitch moment and roll
    # moment, each vertical reaction entering with its own moment arms about the
    # centre of mass.  The columns are the contact points in the order the assembly
    # lists its wheel ends -- a data order, not a corner-name order.
    matrix = np.array(
        [
            np.ones(count),
            [support_points[name][0] - center_of_mass[0] for name in names],
            [support_points[name][1] - center_of_mass[1] for name in names],
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
    tolerance = _residual_tolerance(total_mass, gravity + accel[2])
    if not residual <= tolerance:
        raise IncompatibleStaticLoadsError(
            residual=residual,
            tolerance=tolerance,
            contact_points=count,
        )
    return StaticWheelLoadResult(
        wheel_loads={name: float(loads[index]) for index, name in enumerate(names)},
        total_mass=total_mass,
        center_of_mass=center_of_mass,
        support_points=support_points,
        rank=int(rank),
        residual=residual,
        # `rank` counts the independent equations the contact set can carry, so
        # reaching the number of unknowns is exactly "the answer is pinned down".
        unique=bool(int(rank) == count),
        residual_tolerance=tolerance,
    )


def _validated_acceleration(
    acceleration: np.ndarray | None, gravity: float
) -> np.ndarray:
    """Return the acceleration as a finite 3-vector, refusing an unusable gravity."""
    if gravity <= 0.0 or not np.isfinite(gravity):
        raise ValueError("gravity must be finite and positive")
    accel = np.zeros(3) if acceleration is None else np.asarray(acceleration, dtype=float)
    if accel.shape != (3,) or not np.all(np.isfinite(accel)):
        raise ValueError("acceleration must contain three finite values")
    return accel


def _residual_tolerance(total_mass: float, vertical_acceleration: float) -> float:
    """
    Return the residual the balance equations may leave before the loads are refused.

    The residual is a force and the equations are stated over force, so it is
    measured against the vertical load the vehicle carries -- the scale the
    equations are actually balancing.  See :data:`_RESIDUAL_RELATIVE_TOLERANCE` for
    the value and its justification.
    """
    load_scale = abs(total_mass * vertical_acceleration)
    return _RESIDUAL_RELATIVE_TOLERANCE * max(load_scale, _MINIMUM_LOAD_SCALE)


def _center_of_mass(assembly: VehicleRuntime, total_mass: float) -> np.ndarray:
    weighted = np.zeros(3)
    for name, body in assembly.bodies.items():
        if body.mass <= 0.0:
            continue
        weighted += body.mass * assembly.state.point_world(name, body.center_of_mass)
    return weighted / total_mass


def _support_points(assembly: VehicleRuntime, road_z: float) -> dict[str, np.ndarray]:
    """
    Return every contact point of the assembly, keyed by the wheel end it belongs to.

    The set and its order are the assembly's own wheel table: each entry carries the
    body that holds the wheel centre and the centre's body-local coordinates, and
    the contact point is that point in the world, brought onto the road plane
    because a vertical reaction acts there.  Reading the table rather than a tuple
    of corner names is what makes the count -- four for a car, six for a three-axle
    truck, one for a corner bench -- a property of the vehicle instead of a constant
    here.
    """
    points: dict[str, np.ndarray] = {}
    for wheel, (body, local_center) in assembly.wheel_centers.items():
        center = assembly.state.point_world(body, local_center)
        points[wheel] = np.array(
            [center[0], center[1], road_z],
            dtype=float,
        )
    return points


__all__ = [
    "IncompatibleStaticLoadsError",
    "StaticWheelLoadResult",
    "compute_static_wheel_loads",
    "compute_static_wheel_loads_for_assembly",
]
