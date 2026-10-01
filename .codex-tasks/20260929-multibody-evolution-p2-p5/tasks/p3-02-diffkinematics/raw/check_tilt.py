"""
How the engine-vs-today difference grows as the arm's real pivot is tilted further.

Geometry B in ``raw/probe_numerics.py`` moves ``UPPER_INBOARD_REAR``.  That hardpoint
is not a joint, but the upper arm's *revolute axis* is built from it together with
``UPPER_INBOARD_FRONT`` (``subsystems/suspension.py:332`` -> ``_local_axes``), so moving
it tilts the real pivot.  This script walks that hardpoint upward and reports, on the
same axis-member reading the comparison table uses:

* the pivot tilt (angle between the arm's real axis and x);
* the engine's front-view intersection for the *whole axle*;
* the same with today's arithmetic but the real pivot;
* today's shipped ``_instant_center``.

Run:  uv run --no-sync python <this file>
"""

from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).parent))

from fixtures import double_wishbone_axle_model, two_d_intersection  # noqa: E402

from suspension_multibody.subsystems.entry import compose_axle  # noqa: E402
from suspension_multibody.vehicle import screw_kinematics as sk  # noqa: E402
from suspension_multibody.vehicle.roll_centers import (  # noqa: E402
    _hardpoint,
    _instant_center,
)

FRONT_VIEW_PLANE_X = 1400.0


def comparison_axis(axle):
    """The engine's upright axis, on the family member closest to a roll about x."""
    bodies = sk.free_bodies(axle.constraints, axle.state)
    motion = sk.solve_rigid_motion(
        axle.constraints,
        axle.state,
        [sk.TangentDrive("upright_L", 3, 1.0)],
        points=axle.points,
        bodies=bodies,
        pivot_body="upright_L",
        pivot_point=("upright_L", "spindle"),
    )
    index = 6 * bodies.index("upright_L")
    pose = axle.state.pose("upright_L")
    velocity = np.asarray(motion.velocity, dtype=float)
    base = pose.rotation @ velocity[index + 3 : index + 6]
    free = np.asarray(motion.null_space, dtype=float)
    sensitivity = np.array(
        [pose.rotation @ free[index + 3 : index + 6, column] for column in range(free.shape[1])]
    ).T
    design = np.column_stack((sensitivity, -np.array([1.0, 0.0, 0.0])))
    solution, *_ = np.linalg.lstsq(design, -base, rcond=None)
    trial = velocity + free @ solution[: free.shape[1]]
    omega = pose.rotation @ trial[index + 3 : index + 6]
    twin = sk.Twist(
        omega=omega,
        velocity=pose.rotation @ trial[index : index + 3] + np.cross(pose.translation, omega),
    )
    screw = sk.screw_from_twist(twin, anchor=motion.pivot_point)
    fraction = float(np.linalg.norm(design @ solution + base) / abs(solution[-1]))
    return screw, fraction


def pierce(axis_point: np.ndarray, direction: np.ndarray, plane_x: float) -> np.ndarray:
    """Where the axis line crosses the plane ``x = plane_x``, as ``[y, z]``."""
    parameter = (plane_x - float(axis_point[0])) / float(direction[0])
    return (np.asarray(axis_point, dtype=float) + parameter * np.asarray(direction, dtype=float))[1:]


def pivot_intersection(model, side: str) -> np.ndarray:
    """Today's arithmetic with each arm line anchored on its real pivot."""
    return two_d_intersection(
        _hardpoint(model, "upper_front", side)[1:],
        _hardpoint(model, "upper_outer", side)[1:],
        _hardpoint(model, "lower_front", side)[1:],
        _hardpoint(model, "lower_outer", side)[1:],
    )


def main() -> None:
    print(f"{'z of UPPER_INBOARD_REAR':>24} {'tilt(deg)':>10} {'fraction':>11} "
          f"{'|engine-pivot|':>15} {'|engine-today|':>15}")
    for z in (500.0, 520.0, 560.0, 650.0, 800.0):
        model = double_wishbone_axle_model(UPPER_INBOARD_REAR=(1700.0, -500.0, z))
        axle = compose_axle(model, "K")
        pivot_axis = next(
            item.axis_a for item in axle.constraints if item.name == "uca_mount_L_inner_front"
        )
        tilt = float(np.degrees(np.arccos(abs(float(pivot_axis[0])))))
        screw, fraction = comparison_axis(axle)
        engine = pierce(screw.point, screw.direction, FRONT_VIEW_PLANE_X)
        pivot = pivot_intersection(model, "L")
        today = _instant_center(model, "L")
        print(f"{z:>24.1f} {tilt:>10.3f} {fraction:>11.3e} "
              f"{np.linalg.norm(engine - pivot):>15.3e} {np.linalg.norm(engine - today):>15.3e}")


if __name__ == "__main__":
    main()
