"""
The N-contact-point cases, printed with their numbers.

Run from the repository root:

    uv run --no-sync python \
      .codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-04-static-loads/raw/n_point_cases_probe.py

The three cases the subtask is judged on: a three-axle vehicle (six contact
points), a single contact point under the centre of mass with no horizontal
acceleration (compatible, and unique), and a single contact point whose loads are
not compatible with the geometry it touches.
"""

from __future__ import annotations

import sys

sys.path.insert(0, "packages/suspension_multibody/src")

import numpy as np

from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    TireModelSpec,
    Vec3,
    WheelSpec,
)
from suspension_multibody.subsystems.assembler import AxleEntry, compose_entries_runtime
from suspension_multibody.subsystems.types import (
    DEFAULT_VEHICLE_SUBSYSTEMS,
    AssemblyRequest,
)
from suspension_multibody.vehicle.static_loads import (
    IncompatibleStaticLoadsError,
    compute_static_wheel_loads_for_assembly,
)

REQUEST = AssemblyRequest(mode="K", subsystems=DEFAULT_VEHICLE_SUBSYSTEMS)


def axle(name: str, x: float, *, side_y: float = -750.0) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=side_y + 250.0, z=500.0),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150.0, y=side_y + 250.0, z=500.0),
            "UPPER_OUTBOARD": Vec3(x=x, y=side_y, z=350.0),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=side_y + 250.0, z=100.0),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150.0, y=side_y + 250.0, z=100.0),
            "LOWER_OUTBOARD": Vec3(x=x, y=side_y, z=100.0),
            "TIE_ROD_INBOARD": Vec3(x=x, y=side_y + 300.0, z=250.0),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=side_y, z=250.0),
            "WHEEL_CENTER": Vec3(x=x, y=side_y, z=300.0),
            "RACK_CENTER": Vec3(x=x, y=0.0, z=250.0),
        },
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(
            RigidBodySpec(name=body, mass=10.0)
            for body in (
                "rack", "upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R",
                "upright_L", "upright_R", "tie_rod_L", "tie_rod_R",
            )
        ),
    )


def wheel(name: str) -> WheelSpec:
    return WheelSpec(
        name=name,
        body=f"wheel_{name}",
        center_local=Vec3(),
        mass=20.0,
        axial_inertia=2.0,
        tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0),
    )


def entry(placement, model, wheels, sides=None):
    return AxleEntry(
        placement=placement,
        prefix=f"{placement}_",
        axle=model,
        replace_bodies={},
        wheels=wheels,
        sides=sides,
    )


def assemble(entries):
    return compose_entries_runtime(
        tuple(entries), chassis_name="chassis", mode="K", request=REQUEST
    )


def report(label, result):
    print(f"### {label}")
    print("  contact points N =", len(result.support_points))
    print("  names            =", list(result.support_points))
    print("  rank(A)          =", result.rank)
    print("  unique(rank==N)  =", result.unique)
    print("  residual         =", repr(result.residual))
    print("  tolerance        =", repr(result.residual_tolerance))
    print("  total_mass       =", repr(result.total_mass))
    print("  weight           =", repr(result.total_mass * 9810.0))
    for name, value in result.wheel_loads.items():
        print(f"  load {name:14s} = {value!r}")
    print("  sum of loads     =", repr(float(sum(result.wheel_loads.values()))))
    print()


THREE = assemble(
    entry(placement, axle(placement, x), (wheel(f"{placement}_left"), wheel(f"{placement}_right")))
    for placement, x in (("front", 1400.0), ("middle", 0.0), ("rear", -1400.0))
)
report("three axle, static", compute_static_wheel_loads_for_assembly(THREE))
report(
    "three axle, longitudinal acceleration 1000 m/s^2",
    compute_static_wheel_loads_for_assembly(THREE, acceleration=np.array([1000.0, 0.0, 0.0])),
)

CORNER = assemble(
    [
        entry(
            "corner",
            axle("corner", 0.0, side_y=0.0),
            (wheel("single_left"),),
            sides=("L",),
        )
    ]
)
compatible = compute_static_wheel_loads_for_assembly(CORNER)
report("single contact point under the centre of mass, static", compatible)

AWAY = assemble(
    [
        entry(
            "corner",
            axle("corner", 0.0, side_y=-700.0),
            (wheel("single_left"),),
            sides=("L",),
        )
    ]
)

for label, assembly, acceleration in (
    (
        "single contact point under the centre of mass, lateral acceleration 1000 m/s^2",
        CORNER,
        np.array([0.0, 1000.0, 0.0]),
    ),
    (
        "single contact point 700 mm off the centre of mass, "
        "longitudinal -3000 m/s^2 and lateral 2000 m/s^2",
        AWAY,
        np.array([-3000.0, 2000.0, 0.0]),
    ),
):
    print(f"### {label}")
    try:
        compute_static_wheel_loads_for_assembly(assembly, acceleration=acceleration)
    except IncompatibleStaticLoadsError as error:
        print("  raised           :", type(error).__name__)
        print("  residual         =", repr(error.residual))
        print("  tolerance        =", repr(error.tolerance))
        print("  contact_points   =", error.contact_points)
        print("  message          :", str(error))
    else:  # pragma: no cover - the two cases above must raise
        print("  NO ERROR RAISED -- unexpected")
    print()

print("### the rank of both single-contact-point matrices", "###")
for label, assembly in (("under the centre of mass", CORNER), ("700 mm off it", AWAY)):
    contact = None
    weighted = np.zeros(3)
    for name, body in assembly.bodies.items():
        if body.mass <= 0.0:
            continue
        weighted += body.mass * assembly.state.point_world(name, body.center_of_mass)
    com = weighted / assembly.total_mass
    com = weighted / assembly.total_mass
    for body, local in assembly.wheel_centers.values():
        contact = assembly.state.point_world(body, local)
    matrix = np.array(
        [[1.0], [contact[0] - com[0]], [contact[1] - com[1]]], dtype=float
    )
    print(f"  {label}: A = {matrix.tolist()} rank = {int(np.linalg.matrix_rank(matrix))}")
