"""Bit-for-bit comparison of the four-wheel result, before vs after.

The *before* implementation is the archived copy of the file at its pre-change
state (``raw/static_loads_before.py``, md5 2b989cfe078bacc5a5969dbb4ba19250),
loaded here as its own module so the two implementations are compared in one
process on identical inputs -- not against numbers recorded earlier.
"""
from __future__ import annotations

import hashlib
import importlib.util
import struct
import sys
from pathlib import Path

sys.path.insert(0, "packages/suspension_multibody/src")

import numpy as np  # noqa: E402

from suspension_multibody.schema import (  # noqa: E402
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    SteeringSystemSpec,
    TireModelSpec,
    Vec3,
    VehicleModel,
    WheelSpec,
)
from suspension_multibody.vehicle import static_loads as after_module  # noqa: E402

BEFORE_PATH = Path(
    ".codex-tasks/20260929-multibody-evolution-p2-p5/tasks/p3-04-static-loads/raw/static_loads_before.py"
)
print("before file md5:", hashlib.md5(BEFORE_PATH.read_bytes()).hexdigest())

spec = importlib.util.spec_from_file_location(
    "suspension_multibody.vehicle._static_loads_before_probe", BEFORE_PATH
)
assert spec is not None and spec.loader is not None
before_module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = before_module
spec.loader.exec_module(before_module)
print("before module has _WHEELS:", hasattr(before_module, "_WHEELS"))
print("after module has _WHEELS:", hasattr(after_module, "_WHEELS"))


def axle(name: str, x: float) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=500.0),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=500.0),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750.0, z=350.0),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=100.0),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=100.0),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750.0, z=100.0),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450.0, z=250.0),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750.0, z=250.0),
            "WHEEL_CENTER": Vec3(x=x, y=-750.0, z=300.0),
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


def vehicle() -> VehicleModel:
    return VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=axle("front", 1_400.0),
        rear_axle=axle("rear", -1_400.0),
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=20.0,
                axial_inertia=2.0,
                tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0),
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16.0),
    )


def bits(value: float) -> str:
    return struct.pack(">d", float(value)).hex()


CASES: tuple[tuple[str, np.ndarray | None, float], ...] = (
    ("static", None, 9810.0),
    ("accel_x", np.array([1000.0, 0.0, 0.0]), 9810.0),
    ("accel_y", np.array([0.0, 1000.0, 0.0]), 9810.0),
    ("accel_z", np.array([0.0, 0.0, -500.0]), 9810.0),
    ("accel_xy", np.array([-3000.0, 2000.0, 100.0]), 9810.0),
    ("moon", None, 1_620.0),
)

failures: list[str] = []
for label, acceleration, gravity in CASES:
    before = before_module.compute_static_wheel_loads(
        vehicle(), acceleration=acceleration, gravity=gravity
    )
    after = after_module.compute_static_wheel_loads(
        vehicle(), acceleration=acceleration, gravity=gravity
    )
    print(f"\n=== {label} (gravity={gravity!r}) ===")
    rows: list[tuple[str, str, str]] = [
        ("rank/bool", repr(before.rank), repr(after.rank)),
        ("unique(rank==N)", repr(before.rank == 4), repr(after.unique)),
        ("residual", bits(before.residual), bits(after.residual)),
        ("total_mass", bits(before.total_mass), bits(after.total_mass)),
        (
            "center_of_mass",
            "".join(bits(v) for v in before.center_of_mass),
            "".join(bits(v) for v in after.center_of_mass),
        ),
    ]
    for name in sorted(before.wheel_loads):
        rows.append((f"load {name}", bits(before.wheel_loads[name]), bits(after.wheel_loads[name])))
    for name in sorted(before.support_points):
        rows.append(
            (
                f"support {name}",
                "".join(bits(v) for v in before.support_points[name]),
                "".join(bits(v) for v in after.support_points[name]),
            )
        )
    rows.append(("summary.total", bits(before.summary.total), bits(after.summary.total)))
    same_keys = set(before.wheel_loads) == set(after.wheel_loads) == set(after.support_points)
    rows.append(("wheel name set", repr(sorted(before.wheel_loads)), repr(sorted(after.wheel_loads))))
    for name, before_bits, after_bits in rows:
        marker = "OK " if before_bits == after_bits else "DIFF"
        if before_bits != after_bits:
            failures.append(f"{label}/{name}")
        print(f"  {marker} {name:22s} before={before_bits} after={after_bits}")
    if not same_keys:
        failures.append(f"{label}/wheel-name-set")
    print(
        "  np.array_equal on wheel-load vectors:",
        bool(
            np.array_equal(
                np.array([before.wheel_loads[n] for n in sorted(before.wheel_loads)]),
                np.array([after.wheel_loads[n] for n in sorted(after.wheel_loads)]),
            )
        ),
    )

print("\n== production consumer path (vehicle/service.py:40 shape) ==")
consumer_loads = after_module.compute_static_wheel_loads(vehicle()).wheel_loads
print("  compute_static_wheel_loads(model).wheel_loads =", {
    k: bits(v) for k, v in consumer_loads.items()
})

print("\nDIFFERENCES:", failures if failures else "none")
sys.exit(1 if failures else 0)
