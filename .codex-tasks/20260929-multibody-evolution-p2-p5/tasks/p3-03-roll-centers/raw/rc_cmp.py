from __future__ import annotations
import sys
import numpy as np
sys.path.insert(0, "packages/suspension_multibody/src")
sys.path.insert(0, "packages/suspension_multibody")

from suspension_multibody.schema import (
    FrontAxleModel, MassSpec, RigidBodySpec, SteeringSystemSpec,
    TireModelSpec, Vec3, VehicleModel, WheelSpec,
)

def axle(name, x):
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
        bodies=tuple(RigidBodySpec(name=b, mass=10.0) for b in (
            "rack", "upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R",
            "upright_L", "upright_R", "tie_rod_L", "tie_rod_R")),
    )

def vehicle():
    return VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=axle("front", 1400.0),
        rear_axle=axle("rear", -1400.0),
        wheels=tuple(WheelSpec(name=n, body=f"wheel_{n}", center_local=Vec3(),
                               mass=20.0, axial_inertia=2.0,
                               tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0))
                     for n in ("front_left","front_right","rear_left","rear_right")),
        steering=SteeringSystemSpec(ratio=16.0),
    )

from suspension_multibody.vehicle.roll_centers import (  # noqa: E402
    compute_vehicle_roll_centers,
)
res = compute_vehicle_roll_centers(vehicle())
for k, v in res.items():
    print(f"--- {k} (after) ---")
    print("  center            =", np.array2string(v.center, precision=12))
    print("  left_patch        =", np.array2string(v.left_contact_patch, precision=12))
    print("  left_slope        =", repr(v.left_contact_patch_slope))
    print("  right_patch       =", np.array2string(v.right_contact_patch, precision=12))
    print("  right_slope       =", repr(v.right_contact_patch_slope))
    print("  lateral_force     =", repr(v.lateral_force))
    print("  roll_moment       =", repr(v.roll_moment))
