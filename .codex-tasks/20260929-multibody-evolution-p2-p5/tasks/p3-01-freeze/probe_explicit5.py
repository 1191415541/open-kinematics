"""Can a 5-link / MacPherson axle be declared through the *explicit* topology?"""
from suspension_multibody.schema import (FrontAxleModel, MassSpec, RigidBodySpec,
    SteeringSystemSpec, TireModelSpec, Vec3, VehicleModel, WheelSpec, IdealJointSpec)
from suspension_multibody.subsystems.entry import compose_axle

def axle(topology, bodies, joints, hp):
    return FrontAxleModel(name="probe", topology=topology, hardpoints=hp,
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(RigidBodySpec(name=b, mass=10.0) for b in bodies),
        joints=tuple(joints))

# 5-link, explicit: 5 spherical links per side from chassis to upright
j5 = []
for i in range(1, 6):
    j5.append(IdealJointSpec(name=f"link{i}_L", kind="spherical", body_a="chassis",
        body_b="upright_L", point_a=Vec3(x=1400.0, y=-500.0, z=300.0 + i*10),
        point_b=Vec3(x=1400.0, y=-720.0, z=290.0 + i*10)))
    j5.append(IdealJointSpec(name=f"link{i}_R", kind="spherical", body_a="chassis",
        body_b="upright_R", point_a=Vec3(x=1400.0, y=500.0, z=300.0 + i*10),
        point_b=Vec3(x=1400.0, y=720.0, z=290.0 + i*10)))
hp5 = {"WHEEL_CENTER": Vec3(x=1400.0, y=-720.0, z=300.0),
       "WHEEL_CENTER__R": Vec3(x=1400.0, y=720.0, z=300.0),
       "LINK1_OUTER": Vec3(x=1400.0, y=-720.0, z=310.0)}
m5 = axle("explicit", ("upright_L", "upright_R"), j5, hp5)
print("5-link explicit model declared OK; bodies =", [b.name for b in m5.bodies],
      "joints =", len(m5.joints))
try:
    rt = compose_axle(m5, "K")
    print("   compose_axle OK: bodies =", sorted(rt.bodies), "constraints =", len(rt.constraints))
except Exception as exc:
    print(f"   compose_axle REFUSED: {type(exc).__name__}: {exc}")

# as a full-vehicle axle:
try:
    v = VehicleModel(chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=m5, rear_axle=m5,
        wheels=tuple(WheelSpec(name=n, body=f"wheel_{n}", center_local=Vec3(), mass=20.0,
            axial_inertia=2.0, tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0))
            for n in ("front_left","front_right","rear_left","rear_right")),
        steering=SteeringSystemSpec(ratio=16.0))
    print("   VehicleModel accepted the explicit 5-link axle")
    from suspension_multibody.vehicle.roll_centers import compute_vehicle_roll_centers
    try:
        out = compute_vehicle_roll_centers(v)
        print("   roll centres:", out)
    except Exception as exc:
        print(f"   roll-centre REFUSED: {type(exc).__name__}: {exc}")
except Exception as exc:
    print(f"   VehicleModel REFUSED: {type(exc).__name__}: {exc}")
