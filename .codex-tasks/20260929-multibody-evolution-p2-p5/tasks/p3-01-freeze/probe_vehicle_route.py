"""Does a full-vehicle runtime accept a non-double-wishbone (explicit) axle?"""
import traceback
from suspension_multibody.schema import (FrontAxleModel, MassSpec, RigidBodySpec,
    SteeringSystemSpec, TireModelSpec, Vec3, VehicleModel, WheelSpec, IdealJointSpec)
from suspension_multibody.subsystems.vehicle_assembly import compose_vehicle_runtime
Y = Vec3(x=0.0, y=1.0, z=0.0)
def rev(n,a,ap,b,bp,ax): return IdealJointSpec(name=n,kind="revolute",body_a=a,body_b=b,point_a=ap,point_b=bp,axis_a=ax,axis_b=ax)
def sph(n,a,ap,b,bp): return IdealJointSpec(name=n,kind="spherical",body_a=a,body_b=b,point_a=ap,point_b=bp)

for label, joints, hp in (
  ("5-link", [sph(f"link{i}_L","chassis",Vec3(x=1400.0,y=-500.0,z=300.0+i*10),
                  "upright_L",Vec3(x=1400.0,y=-720.0,z=290.0+i*10)) for i in range(1,6)] +
             [sph(f"link{i}_R","chassis",Vec3(x=1400.0,y=500.0,z=300.0+i*10),
                  "upright_R",Vec3(x=1400.0,y=720.0,z=290.0+i*10)) for i in range(1,6)],
   {"WHEEL_CENTER": Vec3(x=1400.0,y=-720.0,z=300.0),"WHEEL_CENTER__R": Vec3(x=1400.0,y=720.0,z=300.0)}),
  ("twist-beam", [rev("arm_pivot_L","chassis",Vec3(x=1100.0,y=-480.0,z=250.0),
                      "upright_L",Vec3(x=1100.0,y=-480.0,z=250.0),Y),
                  rev("arm_pivot_R","chassis",Vec3(x=1100.0,y=480.0,z=250.0),
                      "upright_R",Vec3(x=1100.0,y=480.0,z=250.0),Y)],
   {"WHEEL_CENTER": Vec3(x=1400.0,y=-700.0,z=300.0),"WHEEL_CENTER__R": Vec3(x=1400.0,y=700.0,z=300.0)}),
):
    ax = FrontAxleModel(name="axle", topology="explicit", hardpoints=hp,
        mass=MassSpec(sprung_mass=600.0),
        bodies=(RigidBodySpec(name="upright_L",mass=25.0), RigidBodySpec(name="upright_R",mass=25.0)),
        joints=tuple(joints))
    v = VehicleModel(chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=ax, rear_axle=ax,
        wheels=tuple(WheelSpec(name=n, body=f"wheel_{n}", center_local=Vec3(), mass=20.0,
            axial_inertia=2.0, tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0))
            for n in ("front_left","front_right","rear_left","rear_right")),
        steering=SteeringSystemSpec(ratio=16.0))
    print(f"\n--- {label}: VehicleModel accepted ---")
    try:
        rt = compose_vehicle_runtime(v, mode="K")
        print(f"  compose_vehicle_runtime OK: bodies={len(rt.bodies)} constraints={len(rt.constraints)} total_mass={rt.total_mass}")
    except Exception as exc:
        print(f"  compose_vehicle_runtime REFUSED: {type(exc).__name__}: {exc}")
        for fr in traceback.extract_tb(exc.__traceback__)[-3:]:
            print(f"      at {fr.filename}:{fr.lineno} in {fr.name}: {fr.line}")
