import traceback
from suspension_multibody.schema import FrontAxleModel, MassSpec, RigidBodySpec, Vec3
from suspension_multibody.subsystems.entry import compose_axle
REQ=("rack","upper_arm_L","upper_arm_R","lower_arm_L","lower_arm_R","upright_L","upright_R","tie_rod_L","tie_rod_R")
hp={"LINK1_INNER": Vec3(x=1380.0,y=-500.0,z=520.0),"LINK1_OUTER": Vec3(x=1400.0,y=-750.0,z=520.0),
    "WHEEL_CENTER": Vec3(x=1400.0,y=-750.0,z=300.0)}
m=FrontAxleModel(name="probe",hardpoints=hp,mass=MassSpec(sprung_mass=600.0),
                 bodies=tuple(RigidBodySpec(name=b,mass=10.0) for b in REQ))
try:
    compose_axle(m,"K")
except Exception as exc:
    print(f"{type(exc).__name__}: {exc}\n--- full traceback ---")
    traceback.print_exc()
