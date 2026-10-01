"""5-link / MacPherson / twist-beam via the explicit topology + compose_axle."""
import traceback
from suspension_multibody.schema import (FrontAxleModel, MassSpec, RigidBodySpec,
    Vec3, IdealJointSpec)
from suspension_multibody.subsystems.entry import compose_axle

def mk(name, joints, hp, bodies, topology="explicit"):
    return FrontAxleModel(name=name, topology=topology, hardpoints=hp,
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(RigidBodySpec(name=b, mass=10.0) for b in bodies),
        joints=tuple(joints))

def sph(name, a, ap, b, bp):
    return IdealJointSpec(name=name, kind="spherical", body_a=a, body_b=b,
                          point_a=ap, point_b=bp)
def rev(name, a, ap, b, bp, axis):
    return IdealJointSpec(name=name, kind="revolute", body_a=a, body_b=b,
                          point_a=ap, point_b=bp, axis_a=axis, axis_b=axis)

Y = Vec3(x=0.0, y=1.0, z=0.0)

def try_compose(label, model):
    print(f"\n--- {label} (topology={model.topology}, bodies={[b.name for b in model.bodies]}) ---")
    try:
        rt = compose_axle(model, "K")
        print(f"  ASSEMBLED: bodies={sorted(rt.bodies)} constraints={len(rt.constraints)}")
    except Exception as exc:
        print(f"  ASSEMBLY REFUSED: {type(exc).__name__}: {exc}")
        for fr in traceback.extract_tb(exc.__traceback__)[-3:]:
            print(f"      at {fr.filename}:{fr.lineno} in {fr.name}: {fr.line}")

# --- 5-link
j5 = []
for i in range(1, 6):
    j5.append(sph(f"link{i}_L", "chassis", Vec3(x=1400.0,y=-500.0,z=300.0+i*10),
                  "upright_L", Vec3(x=1400.0,y=-720.0,z=290.0+i*10)))
    j5.append(sph(f"link{i}_R", "chassis", Vec3(x=1400.0,y=500.0,z=300.0+i*10),
                  "upright_R", Vec3(x=1400.0,y=720.0,z=290.0+i*10)))
try_compose("5-link", mk("five_link", j5,
    {"WHEEL_CENTER": Vec3(x=1400.0,y=-720.0,z=300.0),
     "WHEEL_CENTER__R": Vec3(x=1400.0,y=720.0,z=300.0),
     "LINK1_OUTER": Vec3(x=1400.0,y=-720.0,z=310.0)}, ("upright_L","upright_R")))

# --- MacPherson: lower arm (2 joints) + strut (prismatic/cylindrical + spring)
jm = [
  rev("lca_front_L", "chassis", Vec3(x=1400.0,y=-500.0,z=150.0), "upright_L",
      Vec3(x=1400.0,y=-500.0,z=150.0), Y),
  sph("lca_rear_L", "chassis", Vec3(x=1550.0,y=-500.0,z=150.0), "upright_L",
      Vec3(x=1550.0,y=-500.0,z=150.0)),
  IdealJointSpec(name="strut_top_L", kind="spherical", body_a="chassis", body_b="upright_L",
      point_a=Vec3(x=1400.0,y=-500.0,z=800.0), point_b=Vec3(x=1400.0,y=-500.0,z=800.0)),
  rev("lca_front_R", "chassis", Vec3(x=1400.0,y=500.0,z=150.0), "upright_R",
      Vec3(x=1400.0,y=500.0,z=150.0), Y),
  sph("lca_rear_R", "chassis", Vec3(x=1550.0,y=500.0,z=150.0), "upright_R",
      Vec3(x=1550.0,y=500.0,z=150.0)),
  IdealJointSpec(name="strut_top_R", kind="spherical", body_a="chassis", body_b="upright_R",
      point_a=Vec3(x=1400.0,y=500.0,z=800.0), point_b=Vec3(x=1400.0,y=500.0,z=800.0)),
]
try_compose("MacPherson", mk("macpherson", jm,
    {"WHEEL_CENTER": Vec3(x=1400.0,y=-700.0,z=300.0),
     "WHEEL_CENTER__R": Vec3(x=1400.0,y=700.0,z=300.0)}, ("upright_L","upright_R")))

# --- twist beam: L/R trailing arms joined by a revolute (the beam)
jt = [
  rev("arm_pivot_L", "chassis", Vec3(x=1100.0,y=-480.0,z=250.0), "upright_L",
      Vec3(x=1100.0,y=-480.0,z=250.0), Y),
  rev("arm_pivot_R", "chassis", Vec3(x=1100.0,y=480.0,z=250.0), "upright_R",
      Vec3(x=1100.0,y=480.0,z=250.0), Y),
  IdealJointSpec(name="beam", kind="revolute", body_a="upright_L", body_b="upright_R",
      point_a=Vec3(x=1180.0,y=-100.0,z=300.0), point_b=Vec3(x=1180.0,y=100.0,z=300.0),
      axis_a=Vec3(x=0.0,y=0.0,z=1.0), axis_b=Vec3(x=0.0,y=0.0,z=1.0)),
]
try_compose("twist-beam", mk("twist_beam", jt,
    {"WHEEL_CENTER": Vec3(x=1400.0,y=-700.0,z=300.0),
     "WHEEL_CENTER__R": Vec3(x=1400.0,y=700.0,z=300.0),
     "ARM_PIVOT": Vec3(x=1100.0,y=-480.0,z=250.0)}, ("upright_L","upright_R")))
