"""p3-01 / Goal 2 probe, final form: the four topologies, two entry points."""
import traceback
from suspension_multibody.schema import (FrontAxleModel, MassSpec, RigidBodySpec,
    SteeringSystemSpec, TireModelSpec, Vec3, VehicleModel, WheelSpec)

REQUIRED = ("rack","upper_arm_L","upper_arm_R","lower_arm_L","lower_arm_R",
            "upright_L","upright_R","tie_rod_L","tie_rod_R")
WHEELS = ("front_left","front_right","rear_left","rear_right")

def axle(name, x, hp):
    return FrontAxleModel(name=name, hardpoints=hp, mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(RigidBodySpec(name=b, mass=10.0) for b in REQUIRED))

def vehicle(hp, x=1400.0):
    f = axle("front", x, hp)
    r = axle("rear", -x, {k: Vec3(x=v.x - 2*x, y=v.y, z=v.z) for k, v in hp.items()})
    return VehicleModel(chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=f, rear_axle=r,
        wheels=tuple(WheelSpec(name=n, body=f"wheel_{n}", center_local=Vec3(), mass=20.0,
            axial_inertia=2.0, tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0))
            for n in WHEELS), steering=SteeringSystemSpec(ratio=16.0))

DW = {"UPPER_INBOARD_FRONT": Vec3(x=1400.0,y=-500.0,z=500.0),
      "UPPER_INBOARD_REAR": Vec3(x=1550.0,y=-500.0,z=500.0),
      "UPPER_OUTBOARD": Vec3(x=1400.0,y=-750.0,z=350.0),
      "LOWER_INBOARD_FRONT": Vec3(x=1400.0,y=-500.0,z=100.0),
      "LOWER_INBOARD_REAR": Vec3(x=1550.0,y=-500.0,z=100.0),
      "LOWER_OUTBOARD": Vec3(x=1400.0,y=-750.0,z=100.0),
      "TIE_ROD_INBOARD": Vec3(x=1400.0,y=-450.0,z=250.0),
      "TIE_ROD_OUTBOARD": Vec3(x=1400.0,y=-750.0,z=250.0),
      "WHEEL_CENTER": Vec3(x=1400.0,y=-750.0,z=300.0),
      "RACK_CENTER": Vec3(x=1400.0,y=0.0,z=250.0)}

FIVE_LINK = {"LINK1_INNER": Vec3(x=1380.0,y=-500.0,z=520.0),
             "LINK1_OUTER": Vec3(x=1400.0,y=-750.0,z=520.0),
             "LINK2_INNER": Vec3(x=1520.0,y=-500.0,z=480.0),
             "LINK2_OUTER": Vec3(x=1440.0,y=-750.0,z=500.0),
             "LINK3_INNER": Vec3(x=1380.0,y=-500.0,z=160.0),
             "LINK3_OUTER": Vec3(x=1400.0,y=-750.0,z=140.0),
             "LINK4_INNER": Vec3(x=1520.0,y=-500.0,z=200.0),
             "LINK4_OUTER": Vec3(x=1440.0,y=-750.0,z=180.0),
             "LINK5_INNER": Vec3(x=1450.0,y=-460.0,z=300.0),
             "LINK5_OUTER": Vec3(x=1420.0,y=-740.0,z=290.0),
             "WHEEL_CENTER": Vec3(x=1400.0,y=-750.0,z=300.0)}

MACPHERSON = {"LOWER_INBOARD_FRONT": Vec3(x=1400.0,y=-500.0,z=150.0),
              "LOWER_INBOARD_REAR": Vec3(x=1550.0,y=-500.0,z=150.0),
              "LOWER_OUTBOARD": Vec3(x=1400.0,y=-700.0,z=160.0),
              "STRUT_TOP": Vec3(x=1400.0,y=-500.0,z=800.0),
              "STRUT_LOWER": Vec3(x=1400.0,y=-700.0,z=250.0),
              "TIE_ROD_INBOARD": Vec3(x=1400.0,y=-450.0,z=250.0),
              "TIE_ROD_OUTBOARD": Vec3(x=1400.0,y=-730.0,z=250.0),
              "WHEEL_CENTER": Vec3(x=1400.0,y=-700.0,z=300.0),
              "RACK_CENTER": Vec3(x=1400.0,y=0.0,z=250.0)}

TWIST_BEAM = {"TRAILING_ARM_PIVOT": Vec3(x=1100.0,y=-480.0,z=250.0),
              "WHEEL_CENTER": Vec3(x=1400.0,y=-700.0,z=300.0),
              "BEAM_PIVOT": Vec3(x=1180.0,y=-100.0,z=300.0),
              "SPRING_SEAT": Vec3(x=1250.0,y=-600.0,z=380.0)}

def roll_center(hp, label):
    from suspension_multibody.vehicle.roll_centers import compute_vehicle_roll_centers
    print(f"\n===== [{label}] entry A: compute_vehicle_roll_centers =====")
    try:
        m = vehicle(hp)
    except Exception as exc:
        print(f"MODEL DECLARATION REFUSED: {type(exc).__name__}: {exc}")
        return
    try:
        out = compute_vehicle_roll_centers(m)
    except Exception as exc:
        print(f"CONSTRUCT REFUSED: {type(exc).__name__}: {exc}")
        for fr in traceback.extract_tb(exc.__traceback__)[-3:]:
            print(f"    at {fr.filename}:{fr.lineno} in {fr.name}: {fr.line}")
        return
    for n, r in out.items():
        print(f"CONSTRUCTED: {n} center={r.center!r} left_ic={r.left_instant_center!r} right_ic={r.right_instant_center!r}")

def assembly(hp, label):
    from suspension_multibody.subsystems.entry import compose_axle
    print(f"----- [{label}] entry B: compose_axle(mode='K') -----")
    try:
        rt = compose_axle(axle(label, 1400.0, hp), "K")
    except Exception as exc:
        print(f"ASSEMBLY REFUSED: {type(exc).__name__}: {exc}")
        for fr in traceback.extract_tb(exc.__traceback__)[-2:]:
            print(f"    at {fr.filename}:{fr.lineno} in {fr.name}: {fr.line}")
        return
    print(f"ASSEMBLED: bodies={sorted(rt.bodies)} constraints={len(rt.constraints)}")

for label, hp in (("double_wishbone", DW), ("five_link", FIVE_LINK),
                  ("macpherson", MACPHERSON), ("twist_beam", TWIST_BEAM)):
    roll_center(hp, label)
    assembly(hp, label)
