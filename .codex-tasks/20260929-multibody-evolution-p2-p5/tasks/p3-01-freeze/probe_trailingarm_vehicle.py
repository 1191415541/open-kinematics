import json
import traceback
from pathlib import Path
from suspension_multibody.schema import (FrontAxleModel, RigidBodySpec,
    SteeringSystemSpec, TireModelSpec, Vec3, VehicleModel, WheelSpec)
from suspension_multibody.vehicle.roll_centers import compute_vehicle_roll_centers

p = Path("packages/suspension_multibody/tests/data/composable/synthetic_trailing_arm_axle.json")
payload = json.loads(p.read_text(encoding="utf-8"))
axle = FrontAxleModel.model_validate(payload["model"])
print("explicit trailing-arm axle topology =", axle.topology, "bodies =", [b.name for b in axle.bodies])
for label, kw in (("as-is (topology=explicit, per the fixture)", {}),):
    try:
        model = VehicleModel(
            chassis=RigidBodySpec(name="chassis", mass=1200.0),
            front_axle=axle, rear_axle=axle,
            wheels=tuple(WheelSpec(name=n, body=f"wheel_{n}", center_local=Vec3(), mass=20.0,
                axial_inertia=2.0, tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0))
                for n in ("front_left","front_right","rear_left","rear_right")),
            steering=SteeringSystemSpec(ratio=16.0))
        print(f"[{label}] VEHICLE DECLARED")
        out = compute_vehicle_roll_centers(model)
        print(f"[{label}] roll centres:", {k: v.center for k, v in out.items()})
    except Exception as exc:
        print(f"[{label}] REFUSED: {type(exc).__name__}: {exc}")
        for fr in traceback.extract_tb(exc.__traceback__)[-2:]:
            print(f"    at {fr.filename}:{fr.lineno} in {fr.name}: {fr.line}")
