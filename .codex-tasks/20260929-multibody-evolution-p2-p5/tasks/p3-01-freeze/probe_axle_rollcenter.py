"""p3-01 probe: the explicit-topology route, and the suspension role contract."""
import json
import sys
from pathlib import Path
sys.path.insert(0, "packages/suspension_multibody")
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.templates.registry import get, names
from suspension_multibody.templates.model import TemplateError

# 1. the shipped synthetic trailing-arm axle: does the *assembly* entry accept it?
p = Path("packages/suspension_multibody/tests/data/composable/synthetic_trailing_arm_axle.json")
payload = json.loads(p.read_text(encoding="utf-8"))
model = FrontAxleModel.model_validate(payload["model"])
print("trailing-arm explicit model declared OK; topology =", model.topology)
rt = compose_axle(model, "K")
print("trailing-arm assembled: bodies =", sorted(rt.bodies), "constraints =", len(rt.constraints))

# 2. the four topologies through the *roll-centre* entry.
from suspension_multibody.vehicle.roll_centers import compute_vehicle_roll_centers  # noqa: E402
from suspension_multibody.schema import (  # noqa: E402
    RigidBodySpec, SteeringSystemSpec,
    TireModelSpec, Vec3, VehicleModel, WheelSpec, MassSpec)
def make(hp):
    a = FrontAxleModel(name="rear", hardpoints=hp, mass=MassSpec(sprung_mass=600.0))
    b = FrontAxleModel(name="front", hardpoints={k: Vec3(x=v.x, y=v.y, z=v.z) for k,v in hp.items()},
                       mass=MassSpec(sprung_mass=600.0))
    return VehicleModel(chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=b, rear_axle=a,
        wheels=tuple(WheelSpec(name=n, body=f"wheel_{n}", center_local=Vec3(), mass=20.0,
            axial_inertia=2.0, tire=TireModelSpec(kind="fiala", vertical_stiffness=20.0))
            for n in ("front_left","front_right","rear_left","rear_right")),
        steering=SteeringSystemSpec(ratio=16.0))
try:
    out = compute_vehicle_roll_centers(make(payload["model"]["hardpoints"]))
    print("trailing-arm roll centre:", {k: v.center for k, v in out.items()})
except Exception as exc:
    print(f"trailing-arm roll-centre REFUSED: {type(exc).__name__}: {exc}")

# 3. the suspension role contract: which mounts must a suspension template declare?
print("\n--- suspension role contract (templates/roles.py) ---")
from suspension_multibody.templates.roles import ROLES  # noqa: E402
print("suspension.required_mounts =", ROLES["suspension"].required_mounts)
print("suspension.required_slots  =", ROLES["suspension"].required_slots)
print("suspension.outputs         =", ROLES["suspension"].outputs)

# 4. try to register a five-link suspension template that drops the upper mounts
from suspension_multibody.templates.model import (  # noqa: E402
    ConnectionDefinition, PartDefinition,
    PropertySlot, Template)
from suspension_multibody.templates.registry import register  # noqa: E402
five_link = Template(
    name="five_link_probe", role="suspension",
    parts=(PartDefinition("link_1_L"), PartDefinition("upright_L")),
    connections=tuple(
        ConnectionDefinition(f"link{i}_mount_L", "lower_front" if i == 1 else "lower_rear",
                             "spherical", owner="link_1_L", label=f"inner{i}",
                             far_owner="chassis", far_label=f"link{i}")
        for i in (1, 2)),
    property_slots=(PropertySlot("spring", "N/m"), PropertySlot("damper", "N*s/m")),
    suspension_kind="five_link",
)
try:
    register(five_link, replace=True)
    print("five-link template REGISTERED")
except TemplateError as exc:
    print(f"five-link template REFUSED: {type(exc).__name__}: {exc}")

print("\nregistered templates:", names())
print("double_wishbone.suspension_kind =", get("double_wishbone").suspension_kind)
