"""p4-03 evidence: the suspension's declared ports reach the assembly product."""
from __future__ import annotations
from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.si_assembly import (  # noqa: E402
    contributions_for_axle,
)
from suspension_multibody.templates import DOUBLE_WISHBONE

model = FrontAxleModel(
    hardpoints={
        "UPPER_INBOARD_FRONT": Vec3(x=1400.0, y=-500.0, z=500.0),
        "UPPER_INBOARD_REAR": Vec3(x=1550.0, y=-500.0, z=500.0),
        "UPPER_OUTBOARD": Vec3(x=1400.0, y=-750.0, z=350.0),
        "LOWER_INBOARD_FRONT": Vec3(x=1400.0, y=-500.0, z=100.0),
        "LOWER_INBOARD_REAR": Vec3(x=1550.0, y=-500.0, z=100.0),
        "LOWER_OUTBOARD": Vec3(x=1400.0, y=-750.0, z=100.0),
        "TIE_ROD_INBOARD": Vec3(x=1400.0, y=-450.0, z=250.0),
        "TIE_ROD_OUTBOARD": Vec3(x=1400.0, y=-750.0, z=250.0),
        "WHEEL_CENTER": Vec3(x=1400.0, y=-750.0, z=300.0),
        "RACK_CENTER": Vec3(x=1400.0, y=0.0, z=250.0),
    },
    mass=MassSpec(sprung_mass=600.0),
)
from suspension_multibody.subsystems.types import AssemblyRequest  # noqa: E402
contributions = contributions_for_axle(model, request=AssemblyRequest(mode="K"))

print("template-declared ports (templates/builtin.py):")
for p in DOUBLE_WISHBONE.ports:
    print(f"  {p.name:14} role={p.role:12} owner={p.owner:14} labels={sorted(p.labels)}")

print()
print("assembly product port set (names containing 'arb_mount'):")
found = {}
for contribution in contributions:
    for name, port in contribution.ports.items():
        found[f"{contribution.role}:{name}"] = port
for name, port in sorted(found.items()):
    if "arb_mount" in name:
        print(f"  {name}")
        print(f"      id    = {port.id}")
        print(f"      role  = {port.role}")
        print(f"      owner = {port.owner}")
        print(f"      labels= {sorted(port.labels)}")
print()
print("total ports offered by the axle contributions:", len(found))
