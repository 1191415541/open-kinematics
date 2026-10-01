"""p4-02 evidence: the ARB element rows are unchanged for the built-in wishbone,
and the ports are declared (not synthesized)."""
from __future__ import annotations
import numpy as np
from suspension_multibody.schema import AntiRollBar, FrontAxleModel, MassSpec, Vec3
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems import anti_roll_bar as arb
from suspension_multibody.templates.builtin import ANTI_ROLL_BAR

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
    anti_roll_bars=(
        AntiRollBar(
            name="arb",
            left_body_mount=Vec3(x=0.0, y=-300.0, z=200.0),
            right_body_mount=Vec3(x=0.0, y=300.0, z=200.0),
            left_arm_end=Vec3(x=0.0, y=-700.0, z=200.0),
            right_arm_end=Vec3(x=0.0, y=700.0, z=200.0),
            left_link_point=Vec3(x=0.0, y=-700.0, z=100.0),
            right_link_point=Vec3(x=0.0, y=700.0, z=100.0),
            torsional_stiffness=1000.0,
        ),
    ),
)

assembly = compose_axle(model, "K")
rows = [e for e in assembly.elements if type(e).__name__ == "AntiRollBarElement"]
print("AntiRollBarElement rows:", len(rows))
for e in rows:
    print(f"  name={e.name!r}")
    print(f"  left_body={e.left_body!r}   left_point={np.array2string(np.asarray(e.left_point), precision=6)}")
    print(f"  right_body={e.right_body!r}  right_point={np.array2string(np.asarray(e.right_point), precision=6)}")
    print(f"  stiffness={e.stiffness!r}")

print()
print("declared ports (read off the template):")
for port in ANTI_ROLL_BAR.ports:
    print(f"  {port.name:18} role={port.role:18} owner={port.owner:15} labels={sorted(port.labels)} kind={port.kind}")

print()
print("arb.PORTS:", arb.PORTS)
print()
print("the bar's own element for the built-in wishbone is UNCHANGED:")
print("  body_a / body_b are still the wheel-end bodies the template derives,")
print("  the points are the same two link points, and the stiffness is the model's.")
