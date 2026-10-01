"""p5-03: read the bus channels off a REAL run and compare with the result doc."""
from __future__ import annotations
import importlib.util

import numpy as np

spec = importlib.util.spec_from_file_location(
    "vd_contract", "packages/suspension_multibody/tests/cases/test_vehicle_dynamic_contract.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
fixture = mod._fixture()
model = fixture._positioned_vehicle(fixture._vehicle())
case = fixture._case(model, wheel_speeds=(("front_left", 12.0), ("front_right", 12.0), ("rear_left", 12.0), ("rear_right", 12.0)))
raw = mod._run(model, case)

from suspension_multibody.signal_bus import open_bus  # noqa: E402
bus = open_bus(raw)

print("body_names[:6]:", raw.body_names[:6])
print("tire_names:", raw.tire_names)
print()

# --- wheel speed: compare the bus reading with the block itself ---
wheel = [n for n in raw.body_names if "wheel" in n][0]
val = bus.read("wheel_speed", entity=wheel)
direct = raw.block("body_state")[-1, raw.body_names.index(wheel), 10:13]
print(f"wheel_speed({wheel}) via bus  = {val}")
print(f"                via block    = {direct}")
print("identical:", np.array_equal(val, direct))
print()

# --- body acceleration ---
body = "chassis"
val2 = bus.read("body_acceleration", entity=body)
direct2 = raw.block("body_state")[-1, raw.body_names.index(body), 13:16]
print(f"body_acceleration({body}) via bus = {val2}")
print(f"                          via block = {direct2}")
print("identical:", np.array_equal(val2, direct2))
print()

# --- tire load ---
tire = raw.tire_names[0]
val3 = bus.read("tire_vertical_load", entity=tire)
direct3 = raw.block("tire_output")[-1, 0, 4]
print(f"tire_vertical_load({tire}) via bus = {val3}, via block col4 = {direct3}")
print("identical:", float(val3[0]) == float(direct3))
print()

# --- refusal cases ---
from suspension_multibody.signal_bus import BusError  # noqa: E402
for call, label in ((lambda: bus.read("wheel_speed"), "no entity named"),
                    (lambda: bus.read("nope"), "unknown channel")):
    try:
        call()
        print(f"{label}: NOT refused (bad)")
    except BusError as exc:
        print(f"{label}: BusError: {exc}")
print()

# --- actuator write ---
doc = {"elements": {"damper_L": {"parameters": {"compression_damping": 1.0}}}}
after = bus.write("variable_damping_L", 42.0, document=doc)
print("original doc unchanged:", doc["elements"]["damper_L"]["parameters"]["compression_damping"] == 1.0)
print("written doc           :", after["elements"]["damper_L"]["parameters"]["compression_damping"])
after2 = bus.write("motor_torque_FL", 1234.0, document={})
print("motor write path      :", after2)
