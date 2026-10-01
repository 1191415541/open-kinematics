"""Reproduce: does the production document route accept a rotational_torque element?"""
from __future__ import annotations
import sys
sys.path.insert(0, "packages/suspension_multibody/src")

from suspension_multibody.simulation.backend import run_contract  # the single kernel submit point

doc = {
    "contract": "multibody-model", "contract_version": 1, "kind": "model",
    "name": "torque-route-probe",
    "units": {"length": "mm", "mass": "kg", "time": "s", "angle": "rad"},
    "gravity": [0.0, 0.0, -9810.0],
    "capabilities": ["axle"],
    "bodies": [
        {"name": "ground", "mass": 1.0, "inertia": [[1,0,0],[0,1,0],[0,0,1]],
         "fixed": True, "position": [0,0,0], "quaternion": [1,0,0,0],
         "velocity": [0,0,0], "omega": [0,0,0]},
        {"name": "rotor", "mass": 20.0, "inertia": [[2,0,0],[0,2,0],[0,0,2]],
         "fixed": False, "position": [0,0,0], "quaternion": [1,0,0,0],
         "velocity": [0,0,0], "omega": [0,3.0,0]},
    ],
    "joints": [{"name": "spin", "type": "revolute",
                "body_a": "ground", "body_b": "rotor",
                "point_a": [0,0,0], "point_b": [0,0,0],
                "axis_a": [0,1,0], "axis_b": [0,1,0]}],
    "elements": [
        {"name": "torque", "type": "rotational_torque",
         "body_a": "ground", "body_b": "rotor",
         "parameters": {"axis": [0,1,0], "demand": 1.0, "stiffness": 1000.0}},
    ],
    "tires": [],
}
case = {
    "contract": "multibody-case", "contract_version": 1, "kind": "case",
    "name": "probe-case", "family": "axle_dynamic",
    "duration": 0.001, "step": 0.001,
}
try:
    out = run_contract(doc, case)
    print("run_contract returned:", type(out).__name__)
    status = out.get("status") if isinstance(out, dict) else getattr(out, "status", None)
    print("status =", status)
    if isinstance(out, dict):
        print("keys:", sorted(out)[:20])
        print("message:", out.get("message"))
except Exception as exc:
    print(f"{type(exc).__name__}: {exc}")
