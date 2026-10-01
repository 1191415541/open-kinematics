"""Scratch: existing-family document round trip through the same entry point."""
from __future__ import annotations

import sys

sys.path.insert(0, "packages/suspension_multibody/src")

from suspension_contracts import validate_model  # noqa: E402
from suspension_multibody.kernel import run_contract  # noqa: E402

DOC = {
    "contract": "multibody-model",
    "contract_version": 1,
    "kind": "model",
    "name": "existing-families",
    "units": {"length": "mm", "mass": "kg", "time": "s", "angle": "rad"},
    "gravity": [0.0, 0.0, -9810.0],
    "capabilities": ["axle"],
    "bodies": [
        {
            "name": "ground",
            "mass": 1.0,
            "inertia": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "fixed": True,
            "position": [0, 0, 0],
            "quaternion": [1, 0, 0, 0],
            "velocity": [0, 0, 0],
            "omega": [0, 0, 0],
        },
        {
            "name": "slider",
            "mass": 10.0,
            "inertia": [[100, 0, 0], [0, 100, 0], [0, 0, 100]],
            "fixed": False,
            "position": [0, 0, 0],
            "quaternion": [1, 0, 0, 0],
            "velocity": [0, 0, 0],
            "omega": [0, 0, 0],
        },
    ],
    "joints": [
        {
            "name": "slide",
            "type": "prismatic",
            "body_a": "ground",
            "body_b": "slider",
            "point_a": [0, 0, 0],
            "point_b": [0, 0, 0],
            "axis_a": [0, 0, 1],
            "axis_b": [0, 0, 1],
        }
    ],
    "elements": [
        {
            "name": "spring",
            "type": "spring",
            "body_a": "ground",
            "body_b": "slider",
            "parameters": {
                "point_a": [0, 0, 0],
                "point_b": [0, 0, 0],
                "stiffness": 300.0,
                "free_length": 100.0,
                "preload": 0.0,
            },
        },
        {
            "name": "damper",
            "type": "damper",
            "body_a": "ground",
            "body_b": "slider",
            "parameters": {
                "point_a": [0, 0, 0],
                "point_b": [0, 0, 0],
                "compression_damping": 12.0,
                "rebound_damping": 15.0,
            },
        },
        {
            "name": "stop",
            "type": "bump_stop",
            "body_a": "ground",
            "body_b": "slider",
            "parameters": {
                "point_a": [0, 0, 0],
                "point_b": [0, 0, 0],
                "clearance": 20.0,
                "stiffness": 800.0,
                "direction": 1.0,
                "damping": 4.0,
            },
        },
        {
            "name": "bushing",
            "type": "bushing",
            "body_a": "ground",
            "body_b": "slider",
            "parameters": {
                "point_a": [0, 0, 0],
                "point_b": [0, 0, 0],
                "frame_a_quaternion": [1, 0, 0, 0],
                "frame_b_quaternion": [1, 0, 0, 0],
                "stiffness": [
                    [50, 0, 0, 0, 0, 0],
                    [0, 50, 0, 0, 0, 0],
                    [0, 0, 50, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1],
                ],
                "damping": [
                    [1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0],
                    [0, 0, 0, 0.1, 0, 0],
                    [0, 0, 0, 0, 0.1, 0],
                    [0, 0, 0, 0, 0, 0.1],
                ],
            },
        },
        {
            "name": "aero",
            "type": "aerodynamic_drag",
            "body_a": "slider",
            "parameters": {
                "application_point": [0, 0, 0],
                "forward_axis": [1, 0, 0],
                "coefficient": 0.5,
            },
        },
    ],
    "tires": [],
}

CASE = {
    "contract": "multibody-case",
    "contract_version": 1,
    "kind": "case",
    "family": "axle_dynamic",
    "name": "existing-families-case",
    "time": {"start_s": 0.0, "end_s": 0.002, "step_s": 0.001},
}

validate_model(DOC)
run = run_contract(DOC, CASE)
print("status:", run.status)
print("blocks:", sorted(run.blocks))
print("body_state shape:", run.block("body_state").shape)
print("spring_output shape:", run.block("spring_output").shape)
print("bushing_output shape:", run.block("bushing_output").shape)
print("first sample state row:", run.block("body_state")[0].ravel()[:12])
print("last sample state row:", run.block("body_state")[-1].ravel()[:12])
