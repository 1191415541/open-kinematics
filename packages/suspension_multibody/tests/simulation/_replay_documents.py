"""An ordinary two-coordinate body/rig fixture for prescribed roll and heave."""

import numpy as np

from suspension_multibody.api import simulate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.signals import time_grid


def replay_documents(case):
    if case.vehicle is None:
        raise ValueError("body/rig authoring requires vehicle data")
    if case.mode != "vehicle_kc_dynamic":
        raise ValueError("body/rig fixture requires vehicle_kc_dynamic input")
    common = {"document": "template", "schema_version": 1, "functional_role": "generic",
        "allowed_placement_roles": ["any"], "symmetry": "asymmetric", "units": {"length": "m"},
        "elements": [], "property_slots": []}
    body = TemplateDocument.from_payload({**common, "name": "body",
        "bodies": [{"name": "body", "mass": case.vehicle.mass,
            "inertia": (np.asarray(case.vehicle.inertia)*1e-6).tolist()}],
        "hardpoints": [{"name": "center", "owner": "body", "space": "body"}], "joints": [],
        "ports": [{"name": "mount", "role": "body_mount", "owner": "body", "point": "center"}]})
    rig = TemplateDocument.from_payload({**common, "name": "rig",
        "bodies": [{"name": "ground", "fixed": True}, {"name": "slider", "mass": 1., "inertia": np.eye(3).tolist()}],
        "hardpoints": [{"name": "origin", "owner": "ground", "space": "body"},
            {"name": "slide", "owner": "slider", "space": "body"}],
        "needs": [{"name": "body", "role": "body_mount", "count": 1, "required": True}],
        "joints": [{"name": "heave_joint", "type": "prismatic", "body_a": "ground", "body_b": "slider",
            "point_a": "origin", "point_b": "slide", "axis": [0, 0, 1]},
            {"name": "roll_joint", "type": "revolute", "body_a": "slider", "body_b": "@body",
                "point_a": "slide", "point_b": "@body", "axis": [1, 0, 0]}],
        "coordinates": [{"name": "heave", "joint": "heave_joint", "kind": "translation"},
            {"name": "roll", "joint": "roll_joint", "kind": "rotation"}]})
    subsystems = {name: SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": name, "template": name+".tpl.json", "functional_role": "generic", "placement_role": "any",
        "hardpoints": points, "property_bindings": {}}, template=template)
        for name, template, points in (("body", body, {"center": [0, 0, 0]}),
            ("rig", rig, {"origin": [0, 0, 0], "slide": [0, 0, 0]}))}
    assembly = AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1,
        "name": case.name, "assembly_kind": "generic_multibody", "mode": "K", "gravity": [0, 0, 0],
        "subsystems": [{"ref": name, "functional_role": "generic", "placement_role": "any",
            **({"pairings": [{"requirement_role": "body", "port": "body.mount"}]} if name == "rig" else {})}
            for name in subsystems]}, subsystems=subsystems)
    from suspension_multibody.schema import TimeSignal

    signals = {motion.target: motion.displacement for motion in case.prescribed_motions}
    roll, heave = signals.get("body_roll", TimeSignal(constant=0)), signals.get("body_heave", TimeSignal(constant=0))
    times = time_grid(case)
    angle, height = roll.value_at(times[0]), heave.value_at(times[0])*.001
    rate, speed = roll.derivative_at(times[0]), heave.derivative_at(times[0])*.001
    plan = {"schema_version": 1, "name": case.name, "study": "dynamic", "protocol": "vehicle_dynamic",
        "samples": list(times), "solver": {"initialization_mode": "provided_consistent_state", "internal_step_s": .0001},
        "initial_state": {"body.body": {"position": [0, 0, height], "quaternion": [np.cos(angle/2), np.sin(angle/2), 0, 0],
            "omega": [rate, 0, 0], "velocity": [0, 0, speed]}, "rig.slider": {"position": [0, 0, height], "velocity": [0, 0, speed]}},
        "boundaries": [{"name": "prescribed_"+name, "coordinate": "rig."+name,
            "mode": "prescribed_angle" if name == "roll" else "prescribed_displacement", "program": name,
            "units": "rad" if name == "roll" else "m"} for name in ("roll", "heave")],
        "inputs": [{"name": name, "values": [signal.value_at(time)*scale for time in times],
            "rates": [signal.derivative_at(time)*scale for time in times]}
            for name, signal, scale in (("roll", roll, 1), ("heave", heave, .001))], "outputs": []}
    return assembly, plan


def replay(case):
    return simulate(*replay_documents(case)).result
