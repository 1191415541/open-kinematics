"""Ordinary declarations shared by execution lifecycle tests."""

import numpy as np

from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case

from ..authoring.test_generic_multibody import _assembly, _case
from ..physics.test_wheel_spin_boundary import wheel_assembly

PROTOCOLS = ("kc_quasi_static", "axle_dynamic", "vehicle_kc", "vehicle_dynamic",
             "handling", "ride_four_post", "ride_random_road")


def documents(protocol="axle_dynamic"):
    source = wheel_assembly()
    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "servo", "functional_role": "steering", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"}, "bodies": [], "hardpoints": [],
        "joints": [], "property_slots": [],
        "needs": [{"name": "action", "role": "wheel_hub", "required": True, "count": 1},
                  {"name": "reaction", "role": "wheel_carrier", "required": True, "count": 1}],
        "elements": [{"name": "actuator", "type": "steering_actuator", "target": "turntable",
            "parameters": {"type": "rotation", "body": "@action", "reaction_body": "@reaction",
                           "axis_local": [0, 1, 0], "stiffness": 1000, "damping": 10}}]})
    subsystem = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": "servo", "template": "servo.tpl.json", "functional_role": "steering", "placement_role": "any",
        "hardpoints": {}, "property_bindings": {}}, template=template)
    # Match the body's offered ports by their declared roles.
    pairings = {}
    for entry in source.entries:
        for port in entry.subsystem.template.payload.get("ports", ()):
            if port["role"] in {"wheel_hub", "wheel_carrier"}:
                pairings["action" if port["role"] == "wheel_hub" else "reaction"] = entry.ref+"."+port["name"]
    payload = source.to_payload()
    payload["subsystems"].append({"ref": "servo", "functional_role": "steering", "placement_role": "any",
                                  "pairings": [{"requirement_role": role, "port": port} for role, port in pairings.items()]})
    source = AssemblyDocument.from_payload(payload,
        subsystems={**{entry.ref: entry.subsystem for entry in source.entries}, "servo": subsystem})
    case = {**_case(), "family": protocol, "solver": {"initialization_mode": "provided_consistent_state"}}
    if protocol in {"kc_quasi_static", "vehicle_kc"}:
        case["c"] = {"loads": [{"fz": 0}], "load_marker": "wheel.hub"}
    elif protocol == "handling":
        case["handling"] = {"steering": [{"actuator": "servo.turntable", "shape": "constant", "amplitude_rad": 0}]}
    elif protocol == "ride_four_post":
        case["four_post"] = {"corners": [{"tire": "wheel.tire", "amplitude_m": 0, "frequency_hz": 1}]}
    elif protocol == "ride_random_road":
        case["ride_random_road"] = {"speed_mps": 1, "wheels": [{"tire": "wheel.tire",
            "components": [{"amplitude_m": 0, "wavelength_m": 1}]}]}
    return source, case


def compiled(protocol="axle_dynamic"):
    source, case = documents(protocol)
    model = DocumentLoader().load(source, case).resolve()
    return compile_resolved(model, plan_from_case(case))


def simple_compiled():
    model = DocumentLoader().load(_assembly(), _case()).resolve()
    return compile_resolved(model, plan_from_case(_case()))


def synthetic_run(submitted, *, status="success", manifest=None, diagnostics=None):
    from suspension_multibody.kernel import ContractRun

    bodies = [row["name"] for row in submitted.model_document["bodies"]]
    metadata = {"bodies": bodies, "cases": [{"sample_offset": 0, "sample_count": 1}], **(manifest or {})}
    blocks = {"body_state": np.zeros((1, len(bodies), 19))}
    if diagnostics is not None:
        blocks["diagnostics"] = diagnostics
    return ContractRun(document={"contract_version": 1, "status": status, "manifest": metadata},
        blocks=blocks, model_document=submitted.model_document, case_document=submitted.case_document,
        times_s=np.array([0.0]))


def resolved(source, case):
    return DocumentLoader().load(source, case).resolve()
