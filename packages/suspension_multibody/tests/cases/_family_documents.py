"""Ordinary assembly/case fixtures for independent excitation expansion checks."""

import copy
import json

from suspension_multibody.api import validate
from suspension_multibody.authoring.loader import CaseDocument
from suspension_multibody.authoring.migration import migrate_v1_vehicle_case
from suspension_multibody.compilation.resolved import plan_from_case


def fixture_documents(*, gravity=False):
    from tests.vehicle.vehicle_fixtures import _case, _positioned_vehicle, _vehicle

    model = _positioned_vehicle(_vehicle())
    case = _case(model)
    if gravity:
        from suspension_multibody.schema import Vec3

        case = case.model_copy(update={"solver": case.solver.model_copy(update={
            "gravity": Vec3(x=0, y=0, z=-9806.65), "end_time": .004,
            "step_size": .001, "internal_step_size": .001, "min_internal_step_size": .001})})
    return migrate_v1_vehicle_case(case)


def family_case(base, native):
    plan = plan_from_case(native).to_document()
    original = base.to_payload()
    plan["initial_state"] = original["initial_state"]
    inputs = copy.deepcopy(original["excitation"].get("inputs", {}))
    plan["excitation"]["inputs"] = inputs
    plan["inputs"] = []
    return CaseDocument(plan)


def sampled_case(family, target, rate, role, target_key):
    plan = family.to_payload()
    plan["protocol"] = "vehicle_dynamic"
    for key in ("handling", "four_post", "ride_random_road"):
        plan["excitation"].pop(key, None)
    for quantity, values in ((role, target), ("steering_rate" if role == "steering_target" else "road_velocity", rate)):
        for name, samples in values.items():
            plan["inputs"].append({"name": quantity+":"+name, "role": quantity,
                target_key: name, "values": samples.tolist()})
    return CaseDocument(plan)


def assert_document_parity(assembly, case, tmp_path):
    path = tmp_path / "case.json"
    path.write_text(json.dumps(case.to_payload()), encoding="utf-8")
    memory, file = validate(assembly, case), validate(assembly, path)
    assert memory.model_payload == file.model_payload
    assert memory.case_payload == file.case_payload
    assert memory.metadata["model_fingerprint"] == file.metadata["model_fingerprint"]
    assert memory.case_document["family"] == case.to_payload()["protocol"]
    assert memory.metadata["compiler"] == "ResolvedModelCompiler"
    return memory
