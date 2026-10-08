"""Read-only oracle comparison that separates frozen layout from physical changes."""

import json
import runpy
from pathlib import Path

import numpy as np
from suspension_contracts import pack_container

from suspension_multibody.api import validate
from suspension_multibody.authoring import migrate_v1_vehicle_case
from suspension_multibody.cases.vehicle_dynamic import case_document, model_document
from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
from suspension_multibody.simulation import SimulationRequest, run_compiled
from suspension_multibody.simulation.compiler import compile_document_pair
from suspension_multibody.authoring import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan

ROOT = Path(__file__).resolve().parents[3]
gate = runpy.run_path(str(ROOT/"packages/suspension_multibody/scripts/case_parity_check.py"))
fixture = gate["_load_vehicle_fixture"]()
cases = gate["_vehicle_case_matrix"](fixture)
layouts = json.loads((ROOT/"packages/suspension_multibody/tests/data/vehicle_dynamics_baseline/entity_layout.json").read_text())["cases"]
reports = []
for name, (model, case) in cases.items():
    prepared = prepare_vehicle_run(model, case)
    old, old_blob = model_document(model, prepared)
    old_case, old_case_blob = case_document(model, case, prepared)
    previous = run_compiled(compile_document_pair(SimulationRequest(assembly="vehicle", family="vehicle_dynamic"),
        model_document=old, case_document=old_case, model_payload=pack_container(old, old_blob),
        case_payload=pack_container(old_case, old_case_blob))).raw
    assembly, plan = migrate_v1_vehicle_case(case)
    bundle = DocumentLoader().load(assembly, plan)
    resolved = bundle.resolve()
    graph = resolved.to_document()
    for key, ordering in (("bodies", layouts[name]["bodies"]), ("joints", layouts[name]["constraints"])):
        order = {entity: index for index, entity in enumerate(ordering)}
        graph[key].sort(key=lambda row: order.get(row["name"], len(order)))
    compiled = compile_resolved(ResolvedModel(graph, resolved.resource_payload), ResolvedSolvePlan(plan.to_payload()))
    result = run_compiled(compiled).result
    historical = json.loads(json.dumps(graph))
    historical_bodies = dict(zip((row["name"] for row in old["bodies"]), layouts[name]["bodies"]))
    for tire, before in zip(historical["tires"], old["tires"]):
        tire["parameters"]["frame_body"] = historical_bodies[before["parameters"]["frame_body"]]
        tire["parameters"]["frame_center_local"] = before["parameters"]["frame_center_local"]
    restored = run_compiled(compile_resolved(ResolvedModel(historical, resolved.resource_payload),
        ResolvedSolvePlan(plan.to_payload()))).result
    old_ids = dict(zip((row["name"] for row in old["bodies"]), layouts[name]["bodies"]))
    old_ids.update(dict(zip((row["name"] for row in old["joints"]), layouts[name]["constraints"])))
    old_ids.update({row["name"]: entity for row, entity in zip(old["tires"], layouts[name]["ledgers"]["tire"])})
    old_ids.update({channel.channel_name: "steering_"+channel.channel_name+"."+channel.channel_name
        for channel in (model.steering, *model.steering_channels) if channel.enabled})
    def remap(value):
        if isinstance(value, dict):
            return {key: remap(item) for key, item in value.items()}
        if isinstance(value, list):
            return [remap(item) for item in value]
        if isinstance(value, str):
            return old_ids.get(value, value)
        return value
    old_remapped = remap(old)
    def differences(first, second, where):
        found = []
        if isinstance(first, dict) and isinstance(second, dict):
            for key in first.keys() & second.keys():
                found.extend(differences(first[key], second[key], where+"."+key))
        elif isinstance(first, list) and isinstance(second, list):
            if len(first) != len(second):
                found.append({"field": where, "old_count": len(first), "new_count": len(second)})
            else:
                for index, (a, b) in enumerate(zip(first, second)):
                    found.extend(differences(a, b, where+"["+str(index)+"]"))
        elif first != second:
            found.append({"field": where, "old": first, "new": second})
        return found
    model_differences = []
    for key in ("bodies", "joints", "tires"):
        model_differences.extend(differences(old_remapped[key], compiled.model_document[key], key))
    model_differences.extend(differences(remap(old_case["solver"]), compiled.case_document["solver"], "solver"))
    historical_states = np.stack([restored.body_state(entity) for entity in layouts[name]["bodies"]], axis=1)
    mapped = np.stack([result.body_state(entity) for entity in layouts[name]["bodies"]], axis=1)
    old_state = previous.block("body_state")[:len(previous.times_s)]
    delta = np.abs(mapped-old_state)
    differing = np.argwhere(mapped.view(np.uint64) != old_state.view(np.uint64))
    example = []
    for sample, body, column in differing[:10]:
        example.append({"sample": int(sample), "body": previous.body_names[body], "column": int(column),
            "new": float(mapped[sample, body, column]), "old": float(old_state[sample, body, column]),
            "new_bits": int(mapped.view(np.uint64)[sample, body, column]), "old_bits": int(old_state.view(np.uint64)[sample, body, column])})
    current_diag = result.raw.block("diagnostics")[:len(result.times_s)]
    previous_diag = previous.block("diagnostics")[:len(previous.times_s)]
    report = {"name": name, "producer": "ResolvedModelCompiler", "model_fingerprint": compiled.metadata["model_fingerprint"],
        "historical_frame_restores_states": bool(np.array_equal(historical_states.view(np.uint64), old_state.view(np.uint64))),
        "historical_frame_max_error": float(np.max(np.abs(historical_states-old_state))),
        "historical_matches_baseline": gate["_vehicle_document_digests"](restored, layouts[name]) == json.loads(gate["_VEHICLE_BASELINE"].read_text())["cases"][name],
        "model_differences": model_differences,
        "position_max_m": float(delta[..., :3].max()), "quaternion_max": float(delta[..., 3:7].max()),
        "velocity_max_m_per_s": float(delta[..., 7:10].max()), "omega_max_rad_per_s": float(delta[..., 10:13].max()),
        "acceleration_max_m_per_s2": float(delta[..., 13:16].max()), "alpha_max_rad_per_s2": float(delta[..., 16:19].max()),
        "different_values": int(len(differing)), "examples": example,
        "diagnostic_shape": list(current_diag.shape), "old_diagnostic_shape": list(previous_diag.shape),
        "diagnostic_max_errors": np.max(np.abs(current_diag-previous_diag), axis=0).tolist()}
    reports.append(report)
    print(json.dumps({key: value for key, value in report.items() if key not in {"examples", "diagnostic_max_errors", "model_differences"}}))
(Path(__file__).parent/"vehicle-document-evidence-probe.json").write_text(json.dumps({"producer": str(Path(__file__).relative_to(ROOT)),
    "status": "PHYSICAL_DIFFERENCE", "cases": reports}, indent=2)+"\n", encoding="utf-8")
