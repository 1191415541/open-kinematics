"""Read-only legacy oracle: isolate ordering from corrected contact frames."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "packages/suspension_multibody"))

from suspension_contracts import pack_container
from tests.vehicle.test_native_vehicle import _case, _vehicle

from suspension_multibody.authoring import AssemblyDocument, assemble_generic
from suspension_multibody.authoring.migration import migrate_v1_case, migrate_v1_vehicle
from suspension_multibody.cases.vehicle_dynamic import case_document, model_document
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
from suspension_multibody.schema import TimeSignal
from suspension_multibody.simulation import SimulationRequest, run_compiled
from suspension_multibody.simulation.compiler import compile_document_pair


def legacy_id(name):
    if ".sub.json." in name:
        return name.split("_", 1)[0] + "_" + name.split(".json.", 1)[1]
    return name.split(".", 1)[-1]


parser = argparse.ArgumentParser()
parser.add_argument("--require-equivalence", action="store_true")
args = parser.parse_args()
vehicle = _vehicle()
case = _case(vehicle, steering=TimeSignal(times=(0, .001), values=(0, 1)))
prepared = prepare_vehicle_run(vehicle, case)
old_model, old_blob = model_document(vehicle, prepared)
old_case, old_case_blob = case_document(vehicle, case, prepared)
old_run = run_compiled(compile_document_pair(SimulationRequest(assembly="vehicle", family="vehicle_dynamic"),
    model_document=old_model, case_document=old_case,
    model_payload=pack_container(old_model, old_blob), case_payload=pack_container(old_case, old_case_blob)))
assembly = migrate_v1_vehicle(vehicle)
assembly = AssemblyDocument.from_payload({**assembly.to_payload(), "gravity": [0, 0, 0]},
    subsystems={entry.ref: entry.subsystem for entry in assembly.entries})
resolved = assemble_generic(assembly).resolved_model()
graph = resolved.to_document()
body_ids = {legacy_id(row["name"]): row["name"] for row in graph["bodies"]}
ids = {"body:"+name: value for name, value in body_ids.items()}
ids.update({row["name"].split(".", 1)[1]: row["name"] for row in graph["tires"]})
ids.update({row["target"].split(".", 1)[1]: row["target"] for row in graph["elements"] if row["type"] == "steering_actuator"})
plan = ResolvedSolvePlan(migrate_v1_case(old_case, entity_ids=ids, input_payload=old_case_blob).to_payload())
old_order = {row["name"]: index for index, row in enumerate(old_model["bodies"])}
joint_order = {(row["body_a"], row["body_b"], row["type"]): index for index, row in enumerate(old_model["joints"])}
report = {"producer": str(Path(__file__).relative_to(ROOT)), "status": "FAILED", "variants": []}
for variant in ("original", "body_order", "body_joint_order", "old_contact_frame"):
    candidate = json.loads(json.dumps(graph))
    if variant != "original":
        candidate["bodies"].sort(key=lambda row: old_order[legacy_id(row["name"])])
    if variant in {"body_joint_order", "old_contact_frame"}:
        candidate["joints"].sort(key=lambda row: joint_order[(legacy_id(row["body_a"]), legacy_id(row["body_b"]), row["type"])])
    if variant == "old_contact_frame":
        for tire, old_tire in zip(candidate["tires"], old_model["tires"]):
            tire["parameters"]["frame_body"] = body_ids[old_tire["parameters"]["frame_body"]]
            tire["parameters"]["frame_center_local"] = old_tire["parameters"]["frame_center_local"]
    run = run_compiled(compile_resolved(ResolvedModel(candidate, resolved.resource_payload), plan))
    errors = np.stack([np.max(np.abs(run.raw.body_state(body_ids[name])-old_run.raw.body_state(name)), axis=0) for name in prepared.body_names])
    row = {"variant": variant, "max_column_errors": np.max(errors, axis=0).tolist(), "max_error": float(errors.max())}
    report["variants"].append(row)
    print(json.dumps(row))
report["status"] = "PHYSICAL_CHANGE" if report["variants"][3]["max_error"] == 0 and report["variants"][2]["max_error"] > 1e-11 else "FAILED"
(Path(__file__).parent / "vehicle-order-probe.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
if args.require_equivalence and report["variants"][2]["max_error"] > 1e-11:
    raise SystemExit(1)
