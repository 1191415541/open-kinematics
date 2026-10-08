"""Capture the old ledger order before retirement; no native submission."""

import hashlib
import json
import runpy
from pathlib import Path

from suspension_multibody.authoring import assemble_generic, migrate_v1_vehicle_case
from suspension_multibody.cases.vehicle_dynamic import model_document
from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run

ROOT = Path(__file__).resolve().parents[3]
gate = runpy.run_path(str(ROOT/"packages/suspension_multibody/scripts/case_parity_check.py"))
fixture = gate["_load_vehicle_fixture"]()
cases = gate["_vehicle_case_matrix"](fixture)
layouts = {}


def legacy_name(name):
    if ".sub.json." in name:
        return name.split("_", 1)[0]+"_"+name.split(".json.", 1)[1]
    return name.split(".", 1)[-1]


for name, (model, case) in cases.items():
    prepared = prepare_vehicle_run(model, case)
    old, _ = model_document(model, prepared)
    assembly, _ = migrate_v1_vehicle_case(case)
    graph = assemble_generic(assembly).resolved_model().to_document()
    ids = {legacy_name(row["name"]): row["name"] for row in graph["bodies"]}
    joints = []
    for row in old["joints"]:
        matches = [joint for joint in graph["joints"] if legacy_name(joint["body_a"]) == row["body_a"]
            and legacy_name(joint["body_b"]) == row["body_b"] and joint["type"] == row["type"]]
        if len(matches) != 1:
            matches = [joint for joint in matches if legacy_name(joint["name"]) == row["name"]]
        if len(matches) != 1:
            raise ValueError((name, row, matches))
        joints.append(matches[0]["name"])
    joints.extend(row.get("target", row["name"]) for row in graph["elements"]
        if row["type"] == "steering_actuator" and row["parameters"]["type"].startswith("prescribed"))
    ledger = {}
    for kind in ("spring", "bushing", "anti_roll_bar", "steering_actuator"):
        rows = []
        for row in old["elements"]:
            if row["type"] != kind:
                continue
            matches = [item["name"] for item in graph["elements"] if item["type"] == kind
                and (legacy_name(item["name"]) == row["name"] or
                    kind == "steering_actuator" and legacy_name(item["target"]) == row.get("target"))]
            if len(matches) != 1:
                raise ValueError((kind, row["name"], matches))
            rows.append(matches[0])
        ledger[kind] = rows
    ledger["tire"] = [next(row["name"] for row in graph["tires"] if legacy_name(row["name"]) == tire["name"]) for tire in old["tires"]]
    layouts[name] = {"bodies": [ids[row["name"]] for row in old["bodies"]], "constraints": joints, "ledgers": ledger}
    print(name, len(layouts[name]["bodies"]), len(joints))
baseline = ROOT/"packages/suspension_multibody/tests/data/vehicle_dynamics_baseline/sha256.json"
output = baseline.with_name("entity_layout.json")
payload = {"schema_version": 1, "producer": str(Path(__file__).relative_to(ROOT)),
    "baseline_sha256": hashlib.sha256(baseline.read_bytes()).hexdigest(), "cases": layouts}
output.write_text(json.dumps(payload, indent=2)+"\n", encoding="utf-8")
