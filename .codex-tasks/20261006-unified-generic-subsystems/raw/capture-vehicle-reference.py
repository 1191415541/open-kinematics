"""Capture pre-retirement evidence once and verify every original frozen hash."""

import hashlib
import json
import runpy
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from suspension_contracts import pack_container

from suspension_multibody.cases.vehicle_dynamic import case_document, model_document
from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
from suspension_multibody.simulation import SimulationRequest, run_compiled
from suspension_multibody.simulation.compiler import compile_document_pair

ROOT = Path(__file__).resolve().parents[3]
DATA = ROOT / "packages/suspension_multibody/tests/data/vehicle_dynamics_baseline"
DESTINATION = DATA / "reference"
gate = runpy.run_path(str(ROOT / "packages/suspension_multibody/scripts/case_parity_check.py"))
baseline = json.loads((DATA / "sha256.json").read_text(encoding="utf-8"))["cases"]
layouts = json.loads((DATA / "entity_layout.json").read_text(encoding="utf-8"))["cases"]
manifest = {"schema_version": 1, "producer": str(Path(__file__).relative_to(ROOT)),
    "baseline_sha256": hashlib.sha256((DATA / "sha256.json").read_bytes()).hexdigest(), "cases": {}}
DESTINATION.mkdir(exist_ok=True)
for index, (name, (model, case)) in enumerate(gate["_vehicle_case_matrix"](gate["_load_vehicle_fixture"]()).items()):
    prepared = prepare_vehicle_run(model, case)
    model_doc, model_blob = model_document(model, prepared)
    case_doc, case_blob = case_document(model, case, prepared)
    compiled = compile_document_pair(SimulationRequest(assembly="vehicle", family="vehicle_dynamic"),
        model_document=model_doc, case_document=case_doc, model_payload=pack_container(model_doc, model_blob),
        case_payload=pack_container(case_doc, case_blob))
    raw = run_compiled(compiled).raw
    fields = ("states", "constraint_wrench", "spring_output", "bushing_output", "anti_roll_output", "tire_output", "energy", "steering_output")
    widths = {"spring": 4, "bushing": 12, "anti_roll_bar": 3, "tire": 41, "steering_actuator": 4}
    arrays = {"states": raw.states, "constraint_wrench": raw.block("constraint_wrench"), "energy": raw.block("energy")}
    for kind, width in widths.items():
        block = kind.replace("anti_roll_bar", "anti_roll").replace("steering_actuator", "steering")+"_output"
        count = len(layouts[name]["ledgers"][kind])
        arrays[block] = raw.block(block) if count else np.zeros((len(raw.times_s), 0, width))
    digests = {field: gate["_array_digest"](array) for field, array in arrays.items()}
    diagnostic = raw.block("diagnostics")[:len(raw.times_s)]
    integers = {"internal_steps", "rejected_attempts", "newton_iterations", "active_contacts", "contact_events", "failure_code", "pinned_null_directions"}
    typed = SimpleNamespace(**{field: diagnostic[:, column].astype(bool) if field == "accepted" else
        diagnostic[:, column].astype(int) if field in integers else diagnostic[:, column]
        for column, field in enumerate(gate["_DIAGNOSTIC_FIELDS"])})
    digests["diagnostics"] = gate["_vehicle_diagnostics_digest"](typed)
    if digests != baseline[name]:
        raise ValueError(f"{name}: original frozen hashes differ; capture refused")
    arrays.update(diagnostics=diagnostic, times_s=raw.times_s)
    prefix = str(index)
    native_model = DESTINATION / (prefix+".model.json")
    native_case = DESTINATION / (prefix+".case.json")
    native_model.write_text(json.dumps(model_doc, indent=2)+"\n", encoding="utf-8")
    native_case.write_text(json.dumps(case_doc, indent=2)+"\n", encoding="utf-8")
    (DESTINATION / (prefix+".model.bin")).write_bytes(model_blob)
    (DESTINATION / (prefix+".case.bin")).write_bytes(case_blob)
    np.savez_compressed(DESTINATION / (prefix+".npz"), **arrays)
    ids = dict(zip(raw.body_names, layouts[name]["bodies"]))
    ids.update(dict(zip((row["name"] for row in raw.model_document["joints"]), layouts[name]["constraints"])))
    for kind, names in layouts[name]["ledgers"].items():
        rows = model_doc["tires"] if kind == "tire" else [row for row in model_doc["elements"] if row["type"] == kind]
        ids.update(dict(zip((row["name"] for row in rows), names)))
    ids.update({channel.channel_name: "steering_"+channel.channel_name+"."+channel.channel_name
        for channel in (model.steering, *model.steering_channels) if channel.enabled})
    manifest["cases"][name] = {"prefix": prefix, "source_sha256": hashlib.sha256(case.model_dump_json().encode()).hexdigest(),
        "ids": ids, "files": {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in DESTINATION.glob(prefix+".*")}}
    print(name, "all original hashes verified", len(raw.body_names), flush=True)
(DESTINATION / "manifest.json").write_text(json.dumps(manifest, indent=2)+"\n", encoding="utf-8")
