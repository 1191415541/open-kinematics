"""Inspect all model and case fields against the pinned pre-retirement inputs."""

import json
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SCRIPTS = ROOT / "packages/suspension_multibody/scripts"
sys.path.insert(0, str(SCRIPTS))
from vehicle_physical_evidence import compile_evidence, input_differences, reference_case

gate = runpy.run_path(str(SCRIPTS / "case_parity_check.py"))
baseline = gate["_VEHICLE_BASELINE"]
layouts = json.loads(baseline.with_name("entity_layout.json").read_text(encoding="utf-8"))["cases"]
reports = []
for name, (_, case) in gate["_vehicle_case_matrix"](gate["_load_vehicle_fixture"]()).items():
    compiled = compile_evidence(case, layouts[name])
    old_model, old_case, model_blob, case_blob, _, entry = reference_case(baseline.parent / "reference", name, baseline)
    model_delta, case_delta, fingerprints = input_differences(compiled, old_model, old_case, model_blob, case_blob, layouts[name])
    reports.append({"name": name, "model_differences": model_delta, "case_differences": case_delta, "fingerprints": fingerprints})
    print(name, len(model_delta), len(case_delta), flush=True)
(Path(__file__).parent / "vehicle-reference-inputs.json").write_text(json.dumps(reports, indent=2)+"\n", encoding="utf-8")
