"""Declaring tire mass/inertia must not change the answer in this step."""
import importlib.util
from pathlib import Path
import numpy as np
from suspension_contracts import pack_container
from suspension_multibody.cases import axle_dynamic_model_document
from suspension_multibody.cases.axle_dynamic import case_document
from suspension_multibody.simulation import (
    CompilerRegistry, DocumentPairCompiler, SimulationRequest, run_request,
)

spec = importlib.util.spec_from_file_location(
    "acceptance", Path("packages/suspension_multibody/scripts/run_axle_dynamics_acceptance.py"))
acc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(acc)
model = acc.build_axle_model()
case_name = "road_pulse"
document, blob = axle_dynamic_model_document(model)
case, case_blob = case_document(model, acc.build_case(case_name))

registry = CompilerRegistry()
registry.register(DocumentPairCompiler("axle", "axle_dynamic"))

def run(doc):
    return run_request(SimulationRequest(
        assembly="axle", family="axle_dynamic", model=doc, case=case,
        context={"case_payload": pack_container(case, case_blob)}), registry=registry).raw

base = run(document)
patched = {**document, "tires": [
    {**tire, "mass": 12.5,
     "inertia": [[0.4, 0.0, 0.0], [0.0, 0.7, 0.0], [0.0, 0.0, 0.7]]}
    for tire in document["tires"]]}
with_mass = run(patched)
print("tires:", len(document["tires"]))
for name in ("body_state", "diagnostics", "energy"):
    a, b = base.block(name), with_mass.block(name)
    print(name, a.shape, "bit-identical:", np.array_equal(a, b),
          "max|diff|:", float(np.max(np.abs(a - b))) if a.size else 0.0)

a, b = base.block("diagnostics"), with_mass.block("diagnostics")
same = np.array_equal(a, b, equal_nan=True)
print("diagnostics equal_nan:", same)
print("nan count base/patched:", int(np.isnan(a).sum()), int(np.isnan(b).sum()))
diff = np.nanmax(np.abs(a - b))
print("nanmax diff:", float(diff))
