"""Isolated-wheel probe: native execution and public API end to end.

The benchmark model is a declarative fixture (07 moved it out of
``analysis/benchmarks.py`` into ``tests/data``), so this probe builds the model
from the same JSON the scripts read, by explicit path.
"""
import json
import pathlib
import tempfile

from suspension_multibody.schema import CaseSpec, FrontAxleModel
from suspension_multibody.preparation.assembly import build_front_axle
from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody import read_artifact, run_case, write_artifact

FIXTURE = pathlib.Path(
    r"C:\杂件\open-kinematics\packages\suspension_multibody\tests\data\benchmark_axle.json"
)
payload = json.loads(FIXTURE.read_text(encoding="utf-8"))
model = FrontAxleModel.model_validate(payload["model"])
print("fixture model:", model.name)

print("\n### 5. native execution")
assembly = build_front_axle(model, "K")
doc = model_document(assembly, name="iso", drive_wheels=True)
case = {
    "contract": "multibody-case", "contract_version": 1, "kind": "case",
    "family": "kc_quasi_static", "name": "iso",
    "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
    "k": {"axes": [{"coordinate": "wheel_drive_L", "values_mm": [0.0]}],
          "drive": "wheel_center"},
}
raw = run_request(
    SimulationRequest(assembly="axle", family="kc_quasi_static", model=doc, case=case)
).raw
print("status:", raw.status, "| cases:", len(raw.cases), "| bodies:", len(raw.body_names))
print("contract_version:", raw.document.get("contract_version"))
assert raw.status == "success" and len(raw.cases) == 1

print("\n### 6. public API end to end (run_case + artifact round trip)")
spec = CaseSpec(name="iso-api", mode="K", controls=())
bundle = run_case(model, spec)
print("run_case OK ->", type(bundle).__name__)
out = pathlib.Path(tempfile.mkdtemp())
path = write_artifact(bundle, out / "art", model=model, case=spec)
loaded = read_artifact(path)
print("artifact_type:", loaded["manifest"]["artifact_type"],
      "| status:", loaded["manifest"]["status"])
assert loaded["manifest"]["status"] == "success"
print("\nALL PROBE CHECKS PASSED")
