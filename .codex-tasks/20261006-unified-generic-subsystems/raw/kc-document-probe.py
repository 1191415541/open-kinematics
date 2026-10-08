"""Read-only K/C oracle: isolate changes in frames, inertia and spin constraints."""
import json
import runpy
from copy import deepcopy
from dataclasses import replace
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
from suspension_contracts import pack_container, unpack_container

from suspension_multibody.api import validate
from suspension_multibody.authoring import migrate_v1_kc_case
from suspension_multibody.cases.kc_quasi_static import case_document, model_document
from suspension_multibody.schema.solver import AxleSolverSettings
from suspension_multibody.simulation import SimulationRequest, run_compiled, run_request
from suspension_multibody.subsystems.entry import compose_axle

ROOT = Path(__file__).resolve().parents[3]
FIXTURE = runpy.run_path(str(ROOT/"packages/suspension_multibody/tests/cases/kc_quasi_static/kc_fixtures.py"))
records = []
for mode in ("K", "C"):
    source = FIXTURE["benchmark_model" if mode == "K" else "_compliant_model"]()
    arguments = {"wheel_values_mm": (-10., 0., 10.), "rack_values_mm": (-5., 0., 5.)} if mode == "K" else {
        "paths": ("fx", "fy", "fz", "mx", "my", "mz"), "levels": 3, "maximum": 1.}
    previous = compose_axle(source, mode)
    oracle_model = model_document(previous, name="oracle", drive_wheels=mode == "K")
    oracle_case = case_document(previous, family="kc_quasi_static", name="oracle", drive_wheels=mode == "K",
        times_s=(0, .001), settings=AxleSolverSettings(), **arguments)
    expected = run_request(SimulationRequest(assembly="axle", family="kc_quasi_static", model=oracle_model, case=oracle_case)).raw
    declaration, case = migrate_v1_kc_case(source, mode=mode, **arguments)
    compiled = validate(declaration, case)
    for variant in ("ordinary", "fixed_oracle", "old_inertia"):
        model = deepcopy(compiled.model_document)
        run, payload = unpack_container(compiled.case_payload)
        if variant == "fixed_oracle":
            model["joints"] = [row for row in model["joints"] if not row["name"].startswith("hold:")]
            for joint in model["joints"]:
                if "wheel_spin_joint" in joint["name"]:
                    joint["type"] = "fixed"
            run.get("inputs", {}).pop("motion", None)
            run["blobs"] = [row for row in run.get("blobs", ()) if not row.get("coordinate", "").startswith("hold:")]
        if variant == "old_inertia":
            old = {row["name"]: row for row in oracle_model["bodies"]}
            for body in model["bodies"]:
                name = body["name"].split(".json.")[-1]
                name = "ground" if name == "chassis" else name
                if name in old:
                    body["inertia"] = old[name]["inertia"]
        actual = run_compiled(replace(compiled, model_document=model, model_payload=pack_container(model),
            case_document=run, case_payload=pack_container(run, payload))).raw
        errors = {}
        for index, name in enumerate(expected.body_names):
            target = "chassis" if name == "ground" else name
            new_body = next(row for row in model["bodies"] if row["name"].endswith(".json."+target))
            old_body = next(row for row in oracle_model["bodies"] if row["name"] == name)
            offset = Rotation.from_quat(old_body["quaternion"], scalar_first=True).inv().apply(np.asarray(new_body["position"])-old_body["position"])
            states = expected.states[:, index, :].copy()
            states[:, :3] += Rotation.from_quat(states[:, 3:7], scalar_first=True).apply(np.broadcast_to(offset, (len(states), 3)))
            delta = actual.states[:, actual.body_names.index(new_body["name"]), :] - states
            errors[name] = {key: float(np.max(np.abs(delta[:, begin:end]))) for key, begin, end in (
                ("position_m", 0, 3), ("quaternion", 3, 7), ("velocity_m_s", 7, 10), ("omega_rad_s", 10, 13),
                ("acceleration_m_s2", 13, 16), ("alpha_rad_s2", 16, 19))}
        records.append({"mode": mode, "variant": variant, "errors": errors})
        print(mode, variant, {key: max(row[key] for row in errors.values()) for key in next(iter(errors.values()))})
(Path(__file__).parent/"kc-document-probe.json").write_text(json.dumps(records, indent=2)+"\n", encoding="utf-8")
