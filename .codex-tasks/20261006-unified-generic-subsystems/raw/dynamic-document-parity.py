"""Compare all frozen producers through declared documents before cutover."""
from __future__ import annotations

import hashlib
import json
import runpy
from pathlib import Path

import numpy as np

from suspension_multibody import api
from suspension_multibody.adams.axle_equivalence import _refined_case
from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
from suspension_multibody.axle_dynamics.contract_run import run_axle
from suspension_multibody.simulation.runner import run_compiled


ROOT = Path(__file__).resolve().parents[3]


def main() -> None:
    producer = runpy.run_path(str(ROOT / "packages/suspension_multibody/scripts/run_axle_dynamics_acceptance.py"))
    model = producer["build_axle_model"]()
    evidence = {"producer": "dynamic-document-parity.py", "status": "PASS", "variants": []}
    for name in producer["_CASE_DURATIONS"]:
        case = producer["build_case"](name)
        for variant, active in (("primary", case), ("refined", _refined_case(case))):
            previous = run_axle(model, active)
            assembly, plan = migrate_v1_dynamic_axle(model, active)
            compiled = api.validate(assembly, plan)
            current = run_compiled(compiled).result
            count = len(active.times_s)
            errors = {}
            old_rows, new_rows = [], []
            for body in model.bodies:
                old_rows.append(previous.body_state(body.name))
                new_rows.append(current.body_state(body.name+"."+body.name))
                if not np.array_equal(new_rows[-1], old_rows[-1]):
                    errors["body:"+body.name] = float(np.max(np.abs(new_rows[-1]-old_rows[-1])))
            for tire in model.tires:
                before, after = previous.tire_state(tire.name), current.tire_state(tire.body+"."+tire.name)
                if not np.array_equal(before, after):
                    errors["tire:"+tire.name] = float(np.max(np.abs(before-after)))
            for block in ("constraint_wrench", "spring_output", "damper_output", "bump_stop_output", "bushing_output", "anti_roll_output", "energy", "diagnostics", "contact_events"):
                if block not in previous.raw.blocks and block not in current.named_blocks:
                    continue
                before, after = previous.raw.block(block)[:count], current.raw.block(block)[:count]
                if not np.array_equal(before, after, equal_nan=True):
                    errors[block] = "shape" if before.shape != after.shape else float(np.nanmax(np.abs(before-after)))
            row = {"case": name, "variant": variant, "compiler": compiled.metadata["compiler"],
                "model_fingerprint": current.model_fingerprint, "errors": errors,
                "previous_state_sha256": hashlib.sha256(np.stack(old_rows, axis=1).tobytes()).hexdigest(),
                "document_state_sha256": hashlib.sha256(np.stack(new_rows, axis=1).tobytes()).hexdigest()}
            evidence["variants"].append(row)
            if errors:
                evidence["status"] = "FAILED"
            print(name, variant, "PASS" if not errors else errors, flush=True)
    output = Path(__file__).with_suffix(".json")
    output.write_text(json.dumps(evidence, indent=2), encoding="utf-8")
    if evidence["status"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
