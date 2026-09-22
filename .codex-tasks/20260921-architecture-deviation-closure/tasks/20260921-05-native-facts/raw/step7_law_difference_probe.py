"""Measure the Python-constitutive vs native-element-wrench difference (05 step 7).

Reproducible evidence for `raw/step7_law_difference_registration.md`.  The probe
runs the *same* K&C case twice: once through the native element-wrench channel
(switch on) and once through the retained Python constitutive path, and compares
the two per (element, body).

Method notes that make the comparison meaningful rather than a convergence
artefact:

* Both sides are evaluated at the **same state**: the probe takes sample 0 of the
  native run and rebuilds the Python reporting state from that very sample
  (`api._rigid_state`), so poses are not a source of difference.
* Python's reporting state is millimetres/Newtons, native is metres/Newtons, so
  force columns come back in N (no scaling) and moment columns in N*mm, which are
  scaled by 1/1000.  The scaling itself is exact enough (1e-16 relative) to leave
  the differences reported below meaningful.
* Rows whose force and moment columns are all NaN mean "this element applied
  nothing to this body in this sample" (native contract), so they are skipped.

Run: uv run --package suspension-multibody python <this file>
"""

from __future__ import annotations

import os

import numpy as np

from typing import Literal

from suspension_multibody.analysis.benchmarks import benchmark_model
from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.elements import evaluate_generalized_forces
from suspension_multibody.model import build_front_axle
from suspension_multibody.schema import Bushing6x6, Pose, Vec3
from suspension_multibody.simulation import SimulationRequest, run_request
import suspension_multibody.api as api

# The optional channel is off by default; this probe is what asks for it.  The
# native switch re-reads the environment on every call, so setting it before the
# first run is enough.
os.environ["SUSPENSION_KERNEL_ELEMENT_WRENCH_OUTPUT"] = "1"

MM = 1000.0
SAMPLES_PER_CASE = 2


def compliant_model():
    """The synthetic bushing (C) model the K&C snapshot gates use."""
    base = benchmark_model()
    stiffness = tuple(
        tuple(
            10_000.0 if row == column and row < 3
            else 10_000_000.0 if row == column
            else 0.0
            for column in range(6)
        )
        for row in range(6)
    )

    def mount(name: str) -> Pose:
        point = base.hardpoints[name]
        if hasattr(point, "x"):
            x, y, z = float(point.x), float(point.y), float(point.z)
        elif isinstance(point, dict):
            x, y, z = float(point["x"]), float(point["y"]), float(point["z"])
        else:
            x, y, z = (float(v) for v in point)
        return Pose(translation=Vec3(x=x, y=y, z=z))

    bushings = tuple(
        Bushing6x6(
            name=f"{body}_{index}",
            body_a="chassis",
            body_b=body,
            pose_a=mount(name),
            pose_b=mount(name),
            stiffness=stiffness,
        )
        for body, names in (
            ("upper_arm", ("uca_front", "uca_rear")),
            ("lower_arm", ("lca_front", "lca_rear")),
        )
        for index, name in enumerate(names)
    )
    return base.model_copy(update={"bushings": bushings, "name": f"{base.name}_compliant"})


def _case(name: str, mode: str) -> dict:
    envelope = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": name,
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
    }
    if mode == "K":
        return dict(envelope, k={
            "axes": [{"coordinate": "wheel_drive_L", "values_mm": [0.0]}],
            "drive": "wheel_center",
        })
    return dict(envelope, c={
        "load_marker": "wheel_center_L",
        "mirror_marker": "wheel_center_R",
        "side_mode": "single",
        "loads": [{"fx": 0.0, "fy": 0.0, "fz": 1000.0,
                   "mx": 0.0, "my": 0.0, "mz": 0.0}],
    })


def compare(mode: Literal["K", "C"]) -> dict:
    model = benchmark_model() if mode == "K" else compliant_model()
    assembly = build_front_axle(model, mode)
    document = model_document(assembly, name=f"law-{mode}", drive_wheels=(mode == "K"))
    raw = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=document,
            case=_case(f"law-{mode}", mode),
        )
    ).raw

    try:
        block = raw.named_blocks["element_wrench"]
    except KeyError as error:
        raise SystemExit(f"{mode}: element_wrench block is missing ({error})") from error

    names = list(raw.body_names)
    print(f"\n########## mode {mode} ##########")
    print("contract_version:", raw.document.get("contract_version"))
    print("element_wrench shape:", block.shape)

    flat = block[0].reshape(-1, 13)
    finite_rows = [row for row in flat if not np.isnan(row[0:6]).any()]
    counts: dict[int, int] = {}
    for row in finite_rows:
        counts[int(row[6])] = counts.get(int(row[6]), 0) + 1
    print("finite rows per type code:", counts)

    # structural finding: does native ever give a row whose receiving body is fixed?
    fixed = [names[int(r[12])] for r in flat if int(r[6]) == 7][:1]
    print("external rows receiving body (first):", fixed)

    physical = api._rigid_state(assembly, names, raw.case_body_state(0))
    _force, evaluations = evaluate_generalized_forces(
        physical,
        assembly.elements,
        body_order=tuple(n for n, b in physical.bodies.items() if not b.fixed),
    )
    per_element = {ev.name: ev.body_wrenches_global for ev in evaluations}

    # native bushing group, indexed as (element, end): end 0 = body b, end 1 = body a
    native_bushings = [r for r in flat if int(r[6]) == 2]
    bush_elements = [e for e in assembly.elements if type(e).__name__ == "BushingElement"]

    print(f"\nnative bushing rows={len(native_bushings)} "
          f"python bushing elements={len(bush_elements)}")
    print(f"{'element':26s} {'end':>3s} {'body':13s} "
          f"{'|dF|max':>11s} {'|dM|max':>11s} {'|dM|/|M|':>10s}")

    summary = {"force_max": 0.0, "moment_max": 0.0, "moment_rel_max": 0.0}
    fixed_body_missing: list[str] = []
    for index, element in enumerate(bush_elements):
        for end in (0, 1):
            position = 2 * index + end
            if position >= len(native_bushings):
                print(f"{element.name:26s} {end:3d} <no native row>")
                continue
            row = native_bushings[position]
            body = names[int(row[12])]
            python = per_element.get(element.name, {}).get(body)
            if python is None:
                print(f"{element.name:26s} {end:3d} {body:13s} "
                      f"{'-':>11s} {'-':>11s} {'-':>10s}  <python: no wrench for this body>")
                fixed_body_missing.append(f"{element.name}/{body}")
                continue
            d_force = float(np.max(np.abs(row[0:3] - python[0:3])))
            d_moment = float(np.max(np.abs(row[3:6] - python[3:6] / MM)))
            reference = float(np.max(np.abs(row[3:6]))) or 1.0
            summary["force_max"] = max(summary["force_max"], d_force)
            summary["moment_max"] = max(summary["moment_max"], d_moment)
            summary["moment_rel_max"] = max(summary["moment_rel_max"], d_moment / reference)
            print(f"{element.name:26s} {end:3d} {body:13s} "
                  f"{d_force:11.6g} {d_moment:11.6g} {d_moment / reference:10.3g}")

    print(f"\nworst force diff  = {summary['force_max']:.6g} N")
    print(f"worst moment diff = {summary['moment_max']:.6g} N*m "
          f"(max relative {summary['moment_rel_max']:.3g})")
    print("python-only (body absent from native rows):",
          sorted(set(fixed_body_missing)))

    # the fixed body's own rows: native NaN vs python finite
    if "chassis" in names:
        chassis_rows = [r for r in flat if int(r[6]) == 2 and names[int(r[12])] == "chassis"]
        nan_rows = [r for r in chassis_rows if np.isnan(r[0:6]).any()]
        print(f"native rows receiving 'chassis': {len(chassis_rows)}, "
              f"of which all-NaN (no force applied): {len(nan_rows)}")
        python_chassis = {
            name: wrench
            for name, wrenches in per_element.items()
            for body, wrench in wrenches.items()
            if body == "chassis" and np.any(wrench != 0.0)
        }
        print("python evaluations reporting a non-zero wrench on 'chassis':",
              len(python_chassis))
    return summary


def main() -> None:
    for mode in ("K", "C"):
        compare(mode)


if __name__ == "__main__":
    main()
