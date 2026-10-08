"""
Independent final acceptance for the composable multibody architecture.

This runner does not read the subtask logs and does not trust a subtask being
DONE.  It reconstructs each acceptance scenario from the repo's own public
surface and runs it, then separately re-runs the frozen command set.  A subtask
that recorded a pass and a tree that still passes are two different claims, and
this file is the second one.

Two kinds of check, and the difference matters:

``probes``
    In-process scenarios, written here rather than reused from a subtask's test
    file, so an error in that test cannot hide an error in the tree.  Each probe
    returns the evidence it gathered, and a probe that cannot run **fails**
    rather than being skipped.
``commands``
    The baseline command set from ``tasks/01-baseline/COMMANDS.json``, executed
    as-is, one subprocess each.

The exit status is the verdict: any failed probe or any non-zero command fails
the run.  ``--strict`` additionally refuses a missing prerequisite (no native
library, no built C++ self-test) instead of reporting it as unavailable, and
refuses a skipped or xfailed test in the scenario selections, because a skipped
acceptance scenario is a scenario that did not happen.

    uv run --no-sync python packages/suspension_multibody/scripts/accept_composable_architecture.py --strict
    uv run --no-sync python packages/suspension_multibody/scripts/accept_composable_architecture.py --list
"""

from __future__ import annotations

import argparse
import json
import math
import re
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

from suspension_multibody.results.envelope import ResultEnvelope

ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = ROOT / "packages" / "suspension_multibody"
SOURCE_ROOT = PACKAGE_ROOT / "src" / "suspension_multibody"
BASELINE_COMMANDS = (
    ROOT / ".codex-tasks" / "multibody-composable-architecture"
    / "tasks" / "01-baseline" / "COMMANDS.json"
)
KERNEL_BUILD = ROOT / "packages" / "suspension_kernel" / "build"

#: The pytest selections each A-item owns.  These are the *scenario* files; the
#: whole-suite command is in the frozen command set and runs separately, so a
#: failure here and a failure there stay distinguishable.
SCENARIO_TESTS: dict[str, tuple[str, ...]] = {
    "A1": (
        "packages/suspension_multibody/tests/architecture/test_import_boundaries.py",
        "packages/suspension_multibody/tests/architecture/test_dependency_boundaries.py",
        "packages/suspension_multibody/tests/architecture/test_module_layering_gate.py",
    ),
    "A2": (
        "packages/suspension_multibody/tests/composable",
        "packages/suspension_multibody/tests/rigs",
    ),
    "A3": ("packages/suspension_multibody/tests/composable",),
    "A4": (
        "packages/suspension_multibody/tests/connections",
        "packages/suspension_multibody/tests/composable",
    ),
    "A5": (
        "packages/suspension_multibody/tests/subsystems",
        "packages/suspension_multibody/tests/vehicle_assembly",
        "packages/suspension_multibody/tests/tire_mass",
        "packages/suspension_multibody/tests/modeling",
    ),
    "A6": (
        "packages/suspension_multibody/tests/simulation",
        "packages/suspension_multibody/tests/studies",
    ),
    "A7": (
        "packages/suspension_kernel/tests/test_registry_consistency.py",
        "packages/suspension_kernel/tests",
        "packages/suspension_contracts/tests",
    ),
    "A8": (
        "packages/suspension_multibody/tests/api",
        "packages/suspension_multibody/tests/cases",
        "packages/suspension_multibody/tests/subsystems",
    ),
    "A9": (
        "packages/suspension_multibody/tests/e2e",
        "packages/suspension_multibody/tests/cli",
        "packages/suspension_multibody/tests/io",
        "packages/suspension_multibody/tests/results",
        "packages/suspension_multibody/tests/outputs",
        "packages/suspension_multibody/tests/metrics",
        "packages/suspension_multibody/tests/physics",
    ),
    "A10": ("packages/suspension_multibody/tests/architecture",),
}

#: The frozen command that produces the native library in both packages.
NATIVE_COMMAND = "build_axle_native"

#: The two mirrors the native library lives at, which must agree.
NATIVE_MIRRORS: tuple[Path, ...] = (
    ROOT / "packages" / "suspension_kernel" / "src" / "suspension_kernel"
    / "native" / "suspension_kernel.dll",
    ROOT / "packages" / "suspension_multibody" / "src" / "suspension_multibody"
    / "native" / "suspension_kernel.dll",
)

#: The C++ self-tests A7 needs built, and the output each prints.
KERNEL_SELF_TESTS: tuple[tuple[str, str], ...] = (
    ("mb_contract_selftest", "mb_contract selftest"),
    ("mb_cases_selftest", "mb_cases selftest"),
    ("mb_tire_state_selftest", "mb_tire selftest"),
)


@dataclass
class Outcome:
    """One acceptance item and what happened to it."""

    identifier: str
    title: str
    ok: bool = False
    evidence: list[str] = field(default_factory=list)
    problems: list[str] = field(default_factory=list)

    def report(self) -> str:
        mark = "PASS" if self.ok else "FAIL"
        lines = [f"[{mark}] {self.identifier}  {self.title}"]
        lines.extend(f"        · {item}" for item in self.evidence)
        lines.extend(f"        ! {item}" for item in self.problems)
        return "\n".join(lines)


class AcceptanceError(RuntimeError):
    """An acceptance item could not be constructed, which is a failure."""


# --------------------------------------------------------------------------- #
# the probes
# --------------------------------------------------------------------------- #


def _benchmark_payload() -> dict[str, Any]:
    """Return the shared benchmark axle fixture's model section."""
    path = PACKAGE_ROOT / "tests" / "data" / "benchmark_axle.json"
    return json.loads(path.read_text(encoding="utf-8"))["model"]


def _benchmark_model(*, massed: bool = False):
    """Return the shared benchmark axle as a model, optionally with inertia."""
    from suspension_multibody.schema.model import AxleDeclaration

    raw = dict(_benchmark_payload())
    if massed:
        names = (
            "rack", "upper_arm_L", "lower_arm_L", "upright_L", "tie_rod_L",
            "upper_arm_R", "lower_arm_R", "upright_R", "tie_rod_R",
        )
        inertia = [[100.0, 0.0, 0.0], [0.0, 100.0, 0.0], [0.0, 0.0, 100.0]]
        raw["bodies"] = [
            {"name": name, "mass": 100.0, "inertia": inertia} for name in names
        ]
    return AxleDeclaration.model_validate(raw)


def probe_a1() -> Outcome:
    """
    A1: import order is not load-bearing, and both layering gates pass.

    The probe imports the package in several *orders* in fresh interpreters and
    compares what came along.  The gate commands themselves run in the frozen
    command set; what is checked here is the property those gates exist to
    protect, measured directly.
    """
    outcome = Outcome("A1", "import order is not load-bearing")
    entry_points = (
        "suspension_multibody",
        "suspension_multibody.modeling",
        "suspension_multibody.modeling.primitives",
        "suspension_multibody.templates",
        "suspension_multibody.compilation",
        "suspension_multibody.simulation",
        "suspension_multibody.api",
        "suspension_multibody.rigs",
        "suspension_multibody.subsystems",
        "suspension_multibody.connections",
        "suspension_multibody.results",
    )
    orders = [
        entry_points,
        tuple(reversed(entry_points)),
        tuple(sorted(entry_points, key=len)),
        (entry_points[5], entry_points[0], entry_points[10], *(e for e in entry_points[1:5])),
    ]
    #: The layers the low layer must never reach.  Importing an *upper* entry
    #: point may of course load them; what must not happen is the low layer
    #: reaching them, which is what the dependency direction means.
    forbidden = (
        "suspension_multibody.templates",
        "suspension_multibody.subsystems",
        "suspension_multibody.rigs",
        "suspension_multibody.connections",
        "suspension_multibody.preparation",
        "suspension_multibody.simulation",
        "suspension_multibody.kernel",
        "suspension_multibody.report",
    )
    for index, order in enumerate(orders):
        script = (
            "import importlib, sys\n"
            f"for name in {order!r}:\n"
            "    importlib.import_module(name)\n"
            "low = 'suspension_multibody.modeling.primitives'\n"
            "assert low in sys.modules, 'the low layer did not import'\n"
            "print('LOADED', ','.join(sorted(m for m in sys.modules "
            "if m.startswith('suspension_multibody'))))\n"
        )
        completed = subprocess.run(
            [sys.executable, "-c", script],
            cwd=str(ROOT), capture_output=True, text=True, check=False,
        )
        if completed.returncode != 0:
            outcome.problems.append(
                f"order {index} failed: {completed.stderr.strip().splitlines()[-1]}"
            )

    # The case that actually matters is each low-layer module imported *alone*:
    # measured on its own, an upper entry point's own imports cannot mask it.
    low_only = (
        "suspension_multibody.modeling",
        "suspension_multibody.modeling.primitives",
        "suspension_multibody.modeling.primitives.spatial",
        "suspension_multibody.modeling.primitives.joints",
        "suspension_multibody.modeling.instance",
        "suspension_multibody.modeling.ports",
        "suspension_multibody.modeling.identity",
        "suspension_multibody.modeling.units",
    )
    for module in low_only:
        script = (
            "import importlib, sys\n"
            f"importlib.import_module({module!r})\n"
            "print('LOADED', ','.join(sorted(m for m in sys.modules "
            "if m.startswith('suspension_multibody'))))\n"
        )
        completed = subprocess.run(
            [sys.executable, "-c", script],
            cwd=str(ROOT), capture_output=True, text=True, check=False,
        )
        if completed.returncode != 0:
            outcome.problems.append(
                f"importing {module} alone failed: "
                f"{completed.stderr.strip().splitlines()[-1]}"
            )
            continue
        loaded = completed.stdout.strip().split("LOADED ", 1)[-1].split(",")
        dragged = [name for name in loaded if name in forbidden]
        if dragged:
            outcome.problems.append(f"importing {module} alone pulled in {dragged}")
    if not outcome.problems:
        outcome.evidence.append(
            f"{len(orders)} import orders succeeded and {len(low_only)} low-layer "
            "modules imported alone; none reached an upper layer"
        )
    outcome.ok = not outcome.problems
    return outcome


def probe_a2() -> Outcome:
    """
    A2: a topology nobody wrote a branch for, solved both ways.

    The synthetic trailing-arm axle's connection graph differs structurally from
    the double wishbone (one arm per side, two constraints against thirteen).
    Both readings are run here for real: the quasi-static one through the public
    entry, and the dynamic one through the compile-and-submit pipeline.  Task 11
    proved the dynamic *conversion* only, so the solve is the part that is new
    evidence.
    """
    outcome = Outcome("A2", "a new topology solves in both readings")
    import numpy as np

    from suspension_multibody.api import simulate, validate
    from suspension_multibody.authoring.migration import (
        migrate_v1_axle,
        migrate_v1_kc_case,
    )
    from suspension_multibody.compilation.resolved import compile_resolved
    from suspension_multibody.modeling.resolved import ResolvedSolvePlan
    from suspension_multibody.schema.model import AxleDeclaration
    from suspension_multibody.simulation import run_compiled

    fixture = json.loads(
        (PACKAGE_ROOT / "tests" / "data" / "composable"
         / "synthetic_trailing_arm_axle.json").read_text(encoding="utf-8")
    )
    if fixture.get("_synthetic") is not True:
        outcome.problems.append("the synthetic fixture is not marked _synthetic")
    synthetic = AxleDeclaration.model_validate(fixture["model"])
    wishbone = _benchmark_model()

    # The graph difference, asserted rather than assumed.
    synthetic_document = migrate_v1_axle(synthetic)
    wishbone_document = migrate_v1_axle(wishbone)
    neutral = {"schema_version": 1, "name": "topology", "study": "dynamic",
        "samples": [0, .001], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}
    synthetic_assembly = validate(synthetic_document, neutral).request.model
    wishbone_assembly = validate(wishbone_document, neutral).request.model
    synthetic_joints = synthetic_assembly.to_document()["joints"]
    wishbone_joints = wishbone_assembly.to_document()["joints"]
    synthetic_count = len(synthetic_joints)
    # The two explicitly declared wheel bearings supplement the old 13-row graph.
    wishbone_count = len(wishbone_joints)
    kinds = {row["type"] for row in synthetic_joints}
    if (synthetic_count, wishbone_count, kinds) != (2, 16, {"revolute"}):
        outcome.problems.append(
            f"the graph is not the documented difference: "
            f"{synthetic_count} vs {wishbone_count} constraints, kinds {kinds}"
        )
    outcome.evidence.append(
        f"graph: {synthetic_count} constraints vs double wishbone {wishbone_count}, "
        f"kinds {sorted(kinds)}"
    )

    # The quasi-static reading, through the public entry, against an
    # independently derived rotation.
    expected = fixture["expected"]
    travels = tuple(expected["wheel_travel_mm"])
    assembly, case = migrate_v1_kc_case(synthetic, mode="K", wheel_values_mm=travels,
        drive_mode="kinematics")
    run = simulate(assembly, case.to_payload())
    assert isinstance(run.result, ResultEnvelope)
    solved = [float(run.result.frame_pose("wheel.sub.json.wheel_center_L")[
        run.result.case_samples(index).stop-1, 2, 3])*1000 for index in range(len(run.result.cases))]
    slope = float(expected["wheel_centre_dz_per_radian_mm"])
    pivot = next(
        joint for joint in synthetic.joints if joint.name.startswith("arm_pivot")
    )
    pivot_z = float(pivot.point_a.z)
    offset = expected["wheel_centre_offset_from_pivot_mm"]
    predicted = []
    for travel in travels:
        theta = travel / slope
        import math

        predicted.append(
            pivot_z
            + (
                -float(offset[0]) * math.sin(theta)
                + float(offset[2]) * math.cos(theta)
            )
        )
    worst = max(abs(a - b) for a, b in zip(solved, predicted))
    if worst > 5e-2:
        outcome.problems.append(
            f"quasi-static wheel-centre height differs from the derived rotation "
            f"by {worst:.4f} mm"
        )
    if run.status != "success":
        outcome.problems.append("a quasi-static state did not converge")
    outcome.evidence.append(
        f"quasi-static: {len(run.result.cases)} states converged, worst deviation from "
        f"the derived rotation {worst:.4f} mm"
    )

    # The dynamic reading, solved for real.  The static initialisation of this
    # assembly needs more Newton iterations than the default, which is a solver
    # setting and not a model change.
    plan = ResolvedSolvePlan({**neutral, "name": "trailing-arm", "protocol": "axle_dynamic",
        "samples": [0, .001, .002], "solver": {"max_newton_iterations": 100}})
    run = run_compiled(compile_resolved(synthetic_assembly, plan))
    assert isinstance(run.result, ResultEnvelope)
    states = run.raw.states
    if run.status != "success" or states.size == 0:
        outcome.problems.append(
            f"the dynamic reading did not solve: {run.status} {run.raw.failure_evidence}"
        )
    else:
        if not bool(np.isfinite(states).all()):
            outcome.problems.append("the dynamic state is not finite")
        body = run.result.body_ids.index("wheel.sub.json.upright_L")
        outcome.evidence.append(
            f"dynamic: {len(run.raw.cases)} case(s), {states.shape[0]} samples, "
            f"upright_L z {states[-1, body, 2]:.6f} m, finite "
            f"{bool(np.isfinite(states).all())}"
        )
    outcome.ok = not outcome.problems
    return outcome


def probe_a3() -> Outcome:
    """
    A3: a new bench contributes real entities, and probes the extension boundary.

    The contributed-entity half is checked from the bench's own declaration and
    then in a solve.  The boundary half is what task 11 recorded but did not
    test: the hashes of the central files are captured, the extension is
    performed, and the hashes are compared -- a fixture that had to edit the
    compiler to work would show up here.
    """
    outcome = Outcome("A3", "a new bench contributes entities, without touching core")
    import hashlib

    from suspension_multibody import api
    from suspension_multibody.authoring import (
        AssemblyDocument,
        SubsystemDocument,
        TemplateDocument,
    )
    from suspension_multibody.authoring.properties import ElementPropertyDocument

    core = (
        "authoring/loader.py",
        "authoring/generic.py",
        "compilation/resolved.py",
        "compilation/motion.py",
        "simulation/runner.py",
        "modeling/resolved.py",
        "results/envelope.py",
    )

    def digests() -> dict[str, str]:
        return {
            name: hashlib.sha256((SOURCE_ROOT / name).read_bytes()).hexdigest()
            for name in core
        }

    bench = json.loads(
        (PACKAGE_ROOT / "tests" / "data" / "composable"
         / "synthetic_load_bench.json").read_text(encoding="utf-8")
    )
    if bench.get("_synthetic") is not True:
        outcome.problems.append("the synthetic bench is not marked _synthetic")
    declared = bench["expected"]
    bodies = [item["name"] for item in bench["bodies"]]
    joints = [item["name"] for item in bench["joints"]]
    forces = [item["name"] for item in bench["forces"]]
    if (bodies, joints, forces) != (
        declared["contributes_bodies"],
        declared["contributes_joints"],
        declared["contributes_forces"],
    ):
        outcome.problems.append("the bench's declaration does not match its expectation")
    outcome.evidence.append(
        f"bench declares {len(bodies)} bodies, {len(joints)} joints, "
        f"{len(forces)} forces"
    )

    before = digests()
    points, rows, elements = {}, [], []
    for item in (*bench["joints"], *bench["forces"]):
        row = {"name": item["name"], "type": item["kind"],
            "body_a": item["body_a"], "body_b": item["body_b"]}
        for end in ("a", "b"):
            key = item["name"]+"_"+end
            points[key] = item["point_"+end+"_m"]
            rows.append({"name": key, "owner": item["body_"+end], "space": "body"})
            row["point_"+end] = key
        if item in bench["joints"]:
            row.update(axis=item["axis_a"], axis_b=item["axis_b"], axis_space="body")
        else:
            row["parameters"] = ({"stiffness": item["stiffness_n_per_m"], "free_length": item["free_length_m"]}
                if item["kind"] == "spring" else {"viscous_damping": item["viscous_damping_n_s_per_m"]})
            row["property_slot"] = item["name"]
            elements.append(row)
            continue
        bench.setdefault("declared_joints", []).append(row)
    rows.extend([{"name": "origin", "owner": "bench_frame", "space": "body"},
        {"name": "input", "owner": "load_carriage", "space": "body"}])
    points.update(origin=[0, 0, 0], input=[0, 0, 0])
    joints = bench["declared_joints"] + [{"name": "motion_load_travel", "type": "driven_translation",
        "body_a": "load_carriage", "body_b": "bench_frame", "point_a": "input", "point_b": "origin",
        "axis": [0, 0, 1], "axis_b": [0, 0, 1], "axis_space": "body", "target": "load_travel"}]
    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "loading_bench", "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"},
        "bodies": [{"name": item["name"], "fixed": item["fixed"], "mass": item["mass"],
            "inertia": item["inertia"], "position": item["position_m"], "quaternion": item["quaternion"]}
            for item in bench["bodies"]], "hardpoints": rows, "joints": joints,
        "elements": elements, "ports": [], "property_slots": [
            {"name": item["name"], "element_type": item["kind"], "required": True}
            for item in bench["forces"]]})
    subsystem = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": "loading_bench", "template": "bench.tpl", "functional_role": "generic",
        "placement_role": "any", "hardpoints": points,
        "property_bindings": {item["name"]: item["name"] for item in bench["forces"]}}, template=template,
        properties={item["name"]: ElementPropertyDocument.from_payload({
            "document": "element_properties", "schema_version": 1, "name": item["name"],
            "element_type": item["kind"], "model": "linear", "units": {"length": "m", "force": "N"},
            "parameters": ({"stiffness": item["stiffness_n_per_m"], "free_length": item["free_length_m"]}
                if item["kind"] == "spring" else {"viscous_damping": item["viscous_damping_n_s_per_m"]})
        }) for item in bench["forces"]})
    assembly = AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1,
        "name": "bench_extension", "assembly_kind": "generic_multibody", "gravity": [0, 0, 0],
        "subsystems": [{"ref": "bench", "functional_role": "generic", "placement_role": "any"}]},
        subsystems={"bench": subsystem})
    case = {"schema_version": 1, "name": "loading", "study": "quasi_static", "protocol": "kc_quasi_static",
        "samples": [0, .001], "solver": {}, "inputs": [], "outputs": [], "boundaries": [],
        "excitation": {"drive_mode": "force_balance", "k": {"axes": [{"coordinate": "bench.load_travel", "values_mm": [10.0]}]}}}
    run = api.simulate(assembly, case)
    if run.status != "success":
        outcome.problems.append(f"the bench solve failed: {run.result.failure_evidence}")
    else:
        import numpy as np

        force = run.result.element_wrench("bench.bench_spring", body_id="bench.load_carriage").force
        if not np.allclose(np.linalg.norm(force, axis=1), declared["spring_force_at_10mm_n"], atol=1e-6, rtol=0):
            outcome.problems.append(f"the declared bench spring did not exert 250 N: {force.tolist()}")
        outcome.evidence.append(f"ordinary bench owns two bodies, one guide and one drive; spring wrench {force[-1].tolist()} N")
    if run.compiled.model_document.get("tires"):
        outcome.problems.append("the loading bench created a tire")

    # The extension boundary, measured rather than asserted.
    after = digests()
    changed = sorted(name for name in before if before[name] != after[name])
    if changed:
        outcome.problems.append(f"core files changed during the probe: {changed}")
    outcome.evidence.append(
        f"{len(core)} core source files unchanged across the extension "
        f"({next(iter(before.values()))[:12]})"
    )
    outcome.ok = not outcome.problems
    return outcome


def probe_a4() -> Outcome:
    """
    A4: a hardpoint and an attitude perturbation reach the solve.

    The independently computed attachment is compared against the port transform,
    and the same bench definition is shown unchanged across the perturbation.
    """
    outcome = Outcome("A4", "hardpoint and attitude perturbations reach the solve")

    import numpy as np

    from suspension_multibody import api
    from suspension_multibody.authoring.migration import migrate_v1_kc_case
    from suspension_multibody.connections import port_world_pose, solve_mount
    from suspension_multibody.modeling.identity import EntityId
    from suspension_multibody.modeling.ports import GeometryPort
    from suspension_multibody.modeling.primitives import (
        SE3,
        rotation_vector_to_quaternion,
    )
    from suspension_multibody.schema import MassSpec
    from suspension_multibody.schema.model import AxleDeclaration

    hardpoints = {
        "uca_front": [-100.0, -500.0, 400.0], "uca_rear": [100.0, -500.0, 400.0],
        "uca_outer": [0.0, -700.0, 450.0], "lca_front": [-120.0, -500.0, 150.0],
        "lca_rear": [120.0, -500.0, 150.0], "lca_outer": [0.0, -700.0, 150.0],
        "tierod_inner": [100.0, -400.0, 250.0], "tierod_outer": [50.0, -700.0, 250.0],
        "wheel_center": [0.0, -700.0, 300.0], "rack_center": [0.0, 0.0, 250.0],
    }

    def model(points):
        return AxleDeclaration(hardpoints=dict(points), mass=MassSpec(sprung_mass=1000.0))

    def sweep(points):
        assembly, case = migrate_v1_kc_case(model(points), mode="K", wheel_values_mm=(0., 20.), rack_values_mm=(0.,), drive_mode="kinematics")
        return api.simulate(assembly, case.to_payload())

    moved = dict(hardpoints)
    moved["uca_outer"] = [0.0, -700.0, 470.0]
    base = sweep(hardpoints)
    shifted = sweep(moved)
    def attachment(run):
        graph = run.compiled.request.model.to_document()
        joint = next(row for row in graph["joints"] if row["name"] == "model.sub.json.upper_arm_L_outer_joint")
        body = next(row for row in graph["bodies"] if row["name"] == joint["body_b"])
        return SE3(np.asarray(body["position"]), np.asarray(body["quaternion"])).transform_point(np.asarray(joint["point_b"]))[2]
    np.testing.assert_allclose([attachment(base), attachment(shifted)], [.45, .47], atol=1e-12, rtol=0)
    outcome.evidence.append("hardpoint +20 mm moved the SI attachment +20 mm exactly")
    def cambers(run):
        frames = run.result.frame_pose("wheel.sub.json.wheel_center_L")
        return [math.degrees(math.atan2(frames[run.result.case_samples(i).stop-1, 2, 1],
            frames[run.result.case_samples(i).stop-1, 1, 1])) for i in range(len(run.result.cases))]
    camber_before, camber_after = cambers(base), cambers(shifted)
    delta = abs(camber_after[1] - camber_before[1])
    if delta < 1e-6:
        outcome.problems.append("the perturbation did not change the solved response")
    if any(run.status != "success" for run in (base, shifted)):
        outcome.problems.append("a perturbed state did not converge")
    outcome.evidence.append(
        f"perturbation reached the solve: camber {camber_before[1]:.6f} -> "
        f"{camber_after[1]:.6f} deg ({delta:.6f})"
    )

    # The port's world pose is the owner's pose composed with the port's own
    # local pose.  The rotation is built through the published helper rather than
    # typed as four literals -- the convention is (w, x, y, z), and hand-typing
    # it is exactly how a test ends up proving the wrong transform.
    owner = SE3(
        np.array([100.0, -200.0, 300.0]),
        rotation_vector_to_quaternion(np.array([0.0, 0.15, 0.0])),
    )
    local = SE3(np.array([10.0, -20.0, 5.0]), np.array([1.0, 0.0, 0.0, 0.0]))
    world = port_world_pose(owner, local)
    # `SE3.rotation` is the 3x3 matrix the quaternion denotes, and the
    # independent composition below is written from scalars and `math` so it
    # does not reuse the module's own matrix helper.
    rotation = np.array(
        [
            [math.cos(0.15), 0.0, math.sin(0.15)],
            [0.0, 1.0, 0.0],
            [-math.sin(0.15), 0.0, math.cos(0.15)],
        ]
    )
    composed = owner.translation + rotation @ local.translation
    if not np.allclose(world.translation, composed, atol=1e-9):
        outcome.problems.append(
            f"the port pose is not the composed transform: "
            f"{world.translation} vs {composed}"
        )
    else:
        outcome.evidence.append(
            "the port world pose matches an independently composed transform to 1e-9"
        )

    # The adaptive half: a bench installed on that port, solved independently.
    port = GeometryPort(
        id=EntityId(("axle",), "wheel_centre_L"),
        owner=EntityId(("axle",), "upright_L"),
        role="wheel_centre",
        capabilities=frozenset({"wheel", "load"}),
        labels=frozenset({"L"}),
    )
    solution = solve_mount(port, port_world=world, installation=SE3.identity())
    if not solution.matches(world, tolerance=1e-6):
        outcome.problems.append(
            "the independently solved mount does not match the port pose"
        )
    else:
        outcome.evidence.append("the port match accepts the independently computed pose")
    outcome.ok = not outcome.problems
    return outcome


def probe_a5() -> Outcome:
    """A5: file and memory preserve mass, entities and subsystem provenance."""
    import numpy as np

    from suspension_multibody.api import validate
    from suspension_multibody.authoring import (
        AssemblyDocument,
        SubsystemDocument,
        TemplateDocument,
    )
    from suspension_multibody.authoring.migration import (
        migrate_v1_axle,
        save_migrated_assembly,
    )

    outcome = Outcome("A5", "file and memory share one traceable SI model")
    assembly = migrate_v1_axle(_benchmark_model(massed=True))
    case = {"schema_version": 1, "name": "identity", "study": "dynamic",
        "samples": [0, .001], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}
    memory = validate(assembly, case)
    with tempfile.TemporaryDirectory() as directory:
        filename = save_migrated_assembly(assembly, directory)
        loaded = validate(filename, case)
    if memory.model_payload != loaded.model_payload or memory.case_payload != loaded.case_payload:
        outcome.problems.append("file and memory emitted different contract payloads")
    graph = memory.request.model.to_document()
    expected_mass = sum(row.get("mass", 0) for entry in assembly.entries
        for row in entry.subsystem.template.payload["bodies"])
    actual_mass = sum(row["mass"] for row in graph["bodies"])
    if not np.isclose(actual_mass, expected_mass, atol=1e-9, rtol=0):
        outcome.problems.append(f"declared mass changed: {expected_mass} -> {actual_mass}")
    for row in graph["bodies"]:
        trace = graph["provenance"].get(row["name"], {})
        if not all(trace.get(key) for key in ("template", "revision", "properties_fingerprint")):
            outcome.problems.append(f"body {row['name']} has incomplete provenance")
    wheels = next(entry for entry in assembly.entries if entry.functional_role == "wheel")
    wheel_names = {wheels.ref+"."+row["name"] for row in wheels.subsystem.template.payload["bodies"]}
    if wheel_names != {row["name"] for row in graph["bodies"] if row["name"].startswith(wheels.ref+".")}:
        outcome.problems.append("Wheel body ownership differs from its declaration")
    outcome.evidence.append(f"{len(graph['bodies'])} bodies, mass {actual_mass} kg, complete provenance; identical file/memory payloads")
    # An arbitrary subsystem role can be composed; missing physical interfaces still fail.
    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "unbound_brake", "functional_role": "brake", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"}, "bodies": [], "joints": [],
        "elements": [], "hardpoints": [], "property_slots": [], "ports": [],
        "needs": [{"name": "rotor", "role": "rotor", "count": 1, "required": True}]})
    sub = SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
        "name": "unbound_brake", "template": "brake.tpl", "functional_role": "brake",
        "placement_role": "any", "hardpoints": {}, "property_bindings": {}}, template=template)
    bad = AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1,
        "name": "unbound", "assembly_kind": "generic_multibody",
        "subsystems": [{"ref": "brake", "functional_role": "brake", "placement_role": "any"}]},
        subsystems={"brake": sub})
    try:
        validate(bad, case)
        outcome.problems.append("an unconnected required rotor port was accepted")
    except ValueError as error:
        if "rotor" not in str(error):
            outcome.problems.append(f"missing-port refusal does not identify rotor: {error}")
        else:
            outcome.evidence.append("unconnected rotor requirement is refused by identity")
    outcome.ok = not outcome.problems
    return outcome


def probe_a6() -> Outcome:
    """
    A6: one assembly, two studies, one fingerprint, and a live quasi-static tire.

    The fingerprint identity is checked on one composition object.  The vertical
    tire is checked by changing only its stiffness and watching the force change,
    which is the difference between a tire that is in the residual and one that
    is merely declared.
    """
    outcome = Outcome("A6", "one assembly, two studies, a live vertical tire")
    from suspension_multibody.api import validate
    from suspension_multibody.authoring.migration import migrate_v1_axle
    from suspension_multibody.compilation.resolved import compile_resolved
    from suspension_multibody.modeling.resolved import ResolvedSolvePlan

    assembly = migrate_v1_axle(_benchmark_model(massed=True))
    case = {"schema_version": 1, "name": "acceptance_bench", "study": "dynamic",
        "samples": [0, .001], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}
    resolved = validate(assembly, case).request.model
    dynamic = compile_resolved(resolved, ResolvedSolvePlan(case))
    static_case = {**case, "study": "quasi_static", "protocol": "kc_quasi_static",
        "excitation": {"c": {"loads": [{"fz": 0}], "load_marker": "wheel.sub.json.wheel_center_L"}}}
    quasistatic = compile_resolved(resolved, ResolvedSolvePlan(static_case))
    if quasistatic.metadata["model_fingerprint"] != dynamic.metadata["model_fingerprint"] or dynamic.metadata["model_fingerprint"] != resolved.fingerprint:
        outcome.problems.append("the two studies report different physical model fingerprints")
    if quasistatic.metadata["study"] != "quasi_static" or dynamic.metadata["study"] != "dynamic":
        outcome.problems.append("the compiled studies differ from their declarations")
    for section in ("bodies", "joints", "elements", "tires"):
        if quasistatic.model_document.get(section) != dynamic.model_document.get(section):
            outcome.problems.append(f"study selection changed physical {section}")
    outcome.evidence.append(f"one physical fingerprint under two studies: {resolved.fingerprint[:16]}")
    if dynamic.case_document["family"] == dynamic.case_document["name"]:
        outcome.problems.append("a bench name was interpreted as its execution protocol")
    outcome.evidence.append("arbitrary bench name keeps its explicitly declared study protocol")

    # The quasi-static vertical tire really enters the residual: two stiffnesses
    # over the same travel must report two different forces.
    from suspension_multibody.api import validate
    from suspension_multibody.authoring.loader import CaseDocument
    from suspension_multibody.authoring.migration import migrate_v1_kc_case
    from suspension_multibody.schema import (
        Vec3,
        VerticalTire,
    )
    from suspension_multibody.simulation import run_compiled

    def tire_state(stiffness: float) -> tuple[float, float]:
        """
        Return the solved `(compression_m, normal_force_n)` of one tire.

        The probe is the same one the tree's own tire test uses: a symmetric
        20 mm droop, driven through the compiled submission, which is the route
        the quasi-static tire is consumed on.  The compression is a *driven*
        quantity here -- the wheel centre is prescribed, so it is the same in
        both runs -- and the force is what the tire law moves.  Comparing the
        force is therefore the assertion that the law was evaluated, and
        comparing the compression is what would pass on a law that was not.
        """
        tire = VerticalTire(
            stiffness=stiffness,
            unloaded_radius=320.0,
            contact_point=Vec3(x=0.0, y=0.0, z=0.0),
            local_axis=Vec3(x=0.0, y=0.0, z=1.0),
        )
        model = _benchmark_model().model_copy(update={"tires": (tire,)})
        assembly, case = migrate_v1_kc_case(model, mode="K", name="tire-probe",
            wheel_values_mm=(-20.,), rack_values_mm=(0.,), times_s=(0., .001))
        data = case.to_payload()
        data["element_activation"] = []
        run = run_compiled(validate(assembly, CaseDocument(data)))
        assert isinstance(run.result, ResultEnvelope)
        block = run.result.tire_state("wheel.sub.json.tire_0_L")
        compression_m = float(block[0, 2])
        normal_force_n = float(block[0, 4])
        return compression_m, normal_force_n

    soft_compression, soft_force = tire_state(200.0)
    stiff_compression, stiff_force = tire_state(400.0)
    if not (soft_force > 0.0):
        outcome.problems.append(f"the tire reports no normal force: {soft_force}")
    elif stiff_force < 1.5 * soft_force:
        outcome.problems.append(
            f"the vertical tire does not respond to stiffness: {soft_force} vs "
            f"{stiff_force}"
        )
    else:
        # F = k * delta with the document's stiffness in N/mm and the kernel's
        # penetration in metres: the relation is exact once the millimetre is
        # accounted for, so asserting it is a statement about the law rather
        # than about one fixture's geometry.
        ratio = stiff_force / soft_force
        if not 1.5 < ratio < 2.5:
            outcome.problems.append(
                f"doubling the vertical stiffness changed the force by {ratio}x"
            )
        if abs(soft_compression - stiff_compression) > 1e-9:
            outcome.problems.append(
                "the driven compression moved between the two runs, so the force "
                "comparison is not the same geometry"
            )
        outcome.evidence.append(
            f"the quasi-static vertical tire is in the residual: normal force "
            f"{soft_force:.1f} N at k=200 and {stiff_force:.1f} N at k=400 over the "
            "same prescribed 20 mm droop"
        )
    outcome.ok = not outcome.problems
    return outcome


def probe_a7() -> Outcome:
    """
    A7: the kernel's declared capability is the one it can execute.

    Two halves.  The build-time half runs the three C++ self-tests, each of which
    enumerates its own registry and asserts the handler is reachable; the
    runtime half asserts that a name the protocol knows but the build does not
    implement is refused differently from a name nobody has heard of.
    """
    outcome = Outcome("A7", "declared kernel capability matches the handlers")
    if not KERNEL_BUILD.is_dir():
        raise AcceptanceError(
            f"the kernel build directory {KERNEL_BUILD} is missing; build the kernel first"
        )
    built: dict[str, Path] = {}
    for name, banner in KERNEL_SELF_TESTS:
        candidates = sorted(KERNEL_BUILD.rglob(f"{name}.exe"))
        if not candidates:
            raise AcceptanceError(
                f"the C++ self-test {name} is not built; the strict gate refuses to "
                "treat that as unavailable"
            )
        completed = subprocess.run(
            [str(candidates[0])], capture_output=True, text=True, check=False
        )
        if completed.returncode != 0:
            outcome.problems.append(
                f"{name} failed: {completed.stdout.strip()[-200:]}"
            )
            continue
        checks = re.search(r"OK \((\d+) checks\)", completed.stdout)
        built[name] = candidates[0]
        outcome.evidence.append(
            f"{banner}: OK ({checks.group(1) if checks else '?'} checks)"
        )

    # Runtime: an unknown family is refused, and the refusal names it.  The
    # submission goes through the runner -- the one module allowed to reach the
    # kernel directly -- rather than calling the kernel entry point here, which
    # the public-API boundary gate forbids for a script.
    from suspension_contracts import pack_container

    from suspension_multibody.api import validate
    from suspension_multibody.authoring.migration import migrate_v1_kc_case
    from suspension_multibody.kernel import KernelContractError
    from suspension_multibody.simulation import run_compiled
    from suspension_multibody.simulation.request import (
        CompiledSimulation,
        SimulationRequest,
    )

    # A real model document, authored by the production authoring layer, so the
    # refusal being tested is the family's and not the model reader's.
    assembly, declared_case = migrate_v1_kc_case(_benchmark_model(), mode="K", name="acceptance",
        wheel_values_mm=(0.,), rack_values_mm=(0.,), drive_mode="kinematics")
    compiled = validate(assembly, declared_case)
    model = compiled.model_document

    def refusal_for(family: str) -> str:
        """Return the message a case document for `family` is refused with."""
        case = {
            "contract": "multibody-case",
            "contract_version": 1,
            "name": "probe",
            "kind": "case",
            "family": family,
            "k": {
                "name": "probe",
                "drive": "wheel_center",
                "axis_map": {"wheel": ["wheel_drive_L", "wheel_drive_R"], "rack": "rack_drive"},
                "wheel_values_mm": [0.0],
                "rack_values_mm": [0.0],
                "times_s": [0.0, 1e-3],
            },
        }
        try:
            run_compiled(
                CompiledSimulation(
                    request=SimulationRequest(assembly="generic", family=family, name="probe", model=compiled.request.model),
                    model_document=model,
                    case_document=case,
                    model_payload=pack_container(model),
                    case_payload=pack_container(case),
                )
            )
        except KernelContractError as error:
            return str(error)
        return ""

    unknown = refusal_for("no_such_family_at_all")
    if not unknown:
        outcome.problems.append("an unknown case family was accepted")
    elif "no_such_family_at_all" not in unknown:
        outcome.problems.append(f"the unknown-family refusal is vague: {unknown}")
    else:
        outcome.evidence.append(
            f"an unknown case family is refused by name: {unknown[:80]}"
        )

    # The distinction that matters: a family the *protocol* knows and this build
    # does not implement is refused differently from one nobody has heard of.
    # `comparison` is the per-target gate the kernel never reads a reference for.
    unimplemented = refusal_for("comparison")
    if not unimplemented:
        outcome.problems.append("a known-but-unimplemented family was accepted")
    elif unimplemented == unknown:
        outcome.problems.append(
            "a known-but-unimplemented family is refused with the same message as "
            f"an unknown one: {unimplemented}"
        )
    else:
        outcome.evidence.append(
            f"a known-but-unimplemented family is refused distinctly: "
            f"{unimplemented[:80]}"
        )
    outcome.ok = not outcome.problems
    return outcome


def probe_a8() -> Outcome:
    """A8: removing an optional subsystem removes its entities and result channels."""
    from copy import deepcopy

    from suspension_multibody.api import simulate
    from suspension_multibody.authoring import (
        AssemblyDocument,
        SubsystemDocument,
        TemplateDocument,
    )
    from suspension_multibody.authoring.migration import migrate_v1_kc_case

    outcome = Outcome("A8", "absent steering removes inputs, entities and channels")
    full, case = migrate_v1_kc_case(_benchmark_model(), mode="K",
        wheel_values_mm=(0., 10.), rack_values_mm=(0.,), drive_mode="kinematics")
    documents = {}
    removed = {"rack", "rack_housing", "tie_rod_L", "tie_rod_R"}
    for entry in full.entries:
        template = deepcopy(entry.subsystem.template.to_payload())
        template["bodies"] = [row for row in template["bodies"] if row["name"] not in removed]
        for key in ("joints", "elements"):
            template[key] = [row for row in template[key]
                if row.get("body_a") not in removed and row.get("body_b") not in removed
                and "@rack" not in (row.get("body_a"), row.get("body_b"))]
        template["ports"] = [row for row in template["ports"] if row.get("owner") not in removed]
        template["hardpoints"] = [row for row in template["hardpoints"]
            if row.get("owner") not in removed and row.get("owner") != "@rack"]
        template["needs"] = [row for row in template["needs"] if row["name"] != "rack"]
        points = {row["name"] for row in template["hardpoints"]}
        payload = entry.subsystem.to_payload()
        payload["hardpoints"] = {key: value for key, value in payload["hardpoints"].items() if key in points}
        documents[entry.ref] = SubsystemDocument.from_payload(payload,
            template=TemplateDocument.from_payload(template), properties=entry.subsystem.properties)
    payload = full.to_payload()
    for entry in payload["subsystems"]:
        entry["pairings"] = [row for row in entry.get("pairings", ()) if row["requirement_role"] != "rack"]
    reduced = AssemblyDocument.from_payload(payload, subsystems=documents)
    data = case.to_payload()
    data["excitation"]["k"]["axis_map"].pop("rack")
    data["excitation"]["k"]["rack_values_mm"] = []
    steered, unsteered = simulate(full, case.to_payload()), simulate(reduced, data)
    assert isinstance(steered.result, ResultEnvelope)
    assert isinstance(unsteered.result, ResultEnvelope)
    assert unsteered.compiled is not None
    if any(run.status != "success" for run in (steered, unsteered)):
        outcome.problems.append("an optional-branch solve failed")
    rack = "kc_rig.sub.json.rack_drive"
    if rack not in steered.result.constraint_ids or rack in unsteered.result.constraint_ids:
        outcome.problems.append("rack result channel was not removed with its declaration")
    if any(body.rsplit(".", 1)[-1] in removed for body in unsteered.result.body_ids):
        outcome.problems.append("removed steering bodies remain in the result")
    axes = unsteered.compiled.case_document["k"]["axis_map"]
    if "rack" in axes or len(axes["wheel"]) != 2:
        outcome.problems.append(f"incorrect reduced input interfaces: {axes}")
    outcome.evidence.append(f"{len(unsteered.result.body_ids)} bodies; two wheel drives; no rack entity, input or result channel")
    outcome.ok = not outcome.problems
    return outcome


def probe_a9() -> Outcome:
    """A9: one result envelope survives export and exposes real solver diagnostics."""
    import numpy as np

    from suspension_multibody.api import simulate, validate
    from suspension_multibody.authoring.loader import CaseDocument
    from suspension_multibody.authoring.migration import migrate_v1_kc_case
    from suspension_multibody.io import read_artifact, write_artifact
    from suspension_multibody.schema import Bushing6x6, Pose

    outcome = Outcome("A9", "uniform results, artifact round trip and diagnostics")
    model = _benchmark_model()
    assembly, case = migrate_v1_kc_case(model, mode="K", name="acceptance",
        wheel_values_mm=(0., 10.), rack_values_mm=(0.,), drive_mode="kinematics")
    run = simulate(assembly, case.to_payload())
    assert isinstance(run.result, ResultEnvelope)
    with tempfile.TemporaryDirectory() as directory:
        write_artifact(run.result, directory)
        artifact = read_artifact(directory)
        if artifact["manifest"]["model_fingerprint"] != run.result.model_fingerprint:
            outcome.problems.append("artifact model identity differs from the run")
        for name, values in run.result.named_blocks.items():
            np.testing.assert_array_equal(artifact["arrays"][name], values)
        outcome.evidence.append(f"artifact round trip preserves all {len(run.result.named_blocks)} native channel arrays")
    stiffness = np.diag([10_000.]*3+[10_000_000.]*3).tolist()
    names = {key.lower(): value for key, value in model.hardpoints.items()}
    bushings = tuple(Bushing6x6(name=f"{body}_{i}", body_a="chassis", body_b=body,
        pose_a=Pose(translation=names[key]), pose_b=Pose(translation=names[key]), stiffness=stiffness)
        for body, keys in (("upper_arm", ("uca_front", "uca_rear")),
                           ("lower_arm", ("lca_front", "lca_rear")))
        for i, key in enumerate(keys))
    compliant = model.model_copy(update={"bushings": bushings})
    c_assembly, c_case = migrate_v1_kc_case(compliant, mode="C", paths=("fz",),
        levels=3, maximum=100., side_mode="single", drive_mode="force_balance")
    c_run = simulate(c_assembly, c_case.to_payload())
    assert isinstance(c_run.result, ResultEnvelope)
    if c_run.status != "success" or not c_run.result.cases:
        outcome.problems.append("the compliant load sweep did not converge")
    else:
        outcome.evidence.append(f"C load sweep: {len(c_run.result.cases)} cases, residuals {c_run.result.case_residuals()}")
    invalid = case.to_payload()
    invalid["excitation"]["k"]["axis_map"]["wheel"][0] = "undeclared.contact_patch"
    try:
        validate(assembly, CaseDocument(invalid))
        outcome.problems.append("an undeclared contact drive was accepted")
    except ValueError as error:
        outcome.evidence.append(f"undeclared drive is refused before submission: {error}")
    if run.result.diagnostics is None or not np.isfinite(run.result.diagnostics).all():
        outcome.problems.append("the run has no finite native diagnostics")
    else:
        outcome.evidence.append(f"native diagnostics retain {run.result.diagnostics.shape[1]} columns")
    outcome.ok = not outcome.problems
    return outcome


def probe_a10() -> Outcome:
    """
    A10: the release probe's own results, re-read rather than re-trusted.

    The isolated install and the executable examples are the release probe's
    subject; this item confirms it ran and passed here, so a green end-to-end run
    means the packaged product works and not merely that the tree does.
    """
    outcome = Outcome("A10", "the packaged product builds, installs and runs")
    script = PACKAGE_ROOT / "scripts" / "check_composable_release.py"
    if not script.is_file():
        raise AcceptanceError(f"{script} is missing")
    completed = subprocess.run(
        [sys.executable, str(script), "--skip-native-rebuild"],
        cwd=str(ROOT), capture_output=True, text=True, check=False,
    )
    output = completed.stdout
    for line in output.splitlines():
        if line.startswith(("[PASS]", "[FAIL]")):
            outcome.evidence.append(line)
    if completed.returncode != 0:
        outcome.problems.append(
            "the release probe failed: "
            + (completed.stderr.strip().splitlines()[-1] if completed.stderr.strip() else output[-300:])
        )
    outcome.ok = not outcome.problems
    return outcome


def _title(probe: Callable[[], Outcome]) -> str:
    """Return a probe's one-line title, for an outcome that failed to run."""
    return (probe.__doc__ or "acceptance probe").strip().splitlines()[0]


#: The probes with the acceptance item each one establishes.  The identifier is
#: stated rather than derived from the function name, so renaming a function
#: cannot silently relabel which item it proves.
PROBES: tuple[tuple[str, Callable[[], Outcome]], ...] = (
    ("A1", probe_a1),
    ("A2", probe_a2),
    ("A3", probe_a3),
    ("A4", probe_a4),
    ("A5", probe_a5),
    ("A6", probe_a6),
    ("A7", probe_a7),
    ("A8", probe_a8),
    ("A9", probe_a9),
    ("A10", probe_a10),
)


# --------------------------------------------------------------------------- #
# the scenario selections and the frozen commands
# --------------------------------------------------------------------------- #


def run_scenarios(*, strict: bool) -> list[Outcome]:
    """Run each A-item's pytest selection and report it."""
    outcomes: list[Outcome] = []
    for identifier, targets in SCENARIO_TESTS.items():
        existing = [
            target for target in targets if (ROOT / target).exists()
        ]
        if not existing:
            outcomes.append(
                Outcome(identifier, "scenario selection", False, problems=[
                    f"no scenario file exists for {identifier}: {list(targets)}"
                ])
            )
            continue
        command = [
            sys.executable, "-m", "pytest", *existing, "-q", "-rsx", "-p", "no:cacheprovider",
        ]
        completed = subprocess.run(
            command, cwd=str(ROOT), capture_output=True, text=True, check=False
        )
        summary = next(
            (line for line in reversed(completed.stdout.splitlines()) if " passed" in line or " failed" in line or " error" in line),
            "",
        )
        problems: list[str] = []
        evidence = [f"{summary.strip() or 'no summary line'}"]
        if completed.returncode != 0:
            problems.append(
                "the selection failed: "
                + " | ".join(
                    line.strip()
                    for line in completed.stdout.splitlines()
                    if line.startswith(("FAILED", "ERROR"))
                )[:400]
            )
        if strict:
            counts = {
                word: int(number)
                for number, word in re.findall(
                    r"(\d+) (passed|failed|skipped|xfailed|xpassed|error|errors)",
                    summary,
                )
            }
            grown = {
                key: value
                for key, value in counts.items()
                if key in {"skipped", "xfailed", "xpassed", "error", "errors"} and value
            }
            if grown:
                problems.append(
                    f"a strict scenario selection must not skip or xfail: {grown}"
                )
        outcomes.append(Outcome(identifier, "scenario selection", not problems, evidence, problems))
    return outcomes


def ensure_native(*, rebuild: bool) -> Outcome:
    """
    Establish one native library, identical in both mirrors, before anything runs.

    This is a prerequisite rather than a scenario, and it is checked first for a
    reason that cost real time to learn: a half-completed rebuild -- the kernel
    mirror written and the multibody copy refused because a process still held the
    file open -- leaves the loader's freshness guard refusing to load, and then
    eight unrelated scenarios fail with the same message about a stale mirror.
    Naming the actual problem once is what this exists to do.
    """
    outcome = Outcome("native", "the native library is present and consistent")
    import hashlib

    if rebuild:
        for identifier, command in frozen_commands():
            if identifier == NATIVE_COMMAND:
                completed = subprocess.run(
                    command, shell=True, cwd=str(ROOT), capture_output=True,
                    text=True, check=False,
                )
                if completed.returncode != 0:
                    tail = (completed.stderr or completed.stdout).strip().splitlines()[-3:]
                    outcome.problems.append(
                        f"the native build failed: {' | '.join(tail)}"
                    )
                    return outcome
                break
        else:
            raise AcceptanceError(
                f"the frozen command set has no {NATIVE_COMMAND!r} entry to run"
            )

    missing = [path for path in NATIVE_MIRRORS if not path.is_file()]
    if missing:
        outcome.problems.append(
            "the native library is missing at " + ", ".join(str(p) for p in missing)
        )
        return outcome
    digests = [
        hashlib.sha256(path.read_bytes()).hexdigest() for path in NATIVE_MIRRORS
    ]
    if len(set(digests)) != 1:
        outcome.problems.append(
            "the two native mirrors differ, so the loader will refuse the stale "
            f"one: {[digest[:12] for digest in digests]}; re-run the "
            "build_axle_native command"
        )
        return outcome
    outcome.evidence.append(
        f"both mirrors carry the same library ({digests[0][:16]}, "
        f"{NATIVE_MIRRORS[0].stat().st_size} bytes)"
    )
    outcome.ok = not outcome.problems
    return outcome


def frozen_commands() -> list[tuple[str, str]]:
    """Return the baseline command set as ``(id, command)`` pairs."""
    payload = json.loads(BASELINE_COMMANDS.read_text(encoding="utf-8-sig"))
    return [(item["id"], item["command"]) for item in payload["commands"]]


def run_frozen(*, only: tuple[str, ...] = ()) -> list[Outcome]:
    """Run the frozen command set, minus the ones the probes already cover."""
    outcomes: list[Outcome] = []
    for identifier, command in frozen_commands():
        if only and identifier not in only:
            continue
        completed = subprocess.run(
            command, shell=True, cwd=str(ROOT), capture_output=True, text=True,
            check=False,
        )
        tail = (completed.stdout or completed.stderr).strip().splitlines()[-1:]
        outcomes.append(
            Outcome(
                f"cmd:{identifier}",
                command,
                completed.returncode == 0,
                [f"exit {completed.returncode}: {tail[0][:160] if tail else ''}"],
                [] if completed.returncode == 0 else ["non-zero exit"],
            )
        )
    return outcomes


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #


def main(argv: list[str] | None = None) -> int:
    """Run the acceptance items and return the exit status."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--strict", action="store_true",
                        help="refuse a missing prerequisite or a skip/xfail in a scenario")
    parser.add_argument("--list", action="store_true", help="list the items, run nothing")
    parser.add_argument("--probes-only", action="store_true", help="skip the frozen commands")
    parser.add_argument("--commands-only", action="store_true", help="skip the probes")
    parser.add_argument("--json", default=None, help="write the outcome summary here")
    parser.add_argument(
        "--skip-native-rebuild",
        action="store_true",
        help="use the current native library instead of rebuilding it first",
    )
    args = parser.parse_args(argv)

    if args.list:
        for identifier, probe in PROBES:
            first = (probe.__doc__ or "").strip().splitlines()[0]
            print(f"  probe  {identifier}: {first}")
        for identifier, targets in SCENARIO_TESTS.items():
            print(f"  select {identifier}: {', '.join(targets)}")
        for identifier, command in frozen_commands():
            print(f"  frozen {identifier}: {command}")
        return 0

    print(f"acceptance : {ROOT}")
    print(f"strict     : {args.strict}\n")
    outcomes: list[Outcome] = []
    try:
        native = ensure_native(rebuild=not args.skip_native_rebuild)
    except AcceptanceError as error:
        native = Outcome("native", "the native library is present", False, [], [str(error)])
    outcomes.append(native)
    if not native.ok:
        print(native.report())
        print("the native prerequisite is not met, so no scenario was run")
        return 1
    if not args.commands_only:
        def run_probes() -> None:
            """Run the scenario probes, one outcome each."""
            for identifier, probe in PROBES:
                try:
                    outcomes.append(probe())
                except AcceptanceError as error:
                    outcomes.append(
                        Outcome(identifier, _title(probe), False, [], [str(error)])
                    )
                except Exception as error:  # noqa: BLE001 - a probe that throws fails
                    outcomes.append(
                        Outcome(
                            identifier, _title(probe), False, [],
                            [f"{type(error).__name__}: {error}"],
                        )
                    )

        # The selections run first, and the probes after: A10's probe builds and
        # installs the three wheels, and a wheel build racing a concurrently
        # running selection is how a passing selection reports a spurious
        # failure.  Ordering them is cheaper than explaining that failure.
        outcomes.extend(run_scenarios(strict=args.strict))
        run_probes()
    # `kc_parity` is deliberately absent: without `--actual-dir` it compares the
    # frozen snapshot against itself, so it is a tautology rather than evidence.
    # The K/C equivalence it is meant to establish needs the probe scripts to run
    # first, which is a separate, much longer check.
    frozen_ids = {"dynamic_hash", "case_parity", "kc_perf", "legacy_surface_gate",
                  "kernel_layering", "ruff", "ty"}
    if not args.probes_only:
        for outcome in run_frozen(only=tuple(frozen_ids) if not args.probes_only else ()):
            outcomes.append(outcome)

    for outcome in outcomes:
        print(outcome.report())
    failed = [outcome for outcome in outcomes if not outcome.ok]
    if args.json:
        Path(args.json).write_text(
            json.dumps(
                [
                    {
                        "id": outcome.identifier,
                        "title": outcome.title,
                        "ok": outcome.ok,
                        "evidence": outcome.evidence,
                        "problems": outcome.problems,
                    }
                    for outcome in outcomes
                ],
                indent=1,
            ),
            encoding="utf-8",
        )
    if failed:
        print(f"\nFAIL: {len(failed)} of {len(outcomes)} acceptance items failed")
        for outcome in failed:
            print(f"  - {outcome.identifier}")
        return 1
    print(f"\nOK: {len(outcomes)} acceptance items passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
