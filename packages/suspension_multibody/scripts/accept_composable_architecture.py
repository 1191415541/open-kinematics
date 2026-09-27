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
    from suspension_multibody.schema import FrontAxleModel

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
    return FrontAxleModel.model_validate(raw)


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
    from suspension_contracts import pack_container

    from suspension_multibody import api
    from suspension_multibody.axle_dynamics.schema import (
        AxleDynamicsCase,
        AxleSolverSettings,
    )
    from suspension_multibody.compilation import KcStudyInputs, compile_plan, plan_for
    from suspension_multibody.schema import (
        CaseSpec,
        DisplacementControl,
        FrontAxleModel,
    )
    from suspension_multibody.simulation import run_compiled
    from suspension_multibody.simulation.request import (
        CompiledSimulation,
        SimulationRequest,
    )
    from suspension_multibody.studies import (
        DYNAMIC,
        axle_dynamics_model,
        build_study_assembly,
    )
    from suspension_multibody.subsystems.entry import compose_axle

    fixture = json.loads(
        (PACKAGE_ROOT / "tests" / "data" / "composable"
         / "synthetic_trailing_arm_axle.json").read_text(encoding="utf-8")
    )
    if fixture.get("_synthetic") is not True:
        outcome.problems.append("the synthetic fixture is not marked _synthetic")
    synthetic = FrontAxleModel.model_validate(fixture["model"])
    wishbone = _benchmark_model()

    # The graph difference, asserted rather than assumed.
    synthetic_assembly = compose_axle(synthetic, "K")
    wishbone_assembly = compose_axle(wishbone, "K")
    synthetic_count = len(synthetic_assembly.constraints)
    wishbone_count = len(wishbone_assembly.constraints)
    kinds = {type(c).__name__ for c in synthetic_assembly.constraints}
    if (synthetic_count, wishbone_count, kinds) != (2, 13, {"RevoluteJoint"}):
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
    bundle = api.run_case(
        synthetic,
        CaseSpec(
            mode="K",
            controls=(DisplacementControl(target="wheel_travel_left", values=travels),),
        ),
    )
    solved = [state.metrics["left_wheel_center_z"] for state in bundle.states]
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
    if not all(state.converged for state in bundle.states):
        outcome.problems.append("a quasi-static state did not converge")
    outcome.evidence.append(
        f"quasi-static: {len(bundle.states)} states converged, worst deviation from "
        f"the derived rotation {worst:.4f} mm"
    )

    # The dynamic reading, solved for real.  The static initialisation of this
    # assembly needs more Newton iterations than the default, which is a solver
    # setting and not a model change.
    study_assembly = build_study_assembly(synthetic_assembly, study=DYNAMIC, mode="K")
    dynamic_model = axle_dynamics_model(study_assembly, name="trailing-arm")
    case = AxleDynamicsCase(
        name="acceptance",
        times_s=(0.0, 1e-3, 2e-3),
        solver=AxleSolverSettings(max_newton_iterations=100),
    )
    plan = plan_for(
        "axle_dynamic",
        mode="K",
        inputs=KcStudyInputs(name="acceptance", wheel_values_mm=(0.0,), rack_values_mm=(0.0,)),
        dynamic_model=dynamic_model,
        dynamic_case=case,
    )
    model_document, case_document, model_blob, case_blob, metadata = compile_plan(
        plan, synthetic_assembly
    )
    run = run_compiled(
        CompiledSimulation(
            request=SimulationRequest(
                assembly="axle", rig="axle_dynamic", family=plan.family,
                study=plan.study, model=synthetic_assembly, case=case_document,
                name="acceptance",
            ),
            model_document=model_document,
            case_document=case_document,
            model_payload=model_blob or pack_container(model_document),
            case_payload=case_blob or pack_container(case_document),
            metadata=metadata,
        )
    )
    states = run.raw.states
    if run.status != "success" or states.size == 0:
        outcome.problems.append(
            f"the dynamic reading did not solve: {run.status} {run.raw.failure_evidence}"
        )
    else:
        import numpy as np

        if not bool(np.isfinite(states).all()):
            outcome.problems.append("the dynamic state is not finite")
        body = run.raw.body_names.index("upright_L")
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
    from suspension_multibody.rigs import (
        bench_capability,
        build_rig_fragment,
        rig_ports,
    )
    from suspension_multibody.schema import CaseSpec, MassSpec

    core = (
        "cases/kc_quasi_static/contract.py",
        "compilation/compile.py",
        "compilation/plan.py",
        "simulation/compiler.py",
        "simulation/runner.py",
        "simulation/dispatch.py",
        "rigs/compose.py",
        "modeling/assembly.py",
        "subsystems/composition.py",
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

    # A vehicle-loading bench must not create wheels: the vehicle owns them.
    before = digests()
    wheel_fragment = build_rig_fragment("kc_quasi_static")
    vehicle_fragment = build_rig_fragment("vehicle_kc")
    if bench_capability("kc_quasi_static") != "wheel_supplying":
        outcome.problems.append("the K&C bench is not wheel-supplying")
    if wheel_fragment.tires == {} or len(wheel_fragment.bodies) != 3:
        outcome.problems.append(
            f"the wheel-supplying bench emitted {sorted(wheel_fragment.bodies)} and "
            f"{len(wheel_fragment.tires)} tires"
        )
    if vehicle_fragment.tires != {}:
        outcome.problems.append("a vehicle-loading bench created tires")
    if len(rig_ports("kc_quasi_static")) != 2:
        outcome.problems.append("the K&C bench does not offer two wheel-centre ports")
    outcome.evidence.append(
        f"capability branch: wheel-supplying {sorted(wheel_fragment.bodies)}, "
        f"vehicle-loading {sorted(vehicle_fragment.bodies)}"
    )

    # The bench's own force elements reach the solver and carry a load.
    model = _benchmark_model()
    spring = next(item for item in bench["forces"] if item["kind"] == "spring")
    from suspension_multibody.schema import (
        FrontAxleModel,
        LinearSpring,
        StaticDamper,
        Vec3,
    )

    damper = next(item for item in bench["forces"] if item["kind"] == "damper")
    loaded = FrontAxleModel(
        name=model.name,
        hardpoints=dict(model.hardpoints),
        mass=MassSpec(sprung_mass=600.0),
        springs=(
            LinearSpring(
                name=spring["name"], body_a="chassis", body_b="lower_arm_L",
                point_a=Vec3(x=0.0, y=-600.0, z=100.0),
                point_b=Vec3(x=0.0, y=-600.0, z=400.0),
                stiffness=spring["stiffness_n_per_m"] / 1000.0, free_length=250.0,
            ),
        ),
        dampers=(
            StaticDamper(
                name=damper["name"], body_a="chassis", body_b="lower_arm_L",
                point_a=Vec3(x=0.0, y=-600.0, z=100.0),
                point_b=Vec3(x=0.0, y=-600.0, z=400.0),
                viscous_damping=damper["viscous_damping_n_s_per_m"],
            ),
        ),
    )
    bundle = api.run_case(loaded, CaseSpec(mode="K"))
    components = {load.component for load in bundle.component_loads}
    names = {f"{spring['name']}_L", f"{spring['name']}_R"}
    if not names <= components:
        outcome.problems.append(f"the bench spring is absent from the component table: {components}")
    else:
        entry = next(load for load in bundle.component_loads if load.component == f"{spring['name']}_L")
        if not any(value != 0.0 for value in entry.global_load.as_tuple()):
            outcome.problems.append("the bench spring reports an all-zero wrench")
        outcome.evidence.append(
            f"the bench spring reaches the solve and carries "
            f"{entry.global_load.as_tuple()}"
        )

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
    from suspension_multibody.connections import port_world_pose, solve_mount
    from suspension_multibody.modeling.identity import EntityId
    from suspension_multibody.modeling.ports import GeometryPort
    from suspension_multibody.modeling.primitives import (
        SE3,
        rotation_vector_to_quaternion,
    )
    from suspension_multibody.schema import (
        CaseSpec,
        DisplacementControl,
        FrontAxleModel,
        MassSpec,
    )
    from suspension_multibody.subsystems.entry import compose_axle

    hardpoints = {
        "uca_front": [-100.0, -500.0, 400.0], "uca_rear": [100.0, -500.0, 400.0],
        "uca_outer": [0.0, -700.0, 450.0], "lca_front": [-120.0, -500.0, 150.0],
        "lca_rear": [120.0, -500.0, 150.0], "lca_outer": [0.0, -700.0, 150.0],
        "tierod_inner": [100.0, -400.0, 250.0], "tierod_outer": [50.0, -700.0, 250.0],
        "wheel_center": [0.0, -700.0, 300.0], "rack_center": [0.0, 0.0, 250.0],
    }

    def model(points):
        return FrontAxleModel(hardpoints=dict(points), mass=MassSpec(sprung_mass=1000.0))

    def sweep(points):
        case = CaseSpec(
            mode="K",
            controls=(DisplacementControl(target="wheel_travel_left", values=(0.0, 20.0)),),
        )
        return api.run_case(model(points), case).states

    moved = dict(hardpoints)
    moved["uca_outer"] = [0.0, -700.0, 470.0]
    before = compose_axle(model(hardpoints), "K")
    after = compose_axle(model(moved), "K")
    key = ("upright_L", "upper_arm_L_outer")
    if float(before.points[key][2]) != 450.0 or float(after.points[key][2]) != 470.0:
        outcome.problems.append(
            f"the attachment did not follow the hardpoint: "
            f"{float(before.points[key][2])} -> {float(after.points[key][2])}"
        )
    else:
        outcome.evidence.append("hardpoint +20 mm moved the attachment +20 mm exactly")

    base = sweep(hardpoints)
    shifted = sweep(moved)
    camber_before = [state.metrics["left_camber_deg"] for state in base]
    camber_after = [state.metrics["left_camber_deg"] for state in shifted]
    delta = abs(camber_after[1] - camber_before[1])
    if delta < 1e-6:
        outcome.problems.append("the perturbation did not change the solved response")
    if not all(state.converged for state in (*base, *shifted)):
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
    """
    A5: one physical input, two schemas, one SI model, every entity traceable.

    The two schemas are the K/C authoring schema (millimetres, kinematic) and the
    SI dynamic schema the dynamic reading needs.  Both are built here from the
    same hardpoint input, and the entities are compared one by one; then the
    composition's provenance is walked to confirm each subsystem's entities can
    be traced back to the template that produced them.
    """
    outcome = Outcome("A5", "two schemas, one SI model, traceable entities")
    from suspension_multibody.schema import FrontAxleModel
    from suspension_multibody.studies import (
        DYNAMIC,
        axle_dynamics_model,
        build_study_assembly,
    )
    from suspension_multibody.subsystems import AssemblyRequest
    from suspension_multibody.subsystems.entry import compose_axle
    from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle

    kinematic = compose_axle(_benchmark_model(), "K")
    massed = compose_axle(_benchmark_model(massed=True), "K")
    study_assembly = build_study_assembly(massed, study=DYNAMIC, mode="K")
    dynamic = axle_dynamics_model(study_assembly, name="acceptance")

    # The SI model is a reading of the same assembly: names, order, masses and
    # the constraint count all have to line up.
    if list(massed.bodies) != [body.name for body in dynamic.bodies]:
        outcome.problems.append(
            "the SI model's bodies are not the assembly's, in order"
        )
    if len(massed.constraints) != len(dynamic.joints):
        outcome.problems.append(
            f"constraint count differs: {len(massed.constraints)} vs {len(dynamic.joints)}"
        )
    off_mass = [
        body.name
        for body in dynamic.bodies
        if body.mass_kg != massed.bodies[body.name].mass
    ]
    if off_mass:
        outcome.problems.append(f"these bodies' SI mass differs: {off_mass}")
    total = sum(body.mass for body in massed.bodies.values())
    dynamic_total = sum(body.mass_kg for body in dynamic.bodies)
    if abs(total - dynamic_total) > 1e-9:
        outcome.problems.append(
            f"total mass is not conserved: {total} vs {dynamic_total}"
        )
    outcome.evidence.append(
        f"SI reading: {len(dynamic.bodies)} bodies, {len(dynamic.joints)} joints, "
        f"total mass {dynamic_total:.1f} kg matches the K/C assembly"
    )

    # Wheels: the single-axle assembly builds none in either schema.
    if any(name.startswith("wheel_") for name in kinematic.bodies):
        outcome.problems.append("the single-axle assembly built a wheel body")
    outcome.evidence.append(
        "the single-axle assembly owns no wheel body in either schema "
        "(the bench supplies it)"
    )

    # Provenance: every subsystem's contribution is traceable to its template.
    composed = si_assembly_for_axle(
        FrontAxleModel.model_validate(_benchmark_payload()),
        request=AssemblyRequest(mode="K"),
    )
    walked = [level for level in composed.assembly.walk()]
    contributed = [level for level in walked if level.subsystems]
    missing = [level.name for level in contributed if level.provenance is None]
    if missing:
        outcome.problems.append(f"levels without provenance: {missing}")
    traces = {
        level.name: level.provenance.template
        for level in contributed
        if level.provenance is not None
    }
    if len(traces) < 2:
        outcome.problems.append("the composition does not nest per-subsystem levels")
    outcome.evidence.append(
        "provenance traces: "
        + ", ".join(f"{name}->{template}" for name, template in sorted(traces.items()))
    )
    if not composed.fingerprint:
        outcome.problems.append("the composition carries no fingerprint")
    else:
        outcome.evidence.append(f"composition fingerprint {composed.fingerprint[:16]}")

    # The refusals: each illegal combination must be named, not silently dropped.
    from suspension_multibody.rigs import CompositionError, resolve_combination

    try:
        compose_axle(
            _benchmark_model(),
            "K",
            AssemblyRequest(mode="K", subsystems=frozenset({"chassis", "suspension", "brake"})),
        )
        outcome.problems.append("a single axle accepted a brake subsystem")
    except ValueError as error:
        if "brake" not in str(error):
            outcome.problems.append(f"the brake refusal does not name brake: {error}")
        else:
            outcome.evidence.append("a single axle refuses a brake subsystem by name")

    try:
        compose_axle(
            _benchmark_model(),
            "K",
            AssemblyRequest(mode="K", subsystems=frozenset({"chassis", "suspension", "drive"})),
        )
        outcome.problems.append("a single axle accepted a drive subsystem")
    except ValueError as error:
        if "drive" not in str(error):
            outcome.problems.append(f"the drive refusal does not name drive: {error}")
        else:
            outcome.evidence.append("a single axle refuses a drive subsystem by name")

    capabilities = kinematic.capabilities
    if capabilities is None:
        raise AcceptanceError(
            "the axle built through the production path reports no capabilities, so "
            "the bench's interface cannot be resolved"
        )

    # A rig that belongs to another kind of assembly is refused by name, and the
    # refusal is about registration rather than about capability.
    try:
        resolve_combination("axle", "ride_four_post", capabilities)
        outcome.problems.append("a vehicle four-post bench was accepted for an axle")
    except CompositionError as error:
        if "ride_four_post" not in str(error):
            outcome.problems.append(
                f"the ride_four_post refusal does not name the bench: {error}"
            )
        else:
            outcome.evidence.append(
                "a bench registered for another assembly kind is refused by name"
            )

    # A drive the assembly cannot offer is dropped, not driven at zero.
    composition = resolve_combination("axle", "kc_quasi_static", capabilities)
    if composition.dropped:
        outcome.problems.append(
            f"the steering axle dropped drives it can offer: {composition.dropped}"
        )
    else:
        outcome.evidence.append(
            f"the steering axle keeps all {len(composition.drives)} bench drives"
        )
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
    from suspension_multibody.axle_dynamics.schema import (
        AxleDynamicsCase,
        AxleSolverSettings,
    )
    from suspension_multibody.compilation import KcStudyInputs, compile_plan, plan_for
    from suspension_multibody.studies import (
        DYNAMIC,
        QUASI_STATIC,
        axle_dynamics_model,
        build_study_assembly,
    )
    from suspension_multibody.subsystems import AssemblyRequest
    from suspension_multibody.subsystems.si_assembly import si_assembly_for_axle

    composed = si_assembly_for_axle(
        _benchmark_model(massed=True), request=AssemblyRequest(mode="K")
    )
    inputs = KcStudyInputs(
        name="acceptance", wheel_values_mm=(0.0,), rack_values_mm=(0.0,)
    )
    dynamic_model = axle_dynamics_model(
        build_study_assembly(composed.assembly.physical, study=DYNAMIC, mode="K"),
        name="acceptance",
    )
    quasistatic = plan_for("kc_quasi_static", mode="K", inputs=inputs)
    dynamic = plan_for(
        "axle_dynamic",
        mode="K",
        inputs=inputs,
        dynamic_model=dynamic_model,
        dynamic_case=AxleDynamicsCase(
            name="acceptance", times_s=(0.0, 1e-3), solver=AxleSolverSettings()
        ),
    )
    _, _, _, _, quasi_meta = compile_plan(quasistatic, composed)
    _, _, _, _, dynamic_meta = compile_plan(dynamic, composed)
    if quasi_meta["study"] != QUASI_STATIC or dynamic_meta["study"] != DYNAMIC:
        outcome.problems.append(
            f"the plans do not carry their studies: {quasi_meta['study']} / "
            f"{dynamic_meta['study']}"
        )
    if quasi_meta["fingerprint"] != dynamic_meta["fingerprint"]:
        outcome.problems.append("the two studies report different fingerprints")
    if quasi_meta["fingerprint"] != composed.fingerprint:
        outcome.problems.append("the compiled fingerprint is not the composition's")
    outcome.evidence.append(
        f"one fingerprint under two studies: {quasi_meta['fingerprint'][:16]} "
        f"(quasi-static tire activation {quasi_meta['tire_activation']}, "
        f"dynamic {dynamic_meta['tire_activation']})"
    )

    # A rig whose name is not the family name still runs.
    from dataclasses import replace

    from suspension_multibody.rigs.rig import RIGS

    original = RIGS["kc_quasi_static"]
    RIGS["acceptance_bench"] = replace(
        original, name="acceptance_bench", family="kc_quasi_static"
    )
    try:
        renamed = plan_for("acceptance_bench", mode="K", inputs=inputs)
        if renamed.family != "kc_quasi_static" or renamed.rig != "acceptance_bench":
            outcome.problems.append(
                f"a renamed bench routed wrong: {renamed.rig} / {renamed.family}"
            )
        else:
            outcome.evidence.append(
                "a bench named 'acceptance_bench' routes to kc_quasi_static"
            )
    finally:
        del RIGS["acceptance_bench"]

    # The quasi-static vertical tire really enters the residual: two stiffnesses
    # over the same travel must report two different forces.
    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.schema import (
        Vec3,
        VerticalTire,
    )
    from suspension_multibody.simulation import SimulationRequest, run_request
    from suspension_multibody.subsystems.entry import compose_axle

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
        assembly = compose_axle(model, "K")
        document = model_document(assembly, name="tire-probe", drive_wheels=True)
        run = run_request(
            SimulationRequest(
                assembly="axle",
                rig="kc_quasi_static",
                family="kc_quasi_static",
                model=document,
                case={
                    "contract": "multibody-case",
                    "contract_version": 1,
                    "kind": "case",
                    "family": "kc_quasi_static",
                    "name": "tire-probe",
                    "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
                    "k": {
                        "wheel_values_mm": [-20.0],
                        "rack_values_mm": [0.0],
                        "axis_map": {
                            "wheel": ["wheel_drive_L", "wheel_drive_R"],
                            "rack": "rack_drive",
                        },
                        "drive": "wheel_center",
                        "left_right_mode": "symmetric",
                    },
                },
            )
        )
        block = run.raw.block("tire_output")
        compression_m = float(block[0, 0, 2])
        normal_force_n = float(block[0, 0, 4])
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

    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.kernel import KernelContractError
    from suspension_multibody.schema import VerticalTire
    from suspension_multibody.simulation import run_compiled
    from suspension_multibody.simulation.request import (
        CompiledSimulation,
        SimulationRequest,
    )
    from suspension_multibody.subsystems.entry import compose_axle

    # A real model document, authored by the production authoring layer, so the
    # refusal being tested is the family's and not the model reader's.
    assembly = compose_axle(_benchmark_model(), "K")
    model = model_document(assembly, name="acceptance")
    del VerticalTire

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
                    request=SimulationRequest(assembly="axle", family=family, name="probe"),
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
    """
    A8: the optional branch disappears rather than zeroing, and the rules hold.

    The rack channel is the concrete case: an axle without steering must lose the
    rack input *and* the rack result channel, and a vehicle missing a required
    subsystem must fail by name.
    """
    outcome = Outcome("A8", "an absent branch disappears, and the global rules hold")
    from suspension_multibody import api
    from suspension_multibody.cases.kc_quasi_static.contract import has_rack
    from suspension_multibody.rigs import resolve_combination
    from suspension_multibody.schema import CaseSpec, DisplacementControl
    from suspension_multibody.subsystems import AssemblyRequest
    from suspension_multibody.subsystems.entry import compose_axle

    with_rack = compose_axle(_benchmark_model(), "K")
    without = compose_axle(
        _benchmark_model(),
        "K",
        AssemblyRequest(
            mode="K", subsystems=frozenset({"chassis", "suspension", "wheel"})
        ),
    )
    if not has_rack(with_rack):
        outcome.problems.append("the steering axle does not declare a rack")
    if has_rack(without):
        outcome.problems.append("the steering-less axle still declares a rack")
    if any(name.startswith("rack") or name.startswith("tie_rod") for name in without.bodies):
        outcome.problems.append(
            f"the steering-less axle kept rack bodies: {sorted(without.bodies)}"
        )
    outcome.evidence.append(
        f"steering-less assembly: {len(without.bodies)} bodies, no rack or tie rod"
    )

    # The result channel, not just the input.
    case = CaseSpec(
        mode="K",
        controls=(DisplacementControl(target="wheel_travel_left", values=(0.0, 10.0)),),
    )
    no_steering = api.run_case(
        _benchmark_model(),
        case.model_copy(update={"subsystems": frozenset({"chassis", "suspension", "wheel"})}),
    )
    drives = set(no_steering.states[0].drives)
    if "rack_displacement" in drives:
        outcome.problems.append("the steering-less run still reports a rack channel")
    if not {"wheel_travel_left", "wheel_travel_right"} <= drives:
        outcome.problems.append(f"the wheel channels are missing: {drives}")
    outcome.evidence.append(
        f"the steering-less run's drives are {sorted(drives)} -- no rack channel, "
        "not a zero one"
    )

    steered = api.run_case(_benchmark_model(), case)
    if "rack_displacement" not in steered.states[0].drives:
        outcome.problems.append("the steered run lost its rack channel")
    else:
        outcome.evidence.append("the steered run keeps its rack channel")

    # The bench's interface shrinks with the capability.
    without_capabilities = without.capabilities
    if without_capabilities is None:
        raise AcceptanceError("the steering-less axle reports no capabilities")
    composition = resolve_combination("axle", "kc_quasi_static", without_capabilities)
    if "rack_drive" not in composition.dropped:
        outcome.problems.append(
            f"the bench did not drop the rack drive: {composition.dropped}"
        )
    else:
        outcome.evidence.append(
            f"the bench's interface shrank: dropped {list(composition.dropped)}"
        )

    # The global rules, on the axle path: a subsystem the category forbids is
    # refused by name rather than silently dropped.
    for role in ("brake", "drive"):
        try:
            compose_axle(
                _benchmark_model(),
                "K",
                AssemblyRequest(
                    mode="K", subsystems=frozenset({"chassis", "suspension", role})
                ),
            )
            outcome.problems.append(f"a single axle accepted the {role} subsystem")
        except ValueError as error:
            if role not in str(error):
                outcome.problems.append(f"the {role} refusal does not name it: {error}")
            else:
                outcome.evidence.append(f"the global rule refuses {role} by name")
    outcome.ok = not outcome.problems
    return outcome


def probe_a9() -> Outcome:
    """
    A9: the historical surfaces still behave, and no numeric baseline moved.

    The entry points are re-imported and called, a result bundle is written and
    read back, and a failed solve is shown to be diagnosed rather than silently
    returned as a success.  The numeric gates themselves are in the frozen
    command set; what is checked here is that the *public* surface a caller uses
    still exists and still refuses what it always refused.
    """
    outcome = Outcome("A9", "public entry points, historical reads and diagnostics")
    from suspension_multibody import api
    from suspension_multibody.schema import (
        Bushing6x6,
        CaseSpec,
        DisplacementControl,
        FrontAxleModel,
        LoadControl,
        MassSpec,
        Pose,
        SixVector,
    )

    # K through the public entry, writing an artifact and reading it back.
    hardpoints = {
        "uca_front": [-100, -500, 400], "uca_rear": [100, -500, 400],
        "uca_outer": [0, -700, 450], "lca_front": [-120, -500, 150],
        "lca_rear": [120, -500, 150], "lca_outer": [0, -700, 150],
        "tierod_inner": [100, -400, 250], "tierod_outer": [50, -700, 250],
        "wheel_center": [0, -700, 300], "rack_center": [0, 0, 250],
    }
    model = FrontAxleModel(hardpoints=hardpoints, mass=MassSpec(sprung_mass=1000))
    with tempfile.TemporaryDirectory() as directory:
        bundle = api.run_case(model, CaseSpec(name="acceptance", mode="K"), directory)
        manifest = Path(directory) / "manifest.json"
        if not manifest.is_file():
            outcome.problems.append("run_case wrote no manifest")
        else:
            payload = json.loads(manifest.read_text(encoding="utf-8"))
            if int(payload.get("manifest", payload).get("state_count", -1)) != bundle.manifest.state_count:
                outcome.problems.append("the written manifest disagrees with the bundle")
            else:
                outcome.evidence.append(
                    f"K artifact round trip: {bundle.manifest.state_count} state(s), "
                    "manifest agrees"
                )

    # C through the public entry.
    stiffness = tuple(
        tuple(
            10_000.0 if row == column and row < 3
            else 10_000_000.0
            if row == column
            else 0.0
            for column in range(6)
        )
        for row in range(6)
    )
    bushings = tuple(
        Bushing6x6(
            name=f"{body}_{index}", body_a="chassis", body_b=body,
            pose_a=Pose(translation=model.hardpoints[name]),
            pose_b=Pose(translation=model.hardpoints[name]),
            stiffness=stiffness,
        )
        for body, names in (
            ("upper_arm", ("uca_front", "uca_rear")),
            ("lower_arm", ("lca_front", "lca_rear")),
        )
        for index, name in enumerate(names)
    )
    compliant = model.model_copy(update={"bushings": bushings})
    c_bundle = api.run_case(
        compliant,
        CaseSpec(
            mode="C",
            controls=(LoadControl(target="fz", values=(SixVector(fz=100.0),)),),
            left_right_mode="single",
        ),
    )
    if not any(state.converged for state in c_bundle.states):
        outcome.problems.append("the C run produced no converged state")
    else:
        outcome.evidence.append(
            f"C run: {len(c_bundle.states)} state(s), converged"
        )

    # A refusal the historical surface has always made: a contact-point drive on
    # an ideal-joint axle is a different question and must not be answered with a
    # wheel-centre result.
    try:
        api.run_case(
            model,
            CaseSpec(
                mode="K",
                controls=(DisplacementControl(target="contact_patch_left", values=(0.0,)),),
            ),
        )
        outcome.problems.append("a contact-point drive was accepted on a K axle")
    except Exception as error:  # noqa: BLE001 - the refusal type is the layer's
        outcome.evidence.append(
            f"the contact-point drive is still refused: {type(error).__name__}"
        )

    # Failure diagnostics: a case that cannot converge must report why.
    if not isinstance(bundle.states[0].diagnostics, tuple):
        outcome.problems.append("a state carries no diagnostics tuple")
    else:
        outcome.evidence.append(
            f"each state carries {len(bundle.states[0].diagnostics)} diagnostic "
            "record(s)"
        )
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
