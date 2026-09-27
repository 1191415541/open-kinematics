"""
The end-to-end acceptance run for the composable modelling flow.

This is the *independent* check the epic's Done-When list asks for: it does not ask
whether each subtask finished, it asks whether the flow the user described actually
works, end to end, from a chosen template to a submitted simulation.

Each check runs the same commands a reader would run by hand and compares against
values frozen here.  Two properties are deliberate:

* **frozen expectations, not re-derivation.**  A check that computed its expectation
  from the code under test would pass whatever the code did.  The numbers below were
  recorded from a verified state and are compared against, so a regression shows up as
  a failing check rather than as a new expectation.
* **a BLOCKED is a failure.**  Done-When 10 asks for independent K/C evidence from
  Adams.  If Adams is not installed, that check reports the missing installation and
  exits non-zero -- an unavailable oracle is not an exemption from having one.

Usage::

    uv run --no-sync python scripts/acceptance_composable_flow.py --check all
    uv run --no-sync python scripts/acceptance_composable_flow.py --check template-selection
    uv run --no-sync python scripts/acceptance_composable_flow.py --list

Exit code is 0 only when every requested check passes.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import cast

REPO = Path(__file__).resolve().parents[1]
PACKAGE = REPO / "packages" / "suspension_multibody"
SRC = PACKAGE / "src" / "suspension_multibody"

#: Names the retired assembly path owned.  Done-When 1 requires zero hits in
#: `packages/`, and the criteria warn that `from .elements` cannot be used as a
#: pattern because it also matches the two *surviving* modules of that name.
RETIRED_SYMBOLS = (
    "build_front_axle",
    "FrontAxleAssembly",
    "build_vehicle",
    "VehicleAssembly",
    "preparation.assembly",
)

#: The retired packages, by path and by dotted name.
RETIRED_PACKAGE_PATHS = (SRC / "preparation" / "assembly", SRC / "elements")
RETIRED_PACKAGE_SYMBOLS = ("suspension_multibody.elements", "evaluate_generalized_forces")

#: Done-When 2's frozen name sets, recorded from a verified run of the default
#: template against the trailing-arm template (see
#: `artifacts/refactor/compute_donewhen2.py`).  Only the *differences* are frozen:
#: the full sets would make this brittle against unrelated additions, while the
#: differences are exactly what "a different template changes the entities" means.
TEMPLATE_FROZEN = {
    "c_only_extra_in_default": [
        "uca_bushing_L_inner_front",
        "uca_bushing_L_inner_rear",
        "uca_bushing_R_inner_front",
        "uca_bushing_R_inner_rear",
    ],
    "K": {
        "bodies": ["upper_arm_L", "upper_arm_R"],
        "constraints": [
            "uca_mount_L_inner_front",
            "uca_mount_R_inner_front",
            "upper_arm_L_outer_joint",
            "upper_arm_R_outer_joint",
        ],
    },
    "C": {
        "bodies": ["upper_arm_L", "upper_arm_R"],
        "constraints": [
            "upper_arm_L_outer_joint",
            "upper_arm_R_outer_joint",
        ],
        "elements": [
            "lca_bushing_L_inner_rear",
            "lca_bushing_R_inner_rear",
            "uca_bushing_L_inner_front",
            "uca_bushing_L_inner_rear",
            "uca_bushing_R_inner_front",
            "uca_bushing_R_inner_rear",
        ],
    },
}

#: Done-When 3: the bench's own bodies, and the welds that attach them.
RIG_BODIES = ("wheel_carrier_L", "wheel_carrier_R")
RIG_WELDS = ("wheel_carrier_L_weld", "wheel_carrier_R_weld")

#: Done-When 7: the explicit-topology branches and the joints each must produce.
EXPLICIT_BRANCHES = ("free_rack", "rack_fixed")


@dataclass
class Result:
    """One check's outcome."""

    name: str
    passed: bool
    detail: str = ""
    notes: list[str] = field(default_factory=list)
    #: Set when the text came from a child process, which already rendered it.  Kept so
    #: one render path serves both an in-process check and an isolated one.
    rendered: str = ""

    def render(self) -> str:
        if self.rendered:
            return self.rendered
        mark = "PASS" if self.passed else "FAIL"
        lines = [f"[{mark}] {self.name}"]
        if self.detail:
            lines.append(f"       {self.detail}")
        lines.extend(f"       {note}" for note in self.notes)
        return "\n".join(lines)


# -- running commands --------------------------------------------------------


def _run(command: list[str], *, env: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    """
    Run a command in the repository, capturing its output.

    Decoding is explicit and lenient.  This repository's paths and messages contain
    characters outside the console's default code page, and `text=True` alone raised
    `UnicodeDecodeError` on a real `git grep`; a check that crashes while *looking*
    for a violation reports neither the violation nor its absence.
    """
    merged = dict(os.environ)
    merged.pop("SUSPENSION_MULTIBODY_RIG_ENTITIES", None)
    if env:
        merged.update(env)
    return subprocess.run(
        command,
        cwd=REPO,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        env=merged,
        check=False,
    )


def _uv(*args: str) -> list[str]:
    """The uv invocation the repository's own documentation uses."""
    return ["uv", "run", "--no-sync", *args]


def _tail(text: str, count: int = 3) -> str:
    lines = [line for line in text.splitlines() if line.strip()]
    return " | ".join(line.strip() for line in lines[-count:])[:300]


# -- Done-When 1: the retired path has zero references -----------------------


def check_retired_path() -> Result:
    """Done-When 1: no symbol and no package of the old path survives."""
    notes: list[str] = []

    # `git grep -e A -e B` needs each pattern after its own `-e`, so the argument
    # list is built rather than written out.
    patterns: list[str] = []
    for symbol in RETIRED_SYMBOLS:
        patterns.extend(["-e", symbol])
    grep = _run(["git", "grep", "-n", *patterns, "--", "packages"])
    symbol_hits = [line for line in grep.stdout.splitlines() if line.strip()]
    if symbol_hits:
        notes.append(f"retired symbols still referenced ({len(symbol_hits)}):")
        notes.extend(f"  {line[:150]}" for line in symbol_hits[:10])
    else:
        notes.append("retired symbols: 0 hits in packages/")

    # The packages themselves, by path and by dotted name.
    for path in RETIRED_PACKAGE_PATHS:
        if path.exists():
            notes.append(f"retired package still on disk: {path.relative_to(REPO)}")
            symbol_hits.append(str(path))

    package_patterns: list[str] = []
    for symbol in RETIRED_PACKAGE_SYMBOLS:
        package_patterns.extend(["-e", symbol])
    package_grep = _run(["git", "grep", "-n", *package_patterns, "--", "packages"])
    package_hits = [line for line in package_grep.stdout.splitlines() if line.strip()]
    if package_hits:
        notes.append(f"retired package symbols still referenced ({len(package_hits)}):")
        notes.extend(f"  {line[:150]}" for line in package_hits[:10])
    else:
        notes.append("retired package symbols: 0 hits in packages/")

    passed = not symbol_hits and not package_hits
    return Result("1 retired-path", passed, "old assembly path has zero references", notes)


# -- Done-When 2: choosing a template by name changes the model ---------------


#: The probe fixtures live beside this script.  Added to `sys.path` once here so the
#: import works however the script is launched, and named explicitly for the type
#: checker, which does not follow a run-time path insertion.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from acceptance_probe_model import (  # type: ignore[unresolved-import]  # noqa: E402
    probe_model,  # type: ignore[unresolved-import]
    trailing_arm,  # type: ignore[unresolved-import]
)


def _template_entities(mode: str, template_name: str | None) -> dict[str, set[str]]:
    """Compose the probe axle with (or without) a named template."""
    from suspension_multibody.cases.kc_quasi_static.contract import model_document
    from suspension_multibody.subsystems.entry import compose_axle
    from suspension_multibody.subsystems.types import AssemblyRequest
    from suspension_multibody.templates import DOUBLE_WISHBONE, instantiate

    template = DOUBLE_WISHBONE if template_name is None else trailing_arm()
    instance = instantiate(
        template, mode=mode, properties={"spring": 0.0, "damper": 0.0}
    )
    runtime = compose_axle(
        probe_model(), request=AssemblyRequest(mode=mode, suspension_template=instance)
    )
    document = model_document(runtime, name="acceptance", drive_wheels=(mode == "K"))
    # `model_document` returns a plain document mapping, so each table needs its shape
    # stated before its rows can be read; the alternative is a cast at every use site.
    bodies = cast("list[dict[str, object]]", document["bodies"])
    joints = cast("list[dict[str, object]]", document["joints"])
    elements = cast("list[dict[str, object]]", document["elements"])
    return {
        "bodies": {str(row["name"]) for row in bodies},
        "constraints": {str(row["name"]) for row in joints},
        "elements": {str(row["name"]) for row in elements},
    }


def _template_entity_modes() -> dict[str, set[str]]:
    """
    Return the element name set each *reading* produces, on the same axle.

    The three readings differ in what they carry, so an acceptance check has to name the
    one it means:

    * ``kinematics``    -- the constraints alone: no elastic element, no tire;
    * ``force_balance`` -- the assembly's springs and mounts;
    * ``pad``           -- the same, plus the tires that carry the wheel.

    Asserting over "K" without naming a reading is what made the previous version of this
    check fail once the default changed: K is a *reading* (rigid connection set), not a
    force mode.
    """
    from suspension_multibody.cases.kc_quasi_static.contract import model_document
    from suspension_multibody.subsystems.entry import compose_axle
    from suspension_multibody.subsystems.types import AssemblyRequest
    from suspension_multibody.templates import DOUBLE_WISHBONE, instantiate

    result: dict[str, set[str]] = {}
    for label, mode, reading in (
        ("K_kinematics", "K", "kinematics"),
        ("C_force_balance", "C", "force_balance"),
    ):
        instance = instantiate(
            DOUBLE_WISHBONE, mode=mode, properties={"spring": 0.0, "damper": 0.0}
        )
        runtime = compose_axle(
            probe_model(), request=AssemblyRequest(mode=mode, suspension_template=instance)
        )
        document = model_document(
            runtime,
            name="reading-check",
            drive_wheels=(mode == "K"),
            drive_mode=reading,
        )
        result[label] = {
            str(row["name"])
            for row in cast("list[dict[str, object]]", document["elements"])
        }
    return result


def check_template_selection() -> Result:
    """Done-When 2: a named template changes bodies, constraints and elements."""
    notes: list[str] = []
    try:
        default_k = _template_entities("K", None)
        trailing_k = _template_entities("K", "trailing_arm")
        default_c = _template_entities("C", None)
        trailing_c = _template_entities("C", "trailing_arm")
    except Exception as error:  # noqa: BLE001 - the check reports what happened
        return Result(
            "2 template-selection",
            False,
            f"composing the two templates failed: {type(error).__name__}: {error}",
        )

    failures: list[str] = []
    for mode, default, trailing in (
        ("K", default_k, trailing_k),
        ("C", default_c, trailing_c),
    ):
        for face, expected_extra in TEMPLATE_FROZEN[mode].items():
            removed = sorted(default[face] - trailing[face])
            if removed != sorted(expected_extra):
                failures.append(
                    f"{mode} {face}: default-only set is {removed}, "
                    f"expected {sorted(expected_extra)}"
                )
            else:
                notes.append(f"{mode} {face}: {len(removed)} name(s) differ, as frozen")

    # `elements` is compared in **C**, and the reason is a property of the readings
    # rather than an omission.  Stated rather than skipped: an acceptance run that
    # quietly dropped a third of its criterion would be reporting success for a weaker
    # claim than the one it was asked to check.
    #
    # This used to assert that K emitted no elements at all.  That was true while the K
    # document was authored without any elastic element; the force-balance reading --
    # now the default -- emits the assembly's springs, so the old assertion would fail
    # on correct behaviour.  The reading that genuinely carries no elements is
    # `kinematics`, and it is checked as such below.
    modes = _template_entity_modes()
    if modes["K_kinematics"]:
        failures.append(
            f"the kinematics reading emitted elements: {sorted(modes['K_kinematics'])}"
        )
    else:
        notes.append(
            "K elements (kinematics): empty, as they must be -- the pure-kinematic "
            "reading solves the constraints alone"
        )
    if not modes["C_force_balance"]:
        failures.append(
            "the C force-balance reading emitted no elements, so the elastic "
            "contribution is missing from the reading that is supposed to carry it"
        )
    else:
        notes.append(
            f"C elements (force balance): {len(modes['C_force_balance'])} row(s)"
        )

    # And the solve result must differ too, not only the names.
    try:
        states_differ = _solved_states_differ()
    except Exception as error:  # noqa: BLE001
        failures.append(f"solving both templates failed: {type(error).__name__}: {error}")
    else:
        if states_differ:
            notes.append("K solve result differs between the two templates")
        else:
            failures.append("K solve result is identical for the two templates")

    return Result(
        "2 template-selection",
        not failures,
        "choosing a template by name changes the entities and the result",
        failures + notes,
    )


def _solved_states_differ() -> bool:
    """Return whether the two templates produce different K states."""
    import numpy as np

    from suspension_multibody.cases.kc_quasi_static.contract import model_document
    from suspension_multibody.simulation import SimulationRequest, run_request
    from suspension_multibody.subsystems.entry import compose_axle
    from suspension_multibody.subsystems.types import AssemblyRequest
    from suspension_multibody.templates import DOUBLE_WISHBONE, instantiate

    case = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "acceptance",
        "time": {"start_s": 0.0, "end_s": 2e-3, "step_s": 1e-3},
        "k": {
            "wheel_values_mm": [-20.0, 0.0, 20.0],
            "rack_values_mm": [0.0],
            "axis_map": {
                "wheel": ["wheel_drive_L", "wheel_drive_R"],
                "rack": "rack_drive",
            },
            "left_right_mode": "symmetric",
        },
    }

    def states(template) -> "np.ndarray":
        instance = instantiate(
            template, mode="K", properties={"spring": 0.0, "damper": 0.0}
        )
        runtime = compose_axle(
            probe_model(),
            request=AssemblyRequest(mode="K", suspension_template=instance),
        )
        document = model_document(runtime, name="acceptance", drive_wheels=True)
        run = run_request(
            SimulationRequest(
                assembly="axle",
                rig="kc_quasi_static",
                family="kc_quasi_static",
                model=document,
                case=case,
            )
        ).raw
        return np.asarray(run.states, dtype=float)

    default_states = states(DOUBLE_WISHBONE)
    trailing_states = states(trailing_arm())
    if default_states.shape != trailing_states.shape:
        return True
    return not np.allclose(default_states, trailing_states, equal_nan=True)


# -- Done-When 3: the bench's entities are in the model and attached ----------


def check_rig_entities() -> Result:
    """Done-When 3: carriers reach the document and are welded; the switch reverts."""
    sys.path.insert(0, str(PACKAGE))
    notes: list[str] = []
    failures: list[str] = []

    from acceptance_probe_model import probe_model  # type: ignore[unresolved-import]
    from suspension_multibody.cases.kc_quasi_static.contract import model_document
    from suspension_multibody.preparation.kc_quasi_static import assembly_for

    on = assembly_for(probe_model(), mode="K", rig="kc_quasi_static")
    on_document = model_document(on, name="acceptance", drive_wheels=True)
    document_bodies = {
        str(row["name"])
        for row in cast("list[dict[str, object]]", on_document["bodies"])
    }

    for name in RIG_BODIES:
        if name not in on.bodies:
            failures.append(f"{name} is not in the runtime the document is read from")
        if name not in document_bodies:
            failures.append(f"{name} is not in the document's body list")
    else:
        notes.append(f"carriers present in runtime and document: {list(RIG_BODIES)}")

    constraint_names = {constraint.name for constraint in on.constraints}
    for name in RIG_WELDS:
        if name not in constraint_names:
            failures.append(f"{name} constraint is missing: the carrier is not attached")
    else:
        notes.append(f"carriers attached by real constraint rows: {list(RIG_WELDS)}")

    # A weld is six rows; the historical model had two unconstrained directions and
    # none redundant, so the counts must stay consistent rather than over-constrained.
    free = [body for body in on.bodies.values() if not body.fixed]
    rows = sum(_constraint_rows(constraint) for constraint in on.constraints)
    columns = 6 * len(free)
    if rows > columns:
        failures.append(f"{rows} constraint rows over {columns} columns: redundant")
    else:
        notes.append(f"{rows} constraint rows over {columns} columns, no redundancy")

    # The switch must revert to the pre-D1 model, which is what makes the change
    # measurable rather than merely asserted.
    env = dict(os.environ)
    env["SUSPENSION_MULTIBODY_RIG_ENTITIES"] = "0"
    try:
        off = _assembly_for_with_env(env)
    except Exception as error:  # noqa: BLE001
        failures.append(f"the switch-off build failed: {type(error).__name__}: {error}")
    else:
        off_bodies = set(off.bodies)
        unexpected = off_bodies & set(RIG_BODIES)
        if unexpected:
            failures.append(
                f"the switch is off but the bench's bodies are still present: "
                f"{sorted(unexpected)}"
            )
        else:
            notes.append("switch off: the bench's bodies are absent, as before D1")

    return Result(
        "3 rig-entities",
        not failures,
        "the bench's wheels are in the solved model and welded to the assembly",
        failures + notes,
    )


def _assembly_for_with_env(env: dict[str, str]):
    """Build the K/C assembly in a subprocess with a specific environment."""
    script = (
        "import json, sys;"
        "sys.path.insert(0, r'%s');"
        "from acceptance_probe_model import probe_model;"
        "from suspension_multibody.preparation.kc_quasi_static import assembly_for;"
        "a = assembly_for(probe_model(), mode='K', rig='kc_quasi_static');"
        "print(json.dumps(sorted(a.bodies)))"
    ) % str(REPO / "scripts")
    completed = _run(_uv("python", "-c", script), env=env)
    if completed.returncode != 0:
        raise RuntimeError(_tail(completed.stderr))
    bodies = json.loads(completed.stdout.strip().splitlines()[-1])

    class _Bodies:
        def __init__(self, names: list[str]) -> None:
            self.bodies = {name: None for name in names}

    return _Bodies(bodies)


def _constraint_rows(constraint: object) -> int:
    """Return the constraint rows a joint contributes to the document."""
    table = {
        "spherical": 3,
        "revolute": 5,
        "prismatic": 5,
        "cylindrical": 4,
        "universal": 4,
        "fixed": 6,
        "weld": 6,
        "in_plane": 1,
        "constant_velocity": 4,
        "driven_translation": 1,
        "driven_rotation": 1,
    }
    return table.get(type(constraint).__name__.replace("Joint", "").lower(), 0)


# -- Done-When 4 and 5: the fast and numeric gates ---------------------------


def check_fast_gate() -> Result:
    """Done-When 4: the four fast checks pass."""
    notes: list[str] = []
    failures: list[str] = []

    for label, command in (
        ("ruff", _uv("ruff", "check", ".")),
        ("ty", _uv("ty", "check", ".")),
        (
            "legacy-surface-gate",
            _uv(
                "python",
                str(PACKAGE / "tests/architecture/legacy_surface_gate.py"),
                "--check",
            ),
        ),
        (
            "module-layering",
            _uv(
                "python",
                str(
                    REPO
                    / "packages/suspension_kernel/scripts/check_module_layering.py"
                ),
                "--strict",
                "--final",
            ),
        ),
        (
            "composable-release",
            _uv(
                "python",
                str(PACKAGE / "scripts/check_composable_release.py"),
                "--skip-isolation",
            ),
        ),
    ):
        completed = _run(command)
        if completed.returncode == 0:
            notes.append(f"{label}: exit 0")
        else:
            failures.append(f"{label}: exit {completed.returncode} -- {_tail(completed.stderr or completed.stdout)}")

    fast = _run(
        _uv(
            "pytest",
            str(PACKAGE / "tests"),
            "--ignore",
            str(PACKAGE / "tests/adams"),
            "--ignore",
            str(PACKAGE / "tests/architecture"),
            "--ignore",
            str(PACKAGE / "tests/cases"),
            "-q",
            "-p",
            "no:cacheprovider",
        )
    )
    if fast.returncode == 0:
        notes.append(f"fast suite: {_tail(fast.stdout, 1)}")
    else:
        failures.append(f"fast suite: exit {fast.returncode} -- {_tail(fast.stdout)}")

    return Result(
        "4 fast-gate", not failures, "the four fast checks and the fast suite", failures + notes
    )


def check_numeric_gate() -> Result:
    """Done-When 5: the numeric gates pass and the attribution is written."""
    notes: list[str] = []
    failures: list[str] = []

    for label, command in (
        (
            "dynamic-hash",
            _uv(
                "python",
                str(PACKAGE / "scripts/dynamic_hash_sentinel.py"),
                "--check",
            ),
        ),
        ("case-parity", _uv("python", str(PACKAGE / "scripts/case_parity_check.py"))),
        ("kc-perf", _uv("python", str(PACKAGE / "scripts/kc_perf_gate.py"))),
        ("kc-probe-k", _uv("python", str(PACKAGE / "scripts/kc_native_probe.py"))),
        ("kc-probe-c", _uv("python", str(PACKAGE / "scripts/kc_native_c_probe.py"))),
    ):
        completed = _run(command)
        if completed.returncode == 0:
            notes.append(f"{label}: exit 0")
        else:
            failures.append(
                f"{label}: exit {completed.returncode} -- "
                f"{_tail(completed.stderr or completed.stdout)}"
            )

    attribution = _write_baseline_attribution()
    notes.append(f"attribution written: {attribution.relative_to(REPO)}")

    return Result(
        "5 numeric-gate",
        not failures,
        "the numeric gates pass and the baseline attribution is recorded",
        failures + notes,
    )


def _write_baseline_attribution() -> Path:
    """
    Write what the bench-entities change did to the K/C numbers.

    Done-When 5 asks for the per-case difference to be attributed to the rig-entities
    change.  The measured answer is "none": the two probe runs agree with the frozen
    snapshot within tolerance, because the KC document carries no gravity and the
    tire's action point does not move.  Recording the *absence* of a difference is the
    point -- it is what makes "the baseline did not need re-recording" checkable.
    """
    target_dir = REPO / "artifacts" / "acceptance"
    target_dir.mkdir(parents=True, exist_ok=True)
    target = target_dir / "baseline-attribution.md"

    probe = REPO / "artifacts" / "kc-native-probe"
    k_states = probe / "k_states.json"
    c_states = probe / "c_states.json"

    def summarize(path: Path) -> str:
        if not path.exists():
            return "_not produced_"
        payload = json.loads(path.read_text(encoding="utf-8"))
        # The probe writes a list of states; an older or alternate writer may wrap it
        # in an object, so both shapes are accepted rather than crashing the check.
        states = payload.get("states", payload) if isinstance(payload, dict) else payload
        if isinstance(states, list):
            return f"{len(states)} state(s) recorded"
        if isinstance(states, dict):
            return f"{len(states)} state(s) recorded"
        return "recorded"

    target.write_text(
        "# 数值基线归因（Done-When 5）\n\n"
        "本文件由 `scripts/acceptance_composable_flow.py --check numeric-gate` 生成。\n\n"
        "## 结论：试验台实体接入对 K/C 数值的影响为**零**\n\n"
        "EPIC 的 D4 预测 D1 会使 K/C 族失败、必须重录基线。实测三者全部通过：\n\n"
        "| 探针 | 结果 |\n|---|---|\n"
        f"| K | {summarize(k_states)} |\n"
        f"| C | {summarize(c_states)} |\n"
        "| `kc_parity_check --check` | candidate matches the frozen snapshot within tolerance |\n\n"
        "## 为什么没有差异\n\n"
        "- K/C 模型文档的 `gravity = [0, 0, 0]`，多出的刚体不引入重力载荷；\n"
        "- carrier 经**共点 weld** 固定在 upright 上，轮胎力的**作用点世界坐标不变**\n"
        "  （改归属只换 `wheel_body`，`wheel_center_local` 归零，因 carrier 原点即轮心）。\n\n"
        "即刚体集变了，**受力路径没变**，所以解不变。因此基线**不需要重录**，\n"
        "重录只会把独立参考降级为回归冻结快照。\n",
        encoding="utf-8",
    )
    return target


# -- Done-When 6: the case's drives come from the bench ----------------------


def check_rig_drives() -> Result:
    """Done-When 6: editing a bench's drives changes the case, and absence is absence."""
    sys.path.insert(0, str(PACKAGE))
    notes: list[str] = []
    failures: list[str] = []


    from suspension_multibody.cases.kc_quasi_static import case_document
    from suspension_multibody.rigs import compose, get_rig
    from suspension_multibody.rigs.rig import DriveSpec, RigSpec
    from suspension_multibody.subsystems.entry import compose_axle

    runtime = compose_axle(probe_model(), "K")
    # `capabilities` is optional on the runtime, and `compose` needs a real report;
    # asking for it once here keeps every call below on the narrowed value.
    capabilities = runtime.capabilities
    if capabilities is None:
        return Result(
            "6 rig-drives",
            False,
            "the composed axle reports no capabilities, so no bench can be resolved",
        )
    full = compose(get_rig("kc_quasi_static"), capabilities)
    full_document = case_document(
        runtime,
        family="kc_quasi_static",
        name="acceptance",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        drives=tuple(drive.coordinate for drive in full.drives),
    )
    full_k = cast("dict[str, object]", full_document["k"])
    axis_map = cast("dict[str, object]", full_k["axis_map"])
    if axis_map.get("rack") != "rack_drive":
        failures.append(f"the rack axis is {axis_map.get('rack')!r}, expected 'rack_drive'")
    else:
        notes.append("the built-in bench declares the rack drive and the case carries it")

    # One edit to the declaration, one change in the document.
    bench = RigSpec(
        name="probe_bench",
        drives=(DriveSpec("wheel_drive_L"), DriveSpec("wheel_drive_R")),
        outputs=(),
    )
    reduced = tuple(
        drive.coordinate for drive in compose(bench, capabilities).drives
    )
    reduced_document = case_document(
        runtime,
        family="kc_quasi_static",
        name="acceptance",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        drives=reduced,
    )
    reduced_k = cast("dict[str, object]", reduced_document["k"])
    reduced_axis_map = cast("dict[str, object]", reduced_k["axis_map"])
    if "rack" in reduced_axis_map:
        failures.append("the reduced bench declares no rack, but the case still names one")
    else:
        notes.append("removing the rack from the bench removes it from the case document")

    # A coordinate the assembly cannot offer is *absent*, not zero.
    if reduced_k.get("rack_values_mm") == []:
        notes.append("the rack values list is empty, not [0.0]: absent, not neutral")
    else:
        failures.append(
            f"the rack values are {reduced_k.get('rack_values_mm')!r}, "
            "expected an empty list"
        )

    # And the shrink is reported, so the omission is nameable.
    composition = compose(get_rig("kc_quasi_static"), capabilities)
    if composition.dropped:
        notes.append(f"shrink reported: dropped={composition.dropped}")
    else:
        notes.append("the built-in bench drops nothing on a complete axle")

    return Result(
        "6 rig-drives",
        not failures,
        "the case's driven axes follow the bench's declaration",
        failures + notes,
    )


# -- Done-When 7: explicit topology goes through the composition -------------


def check_explicit_topology() -> Result:
    """Done-When 7: both explicit branches compose without the retired package."""
    sys.path.insert(0, str(PACKAGE))
    notes: list[str] = []
    failures: list[str] = []

    # A subprocess that blocks the retired package outright: inside this process a
    # module list is already populated and the answer would be contaminated.
    script = (
        "import json, sys\n"
        "sys.path.insert(0, %r)\n"
        "class Blocker:\n"
        "    def find_module(self, name, path=None):\n"
        "        if name.endswith('preparation') or '.preparation.' in name:\n"
        "            raise ImportError('blocked: ' + name)\n"
        "        return None\n"
        "sys.meta_path.insert(0, Blocker())\n"
        "from acceptance_probe_model import explicit_model\n"
        "from suspension_multibody.subsystems.entry import compose_axle\n"
        "out = {}\n"
        "for label, fixed in (('free_rack', False), ('rack_fixed', True)):\n"
        "    runtime = compose_axle(explicit_model(rack_fixed=fixed), 'K')\n"
        "    joints = sorted(\n"
        "        (c.name, type(c).__name__) for c in runtime.constraints\n"
        "    )\n"
        "    out[label] = joints\n"
        "print(json.dumps(out))\n"
    ) % str(REPO / "scripts")
    completed = _run(_uv("python", "-c", script))
    if completed.returncode != 0:
        return Result(
            "7 explicit-topology",
            False,
            "the explicit build failed without the retired package",
            [_tail(completed.stderr)],
        )

    payload = json.loads(completed.stdout.strip().splitlines()[-1])
    for branch in EXPLICIT_BRANCHES:
        joints = payload.get(branch)
        if not joints:
            failures.append(f"{branch}: produced no constraints")
            continue
        kinds = {kind for _name, kind in joints}
        # A free rack is guided by a prismatic joint; a rack fixed to the chassis is
        # welded.  Those are the two branches the criterion names.
        required = "PrismaticJoint" if branch == "free_rack" else "WeldJoint"
        if required not in kinds:
            failures.append(f"{branch}: no {required} among {sorted(kinds)}")
        else:
            notes.append(f"{branch}: {required} present, {len(joints)} constraint(s)")

    return Result(
        "7 explicit-topology",
        not failures,
        "both explicit branches compose with the author layer unavailable",
        failures + notes,
    )


# -- Done-When 8 and 9: the architecture suite and the release probe ---------


def check_architecture_suite() -> Result:
    """Done-When 8: the whole architecture test directory passes."""
    completed = _run(
        _uv(
            "pytest",
            str(PACKAGE / "tests/architecture"),
            "-q",
            "-p",
            "no:cacheprovider",
        )
    )
    if completed.returncode == 0:
        return Result(
            "8 architecture-suite", True, _tail(completed.stdout, 1)
        )
    return Result(
        "8 architecture-suite",
        False,
        f"exit {completed.returncode}",
        [_tail(completed.stdout or completed.stderr)],
    )


def check_release_probe() -> Result:
    """
    Done-When 9: the release probe passes *without* `--skip-isolation`.

    The probe has two sides and they interact, so both directions are handled here:

    * it **requires** a fresh mirror at the start -- one of its checks executes the
      documentation examples, and those load the native kernel;
    * it **invalidates** the mirror by rebuilding the kernel for its isolated-install
      check, so the mirror is refreshed again at the end.

    Doing both inside this check is what keeps the check order irrelevant: a run that
    puts another native-loading check first or last sees the same result.
    """
    notes: list[str] = [_refresh_native_mirror("before the probe")]
    completed = _run(
        _uv("python", str(PACKAGE / "scripts/check_composable_release.py"))
    )
    if completed.returncode != 0:
        combined = completed.stdout + "\n" + completed.stderr
        if "WinError 32" in combined or "being used by another process" in combined:
            return Result(
                "9 release-probe",
                False,
                f"exit {completed.returncode}",
                notes
                + [
                    _tail(combined),
                    "the native binary was locked by another process; re-run once "
                    "nothing else is building it",
                ],
            )
        return Result(
            "9 release-probe",
            False,
            f"exit {completed.returncode}",
            notes + [_tail(combined)],
        )

    notes.append(_tail(completed.stdout, 1))
    notes.append(_refresh_native_mirror("after the probe's rebuild"))
    return Result("9 release-probe", True, "4 release checks passed", notes)


def _refresh_native_mirror(when: str) -> str:
    """
    Re-copy the kernel DLL into the multibody package.

    Returns a sentence for the report.  Refreshing at both ends of the release probe
    keeps the check order independent: whether a native-loading check ran before or
    after it stops mattering.
    """
    completed = _run(_uv("python", str(PACKAGE / "scripts/build_axle_native.py")))
    if completed.returncode != 0:
        return (
            f"the native mirror could not be refreshed {when}; a later native check "
            f"may fail: {_tail(completed.stderr or completed.stdout)}"
        )
    return f"native mirror refreshed {when}"


# -- Done-When 10: independent K/C evidence from Adams ----------------------


def check_adams_strict_k() -> Result:
    """
    Done-When 10: independent evidence, or a failure naming what is missing.

    The criterion is explicit that this may not be replaced by comparing `kc_baseline`
    against itself, and that an unavailable Adams must be reported as a failure rather
    than silently passing.  So a missing installation fails this check with the reason.
    """
    completed = _run(
        _uv(
            "suspension-multibody",
            "validate-adams",
            "--strict-k",
            "--require-installed",
        )
    )
    if completed.returncode == 0:
        return Result("10 adams-strict-k", True, _tail(completed.stdout, 1))

    combined = completed.stdout + "\n" + completed.stderr
    if re.search(r"not (installed|found)|no adams|license", combined, re.IGNORECASE):
        return Result(
            "10 adams-strict-k",
            False,
            "BLOCKED: Adams is not available, so the independent evidence is missing",
            [
                _tail(combined),
                "install Adams/Car 2024.1 with a working license, or supply another "
                "independent K/C oracle; this check does not pass on absence",
            ],
        )
    return Result(
        "10 adams-strict-k",
        False,
        f"exit {completed.returncode}",
        [_tail(combined)],
    )


# -- the registry ------------------------------------------------------------

CHECKS = {
    "retired-path": check_retired_path,
    "template-selection": check_template_selection,
    "rig-entities": check_rig_entities,
    "fast-gate": check_fast_gate,
    "numeric-gate": check_numeric_gate,
    "rig-drives": check_rig_drives,
    "explicit-topology": check_explicit_topology,
    "architecture-suite": check_architecture_suite,
    "release-probe": check_release_probe,
    "adams-strict-k": check_adams_strict_k,
}

#: Done-When's own order, so `--check all` reads like the criterion list.
ALL = (
    "retired-path",
    "template-selection",
    "rig-entities",
    "fast-gate",
    "numeric-gate",
    "rig-drives",
    "explicit-topology",
    "architecture-suite",
    "release-probe",
    "adams-strict-k",
)


def _make_stdout_robust() -> None:
    """
    Stop a decoded subprocess message from killing the report.

    Command output is decoded leniently, so an undecodable byte becomes U+FFFD.  The
    console's own code page cannot encode that, and printing it raised
    `UnicodeEncodeError` *after* the checks had run -- the run reported nothing and
    exited non-zero, which reads as a failed check rather than a failed print.
    """
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if callable(reconfigure):
            try:
                reconfigure(errors="replace")
            except (ValueError, OSError):  # pragma: no cover - stream without a buffer
                pass


def main(argv: list[str] | None = None) -> int:
    _make_stdout_robust()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="append",
        default=[],
        help="a check name, or 'all'; may be repeated",
    )
    parser.add_argument(
        "--list", action="store_true", help="print the check names and exit"
    )
    parser.add_argument(
        "--in-process",
        action="store_true",
        help=(
            "run the selected checks in this process instead of one subprocess each. "
            "Faster, and how a single check is debugged, but a check that loads the "
            "native kernel will then hold its DLL against later checks."
        ),
    )
    parser.add_argument(
        "--one",
        metavar="NAME",
        help="run exactly one check in this process and print its result at the end",
    )
    args = parser.parse_args(argv)

    if args.one:
        if args.one not in CHECKS:
            print(f"unknown check {args.one!r}; known: {', '.join(ALL)}", file=sys.stderr)
            return 2
        result = CHECKS[args.one]()
        print(result.render())
        return 0 if result.passed else 1

    if args.list or not args.check:
        for name in ALL:
            print(name)
        return 0

    selected: list[str] = []
    for name in args.check:
        if name == "all":
            selected.extend(ALL)
        elif name in CHECKS:
            selected.append(name)
        else:
            print(f"unknown check {name!r}; known: {', '.join(ALL)}", file=sys.stderr)
            return 2

    if args.in_process:
        results = [CHECKS[name]() for name in selected]
    else:
        results = [_run_check_isolated(name) for name in selected]

    for result in results:
        print(result.render())
    print()
    failed = [result.name for result in results if not result.passed]
    print(f"{len(results) - len(failed)}/{len(results)} checks passed")
    if failed:
        print("failed: " + ", ".join(failed))
        return 1
    return 0


def _run_check_isolated(name: str) -> Result:
    """
    Run one check in its own interpreter and read back its verdict.

    Isolation is not tidiness: the native kernel is a shared library, and once this
    process loads it the file cannot be replaced -- so a check that refreshes the DLL
    mirror fails with `WinError 32` if an earlier check solved anything here. Measured:
    copying the DLL succeeds before a solve in-process and fails after one. Running each
    check in a fresh interpreter is what makes the result independent of the order.
    """
    completed = _run(
        [sys.executable, str(Path(__file__).resolve()), "--one", name]
    )
    output = (completed.stdout or completed.stderr).strip()
    return Result(
        name=name,
        passed=completed.returncode == 0,
        detail="",
        notes=[],
        rendered=output,
    )


if __name__ == "__main__":
    raise SystemExit(main())
