"""
The end-to-end acceptance run for the pad-driven K/C work.

This is the epic's Done-When list as executable checks.  Two properties matter and are
deliberate:

* **the expectations are frozen here, not read from the code under test.**  A check that
  computed its expectation from the thing it is checking passes whatever that thing does.
  The numbers below were recorded from a verified state; a regression shows up as a
  failing check rather than as a new expectation.
* **each check runs in its own interpreter.**  The native kernel is a shared library, and
  once a process loads it the file cannot be replaced -- so a check that refreshes the DLL
  mirror fails if an earlier check solved anything in the same process.  Measured: the copy
  succeeds before a solve in-process and fails after one.

Every check names the fixture it uses.  That is not decoration: a criterion asserted on a
fixture where it is trivially true (or trivially false) is not evidence, and the epic
records two rounds of reviews that found exactly that.

Usage::

    uv run --no-sync python scripts/acceptance_pad_driven_kc.py --check all
    uv run --no-sync python scripts/acceptance_pad_driven_kc.py --list
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import cast

REPO = Path(__file__).resolve().parents[1]
PKG = REPO / "packages" / "suspension_multibody"

sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(PKG / "src"))

# The acceptance fixture lives beside this script.  Added to `sys.path` once here so the
# import works however the script is launched, and named explicitly for the type checker,
# which does not follow a run-time path insertion.
from acceptance_pad_kc_fixture import (  # type: ignore[unresolved-import]  # noqa: E402
    compliant_model,  # type: ignore[unresolved-import]
    rigid_model,  # type: ignore[unresolved-import]
)

#: The pad heights the epic's decision 1 names, in millimetres.
PAD_HEIGHTS_MM = (0.0, 10.0, 20.0)

#: The frozen mode-A snapshot, taken before any emission change.
FROZEN_MODE_A = REPO / "artifacts" / "acceptance" / "mode_a_frozen" / "mode_a_snapshot.json"


@dataclass
class Result:
    """One check's outcome."""

    name: str
    passed: bool
    detail: str = ""
    notes: list[str] = field(default_factory=list)
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


def _run(command: list[str], *, env: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    """
    Run a command in the repository, capturing its output.

    Decoding is explicit and lenient: this repository's paths carry characters outside the
    console's default code page, and a check that crashes while *looking* for a problem
    reports neither the problem nor its absence.
    """
    merged = dict(os.environ)
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
    return ["uv", "run", "--no-sync", *args]


def _tail(text: str, count: int = 2) -> str:
    lines = [line for line in text.splitlines() if line.strip()]
    return " | ".join(line.strip() for line in lines[-count:])[:260]


def _pad_assembly(tire_stiffness: float = 200.0, radius: float = 320.0):
    """Return the C reading of the acceptance fixture with tires, ready for a pad sweep."""
    from suspension_multibody.schema import VerticalTire, Vec3
    from suspension_multibody.subsystems.entry import compose_axle

    model = compliant_model().model_copy(
        update={
            "tires": (
                VerticalTire(
                    stiffness=tire_stiffness,
                    unloaded_radius=radius,
                    contact_point=Vec3(x=0.0, y=0.0, z=0.0),
                    local_axis=Vec3(x=0.0, y=0.0, z=1.0),
                ),
            )
        }
    )
    return compose_axle(model, "C")


def _pad_case(heights) -> dict:
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "pad-acceptance",
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "k": {"pad_height_mm": list(heights)},
    }


def _pad_run(heights=PAD_HEIGHTS_MM):
    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.simulation import SimulationRequest, run_request

    assembly = _pad_assembly()
    document = model_document(
        assembly, name="pad-acceptance", drive_wheels=False, drive_mode="pad"
    )
    return run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=document,
            case=_pad_case(heights),
        )
    ).raw


# -- Done-When 1: the three readings are reachable ---------------------------


def check_modes() -> Result:
    """
    Done-When 1: each reading solves, and each declares what the table says.

    Fixtures: A/B on the rigid (K) fixture, C on the compliant (C) fixture with tires --
    they are different physical questions, so one fixture cannot answer all three.
    """
    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.simulation import SimulationRequest, run_request
    from suspension_multibody.subsystems.entry import compose_axle

    notes: list[str] = []
    failures: list[str] = []
    k_case = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "modes-k",
        "time": {"start_s": 0.0, "end_s": 2e-3, "step_s": 1e-3},
        "k": {
            "wheel_values_mm": [-10.0, 0.0, 10.0],
            "rack_values_mm": [0.0],
            "axis_map": {"wheel": ["wheel_drive_L", "wheel_drive_R"], "rack": "rack_drive"},
            "left_right_mode": "symmetric",
        },
    }

    # A and B on the rigid fixture.
    for reading, want_elements, want_tires in (
        ("kinematics", 0, 0),
        ("force_balance", 2, 0),
    ):
        assembly = compose_axle(rigid_model(), "K")
        document = model_document(
            assembly, name=f"modes-{reading}", drive_wheels=True, drive_mode=reading
        )
        elements = cast("list[dict[str, object]]", document["elements"])
        tires = cast("list[dict[str, object]]", document["tires"])
        got_elements = len(elements)
        got_tires = len(tires)
        ok = got_elements == want_elements and got_tires == want_tires
        notes.append(
            f"{reading:14s} elements={got_elements} (want {want_elements}) "
            f"tires={got_tires} (want {want_tires}) {'OK' if ok else 'MISMATCH'}"
        )
        if not ok:
            failures.append(f"{reading} declared the wrong tables")
        try:
            run_request(
                SimulationRequest(
                    assembly="axle",
                    family="kc_quasi_static",
                    model=document,
                    case=k_case,
                )
            )
        except Exception as error:  # noqa: BLE001
            failures.append(f"{reading} did not solve: {type(error).__name__}")
        else:
            notes.append(f"{reading:14s} solved")

    # C (pad) on the compliant fixture with tires.
    try:
        run = _pad_run((0.0,))
    except Exception as error:  # noqa: BLE001
        failures.append(f"pad did not solve: {type(error).__name__}: {str(error)[:80]}")
    else:
        notes.append(f"pad            solved, cases={len(run.cases)} tires={len(run.tire_names)}")
        document = model_document(
            _pad_assembly(), name="modes-pad", drive_wheels=False, drive_mode="pad"
        )
        if len(cast("list[dict[str, object]]", document["tires"])) != 2:
            failures.append("pad declared no tires, so nothing could carry the wheel")

    return Result("1 modes", not failures, "each reading solves and declares its tables", failures + notes)


# -- Done-When 2: mode A is unchanged ----------------------------------------


def check_mode_a() -> Result:
    """
    Done-When 2: `kinematics` reproduces the frozen native snapshot bit for bit.

    Fixture: the acceptance fixture's **K** reading.  Mode A is scoped to K because on C
    the arm mounts *are* the bushing column, so a document with no elements leaves the
    mechanism unconstrained -- measured, the trim stops at ``force_residual = 0.026635``.

    The snapshot is a *native* snapshot, not the retired Python oracle: native and that
    oracle differ in 76 of 108 fields (worst 1.66e-06 mm), so a bit-identical comparison
    against the oracle could never hold.
    """
    from suspension_multibody.cases.kc_quasi_static import case_document, model_document
    from suspension_multibody.cases.kc_quasi_static.workflow import (
        DEFAULT_SETTINGS,
        DEFAULT_TIMES,
    )
    from suspension_multibody.simulation import SimulationRequest, run_request
    from suspension_multibody.subsystems.entry import compose_axle

    if not FROZEN_MODE_A.exists():
        return Result("2 mode-a", False, f"no frozen snapshot at {FROZEN_MODE_A}")

    frozen = json.loads(FROZEN_MODE_A.read_text(encoding="utf-8"))["K"]
    assembly = compose_axle(rigid_model(), "K")
    document = model_document(
        assembly, name="mode-a-k", drive_wheels=True, drive_mode="kinematics"
    )
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="mode-a-k",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        rack_values_mm=(-5.0, 0.0, 5.0),
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=True,
    )
    run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=document,
            case=case,
        )
    ).raw

    declared = cast("list[dict[str, object]]", document["elements"])
    notes = [
        f"elements declared: {len(declared)} (frozen: {len(frozen['model_elements'])})",
    ]
    failures: list[str] = []
    flat = [float(value) for value in run.states.reshape(-1).tolist()]
    if list(run.states.shape) != frozen["states_shape"]:
        failures.append(f"shape {list(run.states.shape)} != frozen {frozen['states_shape']}")
    elif flat != frozen["states"]:
        import numpy as np

        worst = float(np.nanmax(np.abs(np.array(flat) - np.array(frozen["states"]))))
        failures.append(f"states differ; worst |delta| = {worst:.6e}")
    else:
        notes.append("states bit-identical to the frozen native snapshot")

    # A and B must be distinguishable on this fixture, or the criterion above is vacuous.
    b_document = model_document(
        assembly, name="mode-b-k", drive_wheels=True, drive_mode="force_balance"
    )
    b_run = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=b_document,
            case=case,
        )
    ).raw
    import numpy as np

    delta = float(np.nanmax(np.abs(np.asarray(run.states) - np.asarray(b_run.states))))
    if delta == 0.0:
        failures.append("A and B are identical on this fixture, so the check proves nothing")
    else:
        notes.append(f"A and B differ on this fixture: max |delta| = {delta:.6e}")

    return Result("2 mode-a", not failures, "kinematics reproduces the frozen snapshot", failures + notes)


# -- Done-When 3: force balance is the default -------------------------------


def check_default_mode() -> Result:
    """Done-When 3: omitting the reading gives force balance, and it excludes tires."""
    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.compilation import KcStudyInputs, plan_for
    from suspension_multibody.schema.case import CaseSpec
    from suspension_multibody.subsystems.entry import compose_axle

    notes: list[str] = []
    failures: list[str] = []

    spec = CaseSpec(name="default-probe", mode="K")
    if spec.drive_mode != "force_balance":
        failures.append(f"CaseSpec default is {spec.drive_mode!r}, expected 'force_balance'")
    else:
        notes.append("CaseSpec.drive_mode defaults to force_balance")

    plan = plan_for("kc_quasi_static", mode="K", inputs=KcStudyInputs(name="probe"))
    if plan.drive_wheels is not True:
        failures.append("the K bench no longer drives the wheel centres")
    else:
        notes.append("the legacy boolean still resolves drive_wheels=True for K")

    assembly = compose_axle(rigid_model(), "K")
    default_doc = model_document(assembly, name="default")
    explicit = model_document(assembly, name="explicit", drive_mode="force_balance")
    default_elements = cast("list[dict[str, object]]", default_doc["elements"])
    explicit_elements = cast("list[dict[str, object]]", explicit["elements"])
    default_tires = cast("list[dict[str, object]]", default_doc["tires"])
    explicit_tires = cast("list[dict[str, object]]", explicit["tires"])
    same_elements = [row["name"] for row in default_elements] == [
        row["name"] for row in explicit_elements
    ]
    if not same_elements:
        failures.append("the default and force_balance disagree")
    else:
        notes.append(
            f"the default equals force_balance: {len(explicit_elements)} element(s), "
            f"{len(explicit_tires)} tire(s)"
        )
    if default_tires:
        failures.append("force_balance declared tires, which belong to the pad reading")
    else:
        notes.append("force_balance declares no tires, as the mode table states")

    return Result("3 default-mode", not failures, "force balance is the default", failures + notes)


# -- Done-When 4: the pad drives, one case per height ------------------------


def check_pad_drive() -> Result:
    """
    Done-When 4: the sweep is a ground grid, one case per height, wheel drives absent.

    Fixture: the compliant reading with tires, since the pad carries the wheel.
    """
    notes: list[str] = []
    failures: list[str] = []
    run = _pad_run(PAD_HEIGHTS_MM)

    if len(run.cases) != len(PAD_HEIGHTS_MM):
        failures.append(f"{len(run.cases)} case(s) for {len(PAD_HEIGHTS_MM)} pad height(s)")
    else:
        notes.append(f"one case per pad height: {[str(e['name']) for e in run.cases]}")

    document = _pad_run_document()
    joints = cast("list[dict[str, object]]", document["joints"])
    wheel_drives = [
        row["name"]
        for row in joints
        if row.get("type") == "driven_translation" and "wheel_drive" in str(row["name"])
    ]
    if wheel_drives:
        failures.append(f"the wheel centre is still driven: {wheel_drives}")
    else:
        notes.append("no wheel-drive coordinate: the pad is what moves")

    # Each case trims on its own -- the diagnostics carry one trim row per case.
    import numpy as np

    diagnostics = np.asarray(run.diagnostics, dtype=float)
    if diagnostics.ndim != 2 or diagnostics.shape[0] < len(PAD_HEIGHTS_MM):
        failures.append(f"diagnostics {diagnostics.shape} carry no per-case trim row")
    else:
        notes.append(
            f"per-case trim diagnostics present: {diagnostics.shape[0]} row(s) "
            f"for {len(run.cases)} case(s)"
        )

    return Result("4 pad-drive", not failures, "the pad grid expands one case per height", failures + notes)


def _pad_run_document() -> dict:
    from suspension_multibody.cases.kc_quasi_static import model_document

    return model_document(
        _pad_assembly(), name="pad-acceptance", drive_wheels=False, drive_mode="pad"
    )


# -- Done-When 5: the three outputs -----------------------------------------


def check_pad_outputs() -> Result:
    """
    Done-When 5: every pad height reports the wheel centre, the tire load and the contact
    point, and the contact point lies on the pad.

    Fixture: the compliant reading with tires.
    """
    from suspension_multibody.results.kc_state import pad_contact_from_run

    notes: list[str] = []
    failures: list[str] = []
    run = _pad_run(PAD_HEIGHTS_MM)

    worst = 0.0
    for index, pad in enumerate(PAD_HEIGHTS_MM):
        outputs = pad_contact_from_run(run, index)
        if set(outputs) != {"left", "right"}:
            failures.append(f"pad {pad}: sides reported = {sorted(outputs)}")
            continue
        for side, values in outputs.items():
            missing = [
                key
                for key in ("wheel_center_z_mm", "tire_load_n", "contact_z_mm")
                if key not in values
            ]
            if missing:
                failures.append(f"pad {pad} {side}: missing {missing}")
                continue
            worst = max(worst, abs(values["contact_z_mm"] - pad))
    if not failures:
        notes.append(f"{len(PAD_HEIGHTS_MM)} pad heights x 2 sides all reported")
        notes.append(f"worst |contact_z - pad| = {worst:.3e} mm")
    if worst >= 1e-9:
        failures.append(f"the contact point is not on the pad: worst {worst:.3e} mm")

    return Result("5 pad-outputs", not failures, "wheel centre, tire load and contact point", failures + notes)


# -- Done-When 6: the tire-emission rule is itself verified ------------------


def check_tire_emission() -> Result:
    """
    Done-When 6: on a fixture that *declares* a tire, only the pad reading emits it.

    Fixture: a document whose model carries a tire.  This is the check that keeps "mode A
    emits no tire" from being an unexecuted branch: on a tire-free fixture the rule is
    satisfied by the fixture, not by the emitter.
    """
    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.schema import VerticalTire, Vec3
    from suspension_multibody.subsystems.entry import compose_axle

    model = rigid_model().model_copy(
        update={
            "tires": (
                VerticalTire(
                    stiffness=200.0,
                    unloaded_radius=320.0,
                    contact_point=Vec3(x=0.0, y=0.0, z=0.0),
                    local_axis=Vec3(x=0.0, y=0.0, z=1.0),
                ),
            )
        }
    )
    assembly = compose_axle(model, "K")
    notes: list[str] = []
    failures: list[str] = []
    for reading in ("kinematics", "force_balance", "pad"):
        document = model_document(
            assembly, name=f"tires-{reading}", drive_wheels=True, drive_mode=reading
        )
        count = len(cast("list[dict[str, object]]", document["tires"]))
        expected = 2 if reading == "pad" else 0
        notes.append(f"{reading:14s} tires={count} (want {expected})")
        if count != expected:
            failures.append(f"{reading} declared {count} tire(s), expected {expected}")
    return Result("6 tire-emission", not failures, "only the pad reading emits tires", failures + notes)


# -- Done-When 7: the pad range covers what was asked ----------------------


def check_pad_window() -> Result:
    """
    Done-When 7: the reachable pad range covers 0 .. +20 mm.

    Fixture: the compliant reading with the fixture's own springs.  The range is a range,
    not a set of sampled points: ``pad_height_mm`` accepts any list, so covering the
    interval is the requirement.
    """
    notes: list[str] = []
    failures: list[str] = []
    reached: list[float] = []
    for pad in (-40.0, -30.0, -20.0, -10.0, 0.0, 10.0, 20.0, 30.0, 40.0):
        try:
            _pad_run((pad,))
        except Exception as error:  # noqa: BLE001
            message = str(error)
            residual = ""
            if "force_residual=" in message:
                residual = message.split("force_residual=", 1)[1].split(",")[0].strip()
            notes.append(f"pad {pad:+6.1f} FAIL residual={residual or 'n/a'}")
            continue
        reached.append(pad)
        notes.append(f"pad {pad:+6.1f} OK")

    if not reached:
        failures.append("no pad height solved at all")
    else:
        low, high = min(reached), max(reached)
        notes.append(f"reachable range: {low:+.0f} .. {high:+.0f} mm")
        if low > 0.0 or high < 20.0:
            failures.append(
                f"the reachable range {low:+.0f}..{high:+.0f} mm does not cover 0..+20 mm"
            )
        else:
            notes.append("covers the requested 0 .. +20 mm")

    return Result("7 pad-window", not failures, "the pad range covers 0..+20 mm", failures + notes)


# -- Done-When 8: C is force-balanced and carries tires --------------------


def check_c_force_balance() -> Result:
    """
    Done-When 8: the C reading balances its elements and carries tires -- it is no longer
    pure geometry.

    Fixture: the compliant reading with tires, compared against the same fixture under
    `kinematics` (which on C emits no elements at all).
    """
    from suspension_multibody.cases.kc_quasi_static import model_document

    notes: list[str] = []
    failures: list[str] = []
    document = _pad_run_document()
    c_elements = cast("list[dict[str, object]]", document["elements"])
    element_types = {str(row["type"]) for row in c_elements}
    notes.append(f"C element types: {sorted(element_types)}")
    if "spring" not in element_types:
        failures.append("the C reading declares no spring, so nothing balances it")
    if "bushing" not in element_types:
        failures.append("the C reading declares no bushings, which are its mounts")
    if not cast("list[dict[str, object]]", document["tires"]):
        failures.append("the C reading declares no tires")

    # Not pure geometry: force balance and kinematics must disagree on this fixture.
    steady = model_document(
        _pad_assembly(), name="c-steady", drive_wheels=False, drive_mode="force_balance"
    )
    import numpy as np

    from suspension_multibody.cases.kc_quasi_static.workflow import (
        DEFAULT_SETTINGS,
        DEFAULT_TIMES,
    )
    from suspension_multibody.cases.kc_quasi_static import case_document
    from suspension_multibody.simulation import SimulationRequest, run_request
    from suspension_multibody.subsystems.entry import compose_axle

    assembly = compose_axle(compliant_model(), "C")
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="c-steady",
        paths=("fx", "fy", "fz", "mx", "my", "mz"),
        levels=3,
        maximum=1.0,
        side_mode="single",
        times_s=DEFAULT_TIMES,
        settings=DEFAULT_SETTINGS,
        drive_wheels=False,
    )
    balanced = run_request(
        SimulationRequest(
            assembly="axle",
            family="kc_quasi_static",
            model=steady,
            case=case,
        )
    ).raw

    block = balanced.block("bushing_output")
    total = float(np.nansum(np.abs(np.asarray(block))))
    notes.append(f"bushing load carried by the balanced C reading: sum|F| = {total:.6e}")
    if total == 0.0:
        failures.append("the C reading carries no bushing load, so it is not balanced")

    return Result("8 c-force-balance", not failures, "C balances elements and carries tires", failures + notes)


# -- Done-When 9: the road field and the normative schema ------------------


def check_schemas() -> Result:
    """
    Done-When 9: the model accepts a road, and the *normative* schemas accept the new
    case fields.

    The second half is the load-bearing one.  ``multibody_case.schema.json`` has
    ``additionalProperties: false`` at both the top level and under ``k``, so a
    Python-only field is rejected by ``validate_case`` as "unexpected fields".  Asserting
    against the validator is what proves the schema was updated too.
    """
    from suspension_contracts import validate_case
    from suspension_multibody.schema import FrontAxleModel, MassSpec, RoadSurfaceSpec, Vec3

    notes: list[str] = []
    failures: list[str] = []

    model = FrontAxleModel(
        hardpoints={"wheel_center": Vec3(x=0.0, y=-700.0, z=300.0)},
        mass=MassSpec(sprung_mass=1000.0),
        road=RoadSurfaceSpec(kind="plane", origin=Vec3(x=0.0, y=0.0, z=-20.0)),
    )
    if model.road is None or model.road.origin.z != -20.0:
        failures.append("FrontAxleModel did not carry the road it was given")
    else:
        notes.append(f"a model carries a plane road at z={model.road.origin.z}")

    document = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "schema-probe",
        "drive_mode": "pad",
        "k": {"pad_height_mm": [0.0, 20.0]},
    }
    try:
        validate_case(document)
    except Exception as error:  # noqa: BLE001
        failures.append(f"the normative schema rejects the new fields: {str(error)[:120]}")
    else:
        notes.append("validate_case accepts drive_mode and k.pad_height_mm")

    bad = dict(document, drive_mode="bogus")
    try:
        validate_case(bad)
    except Exception:  # noqa: BLE001
        notes.append("validate_case refuses an unknown drive_mode")
    else:
        failures.append("the normative schema accepted an unknown drive_mode")

    return Result("9 schemas", not failures, "the road field and the normative schemas", failures + notes)


# -- Done-When 10: the fast gates ------------------------------------------


def check_fast_gate() -> Result:
    """Done-When 10: lint, type check, the three architecture gates and the fast suite."""
    notes: list[str] = []
    failures: list[str] = []
    checks = (
        ("ruff", _uv("ruff", "check", ".")),
        ("ty", _uv("ty", "check", ".")),
        (
            "legacy-surface",
            _uv(
                "python",
                str(PKG / "tests/architecture/legacy_surface_gate.py"),
                "--check",
            ),
        ),
        (
            "module-layering",
            _uv(
                "python",
                str(REPO / "packages/suspension_kernel/scripts/check_module_layering.py"),
                "--strict",
                "--final",
            ),
        ),
        (
            "composable-release",
            _uv(
                "python",
                str(PKG / "scripts/check_composable_release.py"),
                "--skip-isolation",
            ),
        ),
    )
    for label, command in checks:
        completed = _run(command)
        if completed.returncode == 0:
            notes.append(f"{label}: exit 0")
        else:
            failures.append(f"{label}: exit {completed.returncode} -- {_tail(completed.stderr or completed.stdout)}")

    fast = _run(
        _uv(
            "pytest",
            str(PKG / "tests"),
            "--ignore",
            str(PKG / "tests/adams"),
            "--ignore",
            str(PKG / "tests/architecture"),
            "--ignore",
            str(PKG / "tests/cases"),
            "-q",
            "-p",
            "no:cacheprovider",
        )
    )
    if fast.returncode == 0:
        notes.append(f"fast suite: {_tail(fast.stdout, 1)}")
    else:
        failures.append(f"fast suite: exit {fast.returncode} -- {_tail(fast.stdout)}")

    return Result("10 fast-gate", not failures, "lint, architecture gates and the fast suite", failures + notes)


# -- Done-When 11: the numeric gates and the attribution -------------------


def check_numeric_gate() -> Result:
    """Done-When 11: the numeric gates pass and the baseline attribution is written."""
    notes: list[str] = []
    failures: list[str] = []
    for label, command in (
        ("dynamic-hash", _uv("python", str(PKG / "scripts/dynamic_hash_sentinel.py"), "--check")),
        ("kc-parity", _uv("python", str(PKG / "scripts/kc_parity_check.py"), "--check", "--actual-dir", "artifacts/kc-native-probe")),
        ("case-parity", _uv("python", str(PKG / "scripts/case_parity_check.py"))),
    ):
        completed = _run(command)
        if completed.returncode == 0:
            notes.append(f"{label}: exit 0")
        else:
            failures.append(f"{label}: exit {completed.returncode} -- {_tail(completed.stderr or completed.stdout)}")

    attribution = REPO / "artifacts" / "acceptance" / "baseline-attribution.md"
    if attribution.exists():
        notes.append(f"attribution present: {attribution.relative_to(REPO).as_posix()}")
    else:
        failures.append("no baseline attribution was written")

    independent = REPO / "artifacts" / "pad-kc" / "task12" / "independent-mode-bc.md"
    if independent.exists():
        notes.append("independent mode B/C evidence present")
    else:
        failures.append("the independent mode B/C evidence is missing")

    return Result("11 numeric-gate", not failures, "the numeric gates and the attribution", failures + notes)


# -- Done-When 12: the signature stayed compatible -------------------------


def check_signature() -> Result:
    """
    Done-When 12: every `drive_wheels` call site still works.

    The count is **enumerated, not frozen**: three measurements of it disagreed (72
    keyword uses, 101 lines, 102 under a different exclusion), and a criterion pinned to a
    number that does not reproduce is either vacuous or permanently red.
    """
    import ast

    notes: list[str] = []
    failures: list[str] = []
    completed = _run(["git", "grep", "-n", "-e", "drive_wheels", "--", "packages", "scripts"])
    rows = [line for line in completed.stdout.splitlines() if line.strip()]
    files = sorted({line.split(":", 1)[0] for line in rows})
    keywords = 0
    for name in files:
        path = REPO / name
        if path.suffix != ".py" or not path.exists():
            continue
        try:
            tree = ast.parse(path.read_text(encoding="utf-8", errors="replace"))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                keywords += sum(1 for kw in node.keywords if kw.arg == "drive_wheels")
    notes.append(f"enumerated: {len(rows)} line(s) in {len(files)} file(s), {keywords} call keyword(s)")

    from suspension_multibody.schema.case import drive_mode_for

    for legacy, expected in ((True, "force_balance"), (False, "pad"), (None, "force_balance")):
        got = drive_mode_for(legacy)
        if got != expected:
            failures.append(f"drive_wheels={legacy} maps to {got!r}, expected {expected!r}")
        else:
            notes.append(f"drive_wheels={str(legacy):5s} -> {got}")

    # And the signature really is incremental, so old callers need no change.
    import inspect

    from suspension_multibody.cases.kc_quasi_static.contract import model_document

    signature = inspect.signature(model_document)
    if "drive_wheels" not in signature.parameters:
        failures.append("model_document lost its drive_wheels parameter")
    else:
        parameter = signature.parameters["drive_wheels"]
        notes.append(
            f"model_document(..., drive_wheels={parameter.default!r}) still accepted; "
            f"drive_mode is optional ({signature.parameters['drive_mode'].default!r})"
        )

    return Result("12 signature", not failures, "the legacy boolean still works everywhere", failures + notes)


# -- Done-When 13: independent Adams evidence ------------------------------


def check_adams() -> Result:
    """
    Done-When 13: the independent Adams gate, or a failure naming what is missing.

    The criterion forbids substituting the baseline for this: `kc_baseline` is the thing
    under test's own snapshot, so comparing it to itself proves nothing.  An unavailable
    Adams is a **failure**, not an exemption.
    """
    completed = _run(_uv("suspension-multibody", "validate-adams", "--strict-k", "--require-installed"))
    if completed.returncode == 0:
        return Result("13 adams", True, _tail(completed.stdout, 1))
    combined = completed.stdout + "\n" + completed.stderr
    lowered = combined.lower()
    if "not installed" in lowered or "no adams" in lowered or "license" in lowered:
        return Result(
            "13 adams",
            False,
            "BLOCKED: Adams is unavailable, so the independent evidence is missing",
            [
                _tail(combined),
                "install Adams/Car with a working license, or supply another independent "
                "K/C oracle; this check does not pass on absence",
            ],
        )
    return Result("13 adams", False, f"exit {completed.returncode}", [_tail(combined)])


# -- the registry ------------------------------------------------------------

CHECKS = {
    "modes": check_modes,
    "mode-a": check_mode_a,
    "default-mode": check_default_mode,
    "pad-drive": check_pad_drive,
    "pad-outputs": check_pad_outputs,
    "tire-emission": check_tire_emission,
    "pad-window": check_pad_window,
    "c-force-balance": check_c_force_balance,
    "schemas": check_schemas,
    "fast-gate": check_fast_gate,
    "numeric-gate": check_numeric_gate,
    "signature": check_signature,
    "adams": check_adams,
}

#: The Done-When order, so `--check all` reads like the criterion list.
ALL = (
    "modes",
    "mode-a",
    "default-mode",
    "pad-drive",
    "pad-outputs",
    "tire-emission",
    "pad-window",
    "c-force-balance",
    "schemas",
    "fast-gate",
    "numeric-gate",
    "signature",
    "adams",
)


def _stdout_robust() -> None:
    """
    Stop a decoded subprocess message from killing the report.

    Output is decoded leniently, so an undecodable byte becomes U+FFFD, which the console's
    own code page cannot encode.  Printing it raised after the checks had run, which reads
    as a failed check rather than a failed print.
    """
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, "reconfigure", None)
        if callable(reconfigure):
            try:
                reconfigure(errors="replace")
            except (ValueError, OSError):  # pragma: no cover - stream without a buffer
                pass


def _run_isolated(name: str) -> Result:
    """
    Run one check in its own interpreter and read back its verdict.

    Isolation is not tidiness: the native kernel is a shared library, and once this process
    loads it the file cannot be replaced -- so a check that refreshes the DLL mirror fails
    if an earlier check solved anything here.  A fresh interpreter per check is what makes
    the result independent of the order.
    """
    completed = _run([sys.executable, str(Path(__file__).resolve()), "--one", name])
    output = (completed.stdout or completed.stderr).strip()
    return Result(name=name, passed=completed.returncode == 0, rendered=output)


def main(argv: list[str] | None = None) -> int:
    _stdout_robust()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="append", default=[], help="a check name, or 'all'")
    parser.add_argument("--list", action="store_true", help="print the check names and exit")
    parser.add_argument("--one", metavar="NAME", help="run exactly one check in this process")
    parser.add_argument("--in-process", action="store_true", help="run every check in this process")
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
        results = [_run_isolated(name) for name in selected]

    for result in results:
        print(result.render())
    print()
    failed = [result.name for result in results if not result.passed]
    print(f"{len(results) - len(failed)}/{len(results)} checks passed")
    if failed:
        print("failed: " + ", ".join(failed))
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
