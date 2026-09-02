"""Strict pure-kinematic Adams/Car comparison on a fixed 3x3 input grid."""

from __future__ import annotations

import hashlib
import json
import math
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

from ..analysis import KModeSolver
from ..model import build_front_axle
from ..schema import FrontAxleModel, MassSpec
from .adapter import SmokeResult, Tolerance
from .equivalent_model import write_equivalent_sources
from .probe import AdamsProfile, _adams_environment
from .reference import _read_hardpoints

CONTRACT = "strict-adams-k-v1"
SCHEMA_VERSION = 1
WHEEL_VALUES_MM = (-10.0, 0.0, 10.0)
RACK_VALUES_MM = (-5.0, 0.0, 5.0)
POSITION_FIELDS = (
    "left_wheel_center_x_mm",
    "left_wheel_center_y_mm",
    "left_wheel_center_z_mm",
    "right_wheel_center_x_mm",
    "right_wheel_center_y_mm",
    "right_wheel_center_z_mm",
)
ANGLE_FIELDS = (
    "left_toe_deg",
    "left_camber_deg",
    "right_toe_deg",
    "right_camber_deg",
)


def validate_strict_k(
    profile: AdamsProfile,
    *,
    evidence_dir: str | Path | None = None,
) -> SmokeResult:
    """Run independent pure-K Adams and suspension_multibody solves and compare all points."""
    destination = (
        Path(evidence_dir)
        if evidence_dir is not None
        else Path(tempfile.mkdtemp(prefix="suspension_multibody_strict_k_evidence_"))
    )
    destination.mkdir(parents=True, exist_ok=True)
    if not profile.available:
        return SmokeResult(False, profile.message, profile)

    try:
        manifest = build_equivalence_manifest(profile)
        manifest_path = destination / "equivalence_manifest.json"
        _write_json(manifest_path, manifest)
        runtime = Path(
            tempfile.mkdtemp(prefix="suspension_multibody_strict_k_runtime_")
        )
        adams_states, execution = run_adams_pure_k(profile, manifest, runtime)
        reference_states = run_suspension_multibody_pure_k(profile)
        comparison = compare_k_states(reference_states, adams_states)
        _write_json(destination / "adams_k_results.json", {"states": adams_states})
        _write_json(
            destination / "suspension_multibody_k_results.json",
            {"states": reference_states},
        )
        _write_json(destination / "adams_execution_evidence.json", execution)
        report = {
            "contract": CONTRACT,
            "schema_version": SCHEMA_VERSION,
            "passed": comparison["passed"],
            "manifest_sha256": _sha256(manifest_path),
            "state_count": len(adams_states),
            "comparison": comparison,
            "execution_evidence": str(destination / "adams_execution_evidence.json"),
        }
        report_path = destination / "comparison_report.json"
        _write_json(report_path, report)
    except Exception as exc:
        report = {
            "contract": CONTRACT,
            "schema_version": SCHEMA_VERSION,
            "passed": False,
            "error": str(exc),
        }
        report_path = destination / "comparison_report.json"
        _write_json(report_path, report)
        return SmokeResult(
            False,
            f"Strict Adams pure-K validation failed: {exc}",
            profile,
            str(report_path),
            report,
        )

    passed = bool(report["passed"])
    message = (
        f"Strict Adams pure-K validation passed; report: {report_path}"
        if passed
        else f"Strict Adams pure-K validation exceeded tolerance; report: {report_path}"
    )
    return SmokeResult(passed, message, profile, str(report_path), report)


def build_equivalence_manifest(profile: AdamsProfile) -> dict[str, Any]:
    """Build the canonical non-proprietary input shared by both K runners."""
    database = _database(profile)
    subsystem = database / "subsystems.tbl" / str(profile.subsystem_id)
    steering = database / "subsystems.tbl" / "TR_Steering.sub"
    hardpoints = _read_hardpoints(subsystem)
    steering_points = _read_hardpoints(steering)
    required = {
        "uca_front",
        "uca_rear",
        "uca_outer",
        "lca_front",
        "lca_rear",
        "lca_outer",
        "tierod_inner",
        "tierod_outer",
        "wheel_center",
    }
    missing = sorted(required - hardpoints.keys())
    if missing:
        raise ValueError(f"Adams suspension is missing hardpoints: {missing}")
    if "tierod_inner" not in steering_points:
        raise ValueError("Adams steering is missing tierod_inner")

    mapped = {
        name: [-hardpoints[name][0], hardpoints[name][1], hardpoints[name][2]]
        for name in sorted(required)
    }
    rack = steering_points["tierod_inner"]
    mapped["rack_center"] = [-rack[0], 0.0, rack[2]]
    physical_input = {
        "coordinates": {
            "adams": "+X forward,+Y right,+Z up",
            "suspension_multibody": "+X rearward,+Y right,+Z up",
            "adams_to_suspension_multibody": [[-1, 0, 0], [0, 1, 0], [0, 0, 1]],
            "length_unit": "mm",
            "angle_unit": "deg",
        },
        "hardpoints_mm": mapped,
        "adams_template_hardpoints_mm": {
            name: list(hardpoints[name]) for name in sorted(hardpoints)
        },
        "initial_alignment_deg": {
            "camber_left": 0.0,
            "camber_right": 0.0,
            "toe_left": 0.0,
            "toe_right": 0.0,
        },
        "wheel": {
            "radius_mm": 300.0,
            "mass_kg": 1.0,
            "inertia_kg_mm2": [1.0, 1.0, 1.0],
        },
        "rack_axis": [0.0, 1.0, 0.0],
        "drive": {
            "kind": "wheel_center",
            "pattern": "parallel",
            "wheel_travel_mm": list(WHEEL_VALUES_MM),
            "rack_displacement_mm": list(RACK_VALUES_MM),
            "state_count": 9,
        },
        "boundaries": {
            "analysis_mode": "kinematic",
            "kinematic_flag": 1,
            "compliance_matrix_flag": 0,
            "compliance_objects_flag": 0,
            "gravity_contributes": False,
            "contact_contributes": False,
            "elastic_forces_contribute": False,
            "external_loads_contribute": False,
        },
    }
    canonical = json.dumps(physical_input, sort_keys=True, separators=(",", ":"))
    canonical_hash = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    source = {
        "profile": profile.name,
        "version": profile.version,
        "template_id": profile.template_id,
        "subsystem_id": profile.subsystem_id,
        "subsystem_sha256": _sha256(subsystem),
        "steering_sha256": _sha256(steering),
    }
    assembly = database / "assemblies.tbl" / "mdi_front_vehicle.asy"
    if assembly.is_file():
        source["assembly_sha256"] = _sha256(assembly)
    return {
        "contract": CONTRACT,
        "schema_version": SCHEMA_VERSION,
        "physical_input": physical_input,
        "adams_snapshot_sha256": canonical_hash,
        "suspension_multibody_snapshot_sha256": canonical_hash,
        "source": source,
    }


def run_suspension_multibody_pure_k(profile: AdamsProfile) -> list[dict[str, Any]]:
    """Solve the fixed grid from installed hardpoints without reading Adams results."""
    manifest = build_equivalence_manifest(profile)
    physical = manifest["physical_input"]
    model = FrontAxleModel(
        name="strict_adams_car_k_equivalent",
        hardpoints=physical["hardpoints_mm"],
        mass=MassSpec(sprung_mass=1.0),
    )
    assembly = build_front_axle(model, "K")
    solver = KModeSolver()
    states: list[dict[str, Any]] = []
    for wheel in WHEEL_VALUES_MM:
        for rack in RACK_VALUES_MM:
            solved = solver.solve(
                assembly,
                wheel_travel_left=wheel,
                wheel_travel_right=wheel,
                rack_displacement=rack,
                drive="wheel_center",
                case_id=_case_id(wheel, rack),
            )
            if not solved.equilibrium.converged:
                raise RuntimeError(
                    f"suspension_multibody did not converge for {solved.case_id}"
                )
            metric = solved.metrics
            states.append(
                {
                    "case_id": solved.case_id,
                    "wheel_travel_mm": wheel,
                    "rack_displacement_mm": rack,
                    **{
                        f"{side}_wheel_center_{axis}_mm": metric[
                            f"{side}_wheel_center_{axis}"
                        ]
                        for side in ("left", "right")
                        for axis in ("x", "y", "z")
                    },
                    **{
                        f"{side}_{angle}_deg": metric[f"{side}_{angle}_deg"]
                        for side in ("left", "right")
                        for angle in ("toe", "camber")
                    },
                }
            )
    return states


def run_adams_pure_k(
    profile: AdamsProfile,
    manifest: dict[str, Any],
    runtime: Path,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    """Generate Adams/Car K models, rerun them with simulate/kinematics, and parse results."""
    runtime.mkdir(parents=True, exist_ok=True)
    local_assembly, generated_hashes = _write_pure_k_sources(profile, manifest, runtime)
    command_file = runtime / "strict_k_generate.cmd"
    command_file.write_text(_generation_command(local_assembly), encoding="ascii")
    executable = Path(profile.executable or "")
    if not executable.is_file():
        raise FileNotFoundError("Adams launcher is unavailable")
    car = _run_process(
        executable,
        ("acar", "ru-acar", "b", str(command_file)),
        runtime,
        timeout=600,
    )
    (runtime / "adams_car.stdout.txt").write_text(
        car.stdout or "", encoding="utf-8", errors="replace"
    )
    (runtime / "adams_car.stderr.txt").write_text(
        car.stderr or "", encoding="utf-8", errors="replace"
    )
    status = runtime / "strict_k_generate_status.txt"
    if (
        car.returncode != 0
        or not status.is_file()
        or status.read_text().strip() != "error=0"
    ):
        raise RuntimeError(
            f"Adams/Car K model generation failed with code {car.returncode}"
        )

    cases: list[dict[str, Any]] = []
    solver_evidence: list[dict[str, Any]] = []
    for wheel in WHEEL_VALUES_MM:
        for rack in RACK_VALUES_MM:
            stem = _stem(wheel, rack)
            acf = runtime / f"{stem}.acf"
            acf_text = acf.read_text(encoding="utf-8", errors="strict")
            pure_text, replacements = re.subn(
                r"(?m)^simulate/static,\s*end=([^,]+),\s*steps=\d+\s*$",
                r"simulate/kinematics, end=\1, steps=1",
                acf_text,
            )
            if replacements != 1:
                raise RuntimeError(
                    f"unexpected Adams ACF simulation command in {acf.name}"
                )
            acf.write_text(pure_text, encoding="utf-8")
            environment = {"MDI_PRODUCT_NAME": "acar"}
            solver = _run_process(
                executable,
                ("ru-standard", stem),
                runtime,
                timeout=300,
                environment=environment,
            )
            solver_log = runtime / f"{stem}.pure_k.stdout.txt"
            solver_log.write_text(
                solver.stdout or "", encoding="utf-8", errors="replace"
            )
            msg = runtime / f"{stem}.msg"
            msg_text = (
                msg.read_text(encoding="utf-8", errors="replace")
                if msg.is_file()
                else ""
            )
            if (
                solver.returncode != 0
                or "Performing Kinematic Simulation" not in msg_text
                or "Simulate status=0" not in msg_text
            ):
                raise RuntimeError(f"Adams pure-K solve failed for {stem}")
            state = _parse_kinematic_result(runtime / f"{stem}.res")
            expected_id = _case_id(wheel, rack)
            state.update(
                case_id=expected_id, wheel_travel_mm=wheel, rack_displacement_mm=rack
            )
            if abs(state.pop("adams_wheel_travel_left_mm") - wheel) > 1e-6:
                raise RuntimeError(f"Adams left wheel input mismatch for {expected_id}")
            if abs(state.pop("adams_wheel_travel_right_mm") - wheel) > 1e-6:
                raise RuntimeError(
                    f"Adams right wheel input mismatch for {expected_id}"
                )
            if abs(state.pop("adams_rack_input_mm") - rack) > 1e-6:
                raise RuntimeError(f"Adams rack input mismatch for {expected_id}")
            cases.append(state)
            solver_evidence.append(
                {
                    "case_id": expected_id,
                    "acf_sha256": _sha256(acf),
                    "result_sha256": _sha256(runtime / f"{stem}.res"),
                    "kinematic_log_marker": True,
                    "simulate_status": 0,
                }
            )

    evidence = {
        "contract": CONTRACT,
        "schema_version": SCHEMA_VERSION,
        "producer": "msc.adams-car.2024.1",
        "adams_version": profile.version,
        "input_manifest_sha256": hashlib.sha256(
            json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode("utf-8")
        ).hexdigest(),
        "command_file_sha256": _sha256(command_file),
        "generated_source_hashes": generated_hashes,
        "analysis_mode": "kinematic",
        "kinematic_flag": 1,
        "compliance_matrix_flag": 0,
        "compliance_objects_flag": 0,
        "force_contribution": False,
        "gravity_contribution": False,
        "contact_contribution": False,
        "runtime_directory": str(runtime),
        "state_count": len(cases),
        "cases": solver_evidence,
    }
    return cases, evidence


def compare_k_states(
    reference: list[dict[str, Any]],
    adams: list[dict[str, Any]],
) -> dict[str, Any]:
    """Compare all 90 absolute K values and reject missing or extra states/fields."""
    expected = {str(state["case_id"]): state for state in reference}
    observed = {str(state["case_id"]): state for state in adams}
    if len(expected) != 9 or len(observed) != 9 or set(expected) != set(observed):
        raise ValueError("strict K comparison requires the same nine case IDs")
    tolerances = {
        "position": Tolerance(0.1, 0.002, "mm"),
        "angle": Tolerance(0.02, 0.005, "deg"),
    }
    comparisons: list[dict[str, Any]] = []
    passed = True
    for case_id in sorted(expected):
        for kind, fields in (("position", POSITION_FIELDS), ("angle", ANGLE_FIELDS)):
            for field in fields:
                if field not in expected[case_id] or field not in observed[case_id]:
                    raise ValueError(f"strict K result is missing {case_id}.{field}")
                target = float(expected[case_id][field])
                actual = float(observed[case_id][field])
                error = abs(actual - target)
                limit = tolerances[kind].limit(target)
                item_passed = math.isfinite(error) and error <= limit
                passed &= item_passed
                comparisons.append(
                    {
                        "case_id": case_id,
                        "field": field,
                        "reference": target,
                        "adams": actual,
                        "absolute_error": error,
                        "tolerance": limit,
                        "passed": item_passed,
                    }
                )
    return {
        "passed": passed,
        "case_count": 9,
        "field_count": len(comparisons),
        "max_position_error_mm": max(
            item["absolute_error"]
            for item in comparisons
            if item["field"] in POSITION_FIELDS
        ),
        "max_angle_error_deg": max(
            item["absolute_error"]
            for item in comparisons
            if item["field"] in ANGLE_FIELDS
        ),
        "fields": comparisons,
    }


def _write_pure_k_sources(
    profile: AdamsProfile, manifest: dict[str, Any], runtime: Path
) -> tuple[Path, dict[str, str]]:
    generated = write_equivalent_sources(profile, manifest, runtime, mode="K")
    return generated.assembly, generated.hashes


def _generation_command(assembly: Path) -> str:
    commands = [
        "defaults command_file echo_commands=off",
        "variable set variable=.ACAR.variables.errorFlag integer=0",
        "acar files assembly open &",
        f' assembly_name="{assembly.as_posix()}" &',
        " error_variable=.ACAR.variables.errorFlag",
    ]
    for wheel in WHEEL_VALUES_MM:
        for rack in RACK_VALUES_MM:
            stem = _stem(wheel, rack)
            commands.extend(
                (
                    "acar analysis suspension parallel_travel submit &",
                    " assembly=.strict_suspension &",
                    ' output_prefix="strict_k" &',
                    f' output_suffix="{stem.removeprefix("strict_k_")}" &',
                    " nsteps=1 &",
                    f" bump_disp={wheel:g} &",
                    f" rebound_disp={wheel:g} &",
                    f" stat_steer_pos={rack:g} &",
                    " load_results=yes &",
                    " vertical_setup=wheel_center_height &",
                    " vertical_input=wheel_center_height &",
                    " vertical_type=relative &",
                    " steering_input=length &",
                    " log_file=yes &",
                    " analysis_mode=interactive &",
                    " create_report=no &",
                    " error_variable=.ACAR.variables.errorFlag",
                )
            )
    commands.extend(
        (
            'file text open file="strict_k_generate_status.txt" open=overwrite',
            'file text write format="error=%d" value=(eval(.ACAR.variables.errorFlag))',
            "file text close",
            "exit confirm=yes",
        )
    )
    return "\n".join(commands) + "\n"


def _parse_kinematic_result(path: Path) -> dict[str, Any]:
    root = ET.parse(path).getroot()
    components = {
        ("gel_spindle_XFORM", "X"): "left_wheel_center_x_mm",
        ("gel_spindle_XFORM", "Y"): "left_wheel_center_y_mm",
        ("gel_spindle_XFORM", "Z"): "left_wheel_center_z_mm",
        ("ger_spindle_XFORM", "X"): "right_wheel_center_x_mm",
        ("ger_spindle_XFORM", "Y"): "right_wheel_center_y_mm",
        ("ger_spindle_XFORM", "Z"): "right_wheel_center_z_mm",
        ("toe_angle", "left"): "left_toe_deg",
        ("toe_angle", "right"): "right_toe_deg",
        ("camber_angle", "left"): "left_camber_deg",
        ("camber_angle", "right"): "right_camber_deg",
        ("steering_rack_input", "rack_input"): "adams_rack_input_mm",
        ("wheel_travel", "vertical_left"): "adams_wheel_travel_left_mm",
        ("wheel_travel", "vertical_right"): "adams_wheel_travel_right_mm",
    }
    ids: dict[tuple[str, str], int] = {}
    for entity in root.findall(".//{*}StepMap/{*}Entity"):
        for component in entity.findall("{*}Component"):
            key = (str(entity.get("name")), str(component.get("name")))
            if key in components:
                ids[key] = int(str(component.get("id")))
    missing = sorted(set(components) - set(ids))
    if missing:
        raise ValueError(f"Adams K result is missing components: {missing}")
    data = [
        item
        for item in root.findall(".//{*}Data")
        if item.get("name") == "kinematic_001"
    ]
    if not data:
        raise ValueError("Adams result has no pure kinematic data")
    steps = data[-1].findall("{*}Step")
    if not steps:
        raise ValueError("Adams pure kinematic result has no steps")
    values = [float(value) for value in "".join(steps[-1].itertext()).split()]
    result: dict[str, Any] = {}
    for key, output_name in components.items():
        component_id = ids[key]
        if component_id > len(values):
            raise ValueError(f"Adams K result component map exceeds data: {key}")
        value = values[component_id - 1]
        if key[0] in {"toe_angle", "camber_angle"}:
            value = math.degrees(value)
        if output_name.endswith("_wheel_center_x_mm"):
            value = -value
        if output_name.endswith("_toe_deg"):
            value = -value
        result[output_name] = value
    return result


def _run_process(
    executable: Path,
    arguments: tuple[str, ...],
    cwd: Path,
    *,
    timeout: int,
    environment: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    env = _adams_environment(cwd)
    if environment:
        env.update(environment)
    return subprocess.run(
        [str(executable), *arguments],
        cwd=cwd,
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def _replace_exact(text: str, old: str, new: str, label: str) -> str:
    if text.count(old) != 1:
        raise ValueError(f"unexpected Adams {label} layout")
    return text.replace(old, new)


def _database(profile: AdamsProfile) -> Path:
    path = Path(profile.database_path or "")
    if not path.is_dir():
        raise FileNotFoundError("Adams database is unavailable")
    return path


def _case_id(wheel: float, rack: float) -> str:
    return f"k-w{wheel:+g}-r{rack:+g}"


def _stem(wheel: float, rack: float) -> str:
    def token(value: float) -> str:
        return (
            "m" if value < 0 else "p" if value > 0 else "z"
        ) + f"{abs(value):g}".replace(".", "d")

    return f"strict_k_w{token(wheel)}_r{token(rack)}"


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
