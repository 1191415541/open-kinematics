"""
Strict compliant Adams/Solver comparison from one canonical C model.

The stock Adams/Car double-wishbone subsystem contains springs, dampers,
stops, and steering compliance that are not represented by the v1 MBD model.
This module therefore writes a small native Adams static model from the same
canonical bodies, ideal joints, four inboard 6x6 bushings, and neutral-rack
boundary used by :mod:`suspension_mbd`.  The generated Adams/Car subsystem and
assembly remain the traceable K artefacts; this native static executable is the
strict C counterpart because its active element set is exactly common.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

from ..analysis import CModeSolver, KReferenceCache, LoadPath
from ..model import build_front_axle, side_hardpoints
from ..schema import Bushing6x6, FrontAxleModel, MassSpec, Pose, Vec3
from .adapter import SmokeResult, Tolerance
from .probe import AdamsProfile
from .strict_k import build_equivalence_manifest

CONTRACT = "strict-adams-c-v1"
SCHEMA_VERSION = 1
TRANSLATIONAL_STIFFNESS_N_PER_MM = 10_000.0
ROTATIONAL_STIFFNESS_N_MM_PER_RAD = 10_000_000.0
NATIVE_BUSHING_ELEMENT_STIFFNESS_SCALE = 0.5
INBOARD_BUSHINGS = (
    ("upper_arm", "uca_front"),
    ("upper_arm", "uca_rear"),
    ("lower_arm", "lca_front"),
    ("lower_arm", "lca_rear"),
)
LOAD_PATHS = (
    LoadPath("fx", "fx", 100.0),
    LoadPath("fy", "fy", 100.0),
    LoadPath("fz", "fz", 100.0),
    LoadPath("mx", "mx", 10_000.0),
    LoadPath("my", "my", 10_000.0),
    LoadPath("mz", "mz", 10_000.0),
)
TRANSLATION_FIELDS = tuple(
    f"{side}_wheel_center_d{axis}_mm"
    for side in ("left", "right")
    for axis in ("x", "y", "z")
)
ROTATION_FIELDS = tuple(
    f"{side}_rotation_{axis}_rad"
    for side in ("left", "right")
    for axis in ("x", "y", "z")
)
ANGLE_FIELDS = tuple(
    f"{side}_{angle}_delta_deg"
    for side in ("left", "right")
    for angle in ("toe", "camber")
)
C_FIELD_KINDS = {
    **{field: "translation" for field in TRANSLATION_FIELDS},
    **{field: "rotation" for field in ROTATION_FIELDS},
    **{field: "angle" for field in ANGLE_FIELDS},
}


@dataclass(frozen=True)
class RawAdamsModel:
    """One raw Adams static dataset and its corresponding path definition."""

    path: Path
    load_path: LoadPath
    model_sha256: str


def build_strict_c_model(profile: AdamsProfile) -> FrontAxleModel:
    """Build the physical C model shared verbatim with the raw Adams writer."""
    manifest = build_equivalence_manifest(profile)
    raw_points = manifest["physical_input"]["hardpoints_mm"]
    if not isinstance(raw_points, dict):
        raise ValueError("strict C manifest hardpoints are invalid")
    hardpoints = {
        str(name): Vec3(x=float(values[0]), y=float(values[1]), z=float(values[2]))
        for name, values in raw_points.items()
    }
    stiffness = _bushing_stiffness()
    bushings = tuple(
        Bushing6x6(
            name=f"{body}_{hardpoint}",
            body_a="chassis",
            body_b=body,
            pose_a=_pose_at(hardpoints[hardpoint]),
            pose_b=_pose_at(hardpoints[hardpoint]),
            stiffness=stiffness,
        )
        for body, hardpoint in INBOARD_BUSHINGS
    )
    return FrontAxleModel(
        name="strict_adams_c_equivalent",
        hardpoints=hardpoints,
        mass=MassSpec(sprung_mass=1.0),
        bushings=bushings,
    )


def run_suspension_mbd_strict_c(profile: AdamsProfile) -> list[dict[str, float | str]]:
    """Solve all strict C load states from the canonical physical model."""
    assembly = build_front_axle(build_strict_c_model(profile), "C")
    solver = CModeSolver()
    cache = KReferenceCache()
    states: list[dict[str, float | str]] = []
    for path in LOAD_PATHS:
        for state in solver.run_path(assembly, path, k_cache=cache):
            states.append(
                {
                    "case_id": state.case_id,
                    "path": path.name,
                    "level": state.level,
                    "left_load_fx_n": state.load_left.fx,
                    "left_load_fy_n": state.load_left.fy,
                    "left_load_fz_n": state.load_left.fz,
                    "left_load_mx_n_mm": state.load_left.mx,
                    "left_load_my_n_mm": state.load_left.my,
                    "left_load_mz_n_mm": state.load_left.mz,
                    "left_wheel_center_dx_mm": state.deformation_left.fx,
                    "left_wheel_center_dy_mm": state.deformation_left.fy,
                    "left_wheel_center_dz_mm": state.deformation_left.fz,
                    "left_rotation_x_rad": state.deformation_left.mx,
                    "left_rotation_y_rad": state.deformation_left.my,
                    "left_rotation_z_rad": state.deformation_left.mz,
                    "right_wheel_center_dx_mm": state.deformation_right.fx,
                    "right_wheel_center_dy_mm": state.deformation_right.fy,
                    "right_wheel_center_dz_mm": state.deformation_right.fz,
                    "right_rotation_x_rad": state.deformation_right.mx,
                    "right_rotation_y_rad": state.deformation_right.my,
                    "right_rotation_z_rad": state.deformation_right.mz,
                    "left_toe_delta_deg": state.c_minus_k["left_toe_deg"],
                    "left_camber_delta_deg": state.c_minus_k["left_camber_deg"],
                    "right_toe_delta_deg": state.c_minus_k["right_toe_deg"],
                    "right_camber_delta_deg": state.c_minus_k["right_camber_deg"],
                }
            )
    return states


def write_raw_adams_models(
    model: FrontAxleModel, runtime: str | Path
) -> tuple[RawAdamsModel, ...]:
    """Write one ten-step raw Adams static dataset for each strict C axis."""
    destination = Path(runtime)
    destination.mkdir(parents=True, exist_ok=True)
    generated: list[RawAdamsModel] = []
    for path in LOAD_PATHS:
        file_path = destination / f"strict_c_{path.name}.adm"
        text = _raw_adams_text(model, path)
        file_path.write_text(text, encoding="ascii")
        generated.append(RawAdamsModel(file_path, path, _sha256(file_path)))
    return tuple(generated)


def build_strict_c_manifest(profile: AdamsProfile) -> dict[str, Any]:
    """Freeze the common physical entity set used by both strict-C solvers."""
    k_manifest = build_equivalence_manifest(profile)
    k_input = k_manifest["physical_input"]
    if not isinstance(k_input, dict):
        raise ValueError("strict C source manifest is invalid")
    hardpoints = k_input.get("hardpoints_mm")
    coordinates = k_input.get("coordinates")
    if not isinstance(hardpoints, dict) or not isinstance(coordinates, dict):
        raise ValueError("strict C source hardpoints are invalid")
    physical_input = {
        "coordinates": coordinates,
        "hardpoints_mm": hardpoints,
        "bodies": [
            "upper_arm_left",
            "lower_arm_left",
            "upright_left",
            "tie_rod_left",
            "upper_arm_right",
            "lower_arm_right",
            "upright_right",
            "tie_rod_right",
            "rack",
        ],
        "ideal_joints": {
            "outboard_ball_joints_per_side": 2,
            "tie_rod_ball_joints_per_side": 2,
            "rack_prismatic_axis": [0.0, 1.0, 0.0],
        },
        "bushings": {
            "per_side": [
                {
                    "body": body,
                    "hardpoint": hardpoint,
                    "stiffness": [list(row) for row in _bushing_stiffness()],
                }
                for body, hardpoint in INBOARD_BUSHINGS
            ],
            "total_count": 8,
            "preload": [0.0] * 6,
            "native_adams_representation": {
                "parallel_elements_per_physical_bushing": 2,
                "native_element_count": 16,
                "per_element_stiffness_scale": NATIVE_BUSHING_ELEMENT_STIFFNESS_SCALE,
                "layout": "reversed_i_j_pairs",
            },
        },
        "wheel_center_wrench": {
            "side": "left",
            "axes": "global",
            "force_unit": "N",
            "moment_unit": "N*mm",
            "load_paths": [
                {
                    "name": path.name,
                    "axis": path.axis,
                    "maximum": path.maximum,
                    "levels": path.levels,
                }
                for path in LOAD_PATHS
            ],
        },
        "boundaries": {
            "analysis_mode": "quasi_static",
            "rack_neutral_drive_mm": 0.0,
            "gravity_contributes": False,
            "contact_contributes": False,
            "springs_contribute": False,
            "dampers_contribute": False,
            "stops_contribute": False,
            "template_compliance_objects_contribute": False,
        },
    }
    canonical = json.dumps(physical_input, sort_keys=True, separators=(",", ":"))
    snapshot = hashlib.sha256(canonical.encode("utf-8")).hexdigest()
    return {
        "contract": CONTRACT,
        "schema_version": SCHEMA_VERSION,
        "physical_input": physical_input,
        "adams_snapshot_sha256": snapshot,
        "suspension_mbd_snapshot_sha256": snapshot,
        "source": k_manifest["source"],
    }


def validate_strict_c(
    profile: AdamsProfile,
    *,
    evidence_dir: str | Path | None = None,
) -> SmokeResult:
    """Run the common compliant model in Adams and suspension_mbd."""
    destination = (
        Path(evidence_dir)
        if evidence_dir is not None
        else Path(tempfile.mkdtemp(prefix="suspension_mbd_strict_c_evidence_"))
    )
    destination.mkdir(parents=True, exist_ok=True)
    if not profile.available:
        return SmokeResult(False, profile.message, profile)

    try:
        manifest = build_strict_c_manifest(profile)
        manifest_path = destination / "equivalence_manifest.json"
        _write_json(manifest_path, manifest)
        model = build_strict_c_model(profile)
        runtime = Path(tempfile.mkdtemp(prefix="suspension_mbd_strict_c_runtime_"))
        adams_states, execution = run_adams_strict_c(profile, model, runtime)
        reference_states = run_suspension_mbd_strict_c(profile)
        comparison = compare_c_states(reference_states, adams_states)
        _write_json(destination / "adams_c_results.json", {"states": adams_states})
        _write_json(
            destination / "suspension_mbd_c_results.json",
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
            f"Strict Adams C validation failed: {exc}",
            profile,
            str(report_path),
            report,
        )

    passed = bool(report["passed"])
    message = (
        f"Strict Adams C validation passed; report: {report_path}"
        if passed
        else f"Strict Adams C validation exceeded tolerance; report: {report_path}"
    )
    return SmokeResult(passed, message, profile, str(report_path), report)


def run_adams_strict_c(
    profile: AdamsProfile,
    model: FrontAxleModel,
    runtime: Path,
) -> tuple[list[dict[str, float | str]], dict[str, Any]]:
    """Execute the six raw native-Adams static paths and parse every step."""
    runtime.mkdir(parents=True, exist_ok=True)
    executable = Path(profile.executable or "")
    if not executable.is_file():
        raise FileNotFoundError("Adams launcher is unavailable")
    raw_models = write_raw_adams_models(model, runtime)
    states: list[dict[str, float | str]] = []
    cases: list[dict[str, Any]] = []
    for raw_model in raw_models:
        stem = raw_model.path.stem
        command = raw_model.path.with_suffix(".acf")
        command.write_text(
            _raw_command_text(stem, raw_model.load_path), encoding="ascii"
        )
        completed = _run_process(
            executable,
            ("ru-standard", stem),
            runtime,
            timeout=300,
        )
        stdout_path = runtime / f"{stem}.stdout.txt"
        stdout_path.write_text(
            completed.stdout or "", encoding="utf-8", errors="replace"
        )
        stderr_path = runtime / f"{stem}.stderr.txt"
        stderr_path.write_text(
            completed.stderr or "", encoding="utf-8", errors="replace"
        )
        message_path = runtime / f"{stem}.msg"
        message = (
            message_path.read_text(encoding="utf-8", errors="replace")
            if message_path.is_file()
            else ""
        )
        solver_log = f"{completed.stdout}\n{completed.stderr}\n{message}"
        result_path = raw_model.path.with_suffix(".res")
        if (
            completed.returncode != 0
            or "Performing Quasi-Static Simulation" not in solver_log
            or "End Simulation" not in solver_log
            or not result_path.is_file()
        ):
            raise RuntimeError(f"Adams strict-C solve failed for {stem}")
        parsed = _parse_raw_adams_result(result_path, raw_model.load_path)
        states.extend(parsed)
        cases.append(
            {
                "path": raw_model.load_path.name,
                "acf_sha256": _sha256(command),
                "model_sha256": raw_model.model_sha256,
                "result_sha256": _sha256(result_path),
                "simulate_status": 0,
                "state_count": len(parsed),
            }
        )
    expected_count = sum(path.levels for path in LOAD_PATHS)
    if len(states) != expected_count:
        raise RuntimeError("Adams strict-C runner returned an incomplete state grid")
    evidence = {
        "contract": CONTRACT,
        "schema_version": SCHEMA_VERSION,
        "producer": "msc.adams-solver.2024.1",
        "adams_version": profile.version,
        "analysis_mode": "quasi_static",
        "gravity_contribution": False,
        "contact_contribution": False,
        "active_bushing_count": 8,
        "native_bushing_element_count": 16,
        "native_bushing_element_stiffness_scale": NATIVE_BUSHING_ELEMENT_STIFFNESS_SCALE,
        "neutral_rack_drive_mm": 0.0,
        "runtime_directory": str(runtime),
        "state_count": len(states),
        "cases": cases,
    }
    return states, evidence


def compare_c_states(
    reference: list[dict[str, float | str]],
    adams: list[dict[str, float | str]],
) -> dict[str, Any]:
    """Compare every strict-C state and reject missing paths, states or fields."""
    expected = {str(state["case_id"]): state for state in reference}
    observed = {str(state["case_id"]): state for state in adams}
    expected_count = sum(path.levels for path in LOAD_PATHS)
    if (
        len(expected) != expected_count
        or len(observed) != expected_count
        or set(expected) != set(observed)
    ):
        raise ValueError("strict C comparison requires the complete common state grid")
    tolerances = {
        "translation": Tolerance(1e-6, 1e-4, "mm"),
        "rotation": Tolerance(1e-8, 1e-4, "rad"),
        "angle": Tolerance(1e-6, 1e-4, "deg"),
    }
    comparisons: list[dict[str, Any]] = []
    passed = True
    for case_id in sorted(expected):
        target_state = expected[case_id]
        actual_state = observed[case_id]
        for field, kind in C_FIELD_KINDS.items():
            if field not in target_state or field not in actual_state:
                raise ValueError(f"strict C result is missing {case_id}.{field}")
            target = float(target_state[field])
            actual = float(actual_state[field])
            error = abs(actual - target)
            tolerance = tolerances[kind].limit(target)
            item_passed = math.isfinite(error) and error <= tolerance
            passed &= item_passed
            comparisons.append(
                {
                    "case_id": case_id,
                    "field": field,
                    "reference": target,
                    "adams": actual,
                    "absolute_error": error,
                    "tolerance": tolerance,
                    "passed": item_passed,
                }
            )
    return {
        "passed": passed,
        "case_count": expected_count,
        "field_count": len(comparisons),
        "max_translation_error_mm": max(
            item["absolute_error"]
            for item in comparisons
            if item["field"] in TRANSLATION_FIELDS
        ),
        "max_rotation_error_rad": max(
            item["absolute_error"]
            for item in comparisons
            if item["field"] in ROTATION_FIELDS
        ),
        "max_angle_error_deg": max(
            item["absolute_error"]
            for item in comparisons
            if item["field"] in ANGLE_FIELDS
        ),
        "fields": comparisons,
    }


def _raw_command_text(stem: str, load_path: LoadPath) -> str:
    """Render the C++ Solver command file; its leading blank is significant."""
    return (
        f"\nfile/model={stem}\n"
        "simulate/static\n"
        f"simulate/statics, end=1, steps={load_path.levels - 1}\n"
        f"simulate/statics, end=2, steps={load_path.levels - 1}\n"
        "stop\n"
    )


def _parse_raw_adams_result(
    path: Path, load_path: LoadPath
) -> list[dict[str, float | str]]:
    """Parse one native-Adams static path into the common strict-C state schema."""
    root = ET.parse(path).getroot()
    component_ids = {
        "time": _component_ids(root, "time", ("TIME",)),
        "left_response": _component_ids(
            root,
            "strict_c_l_wheel_response",
            ("x", "y", "z", "lateral_x", "lateral_y", "lateral_z"),
        ),
        "right_response": _component_ids(
            root,
            "strict_c_r_wheel_response",
            ("x", "y", "z", "lateral_x", "lateral_y", "lateral_z"),
        ),
        "left_longitudinal": _component_ids(
            root, "strict_c_l_wheel_longitudinal", ("x", "y", "z")
        ),
        "right_longitudinal": _component_ids(
            root, "strict_c_r_wheel_longitudinal", ("x", "y", "z")
        ),
    }
    data = [
        item
        for item in root.findall(".//{*}Data")
        if item.get("name") == "quasiStatic_001"
    ]
    if not data:
        raise ValueError("Adams strict-C result has no quasi-static data")
    all_steps = data[-1].findall("{*}Step")
    expected_step_count = 2 * load_path.levels - 1
    if len(all_steps) != expected_step_count:
        raise ValueError(
            "Adams strict-C result has "
            f"{len(all_steps)} steps, expected {expected_step_count}"
        )
    steps = all_steps[-load_path.levels :]
    rows = [
        _result_row("".join(step.itertext()).split(), component_ids, path)
        for step in steps
    ]
    zero_index = load_path.levels // 2
    reference_left = _raw_wheel_sample(rows[zero_index], component_ids, "left")
    reference_right = _raw_wheel_sample(rows[zero_index], component_ids, "right")
    values = load_path.values()
    states: list[dict[str, float | str]] = []
    for index, (row, level) in enumerate(zip(rows, values, strict=True)):
        expected_time = 1.0 + index / (load_path.levels - 1)
        time = _component_value(row, component_ids["time"]["TIME"], "time")
        if not math.isclose(time, expected_time, abs_tol=1e-10):
            raise ValueError(
                f"Adams strict-C time grid disagrees at {load_path.name}:{index}"
            )
        left = _raw_wheel_sample(row, component_ids, "left")
        right = _raw_wheel_sample(row, component_ids, "right")
        states.append(
            _raw_state(
                load_path,
                index,
                level,
                left,
                right,
                reference_left,
                reference_right,
            )
        )
    return states


def _component_ids(
    root: ET.Element, entity_name: str, components: tuple[str, ...]
) -> dict[str, int]:
    for entity in root.findall(".//{*}StepMap/{*}Entity"):
        if entity.get("name") != entity_name:
            continue
        ids = {
            str(component.get("name")): int(str(component.get("id")))
            for component in entity.findall("{*}Component")
        }
        missing = sorted(set(components) - set(ids))
        if missing:
            raise ValueError(
                f"Adams strict-C result is missing {entity_name} components: {missing}"
            )
        return {name: ids[name] for name in components}
    raise ValueError(f"Adams strict-C result is missing entity {entity_name}")


def _result_row(
    tokens: list[str], component_ids: dict[str, dict[str, int]], path: Path
) -> list[float]:
    values = [float(value) for value in tokens]
    required_id = max(
        identifier
        for components in component_ids.values()
        for identifier in components.values()
    )
    if len(values) < required_id:
        raise ValueError(f"Adams strict-C result row is incomplete in {path.name}")
    return values


def _component_value(values: list[float], identifier: int, label: str) -> float:
    if identifier < 1 or identifier > len(values):
        raise ValueError(f"Adams strict-C component index is invalid for {label}")
    return values[identifier - 1]


def _raw_wheel_sample(
    values: list[float],
    component_ids: dict[str, dict[str, int]],
    side: str,
) -> tuple[np.ndarray, np.ndarray]:
    response = component_ids[f"{side}_response"]
    longitudinal = component_ids[f"{side}_longitudinal"]
    position = np.array(
        [
            _component_value(values, response[axis], f"{side}.position.{axis}")
            for axis in ("x", "y", "z")
        ]
    )
    lateral = np.array(
        [
            _component_value(values, response[f"lateral_{axis}"], f"{side}.lateral")
            for axis in ("x", "y", "z")
        ]
    )
    longitudinal_axis = np.array(
        [
            _component_value(values, longitudinal[axis], f"{side}.longitudinal")
            for axis in ("x", "y", "z")
        ]
    )
    return position, _rotation_from_axes(longitudinal_axis, lateral)


def _rotation_from_axes(longitudinal: np.ndarray, lateral: np.ndarray) -> np.ndarray:
    x_axis = np.asarray(longitudinal, dtype=float)
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm <= 1e-12:
        raise ValueError("Adams strict-C longitudinal axis is degenerate")
    x_axis /= x_norm
    y_axis = np.asarray(lateral, dtype=float)
    y_axis -= x_axis * float(np.dot(x_axis, y_axis))
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm <= 1e-12:
        raise ValueError("Adams strict-C lateral axis is collinear")
    y_axis /= y_norm
    z_axis = np.cross(x_axis, y_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def _raw_state(
    load_path: LoadPath,
    index: int,
    level: float,
    left: tuple[np.ndarray, np.ndarray],
    right: tuple[np.ndarray, np.ndarray],
    reference_left: tuple[np.ndarray, np.ndarray],
    reference_right: tuple[np.ndarray, np.ndarray],
) -> dict[str, float | str]:
    left_position, left_rotation = left
    right_position, right_rotation = right
    reference_left_position, reference_left_rotation = reference_left
    reference_right_position, reference_right_rotation = reference_right
    left_response = np.concatenate(
        (
            left_position - reference_left_position,
            _rotation_response(reference_left_rotation, left_rotation),
        )
    )
    right_response = np.concatenate(
        (
            right_position - reference_right_position,
            _rotation_response(reference_right_rotation, right_rotation),
        )
    )
    left_toe, left_camber = _alignment_delta(
        reference_left_rotation, left_rotation, "left"
    )
    right_toe, right_camber = _alignment_delta(
        reference_right_rotation, right_rotation, "right"
    )
    load = np.zeros(6)
    load[("fx", "fy", "fz", "mx", "my", "mz").index(load_path.axis)] = level
    return {
        "case_id": f"c-{load_path.name}-{index:02d}",
        "path": load_path.name,
        "level": level,
        "left_load_fx_n": float(load[0]),
        "left_load_fy_n": float(load[1]),
        "left_load_fz_n": float(load[2]),
        "left_load_mx_n_mm": float(load[3]),
        "left_load_my_n_mm": float(load[4]),
        "left_load_mz_n_mm": float(load[5]),
        "left_wheel_center_dx_mm": float(left_response[0]),
        "left_wheel_center_dy_mm": float(left_response[1]),
        "left_wheel_center_dz_mm": float(left_response[2]),
        "left_rotation_x_rad": float(left_response[3]),
        "left_rotation_y_rad": float(left_response[4]),
        "left_rotation_z_rad": float(left_response[5]),
        "right_wheel_center_dx_mm": float(right_response[0]),
        "right_wheel_center_dy_mm": float(right_response[1]),
        "right_wheel_center_dz_mm": float(right_response[2]),
        "right_rotation_x_rad": float(right_response[3]),
        "right_rotation_y_rad": float(right_response[4]),
        "right_rotation_z_rad": float(right_response[5]),
        "left_toe_delta_deg": left_toe,
        "left_camber_delta_deg": left_camber,
        "right_toe_delta_deg": right_toe,
        "right_camber_delta_deg": right_camber,
    }


def _rotation_response(reference: np.ndarray, current: np.ndarray) -> np.ndarray:
    """Return the global rotation vector used by CModeSolver._wheel_response."""
    relative = reference.T @ current
    return reference @ _rotation_vector_from_matrix(relative)


def _rotation_vector_from_matrix(rotation: np.ndarray) -> np.ndarray:
    cosine = float(np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    vector = np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    if angle <= 1e-7:
        return 0.5 * vector
    sine = 0.5 * float(np.linalg.norm(vector))
    if sine <= 1e-12:
        raise ValueError("Adams strict-C rotation is singular")
    return vector * (angle / (2.0 * sine))


def _alignment_delta(
    reference: np.ndarray, current: np.ndarray, side: str
) -> tuple[float, float]:
    toe, camber = _alignment_angles(current, side)
    reference_toe, reference_camber = _alignment_angles(reference, side)
    return toe - reference_toe, camber - reference_camber


def _alignment_angles(rotation: np.ndarray, side: str) -> tuple[float, float]:
    outward = -1.0 if side == "left" else 1.0
    lateral = rotation[:, 1]
    toe = -outward * math.degrees(math.atan2(float(lateral[0]), float(lateral[1])))
    camber = -outward * math.degrees(math.atan2(float(lateral[2]), float(lateral[1])))
    return toe, camber


def _run_process(
    executable: Path,
    arguments: tuple[str, ...],
    cwd: Path,
    *,
    timeout: int,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(executable), *arguments],
        cwd=cwd,
        env=os.environ.copy(),
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )


def _write_json(path: Path, payload: object) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


def _bushing_stiffness() -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(
            TRANSLATIONAL_STIFFNESS_N_PER_MM
            if row == column and row < 3
            else ROTATIONAL_STIFFNESS_N_MM_PER_RAD
            if row == column
            else 0.0
            for column in range(6)
        )
        for row in range(6)
    )


def _pose_at(point: Vec3) -> Pose:
    return Pose(translation=point)


def _raw_adams_text(model: FrontAxleModel, load_path: LoadPath) -> str:
    """Render a native Adams model with only common strict-C entities."""
    points = {
        side: {
            name: tuple(float(value) for value in point.as_tuple())
            for name, point in side_hardpoints(model.hardpoints, side).items()
        }
        for side in ("L", "R")
    }
    lines = [
        "ADAMS/View model name: strict_c",
        "! Strict C native model; all length values are millimetres.",
        "UNITS/FORCE = NEWTON, MASS = KILOGRAM, LENGTH = MILLIMETER, TIME = SECOND",
        "! Ground",
        "PART/1, GROUND",
        "MARKER/1, PART = 1",
    ]
    body_ids = {
        "upper_arm_L": 2,
        "lower_arm_L": 3,
        "upright_L": 4,
        "tie_rod_L": 5,
        "upper_arm_R": 6,
        "lower_arm_R": 7,
        "upright_R": 8,
        "tie_rod_R": 9,
        "rack": 10,
    }
    for body, identifier in body_ids.items():
        lines.extend(
            (
                f"! adams_view_name='strict_c_{body}'",
                f"PART/{identifier}, QG = 0, 0, 0, MASS = 1, CM = {identifier}, IP = 1, 1, 1",
                f"MARKER/{identifier}, PART = {identifier}",
            )
        )

    marker = _MarkerWriter(lines, start=100)
    for side in ("L", "R"):
        for body, hardpoint in INBOARD_BUSHINGS:
            point = points[side][hardpoint]
            marker.add(f"ground_{side}_{hardpoint}", 1, point)
            marker.add(f"{body}_{side}_{hardpoint}", body_ids[f"{body}_{side}"], point)
        for body, hardpoint in (("upper_arm", "uca_outer"), ("lower_arm", "lca_outer")):
            point = points[side][hardpoint]
            marker.add(f"{body}_{side}_outer", body_ids[f"{body}_{side}"], point)
            marker.add(
                f"upright_{side}_{body}_outer", body_ids[f"upright_{side}"], point
            )
        for body, hardpoint in (("rack", "tierod_inner"), ("tie_rod", "tierod_inner")):
            marker.add(
                f"{body}_{side}_inner",
                body_ids[body if body == "rack" else f"{body}_{side}"],
                points[side][hardpoint],
            )
        point = points[side]["tierod_outer"]
        marker.add(f"tie_rod_{side}_outer", body_ids[f"tie_rod_{side}"], point)
        marker.add(f"upright_{side}_tie_outer", body_ids[f"upright_{side}"], point)
        center = points[side]["wheel_center"]
        marker.add(f"upright_{side}_wheel_center", body_ids[f"upright_{side}"], center)
        marker.add(
            f"upright_{side}_lateral_tip",
            body_ids[f"upright_{side}"],
            (center[0], center[1] + 1.0, center[2]),
        )
        marker.add(
            f"upright_{side}_longitudinal_tip",
            body_ids[f"upright_{side}"],
            (center[0] + 1.0, center[1], center[2]),
        )

    rack_center = points["L"]["rack_center"]
    guide_orientation = "90D, 90D, 270D"
    marker.add("rack_guide", body_ids["rack"], rack_center, guide_orientation)
    marker.add("ground_rack_guide", 1, rack_center, guide_orientation)
    marker.add("load_left_force", 1, floating=True)
    marker.add("load_left_moment", 1, floating=True)

    joint_id = 1
    for side in ("L", "R"):
        for body in ("upper_arm", "lower_arm"):
            lines.append(
                f"JOINT/{joint_id}, SPHERICAL, I = {marker[f'{body}_{side}_outer']}, "
                f"J = {marker[f'upright_{side}_{body}_outer']}"
            )
            joint_id += 1
        lines.append(
            f"JOINT/{joint_id}, SPHERICAL, I = {marker[f'rack_{side}_inner']}, "
            f"J = {marker[f'tie_rod_{side}_inner']}"
        )
        joint_id += 1
        lines.append(
            f"JOINT/{joint_id}, SPHERICAL, I = {marker[f'tie_rod_{side}_outer']}, "
            f"J = {marker[f'upright_{side}_tie_outer']}"
        )
        joint_id += 1
    lines.append(
        f"JOINT/{joint_id}, TRANSLATIONAL, I = {marker['rack_guide']}, "
        f"J = {marker['ground_rack_guide']}"
    )
    lines.extend(
        (
            "! Neutral steering-rack input: matches CModeSolver's CoordinateDrive.",
            "MOTION/1",
            ", TRANSLATIONAL",
            f", JOINT = {joint_id}",
            ", FUNCTION = 0",
        )
    )

    bushing_id = 1
    translational_rate = (
        TRANSLATIONAL_STIFFNESS_N_PER_MM * NATIVE_BUSHING_ELEMENT_STIFFNESS_SCALE
    )
    rotational_rate = (
        ROTATIONAL_STIFFNESS_N_MM_PER_RAD * NATIVE_BUSHING_ELEMENT_STIFFNESS_SCALE
    )
    for side in ("L", "R"):
        for body, hardpoint in INBOARD_BUSHINGS:
            ground_marker = marker[f"ground_{side}_{hardpoint}"]
            body_marker = marker[f"{body}_{side}_{hardpoint}"]
            # Adams BUSHING uses the moving J frame. Reversed half-rate pairs
            # reproduce the symmetric fixed-frame 6x6 element used by MBD.
            for direction, (i_marker, j_marker) in enumerate(
                ((ground_marker, body_marker), (body_marker, ground_marker))
            ):
                lines.extend(
                    (
                        f"! adams_view_name='strict_c_{side}_{hardpoint}_bushing_{direction}'",
                        f"BUSHING/{bushing_id}, I = {i_marker}, J = {j_marker}, "
                        f"K = {translational_rate:.0f}, {translational_rate:.0f}, {translational_rate:.0f}",
                        f", KT = {rotational_rate:.0f}, {rotational_rate:.0f}, {rotational_rate:.0f}",
                    )
                )
                bushing_id += 1

    force, moment = _load_functions(load_path)
    lines.extend(
        (
            "! Wheel-center wrench on the left upright in global axes.",
            f"VFORCE/1, I = {marker['upright_L_wheel_center']}, "
            f"JFLOAT = {marker['load_left_force']}, RM = 1, "
            f"FX = {force[0]}\\ FY = {force[1]}\\ FZ = {force[2]}",
            f"VTORQUE/1, I = {marker['upright_L_wheel_center']}, "
            f"JFLOAT = {marker['load_left_moment']}, RM = 1, "
            f"TX = {moment[0]}\\ TY = {moment[1]}\\ TZ = {moment[2]}",
        )
    )
    _append_wheel_requests(lines, marker)
    lines.extend(
        (
            "ACCGRAV/KGRAV = 0",
            "EQUILIBRIUM/",
            "OUTPUT/REQSAVE",
            "RESULTS/FORMATTED, XRF",
            "END",
        )
    )
    return "\n".join(lines) + "\n"


def _load_functions(
    load_path: LoadPath,
) -> tuple[tuple[str, str, str], tuple[str, str, str]]:
    maximum = load_path.maximum
    ramp = (
        f"IF(TIME-1:-{maximum:g}*TIME,-{maximum:g},"
        f"{2.0 * maximum:g}*TIME-{3.0 * maximum:g})"
    )
    zero = "0"
    forces = [zero, zero, zero]
    moments = [zero, zero, zero]
    index = ("fx", "fy", "fz", "mx", "my", "mz").index(load_path.axis)
    (forces if index < 3 else moments)[index % 3] = ramp
    return (forces[0], forces[1], forces[2]), (moments[0], moments[1], moments[2])


def _append_wheel_requests(lines: list[str], marker: "_MarkerWriter") -> None:
    for identifier, side in enumerate(("L", "R"), start=100):
        wheel = marker[f"upright_{side}_wheel_center"]
        lateral = marker[f"upright_{side}_lateral_tip"]
        longitudinal = marker[f"upright_{side}_longitudinal_tip"]
        name = f"strict_c_{side.lower()}_wheel_response"
        lines.extend(
            (
                f"! adams_view_name='{name}'",
                f"REQUEST/{identifier}",
                f", TITLE = {name}",
                ', CUNITS = "no_units", "length", "length", "length", "length", "length"',
                ', "length", "no_units"',
                ', CNAMES = "", "x", "y", "z", "lateral_x", "lateral_y", "lateral_z", ""',
                f", RESULTS_NAME = {name}",
                f", F2 = DX({wheel},1)\\",
                f", F3 = DY({wheel},1)\\",
                f", F4 = DZ({wheel},1)\\",
                f", F5 = DX({lateral},{wheel},1)\\",
                f", F6 = DY({lateral},{wheel},1)\\",
                f", F7 = DZ({lateral},{wheel},1)",
            )
        )
        axis_name = f"strict_c_{side.lower()}_wheel_longitudinal"
        lines.extend(
            (
                f"! adams_view_name='{axis_name}'",
                f"REQUEST/{identifier + 2}",
                f", TITLE = {axis_name}",
                ', CUNITS = "no_units", "length", "length", "length", "no_units", "no_units"',
                ', "no_units", "no_units"',
                ', CNAMES = "", "x", "y", "z", "", "", "", ""',
                f", RESULTS_NAME = {axis_name}",
                f", F2 = DX({longitudinal},{wheel},1)\\",
                f", F3 = DY({longitudinal},{wheel},1)\\",
                f", F4 = DZ({longitudinal},{wheel},1)",
            )
        )


class _MarkerWriter:
    """Allocate stable marker ids while writing native Adams records."""

    def __init__(self, lines: list[str], *, start: int) -> None:
        self.lines = lines
        self.next = start
        self.ids: dict[str, int] = {}

    def __getitem__(self, key: str) -> int:
        return self.ids[key]

    def add(
        self,
        key: str,
        part: int,
        point: Iterable[float] = (0.0, 0.0, 0.0),
        orientation: str | None = None,
        *,
        floating: bool = False,
    ) -> int:
        if key in self.ids:
            raise ValueError(f"duplicate raw Adams marker {key}")
        identifier = self.next
        self.next += 1
        self.ids[key] = identifier
        values = tuple(float(value) for value in point)
        if len(values) != 3:
            raise ValueError("raw Adams marker needs three coordinates")
        record = f"MARKER/{identifier}, PART = {part}"
        if floating:
            record += ", FLOATING"
        elif values != (0.0, 0.0, 0.0):
            record += f", QP = {values[0]:.12g}, {values[1]:.12g}, {values[2]:.12g}"
        if orientation is not None:
            record += f", REULER = {orientation}"
        self.lines.append(record)
        return identifier


def _sha256(path: Path) -> str:
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()
