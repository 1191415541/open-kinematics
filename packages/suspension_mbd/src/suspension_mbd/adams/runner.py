"""Built-in non-proprietary Adams/Car batch runner and result parser."""

from __future__ import annotations

import json
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

from .probe import AdamsProfile


def run_default_adams(
    profile: AdamsProfile, request_path: Path, output_dir: Path
) -> Path:
    """Run the installed Adams/Car demo K/C cases and export numeric JSON."""
    del request_path
    if not profile.executable:
        raise ValueError("Adams executable is unavailable")
    command_file = output_dir / "suspension_mbd_adams_validation.cmd"
    command_file.write_text(_COMMAND_FILE, encoding="ascii")
    command_line = subprocess.list2cmdline(
        [profile.executable, "acar", "ru-acar", "b", str(command_file)]
    )
    completed = subprocess.run(
        ["cmd.exe", "/d", "/s", "/c", command_line],
        cwd=output_dir,
        capture_output=True,
        text=True,
        timeout=600,
        check=False,
    )
    (output_dir / "adams_batch.stdout.txt").write_text(
        completed.stdout or "", encoding="utf-8", errors="replace"
    )
    (output_dir / "adams_batch.stderr.txt").write_text(
        completed.stderr or "", encoding="utf-8", errors="replace"
    )
    status = output_dir / "suspension_mbd_status.txt"
    if completed.returncode != 0 or not status.is_file() or status.read_text().strip() != "error=0":
        raise RuntimeError(f"Adams batch analysis failed with code {completed.returncode}")

    parallel = (output_dir / "suspension_mbd_validation_parallel.rpt").read_text(
        encoding="utf-8", errors="replace"
    )
    compliance = (output_dir / "suspension_mbd_validation_com.rpt").read_text(
        encoding="utf-8", errors="replace"
    )
    static_result = output_dir / "suspension_mbd_validation_static_load.res"
    left_static = _result_component(static_result, "left_tire_forces", "normal")
    right_static = _result_component(static_result, "right_tire_forces", "normal")

    left_toe = _maximum_minimum(parallel, "Left Toe Angle")
    right_toe = _maximum_minimum(parallel, "Right Toe Angle")
    left_camber = _maximum_minimum(parallel, "Left Camber Angle")
    steer_left, steer_right = _pair(compliance, "Lateral compliance steer")
    camber_left, camber_right = _pair(compliance, "Lateral camber compliance")
    payload = {
        "groups": {
            "K_geometry": {
                "left_toe_change_deg": abs(left_toe[0] - left_toe[1]),
                "right_toe_change_deg": abs(right_toe[0] - right_toe[1]),
                "left_camber_change_deg": abs(left_camber[0] - left_camber[1]),
            },
            "C_compliance": {
                "converging_lateral_steer_symmetry_deg_per_kn": steer_left - steer_right,
                "converging_lateral_camber_symmetry_deg_per_kn": camber_left - camber_right,
            },
            "static_load": {
                "left_wheel_force_n": left_static,
                "right_wheel_force_n": right_static,
            },
        }
    }
    output = output_dir / "adams_results.json"
    output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    return output


def _maximum_minimum(report: str, label: str) -> tuple[float, float]:
    maximum = _scalar(report, rf"Maximum\s+{re.escape(label)}")
    minimum = _scalar(report, rf"Minimum\s+{re.escape(label)}")
    return maximum, minimum


def _scalar(report: str, label_pattern: str) -> float:
    match = re.search(rf"{label_pattern}\s*=\s*([-+0-9.Ee]+)", report, re.IGNORECASE)
    if not match:
        raise ValueError(f"Adams report field was not found: {label_pattern}")
    return float(match.group(1))


def _pair(report: str, label: str) -> tuple[float, float]:
    match = re.search(
        rf"^{re.escape(label)}\s*=\s*([-+0-9.Ee]+)\s+([-+0-9.Ee]+)",
        report,
        re.IGNORECASE | re.MULTILINE,
    )
    if not match:
        raise ValueError(f"Adams report pair was not found: {label}")
    return float(match.group(1)), float(match.group(2))


def _result_component(path: Path, entity_name: str, component_name: str) -> float:
    root = ET.parse(path).getroot()
    component_id: int | None = None
    for entity in root.findall(".//{*}StepMap/{*}Entity"):
        if entity.get("name") != entity_name:
            continue
        for component in entity.findall("{*}Component"):
            if component.get("name") == component_name:
                component_id = int(str(component.get("id")))
                break
    if component_id is None:
        raise ValueError(f"Adams result channel was not found: {entity_name}.{component_name}")
    data = [item for item in root.findall(".//{*}Data") if item.get("name") == "quasiStatic_001"]
    if not data:
        raise ValueError("Adams static result has no quasiStatic_001 data")
    steps = data[-1].findall("{*}Step")
    if not steps:
        raise ValueError("Adams static result has no steps")
    values = [float(value) for value in "".join(steps[-1].itertext()).split()]
    if component_id > len(values):
        raise ValueError("Adams static result channel map exceeds step data")
    return values[component_id - 1]


_COMMAND_FILE = """defaults command_file echo_commands=off
variable set variable=.ACAR.variables.errorFlag integer=0
acar files assembly open &
 assembly_name="<acar_concept>/assemblies.tbl/Demo_Vehicle_Variants.asy" &
 variant=suspfront &
 error_variable=.ACAR.variables.errorFlag
acar analysis suspension parallel_travel submit &
 assembly=.Demo_Vehicle_Variants &
 output_prefix="suspension_mbd_validation" &
 output_suffix="parallel" &
 nsteps=4 &
 bump_disp=10 &
 rebound_disp=-10 &
 stat_steer_pos=0 &
 load_results=yes &
 vertical_setup=wheel_center_height &
 vertical_input=wheel_center_height &
 vertical_type=relative &
 steering_input=length &
 log_file=yes &
 analysis_mode=interactive &
 create_report=yes &
 error_variable=.ACAR.variables.errorFlag
acar analysis suspension compliance submit &
 assembly=.Demo_Vehicle_Variants &
 output_prefix="suspension_mbd_validation" &
 output_suffix="com" &
 nsteps=34 &
 load_results=yes &
 vertical_input=wheel_center_height &
 wheel_fixed_height=0 &
 fore_force_wc=500 &
 aft_force_wc=500 &
 fore_force_cp=500 &
 aft_force_cp=500 &
 lat_force_cp=500 &
 align_torq_wc=1000 &
 lat_force_offset=0 &
 steering_input=length &
 log_file=yes &
 analysis_mode=interactive &
 error_variable=.ACAR.variables.errorFlag
acar analysis report &
 analysis="suspension_mbd_validation_com" &
 report_template="comptest.rtp" &
 error_variable=.ACAR.variables.errorFlag
acar analysis suspension static_load submit &
 assembly=.Demo_Vehicle_Variants &
 output_prefix="suspension_mbd_validation" &
 output_suffix="static_load" &
 nsteps=4 &
 steer_upper=0 &
 steer_lower=0 &
 load_results=yes &
 steering_input=length &
 vertical_setup=wheel_center_height &
 vertical_input=wheel_center_height &
 vertical_type=relative &
 later_for_upr_l=500 &
 later_for_upr_r=500 &
 later_for_lwr_l=-500 &
 later_for_lwr_r=-500 &
 log_file=yes &
 coordinate_system=vehicle &
 analysis_mode=interactive &
 error_variable=.ACAR.variables.errorFlag
file text open file="suspension_mbd_status.txt" open=overwrite
file text write format="error=%d" value=(eval(.ACAR.variables.errorFlag))
file text close
exit confirm=yes
"""
