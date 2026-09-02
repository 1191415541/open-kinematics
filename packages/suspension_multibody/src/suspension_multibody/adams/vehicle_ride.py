"""Real Adams/Car full-vehicle ride-maneuver runner."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import tempfile
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path

from .probe import AdamsProfile, _adams_environment
from .time_domain import AdamsResultChannel, TimeHistory, parse_adams_result_history
from .vehicle_acceptance import RIDE_CASES
from .vehicle_reference import write_vehicle_reference_bundle

RideRunner = Callable[[AdamsProfile, str, Path], TimeHistory]

RIDE_ADAMS_CHANNELS: Mapping[str, AdamsResultChannel] = {
    "body_heave": AdamsResultChannel("ges_chassis_XFORM", "Z"),
    "body_pitch": AdamsResultChannel("ges_chassis_XFORM", "THETA"),
    "body_roll": AdamsResultChannel("ges_chassis_XFORM", "PSI"),
    "body_accel_z": AdamsResultChannel("ges_chassis_XFORM", "ACCZ"),
}
FOUR_POST_ADAMS_CHANNELS: Mapping[str, AdamsResultChannel] = {
    "jms_post_pad_vertical_lf": AdamsResultChannel(
        "jms_post_pad_vertical_lf_data", "displacement"
    ),
    "jms_post_pad_vertical_rf": AdamsResultChannel(
        "jms_post_pad_vertical_rf_data", "displacement"
    ),
    "jms_post_pad_vertical_rr": AdamsResultChannel(
        "jms_post_pad_vertical_rr_data", "displacement"
    ),
    "jms_post_pad_vertical_lr": AdamsResultChannel(
        "jms_post_pad_vertical_lr_data", "displacement"
    ),
}
FOUR_POST_FORCE_CHANNELS: Mapping[str, AdamsResultChannel] = {
    name: AdamsResultChannel(channel.entity, "force")
    for name, channel in FOUR_POST_ADAMS_CHANNELS.items()
}


@dataclass(frozen=True)
class RideExecutionResult:
    """Execution-only Adams acceptance evidence for the ride maneuver matrix."""

    ok: bool
    output_path: str
    report: Mapping[str, object]


def validate_ride_execution(
    profile: AdamsProfile,
    *,
    runner: RideRunner | None = None,
    output_dir: str | Path | None = None,
) -> RideExecutionResult:
    """Execute and validate the required Adams/Car ride maneuver evidence."""
    destination = _destination(output_dir)
    execute = runner if runner is not None else run_adams_car_ride_case
    cases: dict[str, object] = {}
    for name in RIDE_CASES:
        case_dir = destination / name
        try:
            history = execute(profile, name, case_dir)
            missing = sorted(set(RIDE_ADAMS_CHANNELS) - set(history.channels))
            if missing:
                raise ValueError(f"Adams ride history is missing channels: {missing}")
            cases[name] = {
                "passed": True,
                "sample_count": len(history.time),
                "time_start_s": history.time[0],
                "time_end_s": history.time[-1],
                "history_path": str(case_dir / "adams_time_history.json"),
            }
        except Exception as exc:
            cases[name] = {"passed": False, "error": str(exc)}
    report: dict[str, object] = {
        "contract": "adams-car-ride-execution-v1",
        "profile": profile.name,
        "cases": cases,
        "passed": all(bool(item["passed"]) for item in cases.values()),
        "correlation_status": "not_evaluated_without_independent_vehicle_history",
    }
    output = destination / "adams_ride_execution_report.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return RideExecutionResult(bool(report["passed"]), str(output), report)


def run_adams_car_ride_case(
    profile: AdamsProfile, name: str, output_dir: Path
) -> TimeHistory:
    """Run a specified Adams/Car road or four-post ride maneuver."""
    if name not in RIDE_CASES:
        raise ValueError(f"unsupported ride case: {name}")
    if not profile.executable:
        raise ValueError("Adams executable is unavailable")
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime = Path(tempfile.mkdtemp(prefix=f"suspension_multibody_ride_{name}_"))
    stem = f"ride_{name}"
    command = (
        _random_road_command(stem)
        if name == "random_road"
        else _four_post_command(stem)
    )
    command_path = runtime / f"{stem}.cmd"
    command_path.write_text(command, encoding="ascii")
    if name == "random_road":
        (runtime / "ride_straight.dcf").write_text(
            _ride_straight_dcf(), encoding="ascii"
        )
    completed = subprocess.run(
        [profile.executable, "acar", "ru-acar", "b", str(command_path)],
        cwd=runtime,
        env=_adams_environment(runtime),
        capture_output=True,
        text=True,
        timeout=600,
        check=False,
    )
    (output_dir / "adams_car.stdout.txt").write_text(
        completed.stdout or "", encoding="utf-8", errors="replace"
    )
    (output_dir / "adams_car.stderr.txt").write_text(
        completed.stderr or "", encoding="utf-8", errors="replace"
    )
    status = runtime / f"{stem}_status.txt"
    raw_dir = output_dir / "adams_raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    if name == "random_road":
        result = runtime / f"{stem}_dynamic.res"
        message = runtime / f"{stem}_dynamic.msg"
        message_text = (
            message.read_text(encoding="utf-8", errors="replace")
            if message.is_file()
            else ""
        )
        for source in runtime.glob(f"{stem}*"):
            shutil.copy2(source, raw_dir / source.name)
        if (
            completed.returncode != 0
            or not status.is_file()
            or status.read_text(encoding="utf-8").strip() != "error=0"
            or not result.is_file()
            or "Simulate status=0" not in message_text
            or _has_static_equilibrium_failure(message_text)
        ):
            raise RuntimeError(
                f"Adams/Car ride solve failed for {name}: "
                f"{_ride_solver_failure_detail(status, message_text)}"
            )
    else:
        acf = runtime / f"{stem}.acf"
        if (
            completed.returncode != 0
            or not status.is_file()
            or status.read_text(encoding="utf-8").strip() != "error=0"
            or not acf.is_file()
        ):
            raise RuntimeError(
                f"Adams/Car four-post setup failed for {name}: "
                f"{_ride_solver_failure_detail(status, '')}"
            )
        _rewrite_four_post_acf(acf, name)
        standard = subprocess.run(
            [profile.executable, "ru-standard", stem],
            cwd=runtime,
            env={**_adams_environment(runtime), "MDI_PRODUCT_NAME": "acar"},
            capture_output=True,
            text=True,
            timeout=600,
            check=False,
        )
        (output_dir / "adams_solver.stdout.txt").write_text(
            standard.stdout or "", encoding="utf-8", errors="replace"
        )
        (output_dir / "adams_solver.stderr.txt").write_text(
            standard.stderr or "", encoding="utf-8", errors="replace"
        )
        result = runtime / f"{stem}.res"
        message = runtime / f"{stem}.msg"
        message_text = (
            message.read_text(encoding="utf-8", errors="replace")
            if message.is_file()
            else ""
        )
        for source in runtime.glob(f"{stem}*"):
            shutil.copy2(source, raw_dir / source.name)
        if (
            standard.returncode != 0
            or not result.is_file()
            or "Performing Dynamic Simulation" not in message_text
            or "End Simulation" not in message_text
            or _has_static_equilibrium_failure(message_text)
        ):
            raise RuntimeError(
                f"Adams/Car ride solve failed for {name}: "
                f"{_ride_solver_failure_detail(status, message_text)}"
            )
    _copy_builtin_inputs(profile, name, raw_dir)
    history = parse_adams_result_history(
        result,
        RIDE_ADAMS_CHANNELS,
        units={
            "body_heave": "mm",
            "body_pitch": "rad",
            "body_roll": "rad",
            "body_accel_z": "mm/s^2",
        },
    )
    pad_history = (
        parse_adams_result_history(
            result,
            FOUR_POST_ADAMS_CHANNELS,
            units={name: "mm" for name in FOUR_POST_ADAMS_CHANNELS},
        )
        if name != "random_road"
        else None
    )
    pad_force_history = (
        parse_adams_result_history(
            result,
            FOUR_POST_FORCE_CHANNELS,
            units={name: "newton" for name in FOUR_POST_FORCE_CHANNELS},
        )
        if name != "random_road"
        else None
    )
    if pad_history is not None and pad_force_history is not None:
        _validate_four_post_excitations(name, pad_history, pad_force_history)
    (output_dir / "adams_time_history.json").write_text(
        json.dumps(history.as_dict(), indent=2, sort_keys=True), encoding="utf-8"
    )
    (output_dir / "adams_execution.json").write_text(
        json.dumps(
            {
                "producer": "msc.adams-car.2024.1",
                "analysis": name,
                "analysis_mode": (
                    "full_vehicle_sdi_dynamic"
                    if name == "random_road"
                    else "full_vehicle_fourpost_dynamic"
                ),
                "returncode": (
                    completed.returncode if name == "random_road" else standard.returncode
                ),
                "dynamic_log_marker": True,
                "raw_result_path": str(raw_dir / result.name),
                "sample_count": len(history.time),
                "four_post_pad_peak_mm": (
                    {
                        pad: max(abs(value) for value in values)
                        for pad, values in pad_history.channels.items()
                    }
                    if pad_history is not None
                    else None
                ),
                "four_post_force_span_n": (
                    {
                        pad: max(values) - min(values)
                        for pad, values in pad_force_history.channels.items()
                    }
                    if pad_force_history is not None
                    else None
                ),
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    write_vehicle_reference_bundle(
        case=name,
        category="ride",
        history=history,
        output_dir=output_dir,
        profile=profile,
        input_manifest={
            "analysis_mode": (
                "full_vehicle_sdi_dynamic"
                if name == "random_road"
                else "full_vehicle_fourpost_dynamic"
            ),
            "assembly": "Demo_Vehicle_Variants.asy",
            "variant": "default" if name == "random_road" else "ARide_fourpost",
            "road": (
                "road_3d_roughness_example.rdf"
                if name == "random_road"
                else "BEDPLATE"
            ),
            "four_post_functions": dict(_four_post_functions(name))
            if name != "random_road"
            else None,
        },
    )
    return history


def _destination(output_dir: str | Path | None) -> Path:
    destination = (
        Path(output_dir)
        if output_dir is not None
        else Path(tempfile.mkdtemp(prefix="suspension_multibody_ride_gate_"))
    )
    destination.mkdir(parents=True, exist_ok=True)
    return destination


def _copy_builtin_inputs(profile: AdamsProfile, name: str, raw_dir: Path) -> None:
    if name != "random_road":
        return
    if profile.database_path is None:
        raise ValueError("Adams profile database path is unavailable")
    source = Path(profile.database_path) / "roads.tbl" / "road_3d_roughness_example.rdf"
    if not source.is_file():
        raise ValueError(f"Adams built-in input is unavailable: {source}")
    shutil.copy2(source, raw_dir / source.name)


def _four_post_command(stem: str) -> str:
    return f"""defaults command_file echo_commands=off
variable set variable=.ACAR.variables.errorFlag integer=0
acar files assembly open &
 assembly_name="<acar_concept>/assemblies.tbl/Demo_Vehicle_Variants.asy" &
 variant=ARide_fourpost &
 error_variable=.ACAR.variables.errorFlag
acar files assembly switch &
 assembly=.Demo_Vehicle_Variants &
 variant=ARide_fourpost &
 error_variable=.ACAR.variables.errorFlag
acar toolkit read property_files &
 assembly=.Demo_Vehicle_Variants &
 verbose=yes &
 error_variable=.ACAR.variables.errorFlag
acar analysis full_vehicle submit &
 assembly=.Demo_Vehicle_Variants &
 analysis_name="{stem}" &
 end_time=4.0 &
 number_of_steps=400 &
 analysis_mode=files_only &
 load_results=no &
 road_data_file="BEDPLATE" &
 generate_road_geometry=no &
 simulation_type=fourpost_time &
 error_variable=.ACAR.variables.errorFlag
file text open file="{stem}_status.txt" open=overwrite
file text write format="error=%d" value=(eval(.ACAR.variables.errorFlag))
file text close
exit confirm=yes
"""


def _four_post_functions(name: str) -> Mapping[str, str]:
    zero = "0"
    bump = "STEP(TIME,0,0,0.05,30)-STEP(TIME,0.15,0,0.2,30)"
    sweep = "10*sin(180d*TIME)"
    functions = {
        "jms_post_pad_vertical_lf": zero,
        "jms_post_pad_vertical_rf": zero,
        "jms_post_pad_vertical_rr": zero,
        "jms_post_pad_vertical_lr": zero,
    }
    if name == "single_wheel_bump":
        functions["jms_post_pad_vertical_lf"] = bump
    elif name == "double_wheel_bump":
        functions["jms_post_pad_vertical_lf"] = bump
        functions["jms_post_pad_vertical_rf"] = bump
    elif name == "four_post_rig":
        functions = {key: sweep for key in functions}
    else:
        raise ValueError(f"four-post functions are unavailable for {name}")
    return functions


def _validate_four_post_excitations(
    name: str, displacement_history: TimeHistory, force_history: TimeHistory
) -> None:
    for actuator, function in _four_post_functions(name).items():
        peak = max(abs(value) for value in displacement_history.channels[actuator])
        force_values = force_history.channels[actuator]
        force_span = max(force_values) - min(force_values)
        if function != "0" and peak < 1e-3 and force_span < 1.0:
            raise RuntimeError(
                "Adams/Car four-post input was not applied: "
                f"{actuator} peak={peak} mm force_span={force_span} N"
            )


def _rewrite_four_post_acf(path: Path, name: str) -> None:
    mapping = {
        2: _four_post_functions(name)["jms_post_pad_vertical_lf"],
        3: _four_post_functions(name)["jms_post_pad_vertical_rf"],
        4: _four_post_functions(name)["jms_post_pad_vertical_rr"],
        5: _four_post_functions(name)["jms_post_pad_vertical_lr"],
    }
    updated = path.read_text(encoding="ascii")
    for diff_id, function in mapping.items():
        updated = re.sub(
            rf"^diff/{diff_id}, fun=.*$",
            f"diff/{diff_id}, fun={function}",
            updated,
            flags=re.MULTILINE,
        )
    path.write_text(updated, encoding="ascii")


def _has_static_equilibrium_failure(message: str) -> bool:
    normalized = message.lower()
    return "static equilibrium" in normalized and "fail" in normalized


def _ride_solver_failure_detail(status: Path, message: str) -> str:
    status_text = (
        status.read_text(encoding="utf-8", errors="replace").strip()
        if status.is_file()
        else "status-file-missing"
    )
    message_lines = [line.strip() for line in message.splitlines() if line.strip()]
    return f"{status_text}; message_tail={' | '.join(message_lines[-3:])}"


def _random_road_command(stem: str) -> str:
    return f"""defaults command_file echo_commands=off
variable set variable=.ACAR.variables.errorFlag integer=0
acar files assembly open &
 assembly_name="<acar_concept>/assemblies.tbl/Demo_Vehicle_Variants.asy" &
 variant=default &
 error_variable=.ACAR.variables.errorFlag
acar files assembly switch &
 assembly=.Demo_Vehicle_Variants &
 variant=default &
 testrig=.__MDI_SDI_TESTRIG &
 error_variable=.ACAR.variables.errorFlag
acar analysis full_vehicle sdi submit &
 assembly=.Demo_Vehicle_Variants &
 output_prefix="{stem}" &
 output_suffix="dynamic" &
 road_data_file="mdids://acar_shared/roads.tbl/road_3d_roughness_example.rdf" &
 dcf_file="ride_straight.dcf" &
 analysis_mode=background &
 load_results=yes &
 log_file=yes &
 verbose=yes &
 caller=macro &
 error_variable=.ACAR.variables.errorFlag
file text open file="{stem}_status.txt" open=overwrite
file text write format="error=%d" value=(eval(.ACAR.variables.errorFlag))
file text close
exit confirm=yes
"""


def _ride_straight_dcf() -> str:
    return """$---------------------------------------------------------------------MDI_HEADER
[MDI_HEADER]
 FILE_TYPE     = 'dcf'
 FILE_VERSION  = 2.0
 FILE_FORMAT   = 'ASCII'
$--------------------------------------------------------------------------UNITS
[UNITS]
 LENGTH  =  'meter'
 FORCE   =  'newton'
 ANGLE   =  'radians'
 MASS    =  'kg'
 TIME    =  'sec'
$---------------------------------------------------------------------EXPERIMENT
[EXPERIMENT]
EXPERIMENT_NAME = 'Straight rough-road ride'
STATIC_SETUP  = 'STRAIGHT'
INITIAL_SPEED = 12.0
INITIAL_GEAR  = 3
(MINI_MANEUVERS)
{mini_manuever     abort_time   step_size}
'STEP_STEER'         8.0         0.01
$---------------------------------------------------------------------STEP_STEER
[STEP_STEER]
(STEERING)
  ACTUATOR_TYPE         =     'ROTATION'
  METHOD                =     'OPEN'
  MODE                  =     'ABSOLUTE'
  CONTROL_TYPE          =     'STEP'
  START_TIME            =     0.0
  INITIAL_VALUE         =     0.0
  DURATION              =     0.1
  FINAL_VALUE           =     0.0
(THROTTLE)
  METHOD                =     'MACHINE'
(BRAKING)
  METHOD                =     'MACHINE'
(GEAR)
  METHOD                =     'OPEN'
  MODE                  =     'ABSOLUTE'
  CONTROL_TYPE          =     'CONSTANT'
  CONTROL_VALUE         =     3
(CLUTCH)
  METHOD                =     'OPEN'
  MODE                  =     'ABSOLUTE'
  CONTROL_TYPE          =     'CONSTANT'
  CONTROL_VALUE         =     0
(MACHINE_CONTROL)
  SPEED_CONTROL         =     'MAINTAIN'
(END_CONDITIONS)
{measure       test   value      allowed_error filter_time delay_time group}
'TIME'         '=='   8.0              0.0          0.0        0.0
"""
