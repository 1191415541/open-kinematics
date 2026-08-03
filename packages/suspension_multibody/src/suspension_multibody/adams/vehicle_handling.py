"""Real Adams/Car full-vehicle handling maneuver runner."""

from __future__ import annotations

import json
import os
import shutil
import subprocess
import tempfile
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass
from pathlib import Path

from ..analysis.vehicle_correlation_model import Vehicle14DofParameters
from .probe import AdamsProfile
from .time_domain import AdamsResultChannel, TimeHistory, parse_adams_result_history
from .vehicle_acceptance import HANDLING_CASES
from .vehicle_reference import write_vehicle_reference_bundle

HandlingRunner = Callable[[AdamsProfile, str, Path], TimeHistory]

HANDLING_ADAMS_CHANNELS: Mapping[str, AdamsResultChannel] = {
    "steering_angle": AdamsResultChannel("driver_demands", "steering_angle"),
    "lateral_acceleration": AdamsResultChannel(
        "condition_sensors", "lateral_acceleration"
    ),
    "yaw_rate": AdamsResultChannel("condition_sensors", "yaw_rate"),
    "body_roll": AdamsResultChannel("condition_sensors", "roll_angle"),
}


@dataclass(frozen=True)
class HandlingExecutionResult:
    """Execution-only Adams acceptance evidence for the four handling maneuvers."""

    ok: bool
    output_path: str
    report: Mapping[str, object]


def validate_handling_execution(
    profile: AdamsProfile,
    *,
    runner: HandlingRunner | None = None,
    output_dir: str | Path | None = None,
) -> HandlingExecutionResult:
    """Execute and validate the required Adams/Car handling maneuver evidence."""
    destination = _destination(output_dir)
    execute = runner if runner is not None else run_adams_car_handling_case
    cases: dict[str, object] = {}
    for name in HANDLING_CASES:
        case_dir = destination / name
        try:
            history = execute(profile, name, case_dir)
            missing = sorted(set(HANDLING_ADAMS_CHANNELS) - set(history.channels))
            if missing:
                raise ValueError(f"Adams handling history is missing channels: {missing}")
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
        "contract": "adams-car-handling-execution-v1",
        "profile": profile.name,
        "cases": cases,
        "passed": all(bool(item["passed"]) for item in cases.values()),
        "correlation_status": "not_evaluated_without_independent_vehicle_history",
    }
    output = destination / "adams_handling_execution_report.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return HandlingExecutionResult(bool(report["passed"]), str(output), report)


def run_adams_car_handling_case(
    profile: AdamsProfile, name: str, output_dir: Path
) -> TimeHistory:
    """Run one built-in full-vehicle handling maneuver in Adams/Car."""
    if name not in HANDLING_CASES:
        raise ValueError(f"unsupported handling case: {name}")
    if not profile.executable:
        raise ValueError("Adams executable is unavailable")
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime = Path(tempfile.mkdtemp(prefix=f"suspension_multibody_handling_{name}_"))
    stem = f"handling_{name}"
    suffix = "dynamic"
    dcf_file = _dcf_file(name, runtime)
    assembly = _pac2002_assembly(profile, runtime)
    command_path = runtime / f"{stem}.cmd"
    command_path.write_text(
        _command_text(stem, suffix, dcf_file, assembly), encoding="ascii"
    )
    completed = subprocess.run(
        [profile.executable, "acar", "ru-acar", "b", str(command_path)],
        cwd=runtime,
        env=os.environ.copy(),
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
    result = runtime / f"{stem}_{suffix}.res"
    message = runtime / f"{stem}_{suffix}.msg"
    message_text = (
        message.read_text(encoding="utf-8", errors="replace")
        if message.is_file()
        else ""
    )
    if (
        completed.returncode != 0
        or not status.is_file()
        or status.read_text(encoding="utf-8").strip() != "error=0"
        or not result.is_file()
        or "Simulate status=0" not in message_text
    ):
        raise RuntimeError(f"Adams/Car handling solve failed for {name}")
    raw_dir = output_dir / "adams_raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    for source in runtime.glob(f"{stem}*"):
        shutil.copy2(source, raw_dir / source.name)
    shutil.copy2(assembly, raw_dir / assembly.name)
    local_dcf = runtime / dcf_file
    if local_dcf.is_file():
        shutil.copy2(local_dcf, raw_dir / local_dcf.name)
    _copy_builtin_inputs(profile, name, raw_dir)
    history = parse_adams_result_history(
        result,
        HANDLING_ADAMS_CHANNELS,
        units={
            "steering_angle": "rad",
            "lateral_acceleration": "mm/s^2",
            "yaw_rate": "rad/s",
            "body_roll": "rad",
        },
    )
    path = output_dir / "adams_time_history.json"
    path.write_text(json.dumps(history.as_dict(), indent=2, sort_keys=True), encoding="utf-8")
    (output_dir / "adams_execution.json").write_text(
        json.dumps(
            {
                "producer": "msc.adams-car.2024.1",
                "analysis": name,
                "analysis_mode": "full_vehicle_sdi_dynamic",
                "returncode": completed.returncode,
                "dynamic_log_marker": True,
                "raw_result_path": str(raw_dir / result.name),
                "sample_count": len(history.time),
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    write_vehicle_reference_bundle(
        case=name,
        category="handling_stability",
        history=history,
        output_dir=output_dir,
        profile=profile,
        input_manifest=_input_manifest(history, dcf_file, assembly.name),
    )
    return history


def _destination(output_dir: str | Path | None) -> Path:
    destination = (
        Path(output_dir)
        if output_dir is not None
        else Path(tempfile.mkdtemp(prefix="suspension_multibody_handling_gate_"))
    )
    destination.mkdir(parents=True, exist_ok=True)
    return destination


def _dcf_file(name: str, runtime: Path) -> str:
    sources = {
        "steady_state_circle": "mdids://acar_shared/driver_controls.tbl/constant_radius_cornering.dcf",
        "step_steer": "mdids://acar_shared/driver_controls.tbl/step_steer.dcf",
        "double_lane_change": "mdids://acar_shared/driver_controls.tbl/iso_lane_change.dcf",
    }
    if name in sources:
        return sources[name]
    path = runtime / "sine_steer.dcf"
    path.write_text(_sine_steer_dcf(), encoding="ascii")
    return path.name


def _copy_builtin_inputs(profile: AdamsProfile, name: str, raw_dir: Path) -> None:
    if profile.database_path is None:
        raise ValueError("Adams profile database path is unavailable")
    database = Path(profile.database_path)
    sources = [
        database / "roads.tbl" / "2d_flat.rdf",
    ]
    dcf_names = {
        "steady_state_circle": "constant_radius_cornering.dcf",
        "step_steer": "step_steer.dcf",
        "double_lane_change": "iso_lane_change.dcf",
    }
    if name in dcf_names:
        sources.append(database / "driver_controls.tbl" / dcf_names[name])
    for source in sources:
        if not source.is_file():
            raise ValueError(f"Adams built-in input is unavailable: {source}")
        shutil.copy2(source, raw_dir / source.name)


def _input_manifest(
    history: TimeHistory, dcf_file: str, assembly_name: str
) -> dict[str, object]:
    """Freeze the effective Adams driver demand, excluding its static zero bias."""
    steering = history.channels["steering_angle"]
    if len(steering) != len(history.time) or not steering:
        raise ValueError("Adams handling history has no steering-demand time series")
    zero_offset = steering[0]
    return {
        "analysis_mode": "full_vehicle_sdi_dynamic",
        "assembly": assembly_name,
        "variant": "default",
        "testrig": "__MDI_SDI_TESTRIG",
        "driver_control": dcf_file,
        "road": "2d_flat.rdf",
        "tire_model": "adams_builtin_pac2002",
        "tire_property_file": "pac2002_235_60R16.tir",
        "initial_state": {
            "adams": "static_equilibrium",
            "package_relative_coordinates": "zero",
        },
        "response_transform": {"body_roll": "subtract_initial_sample"},
        "vehicle_model_parameters": asdict(Vehicle14DofParameters()),
        "steering_input": {
            "kind": "sampled_driver_demand",
            "source_channel": "driver_demands.steering_angle",
            "sample_period_s": 0.01,
            "zero_offset_rad": zero_offset,
            "angle_rad": [value - zero_offset for value in steering],
        },
    }


def _pac2002_assembly(profile: AdamsProfile, runtime: Path) -> Path:
    """Copy the built-in handling assembly with its PAC2002 tire variants enabled."""
    if profile.home is None:
        raise ValueError("Adams profile home is unavailable")
    source = (
        Path(profile.home)
        / "acar"
        / "acar_concept.cdb"
        / "assemblies.tbl"
        / "Demo_Vehicle_Variants.asy"
    )
    if not source.is_file():
        raise ValueError(f"Adams handling assembly is unavailable: {source}")
    payload = source.read_text(encoding="utf-8")
    replacements = {
        "'<acar_shared>/subsystems.tbl/TR_Front_Tires.sub'": "'<acar_shared>/subsystems.tbl/TR_Front_Tires.sub::rt'",
        "'<acar_shared>/subsystems.tbl/TR_Rear_Tires.sub'": "'<acar_shared>/subsystems.tbl/TR_Rear_Tires.sub::rt'",
    }
    for original, replacement in replacements.items():
        if payload.count(original) != 1:
            raise ValueError(f"Adams handling assembly has unexpected tire usage: {original}")
        payload = payload.replace(original, replacement)
    runtime.mkdir(parents=True, exist_ok=True)
    destination = runtime / "Demo_Vehicle_Variants_pac2002.asy"
    destination.write_text(payload, encoding="utf-8")
    return destination


def _command_text(stem: str, suffix: str, dcf_file: str, assembly: Path) -> str:
    assembly_model = assembly.stem
    return f"""defaults command_file echo_commands=off
variable set variable=.ACAR.variables.errorFlag integer=0
acar files assembly open &
 assembly_name="{assembly.as_posix()}" &
 variant=default &
 error_variable=.ACAR.variables.errorFlag
acar files assembly switch &
 assembly=.{assembly_model} &
 variant=default &
 testrig=.__MDI_SDI_TESTRIG &
 error_variable=.ACAR.variables.errorFlag
acar analysis full_vehicle sdi submit &
 assembly=.{assembly_model} &
 output_prefix="{stem}" &
 output_suffix="{suffix}" &
 road_data_file="mdids://acar_shared/roads.tbl/2d_flat.rdf" &
 dcf_file="{dcf_file}" &
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


def _sine_steer_dcf() -> str:
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
EXPERIMENT_NAME = 'Open Loop Sine Steer'
STATIC_SETUP  = 'STRAIGHT'
INITIAL_SPEED = 16.667
INITIAL_GEAR  = 3
(MINI_MANEUVERS)
{mini_manuever     abort_time   step_size}
'SINE_STEER'         6.0         0.01
$---------------------------------------------------------------------SINE_STEER
[SINE_STEER]
(STEERING)
  ACTUATOR_TYPE         =     'ROTATION'
  METHOD                =     'OPEN'
  MODE                  =     'RELATIVE'
  CONTROL_TYPE          =     'SINE'
  START_TIME            =     0.5
  INITIAL_VALUE         =     0.0
  CYCLE_LENGTH          =     2.0
  AMPLITUDE             =     0.03
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
'TIME'         '=='   6.0              0.0          0.0        0.0
"""
