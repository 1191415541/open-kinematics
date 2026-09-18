"""Real Adams/Car full-vehicle handling maneuver runner."""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import tempfile
from collections.abc import Callable, Mapping
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Literal

from .probe import AdamsProfile, _adams_environment, producer_id
from .time_domain import AdamsResultChannel, TimeHistory, parse_adams_result_history
from .vehicle_acceptance import HANDLING_CASES
from .vehicle_parameters import Vehicle14DofParameters
from .vehicle_reference import write_vehicle_reference_bundle

HandlingRunner = Callable[[AdamsProfile, str, Path], TimeHistory]
HandlingTireModel = Literal["pac2002", "native_brush", "fiala"]

# Straight-line maneuvers, accepted by ``run_adams_car_handling_case`` but kept out
# of HANDLING_CASES.  A USE_MODE that emits only a longitudinal force (1, 11) or no
# slip force at all (0) cannot be trimmed by any steering maneuver, so without one
# of these those modes have no route to an Adams reference.
STRAIGHT_LINE_CASES: tuple[str, ...] = ("acceleration",)

# Stand-still steering, also kept out of HANDLING_CASES.  It is the only maneuver
# that excites turn slip (see ``_parking_steer_dcf``), and it is deliberately not
# part of the handling-evidence report: a vehicle standing still with its wheels
# turning is not handling evidence, it is the USE_MODE-25 parking case.
#
# ``parking_steer`` is the documented stand-still sine steer and is the maneuver the
# parking torque is defined on.  It is also an extreme operating point: at 1 m/s with
# 0.2 rad of steering the tires saturate and the vehicle skids, which the native
# model's own saturation device does not reproduce on a per-wheel basis (measured:
# the per-wheel force channels are equally far off at USE_MODE 24 and 25, while the
# handling channels separate them cleanly -- see the task-D notes).  ``parking_steer_fast``
# is the same regime with the *rate* raised instead of the amplitude: a fast, small
# sine gives a large turn slip (psi_dot_c/|Vx|) at a small slip angle, so the spin
# factors and the turn-slip relaxation are exercised without saturating.
PARKING_CASES: tuple[str, ...] = ("parking_steer", "parking_steer_fast")

# Braking maneuvers, also kept out of HANDLING_CASES.  Braking is the only way to
# give the *front* wheels a large longitudinal force: they are undriven, so a
# constant-speed or accelerating maneuver leaves their slip ratio near zero and
# their longitudinal-force channel sitting at ~0.2 % of the wheel load, where the
# per-wheel peak-relative gate measures numerical noise rather than tire physics.
# ``braking_in_turn`` is Adams' own braking-while-turning control (initial speed
# 13.286 m/s, 1.961 m/s^2 deceleration from t = 1 s, ending at 2.5 m/s), so it also
# exercises combined slip, which no constant-speed maneuver does.
BRAKING_CASES: tuple[str, ...] = ("braking_in_turn",)

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
        "tire_model": "pac2002",
        "cases": cases,
        "passed": all(bool(item["passed"]) for item in cases.values()),
        "correlation_status": "not_evaluated_without_independent_vehicle_history",
    }
    output = destination / "adams_handling_execution_report.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return HandlingExecutionResult(bool(report["passed"]), str(output), report)


def validate_handling_execution_matrix(
    profile: AdamsProfile,
    *,
    runner: HandlingRunner | None = None,
    output_dir: str | Path | None = None,
) -> HandlingExecutionResult:
    """执行操稳模型矩阵，并显式记录缺失的独立 brush 参考。."""
    destination = _destination(output_dir)
    pac2002 = validate_handling_execution(
        profile,
        runner=runner,
        output_dir=destination / "pac2002",
    )
    brush_reason = (
        "独立的 Adams/Car native_brush 整车参考尚未提供，不能复用 PAC2002 参考"
    )
    brush_cases = {
        name: {
            "status": "BLOCKED",
            "passed": False,
            "tire_model": "native_brush",
            "error": brush_reason,
        }
        for name in HANDLING_CASES
    }
    report = {
        "contract": "adams-car-handling-execution-matrix-v1",
        "profile": profile.name,
        "variants": {
            "pac2002": pac2002.report,
            "native_brush": {
                "status": "BLOCKED",
                "tire_model": "native_brush",
                "cases": brush_cases,
                "passed": False,
                "error": brush_reason,
            },
        },
        "passed": False,
    }
    output = destination / "adams_handling_execution_matrix_report.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return HandlingExecutionResult(False, str(output), report)


def run_adams_car_handling_case(
    profile: AdamsProfile,
    name: str,
    output_dir: Path,
    *,
    tire_model: HandlingTireModel = "pac2002",
    tire_property_file: Path | None = None,
) -> TimeHistory:
    """
    Run one built-in full-vehicle handling maneuver in Adams/Car.

    ``tire_property_file`` overrides the tire the assembly would use.  Adams
    hardcodes the tire inside its tire subsystems, so this rewrites run-local
    copies.  It exists so a reference case can be produced for a tire variant
    (for example a different ``USE_MODE``) that native support needs to be
    validated against.

    ``STRAIGHT_LINE_CASES`` are accepted in addition to ``HANDLING_CASES`` but are
    deliberately not part of that tuple: the latter drives the required
    handling-evidence report, and a straight-line run is not handling evidence.  It
    exists because a USE_MODE that emits only a longitudinal force (or no slip
    force at all) cannot be trimmed by any steering maneuver, so the pure-axis modes
    have no other route to an Adams reference.  ``PARKING_CASES`` is out of that
    tuple for the same reason: standing still with the wheels turning is the
    USE_MODE-25 parking case, not handling evidence.
    """
    if (
        name not in HANDLING_CASES
        and name not in STRAIGHT_LINE_CASES
        and name not in PARKING_CASES
        and name not in BRAKING_CASES
    ):
        raise ValueError(f"unsupported handling case: {name}")
    if not profile.executable:
        raise ValueError("Adams executable is unavailable")
    if tire_model == "native_brush":
        raise ValueError(
            "独立的 Adams/Car native_brush 整车参考尚未提供，不能复用 PAC2002 装配"
        )
    output_dir.mkdir(parents=True, exist_ok=True)
    runtime = Path(tempfile.mkdtemp(prefix=f"suspension_multibody_handling_{name}_"))
    stem = f"handling_{name}"
    suffix = "dynamic"
    dcf_file = _dcf_file(name, runtime)
    if tire_property_file is not None:
        assembly = _assembly_with_tire(
            profile, runtime, Path(tire_property_file), "custom"
        )
    else:
        assembly = (
            _fiala_assembly(profile, runtime)
            if tire_model == "fiala"
            else _pac2002_assembly(profile, runtime)
        )
    command_path = runtime / f"{stem}.cmd"
    command_path.write_text(
        _command_text(stem, suffix, dcf_file, assembly), encoding="ascii"
    )
    completed = subprocess.run(
        [profile.executable, "acar", "ru-acar", "b", str(command_path)],
        cwd=runtime,
        env=_adams_environment(runtime),
        capture_output=True,
        text=True,
        # Decode explicitly; see `adams/adapter.py`.
        encoding="utf-8",
        errors="replace",
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
                "producer": producer_id("adams-car", profile.version),
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
        input_manifest=_input_manifest(
            history,
            dcf_file,
            assembly.name,
            tire_model=tire_model,
            tire_property_file_name=(
                Path(tire_property_file).name
                if tire_property_file is not None
                else None
            ),
        ),
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
        "acceleration": "mdids://acar_shared/driver_controls.tbl/acceleration.dcf",
        "braking_in_turn": "mdids://acar_shared/driver_controls.tbl/braking_in_turn.dcf",
    }
    if name in sources:
        return sources[name]
    if name in PARKING_CASES:
        path = runtime / f"{name}.dcf"
        if name == "parking_steer_fast":
            # Turn slip is psi_dot_c/|Vx|, and the slip angle is what saturates the
            # tire.  Raising the steering *rate* at a small amplitude moves the first
            # without the second: 0.03 rad at a 0.15 s cycle gives psi_dot_c up to
            # 1.26 rad/s, i.e. R0*phi ~ 0.4 at 1 m/s, while the slip angle stays near
            # 0.03 rad so the tire stays inside its friction limit.
            path.write_text(
                _parking_steer_dcf(
                    amplitude=0.03, cycle_length=0.15, duration=1.0
                ),
                encoding="ascii",
            )
        else:
            path.write_text(_parking_steer_dcf(), encoding="ascii")
        return path.name
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
        "acceleration": "acceleration.dcf",
        "braking_in_turn": "braking_in_turn.dcf",
    }
    if name in dcf_names:
        sources.append(database / "driver_controls.tbl" / dcf_names[name])
    for source in sources:
        if not source.is_file():
            raise ValueError(f"Adams built-in input is unavailable: {source}")
        shutil.copy2(source, raw_dir / source.name)


def _input_manifest(
    history: TimeHistory,
    dcf_file: str,
    assembly_name: str,
    *,
    tire_model: HandlingTireModel = "pac2002",
    tire_property_file_name: str | None = None,
) -> dict[str, object]:
    """Freeze the effective Adams driver demand, excluding its static zero bias."""
    steering = history.channels["steering_angle"]
    if len(steering) != len(history.time) or not steering:
        raise ValueError("Adams handling history has no steering-demand time series")
    zero_offset = steering[0]
    reference_tire_model = (
        "adams_builtin_pac2002"
        if tire_model == "pac2002"
        else "adams_builtin_fiala"
        if tire_model == "fiala"
        else "adams_generated_brush"
    )
    tire_property_file = tire_property_file_name or (
        "pac2002_235_60R16.tir"
        if tire_model == "pac2002"
        else "fiala_235_45R17.tir"
        if tire_model == "fiala"
        else None
    )
    return {
        "analysis_mode": "full_vehicle_sdi_dynamic",
        "assembly": assembly_name,
        "variant": "default",
        "testrig": "__MDI_SDI_TESTRIG",
        "driver_control": dcf_file,
        "road": "2d_flat.rdf",
        "tire_model": reference_tire_model,
        "tire_property_file": tire_property_file,
        "tire_property_source": (
            "adams_builtin_tir"
            if tire_model in {"pac2002", "fiala"}
            else "native_brush_generator"
        ),
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


def _replace_rt_tire_property(payload: str, tire_file: Path, source_name: str) -> str:
    """
    Point the ``{rt}`` variant of a tire subsystem at ``tire_file``.

    A tire subsystem declares its default tire in ``[WHEEL_ASSEMBLY]`` and the
    high-performance tire inside the ``{rt}`` variant block.  The handling
    assembly selects the variant (``...TR_Front_Tires.sub::rt``), so only the
    variant entry may change; rewriting the default entry as well would alter an
    unrelated configuration.
    """
    marker = "{rt}"
    head, separator, tail = payload.partition(marker)
    if not separator:
        raise ValueError(f"{source_name} has no {{rt}} variant block")
    patched, count = re.subn(
        r"(PROPERTY_FILE\s*=\s*)'[^']+\.tir'",
        rf"\1'{tire_file.as_posix()}'",
        tail,
        count=1,
        flags=re.IGNORECASE,
    )
    if count != 1:
        raise ValueError(
            f"expected one PROPERTY_FILE in the {{rt}} block of {source_name}"
        )
    return head + separator + patched


def _assembly_with_tire(
    profile: AdamsProfile, runtime: Path, tire_file: Path, label: str
) -> Path:
    """
    Create a run-local assembly whose tire variants use ``tire_file``.

    Adams hardcodes the tire property file inside ``TR_<axle>_Tires.sub``, so a
    different tire requires run-local copies of those subsystems plus an assembly
    that points at them.  ``label`` only distinguishes the generated file names.
    """
    if profile.home is None:
        raise ValueError("Adams profile home is unavailable")
    home = Path(profile.home)
    source = (
        home / "acar" / "acar_concept.cdb" / "assemblies.tbl"
        / "Demo_Vehicle_Variants.asy"
    )
    database = home / "acar" / "shared_car_database.cdb"
    if not source.is_file() or not tire_file.is_file():
        raise ValueError(f"Adams handling inputs are unavailable: {tire_file}")
    runtime.mkdir(parents=True, exist_ok=True)
    replacements: dict[str, str] = {}
    for axle in ("Front", "Rear"):
        source_sub = database / "subsystems.tbl" / f"TR_{axle}_Tires.sub"
        destination_sub = runtime / f"TR_{axle}_Tires_{label}.sub"
        patched = _replace_rt_tire_property(
            source_sub.read_text(encoding="utf-8"), tire_file, source_sub.name
        )
        destination_sub.write_text(patched, encoding="ascii")
        replacements[
            f"'<acar_shared>/subsystems.tbl/TR_{axle}_Tires.sub'"
        ] = f"'{destination_sub.as_posix()}::rt'"
    assembly_payload = source.read_text(encoding="utf-8")
    for original, replacement in replacements.items():
        if assembly_payload.count(original) != 1:
            raise ValueError(
                f"Adams handling assembly has unexpected tire usage: {original}"
            )
        assembly_payload = assembly_payload.replace(original, replacement)
    destination = runtime / f"Demo_Vehicle_Variants_{label}.asy"
    destination.write_text(assembly_payload, encoding="utf-8")
    return destination


def _fiala_assembly(profile: AdamsProfile, runtime: Path) -> Path:
    """Create a run-local assembly whose tire variants use Adams Fiala."""
    if profile.home is None:
        raise ValueError("Adams profile home is unavailable")
    fiala = (
        Path(profile.home) / "acar" / "acar_concept.cdb" / "tires.tbl"
        / "fiala_235_45R17.tir"
    )
    return _assembly_with_tire(profile, runtime, fiala, "fiala")


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


def _parking_steer_dcf(
    *,
    initial_speed: float = 1.0,
    amplitude: float = 0.2,
    cycle_length: float = 1.0,
    duration: float = 4.0,
) -> str:
    r"""
    Very low speed with aggressive steering: the maneuver that excites turn slip.

    Turn slip is a curvature (spin per unit distance), so a driving maneuver barely
    produces any: a step steer at 20 m/s with 0.3 rad/s of yaw gives
    ``phi = yaw_rate/Vx = 0.015 rad/m``, hence ``R0*phi = 0.005`` and every spin
    factor of Eq3329-Eq3356 is one to six decimals.  A steady-state circle is no
    better -- its turn slip is exactly ``1/R``, independent of speed, so it needs a
    radius of roughly 0.4 m to matter.

    What does matter is a large steering *rate* at a small speed, because
    ``phi = psi_dot_c/|Vx|``: the default here is 0.2 rad of steering at a 1 s cycle
    (``psi_dot_c`` up to 1.26 rad/s) at 1 m/s, which gives ``R0*phi`` near 0.4 and
    therefore ``zeta2 = cos[arctan(0.4*R0*phi)] ~ 0.94`` and a sharply reduced
    cornering stiffness -- comfortably outside the noise of the steering maneuvers
    the handling gate uses.

    ``initial_speed`` cannot be zero: the SDI test rig aborts with
    "Initial velocity too small" (ID 1020).  The relaxation still behaves like
    stand-still here, because the turn-slip time constant ``sigma_c/|Vx|`` is short
    against the steering cycle while the steady turn slip stays large.
    """
    return f"""$---------------------------------------------------------------------MDI_HEADER
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
EXPERIMENT_NAME = 'Low Speed Parking Steer'
STATIC_SETUP  = 'STRAIGHT'
INITIAL_SPEED = {initial_speed:.6g}
INITIAL_GEAR  = 1
(MINI_MANEUVERS)
{{mini_manuever     abort_time   step_size}}
'SINE_STEER'         {duration:.6g}         0.01
$---------------------------------------------------------------------SINE_STEER
[SINE_STEER]
(STEERING)
  ACTUATOR_TYPE         =     'ROTATION'
  METHOD                =     'OPEN'
  MODE                  =     'RELATIVE'
  CONTROL_TYPE          =     'SINE'
  START_TIME            =     0.5
  INITIAL_VALUE         =     0.0
  CYCLE_LENGTH          =     {cycle_length:.6g}
  AMPLITUDE             =     {amplitude:.6g}
(THROTTLE)
  METHOD                =     'MACHINE'
(BRAKING)
  METHOD                =     'MACHINE'
(GEAR)
  METHOD                =     'OPEN'
  MODE                  =     'ABSOLUTE'
  CONTROL_TYPE          =     'CONSTANT'
  CONTROL_VALUE         =     1
(CLUTCH)
  METHOD                =     'OPEN'
  MODE                  =     'ABSOLUTE'
  CONTROL_TYPE          =     'CONSTANT'
  CONTROL_VALUE         =     0
(MACHINE_CONTROL)
  SPEED_CONTROL         =     'MAINTAIN'
(END_CONDITIONS)
{{measure       test   value      allowed_error filter_time delay_time group}}
'TIME'         '=='   {duration:.6g}              0.0          0.0        0.0
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
