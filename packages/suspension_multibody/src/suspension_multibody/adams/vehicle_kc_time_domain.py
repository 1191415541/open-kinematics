"""Native Adams dynamic runner and acceptance gate for body-roll KC traces."""

from __future__ import annotations

import json
import shutil
import subprocess
import tempfile
from pathlib import Path

from ..api import run_dynamic_case
from ..schema import DynamicCaseSpec, FrontAxleModel, TimeSignal
from .probe import AdamsProfile, _adams_environment
from .time_domain import (
    AdamsResultChannel,
    TimeHistoryTolerance,
    history_from_dynamic_bundle,
    parse_adams_result_history,
    write_time_history,
)
from .time_domain_gate import (
    AdamsTimeDomainAdapter,
    TimeDomainGateResult,
    TimeDomainRunner,
)

VEHICLE_KC_CHANNELS = ("body_roll",)


def validate_vehicle_kc_time_domain(
    profile: AdamsProfile,
    model: FrontAxleModel,
    case: DynamicCaseSpec,
    *,
    runner: TimeDomainRunner | None = None,
    output_dir: str | Path | None = None,
) -> TimeDomainGateResult:
    """Validate a prescribed body-roll KC time trace against native Adams."""
    if case.mode != "vehicle_kc_dynamic":
        raise ValueError("vehicle KC Adams gate requires mode='vehicle_kc_dynamic'")
    if case.vehicle is None:
        raise ValueError("vehicle KC Adams gate requires a vehicle body model")
    reference = history_from_dynamic_bundle(
        run_dynamic_case(model, case),
        body=case.vehicle.name,
        channels=VEHICLE_KC_CHANNELS,
        units={"body_roll": "rad"},
    )
    return AdamsTimeDomainAdapter(
        profile,
        runner if runner is not None else run_vehicle_kc_roll_adams,
    ).validate(
        analysis="vehicle_kc_time_domain",
        model=model,
        case=case,
        reference=reference,
        tolerances={
            "body_roll": TimeHistoryTolerance(
                absolute=2e-5,
                peak_relative_percent=0.1,
                rms_relative_percent=0.1,
                phase_ms=1.0,
            )
        },
        output_dir=output_dir,
    )


def run_vehicle_kc_roll_adams(
    profile: AdamsProfile, request_path: Path, output_dir: Path
) -> Path:
    """Run a real native Adams dynamic body-roll KC result and retain evidence."""
    if not profile.executable:
        raise ValueError("Adams executable is unavailable")
    request = json.loads(request_path.read_text(encoding="utf-8"))
    case = DynamicCaseSpec.model_validate(request["case"])
    if case.vehicle is None:
        raise ValueError("vehicle KC Adams runner requires vehicle data")
    roll = _supported_roll_signal(case)
    runtime = Path(tempfile.mkdtemp(prefix="suspension_multibody_vehicle_kc_"))
    stem = "vehicle_kc_time_domain"
    model_path = runtime / f"{stem}.adm"
    command_path = runtime / f"{stem}.acf"
    model_path.write_text(_vehicle_kc_model_text(case, roll), encoding="ascii")
    command_path.write_text(_command_text(stem, case), encoding="ascii")
    completed = subprocess.run(
        [profile.executable, "ru-standard", stem],
        cwd=runtime,
        env=_adams_environment(runtime),
        capture_output=True,
        text=True,
        timeout=300,
        check=False,
    )
    (output_dir / "adams_vehicle_kc.stdout.txt").write_text(
        completed.stdout or "", encoding="utf-8", errors="replace"
    )
    (output_dir / "adams_vehicle_kc.stderr.txt").write_text(
        completed.stderr or "", encoding="utf-8", errors="replace"
    )
    message_path = runtime / f"{stem}.msg"
    result_path = runtime / f"{stem}.res"
    message = (
        message_path.read_text(encoding="utf-8", errors="replace")
        if message_path.is_file()
        else ""
    )
    if (
        completed.returncode != 0
        or "Performing Dynamic Simulation" not in message
        or "End Simulation" not in message
        or not result_path.is_file()
    ):
        raise RuntimeError(
            f"native Adams vehicle-KC dynamic solve failed with code {completed.returncode}"
        )
    raw_dir = output_dir / "adams_vehicle_kc_raw"
    raw_dir.mkdir(parents=True, exist_ok=True)
    for source in runtime.glob(f"{stem}.*"):
        shutil.copy2(source, raw_dir / source.name)
    history = parse_adams_result_history(
        result_path,
        {"body_roll": AdamsResultChannel("roll_angle", "roll_angle")},
        units={"body_roll": "rad"},
    )
    output = write_time_history(history, output_dir / "adams_time_history.json")
    (output_dir / "adams_vehicle_kc_execution.json").write_text(
        json.dumps(
            {
                "producer": "msc.adams-solver.2024.1",
                "analysis": "vehicle_kc_time_domain",
                "analysis_mode": "dynamic",
                "runtime_directory": str(runtime),
                "result_path": str(raw_dir / result_path.name),
                "sample_count": len(history.time),
                "returncode": completed.returncode,
                "dynamic_log_marker": True,
            },
            indent=2,
            sort_keys=True,
        ),
        encoding="utf-8",
    )
    return output


def _supported_roll_signal(case: DynamicCaseSpec) -> TimeSignal:
    values = {
        motion.target: motion.displacement
        for motion in case.prescribed_motions
        if motion.target in {"body_roll", "body_pitch", "body_yaw", "body_heave"}
    }
    for target in ("body_pitch", "body_yaw", "body_heave"):
        signal = values.get(target)
        if signal is not None and not _is_zero(signal):
            raise ValueError(
                f"native vehicle-KC Adams runner currently supports body_roll only; "
                f"{target} must be zero"
            )
    if case.wrench_inputs:
        raise ValueError(
            "native vehicle-KC Adams runner does not accept applied wrenches"
        )
    return values.get("body_roll", TimeSignal(constant=0.0))


def _is_zero(signal: TimeSignal) -> bool:
    values = (signal.constant,) if signal.constant is not None else signal.values
    return all(value == 0.0 for value in values)


def _vehicle_kc_model_text(case: DynamicCaseSpec, roll: TimeSignal) -> str:
    assert case.vehicle is not None
    inertia = case.vehicle.inertia
    return "\n".join(
        (
            "ADAMS/View model name: vehicle_kc_time_domain",
            "UNITS/FORCE = NEWTON, MASS = KILOGRAM, LENGTH = MILLIMETER, TIME = SECOND",
            "PART/1, GROUND",
            "MARKER/1, PART = 1",
            (
                "PART/2, QG = 0, 0, 0, "
                f"MASS = {case.vehicle.mass:.12g}, CM = 2, "
                f"IP = {inertia[0][0]:.12g}, {inertia[1][1]:.12g}, {inertia[2][2]:.12g}"
            ),
            "MARKER/2, PART = 2",
            "JOINT/1, REVOLUTE, I = 2, J = 1",
            "MOTION/1",
            ", ROTATIONAL",
            ", JOINT = 1",
            f", FUNCTION = {_adams_piecewise_linear(roll)}",
            "REQUEST/1",
            ", TITLE = roll_angle",
            ', CUNITS = "no_units", "angle"',
            ', CNAMES = "", "roll_angle"',
            ", RESULTS_NAME = roll_angle",
            ", F2 = AZ(2, 1)",
            "ACCGRAV/KGRAV = 0",
            "EQUILIBRIUM/",
            "OUTPUT/REQSAVE",
            "RESULTS/FORMATTED, XRF",
            "END",
            "",
        )
    )


def _command_text(stem: str, case: DynamicCaseSpec) -> str:
    output_step = case.solver.output_step or case.solver.step_size
    return (
        f"\nfile/model={stem}\n"
        f"simulate/dynamic, end={case.solver.end_time:.12g}, dtout={output_step:.12g}\n"
        "stop\n"
    )


def _adams_piecewise_linear(signal: TimeSignal) -> str:
    if signal.constant is not None:
        return f"{signal.constant:.12g}"
    times = signal.times
    values = signal.values
    expression = f"{values[-1]:.12g}"
    for index in range(len(times) - 2, -1, -1):
        start = times[index]
        end = times[index + 1]
        value = values[index]
        next_value = values[index + 1]
        slope = (next_value - value) / (end - start)
        linear = f"({value:.12g}+{slope:.12g}*(TIME-{start:.12g}))"
        expression = (
            f"IF(TIME-{start:.12g}:{value:.12g},"
            f"{value:.12g},IF(TIME-{end:.12g}:{linear},"
            f"{next_value:.12g},{expression}))"
        )
    return expression
