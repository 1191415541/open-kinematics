"""Record a reproducible full-MBD timing baseline and the Adams reference timing."""

from __future__ import annotations

import argparse
import json
import statistics
import time
from pathlib import Path

from suspension_multibody.adams import (
    build_adams_vehicle_case,
    build_adams_vehicle_model,
    load_adams_full_vehicle_input,
    read_vehicle_reference_bundle,
    steering_signal_from_manifest,
)
from suspension_multibody.analysis import FullVehicleDynamicSolver


ROOT = Path("artifacts/adams/correlation-reference-real-si/handling-pac2002-v1/step_steer")
TASK_RAW = Path(".codex-tasks/20260805-optimize-full-mbd-speed/raw")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=TASK_RAW / "baseline_report.json")
    parser.add_argument("--adams-record", type=Path, default=TASK_RAW / "adams_baseline.json")
    parser.add_argument("--repeat", type=int, default=3)
    parser.add_argument("--end-time", type=float, default=0.002)
    parser.add_argument("--step-size", type=float, default=0.001)
    parser.add_argument("--internal-step-size", type=float)
    parser.add_argument("--max-corrector-iterations", type=int)
    parser.add_argument("--reuse-constraint-linearization", action="store_true")
    parser.add_argument("--disable-velocity-recovery", action="store_true")
    parser.add_argument("--generalized-alpha-rho-inf", type=float)
    args = parser.parse_args()
    if args.repeat < 1:
        raise ValueError("--repeat must be at least one")
    if args.end_time <= 0.0 or args.step_size <= 0.0:
        raise ValueError("--end-time and --step-size must be positive")
    if args.internal_step_size is not None and args.internal_step_size <= 0.0:
        raise ValueError("--internal-step-size must be positive")
    if args.max_corrector_iterations is not None and args.max_corrector_iterations < 1:
        raise ValueError("--max-corrector-iterations must be at least one")

    reference = read_vehicle_reference_bundle(ROOT / "adams_reference_bundle.json")
    data = load_adams_full_vehicle_input(ROOT)
    started = time.perf_counter()
    model = build_adams_vehicle_model(data)
    model_build_seconds = time.perf_counter() - started
    case = build_adams_vehicle_case(
        data,
        model,
        case_name="step_steer",
        steering_input=steering_signal_from_manifest(reference.input_manifest),
        end_time=args.end_time,
        step_size=args.step_size,
    )
    solver_updates: dict[str, object] = {
        "reuse_constraint_linearization": args.reuse_constraint_linearization,
    }
    if args.disable_velocity_recovery:
        solver_updates["velocity_recovery_enabled"] = False
    if args.generalized_alpha_rho_inf is not None:
        if not 0.0 < args.generalized_alpha_rho_inf <= 1.0:
            raise ValueError("--generalized-alpha-rho-inf must be in (0, 1]")
        solver_updates["generalized_alpha_rho_inf"] = args.generalized_alpha_rho_inf
    if args.internal_step_size is not None:
        solver_updates["internal_step_size"] = args.internal_step_size
        solver_updates["min_internal_step_size"] = min(args.internal_step_size, 1.0e-4)
    if args.max_corrector_iterations is not None:
        solver_updates["max_corrector_iterations"] = args.max_corrector_iterations
    case = case.model_copy(
        update={"solver": case.solver.model_copy(update=solver_updates)}
    )
    if not args.adams_record.is_file():
        raise FileNotFoundError(
            f"Adams timing record is missing: {args.adams_record}; run the Adams baseline first"
        )
    adams = json.loads(args.adams_record.read_text(encoding="utf-8"))
    solve_times: list[float] = []
    runs = []
    try:
        for _ in range(args.repeat):
            started = time.perf_counter()
            runs.append(FullVehicleDynamicSolver().run(case))
            solve_times.append(time.perf_counter() - started)
    except Exception as error:
        elapsed = time.perf_counter() - started
        failure = {
            "contract": "full-mbd-performance-baseline-v1",
            "reference_bundle": str(ROOT),
            "full_mbd": {
                "solver": "full_mbd",
                "case": "step_steer",
                "status": "FAILED",
                "end_time_s": case.solver.end_time,
                "output_step_s": case.solver.step_size,
                "internal_step_s": case.solver.internal_step_size,
                "max_corrector_iterations": case.solver.max_corrector_iterations,
                "reuse_constraint_linearization": case.solver.reuse_constraint_linearization,
                "wall_until_failure_s": elapsed,
                "error": f"{type(error).__name__}: {error}",
            },
            "adams": adams,
            "comparable_timing": False,
            "speedup_ratio_adams_over_mbd": None,
            "timing_note": "MBD did not complete the requested run; no speed comparison is valid.",
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(failure, indent=2, sort_keys=True), encoding="utf-8")
        print(json.dumps(failure, indent=2, sort_keys=True))
        raise SystemExit(2)
    solve_seconds = statistics.median(solve_times)
    run = runs[0]
    mbd = {
        "solver": "full_mbd",
        "case": "step_steer",
        "end_time_s": case.solver.end_time,
        "output_step_s": case.solver.step_size,
        "internal_step_s": case.solver.internal_step_size,
        "max_corrector_iterations": case.solver.max_corrector_iterations,
        "reuse_constraint_linearization": case.solver.reuse_constraint_linearization,
        "model_build_s": model_build_seconds,
        "solve_wall_s": solve_seconds,
        "solve_wall_samples_s": solve_times,
        "solve_wall_min_s": min(solve_times),
        "solve_wall_median_s": solve_seconds,
        "repeat_count": args.repeat,
        "sample_count": len(run.samples),
        "max_position_residual": max(
            float(sample.constraint_residual) for item in runs for sample in item.samples
        ),
        "max_velocity_residual": max(
            float(sample.velocity_residual) for item in runs for sample in item.samples
        ),
        "event_count": sum(len(sample.events) for sample in run.samples),
    }
    comparable = (
        abs(float(adams.get("end_time_s", -1.0)) - float(case.solver.end_time)) <= 1e-12
        and abs(float(adams.get("output_step_s", -1.0)) - float(case.solver.step_size)) <= 1e-12
    )
    payload = {
        "contract": "full-mbd-performance-baseline-v1",
        "reference_bundle": str(ROOT),
        "hardware_note": "wall clock measured on the current Windows host; repeat before comparing releases",
        "full_mbd": mbd,
        "adams": adams,
        "comparable_timing": comparable,
        "speedup_ratio_adams_over_mbd": (
            float(adams["wall_s"]) / solve_seconds
            if comparable and float(solve_seconds) > 0.0 and "wall_s" in adams
            else None
        ),
        "timing_note": (
            "Adams record is a full 5 s run while the MBD record is a 2 ms probe; "
            "do not interpret the separate wall times as a solver speed comparison."
            if not comparable
            else "same duration and output step"
        ),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
