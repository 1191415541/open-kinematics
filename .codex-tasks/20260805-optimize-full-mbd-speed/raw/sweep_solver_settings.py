"""Sweep full-vehicle time-step settings against residual and event gates."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from time import perf_counter

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
    parser.add_argument("--end-time", type=float, default=0.02)
    parser.add_argument("--reuse-constraint-linearization", action="store_true")
    parser.add_argument("--output", type=Path, default=TASK_RAW / "internal_step_sweep.json")
    args = parser.parse_args()

    reference = read_vehicle_reference_bundle(ROOT / "adams_reference_bundle.json")
    data = load_adams_full_vehicle_input(ROOT)
    model = build_adams_vehicle_model(data)
    base = build_adams_vehicle_case(
        data,
        model,
        case_name="step_steer",
        steering_input=steering_signal_from_manifest(reference.input_manifest),
        end_time=args.end_time,
        step_size=min(0.01, args.end_time),
    )
    records: list[dict[str, object]] = []
    for internal_step in (0.00025, 0.0005, 0.001, 0.002, 0.005):
        for correctors in (1, 2, 3):
            settings = base.solver.model_copy(
                update={
                    "internal_step_size": internal_step,
                    "min_internal_step_size": min(internal_step, 1.0e-4),
                    "adaptive_substepping": False,
                    "max_corrector_iterations": correctors,
                    "reuse_constraint_linearization": args.reuse_constraint_linearization,
                }
            )
            case = base.model_copy(update={"solver": settings})
            started = perf_counter()
            try:
                run = FullVehicleDynamicSolver().run(case)
            except Exception as error:
                records.append(
                    {
                        "internal_step_s": internal_step,
                        "max_corrector_iterations": correctors,
                        "status": "FAILED",
                        "error": f"{type(error).__name__}: {error}",
                        "wall_s": perf_counter() - started,
                    }
                )
                continue
            records.append(
                {
                    "internal_step_s": internal_step,
                    "max_corrector_iterations": correctors,
                    "status": "OK",
                    "wall_s": perf_counter() - started,
                    "max_position_residual": max(
                        sample.constraint_residual for sample in run.samples
                    ),
                    "max_velocity_residual": max(
                        sample.velocity_residual for sample in run.samples
                    ),
                    "event_count": sum(len(sample.events) for sample in run.samples),
                    "final_metrics": run.final.metrics,
                }
            )
    payload = {
        "contract": "full-mbd-internal-step-sweep-v1",
        "case": "step_steer",
        "end_time_s": args.end_time,
        "reuse_constraint_linearization": args.reuse_constraint_linearization,
        "records": records,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
