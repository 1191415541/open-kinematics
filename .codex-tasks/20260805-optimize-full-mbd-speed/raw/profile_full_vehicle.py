"""Profile the short real-Adams-input full-vehicle MBD solve."""

from __future__ import annotations

import cProfile
import io
import pstats
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
    reference = read_vehicle_reference_bundle(ROOT / "adams_reference_bundle.json")
    data = load_adams_full_vehicle_input(ROOT)
    model = build_adams_vehicle_model(data)
    case = build_adams_vehicle_case(
        data,
        model,
        case_name="step_steer",
        steering_input=steering_signal_from_manifest(reference.input_manifest),
        end_time=0.002,
        step_size=0.001,
    )
    profiler = cProfile.Profile()
    profiler.enable()
    run = FullVehicleDynamicSolver().run(case)
    profiler.disable()
    stats_path = TASK_RAW / "full_vehicle_short.prof"
    stats_path.parent.mkdir(parents=True, exist_ok=True)
    profiler.dump_stats(str(stats_path))
    stream = io.StringIO()
    pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(80)
    report_path = TASK_RAW / "full_vehicle_profile.txt"
    report_path.write_text(stream.getvalue(), encoding="utf-8")
    print(stream.getvalue())
    print(f"sample_count={len(run.samples)}")
    print(f"max_position_residual={max(sample.constraint_residual for sample in run.samples):.12g}")
    print(f"max_velocity_residual={max(sample.velocity_residual for sample in run.samples):.12g}")
    print(f"profile={stats_path}")


if __name__ == "__main__":
    main()
