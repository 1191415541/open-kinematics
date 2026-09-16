"""Measure Native Fiala solve time against Adams solver elapsed."""

from __future__ import annotations

import argparse
import json
import os
import re
from dataclasses import asdict
from pathlib import Path
from time import perf_counter
from typing import Any

import numpy as np
from run_full_native_three_model_comparison import (  # noqa: E402
    _assert_adams_tire_geometry_matches,
    _native_case,
)

from suspension_multibody.adams import (  # noqa: E402
    adams_contact_patch_plane_height_m,
    adams_rack_displacement_signal_from_result,
    build_native_rack_steering_model,
    load_adams_full_vehicle_input,
    resolve_adams_home,
)
from suspension_multibody.adams.full_vehicle_model import (  # noqa: E402
    build_adams_source_vehicle_model,
)
from suspension_multibody.vehicle_dynamics import run_vehicle_dynamics  # noqa: E402


def _default_fiala_tire_property() -> Path:
    """Locate the Fiala tire of the installed Adams/Car concept database."""
    home = resolve_adams_home()
    relative = Path(
        "acar/acar_concept.cdb/tires.tbl/fiala_235_45R17.tir"
    )
    if home is not None and (home / relative).is_file():
        return home / relative
    return Path("adams-car-installation-is-unavailable") / relative


def _adams_solver_elapsed(source_root: Path) -> dict[str, float]:
    msg = source_root / "adams_raw" / "handling_step_steer_dynamic.msg"
    text = msg.read_text(encoding="utf-8", errors="replace")
    elapsed = re.search(r"Elapsed time\s*=\s*([0-9.]+)s", text)
    cpu = re.search(r"CPU time\s*=\s*([0-9.]+)s", text)
    simulation = re.search(r"Simulation time is\s*([0-9.Ee+-]+)", text)
    if elapsed is None or simulation is None:
        raise ValueError(f"Adams msg 缺少 solver elapsed 或 simulation time: {msg}")
    return {
        "simulation_time_s": float(simulation.group(1)),
        "solver_elapsed_s": float(elapsed.group(1)),
        "solver_cpu_s": float(cpu.group(1)) if cpu is not None else float("nan"),
    }


def _with_profile_flag(enabled: bool):
    previous = os.environ.get("SUSPENSION_AXLE_PROFILE")
    os.environ["SUSPENSION_AXLE_PROFILE"] = "1" if enabled else "0"
    return previous


def _restore_profile_flag(previous: str | None) -> None:
    if previous is None:
        os.environ.pop("SUSPENSION_AXLE_PROFILE", None)
    else:
        os.environ["SUSPENSION_AXLE_PROFILE"] = previous


def measure(
    source_root: Path,
    tire_property_file: Path,
    *,
    end_time: float,
    step: float,
    warmups: int,
    measured_runs: int,
    profile_run: bool,
) -> dict[str, Any]:
    """测量 native Fiala 求解器的重复墙钟时间."""
    source_root = source_root.resolve()
    result_path = source_root / "adams_raw" / "handling_step_steer_dynamic.res"
    data = load_adams_full_vehicle_input(
        source_root, tire_property_file=tire_property_file
    )
    expected_radius_mm = float(data.fiala_parameters.get("UNLOADED_RADIUS_MM", 0.0))
    _assert_adams_tire_geometry_matches(result_path, expected_radius_mm, "Fiala")
    source_model = build_adams_source_vehicle_model(data, tire_kind="fiala")
    model = build_native_rack_steering_model(source_model)
    case = _native_case(
        data,
        model,
        adams_rack_displacement_signal_from_result(result_path),
        tire_kind="fiala",
        end_time=end_time,
        output_step=step,
        internal_step=step,
        road_origin_z_m=adams_contact_patch_plane_height_m(result_path),
        source_drive_brake_result_path=result_path,
        adaptive_substepping=False,
    )

    def run_once(enabled: bool) -> tuple[float, float, Any]:
        previous = _with_profile_flag(enabled)
        try:
            started = perf_counter()
            result = run_vehicle_dynamics(model, case)
            wrapper_wall_s = perf_counter() - started
            return wrapper_wall_s, float(result.native_kernel_wall_time_s), result
        finally:
            _restore_profile_flag(previous)

    for _ in range(warmups):
        run_once(False)

    wrapper_wall_times = []
    kernel_wall_times = []
    last_result = None
    for _ in range(measured_runs):
        wrapper_elapsed, kernel_elapsed, last_result = run_once(False)
        wrapper_wall_times.append(wrapper_elapsed)
        kernel_wall_times.append(kernel_elapsed)

    profile_wrapper_elapsed = None
    profile_kernel_elapsed = None
    profile_performance = None
    if profile_run:
        profile_wrapper_elapsed, profile_kernel_elapsed, profiled = run_once(True)
        profile_performance = asdict(profiled.axle.performance)

    if last_result is None:
        raise RuntimeError("native measured run was not executed")
    adams = _adams_solver_elapsed(source_root)
    return {
        "case": "full_native_fiala_step_steer",
        "timing_contract": {
            "adams": "solver elapsed from Adams .msg, excluding Python wrapper and file copy",
            "native": "kernel wall time around the C++ vehicle_run call; Python assembly/wrapper time is reported separately",
        },
        "solver_settings": {
            "end_time_s": end_time,
            "step_size_s": step,
            "internal_step_size_s": step,
            "adaptive_substepping": False,
            "integration_error_tolerance": 1.0e-2,
        },
        "adams": adams,
        "native": {
            "kernel_wall_times_s": kernel_wall_times,
            "best_kernel_wall_s": float(np.min(kernel_wall_times)),
            "median_kernel_wall_s": float(np.median(kernel_wall_times)),
            "wrapper_wall_times_s": wrapper_wall_times,
            "best_wrapper_wall_s": float(np.min(wrapper_wall_times)),
            "median_wrapper_wall_s": float(np.median(wrapper_wall_times)),
            "sample_count": int(len(last_result.times_s)),
            "start_s": float(last_result.times_s[0]),
            "end_s": float(last_result.times_s[-1]),
            "profile_kernel_wall_s": profile_kernel_elapsed,
            "profile_wrapper_wall_s": profile_wrapper_elapsed,
            "performance": profile_performance,
        },
        "speed_ratio_native_best_vs_adams_elapsed": float(
            np.min(kernel_wall_times) / adams["solver_elapsed_s"]
        ),
        "passed_speed_gate": bool(
            np.min(kernel_wall_times) <= adams["solver_elapsed_s"]
        ),
    }


def main() -> None:
    """运行 native Fiala 计时测量."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path("artifacts/adams-full-source-fiala/step_steer"),
    )
    parser.add_argument(
        "--tire-property-file",
        type=Path,
        default=_default_fiala_tire_property(),
    )
    parser.add_argument("--end-time", type=float, default=5.0)
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--warmups", type=int, default=1)
    parser.add_argument("--measured-runs", type=int, default=3)
    parser.add_argument("--profile-run", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    payload = measure(
        args.source_root,
        args.tire_property_file,
        end_time=args.end_time,
        step=args.step,
        warmups=args.warmups,
        measured_runs=args.measured_runs,
        profile_run=args.profile_run,
    )
    text = json.dumps(payload, indent=2, ensure_ascii=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
