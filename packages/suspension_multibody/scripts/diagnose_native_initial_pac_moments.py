"""隔离 PAC2002 轮胎力矩对 Native 初始平衡的影响."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from run_full_native_three_model_comparison import _native_case
from suspension_multibody.adams import (
    adams_contact_patch_plane_height_m,
    adams_rack_displacement_signal_from_result,
    build_native_rack_steering_model,
    load_adams_full_vehicle_input,
)
from suspension_multibody.adams.full_vehicle_model import (
    build_adams_source_vehicle_model,
)
from suspension_multibody.vehicle_dynamics import run_vehicle_dynamics

MOMENT_PREFIXES = ("QSX", "QSY", "QDZ", "SSZ")


def _without_pac_moments(model: Any) -> Any:
    wheels = []
    for wheel in model.wheels:
        coefficients = {
            name: (0.0 if name.startswith(MOMENT_PREFIXES) else value)
            for name, value in wheel.tire.pac2002_coefficients.items()
        }
        wheels.append(
            wheel.model_copy(
                update={
                    "tire": wheel.tire.model_copy(
                        update={"pac2002_coefficients": coefficients}
                    )
                }
            )
        )
    return model.model_copy(update={"wheels": tuple(wheels)})


def _run_variant(data: Any, model: Any, result_path: Path, road_z: float) -> dict[str, Any]:
    rack = adams_rack_displacement_signal_from_result(result_path)
    case = _native_case(
        data,
        model,
        rack,
        tire_kind="pac2002",
        end_time=0.01,
        output_step=0.01,
        internal_step=2.5e-4,
        road_origin_z_m=road_z,
        source_drive_brake_result_path=result_path,
    )
    result = run_vehicle_dynamics(model, case)
    chassis = result.body_state("chassis")
    return {
        "linear_acceleration_m_per_s2": chassis[0, 13:16].tolist(),
        "angular_acceleration_rad_per_s2": chassis[0, 16:19].tolist(),
        "tire_force_n": result.axle.tire_output[0, :, 4:7].tolist(),
        "tire_moment_n_m": result.axle.tire_output[0, :, 12:15].tolist(),
    }


def diagnose(source_root: Path) -> dict[str, Any]:
    data = load_adams_full_vehicle_input(source_root)
    source = build_adams_source_vehicle_model(data, tire_kind="pac2002")
    baseline = build_native_rack_steering_model(source)
    no_moments = _without_pac_moments(baseline)
    result_path = source_root / "adams_raw" / "handling_step_steer_dynamic.res"
    road_z = adams_contact_patch_plane_height_m(result_path)
    return {
        "baseline": _run_variant(data, baseline, result_path, road_z),
        "pac_moments_disabled": _run_variant(data, no_moments, result_path, road_z),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = diagnose(args.source_root.resolve())
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
