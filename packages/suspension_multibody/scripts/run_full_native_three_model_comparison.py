"""生成完整 Native 多体与 Adams PAC2002 的等价坐标对比历史."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from suspension_multibody.adams import (
    adams_contact_patch_plane_height_m,
    adams_rack_displacement_signal_from_result,
    build_adams_vehicle_case,
    build_native_rack_steering_model,
    load_adams_full_vehicle_input,
)
from suspension_multibody.adams.full_vehicle_correlation import (
    full_vehicle_time_history,
)
from suspension_multibody.adams.full_vehicle_model import (
    build_adams_source_vehicle_model,
)
from suspension_multibody.adams.time_domain import (
    AdamsResultChannel,
    TimeHistory,
    parse_adams_result_history,
    read_time_history,
)
from suspension_multibody.axle_dynamics import NativeAxleError
from suspension_multibody.schema import UnitSystem
from suspension_multibody.vehicle_dynamics import (
    run_vehicle_dynamics,
    write_vehicle_dynamics_artifact,
)

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
TIRE_OUTPUT_COLUMNS = {
    "normal_force": 4,
    "longitudinal_force": 5,
    "lateral_force": 6,
}
ADAMS_TIRE_CHANNELS = {
    f"{wheel}.tire_{force}": AdamsResultChannel(
        f"{prefix}_wheel_tire_forces", f"{component}_{axle}"
    )
    for wheel, prefix, axle in (
        ("front_left", "til", "front"),
        ("front_right", "tir", "front"),
        ("rear_left", "til", "rear"),
        ("rear_right", "tir", "rear"),
    )
    for force, component in (
        ("normal_force", "normal"),
        ("longitudinal_force", "longitudinal"),
        ("lateral_force", "lateral"),
    )
}


def _length_scale(units: UnitSystem) -> float:
    """返回模型长度单位到米的换算系数."""
    return 1.0e-3 if units == UnitSystem.ENGINEERING else 1.0


def _write_history(
    path: Path, history: TimeHistory, metadata: dict[str, object]
) -> None:
    payload = history.as_dict()
    payload["metadata"] = metadata
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def _truncate_history(history: TimeHistory, end_time: float) -> TimeHistory:
    """截取 Adams 原始采样前缀，保持公共时间网格和原始采样值不变."""
    if end_time < history.time[0] or end_time > history.time[-1]:
        raise ValueError("对比终止时间超出 Adams 原始历史范围")
    index = min(
        range(len(history.time)),
        key=lambda value: abs(history.time[value] - end_time),
    )
    if not np.isclose(history.time[index], end_time, atol=1.0e-12):
        raise ValueError("对比终止时间必须落在 Adams 原始采样网格上")
    stop = index + 1
    return TimeHistory(
        time=history.time[:stop],
        channels={name: values[:stop] for name, values in history.channels.items()},
        units=history.units,
    )


def _adams_tire_history(result_path: Path) -> TimeHistory:
    return parse_adams_result_history(
        result_path,
        ADAMS_TIRE_CHANNELS,
        units={name: "N" for name in ADAMS_TIRE_CHANNELS},
    )


def _native_tire_history(result: Any) -> TimeHistory:
    """提取与 Adams wheel_tire_forces 等价的轮胎 ISO 输出."""
    output = np.asarray(result.axle.tire_output, dtype=float)
    if output.shape[:2] != (len(result.times_s), len(result.tire_names)):
        raise ValueError(f"native tire_output shape invalid: {output.shape}")
    channels: dict[str, tuple[float, ...]] = {}
    for tire_index, wheel in enumerate(result.tire_names):
        if wheel not in WHEELS:
            raise ValueError(f"unexpected native wheel name: {wheel}")
        for force, column in TIRE_OUTPUT_COLUMNS.items():
            channels[f"{wheel}.tire_{force}"] = tuple(
                float(value) for value in output[:, tire_index, column]
            )
    return TimeHistory(
        time=tuple(float(value) for value in result.times_s),
        channels=channels,
        units={name: "N" for name in channels},
    )


def _relative_body_roll(history: TimeHistory) -> TimeHistory:
    """Apply the shared handling contract for the body-roll response."""
    channels = dict(history.channels)
    body_roll = channels.get("body_roll")
    if body_roll is None:
        raise ValueError("操稳历史缺少 body_roll 通道")
    channels["body_roll"] = tuple(value - body_roll[0] for value in body_roll)
    return TimeHistory(time=history.time, channels=channels, units=history.units)


def _native_handling_history(result: Any, case: Any) -> TimeHistory:
    length_scale = _length_scale(case.vehicle.units)
    steering_ratio = (
        case.vehicle.steering.rack_displacement_per_steering_wheel_angle
        or case.vehicle.steering.ratio
    )
    history = full_vehicle_time_history(
        result,
        "handling_stability",
        steering_ratio_m_per_rad=steering_ratio * length_scale,
        chassis_center_of_mass_m=tuple(
            value * length_scale
            for value in case.vehicle.chassis.center_of_mass.as_tuple()
        ),
    )
    return _relative_body_roll(history)


def _native_case(
    data: Any,
    model: Any,
    steering_input: Any,
    *,
    tire_kind: str,
    end_time: float,
    output_step: float,
    internal_step: float,
    road_origin_z_m: float,
    source_drive_brake_result_path: Path | None = None,
) -> Any:
    case = build_adams_vehicle_case(
        data,
        model,
        case_name=f"step_steer_full_native_{tire_kind}",
        steering_input=steering_input,
        end_time=end_time,
        step_size=output_step,
        source_drive_brake_result_path=source_drive_brake_result_path,
    )
    solver = case.solver.model_copy(
        update={
            # Adams reports a 10 ms integration step at the settled run and
            # Integration error = 1e-2.  Keep the Native comparison on that
            # same fixed step; the Newton/constraint tolerances remain the
            # stricter physical convergence gate.
            # Adams 的外层步长和 Integration error 保持一致；Fiala 的
            # 内部松弛状态使用已验证的固定细分，避免把不同的步长加倍
            # 误差估计器混入模型精度比较。
            "adaptive_substepping": False,
            "step_size": output_step,
            "internal_step_size": internal_step,
            # Adams 在 Fiala 首步会自动细分；Native 保持相同外层步长，
            # 允许内部步长下降到已验证的接触事件分辨率。
            "min_internal_step_size": min(internal_step, 5.0e-4)
            if tire_kind == "fiala"
            else internal_step,
            "integration_error_tolerance": 1.0e-2,
        }
    )
    length_scale = _length_scale(case.vehicle.units)
    return case.model_copy(
        update={
            "solver": solver,
            "road": case.road.model_copy(
                update={
                    "origin": case.road.origin.model_copy(
                        update={"z": road_origin_z_m / length_scale}
                    )
                }
            ),
        }
    )


def generate(
    source_root: Path,
    output_root: Path,
    *,
    end_time: float,
    output_step: float,
    internal_step: float,
    tire_kinds: tuple[str, ...] = ("native_brush", "pac2002"),
    tire_property_file: Path | None = None,
) -> Path:
    """使用同一 Adams 初始状态和输入生成指定 Native 轮胎历史."""
    source_root = source_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    data = load_adams_full_vehicle_input(
        source_root, tire_property_file=tire_property_file
    )
    adams_result_path = source_root / "adams_raw" / "handling_step_steer_dynamic.res"
    road_origin_z_m = adams_contact_patch_plane_height_m(adams_result_path)
    adams_tire = _adams_tire_history(adams_result_path)
    adams_handling = _relative_body_roll(
        read_time_history(source_root / "adams_time_history.json")
    )
    if adams_tire.time != adams_handling.time:
        raise ValueError("Adams 轮胎力和操稳历史时间网格不一致")
    adams_tire = _truncate_history(adams_tire, end_time)
    adams_handling = _truncate_history(adams_handling, end_time)
    _write_history(
        output_root / f"adams_{'fiala' if tire_property_file is not None else 'pac2002'}_time_history.json",
        TimeHistory(
            time=adams_tire.time,
            channels={**adams_tire.channels, **adams_handling.channels},
            units={**(adams_tire.units or {}), **(adams_handling.units or {})},
        ),
        {
            "model": "adams Fiala" if tire_property_file is not None else "adams PAC2002",
            "model_kind": "adams_fiala" if tire_property_file is not None else "adams_pac2002",
            "source_result": str(adams_result_path),
            "handling_source_history": str(source_root / "adams_time_history.json"),
            "tire_channel_map": {
                name: {
                    "entity": channel.entity,
                    "component": channel.component,
                }
                for name, channel in ADAMS_TIRE_CHANNELS.items()
            },
            "response_transform": {"body_roll": "subtract_initial_sample"},
            "road_origin_z_m": road_origin_z_m,
            "native_steering_input": "prescribed_adams_rack_displacement",
            "complete_vehicle_reference": True,
        },
    )
    native_models: dict[str, object] = {}
    for tire_kind in tire_kinds:
        source_model = build_adams_source_vehicle_model(data, tire_kind=tire_kind)
        model = build_native_rack_steering_model(source_model)
        rack_steering = adams_rack_displacement_signal_from_result(adams_result_path)
        case = _native_case(
            data,
            model,
            rack_steering,
            tire_kind=tire_kind,
            end_time=end_time,
            output_step=output_step,
            internal_step=internal_step,
            road_origin_z_m=road_origin_z_m,
            source_drive_brake_result_path=adams_result_path,
        )
        try:
            result = run_vehicle_dynamics(model, case)
        except NativeAxleError as exc:
            write_vehicle_dynamics_artifact(
                None,
                model,
                case,
                output_root / f"native_{tire_kind}_artifact",
                failure=exc,
            )
            raise
        if not bool(np.all(result.diagnostics.accepted)):
            raise RuntimeError(f"完整 native {tire_kind} 运行存在未接受采样")
        native_tire = _native_tire_history(result)
        native_handling = _native_handling_history(result, case)
        if native_tire.time != native_handling.time:
            raise ValueError(f"native {tire_kind} 轮胎力和操稳时间网格不一致")
        history = TimeHistory(
            time=native_tire.time,
            channels={**native_tire.channels, **native_handling.channels},
            units={**(native_tire.units or {}), **(native_handling.units or {})},
        )
        history_path = output_root / f"native_{tire_kind}_time_history.json"
        _write_history(
            history_path,
            history,
            {
                "model": f"native {tire_kind}",
                "model_kind": tire_kind,
                "complete_vehicle_reference": True,
                "source_adams_case": str(source_root),
                "solver_integrator": case.solver.integrator,
                "output_step_s": output_step,
                "internal_step_s": internal_step,
                "road_origin_z_m": road_origin_z_m,
                "steering_input": "prescribed_adams_rack_displacement",
                "wheel_torque_input": "direct_adams_drive_brake_replay",
            "tire_force_coordinates": "adams_fiala_or_pac2002_tire_iso_output",
            },
        )
        manifest_path = write_vehicle_dynamics_artifact(
            result, model, case, output_root / f"native_{tire_kind}_artifact"
        )
        native_models[tire_kind] = {
            "history": str(history_path),
            "artifact_manifest": str(manifest_path),
            "sample_count": len(history.time),
        }
    manifest_path = output_root / "comparison_manifest.json"
    manifest_path.write_text(
        json.dumps(
            {
                "contract": "full-native-model-comparison-v2",
                "case": "step_steer",
                "source_adams_case": str(source_root),
                "adams_result": str(adams_result_path),
                "adams_tire_channel_map": {
                    name: {
                        "entity": channel.entity,
                        "component": channel.component,
                    }
                    for name, channel in ADAMS_TIRE_CHANNELS.items()
                },
                "response_transform": {"body_roll": "subtract_initial_sample"},
                "road_origin_z_m": road_origin_z_m,
                "native_steering_input": "prescribed_adams_rack_displacement",
                "native_wheel_torque_input": "direct_adams_drive_brake_replay",
                "tire_force_coordinates": "adams_fiala_or_pac2002_tire_iso_output",
                "matched_solver_settings": {
                    "adams_reported_step_size_s": 1.0e-2,
                    "adams_integration_error_tolerance": 1.0e-2,
                    "native_adaptive_substepping": False,
                    "native_step_size_s": output_step,
                    "native_internal_step_size_s": internal_step,
                    "native_min_internal_step_size_s": min(
                        internal_step, 5.0e-4
                    ) if "fiala" in native_models else internal_step,
                    "native_integration_error_tolerance": 1.0e-2,
                    "same_external_step_and_tolerance": (
                        abs(output_step - 1.0e-2) <= 1.0e-12
                    ),
                    "same_step_and_tolerance": (
                        abs(output_step - 1.0e-2) <= 1.0e-12
                        and abs(internal_step - 1.0e-2) <= 1.0e-12
                    ),
                },
                "time_grid": {
                    "start_s": adams_tire.time[0],
                    "end_s": adams_tire.time[-1],
                    "sample_count": len(adams_tire.time),
                    "step_s": float(np.median(np.diff(adams_tire.time))),
                },
                "models": {
                    "adams_fiala" if tire_property_file is not None else "adams_pac2002": {
                        "history": str(output_root / f"adams_{'fiala' if tire_property_file is not None else 'pac2002'}_time_history.json"),
                        "complete_vehicle_reference": True,
                    },
                    **(
                        {"native_brush": native_models["native_brush"]}
                        if "native_brush" in native_models
                        else {}
                    ),
                    **(
                        {"native_pac2002": native_models["pac2002"]}
                        if "pac2002" in native_models
                        else {}
                    ),
                    **(
                        {"native_fiala": native_models["fiala"]}
                        if "fiala" in native_models
                        else {}
                    ),
                },
                "same_initial_state_and_inputs": True,
                "native_solver": "run_vehicle_dynamics",
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    return manifest_path


def main() -> None:
    """解析命令行参数并生成对比产物."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source-root",
        type=Path,
        default=Path("artifacts/adams-full-source/step_steer"),
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=Path("artifacts/visuals/full-native-three-model-step-steer/step_steer"),
    )
    parser.add_argument("--end-time", type=float, default=5.0)
    parser.add_argument("--output-step", type=float, default=0.01)
    parser.add_argument("--internal-step", type=float, default=0.01)
    parser.add_argument(
        "--pac-only",
        action="store_true",
        help="只运行完整 Native PAC2002，不运行 Brush",
    )
    parser.add_argument("--fiala-only", action="store_true")
    parser.add_argument("--tire-property-file", type=Path)
    args = parser.parse_args()
    print(
        generate(
            args.source_root,
            args.output_root,
            end_time=args.end_time,
            output_step=args.output_step,
            internal_step=args.internal_step,
            tire_kinds=("fiala",)
            if args.fiala_only
            else ("pac2002",)
            if args.pac_only
            else ("native_brush", "pac2002"),
            tire_property_file=args.tire_property_file,
        )
    )


if __name__ == "__main__":
    main()
