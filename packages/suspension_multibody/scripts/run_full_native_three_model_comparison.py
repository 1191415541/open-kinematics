"""生成完整 Native 多体与 Adams PAC2002 的等价坐标对比历史."""

from __future__ import annotations

import argparse
import json
import os
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
    # The patch's slide velocity.  It is what the slip angle is computed from, so it
    # is how "do the front wheels actually steer?" can be answered from a history
    # instead of assumed from the steering-wheel channel.
    "longitudinal_slip_velocity_m_per_s": 7,
    "lateral_slip_velocity_m_per_s": 8,
    # The tire aligning moment (Mz, ISO).  Modes below 25 are not gated on it yet,
    # but USE_MODE 25 is the parking-torque mode and that torque lives here: on the
    # low-speed parking maneuver the front wheels' aligning torque grows by a factor
    # of ten between mode 24 and mode 25, while the unsteered rear wheels move by
    # ~4 % (tasks/D/raw/compare_parking_modes.py).
    "aligning_moment": 14,
    # The overturning moment, for the camber-vs-force-law question above.
    "overturning_moment": 12,
    # The contact-body states of the advanced transient modes.  Adams has no
    # comparable request channel in this assembly, so these are native-only columns:
    # they exist so a comparison can *show* the second layer moving instead of
    # inferring it from the force difference, and the diagnostic must stay out of
    # them (they are not gated).
    "contact_body_longitudinal_m": 15,
    "contact_body_longitudinal_rate_m_per_s": 16,
    "contact_body_lateral_m": 17,
    "contact_body_lateral_rate_m_per_s": 18,
    "contact_body_yaw_rad": 19,
    "contact_body_yaw_rate_rad_per_s": 20,
    # Turn-slip relaxation states of USE_MODE 25 (native-only, like the contact-body
    # columns above): they are what the parking torque is built from, so a comparison
    # can show the filter moving instead of inferring it from the moment.
    "turn_slip_phi_c_rad_per_m": 21,
    "turn_slip_phi_f2_rad_per_m": 22,
    "turn_slip_phi_1_rad_per_m": 23,
    "turn_slip_phi_2_rad_per_m": 24,
    # Native-only Eq3961 / turn-slip decomposition columns for parking parity.
    "rolling_speed_m_per_s": 25,
    "slip_reference_speed_m_per_s": 26,
    "lateral_slip_base_rad": 27,
    "lateral_slip_beta_term_rad": 28,
    "lateral_slip_beta_st_term_rad": 29,
    "lateral_slip_target_rad": 30,
    "lateral_slip_target_clamped_rad": 31,
    "turn_slip_force_rad_per_m": 32,
    "turn_slip_moment_rad_per_m": 33,
    "turn_slip_drive_rad_per_s": 34,
    "turn_slip_yaw_rate_rad_per_s": 35,
    "turn_slip_camber_term_rad_per_s": 36,
    "turn_slip_total_spin_rate_rad_per_s": 37,
    "lateral_slip_relaxation_length_m": 38,
    # What the Magic Formula actually evaluated: the relaxed slip state of
    # Eq3960-Eq3962 after the clamp, as opposed to the instantaneous target at 30/31.
    "longitudinal_relaxed_slip": 39,
    "lateral_relaxed_slip": 40,
}
# Adams reports the aligning torque in the result file's torque unit (N*mm for these
# assemblies) while the native tire output is in N*m, so the parsed channel has to be
# scaled before the two can be compared.
ADAMS_CHANNEL_SCALE = {
    "tire_aligning_moment": 1.0e-3,
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
        ("aligning_moment", "aligning_torque"),
        # The overturning moment is dominated by the camber, so it is the cheap way
        # to tell "the camber is wrong" from "the lateral force law is wrong" when a
        # per-wheel lateral channel disagrees.
        ("overturning_moment", "overturning_moment"),
    )
}
ADAMS_ROLLING_STATE_CHANNELS = {
    f"{wheel}.{quantity}": AdamsResultChannel(
        f"{prefix}_wheel_tire_rolling_states", f"{component}_{axle}"
    )
    for wheel, prefix, axle in (
        ("front_left", "til", "front"),
        ("front_right", "tir", "front"),
        ("rear_left", "til", "rear"),
        ("rear_right", "tir", "rear"),
    )
    for quantity, component in (
        ("deflection", "tire_deflection"),
        ("loaded_radius", "loaded_radius"),
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
    history = parse_adams_result_history(
        result_path,
        ADAMS_TIRE_CHANNELS,
        units={name: "N" for name in ADAMS_TIRE_CHANNELS},
    )
    channels = {
        name: tuple(
            value*ADAMS_CHANNEL_SCALE.get(name.rsplit(".", 1)[-1], 1.0)
            for value in values
        )
        for name, values in history.channels.items()
    }
    return TimeHistory(time=history.time, channels=channels, units=history.units)


def _assert_adams_tire_geometry_matches(
    result_path: Path, expected_radius_mm: float, tire_label: str
) -> None:
    """防止把不同轮胎几何的 Adams 结果误用为当前轮胎基准。 ."""
    rolling = parse_adams_result_history(
        result_path,
        ADAMS_ROLLING_STATE_CHANNELS,
        units={name: "mm" for name in ADAMS_ROLLING_STATE_CHANNELS},
    )
    mismatches: list[str] = []
    for wheel in WHEELS:
        radius_mm = (
            rolling.channels[f"{wheel}.deflection"][0]
            + rolling.channels[f"{wheel}.loaded_radius"][0]
        )
        if abs(radius_mm - expected_radius_mm) > 1.0e-3:
            mismatches.append(
                f"{wheel}: Adams={radius_mm:.6g} mm, expected={expected_radius_mm:.6g} mm"
            )
    if mismatches:
        raise ValueError(
            f"Adams {tire_label} 结果与 Native 轮胎几何不一致；"
            "不能作为同条件基准。" + "；".join(mismatches)
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
    adaptive_substepping: bool = False,
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
            # Integration error = 1e-2.  Keep the default Native comparison on
            # that same fixed step; the Newton/constraint tolerances remain the
            # stricter physical convergence gate.
            "adaptive_substepping": adaptive_substepping,
            "step_size": output_step,
            "internal_step_size": internal_step,
            # The floor has to be *below* the nominal step or the solver cannot
            # reduce one that it cannot resolve.  PAC2002 used to declare the floor
            # equal to the step, which made every step rigid: the advanced
            # transient modes then had nowhere to go when a step landed on a kink
            # in the force law and aborted the whole run.  A run that converges
            # never reduces, so its step sequence -- and its results -- are
            # unchanged; only a step that would otherwise be fatal gets halved.
            "min_internal_step_size": min(internal_step, 1.0e-4),
            "integration_error_tolerance": 1.0e-2,
            **(
                {"projection_backtracking": int(backtracking)}
                if (backtracking := os.environ.get(
                    "SUSPENSION_NATIVE_PROJECTION_BACKTRACKING"
                ))
                else {}
            ),
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
    pac2002_tire_file: Path | None = None,
    adaptive_substepping: bool = False,
) -> Path:
    """使用同一 Adams 初始状态和输入生成指定 Native 轮胎历史."""
    source_root = source_root.resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    # ``tire_property_file`` means "solve with Adams Fiala"; ``pac2002_tire_file``
    # only chooses which PAC2002 tire the native model reads.  They are separate
    # because the reference case's tire need not be the stock one -- the
    # advanced-transient references in artifacts/adams-mode-ref are built from the
    # parking tire, which carries the contact-mass coefficients.
    data = load_adams_full_vehicle_input(
        source_root,
        tire_property_file=tire_property_file or pac2002_tire_file,
    )
    # The maneuver is not always the step steer: the straight-line acceleration case
    # writes handling_acceleration_dynamic.res.  Find whatever the case produced
    # rather than assuming the stem.
    candidates = sorted(
        (source_root / "adams_raw").glob("handling_*_dynamic.res")
    )
    if len(candidates) != 1:
        raise ValueError(
            "expected exactly one Adams dynamic result in "
            f"{source_root / 'adams_raw'}, found "
            f"{[path.name for path in candidates]}"
        )
    adams_result_path = candidates[0]
    # Only an explicit Fiala request switches the parameter set; overriding the
    # PAC2002 tire must still be checked as a PAC2002 tire.
    if tire_property_file is not None:
        expected_radius_mm = float(data.fiala_parameters.get("UNLOADED_RADIUS_MM", 0.0))
        tire_label = "Fiala"
    else:
        expected_radius_mm = float(data.pac2002_coefficients.get("UNLOADED_RADIUS_MM", 0.0))
        tire_label = "PAC2002"
    if expected_radius_mm <= 0.0:
        raise ValueError(f"Native {tire_label} 轮胎缺少有效 UNLOADED_RADIUS_MM")
    _assert_adams_tire_geometry_matches(
        adams_result_path, expected_radius_mm, tire_label
    )
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
            adaptive_substepping=adaptive_substepping,
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
                # One history per tire kind, so the tag names the coordinates of
                # that kind instead of the generic either/or form the Adams
                # reference bundle uses.
                "tire_force_coordinates": f"{tire_kind}_tire_iso_output",
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
                "tire_force_coordinates": (
                    "pac2002_tire_iso_output"
                    if "pac2002" in native_models
                    else "fiala_tire_iso_output"
                ),
                "matched_solver_settings": {
                    "adams_reported_step_size_s": 1.0e-2,
                    "adams_integration_error_tolerance": 1.0e-2,
                    "native_adaptive_substepping": adaptive_substepping,
                    "native_step_size_s": output_step,
                    "native_internal_step_size_s": internal_step,
                    "native_min_internal_step_size_s": min(
                        internal_step, 1.0e-4
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
        default=Path("artifacts/adams-full-source-2025_1_1/step_steer"),
        help=(
            "Adams reference case. Defaults to the case regenerated with the "
            "installed Adams 2025.1.1; the older artifacts/adams-full-source "
            "case was produced by Adams 2024.1 and yields bit-identical native "
            "results, so either can be used."
        ),
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
    parser.add_argument(
        "--pac2002-tire-file",
        type=Path,
        help=(
            "Override the PAC2002 tire the native model uses. Needed when the "
            "Adams reference was produced with a different tire than the stock "
            "pac2002_235_60R16.tir: the comparison rejects mismatched tire "
            "geometry, correctly, because it would otherwise report a tire "
            "difference as model error."
        ),
    )
    parser.add_argument(
        "--fixed-inner-step",
        action="store_true",
        help="关闭内步误差控制，用于固定 10 ms 性能基准",
    )
    parser.add_argument(
        "--adaptive-substepping",
        action="store_true",
        help="打开内步误差控制，用于诊断收敛余量",
    )
    args = parser.parse_args()
    print(
        generate(
            args.source_root,
            args.output_root,
            end_time=args.end_time,
            output_step=args.output_step,
            internal_step=args.internal_step,
            adaptive_substepping=(args.adaptive_substepping and not args.fixed_inner_step),
            tire_kinds=("fiala",)
            if args.fiala_only
            else ("pac2002",)
            if args.pac_only
            else ("native_brush", "pac2002"),
            tire_property_file=args.tire_property_file,
            pac2002_tire_file=args.pac2002_tire_file,
        )
    )


if __name__ == "__main__":
    main()
