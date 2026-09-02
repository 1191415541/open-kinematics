"""定位完整 Native PAC2002 与 Adams PAC2002 的首次状态分离."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from suspension_multibody.adams.time_domain import (
    AdamsResultChannel,
    parse_adams_result_history,
)

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
TIRE_COLUMNS = {
    "normal_force_n": 4,
    "longitudinal_force_n": 5,
    "lateral_force_n": 6,
    "longitudinal_slip_velocity_m_per_s": 7,
    "lateral_slip_velocity_m_per_s": 8,
    "effective_longitudinal_slip": 10,
    "effective_lateral_slip_rad": 11,
}
THRESHOLDS = {
    "normal_force_n": 50.0,
    "longitudinal_force_n": 50.0,
    "lateral_force_n": 50.0,
    "instantaneous_longitudinal_slip": 5.0e-4,
    "effective_longitudinal_slip": 5.0e-4,
    "instantaneous_lateral_slip_rad": 5.0e-4,
    "effective_lateral_slip_rad": 5.0e-4,
    "rolling_radius_m": 5.0e-4,
    "rotational_velocity_rad_per_s": 5.0e-2,
}
COMPARABLE_THRESHOLDS = {
    name: threshold
    for name, threshold in THRESHOLDS.items()
    if not name.startswith("effective_")
}


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON 根节点必须是对象: {path}")
    return payload


def _resolve_path(value: str, root: Path) -> Path:
    path = Path(value)
    if path.is_absolute():
        return path
    workspace_path = Path.cwd() / path
    return workspace_path if workspace_path.exists() else root / path


def _history(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    payload = _read_json(path)
    time = np.asarray(payload.get("time"), dtype=float)
    raw_channels = payload.get("channels")
    if time.ndim != 1 or not isinstance(raw_channels, dict):
        raise ValueError(f"无效时域历史: {path}")
    channels = {
        str(name): np.asarray(values, dtype=float)
        for name, values in raw_channels.items()
    }
    return time, channels


def _adams_state_channels() -> dict[str, AdamsResultChannel]:
    channels: dict[str, AdamsResultChannel] = {}
    for wheel, prefix, axle in (
        ("front_left", "til", "front"),
        ("front_right", "tir", "front"),
        ("rear_left", "til", "rear"),
        ("rear_right", "tir", "rear"),
    ):
        kinematics = f"{prefix}_wheel_tire_kinematics"
        rolling = f"{prefix}_wheel_tire_rolling_states"
        channels[f"{wheel}.longitudinal_slip"] = AdamsResultChannel(
            kinematics, f"longitudinal_slip_{axle}"
        )
        channels[f"{wheel}.lateral_slip_rad"] = AdamsResultChannel(
            kinematics, f"lateral_slip_{axle}"
        )
        channels[f"{wheel}.rolling_radius_m"] = AdamsResultChannel(
            rolling, f"rolling_radius_{axle}"
        )
        channels[f"{wheel}.rotational_velocity_rad_per_s"] = AdamsResultChannel(
            rolling, f"rotational_velocity_{axle}"
        )
    return channels


def _first_exceedance(
    time: np.ndarray, reference: np.ndarray, actual: np.ndarray, threshold: float
) -> dict[str, float | None]:
    error = np.abs(actual - reference)
    indices = np.flatnonzero(error > threshold)
    index = int(indices[0]) if indices.size else None
    return {
        "threshold": threshold,
        "first_time_s": None if index is None else float(time[index]),
        "first_absolute_error": None if index is None else float(error[index]),
        "maximum_absolute_error": float(np.max(error)),
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
    }


def _channel_metrics(reference: np.ndarray, actual: np.ndarray) -> dict[str, float]:
    error = actual - reference
    reference_centered = reference - float(np.mean(reference))
    actual_centered = actual - float(np.mean(actual))
    correlation_scale = float(
        np.linalg.norm(reference_centered) * np.linalg.norm(actual_centered)
    )
    rms_reference = float(np.sqrt(np.mean(np.square(reference))))
    return {
        "nrmse_percent": 100.0
        * float(np.sqrt(np.mean(np.square(error))))
        / max(rms_reference, np.finfo(float).eps),
        "maximum_absolute_error": float(np.max(np.abs(error))),
        "correlation": (
            float(reference_centered @ actual_centered) / correlation_scale
            if correlation_scale > 0.0
            else 0.0
        ),
    }


def _quaternion_rotation(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _wheel_body_name(wheel: str, spec: dict[str, Any], body_names: list[str]) -> str:
    candidates = (
        str(spec.get("body", "")),
        f"{wheel.split('_', 1)[0]}_{spec.get('mount_body', '')}",
    )
    for candidate in candidates:
        if candidate in body_names:
            return candidate
    raise ValueError(f"无法确定 {wheel} 对应的 Native 轮体: {candidates}")


def _native_rolling_states(
    wheel: str,
    spec: dict[str, Any],
    body_names: list[str],
    states: np.ndarray,
    tire_output: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    body_index = body_names.index(_wheel_body_name(wheel, spec, body_names))
    angular_velocity = states[:, body_index, 10:13]
    rotational_velocity = np.linalg.norm(angular_velocity, axis=1)

    tire = spec["tire"]
    coefficients = tire["pac2002_coefficients"]
    scale = 1.0e-3 if float(tire["unloaded_radius"]) > 2.0 else 1.0
    radius = float(tire["unloaded_radius"]) * scale
    stiffness = float(tire["vertical_stiffness"]) / scale
    nominal_load = max(float(coefficients.get("FNOMIN", 4850.0)), 1.0e-9)
    nominal_deflection = nominal_load / max(stiffness, 1.0e-9)
    normalized_deflection = np.maximum(tire_output[:, 2], 0.0) / max(
        nominal_deflection, 1.0e-9
    )
    speed_reference = max(float(coefficients.get("LONGVL", 16.6)), 1.0e-9)
    speed_growth = (
        float(coefficients.get("QV1", 0.0))
        * radius
        * np.square(rotational_velocity * radius / speed_reference)
    )
    correction = nominal_deflection * (
        float(coefficients.get("DREFF", 0.27))
        * np.arctan(float(coefficients.get("BREFF", 8.4)) * normalized_deflection)
        + float(coefficients.get("FREFF", 0.07)) * normalized_deflection
    )
    rolling_radius = np.maximum(
        radius * float(coefficients.get("QRE0", 1.0)) + speed_growth - correction,
        1.0e-9,
    )
    return rotational_velocity, rolling_radius


def diagnose(comparison_root: Path) -> dict[str, Any]:
    """读取同输入产物并返回逐状态首次越限时刻."""
    root = comparison_root.resolve()
    manifest = _read_json(root / "comparison_manifest.json")
    if manifest.get("same_initial_state_and_inputs") is not True:
        raise ValueError("对比产物不满足相同初始状态和输入契约")
    native_model = manifest["models"]["native_pac2002"]
    artifact_manifest_path = _resolve_path(native_model["artifact_manifest"], root)
    artifact_manifest = _read_json(artifact_manifest_path)
    arrays_path = artifact_manifest_path.parent / artifact_manifest["arrays_file"]
    arrays = np.load(arrays_path)

    time = np.asarray(arrays["times_s"], dtype=float)
    body_names = [str(value) for value in arrays["body_names"]]
    tire_names = [str(value) for value in arrays["tire_names"]]
    states = np.asarray(arrays["states"], dtype=float)
    tire_output = np.asarray(arrays["tire_output"], dtype=float)
    if tuple(tire_names) != WHEELS:
        raise ValueError(f"Native 轮胎顺序不符合四轮契约: {tire_names}")

    adams_result = _resolve_path(str(manifest["adams_result"]), root)
    adams_history = parse_adams_result_history(
        adams_result, _adams_state_channels()
    )
    adams_time = np.asarray(adams_history.time[: time.size], dtype=float)
    if time.shape != adams_time.shape or not np.allclose(
        time, adams_time, rtol=0.0, atol=1.0e-12
    ):
        raise ValueError("Native 与 Adams 状态历史不在同一时间网格")

    adams_force_time, adams_force = _history(
        _resolve_path(manifest["models"]["adams_pac2002"]["history"], root)
    )
    native_force_time, native_force = _history(
        _resolve_path(native_model["history"], root)
    )
    if (
        adams_force_time.shape != native_force_time.shape
        or not np.allclose(
            adams_force_time, native_force_time, rtol=0.0, atol=1.0e-12
        )
        or not np.allclose(time, adams_force_time, rtol=0.0, atol=1.0e-12)
    ):
        raise ValueError("轮胎力历史与状态历史不在同一时间网格")

    specs = {str(item["name"]): item for item in artifact_manifest["model"]["wheels"]}
    by_wheel: dict[str, Any] = {}
    events: list[dict[str, Any]] = []
    for tire_index, wheel in enumerate(WHEELS):
        native_tire = tire_output[:, tire_index, :]
        rotational_velocity, rolling_radius = _native_rolling_states(
            wheel, specs[wheel], body_names, states, native_tire
        )
        rolling_speed = np.maximum(
            np.abs(
                native_tire[:, TIRE_COLUMNS["longitudinal_slip_velocity_m_per_s"]]
                + rotational_velocity * rolling_radius
            ),
            1.0e-3,
        )
        native_signals = {
            "instantaneous_longitudinal_slip": -native_tire[
                :, TIRE_COLUMNS["longitudinal_slip_velocity_m_per_s"]
            ]
            / rolling_speed,
            "effective_longitudinal_slip": native_tire[
                :, TIRE_COLUMNS["effective_longitudinal_slip"]
            ],
            "instantaneous_lateral_slip_rad": np.arctan2(
                native_tire[:, TIRE_COLUMNS["lateral_slip_velocity_m_per_s"]],
                rolling_speed,
            ),
            "effective_lateral_slip_rad": native_tire[
                :, TIRE_COLUMNS["effective_lateral_slip_rad"]
            ],
            "rolling_radius_m": rolling_radius,
            "rotational_velocity_rad_per_s": rotational_velocity,
            "normal_force_n": native_force[f"{wheel}.tire_normal_force"],
            "longitudinal_force_n": native_force[f"{wheel}.tire_longitudinal_force"],
            "lateral_force_n": native_force[f"{wheel}.tire_lateral_force"],
        }
        adams_longitudinal = (
            np.asarray(
                adams_history.channels[f"{wheel}.longitudinal_slip"][: time.size]
            )
            * 0.01
        )
        reference_signals = {
            "instantaneous_longitudinal_slip": adams_longitudinal,
            "instantaneous_lateral_slip_rad": np.asarray(
                adams_history.channels[f"{wheel}.lateral_slip_rad"][: time.size]
            ),
            "rolling_radius_m": np.asarray(
                adams_history.channels[f"{wheel}.rolling_radius_m"][: time.size]
            )
            * 1.0e-3,
            "rotational_velocity_rad_per_s": np.asarray(
                adams_history.channels[
                    f"{wheel}.rotational_velocity_rad_per_s"
                ][: time.size]
            ),
            "normal_force_n": adams_force[f"{wheel}.tire_normal_force"],
            "longitudinal_force_n": adams_force[f"{wheel}.tire_longitudinal_force"],
            "lateral_force_n": adams_force[f"{wheel}.tire_lateral_force"],
        }
        wheel_result: dict[str, Any] = {}
        for name, threshold in COMPARABLE_THRESHOLDS.items():
            event = _first_exceedance(
                time, reference_signals[name], native_signals[name], threshold
            )
            wheel_result[name] = event
            if event["first_time_s"] is not None:
                events.append({"wheel": wheel, "signal": name, **event})
        wheel_result["native_relaxation_state"] = {
            "effective_longitudinal_minus_instantaneous": _channel_metrics(
                native_signals["instantaneous_longitudinal_slip"],
                native_signals["effective_longitudinal_slip"],
            ),
            "effective_lateral_minus_instantaneous": _channel_metrics(
                native_signals["instantaneous_lateral_slip_rad"],
                native_signals["effective_lateral_slip_rad"],
            ),
        }
        by_wheel[wheel] = wheel_result

    events.sort(key=lambda item: (float(item["first_time_s"]), item["wheel"], item["signal"]))
    lateral_reference = adams_force["lateral_acceleration"]
    lateral_native = native_force["lateral_acceleration"]
    return {
        "comparison_root": str(root),
        "adams_state_observability": {
            "instantaneous_slip": "Adams wheel-tire kinematics public channel",
            "effective_slip": (
                "Adams PAC2002 local high-performance solver state is internal and "
                "is not exposed by this result channel"
            ),
        },
        "time_grid": {
            "start_s": float(time[0]),
            "end_s": float(time[-1]),
            "sample_count": int(time.size),
        },
        "by_wheel": by_wheel,
        "ordered_first_exceedances": events,
        "lateral_acceleration_coordinate_probe": {
            "raw": _channel_metrics(lateral_reference, lateral_native),
            "native_sign_reversed": _channel_metrics(lateral_reference, -lateral_native),
        },
        "gate": {
            "passed": not events,
            "failure_count": len(events),
        },
    }


def main() -> None:
    """运行状态分离诊断并按门限设置退出状态."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison-root", type=Path, required=True)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--no-gate", action="store_true")
    args = parser.parse_args()
    result = diagnose(args.comparison_root)
    rendered = json.dumps(result, indent=2)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(rendered, encoding="utf-8")
    print(rendered)
    if not result["gate"]["passed"] and not args.no_gate:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
