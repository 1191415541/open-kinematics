"""Diagnose and gate complete Native PAC2002 against Adams PAC2002."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
FORCE_COMPONENTS = ("normal_force", "longitudinal_force", "lateral_force")
HANDLING_CHANNELS = ("lateral_acceleration", "yaw_rate", "body_roll")
DEFAULT_LIMITS_PERCENT = {
    "normal_force": 3.0,
    "longitudinal_force": 50.0,
    "lateral_force": 20.0,
    "lateral_acceleration": 50.0,
    "yaw_rate": 20.0,
    "body_roll": 50.0,
}


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return payload


def _history(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    payload = _read_json(path)
    time = np.asarray(payload.get("time"), dtype=float)
    raw_channels = payload.get("channels")
    if time.ndim != 1 or not isinstance(raw_channels, dict):
        raise ValueError(f"invalid time history: {path}")
    channels = {
        str(name): np.asarray(values, dtype=float)
        for name, values in raw_channels.items()
    }
    if any(values.shape != time.shape for values in channels.values()):
        raise ValueError(f"channel length does not match time grid: {path}")
    return time, channels


def _nrmse(reference: np.ndarray, actual: np.ndarray) -> float:
    denominator = float(np.sqrt(np.mean(np.square(reference))))
    return 100.0 * float(np.sqrt(np.mean(np.square(actual - reference)))) / max(
        denominator, np.finfo(float).eps
    )


def _channel_metrics(reference: np.ndarray, actual: np.ndarray) -> dict[str, float]:
    reference_centered = reference - float(np.mean(reference))
    actual_centered = actual - float(np.mean(actual))
    denominator = float(reference @ reference)
    correlation_denominator = float(
        np.linalg.norm(reference_centered) * np.linalg.norm(actual_centered)
    )
    return {
        "nrmse_percent": _nrmse(reference, actual),
        "maximum_absolute_error": float(np.max(np.abs(actual - reference))),
        "least_squares_gain": (
            float(reference @ actual) / denominator if denominator > 0.0 else 0.0
        ),
        "correlation": (
            float(reference_centered @ actual_centered) / correlation_denominator
            if correlation_denominator > 0.0
            else 0.0
        ),
        "sign_agreement_percent": 100.0
        * float(np.mean(np.signbit(reference) == np.signbit(actual))),
    }


def diagnose(comparison_root: Path, split_time_s: float) -> dict[str, Any]:
    """计算完整 Native PAC2002 与 Adams PAC2002 的逐通道误差."""
    root = comparison_root.resolve()
    manifest = _read_json(root / "comparison_manifest.json")
    expected_contract = {
        "same_initial_state_and_inputs": True,
        "native_steering_input": "prescribed_adams_rack_displacement",
        "native_wheel_torque_input": "direct_adams_drive_brake_replay",
        "tire_force_coordinates": "pac2002_tire_iso_output",
    }
    mismatches = {
        key: {"expected": expected, "actual": manifest.get(key)}
        for key, expected in expected_contract.items()
        if manifest.get(key) != expected
    }
    if mismatches:
        raise ValueError(f"comparison input contract mismatch: {mismatches}")

    reference_time, reference = _history(root / "adams_pac2002_time_history.json")
    native_time, native = _history(root / "native_pac2002_time_history.json")
    if reference_time.shape != native_time.shape or not np.allclose(
        reference_time, native_time, rtol=0.0, atol=1.0e-12
    ):
        raise ValueError("Native and Adams time grids differ")

    before = reference_time < split_time_s
    after = ~before
    if not np.any(before) or not np.any(after):
        raise ValueError("split time must leave samples on both sides")

    force_metrics: dict[str, Any] = {}
    for component in FORCE_COMPONENTS:
        names = [f"{wheel}.tire_{component}" for wheel in WHEELS]
        reference_values = np.concatenate([reference[name] for name in names])
        native_values = np.concatenate([native[name] for name in names])
        force_metrics[component] = {
            "combined": _channel_metrics(reference_values, native_values),
            "by_wheel": {
                wheel: {
                    "full": _channel_metrics(
                        reference[f"{wheel}.tire_{component}"],
                        native[f"{wheel}.tire_{component}"],
                    ),
                    "before_split_nrmse_percent": _nrmse(
                        reference[f"{wheel}.tire_{component}"][before],
                        native[f"{wheel}.tire_{component}"][before],
                    ),
                    "after_split_nrmse_percent": _nrmse(
                        reference[f"{wheel}.tire_{component}"][after],
                        native[f"{wheel}.tire_{component}"][after],
                    ),
                }
                for wheel in WHEELS
            },
        }

    handling_metrics = {
        name: _channel_metrics(reference[name], native[name])
        for name in HANDLING_CHANNELS
    }
    return {
        "comparison_root": str(root),
        "time_grid": {
            "start_s": float(reference_time[0]),
            "end_s": float(reference_time[-1]),
            "sample_count": int(reference_time.size),
            "split_time_s": split_time_s,
        },
        "force": force_metrics,
        "handling": handling_metrics,
    }


def main() -> None:
    """执行 PAC2002 差分诊断并按门限返回进程状态."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison-root", type=Path, required=True)
    parser.add_argument("--split-time", type=float, default=1.0)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--no-gate", action="store_true")
    args = parser.parse_args()

    result = diagnose(args.comparison_root, args.split_time)
    observed = {
        component: result["force"][component]["combined"]["nrmse_percent"]
        for component in FORCE_COMPONENTS
    }
    observed.update(
        {
            name: result["handling"][name]["nrmse_percent"]
            for name in HANDLING_CHANNELS
        }
    )
    result["gate"] = {
        "limits_percent": DEFAULT_LIMITS_PERCENT,
        "observed_percent": observed,
        "failures": {
            name: value
            for name, value in observed.items()
            if value > DEFAULT_LIMITS_PERCENT[name]
        },
    }
    rendered = json.dumps(result, indent=2)
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(rendered, encoding="utf-8")
    print(rendered)
    if result["gate"]["failures"] and not args.no_gate:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
