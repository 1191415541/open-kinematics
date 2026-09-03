"""计算完整 Native Fiala 与 Adams Fiala 的逐通道误差。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np


WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
FORCES = ("normal_force", "longitudinal_force", "lateral_force")


def _read(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"JSON 根节点必须是对象: {path}")
    return value


def _history(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    payload = _read(path)
    time = np.asarray(payload["time"], dtype=float)
    channels = {
        str(name): np.asarray(values, dtype=float)
        for name, values in payload["channels"].items()
    }
    if any(values.shape != time.shape for values in channels.values()):
        raise ValueError(f"历史通道长度与时间网格不一致: {path}")
    return time, channels


def _metric(reference: np.ndarray, actual: np.ndarray) -> dict[str, float]:
    error = actual - reference
    reference_rms = float(np.sqrt(np.mean(np.square(reference))))
    error_rms = float(np.sqrt(np.mean(np.square(error))))
    return {
        "nrmse_percent": 100.0 * error_rms / max(reference_rms, 1.0e-15),
        "reference_rms": reference_rms,
        "error_rms": error_rms,
        "maximum_absolute_error": float(np.max(np.abs(error))),
        "reference_peak": float(np.max(np.abs(reference))),
    }


def diagnose(root: Path) -> dict[str, Any]:
    """在同一公共时间网格上计算 Adams 与 Native Fiala 误差。"""
    root = root.resolve()
    manifest = _read(root / "comparison_manifest.json")
    required = {
        "same_initial_state_and_inputs": True,
        "native_steering_input": "prescribed_adams_rack_displacement",
        "native_wheel_torque_input": "direct_adams_drive_brake_replay",
    }
    mismatches = {
        name: {"expected": expected, "actual": manifest.get(name)}
        for name, expected in required.items()
        if manifest.get(name) != expected
    }
    if mismatches:
        raise ValueError(f"对比输入契约不一致: {mismatches}")
    adams_time, adams = _history(root / "adams_fiala_time_history.json")
    native_time, native = _history(root / "native_fiala_time_history.json")
    if adams_time.shape != native_time.shape or not np.allclose(
        adams_time, native_time, rtol=0.0, atol=1.0e-12
    ):
        raise ValueError("Adams 与 Native 不在同一时间网格")

    force: dict[str, Any] = {}
    for component in FORCES:
        by_wheel = {}
        reference_values = []
        actual_values = []
        for wheel in WHEELS:
            name = f"{wheel}.tire_{component}"
            reference = adams[name]
            actual = native[name]
            by_wheel[wheel] = _metric(reference, actual)
            reference_values.append(reference)
            actual_values.append(actual)
        force[component] = {
            "combined": _metric(
                np.concatenate(reference_values), np.concatenate(actual_values)
            ),
            "by_wheel": by_wheel,
        }

    gate = {
        "criterion": "四轮合并的每个力分量 NRMSE < 1%",
        "limits_percent": {component: 1.0 for component in FORCES},
        "observed_percent": {
            component: values["combined"]["nrmse_percent"]
            for component, values in force.items()
        },
    }
    gate["passed"] = all(
        value < 1.0 for value in gate["observed_percent"].values()
    )
    return {
        "comparison_root": str(root),
        "time_grid": {
            "start_s": float(adams_time[0]),
            "end_s": float(adams_time[-1]),
            "sample_count": int(adams_time.size),
            "step_s": float(np.median(np.diff(adams_time))),
        },
        "force": force,
        "gate": gate,
    }


def main() -> None:
    """执行 Fiala 误差门检。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--comparison-root", type=Path, required=True)
    args = parser.parse_args()
    result = diagnose(args.comparison_root)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    if not result["gate"]["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
