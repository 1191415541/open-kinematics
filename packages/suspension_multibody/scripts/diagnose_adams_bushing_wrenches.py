"""逐衬套比较 Adams 请求输出与 Native 局部变形和载荷."""

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

COMPONENTS = ("x", "y", "z")
FIELD_REQUESTS = {
    5: ("bkl_top_mount", "front"),
    6: ("bkr_top_mount", "front"),
    7: ("bkl_uca_front", "front"),
    8: ("bkr_uca_front", "front"),
    9: ("bkl_uca_rear", "front"),
    10: ("bkr_uca_rear", "front"),
    11: ("bkl_lwr_strut", "front"),
    12: ("bkr_lwr_strut", "front"),
    13: ("bkl_lca_front", "front"),
    14: ("bkr_lca_front", "front"),
    15: ("bkl_lca_rear", "front"),
    16: ("bkr_lca_rear", "front"),
    20: ("bgr_subframe_front", "rear"),
    21: ("bgl_subframe_rear", "rear"),
    22: ("bgr_subframe_rear", "rear"),
    23: ("bkl_top_mount", "rear"),
    24: ("bkr_top_mount", "rear"),
    25: ("bkl_uca_front", "rear"),
    26: ("bkr_uca_front", "rear"),
    27: ("bkl_uca_rear", "rear"),
    28: ("bkr_uca_rear", "rear"),
    29: ("bkl_lwr_strut", "rear"),
    30: ("bkr_lwr_strut", "rear"),
    31: ("bkl_lca_front", "rear"),
    32: ("bkr_lca_front", "rear"),
    33: ("bkl_lca_rear", "rear"),
    34: ("bkr_lca_rear", "rear"),
    35: ("bgl_subframe_front", "rear"),
    38: ("bkl_rack_housing_bushing", "front"),
    39: ("bkr_rack_housing_bushing", "front"),
}


def _read_json(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"JSON 根节点必须是对象: {path}")
    return payload


def _channels() -> dict[str, AdamsResultChannel]:
    channels: dict[str, AdamsResultChannel] = {}
    for field, (request, axle) in FIELD_REQUESTS.items():
        for kind, prefix in (("translation", "d"), ("rotation", "a")):
            for axis in COMPONENTS:
                channels[f"field_{field}.{kind}_{axis}"] = AdamsResultChannel(
                    f"{request}_disp", f"{prefix}{axis}_{axle}"
                )
        for kind, prefix in (("force", "f"), ("moment", "t")):
            for axis in COMPONENTS:
                channels[f"field_{field}.{kind}_{axis}"] = AdamsResultChannel(
                    f"{request}_force", f"{prefix}{axis}_{axle}"
                )
    return channels


def _metrics(
    reference: np.ndarray, actual: np.ndarray, end_time: float
) -> dict[str, float | str]:
    direct = actual - reference
    reversed_error = actual + reference
    direct_rmse = float(np.sqrt(np.mean(np.square(direct))))
    reversed_rmse = float(np.sqrt(np.mean(np.square(reversed_error))))
    reference_rms = float(np.sqrt(np.mean(np.square(reference))))
    return {
        "best_sign": "+" if direct_rmse <= reversed_rmse else "-",
        "direct_nrmse_percent": 100.0 * direct_rmse / max(reference_rms, 1.0e-12),
        "reversed_nrmse_percent": 100.0 * reversed_rmse / max(reference_rms, 1.0e-12),
        "adams_initial": float(reference[0]),
        "native_initial": float(actual[0]),
        "comparison_end_time_s": end_time,
        "adams_at_end": float(reference[-1]),
        "native_at_end": float(actual[-1]),
    }


def diagnose(
    adams_result: Path, native_artifact: Path, end_time: float
) -> dict[str, Any]:
    """比较相同时间网格上的衬套输出."""
    manifest = _read_json(native_artifact / "manifest.json")
    arrays = np.load(native_artifact / str(manifest["arrays_file"]))
    time = np.asarray(arrays["times_s"], dtype=float)
    native_names = [str(value) for value in arrays["bushing_names"]]
    native_output = np.asarray(arrays["bushing_output"], dtype=float)
    history = parse_adams_result_history(adams_result, _channels())
    adams_time = np.asarray(history.time, dtype=float)
    end = min(end_time, float(time[-1]), float(adams_time[-1]))
    mask = time <= end + 1.0e-12
    compare_time = time[mask]

    result: dict[str, Any] = {
        "comparison_end_time_s": end,
        "sample_count": int(compare_time.size),
        "fields": {},
    }
    output_columns = {
        "translation": (0, 1.0e-3),
        "rotation": (3, 1.0),
        "force": (6, 1.0),
        "moment": (9, 1.0e-3),
    }
    for field, (_, axle) in FIELD_REQUESTS.items():
        native_name = f"{axle}_adams_field_{field}"
        if native_name not in native_names:
            continue
        item: dict[str, Any] = {}
        native_index = native_names.index(native_name)
        for kind, (offset, adams_scale) in output_columns.items():
            for axis_index, axis in enumerate(COMPONENTS):
                name = f"field_{field}.{kind}_{axis}"
                reference = np.interp(
                    compare_time,
                    adams_time,
                    np.asarray(history.channels[name], dtype=float),
                ) * adams_scale
                actual = native_output[mask, native_index, offset + axis_index]
                item[f"{kind}_{axis}"] = _metrics(reference, actual, end)
        result["fields"][native_name] = item
    return result


def main() -> int:
    """运行衬套时域对比诊断."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--adams-result", type=Path, required=True)
    parser.add_argument("--native-artifact", type=Path, required=True)
    parser.add_argument("--end-time", type=float, default=1.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = diagnose(
        args.adams_result.resolve(),
        args.native_artifact.resolve(),
        args.end_time,
    )
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
