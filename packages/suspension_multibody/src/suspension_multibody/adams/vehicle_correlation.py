"""Numerical correlation gates for independent vehicle and Adams histories."""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

from ..analysis.vehicle_correlation_model import (
    VehicleCorrelationRun,
    simulate_vehicle_correlation_case,
)
from .time_domain import (
    TimeHistory,
    TimeHistoryTolerance,
    compare_time_histories,
    write_time_history,
)
from .vehicle_reference import VehicleReferenceBundle, read_vehicle_reference_bundle

CorrelationCategory = Literal["handling_stability", "ride"]
VehicleTireModel = Literal["pac2002", "native_brush"]
CorrelationSimulator = Callable[[str], VehicleCorrelationRun]

_REFERENCE_TIRE_MODELS = {
    "pac2002": "adams_builtin_pac2002",
    "native_brush": "adams_generated_brush",
}

_HANDLING_CASES = ("steady_state_circle", "step_steer", "sine_steer", "double_lane_change")
_RIDE_CASES = ("single_wheel_bump", "double_wheel_bump", "random_road", "four_post_rig")


@dataclass(frozen=True)
class VehicleCorrelationResult:
    """Per-family numerical-correlation report."""

    ok: bool
    output_path: str
    report: Mapping[str, object]


def validate_handling_correlation(
    reference_root: str | Path,
    *,
    output_dir: str | Path,
    simulator: CorrelationSimulator | None = None,
    tire_model: VehicleTireModel = "pac2002",
) -> VehicleCorrelationResult:
    """Compare all four handling cases using independent package histories."""
    return _validate(
        "handling_stability",
        _HANDLING_CASES,
        reference_root,
        output_dir,
        simulator=simulator,
        tire_model=tire_model,
    )


def validate_handling_correlation_matrix(
    reference_roots: Mapping[str, str | Path],
    *,
    output_dir: str | Path,
    simulators: Mapping[str, CorrelationSimulator] | None = None,
) -> VehicleCorrelationResult:
    """对 PAC2002 和 native_brush 分别验证，缺少参考时保留 BLOCKED。."""
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    variants: dict[str, object] = {}
    for tire_model in ("pac2002", "native_brush"):
        root = reference_roots.get(tire_model)
        if root is None or not Path(root).is_dir():
            reason = (
                "独立的 Adams/Car native_brush 参考尚未提供"
                if tire_model == "native_brush"
                else "PAC2002 Adams/Car 参考目录不存在"
            )
            variants[tire_model] = {
                "status": "BLOCKED",
                "tire_model": tire_model,
                "passed": False,
                "error": reason,
            }
            continue
        result = validate_handling_correlation(
            root,
            output_dir=destination / tire_model,
            simulator=(simulators or {}).get(tire_model),
            tire_model=tire_model,
        )
        variants[tire_model] = dict(result.report)
    report = {
        "contract": "vehicle-adams-correlation-matrix-v1",
        "category": "handling_stability",
        "variants": variants,
        "passed": bool(
            all(bool(item.get("passed")) for item in variants.values())
        ),
    }
    output = destination / "adams_handling_correlation_matrix_report.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return VehicleCorrelationResult(bool(report["passed"]), str(output), report)


def validate_ride_correlation(
    reference_root: str | Path,
    *,
    output_dir: str | Path,
    simulator: CorrelationSimulator | None = None,
) -> VehicleCorrelationResult:
    """Compare all four ride cases using independent package histories."""
    return _validate("ride", _RIDE_CASES, reference_root, output_dir, simulator=simulator)


def _validate(
    category: CorrelationCategory,
    cases: tuple[str, ...],
    reference_root: str | Path,
    output_dir: str | Path,
    *,
    simulator: CorrelationSimulator | None,
    tire_model: VehicleTireModel = "pac2002",
) -> VehicleCorrelationResult:
    root = Path(reference_root)
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    case_reports: dict[str, object] = {}
    for case in cases:
        case_dir = destination / case
        case_dir.mkdir(parents=True, exist_ok=True)
        try:
            bundle = _reference(root, case, category)
            expected_reference_model = _REFERENCE_TIRE_MODELS[tire_model]
            if bundle.input_manifest.get("tire_model") != expected_reference_model:
                raise ValueError(
                    "reference tire model does not match the selected comparison model"
                )
            run = (
                simulator(case)
                if simulator is not None
                else simulate_vehicle_correlation_case(
                    case,
                    input_manifest=bundle.input_manifest,
                    tire_model=tire_model,
                )
            )
            if run.input_manifest_hash != bundle.input_manifest_hash:
                raise ValueError("package and reference input manifest hashes differ")
            actual = _response_history(run.history, bundle)
            reference = _response_history(bundle.history, bundle)
            comparison = compare_time_histories(
                reference,
                actual,
                _tolerances(category, case, reference),
            )
            write_time_history(actual, case_dir / "package_time_history.json")
            case_reports[case] = {
                "status": "PASS" if comparison["passed"] else "FAIL",
                "reference_bundle": str(root / case / "adams_reference_bundle.json"),
                "input_manifest_hash": run.input_manifest_hash,
                "reference_input_manifest_hash": bundle.input_manifest_hash,
                "source_access_audit": list(run.source_access_audit),
                "comparison": comparison,
            }
        except Exception as exc:
            case_reports[case] = {"status": "BLOCKED", "error": str(exc)}
    passed = all(item["status"] == "PASS" for item in case_reports.values())
    report = {
        "contract": "vehicle-adams-correlation-v1",
        "category": category,
        "tire_model": tire_model,
        "cases": case_reports,
        "passed": passed,
    }
    output = destination / f"adams_{category}_correlation_report.json"
    output.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    return VehicleCorrelationResult(passed, str(output), report)


def _reference(root: Path, case: str, category: CorrelationCategory) -> VehicleReferenceBundle:
    bundle = read_vehicle_reference_bundle(root / case / "adams_reference_bundle.json")
    if bundle.category != category:
        raise ValueError(f"reference category mismatch for {case}")
    return bundle


def _response_history(history: TimeHistory, bundle: VehicleReferenceBundle) -> TimeHistory:
    channels = {
        name: _relative_handling_roll(name, history.channels[name], bundle)
        for name in bundle.response_channels
    }
    return TimeHistory(
        time=history.time,
        channels=channels,
        units={name: (history.units or {})[name] for name in bundle.response_channels},
    )


def _relative_handling_roll(
    name: str, values: tuple[float, ...], bundle: VehicleReferenceBundle
) -> tuple[float, ...]:
    if bundle.category != "handling_stability" or name != "body_roll" or not values:
        return values
    initial = values[0]
    return tuple(value - initial for value in values)


def _tolerances(
    category: CorrelationCategory, case: str, history: TimeHistory
) -> dict[str, TimeHistoryTolerance]:
    return {
        name: TimeHistoryTolerance(
            peak_relative_percent=_relative_tolerance(category, case, name),
            rms_relative_percent=_relative_tolerance(category, case, name),
            phase_ms=100.0,
        )
        for name in history.channels
    }


def _relative_tolerance(
    category: CorrelationCategory, case: str, channel: str
) -> float:
    if category == "ride":
        return 20.0 if channel == "body_accel_z" else 15.0
    if case == "steady_state_circle" and channel in {
        "lateral_acceleration",
        "yaw_rate",
    }:
        return 10.0
    return 15.0
