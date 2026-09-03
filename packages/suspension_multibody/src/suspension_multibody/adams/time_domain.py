"""Time-history interchange and engineering comparison for Adams gates."""

from __future__ import annotations

import csv
import json
import math
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import cast

import numpy as np

from ..schema import DynamicResultBundle, TimeSignal


@dataclass(frozen=True)
class TimeHistory:
    """One named set of scalar histories on a strictly increasing time grid."""

    time: tuple[float, ...]
    channels: Mapping[str, tuple[float, ...]]
    units: Mapping[str, str] | None = None

    def __post_init__(self) -> None:
        if len(self.time) < 2:
            raise ValueError("time history requires at least two samples")
        if any(not math.isfinite(value) for value in self.time):
            raise ValueError("time samples must be finite")
        if any(right <= left for left, right in zip(self.time, self.time[1:])):
            raise ValueError("time samples must be strictly increasing")
        if not self.channels:
            raise ValueError("time history requires at least one channel")
        for name, values in self.channels.items():
            if not name:
                raise ValueError("time-history channel name cannot be empty")
            if len(values) != len(self.time):
                raise ValueError(f"channel {name!r} sample count does not match time")
            if any(not math.isfinite(value) for value in values):
                raise ValueError(f"channel {name!r} contains non-finite values")
        if self.units is not None:
            unknown = sorted(set(self.units) - set(self.channels))
            if unknown:
                raise ValueError(f"units specify unknown channels: {unknown}")
            if any(not unit for unit in self.units.values()):
                raise ValueError("channel unit cannot be empty")

    @classmethod
    def from_mapping(cls, payload: Mapping[str, object]) -> TimeHistory:
        """Build a history from the non-proprietary JSON interchange shape."""
        channels_data = payload.get("channels")
        if not isinstance(channels_data, Mapping):
            raise ValueError("time history channels must be a mapping")
        time = _finite_sequence(payload.get("time"), "time")
        channels = {
            str(name): _finite_sequence(values, f"channels.{name}")
            for name, values in channels_data.items()
        }
        units_data = payload.get("units")
        if units_data is not None and not isinstance(units_data, Mapping):
            raise ValueError("time history units must be a mapping")
        units = (
            {str(name): str(unit) for name, unit in units_data.items()}
            if isinstance(units_data, Mapping)
            else None
        )
        return cls(time=time, channels=channels, units=units)

    def as_dict(self) -> dict[str, object]:
        """Return the non-proprietary JSON interchange shape."""
        payload: dict[str, object] = {
            "time": list(self.time),
            "channels": {name: list(values) for name, values in self.channels.items()},
        }
        if self.units:
            payload["units"] = dict(self.units)
        return payload


@dataclass(frozen=True)
class TimeHistoryTolerance:
    """Acceptance limits for one scalar time-history channel."""

    absolute: float | None = None
    peak_relative_percent: float | None = None
    rms_relative_percent: float | None = None
    phase_ms: float | None = None

    def __post_init__(self) -> None:
        values = (
            self.absolute,
            self.peak_relative_percent,
            self.rms_relative_percent,
            self.phase_ms,
        )
        if not any(value is not None for value in values):
            raise ValueError("time-history tolerance requires at least one limit")
        if any(value is not None and value < 0 for value in values):
            raise ValueError("time-history tolerance limits must be non-negative")


@dataclass(frozen=True)
class AdamsResultChannel:
    """One scalar channel to extract from an Adams formatted result file."""

    entity: str
    component: str


_ADAMS_CONTACT_PATCH_CHANNELS = {
    "front_left": AdamsResultChannel("til_wheel_contact_patch", "z_front"),
    "front_right": AdamsResultChannel("tir_wheel_contact_patch", "z_front"),
    "rear_left": AdamsResultChannel("til_wheel_contact_patch", "z_rear"),
    "rear_right": AdamsResultChannel("tir_wheel_contact_patch", "z_rear"),
}


def read_time_history(source: str | Path | Mapping[str, object]) -> TimeHistory:
    """Read a time history from mapping, JSON, or CSV."""
    if isinstance(source, Mapping):
        return TimeHistory.from_mapping(cast(Mapping[str, object], source))
    path = Path(source)
    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("time-history JSON root must be an object")
        return TimeHistory.from_mapping(payload)
    if path.suffix.lower() != ".csv":
        raise ValueError("time-history input must be JSON or CSV")
    with path.open("r", newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows or not rows[0]:
        raise ValueError("time-history CSV must contain a header and samples")
    names = tuple(rows[0])
    if "time" not in names:
        raise ValueError("time-history CSV requires a time column")
    channels = tuple(name for name in names if name != "time")
    if not channels:
        raise ValueError("time-history CSV requires at least one channel column")
    return TimeHistory(
        time=tuple(float(row["time"]) for row in rows),
        channels={
            name: tuple(float(row[name]) for row in rows)
            for name in channels
        },
    )


def parse_adams_result_history(
    path: str | Path,
    channels: Mapping[str, AdamsResultChannel],
    *,
    units: Mapping[str, str] | None = None,
) -> TimeHistory:
    """Parse initial-condition and dynamic data from an Adams formatted result."""
    root = ET.parse(path).getroot()
    component_ids = {
        name: _adams_component_id(root, channel)
        for name, channel in channels.items()
    }
    time_id = _adams_component_id(root, AdamsResultChannel("time", "TIME"))
    datasets = [
        item
        for item in root.findall(".//{*}Data")
        if item.get("name") in {"initialConditions_001", "dynamic_001"}
    ]
    if not any(item.get("name") == "dynamic_001" for item in datasets):
        raise ValueError("Adams result has no dynamic_001 data")
    samples: list[tuple[float, dict[str, float]]] = []
    for dataset in datasets:
        for step in dataset.findall("{*}Step"):
            values = [float(value) for value in "".join(step.itertext()).split()]
            time = _adams_value(values, time_id, "time")
            current = {
                name: _adams_value(values, identifier, name)
                for name, identifier in component_ids.items()
            }
            if samples and time <= samples[-1][0]:
                if math.isclose(time, samples[-1][0], abs_tol=1e-12):
                    samples[-1] = (time, current)
                    continue
                raise ValueError("Adams result time samples are not strictly increasing")
            samples.append((time, current))
    if len(samples) < 2:
        raise ValueError("Adams result requires at least two dynamic samples")
    return TimeHistory(
        time=tuple(sample[0] for sample in samples),
        channels={
            name: tuple(sample[1][name] for sample in samples)
            for name in channels
        },
        units=units,
    )


def adams_contact_patch_plane_height_m(path: str | Path) -> float:
    """从 Adams 四轮接触面结果读取公共平面高度并转换为米。"""
    channels = {
        f"{wheel}.contact_patch_z": channel
        for wheel, channel in _ADAMS_CONTACT_PATCH_CHANNELS.items()
    }
    history = parse_adams_result_history(
        path,
        channels,
        units={name: "mm" for name in channels},
    )
    initial_heights_mm = np.asarray(
        [history.channels[name][0] for name in channels],
        dtype=float,
    )
    if not np.all(np.isfinite(initial_heights_mm)):
        raise ValueError("Adams 初始接触面高度包含非有限值")
    if float(np.ptp(initial_heights_mm)) > 1.0e-6:
        raise ValueError(
            "Adams 四轮初始接触面不是公共平面: "
            f"范围为 {float(np.ptp(initial_heights_mm)):.9g} mm"
        )
    return float(np.mean(initial_heights_mm) * 1.0e-3)


def adams_rack_displacement_signal_from_result(path: str | Path) -> TimeSignal:
    """从 Adams steering_Input 结果读取齿条位移历史，单位为毫米."""
    channel_name = "steering_rack_displacement"
    history = parse_adams_result_history(
        path,
        {
            channel_name: AdamsResultChannel(
                "steering_Input",
                "pitman_arm_rotation_or_rack_travel_front",
            )
        },
        units={channel_name: "mm"},
    )
    return TimeSignal(times=history.time, values=history.channels[channel_name])


def write_time_history(history: TimeHistory, path: str | Path) -> Path:
    """Write the non-proprietary JSON interchange representation."""
    destination = Path(path)
    destination.write_text(
        json.dumps(history.as_dict(), indent=2, sort_keys=False), encoding="utf-8"
    )
    return destination


def history_from_dynamic_bundle(
    bundle: DynamicResultBundle,
    *,
    body: str,
    channels: Sequence[str],
    units: Mapping[str, str] | None = None,
) -> TimeHistory:
    """Extract explicitly named metrics from one body in a dynamic result bundle."""
    samples = [sample for sample in bundle.samples if sample.body == body]
    if not samples:
        raise ValueError(f"dynamic result has no samples for body {body!r}")
    samples.sort(key=lambda sample: sample.time)
    values: dict[str, tuple[float, ...]] = {}
    for name in channels:
        if not all(name in sample.metrics for sample in samples):
            raise ValueError(
                f"dynamic result metric {name!r} is unavailable for body {body!r}"
            )
        values[name] = tuple(float(sample.metrics[name]) for sample in samples)
    return TimeHistory(
        time=tuple(float(sample.time) for sample in samples),
        channels=values,
        units=units,
    )


def compare_time_histories(
    reference: TimeHistory,
    actual: TimeHistory,
    tolerances: Mapping[str, TimeHistoryTolerance],
) -> dict[str, object]:
    """Compare a candidate history against a reference on the reference time grid."""
    reference_channels = set(reference.channels)
    actual_channels = set(actual.channels)
    missing = sorted(reference_channels - actual_channels)
    unexpected = sorted(actual_channels - reference_channels)
    if missing:
        raise ValueError(f"actual time history is missing channels: {missing}")
    if unexpected:
        raise ValueError(f"actual time history has unexpected channels: {unexpected}")
    missing_tolerances = sorted(reference_channels - set(tolerances))
    if missing_tolerances:
        raise ValueError(f"time-history tolerances are missing: {missing_tolerances}")
    unexpected_tolerances = sorted(set(tolerances) - reference_channels)
    if unexpected_tolerances:
        raise ValueError(f"time-history tolerances are unexpected: {unexpected_tolerances}")
    _validate_units(reference, actual)
    if actual.time[0] > reference.time[0] or actual.time[-1] < reference.time[-1]:
        raise ValueError("actual time history does not cover the reference time window")

    comparisons: dict[str, dict[str, float | bool]] = {}
    for name in sorted(reference_channels):
        reference_values = np.asarray(reference.channels[name], dtype=float)
        actual_values = np.interp(
            np.asarray(reference.time, dtype=float),
            np.asarray(actual.time, dtype=float),
            np.asarray(actual.channels[name], dtype=float),
        )
        comparisons[name] = _compare_channel(
            np.asarray(reference.time, dtype=float),
            reference_values,
            actual_values,
            tolerances[name],
        )
    return {
        "contract": "time-history-engineering-v1",
        "time_window": {
            "start_s": reference.time[0],
            "end_s": reference.time[-1],
            "reference_sample_count": len(reference.time),
            "actual_sample_count": len(actual.time),
        },
        "channels": comparisons,
        "passed": bool(all(values["passed"] for values in comparisons.values())),
    }


def _finite_sequence(value: object, name: str) -> tuple[float, ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{name} must be a numeric sequence")
    values = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in values):
        raise ValueError(f"{name} must contain finite values")
    return values


def _adams_component_id(root: ET.Element, channel: AdamsResultChannel) -> int:
    identifiers = {
        int(str(component.get("id")))
        for entity in root.findall(".//{*}StepMap/{*}Entity")
        if entity.get("name") == channel.entity
        for component in entity.findall("{*}Component")
        if component.get("name") == channel.component
    }
    if not identifiers:
        raise ValueError(
            f"Adams result channel was not found: {channel.entity}.{channel.component}"
        )
    if len(identifiers) != 1:
        raise ValueError(
            f"Adams result channel is ambiguous: {channel.entity}.{channel.component}"
        )
    return next(iter(identifiers))


def _adams_value(values: Sequence[float], identifier: int, name: str) -> float:
    if identifier > len(values):
        raise ValueError(f"Adams result channel map exceeds step data: {name}")
    return values[identifier - 1]


def _validate_units(reference: TimeHistory, actual: TimeHistory) -> None:
    reference_units = reference.units or {}
    actual_units = actual.units or {}
    for name in reference.channels:
        reference_unit = reference_units.get(name)
        actual_unit = actual_units.get(name)
        if reference_unit != actual_unit:
            raise ValueError(
                f"channel {name!r} units differ: {reference_unit!r} != {actual_unit!r}"
            )


def _compare_channel(
    time: np.ndarray,
    reference: np.ndarray,
    actual: np.ndarray,
    tolerance: TimeHistoryTolerance,
) -> dict[str, float | bool]:
    error = actual - reference
    reference_peak = float(np.max(np.abs(reference)))
    actual_peak = float(np.max(np.abs(actual)))
    peak_error = abs(actual_peak - reference_peak)
    max_absolute_error = float(np.max(np.abs(error)))
    reference_rms = _rms(reference)
    error_rms = _rms(error)
    scale = max(reference_peak, np.finfo(float).eps)
    rms_scale = max(reference_rms, np.finfo(float).eps)
    peak_relative_percent = 100.0 * peak_error / scale
    rms_relative_percent = 100.0 * error_rms / rms_scale
    phase_lag_ms = _phase_lag_ms(time, reference, actual)
    checks = {
        "absolute": (
            tolerance.absolute is None or max_absolute_error <= tolerance.absolute
        ),
        "peak_relative_percent": (
            tolerance.peak_relative_percent is None
            or peak_relative_percent <= tolerance.peak_relative_percent
        ),
        "rms_relative_percent": (
            tolerance.rms_relative_percent is None
            or rms_relative_percent <= tolerance.rms_relative_percent
        ),
        "phase_ms": (
            tolerance.phase_ms is None or abs(phase_lag_ms) <= tolerance.phase_ms
        ),
    }
    return {
        "reference_peak": reference_peak,
        "actual_peak": actual_peak,
        "peak_absolute_error": peak_error,
        "peak_relative_percent": peak_relative_percent,
        "reference_rms": reference_rms,
        "error_rms": error_rms,
        "rms_relative_percent": rms_relative_percent,
        "max_absolute_error": max_absolute_error,
        "phase_lag_ms": phase_lag_ms,
        "passed": bool(all(checks.values())),
        **{f"{name}_passed": bool(passed) for name, passed in checks.items()},
    }


def _rms(values: np.ndarray) -> float:
    return float(np.sqrt(np.mean(np.square(values))))


def _phase_lag_ms(time: np.ndarray, reference: np.ndarray, actual: np.ndarray) -> float:
    if len(time) < 3:
        return 0.0
    reference_centered = reference - np.mean(reference)
    actual_centered = actual - np.mean(actual)
    if (
        np.linalg.norm(reference_centered) <= np.finfo(float).eps
        or np.linalg.norm(actual_centered) <= np.finfo(float).eps
    ):
        return 0.0
    correlation = np.correlate(actual_centered, reference_centered, mode="full")
    lag_samples = int(np.argmax(correlation)) - (len(reference) - 1)
    step = float(np.median(np.diff(time)))
    return 1000.0 * lag_samples * step
