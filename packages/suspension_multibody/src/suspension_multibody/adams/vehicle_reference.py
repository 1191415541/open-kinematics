"""Versioned, traceable Adams/Car reference bundles for vehicle correlation."""

from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, cast

from .probe import AdamsProfile
from .time_domain import TimeHistory

ReferenceCategory = Literal["handling_stability", "ride"]

REFERENCE_BUNDLE_CONTRACT = "vehicle-adams-reference-v1"

HANDLING_REFERENCE_CHANNELS = (
    "steering_angle",
    "lateral_acceleration",
    "yaw_rate",
    "body_roll",
)
RIDE_REFERENCE_CHANNELS = (
    "body_accel_z",
    "body_heave",
    "body_pitch",
    "body_roll",
)

_CASE_DURATION_S = {
    "steady_state_circle": 17.0,
    "step_steer": 5.0,
    "sine_steer": 6.0,
    "double_lane_change": 12.0,
    "single_wheel_bump": 4.0,
    "double_wheel_bump": 4.0,
    "random_road": 8.0,
    "four_post_rig": 4.0,
}
_SOURCE_TO_CANONICAL = {
    "steering": "steering_angle",
    "steering_angle": "steering_angle",
    "roll_angle": "body_roll",
    "body_roll": "body_roll",
    "lateral_acceleration": "lateral_acceleration",
    "yaw_rate": "yaw_rate",
    "body_acceleration": "body_accel_z",
    "body_accel_z": "body_accel_z",
    "body_heave": "body_heave",
    "body_pitch": "body_pitch",
}
_CANONICAL_UNITS = {
    "steering_angle": "rad",
    "lateral_acceleration": "m/s^2",
    "yaw_rate": "rad/s",
    "body_roll": "rad",
    "body_accel_z": "m/s^2",
    "body_heave": "m",
    "body_pitch": "rad",
}
_UNIT_FACTORS = {
    ("rad", "rad"): 1.0,
    ("rad/s", "rad/s"): 1.0,
    ("m/s^2", "m/s^2"): 1.0,
    ("mm/s^2", "m/s^2"): 1e-3,
    ("m", "m"): 1.0,
    ("mm", "m"): 1e-3,
    ("deg", "rad"): math.pi / 180.0,
}
_REQUIRED_RAW_SUFFIXES = (".adm", ".cmd", ".msg", ".res")


@dataclass(frozen=True)
class VehicleReferenceBundle:
    """Canonical history plus immutable Adams input/output evidence."""

    case: str
    category: ReferenceCategory
    history: TimeHistory
    response_channels: tuple[str, ...]
    input_manifest: Mapping[str, object]
    input_manifest_hash: str
    raw_artifacts: Mapping[str, str]
    producer: Mapping[str, str]

    def as_dict(self) -> dict[str, object]:
        return {
            "contract": REFERENCE_BUNDLE_CONTRACT,
            "case": self.case,
            "category": self.category,
            "history": self.history.as_dict(),
            "response_channels": list(self.response_channels),
            "input_manifest": dict(self.input_manifest),
            "input_manifest_hash": self.input_manifest_hash,
            "raw_artifacts": dict(self.raw_artifacts),
            "producer": dict(self.producer),
        }


def canonicalize_vehicle_history(
    history: TimeHistory, category: ReferenceCategory
) -> TimeHistory:
    """Rename runner aliases to the fixed vehicle-correlation channel contract."""
    canonical: dict[str, tuple[float, ...]] = {}
    units: dict[str, str] = {}
    source_units = history.units or {}
    for source, values in history.channels.items():
        target = _SOURCE_TO_CANONICAL.get(source)
        if target is None:
            raise ValueError(f"unsupported {category} reference channel: {source}")
        if target in canonical:
            raise ValueError(f"duplicate canonical reference channel: {target}")
        unit = source_units.get(source)
        expected = _CANONICAL_UNITS[target]
        factor = _UNIT_FACTORS.get((unit or "", expected))
        if factor is None:
            raise ValueError(
                f"reference channel {source!r} has unit {unit!r}; expected {expected!r}"
            )
        canonical[target] = tuple(value * factor for value in values)
        units[target] = expected
    required = _required_channels(category)
    if set(canonical) != set(required):
        missing = sorted(set(required) - set(canonical))
        unexpected = sorted(set(canonical) - set(required))
        raise ValueError(
            f"{category} reference channels differ; missing={missing}, unexpected={unexpected}"
        )
    return TimeHistory(
        time=history.time,
        channels={name: canonical[name] for name in required},
        units={name: units[name] for name in required},
    )


def write_vehicle_reference_bundle(
    *,
    case: str,
    category: ReferenceCategory,
    history: TimeHistory,
    output_dir: str | Path,
    profile: AdamsProfile,
    input_manifest: Mapping[str, object],
) -> Path:
    """Write a verifiable reference bundle after a real Adams/Car execution."""
    destination = Path(output_dir)
    canonical = canonicalize_vehicle_history(history, category)
    duration = canonical.time[-1]
    _validate_case_grid(case, canonical, duration)
    raw_artifacts = _hash_raw_artifacts(destination)
    missing_suffixes = [
        suffix
        for suffix in _REQUIRED_RAW_SUFFIXES
        if not any(path.endswith(suffix) for path in raw_artifacts)
    ]
    if missing_suffixes:
        raise ValueError(
            f"reference evidence is incomplete for {case}: missing raw {missing_suffixes}"
        )
    manifest = dict(input_manifest)
    manifest.update(
        {
            "schema": "vehicle-adams-case-input-v1",
            "case": case,
            "category": category,
            "duration_s": duration,
            "output_step_s": 0.01,
            "frame": "vehicle_x_forward_y_left_z_up",
            "units": "SI",
            "tire_model": manifest.get("tire_model", "adams_builtin_pac2002"),
        }
    )
    bundle = VehicleReferenceBundle(
        case=case,
        category=category,
        history=canonical,
        response_channels=_response_channels(category),
        input_manifest=manifest,
        input_manifest_hash=_payload_hash(manifest),
        raw_artifacts=raw_artifacts,
        producer={
            "name": "msc.adams-car",
            "version": profile.version or "unknown",
            "profile": profile.name,
            "template": "Demo_Vehicle_Variants.asy/default",
            "tire_model": manifest.get("tire_model", "adams_builtin_pac2002"),
        },
    )
    output = destination / "adams_reference_bundle.json"
    output.write_text(
        json.dumps(bundle.as_dict(), indent=2, sort_keys=True), encoding="utf-8"
    )
    return output


def read_vehicle_reference_bundle(path: str | Path) -> VehicleReferenceBundle:
    """Read and validate a vehicle reference bundle without touching Adams files."""
    source = Path(path)
    payload = json.loads(source.read_text(encoding="utf-8"))
    if not isinstance(payload, Mapping):
        raise ValueError("vehicle reference bundle root must be an object")
    if payload.get("contract") != REFERENCE_BUNDLE_CONTRACT:
        raise ValueError("unsupported vehicle reference bundle contract")
    case = payload.get("case")
    category = payload.get("category")
    if not isinstance(case, str) or case not in _CASE_DURATION_S:
        raise ValueError("vehicle reference bundle has unknown case")
    if category not in {"handling_stability", "ride"}:
        raise ValueError("vehicle reference bundle has invalid category")
    history_payload = payload.get("history")
    if not isinstance(history_payload, Mapping):
        raise ValueError("vehicle reference bundle history must be an object")
    history = canonicalize_vehicle_history(
        TimeHistory.from_mapping(cast(Mapping[str, object], history_payload)),
        cast(ReferenceCategory, category),
    )
    response_channels = _string_tuple(payload.get("response_channels"), "response_channels")
    if response_channels != _response_channels(cast(ReferenceCategory, category)):
        raise ValueError("vehicle reference response channels do not match contract")
    manifest = payload.get("input_manifest")
    if not isinstance(manifest, Mapping):
        raise ValueError("vehicle reference input manifest must be an object")
    duration = manifest.get("duration_s")
    if not isinstance(duration, (float, int)):
        raise ValueError("vehicle reference input manifest duration_s must be numeric")
    _validate_case_grid(case, history, float(duration))
    manifest_hash = payload.get("input_manifest_hash")
    if not isinstance(manifest_hash, str) or manifest_hash != _payload_hash(manifest):
        raise ValueError("vehicle reference input manifest hash does not match content")
    raw = payload.get("raw_artifacts")
    if not isinstance(raw, Mapping) or not raw:
        raise ValueError("vehicle reference raw artifact hashes are required")
    raw_artifacts = {str(name): str(value) for name, value in raw.items()}
    missing_suffixes = [
        suffix
        for suffix in _REQUIRED_RAW_SUFFIXES
        if not any(name.endswith(suffix) for name in raw_artifacts)
    ]
    if missing_suffixes:
        raise ValueError(
            f"vehicle reference bundle misses required raw evidence: {missing_suffixes}"
        )
    for relative_path, expected_hash in raw_artifacts.items():
        artifact = source.parent / relative_path
        if not artifact.is_file():
            raise ValueError(f"vehicle reference raw artifact is missing: {relative_path}")
        if _file_hash(artifact) != expected_hash:
            raise ValueError(f"vehicle reference raw artifact hash changed: {relative_path}")
    producer = payload.get("producer")
    if not isinstance(producer, Mapping):
        raise ValueError("vehicle reference producer metadata must be an object")
    return VehicleReferenceBundle(
        case=case,
        category=cast(ReferenceCategory, category),
        history=history,
        response_channels=response_channels,
        input_manifest={str(name): value for name, value in manifest.items()},
        input_manifest_hash=manifest_hash,
        raw_artifacts=raw_artifacts,
        producer={str(name): str(value) for name, value in producer.items()},
    )


def _required_channels(category: ReferenceCategory) -> tuple[str, ...]:
    return (
        HANDLING_REFERENCE_CHANNELS
        if category == "handling_stability"
        else RIDE_REFERENCE_CHANNELS
    )


def _response_channels(category: ReferenceCategory) -> tuple[str, ...]:
    return tuple(
        channel
        for channel in _required_channels(category)
        if channel not in {"steering_angle"}
    )


def _validate_case_grid(
    case: str, history: TimeHistory, expected_end: float
) -> None:
    if expected_end <= 0.0:
        raise ValueError(f"reference {case} duration must be positive")
    expected_count = int(round(expected_end / 0.01)) + 1
    if len(history.time) != expected_count:
        raise ValueError(
            f"reference {case} sample count is {len(history.time)}, expected {expected_count}"
        )
    for index, value in enumerate(history.time):
        expected = 0.01 * index
        if not math.isclose(value, expected, abs_tol=1e-9):
            raise ValueError(
                f"reference {case} has noncanonical time at {index}: {value} != {expected}"
            )


def _hash_raw_artifacts(destination: Path) -> dict[str, str]:
    raw_dir = destination / "adams_raw"
    if not raw_dir.is_dir():
        raise ValueError("reference evidence has no adams_raw directory")
    artifacts = {
        path.relative_to(destination).as_posix(): _file_hash(path)
        for path in sorted(raw_dir.rglob("*"))
        if path.is_file()
    }
    if not artifacts:
        raise ValueError("reference evidence has no raw artifacts")
    return artifacts


def _file_hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _payload_hash(payload: Mapping[str, object]) -> str:
    return hashlib.sha256(
        json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


def _string_tuple(value: object, name: str) -> tuple[str, ...]:
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise ValueError(f"vehicle reference {name} must be a string list")
    return tuple(cast(str, item) for item in value)
