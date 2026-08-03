"""Canonical, traceable reference-bundle tests for vehicle Adams correlation."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import TimeHistory
from suspension_multibody.adams.vehicle_reference import (
    canonicalize_vehicle_history,
    read_vehicle_reference_bundle,
    write_vehicle_reference_bundle,
)


def _profile(tmp_path: Path) -> AdamsProfile:
    return AdamsProfile(
        name="adams-car-2024.1",
        home=str(tmp_path),
        executable="adams.bat",
        version="2024.1",
        license_file=None,
        template_id="fixture",
        subsystem_id="fixture",
        database_path=str(tmp_path),
        report_dictionary=None,
        export_fields=(),
        available=True,
        license_probe="passed",
        message="fixture",
    )


def _history() -> TimeHistory:
    time = tuple(index * 0.01 for index in range(501))
    return TimeHistory(
        time=time,
        channels={
            "steering": tuple(0.01 for _ in time),
            "lateral_acceleration": tuple(1.0 for _ in time),
            "yaw_rate": tuple(0.1 for _ in time),
            "roll_angle": tuple(0.02 for _ in time),
        },
        units={
            "steering": "rad",
            "lateral_acceleration": "m/s^2",
            "yaw_rate": "rad/s",
            "roll_angle": "rad",
        },
    )


def _raw_evidence(directory: Path) -> None:
    raw = directory / "adams_raw"
    raw.mkdir()
    for suffix in (".adm", ".cmd", ".msg", ".res"):
        (raw / f"step_steer{suffix}").write_text("fixture", encoding="ascii")


def test_reference_bundle_canonicalizes_aliases_and_hashes_evidence(
    tmp_path: Path,
) -> None:
    _raw_evidence(tmp_path)
    history = _history()

    path = write_vehicle_reference_bundle(
        case="step_steer",
        category="handling_stability",
        history=history,
        output_dir=tmp_path,
        profile=_profile(tmp_path),
        input_manifest={"input": "fixture"},
    )
    bundle = read_vehicle_reference_bundle(path)

    assert tuple(bundle.history.channels) == (
        "steering_angle",
        "lateral_acceleration",
        "yaw_rate",
        "body_roll",
    )
    assert bundle.history.units == {
        "steering_angle": "rad",
        "lateral_acceleration": "m/s^2",
        "yaw_rate": "rad/s",
        "body_roll": "rad",
    }
    assert bundle.input_manifest_hash
    assert any(name.endswith(".res") for name in bundle.raw_artifacts)


def test_reference_bundle_rejects_missing_raw_solver_result(tmp_path: Path) -> None:
    _raw_evidence(tmp_path)
    (tmp_path / "adams_raw" / "step_steer.res").unlink()

    with pytest.raises(ValueError, match="missing raw"):
        write_vehicle_reference_bundle(
            case="step_steer",
            category="handling_stability",
            history=_history(),
            output_dir=tmp_path,
            profile=_profile(tmp_path),
            input_manifest={"input": "fixture"},
        )


def test_reference_bundle_rejects_changed_raw_artifact(tmp_path: Path) -> None:
    _raw_evidence(tmp_path)
    path = write_vehicle_reference_bundle(
        case="step_steer",
        category="handling_stability",
        history=_history(),
        output_dir=tmp_path,
        profile=_profile(tmp_path),
        input_manifest={"input": "fixture"},
    )
    (tmp_path / "adams_raw" / "step_steer.res").write_text(
        "changed", encoding="ascii"
    )

    with pytest.raises(ValueError, match="hash changed"):
        read_vehicle_reference_bundle(path)


def test_reference_history_rejects_noncanonical_units() -> None:
    history = _history()
    with pytest.raises(ValueError, match="expected"):
        canonicalize_vehicle_history(
            TimeHistory(
                time=history.time,
                channels=history.channels,
                units={**history.units, "yaw_rate": "deg/s"},  # type: ignore[arg-type]
            ),
            "handling_stability",
        )


def test_reference_history_converts_native_adams_units_to_si() -> None:
    history = TimeHistory(
        time=(0.0, 0.01),
        channels={
            "body_acceleration": (1000.0, -2000.0),
            "body_heave": (10.0, -20.0),
            "body_pitch": (180.0, -90.0),
            "body_roll": (90.0, -45.0),
        },
        units={
            "body_acceleration": "mm/s^2",
            "body_heave": "mm",
            "body_pitch": "deg",
            "body_roll": "deg",
        },
    )

    canonical = canonicalize_vehicle_history(history, "ride")

    assert canonical.channels["body_accel_z"] == (1.0, -2.0)
    assert canonical.channels["body_heave"] == (0.01, -0.02)
    assert canonical.channels["body_pitch"] == pytest.approx((math.pi, -math.pi / 2.0))
