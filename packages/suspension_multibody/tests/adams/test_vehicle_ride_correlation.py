"""Ride numerical-correlation gate tests."""

from __future__ import annotations

import json
from pathlib import Path
from typing import cast

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import TimeHistory
from suspension_multibody.adams.vehicle_correlation import validate_ride_correlation
from suspension_multibody.adams.vehicle_reference import write_vehicle_reference_bundle
from suspension_multibody.analysis.vehicle_correlation_model import (
    VehicleCorrelationRun,
)


def _profile(tmp_path: Path) -> AdamsProfile:
    return AdamsProfile(
        "fixture",
        str(tmp_path),
        "adams.bat",
        "2024.1",
        None,
        "t",
        "s",
        str(tmp_path),
        None,
        (),
        True,
        "passed",
        "fixture",
    )


def _bundle(root: Path, case: str) -> None:
    path = root / case
    raw = path / "adams_raw"
    raw.mkdir(parents=True)
    for suffix in (".adm", ".cmd", ".msg", ".res"):
        (raw / f"{case}{suffix}").write_text("fixture", encoding="ascii")
    duration = {
        "single_wheel_bump": 4.0,
        "double_wheel_bump": 4.0,
        "random_road": 8.0,
        "four_post_rig": 4.0,
    }[case]
    time = tuple(index * 0.01 for index in range(int(duration * 100) + 1))
    history = TimeHistory(
        time=time,
        channels={
            "body_acceleration": tuple(1.0 for _ in time),
            "body_heave": tuple(0.01 for _ in time),
            "body_pitch": tuple(0.02 for _ in time),
            "body_roll": tuple(0.01 for _ in time),
        },
        units={
            "body_acceleration": "m/s^2",
            "body_heave": "m",
            "body_pitch": "rad",
            "body_roll": "rad",
        },
    )
    write_vehicle_reference_bundle(
        case=case,
        category="ride",
        history=history,
        output_dir=path,
        profile=_profile(root),
        input_manifest={},
    )


def test_ride_gate_compares_all_cases(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    cases = (
        "single_wheel_bump",
        "double_wheel_bump",
        "random_road",
        "four_post_rig",
    )
    for case in cases:
        _bundle(reference, case)

    def simulator(case: str) -> VehicleCorrelationRun:
        payload = json.loads(
            (reference / case / "adams_reference_bundle.json").read_text()
        )
        history = TimeHistory.from_mapping(payload["history"])
        return VehicleCorrelationRun(
            case,
            15,
            history,
            cast(str, payload["input_manifest_hash"]),
            ("test",),
        )

    result = validate_ride_correlation(
        reference, output_dir=tmp_path / "out", simulator=simulator
    )
    case_reports = cast(dict[str, dict[str, object]], result.report["cases"])

    assert result.ok
    assert all(item["status"] == "PASS" for item in case_reports.values())
