"""Regression tests for correlation-input and physical-model diagnostics."""

from __future__ import annotations

import json
from pathlib import Path

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import TimeHistory
from suspension_multibody.adams.vehicle_correlation import (
    validate_handling_correlation,
)
from suspension_multibody.adams.vehicle_reference import (
    write_vehicle_reference_bundle,
)
from suspension_multibody.analysis import simulate_vehicle_correlation_case
from suspension_multibody.analysis.vehicle_correlation_model import (
    VehicleCorrelationRun,
)


def test_zero_four_post_manifest_produces_no_ride_response() -> None:
    """The package must solve the road functions frozen in the input manifest."""
    zero_input = {
        "case": "single_wheel_bump",
        "duration_s": 4.0,
        "output_step_s": 0.01,
        "four_post_functions": {
            "jms_post_pad_vertical_lf": "0",
            "jms_post_pad_vertical_rf": "0",
            "jms_post_pad_vertical_lr": "0",
            "jms_post_pad_vertical_rr": "0",
        },
    }

    run = simulate_vehicle_correlation_case(
        "single_wheel_bump",
        input_manifest=zero_input,
    )

    assert all(
        max(abs(value) for value in channel) == 0.0
        for channel in run.history.channels.values()
    )


def test_manifest_hash_mismatch_blocks_correlation(tmp_path: Path) -> None:
    """A numerical comparison is invalid when package and Adams inputs differ."""
    reference = tmp_path / "reference"
    histories = {
        case: _write_handling_bundle(reference, case)
        for case in (
            "steady_state_circle",
            "step_steer",
            "sine_steer",
            "double_lane_change",
        )
    }

    result = validate_handling_correlation(
        reference,
        output_dir=tmp_path / "out",
        simulator=lambda case: VehicleCorrelationRun(
            case,
            15,
            histories[case],
            "different-manifest",
            ("test",),
        ),
    )

    assert not result.ok
    assert all(
        item["status"] == "BLOCKED" for item in result.report["cases"].values()
    )


def test_handling_gate_removes_static_roll_zero_offset(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    for case in (
        "steady_state_circle",
        "step_steer",
        "sine_steer",
        "double_lane_change",
    ):
        _write_handling_bundle(reference, case)

    def simulator(case: str) -> VehicleCorrelationRun:
        payload = json.loads(
            (reference / case / "adams_reference_bundle.json").read_text()
        )
        source = TimeHistory.from_mapping(payload["history"])
        return VehicleCorrelationRun(
            case,
            15,
            TimeHistory(
                time=source.time,
                channels={
                    "lateral_acceleration": source.channels["lateral_acceleration"],
                    "yaw_rate": source.channels["yaw_rate"],
                    "body_roll": tuple(0.0 for _ in source.time),
                },
                units={
                    "lateral_acceleration": "m/s^2",
                    "yaw_rate": "rad/s",
                    "body_roll": "rad",
                },
            ),
            payload["input_manifest_hash"],
            ("test",),
        )

    result = validate_handling_correlation(
        reference, output_dir=tmp_path / "out", simulator=simulator
    )

    assert result.ok


def _write_handling_bundle(root: Path, case: str) -> TimeHistory:
    """Create a valid canonical handling bundle with complete raw evidence."""
    destination = root / case
    raw = destination / "adams_raw"
    raw.mkdir(parents=True)
    for suffix in (".adm", ".cmd", ".msg", ".res"):
        (raw / f"{case}{suffix}").write_text("fixture", encoding="ascii")
    duration = {
        "steady_state_circle": 17.0,
        "step_steer": 5.0,
        "sine_steer": 6.0,
        "double_lane_change": 12.0,
    }[case]
    time = tuple(index * 0.01 for index in range(int(duration * 100) + 1))
    source = TimeHistory(
        time=time,
        channels={
            "steering": tuple(0.0 for _ in time),
            "lateral_acceleration": tuple(1.0 for _ in time),
            "yaw_rate": tuple(0.1 for _ in time),
            "roll_angle": tuple(0.01 for _ in time),
        },
        units={
            "steering": "rad",
            "lateral_acceleration": "m/s^2",
            "yaw_rate": "rad/s",
            "roll_angle": "rad",
        },
    )
    write_vehicle_reference_bundle(
        case=case,
        category="handling_stability",
        history=source,
        output_dir=destination,
        profile=AdamsProfile(
            "fixture",
            str(root),
            "adams.bat",
            "2024.1",
            None,
            "t",
            "s",
            str(root),
            None,
            (),
            True,
            "passed",
            "fixture",
        ),
        input_manifest={},
    )
    return TimeHistory(
        time=time,
        channels={
            "lateral_acceleration": source.channels["lateral_acceleration"],
            "yaw_rate": source.channels["yaw_rate"],
            "body_roll": source.channels["roll_angle"],
        },
        units={
            "lateral_acceleration": "m/s^2",
            "yaw_rate": "rad/s",
            "body_roll": "rad",
        },
    )
