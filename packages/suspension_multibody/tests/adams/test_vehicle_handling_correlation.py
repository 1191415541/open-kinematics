"""Handling numerical-correlation gate tests."""

from __future__ import annotations

from pathlib import Path

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import TimeHistory
from suspension_multibody.adams.vehicle_correlation import validate_handling_correlation
from suspension_multibody.adams.vehicle_reference import write_vehicle_reference_bundle
from suspension_multibody.analysis.vehicle_correlation_model import (
    VehicleCorrelationRun,
)


def _profile(tmp_path: Path) -> AdamsProfile:
    return AdamsProfile("fixture", str(tmp_path), "adams.bat", "2024.1", None, "t", "s", str(tmp_path), None, (), True, "passed", "fixture")


def _bundle(root: Path, case: str) -> None:
    path = root / case
    raw = path / "adams_raw"
    raw.mkdir(parents=True)
    for suffix in (".adm", ".cmd", ".msg", ".res"):
        (raw / f"{case}{suffix}").write_text("fixture", encoding="ascii")
    duration = {"steady_state_circle": 17.0, "step_steer": 5.0, "sine_steer": 6.0, "double_lane_change": 12.0}[case]
    time = tuple(index * 0.01 for index in range(int(duration * 100) + 1))
    history = TimeHistory(
        time=time,
        channels={"steering": tuple(0.0 for _ in time), "lateral_acceleration": tuple(1.0 for _ in time), "yaw_rate": tuple(0.1 for _ in time), "roll_angle": tuple(0.01 for _ in time)},
        units={"steering": "rad", "lateral_acceleration": "m/s^2", "yaw_rate": "rad/s", "roll_angle": "rad"},
    )
    write_vehicle_reference_bundle(case=case, category="handling_stability", history=history, output_dir=path, profile=_profile(root), input_manifest={})


def test_handling_gate_compares_all_cases(tmp_path: Path) -> None:
    reference = tmp_path / "reference"
    for case in ("steady_state_circle", "step_steer", "sine_steer", "double_lane_change"):
        _bundle(reference, case)

    def simulator(case: str) -> VehicleCorrelationRun:
        payload = __import__("json").loads(
            (reference / case / "adams_reference_bundle.json").read_text()
        )
        bundle = TimeHistory.from_mapping(payload["history"])
        return VehicleCorrelationRun(
            case,
            15,
            bundle,
            payload["input_manifest_hash"],
            ("test",),
        )

    result = validate_handling_correlation(reference, output_dir=tmp_path / "out", simulator=simulator)

    assert result.ok
    assert all(item["status"] == "PASS" for item in result.report["cases"].values())
