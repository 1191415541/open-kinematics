"""Full-vehicle Adams handling execution-gate tests."""

from __future__ import annotations

from pathlib import Path

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import TimeHistory
from suspension_multibody.adams.vehicle_handling import (
    HANDLING_ADAMS_CHANNELS,
    _input_manifest,
    _pac2002_assembly,
    validate_handling_execution,
)


def _profile(tmp_path: Path) -> AdamsProfile:
    return AdamsProfile(
        name="fixture",
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
    return TimeHistory(
        time=(0.0, 0.1),
        channels={name: (0.0, 1.0) for name in HANDLING_ADAMS_CHANNELS},
    )


def test_handling_execution_runs_all_required_cases(tmp_path: Path) -> None:
    invoked: list[str] = []

    def runner(_profile: AdamsProfile, name: str, _output_dir: Path) -> TimeHistory:
        invoked.append(name)
        return _history()

    result = validate_handling_execution(
        _profile(tmp_path), runner=runner, output_dir=tmp_path / "handling"
    )

    assert result.ok
    assert set(invoked) == {
        "steady_state_circle",
        "step_steer",
        "sine_steer",
        "double_lane_change",
    }
    assert result.report["correlation_status"].startswith("not_evaluated")


def test_handling_execution_rejects_missing_adams_response_channel(tmp_path: Path) -> None:
    result = validate_handling_execution(
        _profile(tmp_path),
        runner=lambda _profile, _name, _output: TimeHistory(
            time=(0.0, 0.1), channels={"steering": (0.0, 1.0)}
        ),
        output_dir=tmp_path / "handling",
    )

    assert not result.ok
    assert not result.report["cases"]["step_steer"]["passed"]


def test_handling_pac2002_assembly_uses_builtin_rt_tires(tmp_path: Path) -> None:
    source = (
        tmp_path
        / "acar"
        / "acar_concept.cdb"
        / "assemblies.tbl"
        / "Demo_Vehicle_Variants.asy"
    )
    source.parent.mkdir(parents=True)
    source.write_text(
        "USAGE = '<acar_shared>/subsystems.tbl/TR_Front_Tires.sub'\n"
        "USAGE = '<acar_shared>/subsystems.tbl/TR_Rear_Tires.sub'\n",
        encoding="utf-8",
    )

    assembly = _pac2002_assembly(_profile(tmp_path), tmp_path / "runtime")
    manifest = _input_manifest(_history(), "fixture.dcf", assembly.name)

    assert "TR_Front_Tires.sub::rt" in assembly.read_text(encoding="utf-8")
    assert "TR_Rear_Tires.sub::rt" in assembly.read_text(encoding="utf-8")
    assert manifest["tire_model"] == "adams_builtin_pac2002"
