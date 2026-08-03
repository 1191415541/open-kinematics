"""Full-vehicle Adams ride execution-gate tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from suspension_multibody.adams import AdamsProfile
from suspension_multibody.adams.time_domain import TimeHistory
from suspension_multibody.adams.vehicle_ride import (
    FOUR_POST_ADAMS_CHANNELS,
    RIDE_ADAMS_CHANNELS,
    _four_post_command,
    _four_post_functions,
    _has_static_equilibrium_failure,
    _rewrite_four_post_acf,
    _validate_four_post_excitations,
    validate_ride_execution,
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
        channels={name: (0.0, 1.0) for name in RIDE_ADAMS_CHANNELS},
    )


def test_ride_execution_runs_required_cases(tmp_path: Path) -> None:
    invoked: list[str] = []

    def runner(_profile: AdamsProfile, name: str, _output_dir: Path) -> TimeHistory:
        invoked.append(name)
        return _history()

    result = validate_ride_execution(
        _profile(tmp_path), runner=runner, output_dir=tmp_path / "ride"
    )

    assert result.ok
    assert set(invoked) == {
        "single_wheel_bump",
        "double_wheel_bump",
        "random_road",
        "four_post_rig",
    }


def test_ride_four_post_inputs_distinguish_single_and_double_wheel_bumps() -> None:
    single = _four_post_functions("single_wheel_bump")
    double = _four_post_functions("double_wheel_bump")

    assert single["jms_post_pad_vertical_lf"] != "0"
    assert single["jms_post_pad_vertical_rf"] == "0"
    assert double["jms_post_pad_vertical_lf"] != "0"
    assert double["jms_post_pad_vertical_rf"] != "0"


def test_ride_four_post_command_targets_template_actuator_objects() -> None:
    command = _four_post_command("fixture")

    assert "acar files assembly switch" in command
    assert "read property_files" in command
    assert "simulation_type=fourpost_time" in command
    assert "analysis_mode=files_only" in command


def test_ride_four_post_acf_rewrite_injects_corner_functions(tmp_path: Path) -> None:
    path = tmp_path / "probe.acf"
    path.write_text(
        "\n".join(
            (
                "diff/2, fun=0",
                "diff/3, fun=0",
                "diff/5, fun=0",
                "diff/4, fun=0",
            )
        ),
        encoding="ascii",
    )

    _rewrite_four_post_acf(path, "single_wheel_bump")

    text = path.read_text(encoding="ascii")
    assert 'diff/2, fun=STEP(TIME,0,0,0.05,30)-STEP(TIME,0.15,0,0.2,30)' in text
    assert "diff/3, fun=0" in text


def test_ride_four_post_execution_rejects_unapplied_input() -> None:
    displacement = TimeHistory(
        time=(0.0, 0.1),
        channels={name: (0.0, 0.0) for name in FOUR_POST_ADAMS_CHANNELS},
        units={name: "mm" for name in FOUR_POST_ADAMS_CHANNELS},
    )
    force = TimeHistory(
        time=(0.0, 0.1),
        channels={name: (0.0, 0.0) for name in FOUR_POST_ADAMS_CHANNELS},
        units={name: "newton" for name in FOUR_POST_ADAMS_CHANNELS},
    )

    with pytest.raises(RuntimeError, match="input was not applied"):
        _validate_four_post_excitations("single_wheel_bump", displacement, force)


def test_ride_four_post_execution_accepts_force_driven_input() -> None:
    displacement = TimeHistory(
        time=(0.0, 0.1),
        channels={name: (0.0, 0.0) for name in FOUR_POST_ADAMS_CHANNELS},
        units={name: "mm" for name in FOUR_POST_ADAMS_CHANNELS},
    )
    force = TimeHistory(
        time=(0.0, 0.1),
        channels={
            "jms_post_pad_vertical_lf": (0.0, 5.0),
            "jms_post_pad_vertical_rf": (0.0, 0.0),
            "jms_post_pad_vertical_rr": (0.0, 0.0),
            "jms_post_pad_vertical_lr": (0.0, 0.0),
        },
        units={name: "newton" for name in FOUR_POST_ADAMS_CHANNELS},
    )

    _validate_four_post_excitations("single_wheel_bump", displacement, force)


def test_ride_static_equilibrium_failure_is_rejected() -> None:
    assert _has_static_equilibrium_failure("Static equilibrium solution failed")
    assert not _has_static_equilibrium_failure("Simulate status=0")


def test_ride_execution_rejects_missing_response_channel(tmp_path: Path) -> None:
    result = validate_ride_execution(
        _profile(tmp_path),
        runner=lambda _profile, _name, _output: TimeHistory(
            time=(0.0, 0.1), channels={"body_roll": (0.0, 1.0)}
        ),
        output_dir=tmp_path / "ride",
    )

    assert not result.ok
