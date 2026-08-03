"""Time-history comparison protocol tests."""

from __future__ import annotations

import json

import pytest

from suspension_multibody.adams.time_domain import (
    AdamsResultChannel,
    TimeHistory,
    TimeHistoryTolerance,
    compare_time_histories,
    parse_adams_result_history,
    read_time_history,
    write_time_history,
)


def test_time_history_json_round_trip_and_interpolation(tmp_path) -> None:
    reference = TimeHistory(
        time=(0.0, 0.5, 1.0),
        channels={"body_heave": (0.0, 10.0, 0.0)},
        units={"body_heave": "mm"},
    )
    path = write_time_history(reference, tmp_path / "reference.json")
    actual = TimeHistory(
        time=(0.0, 0.25, 0.5, 0.75, 1.0),
        channels={"body_heave": (0.0, 5.0, 10.0, 5.0, 0.0)},
        units={"body_heave": "mm"},
    )

    report = compare_time_histories(
        read_time_history(path),
        actual,
        {
            "body_heave": TimeHistoryTolerance(
                absolute=0.001,
                peak_relative_percent=0.01,
                rms_relative_percent=0.01,
                phase_ms=1.0,
            )
        },
    )

    assert report["passed"]
    assert report["channels"]["body_heave"]["max_absolute_error"] == 0.0  # type: ignore[index]


def test_time_history_csv_is_supported(tmp_path) -> None:
    path = tmp_path / "result.csv"
    path.write_text("time,yaw_rate\n0,0\n0.1,1\n", encoding="utf-8")

    history = read_time_history(path)

    assert history.channels == {"yaw_rate": (0.0, 1.0)}


def test_time_history_rejects_missing_or_truncated_actual_channels() -> None:
    reference = TimeHistory(
        time=(0.0, 1.0),
        channels={"roll_angle": (0.0, 1.0)},
    )
    tolerance = {"roll_angle": TimeHistoryTolerance(absolute=0.1)}

    with pytest.raises(ValueError, match="missing channels"):
        compare_time_histories(
            reference,
            TimeHistory(time=(0.0, 1.0), channels={"yaw_rate": (0.0, 1.0)}),
            tolerance,
        )
    with pytest.raises(ValueError, match="does not cover"):
        compare_time_histories(
            reference,
            TimeHistory(
                time=(0.1, 1.0),
                channels={"roll_angle": (0.0, 1.0)},
            ),
            tolerance,
        )


def test_time_history_phase_tolerance_fails_for_delayed_sine() -> None:
    reference = TimeHistory(
        time=(0.0, 0.1, 0.2, 0.3, 0.4),
        channels={"yaw_rate": (0.0, 1.0, 0.0, -1.0, 0.0)},
    )
    delayed = TimeHistory(
        time=(0.0, 0.1, 0.2, 0.3, 0.4),
        channels={"yaw_rate": (0.0, 0.0, 1.0, 0.0, -1.0)},
    )

    report = compare_time_histories(
        reference,
        delayed,
        {
            "yaw_rate": TimeHistoryTolerance(
                absolute=2.0,
                peak_relative_percent=1.0,
                rms_relative_percent=200.0,
                phase_ms=50.0,
            )
        },
    )

    assert not report["passed"]
    assert not report["channels"]["yaw_rate"]["phase_ms_passed"]  # type: ignore[index]


def test_time_history_json_requires_non_proprietary_shape(tmp_path) -> None:
    path = tmp_path / "invalid.json"
    path.write_text(json.dumps({"samples": []}), encoding="utf-8")

    with pytest.raises(ValueError, match="channels"):
        read_time_history(path)


def test_adams_result_parser_includes_initial_and_dynamic_samples(tmp_path) -> None:
    path = tmp_path / "dynamic.res"
    path.write_text(
        """<Results xmlns="urn:test"><Analysis><StepMap>
<Entity name="time"><Component name="TIME" id="1"/></Entity>
<Entity name="roll"><Component name="angle" id="2"/></Entity>
</StepMap><Data name="initialConditions_001"><Step>0 0</Step></Data>
<Data name="dynamic_001"><Step>0.1 0.2</Step><Step>0.2 0.4</Step></Data>
</Analysis></Results>""",
        encoding="utf-8",
    )

    history = parse_adams_result_history(
        path,
        {"body_roll": AdamsResultChannel("roll", "angle")},
        units={"body_roll": "rad"},
    )

    assert history.time == (0.0, 0.1, 0.2)
    assert history.channels["body_roll"] == (0.0, 0.2, 0.4)
