from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.adams import (
    TimeHistory,
    adams_axle_raw_channel_map,
    compare_pac2002_tire_force_histories,
    compare_strict_axle_histories,
    compare_tire_force_histories,
    load_axle_acceptance_contract,
)
from suspension_multibody.adams.axle_contract import load_axle_channel_contract
from suspension_multibody.adams.axle_dynamic_history import (
    _build_result,
    _spring_outputs,
    _tire_outputs,
)


def test_raw_channel_map_uses_complete_standard_adams_entities() -> None:
    model = SimpleNamespace(
        bodies=(SimpleNamespace(name="body", fixed=False),),
        joints=(SimpleNamespace(name="plane", kind="inplane"),),
        springs=(SimpleNamespace(name="spring"),),
        tires=(SimpleNamespace(name="tire"),),
    )
    entity_ids = {
        "body:body:cm": 2,
        **{
            f"body:body:state:{component}": index
            for index, component in enumerate(
                (
                    "X",
                    "Y",
                    "Z",
                    "PSI",
                    "THETA",
                    "PHI",
                    "VX",
                    "VY",
                    "VZ",
                    "WX",
                    "WY",
                    "WZ",
                    "ACCX",
                    "ACCY",
                    "ACCZ",
                    "WDX",
                    "WDY",
                    "WDZ",
                ),
                start=100,
            )
        },
        "joint:plane": 3,
        "spring:spring": 4,
        **{f"tire:tire:{name}": index for index, name in enumerate(
            (
                "penetration",
                "penetration_rate",
                "normal_force",
                "longitudinal_force",
                "lateral_force",
                "longitudinal_slip",
                "lateral_slip",
                "friction_utilization",
                "brush_x",
                "brush_y",
            ),
            start=5,
        )},
    }

    channels = adams_axle_raw_channel_map(model, {"entity_ids": entity_ids})

    assert channels["body:body:X"].entity == "VARIABLE_100"
    assert channels["body:body:X"].component == "Q"
    assert channels["body:body:WZ"].entity == "VARIABLE_111"
    assert channels["body:body:WZ"].component == "Q"
    assert channels["joint:plane:FX"].entity == "JPRIM_3"
    assert channels["sforce:spring:FZ"].entity == "SFORCE_4"
    assert channels["tire:tire:normal_force"].entity == "VARIABLE_7"


def test_adams_initial_sample_preserves_shared_manifest_quaternion() -> None:
    body = SimpleNamespace(
        name="body",
        fixed=False,
        position_m=(0.0, 0.0, 0.0),
        quaternion_body_to_world=(0.8660254037844386, 0.0, 0.5, 0.0),
    )
    model = SimpleNamespace(
        bodies=(body,),
        joints=(),
        springs=(),
        bushings=(),
        anti_roll_bars=(),
        tires=(),
    )
    components = (
        "X",
        "Y",
        "Z",
        "PSI",
        "THETA",
        "PHI",
        "VX",
        "VY",
        "VZ",
        "WX",
        "WY",
        "WZ",
        "ACCX",
        "ACCY",
        "ACCZ",
        "WDX",
        "WDY",
        "WDZ",
    )
    raw = TimeHistory(
        time=(0.0, 0.1),
        channels={f"body:body:{component}": (0.0, 0.0) for component in components},
    )

    result = _build_result(model, {}, raw)

    assert result.states[0, 0, 3:7] == pytest.approx(
        body.quaternion_body_to_world
    )


def test_derived_spring_and_tire_columns_preserve_adams_sign_conventions() -> None:
    states = np.zeros((2, 2, 19), dtype=float)
    states[:, :, 3] = 1.0
    states[:, 1, 2] = 0.5
    spring = SimpleNamespace(
        name="spring",
        body_a="a",
        body_b="b",
        point_a_m=(0.0, 0.0, 0.0),
        point_b_m=(0.0, 0.0, 0.0),
        stiffness_n_per_m=10.0,
        free_length_m=0.6,
        minimum_length_m=None,
        maximum_length_m=None,
        compression_stop_stiffness_n_per_m=0.0,
        rebound_stop_stiffness_n_per_m=0.0,
    )
    tire = SimpleNamespace(name="tire")
    model = SimpleNamespace(springs=(spring,), tires=(tire,))
    raw = TimeHistory(
        time=(0.0, 0.1),
        channels={
            "sforce:spring:FX": (0.0, 0.0),
            "sforce:spring:FY": (0.0, 0.0),
            "sforce:spring:FZ": (2.0, 2.0),
            "tire:tire:penetration": (0.1, 0.2),
            "tire:tire:penetration_rate": (0.3, 0.4),
            "tire:tire:normal_force": (1.0, 0.0),
            "tire:tire:longitudinal_force": (2.0, 3.0),
            "tire:tire:lateral_force": (4.0, 5.0),
            "tire:tire:longitudinal_slip": (6.0, 7.0),
            "tire:tire:lateral_slip": (8.0, 9.0),
            "tire:tire:friction_utilization": (0.1, 0.2),
            "tire:tire:brush_x": (0.3, 0.4),
            "tire:tire:brush_y": (0.5, 0.6),
        },
    )

    spring_output = _spring_outputs(
        model,
        raw,
        states,
        {"a": 0, "b": 1},
    )
    tire_output = _tire_outputs(model, raw)

    assert spring_output[0, 0, 0] == pytest.approx(0.5)
    assert spring_output[0, 0, 2] == pytest.approx(1.0)
    assert spring_output[0, 0, 3] == pytest.approx(1.0)
    assert spring_output[0, 0, 6] == pytest.approx(2.0)
    assert tire_output[0, 0, 1] == pytest.approx(-0.1)
    assert tire_output[0, 0, 3] == pytest.approx(-0.3)
    assert tire_output[1, 0, 0] == pytest.approx(0.0)


def _canonical_history(
    time: tuple[float, ...],
    changed: tuple[float, ...] | None = None,
) -> TimeHistory:
    contract = load_axle_channel_contract()
    channels = {
        name: tuple(0.0 for _ in time)
        for name in contract["channels"]
    }
    if changed is not None:
        channels["sprung_body.heave"] = changed
    return TimeHistory(
        time=time,
        channels=channels,
        units={
            name: str(values["unit"])
            for name, values in contract["channels"].items()
        },
    )


def test_strict_dynamic_comparison_reports_channel_metrics_and_failure() -> None:
    reference = _canonical_history((0.0, 0.001, 0.002))
    candidate = _canonical_history((0.0, 0.001, 0.002), (0.0, 0.001, 0.0))

    report = compare_strict_axle_histories(
        reference,
        candidate,
        acceptance=load_axle_acceptance_contract(),
        case_name="road_sine",
        include_harmonic=False,
    )

    assert not report["passed"]
    metrics = report["channels"]["sprung_body.heave"]
    assert metrics["nrmse"] > 0.0
    assert "phase_lag_ms" in metrics
    assert not metrics["passed"]


def test_near_zero_channel_does_not_fail_peak_timing_gate() -> None:
    reference = _canonical_history((0.0, 0.001, 0.002), (0.0, 1.0e-8, -1.0e-8))
    candidate = _canonical_history((0.0, 0.001, 0.002), (0.0, -1.0e-8, 1.0e-8))

    report = compare_strict_axle_histories(
        reference,
        candidate,
        acceptance=load_axle_acceptance_contract(),
        case_name="near_zero",
        include_harmonic=False,
    )

    metrics = report["channels"]["sprung_body.heave"]
    assert not metrics["peak_timing_applicable"]
    assert metrics["peak_timing_passed"]


def test_pac2002_tire_force_comparison_uses_only_six_force_channels() -> None:
    reference = _canonical_history((0.0, 0.001, 0.002))
    candidate = _canonical_history((0.0, 0.001, 0.002))
    candidate.channels["sprung_body.heave"] = (1.0, 1.0, 1.0)

    report = compare_pac2002_tire_force_histories(
        reference,
        candidate,
        acceptance=load_axle_acceptance_contract(),
    )

    assert report["passed"]
    assert report["channels_expected"] == [
        "left.tire_normal_force",
        "right.tire_normal_force",
        "left.tire_longitudinal_force",
        "right.tire_longitudinal_force",
        "left.tire_lateral_force",
        "right.tire_lateral_force",
    ]


def test_tire_force_comparison_records_native_brush_model() -> None:
    reference = _canonical_history((0.0, 0.001, 0.002))
    candidate = _canonical_history((0.0, 0.001, 0.002))

    report = compare_tire_force_histories(
        reference,
        candidate,
        tire_model="native_brush",
        acceptance=load_axle_acceptance_contract(),
    )

    assert report["passed"]
    assert report["tire_model"] == "native_brush"
    assert report["contract"] == "tire-force-comparison-v2"


def test_fixture_peak_timing_ignores_derived_endpoint_samples() -> None:
    time = (0.0, 0.001, 0.002, 0.003, 0.004, 0.005, 0.006)
    reference = _canonical_history(time)
    candidate = _canonical_history(time)
    reference.channels["fixture.moment_y"] = (
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.05
    )
    candidate.channels["fixture.moment_y"] = (
        0.0, 0.0, 0.0, 0.0, 1.01, 0.0, 0.95
    )

    report = compare_strict_axle_histories(
        reference,
        candidate,
        acceptance=load_axle_acceptance_contract(),
        case_name="endpoint_fixture_wrench",
        include_harmonic=False,
    )

    metrics = report["channels"]["fixture.moment_y"]
    assert metrics["peak_window_edge_exclusion_samples"] == 3
    assert metrics["peak_timing_error_s"] == pytest.approx(0.0)
    assert metrics["peak_timing_passed"]


def test_strict_dynamic_comparison_rejects_time_grid_mismatch() -> None:
    reference = _canonical_history((0.0, 0.001, 0.002))
    candidate = _canonical_history((0.0, 0.002, 0.004))

    with pytest.raises(ValueError, match="identical time grid"):
        compare_strict_axle_histories(
            reference,
            candidate,
            acceptance=load_axle_acceptance_contract(),
            case_name="short",
        )
