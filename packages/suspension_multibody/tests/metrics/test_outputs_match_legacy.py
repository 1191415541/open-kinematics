"""
The 27 legacy metrics, restated: values, not intentions.

Subtask 07 moves the numbers `report/metrics` produces onto derived outputs.  A
migration like that is only worth anything if the new path produces *the same
numbers*, so this compares them key by key on shared inputs -- including the
`NaN`/absence semantics the legacy code has.

The last test is the completeness check: the classification table must name all 27
functions, so a function cannot be left behind by the move.

`wheel_load_metrics` is subtask p3-05's subject: its table is now generated from
the placements the wheel-end names state, so it publishes the historical axle
spellings *beside* the placement-driven `normal_load_axle_{placement}` ones.  The
wheel-load tests below therefore pin the historical names and their values as a
subset of what the table emits -- the addition is checked, not assumed -- rather
than as an equality of the whole table.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.modeling.primitives import RigidBodyState
from suspension_multibody.outputs import BUILTIN, builtin
from suspension_multibody.outputs.builtin import LEGACY_CLASSIFICATION
from suspension_multibody.report.metrics import (
    compute_axle_metrics,
    compute_common_metrics,
    compute_vehicle_metrics,
    wheel_load_metrics,
)
from suspension_multibody.report.metrics.case_specific import (
    compute_k_metrics,
    wheel_metrics,
)
from suspension_multibody.subsystems.entry import compose_axle
from tests.benchmark_fixture import benchmark_model

#: The 27 functions of `report/metrics/`, one entry per function, as
#: `"module.function"`.  Written out rather than derived from the table so the
#: table is checked against an independent list.
LEGACY_NAMES = (
    "report.metrics.axle._tire_column",
    "report.metrics.axle.compute_axle_metrics",
    "report.metrics.axle.axle_metrics",
    "report.metrics.case_specific.register_case_metric",
    "report.metrics.case_specific.compute_case_metrics",
    "report.metrics.case_specific.case_metric_for",
    "report.metrics.case_specific.registered_case_metric_families",
    "report.metrics.case_specific.not_applicable_metrics",
    "report.metrics.case_specific.wheel_metrics",
    "report.metrics.case_specific.compute_k_metrics",
    "report.metrics.case_specific._summarize_series_values",
    "report.metrics.case_specific._kc_quasi_static_metrics",
    "report.metrics.case_specific._axle_dynamic_metrics",
    "report.metrics.case_specific._vehicle_dynamic_metrics",
    "report.metrics.case_specific._register_defaults",
    "report.metrics.common._finite_array",
    "report.metrics.common.peak",
    "report.metrics.common.rms",
    "report.metrics.common.residual_norm",
    "report.metrics.common.time_metrics",
    "report.metrics.common.convergence_metrics",
    "report.metrics.common.contact_metrics",
    "report.metrics.common.compute_common_metrics",
    "report.metrics.vehicle._embedded_wheel_loads",
    "report.metrics.vehicle.wheel_load_metrics",
    "report.metrics.vehicle.compute_vehicle_metrics",
    "report.metrics.vehicle.vehicle_metrics",
)


class FakeDiagnostics:
    """The attribute-shaped diagnostics a decoded result can carry."""

    accepted = np.array([True, True, False, True])
    rejected_attempts = np.array([0.0, 1.0, 2.0, 0.0])
    newton_iterations = np.array([3.0, 4.0, 5.0, 3.0])
    active_contacts = np.array([4.0, 4.0, 3.0, 4.0])
    contact_events = np.array([0.0, 1.0, 0.0, 0.0])


class FakeResult:
    times_s = np.array([0.0, 0.5, 1.0])
    tire_output = np.zeros((3, 1, 7))
    tire_output[:, 0, 4] = [1.0, 2.0, 3.0]
    tire_output[:, 0, 5] = [0.0, 1.0, 0.0]
    tire_output[:, 0, 6] = [2.0, 2.0, 2.0]
    diagnostics = None
    performance = SimpleNamespace(available=False)


class FakeDiagnosedResult(FakeResult):
    diagnostics = FakeDiagnostics()

#: The historical channels `wheel_load_metrics` published before its table became
#: placement-driven, with the values the four-corner loads above give.  Written out
#: rather than derived from the function, so the compatibility claim is checked
#: against an independent list.
LEGACY_LOAD_CHANNELS: dict[str, float] = {
    "normal_load_front_left": 100.0,
    "normal_load_front_right": 120.0,
    "normal_load_rear_left": 80.0,
    "normal_load_rear_right": 90.0,
    "normal_load_total": 390.0,
    "normal_load_front_axle": 220.0,
    "normal_load_rear_axle": 170.0,
    "normal_load_left_side": 180.0,
    "normal_load_right_side": 210.0,
    "load_transfer_front_minus_rear": 50.0,
    "load_transfer_right_minus_left": 30.0,
}


LOADS = {
    "front_left": 100.0,
    "front_right": 120.0,
    "rear_left": 80.0,
    "rear_right": 90.0,
}


def _compare(name: str, values: dict[str, object], expected: float) -> None:
    got = BUILTIN.evaluate(name, values)
    assert got == pytest.approx(expected), name


def test_the_common_and_axle_metrics_are_restated_value_for_value() -> None:
    result = FakeResult()
    values = builtin.minimum_unit_outputs(result)
    legacy = compute_axle_metrics(result)
    common = compute_common_metrics(result)

    for name in (
        "maximum_normal_force_n",
        "rms_normal_force_n",
        "maximum_longitudinal_force_n",
        "rms_longitudinal_force_n",
        "maximum_lateral_force_n",
        "rms_lateral_force_n",
    ):
        _compare(name, values, legacy[name])
    for name in ("sample_count", "start_s", "end_s", "duration_s"):
        _compare(name, values, common[name])
    # No diagnostics on this result, and the legacy code says so rather than
    # inventing values -- the restatement must agree on that too.
    for name in ("convergence_available", "contact_available"):
        assert BUILTIN.evaluate(name, values) == common[name]


def test_the_diagnostic_restatements_keep_their_availability_rule() -> None:
    result = FakeDiagnosedResult()
    values = builtin.minimum_unit_outputs(result)
    common = compute_common_metrics(result)

    for name, key in (
        ("convergence_available", "convergence_available"),
        ("convergence_accepted_fraction", "convergence_accepted_fraction"),
        ("convergence_accepted_sample_count", "convergence_accepted_sample_count"),
        ("convergence_rejected_attempt_count", "convergence_rejected_attempt_count"),
        ("convergence_maximum_newton_iterations", "convergence_maximum_newton_iterations"),
        ("contact_available", "contact_available"),
        ("contact_maximum_active_contact_count", "contact_maximum_active_contact_count"),
        ("contact_contact_event_count", "contact_contact_event_count"),
    ):
        _compare(name, values, common[key])


def test_the_wheel_load_metrics_are_restated_value_for_value() -> None:
    result = SimpleNamespace(times_s=np.array([0.0, 1.0]))
    values = builtin.minimum_unit_outputs(result, wheel_loads=LOADS)
    legacy = wheel_load_metrics(LOADS)
    assert legacy, "the legacy reader should produce the aggregate keys"
    # Every name the table emits is a declared output, and the declared output
    # evaluates to the same number.
    for name, expected in legacy.items():
        _compare(name, values, expected)


def test_the_wheel_load_metrics_still_publish_every_historical_channel() -> None:
    """
    The compatibility gate for p3-05: the old names survive with the old values.

    The table is now generated from the placements the wheel ends state, so the
    names it emits are a property of the loads rather than a constant in the
    module.  That is exactly the change that could quietly drop a name a consumer
    reads, so the historical channel set is written out independently here and
    compared value for value -- not merely "the same keys exist somewhere".
    """
    assert len(LEGACY_LOAD_CHANNELS) == 11
    legacy = wheel_load_metrics(LOADS)
    assert {name: legacy[name] for name in LEGACY_LOAD_CHANNELS} == LEGACY_LOAD_CHANNELS
    # The placement-driven spelling is an addition beside them, not a rename of
    # them: both are published, carrying the same total.
    assert legacy["normal_load_axle_front"] == LEGACY_LOAD_CHANNELS["normal_load_front_axle"]
    assert legacy["normal_load_axle_rear"] == LEGACY_LOAD_CHANNELS["normal_load_rear_axle"]
    assert set(legacy) == set(LEGACY_LOAD_CHANNELS) | {
        "normal_load_axle_front",
        "normal_load_axle_rear",
    }


def test_the_declared_load_outputs_are_exactly_the_report_channel_table() -> None:
    """
    The two halves of the wheel-load table agree, name for name.

    `report/metrics/vehicle.py` publishes a channel table and `outputs/builtin.py`
    declares the derived outputs for it.  The declaration cannot import the report
    (a report derives *from* a run's outputs, it is not one of them), so the two
    are written twice; this is what keeps the second copy from drifting -- a name
    in one and not the other fails here, not in a review.
    """
    result = SimpleNamespace(times_s=np.array([0.0, 1.0]))
    values = builtin.minimum_unit_outputs(result, wheel_loads=LOADS)
    declared = sorted(output.name for output in builtin.DERIVED_OUTPUTS if output.reads == (
        "wheel_load_front_left",
        "wheel_load_front_right",
        "wheel_load_rear_left",
        "wheel_load_rear_right",
    ))
    assert sorted(wheel_load_metrics(LOADS)) == declared
    for name, expected in wheel_load_metrics(LOADS).items():
        _compare(name, values, expected)


def test_the_vehicle_metrics_are_restated_value_for_value() -> None:
    vehicle = SimpleNamespace(
        axle=FakeResult(),
        steering_output=np.array([[1.0, -2.0], [3.0, 0.0]]),
        native_kernel_wall_time_s=1.5,
    )
    values = builtin.minimum_unit_outputs(vehicle, wheel_loads=LOADS)
    legacy = compute_vehicle_metrics(vehicle, wheel_loads=LOADS)
    for name in (
        "maximum_steering_output",
        "rms_steering_output",
        "native_kernel_wall_time_s",
        "normal_load_total",
        "normal_load_front_axle",
        "normal_load_rear_axle",
        "normal_load_left_side",
        "normal_load_right_side",
        "load_transfer_front_minus_rear",
        "load_transfer_right_minus_left",
    ):
        _compare(name, values, legacy[name])


def test_the_steering_restatement_ignores_non_finite_samples_like_the_legacy_code() -> None:
    vehicle = SimpleNamespace(
        axle=FakeResult(),
        steering_output=np.array(
            [[[1.0, np.nan], [3.0, np.nan]], [[2.0, np.nan], [0.0, np.nan]]]
        ),
        native_kernel_wall_time_s=0.0,
    )
    values = builtin.minimum_unit_outputs(vehicle)
    legacy = compute_vehicle_metrics(vehicle)
    _compare("maximum_steering_output", values, legacy["maximum_steering_output"])
    _compare("rms_steering_output", values, legacy["rms_steering_output"])


def test_the_kc_geometry_metrics_are_restated_value_for_value() -> None:
    """The pose-based metrics, over a real assembly rather than a fake result."""
    assembly = compose_axle(benchmark_model(), "K")
    state = RigidBodyState(assembly.bodies)
    values: dict[str, object] = {}
    for side in ("L", "R"):
        values.update(builtin.kc_minimum_unit_outputs(state, assembly, side))

    legacy: dict[str, float] = {}
    legacy.update(wheel_metrics(state, assembly, "L"))
    legacy.update(wheel_metrics(state, assembly, "R"))
    legacy.update(compute_k_metrics(state, assembly))

    assert len(legacy) == 15, "ten per-side keys plus five axle-level keys"
    for name, expected in legacy.items():
        _compare(name, values, expected)


def test_the_kc_restatement_keeps_the_side_sign_conventions() -> None:
    """
    Camber and toe are the metrics where a sign error looks plausible.

    The legacy convention is `-outward * degrees(atan2(axis, R[1, 1]))`, with the
    left side outward-negative and the right side outward-positive.  A single
    rotation matrix therefore gives *opposite* camber signs on the two sides --
    "inward" is a mirrored rotation on each side, not the same matrix.  Comparing
    against the legacy function would not catch a convention that was applied
    consistently wrong, so the expected values are computed here from the formula
    itself, and the sign is shown to follow the matrix entry it reads.
    """
    tip = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 1.0e-3, 1.0],
        ]
    )
    angle = float(np.degrees(np.arctan2(tip[2, 1], tip[1, 1])))
    assert angle > 0.0

    for side_name, rotation, outward in (
        ("left", "upright_left_rotation", -1.0),
        ("right", "upright_right_rotation", 1.0),
    ):
        values = {rotation: tip}
        camber = BUILTIN.evaluate(f"{side_name}_camber_deg", values)
        assert camber == pytest.approx(-outward * angle), side_name
        toe = BUILTIN.evaluate(f"{side_name}_toe_deg", values)
        assert toe == pytest.approx(0.0, abs=1e-12)


def test_the_kc_camber_follows_the_matrix_entry_it_reads() -> None:
    """Flipping the entry the angle reads must flip the angle."""
    base = {"upright_left_rotation": np.eye(3)}
    tipped = {
        "upright_left_rotation": np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 1.0e-3, 1.0],
            ]
        )
    }
    opposite = {
        "upright_left_rotation": np.array(
            [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, -1.0e-3, 1.0],
            ]
        )
    }
    assert BUILTIN.evaluate("left_camber_deg", base) == pytest.approx(0.0, abs=1e-12)
    positive = BUILTIN.evaluate("left_camber_deg", tipped)
    negative = BUILTIN.evaluate("left_camber_deg", opposite)
    assert positive == pytest.approx(-negative)
    assert positive != pytest.approx(0.0)


def test_every_legacy_function_is_classified_and_there_are_27() -> None:
    assert len(LEGACY_NAMES) == 27
    assert set(LEGACY_CLASSIFICATION) == set(LEGACY_NAMES)


def test_every_restated_output_names_the_legacy_function_it_replaces() -> None:
    restated = [output for output in builtin.DERIVED_OUTPUTS if output.legacy]
    assert restated, "the built-in set should restate the legacy metrics"
    unknown = sorted(
        {output.legacy for output in restated} - set(LEGACY_NAMES)
    )
    assert unknown == [], f"restated from functions that are not in report/metrics: {unknown}"
    for output in restated:
        assert output.reads, output.name
