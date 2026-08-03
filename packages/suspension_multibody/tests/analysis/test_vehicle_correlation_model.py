"""Independent full-vehicle 14/15-DOF response-model tests."""

from __future__ import annotations

import math
from dataclasses import asdict

from suspension_multibody.analysis import simulate_vehicle_correlation_case
from suspension_multibody.analysis.vehicle_correlation_model import (
    Vehicle14DofParameters,
)


def test_15_dof_handling_response_has_canonical_channels() -> None:
    run = simulate_vehicle_correlation_case(
        "step_steer", degrees_of_freedom=15, tire_model="fiala"
    )

    assert run.degrees_of_freedom == 15
    assert len(run.history.time) == 501
    assert tuple(run.history.channels) == (
        "steering_angle",
        "lateral_acceleration",
        "yaw_rate",
        "body_roll",
    )
    assert max(abs(value) for value in run.history.channels["yaw_rate"]) > 0.0
    assert min(run.history.channels["body_roll"]) < 0.0
    assert all(
        math.isfinite(value)
        for values in run.history.channels.values()
        for value in values
    )


def test_14_dof_ride_response_has_canonical_channels() -> None:
    run = simulate_vehicle_correlation_case(
        "single_wheel_bump", degrees_of_freedom=14, tire_model="pac2002"
    )

    assert run.degrees_of_freedom == 14
    assert len(run.history.time) == 401
    assert tuple(run.history.channels) == (
        "body_accel_z",
        "body_heave",
        "body_pitch",
        "body_roll",
    )
    assert max(abs(value) for value in run.history.channels["body_heave"]) > 0.0


def test_model_refuses_reference_bundle_as_input() -> None:
    try:
        simulate_vehicle_correlation_case(
            "step_steer",
            input_manifest="adams_reference_bundle.json",
        )
    except ValueError as exc:
        assert "input manifest" in str(exc)
    else:
        raise AssertionError("reference bundle input must be rejected")


def test_handling_manifest_uses_frozen_driver_demand() -> None:
    samples = [0.0] * 601
    samples[100] = 0.2
    run = simulate_vehicle_correlation_case(
        "sine_steer",
        degrees_of_freedom=14,
        input_manifest={
            "schema": "vehicle-adams-case-input-v1",
            "case": "sine_steer",
            "duration_s": 6.0,
            "output_step_s": 0.01,
            "vehicle_model_parameters": asdict(Vehicle14DofParameters()),
            "steering_input": {
                "kind": "sampled_driver_demand",
                "sample_period_s": 0.01,
                "angle_rad": samples,
            },
        },
    )

    assert run.history.channels["steering_angle"][100] == 0.2
    assert run.history.channels["steering_angle"][101] == 0.0


def test_adams_handling_manifest_requires_frozen_driver_demand() -> None:
    try:
        simulate_vehicle_correlation_case(
            "step_steer",
            input_manifest={
                "schema": "vehicle-adams-case-input-v1",
                "case": "step_steer",
                "duration_s": 5.0,
                "output_step_s": 0.01,
            },
        )
    except ValueError as exc:
        assert "steering_input" in str(exc)
    else:
        raise AssertionError("Adams handling manifests require a frozen driver demand")
