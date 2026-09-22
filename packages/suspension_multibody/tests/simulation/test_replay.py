"""
Prescribed-motion replay: orchestration in ``simulation``, aggregation in
``results``, and no integrator anywhere in between.

The replay is what replaced the retired Python integrator for the vehicle K/C
family: a sample *is* the value of the case's own motion signal at that time, so
nothing is propagated from one sample to the next.  These tests pin the three
things that could drift while the code moved from ``analysis`` to
``simulation/replay.py`` -- the time grid, the sample contents, and the
aggregation protocol -- and they pin the absence of integration in a way that an
integrator would fail: a coarser grid reaches the same pose at a shared time.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody import __version__
from suspension_multibody.preparation.geometry import rotation_vector_to_quaternion
from suspension_multibody.preparation.signals import time_grid
from suspension_multibody.results import TimeSeriesResult, TimeSeriesSample
from suspension_multibody.results.timeseries import aggregate_replay_samples
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    FrontAxleModel,
    MassSpec,
    PrescribedMotion,
    TimeSignal,
    VehicleBodyModel,
)
from suspension_multibody.simulation.replay import VehicleKCTimeDomainSolver

#: The declared window and step; the grid is (0, 1e-3, 2e-3).
_END_S = 2e-3
_STEP_S = 1e-3
_ROLL = (0.0, 0.05, 0.1)
_PITCH = (0.0, 0.0, 0.0)
_YAW = (0.0, 0.0, 0.0)
_HEAVE = (0.0, 1.0, 2.0)


def _model() -> FrontAxleModel:
    return FrontAxleModel(
        hardpoints={"wheel_center": [0, -700, 300]},
        mass=MassSpec(sprung_mass=1000.0),
    )


def _vehicle() -> VehicleBodyModel:
    return VehicleBodyModel(
        mass=1500.0,
        inertia=(
            (600_000.0, 0.0, 0.0),
            (0.0, 1_800_000.0, 0.0),
            (0.0, 0.0, 2_000_000.0),
        ),
        wheelbase=2800.0,
        front_track=1600.0,
        rear_track=1600.0,
    )


def _case(
    *, mode: str = "vehicle_kc_dynamic", step_size: float = _STEP_S
) -> DynamicCaseSpec:
    times = tuple(float(value) for value in np.linspace(0.0, _END_S, 3))
    return DynamicCaseSpec(
        mode=mode,
        solver=DynamicSolverSettings(end_time=_END_S, step_size=step_size),
        vehicle=_vehicle(),
        prescribed_motions=(
            PrescribedMotion(
                target="body_roll",
                displacement=TimeSignal(times=times, values=_ROLL),
            ),
            PrescribedMotion(
                target="body_pitch",
                displacement=TimeSignal(times=times, values=_PITCH),
            ),
            PrescribedMotion(
                target="body_yaw",
                displacement=TimeSignal(times=times, values=_YAW),
            ),
            PrescribedMotion(
                target="body_heave",
                displacement=TimeSignal(times=times, values=_HEAVE),
            ),
        ),
    )


def _replay(case: DynamicCaseSpec | None = None) -> TimeSeriesResult:
    return VehicleKCTimeDomainSolver().run(_model(), case or _case())


# --------------------------------------------------------------------------- #
# the time grid
# --------------------------------------------------------------------------- #


def test_replay_uses_the_case_time_grid_unchanged() -> None:
    """The replay publishes the case's own grid, in order and without gaps."""
    case = _case()
    result = _replay(case)
    expected = time_grid(case)
    assert expected == (0.0, _STEP_S, _END_S)
    assert tuple(result.times_s) == expected
    assert [sample.time for sample in result.samples] == list(expected)
    assert np.all(np.diff(result.times_s) > 0.0)


def test_replay_honours_the_output_step() -> None:
    """``output_step`` selects the grid the replay reports on."""
    case = _case().model_copy(update={"solver": DynamicSolverSettings(
        end_time=_END_S, step_size=1e-4, output_step=1e-3
    )})
    assert tuple(_replay(case).times_s) == (0.0, 1e-3, 2e-3)


# --------------------------------------------------------------------------- #
# the samples: prescribed values, no propagated state
# --------------------------------------------------------------------------- #


def test_every_sample_is_the_prescribed_motion_at_that_time() -> None:
    """A sample is the signal's value, not a state reached by integration."""
    case = _case()
    result = _replay(case)
    roll = _motion(case, "body_roll")
    heave = _motion(case, "body_heave")
    for sample in result.samples:
        assert sample.body == case.vehicle.name  # type: ignore[union-attr]
        assert sample.metrics["body_roll"] == pytest.approx(roll.value_at(sample.time))
        assert sample.metrics["roll_angle"] == sample.metrics["body_roll"]
        assert sample.metrics["degrees_of_freedom"] == 14.0
        expected = rotation_vector_to_quaternion(
            np.array([roll.value_at(sample.time), 0.0, 0.0])
        )
        assert sample.pose.rotation.w == pytest.approx(float(expected[0]))
        assert sample.pose.rotation.x == pytest.approx(float(expected[1]))
        assert sample.pose.translation.z == pytest.approx(heave.value_at(sample.time))


def test_no_sample_carries_propagated_state() -> None:
    """Nothing is integrated, so no sample has a velocity or a diagnostic."""
    result = _replay()
    assert result.diagnostics == ()
    for sample in result.samples:
        assert sample.velocity is None
        assert sample.acceleration is None
        assert sample.result is None
        assert sample.converged is True
        assert sample.events == ()
        assert dict(sample.loads) == {}


def test_replay_is_not_path_dependent() -> None:
    """A coarser grid reaches the same pose at a shared time."""
    coarse = _replay(_case(step_size=_STEP_S))
    fine = _replay(_case(step_size=_STEP_S / 4))
    assert len(fine.samples) > len(coarse.samples)
    shared = {sample.time: sample for sample in fine.samples}
    for sample in coarse.samples:
        other = shared[sample.time]
        assert sample.pose.rotation.w == pytest.approx(other.pose.rotation.w)
        assert sample.pose.translation.z == pytest.approx(other.pose.translation.z)


# --------------------------------------------------------------------------- #
# the aggregate
# --------------------------------------------------------------------------- #


def test_replay_aggregates_with_the_shared_protocol() -> None:
    """The result is the aggregation of its samples, with the sample count."""
    case = _case()
    result = _replay(case)
    assert isinstance(result, TimeSeriesResult)
    assert result.mode == "vehicle_kc_dynamic"
    assert result.status == "success"
    assert result.metrics == {"sample_count": len(result.samples)}
    assert result.sample_count == len(result.samples)
    assert result.manifest.sample_count == len(result.samples)
    assert result.manifest.mode == "vehicle_kc_dynamic"
    provenance = dict(result.provenance)
    assert provenance["package_version"] == __version__
    assert provenance["model_hash"] and provenance["case_hash"]


def test_aggregate_replay_samples_fixes_the_grid_and_the_default_metric() -> None:
    """Aggregation is a protocol, so orchestration does not choose it."""
    samples = (
        TimeSeriesSample(time=0.0, body="axle", metrics={"camber_deg": 1.0}),
        TimeSeriesSample(time=1e-3, body="axle", metrics={"camber_deg": 2.0}),
    )
    result = aggregate_replay_samples(
        samples, mode="kc_quasi_static", provenance={"package_version": "0.1.0"}
    )
    assert tuple(result.times_s) == (0.0, 1e-3)
    assert result.metrics == {"sample_count": 2}
    assert result.mode == "kc_quasi_static"
    assert result.provenance == {"package_version": "0.1.0"}
    assert result.diagnostics == ()

    overridden = aggregate_replay_samples(
        samples, mode=None, metrics={"sample_count": 0, "extra": 1}
    )
    assert overridden.metrics == {"sample_count": 0, "extra": 1}


# --------------------------------------------------------------------------- #
# refusals
# --------------------------------------------------------------------------- #


def test_replay_refuses_a_mode_it_does_not_own() -> None:
    with pytest.raises(ValueError, match="vehicle_kc_dynamic"):
        _replay(_case(mode="axle_dynamic"))


def test_replay_requires_vehicle_data() -> None:
    case = _case().model_copy(update={"vehicle": None})
    with pytest.raises(ValueError, match="requires vehicle data"):
        _replay(case)


def _motion(case: DynamicCaseSpec, target: str) -> TimeSignal:
    for prescribed in case.prescribed_motions:
        if prescribed.target == target:
            return prescribed.displacement
    raise AssertionError(f"the fixture declares no {target} motion")
