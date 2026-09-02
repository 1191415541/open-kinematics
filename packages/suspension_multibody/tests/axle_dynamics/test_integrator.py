from __future__ import annotations

import math

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleSolverSettings,
    AxleSpringDamper,
    NativeAxleError,
    run_axle_dynamics,
)

_I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_ZERO_I = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def _oscillator() -> AxleDynamicsModel:
    return AxleDynamicsModel(
        name="oscillator",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.20),
            ),
        ),
        joints=(),
        springs=(
            AxleSpringDamper(
                name="spring",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=10_000.0,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.25,
            ),
        ),
    )


def _analytic_damped_response(times: np.ndarray) -> np.ndarray:
    mass = 10.0
    stiffness = 10_000.0
    damping = 100.0
    free_length = 0.25
    initial_offset = 0.20 - free_length
    decay = damping / (2.0 * mass)
    natural = math.sqrt(stiffness / mass)
    damped = math.sqrt(natural**2 - decay**2)
    return free_length + initial_offset * np.exp(-decay * times) * (
        np.cos(damped * times) + decay / damped * np.sin(damped * times)
    )


def test_fixed_and_adaptive_integrators_match_damped_oscillator() -> None:
    times = np.linspace(0.0, 0.1, 101)
    errors: list[float] = []
    for adaptive, step in ((False, 0.0005), (True, 0.004)):
        result = run_axle_dynamics(
            _oscillator(),
            AxleDynamicsCase(
                name="damped-oscillator",
                times_s=tuple(float(value) for value in times),
                solver=AxleSolverSettings(
                    initialization_mode="provided_consistent_state",
                    adaptive_step=adaptive,
                    internal_step_s=step,
                    maximum_step_s=0.004,
                    local_relative_tolerance=1e-5,
                    local_position_tolerance_m=1e-8,
                    local_velocity_tolerance_m_per_s=1e-7,
                ),
            ),
        )
        error = float(
            np.max(
                np.abs(
                    result.body_state("body")[:, 2]
                    - _analytic_damped_response(times)
                )
            )
        )
        errors.append(error)
        assert np.all(result.diagnostics.accepted)
        assert np.all(np.isfinite(result.states))
    assert errors[0] < 2.0e-6
    assert errors[1] < 2.0e-6


def test_provided_initial_state_is_not_replaced_by_static_trim() -> None:
    result = run_axle_dynamics(
        _oscillator(),
        AxleDynamicsCase(
            name="provided-state",
            times_s=(0.0, 0.001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )
    np.testing.assert_allclose(
        result.body_state("body")[0, 2], 0.20, atol=1e-12, rtol=0.0
    )


def test_stop_output_separates_conservative_and_dissipative_force() -> None:
    model = AxleDynamicsModel(
        name="compression-stop-decomposition",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.15),
                linear_velocity_m_per_s=(0.0, 0.0, -0.1),
            ),
        ),
        joints=(),
        springs=(
            AxleSpringDamper(
                name="suspension",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=10_000.0,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.25,
                minimum_length_m=0.20,
                compression_stop_stiffness_n_per_m=10_000.0,
                compression_stop_damping_n_s_per_m=50.0,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="compression-stop-decomposition",
            times_s=(0.0, 0.0001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.0001,
            ),
        ),
    )

    output = result.spring_state("suspension")[0]
    np.testing.assert_allclose(
        output,
        (0.15, -0.1, 1000.0, 10.0, 500.0, 0.0, 1515.0),
        atol=1e-10,
        rtol=0.0,
    )
    conservative = output[2] + output[4] + output[5]
    dissipative = output[6] - conservative
    assert conservative == pytest.approx(1500.0)
    assert dissipative == pytest.approx(15.0)


def test_failed_step_preserves_partial_result_and_failure_diagnostics() -> None:
    model = _oscillator()
    spring = model.springs[0].model_copy(update={"point_b_m": (0.10, 0.0, 0.0)})
    model = model.model_copy(update={"springs": (spring,)})
    with pytest.raises(NativeAxleError) as captured:
        run_axle_dynamics(
            model,
            AxleDynamicsCase(
                name="forced-newton-failure",
                times_s=(0.0, 0.01, 0.02),
                solver=AxleSolverSettings(
                    initialization_mode="provided_consistent_state",
                    adaptive_step=False,
                    internal_step_s=0.01,
                    maximum_step_s=0.01,
                    max_newton_iterations=1,
                ),
            ),
        )

    error = captured.value
    assert error.status == 5
    assert error.partial_result is not None
    np.testing.assert_allclose(error.partial_result.times_s, (0.0,))
    assert error.failure_diagnostics is not None
    assert error.failed_sample_index == 1
    assert error.failed_time_s == 0.01
    assert error.named_failure_diagnostics is not None
    assert error.named_failure_diagnostics["failure_code"] == 1.0
    assert error.failure_diagnostics[0] == 0.0
    assert error.failure_diagnostics[2] == 1.0
    assert error.failure_diagnostics[14] == 1.0
