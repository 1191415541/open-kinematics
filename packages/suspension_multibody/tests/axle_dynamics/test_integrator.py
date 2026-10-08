from __future__ import annotations

import math

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleBumpStop,
    AxleDamper,
    AxleDynamicsCase,
    AxleSolverSettings,
    AxleSpring,
    NativeAxleError,
)
from suspension_multibody.axle_dynamics.schema import AxleDynamicsModel
from tests.axle_dynamics._unified_entry import solve_axle

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
            AxleSpring(
                name="spring",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=10_000.0,
                free_length_m=0.25,
            ),
        ),
        dampers=(
            AxleDamper(
                name="spring_damper",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
            ),
        ),
        bump_stops=(),
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
        result = solve_axle(
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
    result = solve_axle(
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
            AxleSpring(
                name="suspension",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=10_000.0,
                free_length_m=0.25,
            ),
        ),
        dampers=(
            AxleDamper(
                name="suspension_damper",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
            ),
        ),
        bump_stops=(
            AxleBumpStop(
                name="suspension_stop",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                clearance_m=0.20,
                stiffness_n_per_m=10_000.0,
                direction=1.0,
                damping_n_s_per_m=50.0,
            ),
        ),
    )
    result = solve_axle(
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

    # The corner is three records now, so its decomposition is read from three
    # ledgers instead of from the seven columns one row used to carry.  The
    # quantities asserted are the same ones: the length and rate the corner is
    # at, the elastic force the spring applies, the damping force the damper
    # applies, and the stop's penetration and force.
    spring = result.spring_state("suspension")[0]
    damper = result.damper_state("suspension_damper")[0]
    stop = result.bump_stop_state("suspension_stop")[0]
    np.testing.assert_allclose(
        (spring[0], spring[1], spring[2], spring[3]),
        (0.15, -0.1, 1000.0, 0.0),
        atol=1e-10,
        rtol=0.0,
    )
    np.testing.assert_allclose(
        (damper[0], damper[1], damper[2]),
        (0.15, -0.1, 10.0),
        atol=1e-10,
        rtol=0.0,
    )
    np.testing.assert_allclose(
        (stop[0], stop[1], stop[2], stop[3], stop[4]),
        (0.15, -0.1, 0.05, 500.0, 1.0),
        atol=1e-10,
        rtol=0.0,
    )
    # The conservative part is the elastic law plus the stop; the dissipative
    # part is the damper's, and the two together are the corner's whole force.
    conservative = spring[2] + stop[3]
    dissipative = damper[2]
    assert conservative == pytest.approx(1500.0)
    assert dissipative == pytest.approx(10.0)
    assert conservative + dissipative == pytest.approx(1510.0)


def test_failed_step_preserves_partial_result_and_failure_diagnostics() -> None:
    """
    A fatal Newton failure still reports the partial result and diagnostics.

    The floor is deliberately equal to the step here: with no room to reduce, the
    failure is fatal.  When the floor is lower the solver halves the step and
    retries instead -- see
    ``test_newton_failure_halves_the_step_when_a_floor_is_available``.
    """
    model = _oscillator()
    spring = model.springs[0].model_copy(update={"point_b_m": (0.10, 0.0, 0.0)})
    model = model.model_copy(update={"springs": (spring,)})
    with pytest.raises(NativeAxleError) as captured:
        solve_axle(
            model,
            AxleDynamicsCase(
                name="forced-newton-failure",
                times_s=(0.0, 0.01, 0.02),
                solver=AxleSolverSettings(
                    initialization_mode="provided_consistent_state",
                    adaptive_step=False,
                    internal_step_s=0.01,
                    maximum_step_s=0.01,
                    minimum_step_s=0.01,
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
