from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor

import numpy as np

from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
    native_build_metadata,
    run_axle_dynamics,
)


def _model() -> AxleDynamicsModel:
    mass = 10.0
    stiffness = 10_000.0
    inertia = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    return AxleDynamicsModel(
        name="performance-slider",
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=inertia,
                fixed=True,
            ),
            AxleBody(
                name="slider",
                mass_kg=mass,
                inertia_kg_m2=inertia,
                position_m=(0.0, 0.0, 0.20),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="spring",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=stiffness,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.25,
            ),
        ),
    )


def _case() -> AxleDynamicsCase:
    return AxleDynamicsCase(
        name="performance-metrics",
        times_s=(0.0, 0.001, 0.002),
        solver=AxleSolverSettings(
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=0.00025,
        ),
    )


def _tire_contact_boundary_model() -> AxleDynamicsModel:
    inertia = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    return AxleDynamicsModel(
        name="performance-contact-boundary",
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=inertia,
                fixed=True,
            ),
            AxleBody(
                name="wheel",
                mass_kg=1.0,
                inertia_kg_m2=inertia,
                position_m=(0.0, 0.0, 0.3),
            ),
        ),
        joints=(),
        tires=(
            AxleTire(
                name="tire",
                body="wheel",
                unloaded_radius_m=0.3,
                maximum_compression_m=0.05,
                vertical_stiffness_n_per_m=10_000.0,
                vertical_damping_n_s_per_m=100.0,
                longitudinal_friction_coefficient=0.2,
                lateral_friction_coefficient=0.3,
                longitudinal_brush_stiffness_n_per_m=100_000.0,
                lateral_brush_stiffness_n_per_m=80_000.0,
                longitudinal_relaxation_length_m=0.2,
                lateral_relaxation_length_m=0.25,
                detached_relaxation_s=0.02,
            ),
        ),
    )


def test_native_performance_counters_are_exposed(monkeypatch) -> None:
    monkeypatch.setenv("SUSPENSION_AXLE_PROFILE", "1")
    monkeypatch.setenv("SUSPENSION_AXLE_VALIDATE_JACOBIAN", "1")

    result = run_axle_dynamics(_model(), _case())

    performance = result.performance
    assert performance.available
    assert performance.residual_calls > 0
    assert performance.constraint_jacobian_calls > 0
    assert performance.force_evaluations >= performance.residual_calls
    assert performance.mass_inverse_calls > 0
    assert performance.linear_factorizations > 0
    assert performance.linear_solves > 0
    assert performance.line_search_trials >= performance.linear_solves
    assert performance.newton_iterations > 0
    assert performance.accepted_steps > 0
    assert performance.rejected_attempts >= 0
    assert performance.analytic_jacobian_columns > 0
    assert performance.finite_difference_jacobian_columns == 0
    assert performance.nonsmooth_fallback_columns == 0
    assert performance.residual_time_s >= 0.0
    assert performance.force_time_s >= 0.0
    assert performance.linear_factorization_time_s >= 0.0
    assert performance.linear_solve_time_s >= 0.0


def test_native_build_metadata_keeps_safe_optimization_flags() -> None:
    metadata = native_build_metadata()
    flags = tuple(str(flag) for flag in metadata["flags"])

    assert metadata["abi_version"] == 14
    assert metadata["configuration"] == "Release"
    assert "-ffast-math" not in flags
    assert "-fno-fast-math" in flags
    assert "-fopenmp" in flags
    assert any(flag in flags for flag in ("-O2", "-O3"))


def test_nonsmooth_contact_boundary_uses_finite_difference_fallback(monkeypatch) -> None:
    monkeypatch.setenv("SUSPENSION_AXLE_PROFILE", "1")
    monkeypatch.setenv("SUSPENSION_AXLE_VALIDATE_JACOBIAN", "1")

    result = run_axle_dynamics(
        _tire_contact_boundary_model(),
        AxleDynamicsCase(
            name="contact-boundary",
            times_s=(0.0, 0.001, 0.002),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )

    performance = result.performance
    assert performance.available
    assert performance.nonsmooth_fallback_columns > 0
    assert performance.finite_difference_jacobian_columns >= (
        performance.nonsmooth_fallback_columns
    )


def test_concurrent_native_runs_are_isolated() -> None:
    model = _model()
    case = _case()

    def run_once() -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        result = run_axle_dynamics(model, case)
        return result.states, result.diagnostics.position_residual, result.energy

    with ThreadPoolExecutor(max_workers=4) as executor:
        results = list(executor.map(lambda _: run_once(), range(4)))

    reference = results[0]
    for result in results[1:]:
        np.testing.assert_array_equal(result[0], reference[0])
        np.testing.assert_array_equal(result[1], reference[1])
        np.testing.assert_array_equal(result[2], reference[2])
