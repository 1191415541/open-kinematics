from __future__ import annotations

import numpy as np

from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
    run_axle_dynamics,
)

_I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_ZERO_I = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_undamped_linear_oscillator_has_no_energy_drift() -> None:
    model = AxleDynamicsModel(
        name="energy",
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
                compression_damping_n_s_per_m=0.0,
                rebound_damping_n_s_per_m=0.0,
                free_length_m=0.25,
            ),
        ),
    )
    times = np.linspace(0.0, 0.1, 101)
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="energy",
            times_s=tuple(float(value) for value in times),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                rho_inf=1.0,
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )
    assert float(np.ptp(result.energy[:, 2])) < 2.0e-8
    assert float(np.max(np.abs(result.energy[:, 3]))) < 2.0e-8
    np.testing.assert_allclose(result.energy[:, 4:13], 0.0, atol=2e-8)
    np.testing.assert_allclose(
        np.sum(result.energy[:, 14:21], axis=1),
        result.energy[:, 1],
        atol=1e-12,
        rtol=0.0,
    )
    np.testing.assert_allclose(
        result.energy[:, 15],
        result.energy[:, 1],
        atol=1e-12,
        rtol=0.0,
    )


def test_damped_oscillator_reports_only_passive_damper_dissipation() -> None:
    model = AxleDynamicsModel(
        name="damped-energy",
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
    times = np.linspace(0.0, 0.1, 101)
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="damped-energy",
            times_s=tuple(float(value) for value in times),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                rho_inf=1.0,
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )

    assert np.all(result.energy[:, 7] >= 0.0)
    assert float(np.sum(result.energy[:, 7])) > 0.0
    np.testing.assert_allclose(result.energy[:, [4, 5, 6, 8, 9, 10]], 0.0)
    np.testing.assert_allclose(result.energy[:, 12], result.energy[:, 7])
    assert float(np.max(np.abs(result.energy[:, 3]))) < 2e-5


def test_drive_work_closes_free_wheel_rotational_energy() -> None:
    model = AxleDynamicsModel(
        name="drive-work",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="wheel",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 1.0),
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
                longitudinal_friction_coefficient=1.0,
                lateral_friction_coefficient=1.0,
                longitudinal_brush_stiffness_n_per_m=100_000.0,
                lateral_brush_stiffness_n_per_m=100_000.0,
                longitudinal_relaxation_length_m=0.2,
                lateral_relaxation_length_m=0.2,
                detached_relaxation_s=0.02,
            ),
        ),
    )
    times = tuple(float(value) for value in np.linspace(0.0, 0.01, 11))
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="drive-work",
            times_s=times,
            wheel_torque_n_m={"tire": (2.0,) * len(times)},
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                rho_inf=1.0,
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )

    assert float(np.sum(result.energy[:, 6])) > 0.0
    np.testing.assert_allclose(result.energy[:, [4, 5, 7, 8, 9, 10, 12]], 0.0)
    np.testing.assert_allclose(
        result.energy[-1, 2] - result.energy[0, 2],
        np.sum(result.energy[:, 6]),
        atol=2e-9,
        rtol=1e-9,
    )
    assert float(np.max(np.abs(result.energy[:, 3]))) < 2e-9
