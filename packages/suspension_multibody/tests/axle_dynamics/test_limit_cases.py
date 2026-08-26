"""Limit cases: friction saturation, liftoff, and extreme parameter ratios."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.adams import load_axle_acceptance_contract
from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleDynamicsResult,
    AxleJoint,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
    run_axle_dynamics,
)

_I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_ZERO_I = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
_FIXTURE = AxleBody(
    name="fixture",
    mass_kg=0.0,
    inertia_kg_m2=_ZERO_I,
    fixed=True,
)


def _grid(duration_s: float, step_s: float) -> tuple[float, ...]:
    count = int(round(duration_s / step_s)) + 1
    return tuple(index * step_s for index in range(count))


def _energy_normalization(result: AxleDynamicsResult) -> float:
    """Apply the frozen energy normalization denominator."""
    floor_j = float(
        load_axle_acceptance_contract()["solver_internal_gates"][
            "energy_normalization"
        ]["energy_floor_j"]
    )
    supplied = result.energy[:, 4] + result.energy[:, 5] + result.energy[:, 6]
    elastic = np.sum(result.energy[:, 15:21], axis=1)
    return max(
        abs(float(result.energy[0, 2])),
        float(np.max(np.abs(np.cumsum(supplied)))),
        float(np.max(elastic)),
        floor_j,
    )


def _sliding_tire(*, friction_coefficient: float) -> AxleTire:
    return AxleTire(
        name="tire",
        body="wheel",
        unloaded_radius_m=0.3,
        maximum_compression_m=0.05,
        vertical_stiffness_n_per_m=100_000.0,
        vertical_damping_n_s_per_m=0.0,
        longitudinal_friction_coefficient=friction_coefficient,
        lateral_friction_coefficient=friction_coefficient,
        longitudinal_brush_stiffness_n_per_m=200_000.0,
        lateral_brush_stiffness_n_per_m=200_000.0,
        longitudinal_relaxation_length_m=1.0e9,
        lateral_relaxation_length_m=1.0e9,
        detached_relaxation_s=0.02,
    )


def _sliding_model(*, friction_coefficient: float) -> AxleDynamicsModel:
    """Build a vertically supported wheel free to translate along the patch."""
    return AxleDynamicsModel(
        name="sliding-wheel",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            _FIXTURE,
            AxleBody(
                name="wheel",
                mass_kg=20.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.29),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="fixture",
                body_b="wheel",
                point_a_m=(0.0, 0.0, 0.29),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(1.0, 0.0, 0.0),
                axis_b=(1.0, 0.0, 0.0),
            ),
        ),
        tires=(_sliding_tire(friction_coefficient=friction_coefficient),),
    )


def _run_sliding(
    *,
    friction_coefficient: float,
    drive_force_n: float,
    duration_s: float,
    internal_step_s: float,
    rho_inf: float = 0.8,
) -> AxleDynamicsResult:
    times = _grid(duration_s, 0.0005)
    return run_axle_dynamics(
        _sliding_model(friction_coefficient=friction_coefficient),
        AxleDynamicsCase(
            name="sliding-wheel",
            times_s=times,
            body_wrench_n_n_m={
                "wheel": ((drive_force_n, 0.0, 0.0, 0.0, 0.0, 0.0),)
                * len(times)
            },
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                rho_inf=rho_inf,
                adaptive_step=False,
                internal_step_s=internal_step_s,
                maximum_step_s=max(internal_step_s, 0.001),
            ),
        ),
    )


@pytest.mark.parametrize("rho_inf", (1.0, 0.8))
def test_sticking_brush_matches_the_analytic_compliance_oscillator(
    rho_inf: float,
) -> None:
    """Below saturation the patch is an exact linear spring in series."""
    mass_kg = 20.0
    brush_stiffness_n_per_m = 200_000.0
    drive_force_n = 50.0
    result = _run_sliding(
        friction_coefficient=50.0,
        drive_force_n=drive_force_n,
        duration_s=0.02,
        internal_step_s=1.0e-5,
        rho_inf=rho_inf,
    )

    times = np.asarray(result.times_s)
    natural = np.sqrt(brush_stiffness_n_per_m / mass_kg)
    exact = (drive_force_n / brush_stiffness_n_per_m) * (
        1.0 - np.cos(natural * times)
    )
    travel = result.body_state("wheel")[:, 0]
    tire = result.tire_state("tire")

    assert float(np.max(np.abs(travel - exact))) / float(
        np.max(exact)
    ) < 1.0e-6
    # The reported force must be exactly the brush force, not a refit.
    np.testing.assert_allclose(
        tire[:, 5],
        -brush_stiffness_n_per_m * tire[:, 10],
        atol=1.0e-12,
        rtol=0.0,
    )
    assert float(np.max(tire[:, 9])) < 1.0
    assert float(np.max(np.abs(result.energy[:, 8]))) < 1.0e-12


@pytest.mark.parametrize("rho_inf", (1.0, 0.8))
def test_saturated_friction_opposes_slip_and_never_injects_energy(
    rho_inf: float,
) -> None:
    """On the friction ellipse the force must brake, not drive, the slip."""
    friction_coefficient = 0.2
    result = _run_sliding(
        friction_coefficient=friction_coefficient,
        drive_force_n=400.0,
        duration_s=0.05,
        internal_step_s=5.0e-5,
        rho_inf=rho_inf,
    )

    tire = result.tire_state("tire")
    saturated = tire[:, 9] >= 1.0 - 1.0e-9
    normal_force = tire[:, 4]
    slip_velocity = tire[:, 7]
    longitudinal_force = tire[:, 5]

    assert np.any(saturated)
    # Coulomb limit is respected and reached, and the sign always dissipates.
    assert float(np.max(tire[:, 9])) <= 1.0 + 1.0e-9
    np.testing.assert_allclose(
        np.abs(longitudinal_force[saturated]),
        friction_coefficient * normal_force[saturated],
        rtol=1.0e-9,
    )
    assert np.all(longitudinal_force * slip_velocity <= 1.0e-9)
    assert np.all(result.energy[:, 8] >= 0.0)
    assert float(np.sum(result.energy[:, 8])) > 0.0


def test_saturated_sliding_force_is_step_size_independent() -> None:
    """A physical Coulomb limit must not depend on the internal step."""
    forces = []
    for internal_step_s in (2.5e-4, 5.0e-5, 1.0e-5):
        result = _run_sliding(
            friction_coefficient=0.2,
            drive_force_n=400.0,
            duration_s=0.02,
            internal_step_s=internal_step_s,
        )
        tire = result.tire_state("tire")
        forces.append(float(tire[-1, 5]))
        assert np.all(tire[:, 5] * tire[:, 7] <= 1.0e-9)

    assert forces[0] == pytest.approx(forces[-1], rel=1.0e-9)
    assert forces[1] == pytest.approx(forces[-1], rel=1.0e-9)


@pytest.mark.parametrize("rho_inf", (1.0, 0.8))
def test_brush_state_converges_at_second_order(rho_inf: float) -> None:
    """The first-order internal state must inherit the second-order rate."""
    reference = _run_sliding(
        friction_coefficient=50.0,
        drive_force_n=50.0,
        duration_s=0.02,
        internal_step_s=1.0e-6,
        rho_inf=rho_inf,
    ).tire_state("tire")[:, 10]
    errors = []
    for internal_step_s in (4.0e-5, 2.0e-5):
        brush = _run_sliding(
            friction_coefficient=50.0,
            drive_force_n=50.0,
            duration_s=0.02,
            internal_step_s=internal_step_s,
            rho_inf=rho_inf,
        ).tire_state("tire")[:, 10]
        errors.append(float(np.sqrt(np.mean(np.square(brush - reference)))))

    assert errors[0] > 0.0
    assert errors[0] / errors[1] == pytest.approx(4.0, rel=0.25)


def _liftoff_model() -> AxleDynamicsModel:
    return AxleDynamicsModel(
        name="liftoff",
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
        bodies=(
            _FIXTURE,
            AxleBody(
                name="wheel",
                mass_kg=40.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.29),
                linear_velocity_m_per_s=(0.0, 0.0, 3.0),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="fixture",
                body_b="wheel",
                point_a_m=(0.0, 0.0, 0.29),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
        tires=(
            AxleTire(
                name="tire",
                body="wheel",
                unloaded_radius_m=0.3,
                maximum_compression_m=0.06,
                vertical_stiffness_n_per_m=200_000.0,
                vertical_damping_n_s_per_m=500.0,
                longitudinal_friction_coefficient=1.0,
                lateral_friction_coefficient=1.0,
                longitudinal_brush_stiffness_n_per_m=200_000.0,
                lateral_brush_stiffness_n_per_m=150_000.0,
                longitudinal_relaxation_length_m=0.2,
                lateral_relaxation_length_m=0.25,
                detached_relaxation_s=0.02,
            ),
        ),
    )


def test_full_liftoff_and_recontact_stays_unilateral_and_dissipative() -> None:
    """A ballistic flight phase must release and land without pulling."""
    result = run_axle_dynamics(
        _liftoff_model(),
        AxleDynamicsCase(
            name="tire_liftoff_and_recontact",
            times_s=_grid(0.8, 0.005),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=True,
                internal_step_s=0.0005,
                maximum_step_s=0.001,
                contact_event_tolerance_s=1e-7,
            ),
        ),
    )

    tire = result.tire_state("tire")
    transitions = [event.transition for event in result.contact_events]
    gate = load_axle_acceptance_contract()["solver_internal_gates"]

    # The wheel must genuinely leave the ground and come back.
    assert transitions[:2] == ["exit", "enter"]
    assert np.any(tire[:, 0] == 0.0)
    assert np.any(tire[:, 0] == 1.0)
    # A unilateral contact never pulls and never exceeds the frozen limit.
    assert np.all(tire[:, 4] >= 0.0)
    assert float(np.max(tire[:, 2])) <= 0.06 + 1e-12
    # Detached brushes must not store force that would slam the recontact.
    np.testing.assert_allclose(tire[tire[:, 0] == 0.0][:, 5:7], 0.0)
    assert np.all(result.energy[:, 9] >= 0.0)
    assert np.all(result.energy[:, 8] >= 0.0)
    assert float(np.max(result.diagnostics.position_residual)) <= float(
        gate["constraint_position_m"]
    )
    closure = float(
        gate["contact_event_case_energy_closure_relative"]
    )
    assert float(
        np.max(np.abs(result.energy[:, 3]))
    ) / _energy_normalization(result) <= closure


@pytest.mark.parametrize(
    ("mass_kg", "stiffness_n_per_m"),
    (
        (2000.0, 2_000_000.0),
        (2.0, 2_000_000.0),
        (2000.0, 100_000.0),
    ),
)
def test_extreme_mass_stiffness_ratios_still_trim_and_integrate(
    mass_kg: float, stiffness_n_per_m: float
) -> None:
    """Static trim and integration must hold across 10^3 in natural frequency."""
    free_length_m = 0.25
    model = AxleDynamicsModel(
        name="ratio",
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
        bodies=(
            _FIXTURE,
            AxleBody(
                name="body",
                mass_kg=mass_kg,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.24),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="spring",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=stiffness_n_per_m,
                compression_damping_n_s_per_m=0.0,
                rebound_damping_n_s_per_m=0.0,
                free_length_m=free_length_m,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="ratio",
            times_s=_grid(0.02, 0.001),
            solver=AxleSolverSettings(
                initialization_mode="static_equilibrium",
                adaptive_step=True,
                internal_step_s=0.00025,
                maximum_step_s=0.001,
            ),
        ),
    )

    height = result.body_state("body")[:, 2]
    weight_n = mass_kg * 9.80665
    expected = free_length_m - weight_n / stiffness_n_per_m
    gate = load_axle_acceptance_contract()["solver_internal_gates"]

    # A trimmed state must be an exact equilibrium and must not drift.
    assert float(height[0]) == pytest.approx(expected, abs=1e-12)
    assert float(np.ptp(height)) <= 1e-9
    np.testing.assert_allclose(
        result.spring_state("spring")[:, 6], weight_n, rtol=1e-9
    )
    assert float(np.max(result.diagnostics.position_residual)) <= float(
        gate["constraint_position_m"]
    )


def _spin_axle(spin_kind: str) -> AxleDynamicsModel:
    """Build a strut corner whose wheel spin is statically indeterminate."""
    wheel_inertia = ((0.7, 0.0, 0.0), (0.0, 1.1, 0.0), (0.0, 0.0, 0.7))
    return AxleDynamicsModel(
        name="wheel-spin-trim",
        bodies=(
            _FIXTURE,
            AxleBody(
                name="carrier",
                mass_kg=18.0,
                inertia_kg_m2=wheel_inertia,
                position_m=(0.0, 0.0, 0.32),
            ),
            AxleBody(
                name="wheel",
                mass_kg=22.0,
                inertia_kg_m2=wheel_inertia,
                position_m=(0.0, 0.0, 0.32),
            ),
        ),
        joints=(
            AxleJoint(
                name="slide",
                kind="prismatic",
                body_a="fixture",
                body_b="carrier",
                point_a_m=(0.0, 0.0, 0.32),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
            AxleJoint(
                name="spin",
                kind=spin_kind,  # type: ignore[arg-type]
                body_a="carrier",
                body_b="wheel",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 1.0, 0.0),
                axis_b=(0.0, 1.0, 0.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="spring",
                body_a="fixture",
                body_b="carrier",
                point_a_m=(0.0, 0.0, 0.62),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=32_000.0,
                compression_damping_n_s_per_m=2600.0,
                rebound_damping_n_s_per_m=2600.0,
                free_length_m=0.32,
            ),
        ),
        tires=(
            AxleTire(
                name="tire",
                body="wheel",
                unloaded_radius_m=0.32,
                maximum_compression_m=0.05,
                vertical_stiffness_n_per_m=260_000.0,
                vertical_damping_n_s_per_m=800.0,
                longitudinal_friction_coefficient=1.0,
                lateral_friction_coefficient=0.95,
                longitudinal_brush_stiffness_n_per_m=180_000.0,
                lateral_brush_stiffness_n_per_m=120_000.0,
                longitudinal_relaxation_length_m=0.25,
                lateral_relaxation_length_m=0.35,
                detached_relaxation_s=0.05,
            ),
        ),
    )


def _trim(spin_kind: str) -> AxleDynamicsResult:
    return run_axle_dynamics(
        _spin_axle(spin_kind),
        AxleDynamicsCase(
            name="static_equilibrium",
            times_s=_grid(0.01, 0.001),
            solver=AxleSolverSettings(
                initialization_mode="static_equilibrium",
                adaptive_step=True,
                internal_step_s=0.00025,
                maximum_step_s=0.001,
            ),
        ),
    )


def test_free_wheel_spin_is_pinned_and_reported_without_changing_trim() -> None:
    """A wheel spin carries no static load, so trim must pin and report it."""
    spinning = _trim("revolute")
    locked = _trim("fixed")

    # Exactly one direction (the spin) is inert and must be reported as pinned.
    assert int(spinning.diagnostics.pinned_null_directions[0]) == 1
    assert int(locked.diagnostics.pinned_null_directions[0]) == 0
    # Pinning an inert direction must not move the physical equilibrium.
    assert float(spinning.body_state("carrier")[0, 2]) == pytest.approx(
        float(locked.body_state("carrier")[0, 2]), abs=1e-12
    )
    # The trimmed corner balances its weight plus the compressed spring load.
    weight_n = (18.0 + 22.0) * 9.80665
    assert float(spinning.tire_state("tire")[0, 4]) == pytest.approx(
        weight_n + float(spinning.spring_state("spring")[0, 6]), rel=1e-9
    )
    assert float(np.ptp(spinning.body_state("carrier")[:, 2])) <= 1e-9


def test_measured_damper_curve_is_reproduced_not_fitted() -> None:
    """A supplied force-velocity curve must be evaluated, never approximated."""
    velocity = (-4.0, -0.5, 0.0, 0.5, 4.0)
    # A real shock is asymmetric and carries gas preload at zero velocity, so
    # no pair of constant coefficients can represent this curve.
    force = (-1000.0, -400.0, -142.3, 180.0, 1400.0)
    model = AxleDynamicsModel(
        name="measured-damper",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            _FIXTURE,
            AxleBody(
                name="body",
                mass_kg=50.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.30),
                linear_velocity_m_per_s=(0.0, 0.0, -0.25),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(0.0, 0.0, 1.0),
                axis_b=(0.0, 0.0, 1.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="damper",
                body_a="fixture",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=0.0,
                compression_damping_n_s_per_m=0.0,
                rebound_damping_n_s_per_m=0.0,
                free_length_m=0.30,
                damper_curve_velocity_m_per_s=velocity,
                damper_curve_force_n=force,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="measured-damper",
            times_s=_grid(0.02, 0.001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=1.0e-5,
                maximum_step_s=0.001,
            ),
        ),
    )

    output = result.spring_state("damper")
    rate = output[:, 1]
    reported = output[:, 3]

    np.testing.assert_allclose(
        reported, -np.interp(rate, velocity, force), atol=0.0, rtol=0.0
    )
    # The body must actually move, otherwise the curve is never exercised.
    assert float(np.ptp(rate)) > 0.05
