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
    run_axle_dynamics,
)

_ZERO_I = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
_FIXTURE = AxleBody(
    name="fixture",
    mass_kg=0.0,
    inertia_kg_m2=_ZERO_I,
    fixed=True,
)


def _rotation(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float)
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _grid(duration_s: float, step_s: float = 0.001) -> tuple[float, ...]:
    count = int(round(duration_s / step_s)) + 1
    return tuple(index * step_s for index in range(count))


def test_torque_free_asymmetric_body_conserves_momentum_and_energy() -> None:
    """A free asymmetric top must keep world angular momentum and energy."""
    inertia = ((2.0, 0.0, 0.0), (0.0, 5.0, 0.0), (0.0, 0.0, 9.0))
    model = AxleDynamicsModel(
        name="torque-free-top",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            _FIXTURE,
            AxleBody(
                name="top",
                mass_kg=4.0,
                inertia_kg_m2=inertia,
                angular_velocity_rad_per_s=(3.0, 1.0, 2.0),
            ),
        ),
        joints=(),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="torque-free-top",
            times_s=_grid(0.2),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                rho_inf=1.0,
                adaptive_step=False,
                internal_step_s=0.0001,
            ),
        ),
    )

    state = result.body_state("top")
    body_inertia = np.asarray(inertia, dtype=float)
    world_inertia = np.asarray(
        [
            _rotation(sample[3:7]) @ body_inertia @ _rotation(sample[3:7]).T
            for sample in state
        ]
    )
    momentum = np.einsum("tij,tj->ti", world_inertia, state[:, 10:13])
    energy = 0.5 * np.einsum("ti,ti->t", state[:, 10:13], momentum)

    reference = float(np.linalg.norm(momentum[0]))
    assert float(np.max(np.abs(momentum - momentum[0]))) / reference < 1e-7
    assert float(np.max(np.abs(energy - energy[0]))) / energy[0] < 1e-7
    # A genuinely asymmetric top must actually tumble, otherwise the test would
    # pass on a body that never rotates away from its initial axes.
    assert float(np.max(np.abs(state[:, 10:13] - state[0, 10:13]))) > 0.1


def test_internal_forces_conserve_total_linear_momentum() -> None:
    """Action equals reaction: an isolated spring pair keeps its momentum."""
    inertia = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    model = AxleDynamicsModel(
        name="action-reaction",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            _FIXTURE,
            AxleBody(
                name="left",
                mass_kg=3.0,
                inertia_kg_m2=inertia,
                position_m=(0.0, -0.1, 0.0),
                linear_velocity_m_per_s=(0.0, -0.5, 0.0),
            ),
            AxleBody(
                name="right",
                mass_kg=7.0,
                inertia_kg_m2=inertia,
                position_m=(0.0, 0.1, 0.0),
                linear_velocity_m_per_s=(0.0, 0.2, 0.0),
            ),
        ),
        joints=(),
        springs=(
            AxleSpringDamper(
                name="coupling",
                body_a="left",
                body_b="right",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=5_000.0,
                compression_damping_n_s_per_m=40.0,
                rebound_damping_n_s_per_m=40.0,
                free_length_m=0.25,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="action-reaction",
            times_s=_grid(0.1),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.0001,
            ),
        ),
    )

    left = 3.0 * result.body_state("left")[:, 7:10]
    right = 7.0 * result.body_state("right")[:, 7:10]
    momentum = left + right
    scale = float(np.max(np.abs(np.concatenate((left, right)))))

    assert float(np.max(np.abs(momentum - momentum[0]))) / scale < 1e-8
    assert float(np.ptp(result.body_state("left")[:, 8])) > 0.1


def _pendulum(length_m: float = 0.4) -> AxleDynamicsModel:
    inertia = ((0.05, 0.0, 0.0), (0.0, 0.05, 0.0), (0.0, 0.0, 0.05))
    return AxleDynamicsModel(
        name="pendulum",
        gravity_m_per_s2=(0.0, 0.0, -9.80665),
        bodies=(
            _FIXTURE,
            AxleBody(
                name="bob",
                mass_kg=2.0,
                inertia_kg_m2=inertia,
                position_m=(0.0, length_m, 0.0),
            ),
        ),
        joints=(
            AxleJoint(
                name="pivot",
                kind="spherical",
                body_a="fixture",
                body_b="bob",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, -length_m, 0.0),
            ),
        ),
    )


def _run_pendulum(
    internal_step_s: float, duration_s: float, rho_inf: float = 1.0
) -> AxleDynamicsResult:
    return run_axle_dynamics(
        _pendulum(),
        AxleDynamicsCase(
            name="pendulum",
            times_s=_grid(duration_s),
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
def test_long_run_holds_position_and_velocity_constraints_at_the_frozen_gate(
    rho_inf: float,
) -> None:
    gate = load_axle_acceptance_contract()["solver_internal_gates"]
    result = _run_pendulum(0.0005, 1.0, rho_inf)

    assert float(np.max(result.diagnostics.position_residual)) <= float(
        gate["constraint_position_m"]
    )
    assert float(np.max(result.diagnostics.velocity_residual)) <= float(
        gate["constraint_velocity_m_per_s"]
    )
    quaternion_error = float(
        np.max(np.abs(np.linalg.norm(result.states[:, :, 3:7], axis=2) - 1.0))
    )
    assert quaternion_error <= float(gate["quaternion_norm_error"])
    # The pendulum must have swung, otherwise the residuals are trivially zero.
    assert float(np.ptp(result.body_state("bob")[:, 2])) > 0.05


def test_step_halving_shows_second_order_convergence() -> None:
    """Halving the internal step must cut the state error by about four."""
    duration_s = 0.4
    coarse = _run_pendulum(0.0005, duration_s).body_state("bob")[:, :3]
    medium = _run_pendulum(0.00025, duration_s).body_state("bob")[:, :3]
    reference = _run_pendulum(0.00002, duration_s).body_state("bob")[:, :3]

    coarse_error = float(np.sqrt(np.mean(np.square(coarse - reference))))
    medium_error = float(np.sqrt(np.mean(np.square(medium - reference))))

    assert coarse_error > 0.0
    assert coarse_error / medium_error == pytest.approx(4.0, rel=0.25)
    convergence = load_axle_acceptance_contract()["solver_internal_gates"][
        "time_convergence"
    ]
    travel = float(np.max(reference) - np.min(reference))
    assert medium_error / travel <= float(
        convergence["state_nrmse_h_vs_h2_max"]
    )
