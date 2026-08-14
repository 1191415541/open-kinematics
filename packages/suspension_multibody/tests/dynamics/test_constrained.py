"""Coupled rigid-body dynamics consistency tests."""

import numpy as np
import pytest

from suspension_multibody.core import CoordinateDrive, RigidBody, RigidBodyState
from suspension_multibody.dynamics import (
    ConstrainedDynamicIntegrator,
    DynamicRigidBodyState,
)
from suspension_multibody.schema import DynamicSolverSettings, Vec3


def test_free_asymmetric_body_includes_gyroscopic_bias() -> None:
    inertia = np.diag([2.0, 3.0, 4.0])
    state = DynamicRigidBodyState(
        RigidBodyState({"body": RigidBody("body", mass=1.0, inertia=inertia)}),
        velocities={"body": np.array([0.0, 0.0, 0.0, 1.0, 2.0, 3.0])},
    )
    settings = DynamicSolverSettings(
        end_time=0.01,
        step_size=0.01,
        gravity=Vec3(),
    )
    result = ConstrainedDynamicIntegrator(settings).integrate(state)
    angular_velocity = np.array([1.0, 2.0, 3.0])
    expected = -np.cross(angular_velocity, inertia @ angular_velocity)
    expected = np.linalg.solve(inertia, expected)

    assert np.allclose(result[1].state.accelerations["body"][3:], expected)
    assert not np.allclose(result[1].state.accelerations["body"][3:], 0.0)


@pytest.mark.parametrize("integrator", ("semi_implicit_euler", "newmark", "generalized_alpha"))
def test_constrained_dynamics_solves_reaction_multiplier(integrator: str) -> None:
    state = DynamicRigidBodyState(
        RigidBodyState({"body": RigidBody("body", mass=2.0, inertia=np.eye(3))})
    )
    result = ConstrainedDynamicIntegrator(
        DynamicSolverSettings(
            end_time=0.01,
            step_size=0.01,
            gravity=Vec3(),
            integrator=integrator,
        )
    ).integrate(
        state,
        constraints=(
            CoordinateDrive(
                "body", np.zeros(3), np.array([1.0, 0.0, 0.0]), target=0.0
            ),
        ),
        external_wrenches=lambda _time, _state: {
            "body": np.array([4.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        },
    )

    final = result[-1].state
    assert final.velocity("body")[0] == pytest.approx(0.0)
    assert final.multipliers.size == 1
    assert abs(final.multipliers[0]) > 1.0


def test_failed_step_records_candidate_constraint_context(monkeypatch) -> None:
    state = DynamicRigidBodyState(
        RigidBodyState({"body": RigidBody("body", mass=1.0, inertia=np.eye(3))})
    )
    settings = DynamicSolverSettings(
        end_time=0.01,
        step_size=0.01,
        internal_step_size=0.01,
        min_internal_step_size=1.0e-6,
        projection_failure_tolerance=1.0e-6,
        gravity=Vec3(),
    )
    integrator = ConstrainedDynamicIntegrator(settings)
    constraint = CoordinateDrive(
        "body", np.zeros(3), np.array([1.0, 0.0, 0.0]), target=0.0
    )
    candidate = state.retract_unchecked(
        {"body": np.array([1.0e-3, 0.0, 0.0, 0.0, 0.0, 0.0])}
    )

    def fail_trial(*_args, **_kwargs):
        return (
            candidate,
            {"body": np.zeros(6)},
            {"body": np.zeros(6)},
            np.array([2.0]),
            ("fake_event",),
        )

    monkeypatch.setattr(integrator, "_advance_trial", fail_trial)
    with pytest.raises(RuntimeError, match="cannot maintain the position manifold"):
        integrator.integrate(state, constraints=(constraint,))

    failure = integrator.last_failure
    assert failure is not None
    first_candidate = failure["first_candidate"]
    assert isinstance(first_candidate, dict)
    assert first_candidate["position_residual"] == pytest.approx(1.0e-3)
    assert first_candidate["constraint_residual"] == pytest.approx([1.0e-3])
    assert first_candidate["constraint_rows"] == [
        {
            "constraint_index": 0,
            "name": "coordinate_drive",
            "start": 0,
            "stop": 1,
            "max_abs": pytest.approx(1.0e-3),
        }
    ]
    assert first_candidate["events"] == ("fake_event",)
