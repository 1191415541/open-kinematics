"""Dynamic integrator tests."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.core import CoordinateDrive, RigidBody, RigidBodyState
from suspension_multibody.dynamics import DynamicIntegrator, DynamicRigidBodyState
from suspension_multibody.schema import DynamicSolverSettings


def _body_state() -> DynamicRigidBodyState:
    return DynamicRigidBodyState(
        RigidBodyState(
            {
                "body": RigidBody(
                    "body",
                    mass=2.0,
                    inertia=np.diag([2.0, 2.0, 2.0]),
                )
            }
        )
    )


def test_integrator_applies_constant_force_to_velocity() -> None:
    integrator = DynamicIntegrator(
        DynamicSolverSettings(
            end_time=1.0,
            step_size=0.1,
            gravity={"x": 0.0, "y": 0.0, "z": 0.0},
        )
    )

    results = integrator.integrate(
        _body_state(),
        external_wrenches=lambda _time, _state: {
            "body": np.array([4.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        },
    )

    assert results[-1].state.velocity("body")[0] == pytest.approx(2.0)


def test_integrator_projects_coordinate_drive_position_and_velocity() -> None:
    integrator = DynamicIntegrator(
        DynamicSolverSettings(
            end_time=0.1,
            step_size=0.1,
            gravity={"x": 0.0, "y": 0.0, "z": 0.0},
        )
    )
    state = DynamicRigidBodyState(
        _body_state().pose_state,
        velocities={"body": np.array([1.0, 0.0, 0.0, 0.0, 0.0, 0.0])},
    )

    results = integrator.integrate(
        state,
        constraints=(
            CoordinateDrive(
                "body",
                np.zeros(3),
                np.array([1.0, 0.0, 0.0]),
                target=0.0,
            ),
        ),
    )

    assert results[-1].state.pose_state.pose("body").translation[0] == pytest.approx(0.0)
    assert results[-1].state.velocity("body")[0] == pytest.approx(0.0)
