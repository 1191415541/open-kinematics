"""Static KKT equilibrium tests."""

import numpy as np

from suspension_multibody.core import SE3, BallJoint, RigidBody, RigidBodyState
from suspension_multibody.elements import LinearSpringElement
from suspension_multibody.solver import EquilibriumSettings, EquilibriumSolver


def test_kkt_uses_ideal_joint_reactions_for_force_balance() -> None:
    state = RigidBodyState(
        {
            "chassis": RigidBody("chassis", fixed=True),
            "upright": RigidBody(
                "upright", SE3(np.array([0.0, 0.0, 1.0]), np.array([1.0, 0, 0, 0]))
            ),
        }
    )
    result = EquilibriumSolver().solve(
        state,
        constraints=(BallJoint("chassis", [0, 0, 1], "upright", [0, 0, 0]),),
        external_wrenches_global={"upright": np.array([0, 0, 10.0, 0, 0, 0])},
    )
    assert result.converged
    assert result.constraint_residual < 1e-8


def test_free_body_spring_equilibrium_reaches_free_length() -> None:
    state = RigidBodyState(
        {
            "chassis": RigidBody("chassis", fixed=True),
            "body": RigidBody(
                "body", SE3(np.array([0.0, 0.0, 2.0]), np.array([1.0, 0, 0, 0]))
            ),
        }
    )
    spring = LinearSpringElement(
        "spring", "chassis", [0, 0, 0], "body", [0, 0, 0], 100, free_length=1
    )
    result = EquilibriumSolver(EquilibriumSettings(force_tolerance=1e-5)).solve(
        state, elements=(spring,)
    )
    assert result.converged
    assert np.isclose(result.state.pose("body").translation[2], 1.0, atol=1e-5)
