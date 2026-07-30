"""Ideal constraint residual and analytic Jacobian tests."""

import numpy as np

from suspension_multibody.core import (
    SE3,
    BallJoint,
    ConstraintSystem,
    CoordinateDrive,
    DistanceConstraint,
    PrismaticJoint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
)


def _state() -> RigidBodyState:
    return RigidBodyState(
        {
            "a": RigidBody("a", SE3.identity()),
            "b": RigidBody(
                "b", SE3(np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0, 0.0]))
            ),
        }
    )


def test_ball_joint_residual_and_jacobian() -> None:
    state = _state()
    joint = BallJoint("a", [0, 0, 0], "b", [0, 0, 0])
    assert np.allclose(joint.residual(state), [0, 0, -1])
    assert joint.jacobian(state)["a"].shape == (3, 6)


def test_distance_constraint_has_expected_sign() -> None:
    state = _state()
    constraint = DistanceConstraint("a", [0, 0, 0], "b", [0, 0, 0], 0.5)
    assert np.isclose(constraint.residual(state)[0], 0.5)
    assert np.isclose(constraint.jacobian(state)["a"][0, 2], -1.0)


def test_revolute_and_prismatic_constraints_have_five_rows() -> None:
    state = _state()
    revolute = RevoluteJoint("a", [0, 0, 0], [0, 0, 1], "b", [0, 0, 0], [0, 0, 1])
    prismatic = PrismaticJoint("a", [0, 0, 0], [0, 0, 1], "b", [0, 0, 0], [0, 0, 1])
    assert revolute.residual(state).shape == (5,)
    assert revolute.jacobian(state)["a"].shape == (5, 6)
    assert prismatic.residual(state).shape == (5,)
    assert prismatic.jacobian(state)["b"].shape == (5, 6)


def test_drive_and_system_assembly() -> None:
    state = _state()
    system = ConstraintSystem((CoordinateDrive("b", [0, 0, 0], [0, 0, 1], 2.0),))
    assert np.isclose(system.residual(state)[0], -1.0)
    assert system.jacobian(state).shape == (1, 12)
