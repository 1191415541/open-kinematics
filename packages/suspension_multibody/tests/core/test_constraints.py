"""Ideal constraint residual and analytic Jacobian tests."""

import numpy as np

from suspension_multibody.core import (
    SE3,
    BallJoint,
    ConstantVelocityJoint,
    ConstraintSystem,
    CoordinateDrive,
    CylindricalJoint,
    DistanceConstraint,
    InPlaneJoint,
    PrismaticJoint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
    UniversalJoint,
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


def test_extended_joint_jacobians_match_local_retraction() -> None:
    state = RigidBodyState(
        {
            "a": RigidBody("a", SE3.identity()),
            "b": RigidBody(
                "b",
                SE3(
                    np.array([0.2, 0.3, 0.4]),
                    np.array([0.98, 0.1, -0.05, 0.15]),
                ),
            ),
        }
    )
    constraints = (
        UniversalJoint(
            "a", [0.1, 0.2, 0.3], [0.0, 0.0, 1.0],
            "b", [-0.2, 0.1, 0.05], [1.0, 0.0, 0.0],
        ),
        CylindricalJoint(
            "a", [0.1, 0.2, 0.3], [0.0, 0.0, 1.0],
            "b", [-0.2, 0.1, 0.05], [0.0, 0.0, 1.0],
        ),
        InPlaneJoint(
            "a", [0.1, 0.2, 0.3], [0.0, 0.0, 1.0],
            "b", [-0.2, 0.1, 0.05],
        ),
        ConstantVelocityJoint(
            "a", [0.1, 0.2, 0.3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
            "b", [-0.2, 0.1, 0.05], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],
        ),
    )
    step = 1e-7
    for constraint in constraints:
        analytic = constraint.jacobian(state)
        for body in ("a", "b"):
            numeric = np.empty((constraint.residual(state).size, 6))
            for column in range(6):
                increment = np.zeros(6)
                increment[column] = step
                numeric[:, column] = (
                    constraint.residual(state.retract({body: increment}))
                    - constraint.residual(state.retract({body: -increment}))
                ) / (2.0 * step)
            np.testing.assert_allclose(numeric, analytic[body], atol=1e-7, rtol=1e-7)
def test_drive_and_system_assembly() -> None:
    state = _state()
    system = ConstraintSystem((CoordinateDrive("b", [0, 0, 0], [0, 0, 1], 2.0),))
    assert np.isclose(system.residual(state)[0], -1.0)
    assert system.jacobian(state).shape == (1, 12)
