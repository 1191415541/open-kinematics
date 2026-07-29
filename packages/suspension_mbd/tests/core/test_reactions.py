"""Rank and Lagrange reaction tests."""

import numpy as np

from suspension_mbd.core import (
    SE3,
    BallJoint,
    ConstraintSystem,
    RigidBody,
    RigidBodyState,
    body_equilibrium_wrench,
    diagnose_rank,
    recover_reactions,
)


def test_rank_diagnostic_detects_under_and_over_constraint() -> None:
    under = diagnose_rank(np.eye(2, 6))
    over = diagnose_rank(np.vstack((np.eye(2, 2), np.eye(2, 2))))
    assert under.underconstrained
    assert over.overconstrained


def test_reaction_wrench_is_balanced_between_ball_joint_bodies() -> None:
    state = RigidBodyState(
        {
            "a": RigidBody("a", SE3.identity()),
            "b": RigidBody("b", SE3(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))),
        }
    )
    system = ConstraintSystem((BallJoint("a", [0, 0, 0], "b", [0, 0, 0]),))
    reactions = recover_reactions(system, state, np.array([10.0, -4.0, 3.0]))
    total = body_equilibrium_wrench(reactions.body_global_wrenches)
    assert np.allclose(total, np.zeros(6))
