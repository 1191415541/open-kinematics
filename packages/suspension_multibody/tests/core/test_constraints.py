"""
Joint-declaration tests: the data an assembly is described with.

The joint *declarations* live in ``preparation/assembly/types.py`` -- the
authoring layer builds them, so this is their live home.  What used to sit
beside them in ``core/constraints.py`` was the residual/Jacobian implementation
and ``ConstraintSystem``; subtask 08 deleted both, because the native kernel
owns the solving side and nothing in the production tree called them any more.

The physics those solvers asserted did not disappear with them: the joint row
counts each kind contributes are asserted against the native contract by
``tests/vehicle/test_native_vehicle.py`` (``fixed`` is a two-run joint) and the
solver-level behaviour by ``tests/axle_dynamics/test_solver_invariants.py``.
What remains here is what these declarations must carry on their own.
"""

import numpy as np

from suspension_multibody.preparation.assembly.types import (
    BallJoint,
    ConstantVelocityJoint,
    Constraint,
    CoordinateDrive,
    CylindricalJoint,
    DistanceConstraint,
    InPlaneJoint,
    PointCoincidence,
    PrismaticJoint,
    RevoluteJoint,
    UniversalJoint,
    WeldJoint,
)


def test_a_declaration_carries_its_bodies_points_and_axes() -> None:
    joint = RevoluteJoint("a", [0, 0, 0], [0, 0, 1], "b", [0, 0, 0], [0, 0, 1])

    assert isinstance(joint, Constraint)
    assert (joint.body_a, joint.body_b) == ("a", "b")
    assert np.allclose(joint.point_a, [0, 0, 0])
    assert np.allclose(joint.axis_b, [0, 0, 1])


def test_a_declaration_carries_no_solving_surface() -> None:
    """
    A declaration is data: no residual, no Jacobian, no evaluate.

    This is the boundary subtask 06 drew and subtask 08 kept -- the solving side
    is the native kernel's, and a declaration that grew a residual back would
    put the retired solver in the authoring layer.
    """
    assert not hasattr(Constraint, "residual")
    assert not hasattr(Constraint, "jacobian")
    assert not hasattr(Constraint, "evaluate")

    for kind in (
        PointCoincidence,
        BallJoint,
        WeldJoint,
        RevoluteJoint,
        PrismaticJoint,
        UniversalJoint,
        CylindricalJoint,
        InPlaneJoint,
        ConstantVelocityJoint,
        DistanceConstraint,
        CoordinateDrive,
    ):
        assert not hasattr(kind, "residual"), kind.__name__
        assert not hasattr(kind, "jacobian"), kind.__name__
        assert not hasattr(kind, "evaluate"), kind.__name__


def test_a_declaration_names_the_two_bodies_it_ties() -> None:
    for joint in (
        BallJoint("a", [0, 0, 0], "b", [0, 0, 0]),
        DistanceConstraint("a", [0, 0, 0], "b", [0, 0, 0], 0.5),
        InPlaneJoint("a", [0, 0, 0], [0, 0, 1], "b", [0, 0, 0]),
        UniversalJoint("a", [0, 0, 0], [0, 0, 1], "b", [0, 0, 0], [1, 0, 0]),
    ):
        assert joint.body_a == "a"
        assert joint.body_b == "b"
