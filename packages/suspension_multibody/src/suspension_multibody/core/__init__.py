"""
Core spatial algebra, rigid-body state and joint declarations.

The package survives the 08 deletion because ``elements/`` and the authoring
layer depend on ``core/spatial.py`` and ``core/rigid_body.py``, and
``core/constraints.py`` carries the declaration re-exports plus the
residual/Jacobian kernel.  ``core/rank.py`` and ``core/reactions.py`` had no
production caller and were deleted; their physical assertions are asserted
against the native contract in ``tests/axle_dynamics/test_solver_invariants.py``.
"""

from .constraints import (
    BallJoint,
    ConstantVelocityJoint,
    Constraint,
    ConstraintSystem,
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
from .rigid_body import RigidBody, RigidBodyState, body_point_wrench
from .spatial import (
    SE3,
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_matrix,
    quaternion_to_rotation_vector,
    rotation_vector_to_quaternion,
    skew,
    twist_local_to_global,
    wrench_global_to_local,
    wrench_local_to_global,
    wrench_matrix,
    wrench_translation_tangent,
)

__all__ = [
    "RigidBody",
    "RigidBodyState",
    "SE3",
    "BallJoint",
    "ConstantVelocityJoint",
    "CylindricalJoint",
    "Constraint",
    "ConstraintSystem",
    "CoordinateDrive",
    "DistanceConstraint",
    "InPlaneJoint",
    "PointCoincidence",
    "PrismaticJoint",
    "RevoluteJoint",
    "WeldJoint",
    "UniversalJoint",
    "body_point_wrench",
    "normalize_quaternion",
    "quaternion_conjugate",
    "quaternion_multiply",
    "quaternion_to_matrix",
    "quaternion_to_rotation_vector",
    "rotation_vector_to_quaternion",
    "skew",
    "twist_local_to_global",
    "wrench_global_to_local",
    "wrench_local_to_global",
    "wrench_matrix",
    "wrench_translation_tangent",
]
