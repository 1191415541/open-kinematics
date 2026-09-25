"""
The low layer: declarations and spatial algebra, and nothing above them.

Nothing in this package may import ``templates``, ``subsystems``, ``rigs``,
``connections``, ``preparation``, ``simulation``, ``kernel`` or ``report``.  A
declaration has to be usable without loading the authoring chain that assembles
it -- that is the property this layer exists to provide, and
``tests/architecture/test_import_boundaries.py`` enforces it.
"""

from .joints import (
    SE3,
    Array,
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
    RigidBody,
    RigidBodyState,
    UniversalJoint,
    WeldJoint,
)
from .spatial import (
    cross3,
    normalize_quaternion,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_matrix,
    quaternion_to_rotation_vector,
    rotation_vector_to_quaternion,
    skew,
    wrench_global_to_local,
)

__all__ = [
    "Array",
    "BallJoint",
    "ConstantVelocityJoint",
    "Constraint",
    "CoordinateDrive",
    "CylindricalJoint",
    "DistanceConstraint",
    "InPlaneJoint",
    "PointCoincidence",
    "PrismaticJoint",
    "RevoluteJoint",
    "RigidBody",
    "RigidBodyState",
    "SE3",
    "UniversalJoint",
    "WeldJoint",
    "cross3",
    "normalize_quaternion",
    "quaternion_conjugate",
    "quaternion_multiply",
    "quaternion_to_matrix",
    "quaternion_to_rotation_vector",
    "rotation_vector_to_quaternion",
    "skew",
    "wrench_global_to_local",
]
