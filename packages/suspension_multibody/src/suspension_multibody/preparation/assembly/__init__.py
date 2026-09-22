"""
The author-side assembly package: hardpoints, declarations and the vehicle
topology, in the shapes the contract documents are authored from.

``front_axle`` and ``vehicle`` mirror and name; ``types`` holds the declaration
data they produce.  Nothing in this package solves, submits native or decodes a
result: the assembly is authored here and solved in the kernel.
"""

from .front_axle import (
    Connection,
    FrontAxleAssembly,
    build_front_axle,
    mirror_hardpoints,
    side_hardpoints,
)
from .types import (
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
from .vehicle import VehicleAssembly, build_vehicle

__all__ = [
    "BallJoint",
    "Connection",
    "ConstantVelocityJoint",
    "Constraint",
    "CoordinateDrive",
    "CylindricalJoint",
    "DistanceConstraint",
    "FrontAxleAssembly",
    "InPlaneJoint",
    "PointCoincidence",
    "PrismaticJoint",
    "RevoluteJoint",
    "RigidBody",
    "RigidBodyState",
    "UniversalJoint",
    "VehicleAssembly",
    "WeldJoint",
    "build_front_axle",
    "build_vehicle",
    "mirror_hardpoints",
    "side_hardpoints",
]
