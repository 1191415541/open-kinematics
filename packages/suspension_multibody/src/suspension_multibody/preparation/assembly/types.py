"""
Author-side data objects: the joint declarations and rigid-body data an
assembly is described with.

A declaration is data and nothing else.  A joint carries its two bodies, their
points and its axes; a rigid body carries its pose and its mass properties.
Neither carries a *residual*, a *Jacobian* or an *evaluate*: those are the
solving side of the boundary, and the native kernel owns them now.  The
residual/Jacobian implementation over these objects is the only thing left in
``core/constraints.py``, which no production path calls any more.

The bodies were ``core/constraints.py`` and ``core/rigid_body.py`` data fields
moved verbatim, with the pose algebra they need taken from
``preparation/geometry.py``; 08 deletes the legacy modules.
"""

from __future__ import annotations

from abc import ABC
from dataclasses import dataclass, field

import numpy as np

from ..geometry import SE3, Array


class Constraint(ABC):
    """
    Base class of the joint declarations.

    It stays an ``ABC`` so the declaration hierarchy is unchanged, but it
    declares no abstract method: a residual or a Jacobian is not part of a
    constraint's data, so there is nothing here for a subclass to implement.
    """

    name: str


@dataclass(frozen=True)
class PointCoincidence(Constraint):
    """Three position constraints between two body points."""

    body_a: str
    point_a: Array
    body_b: str
    point_b: Array
    name: str = "point_coincidence"


@dataclass(frozen=True)
class BallJoint(PointCoincidence):
    """Ideal spherical joint."""

    name: str = "ball_joint"


@dataclass(frozen=True)
class WeldJoint(Constraint):
    """Six-constraint rigid connection that preserves relative pose."""

    body_a: str
    point_a: Array
    body_b: str
    point_b: Array
    name: str = "weld_joint"


@dataclass(frozen=True)
class DistanceConstraint(Constraint):
    """One scalar fixed-distance constraint."""

    body_a: str
    point_a: Array
    body_b: str
    point_b: Array
    distance: float
    name: str = "distance"


@dataclass(frozen=True)
class RevoluteJoint(Constraint):
    """Five-constraint ideal revolute joint."""

    body_a: str
    point_a: Array
    axis_a: Array
    body_b: str
    point_b: Array
    axis_b: Array
    name: str = "revolute_joint"


@dataclass(frozen=True)
class UniversalJoint(Constraint):
    """Four-constraint Hooke/universal joint with coincident centers."""

    body_a: str
    point_a: Array
    axis_a: Array
    body_b: str
    point_b: Array
    axis_b: Array
    name: str = "universal_joint"


@dataclass(frozen=True)
class ConstantVelocityJoint(Constraint):
    """按 Adams 定义实现的四约束 CONVEL 关节."""

    body_a: str
    point_a: Array
    axis_a: Array
    axis_a_secondary: Array
    body_b: str
    point_b: Array
    axis_b: Array
    axis_b_secondary: Array
    name: str = "constant_velocity_joint"
    angle_target: float = 0.0


@dataclass(frozen=True)
class CylindricalJoint(Constraint):
    """Four-constraint cylindrical joint allowing axial slide and spin."""

    body_a: str
    point_a: Array
    axis_a: Array
    body_b: str
    point_b: Array
    axis_b: Array
    name: str = "cylindrical_joint"


@dataclass(frozen=True)
class InPlaneJoint(Constraint):
    """One-constraint joint keeping body-B's point in body-A's plane."""

    body_a: str
    point_a: Array
    axis_a: Array
    body_b: str
    point_b: Array
    name: str = "inplane_joint"


@dataclass(frozen=True)
class PrismaticJoint(Constraint):
    """Five-constraint ideal prismatic joint along a body-A axis."""

    body_a: str
    point_a: Array
    axis_a: Array
    body_b: str
    point_b: Array
    axis_b: Array
    name: str = "prismatic_joint"


@dataclass(frozen=True)
class CoordinateDrive(Constraint):
    """Scalar point-coordinate displacement drive."""

    body: str
    point: Array
    axis: Array
    target: float
    name: str = "coordinate_drive"


@dataclass(frozen=True)
class RigidBody:
    """Mass properties and initial pose for one rigid body."""

    name: str
    pose: SE3 = field(default_factory=SE3.identity)
    mass: float = 0.0
    inertia: Array = field(default_factory=lambda: np.eye(3))
    center_of_mass: Array = field(default_factory=lambda: np.zeros(3))
    fixed: bool = False

    def __post_init__(self) -> None:
        inertia = np.asarray(self.inertia, dtype=float)
        center_of_mass = np.asarray(self.center_of_mass, dtype=float)
        if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)):
            raise ValueError("inertia must be a finite 3x3 matrix")
        if center_of_mass.shape != (3,) or not np.all(np.isfinite(center_of_mass)):
            raise ValueError("center_of_mass must contain three finite values")
        if self.mass < 0 or not np.isfinite(self.mass):
            raise ValueError("mass must be finite and non-negative")
        object.__setattr__(self, "inertia", inertia.copy())
        object.__setattr__(self, "center_of_mass", center_of_mass.copy())

    def _with_pose_unchecked(self, pose: SE3) -> RigidBody:
        """Reuse validated immutable body data for an integrator trial pose."""
        updated = object.__new__(type(self))
        object.__setattr__(updated, "name", self.name)
        object.__setattr__(updated, "pose", pose)
        object.__setattr__(updated, "mass", self.mass)
        object.__setattr__(updated, "inertia", self.inertia)
        object.__setattr__(updated, "center_of_mass", self.center_of_mass)
        object.__setattr__(updated, "fixed", self.fixed)
        return updated


@dataclass(frozen=True)
class RigidBodyState:
    """Immutable collection of body poses with local increment retraction."""

    bodies: dict[str, RigidBody]

    def __post_init__(self) -> None:
        if len(self.bodies) != len(set(self.bodies)):
            raise ValueError("body names must be unique")

    def pose(self, body: str) -> SE3:
        """Return a body's current pose."""
        try:
            return self.bodies[body].pose
        except KeyError as exc:
            raise KeyError(f"unknown body {body!r}") from exc

    def point_world(self, body: str, point_local: Array) -> Array:
        """Return a local body point in global coordinates."""
        return self.pose(body).transform_point(point_local)

    def retract(self, increments: dict[str, Array]) -> RigidBodyState:
        """Apply per-body local increments and return a new state."""
        updated: dict[str, RigidBody] = {}
        for name, body in self.bodies.items():
            increment = increments.get(name)
            if increment is None or body.fixed:
                updated[name] = body
            else:
                updated[name] = RigidBody(
                    name=body.name,
                    pose=body.pose.retract(increment),
                    mass=body.mass,
                    inertia=body.inertia,
                    center_of_mass=body.center_of_mass,
                    fixed=body.fixed,
                )
        return RigidBodyState(updated)

    def retract_unchecked(self, increments: dict[str, Array]) -> RigidBodyState:
        """Apply trusted trial increments without rebuilding mass metadata."""
        if not increments:
            return self
        updated: dict[str, RigidBody] = {}
        for name, body in self.bodies.items():
            increment = increments.get(name)
            if increment is None or body.fixed:
                updated[name] = body
            else:
                updated[name] = body._with_pose_unchecked(
                    body.pose._retract_unchecked(increment)
                )
        return RigidBodyState(updated)


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
]
