"""
Shared fixtures and helpers for the p3-02 evidence scripts.  Not a product module:
it only builds the double-wishbone axle the tests use, and the planar four-bar used
to isolate the front-view reduction.

Run through the sibling ``probe_*.py`` scripts.
"""

from __future__ import annotations

import numpy as np

from suspension_multibody.modeling.primitives import (
    BallJoint,
    RevoluteJoint,
    RigidBody,
    RigidBodyState,
)
from suspension_multibody.schema import (
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    Vec3,
)

#: The double-wishbone hardpoint geometry ``tests/conftest.py::full_vehicle_model``
#: declares for one axle.  Copied here so the evidence scripts do not have to call a
#: pytest fixture.
_HARDPOINTS = (
    ("UPPER_INBOARD_FRONT", 0.0, -500.0, 500.0),
    ("UPPER_INBOARD_REAR", 150.0, -500.0, 500.0),
    ("UPPER_OUTBOARD", 0.0, -750.0, 350.0),
    ("LOWER_INBOARD_FRONT", 0.0, -500.0, 100.0),
    ("LOWER_INBOARD_REAR", 150.0, -500.0, 100.0),
    ("LOWER_OUTBOARD", 0.0, -750.0, 100.0),
    ("TIE_ROD_INBOARD", 0.0, -450.0, 250.0),
    ("TIE_ROD_OUTBOARD", 0.0, -750.0, 250.0),
    ("WHEEL_CENTER", 0.0, -750.0, 300.0),
    ("RACK_CENTER", 0.0, 0.0, 250.0),
)

_BODIES = (
    "rack",
    "upper_arm_L",
    "upper_arm_R",
    "lower_arm_L",
    "lower_arm_R",
    "upright_L",
    "upright_R",
    "tie_rod_L",
    "tie_rod_R",
)


def double_wishbone_axle_model(
    name: str = "front", x: float = 1400.0, **overrides: tuple[float, float, float]
) -> FrontAxleModel:
    """Return the fixture axle's model, with optional hardpoint overrides."""
    hardpoints = {
        point: Vec3(x=x + dx, y=dy, z=dz) for point, dx, dy, dz in _HARDPOINTS
    }
    for point, (px, py, pz) in overrides.items():
        hardpoints[point] = Vec3(x=px, y=py, z=pz)
    return FrontAxleModel(
        name=name,
        hardpoints=hardpoints,
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(RigidBodySpec(name=body, mass=10.0) for body in _BODIES),
    )


def two_d_intersection(
    upper_inner: tuple[float, float],
    upper_outer: tuple[float, float],
    lower_inner: tuple[float, float],
    lower_outer: tuple[float, float],
) -> np.ndarray:
    """The front-view intersection of the two arm lines, ``[y, z]``.

    This is exactly the arithmetic ``vehicle/roll_centers.py::_line_intersection``
    performs on the two arm lines (``:117-129``): solve the 2x2 system
    ``d_a * s - d_b * t = c - a`` for ``s`` and walk ``s`` along the first line.
    """
    direction_a = np.asarray(upper_outer) - np.asarray(upper_inner)
    direction_b = np.asarray(lower_outer) - np.asarray(lower_inner)
    matrix = np.column_stack((direction_a, -direction_b))
    parameters = np.linalg.solve(matrix, np.asarray(lower_inner) - np.asarray(upper_inner))
    return np.asarray(upper_inner, dtype=float) + parameters[0] * direction_a


def planar_four_bar(
    upper_inner: tuple[float, float],
    upper_outer: tuple[float, float],
    lower_inner: tuple[float, float],
    lower_outer: tuple[float, float],
) -> tuple[tuple[object, ...], RigidBodyState]:
    """A four-bar lying in the ``x = 0`` plane, whose arm axes are exactly along x.

    Both arms pivot on the ground about a line strictly parallel to x through their
    inboard point, so the two arm lines of the front view are the *exact* lines of
    motion and the planar construction is the exact answer for this mechanism.  That
    makes it the place where a difference between the engine and the planar
    construction can only come from the engine, not from the planar reduction.
    """
    bodies = {
        "ground": RigidBody("ground", fixed=True),
        "upper": RigidBody("upper"),
        "lower": RigidBody("lower"),
        "upright": RigidBody("upright"),
    }
    state = RigidBodyState(bodies)
    axis = np.array([1.0, 0.0, 0.0])
    constraints = (
        RevoluteJoint(
            "ground", np.array([0.0, *upper_inner]), axis,
            "upper", np.array([0.0, *upper_inner]), axis,
        ),
        RevoluteJoint(
            "ground", np.array([0.0, *lower_inner]), axis,
            "lower", np.array([0.0, *lower_inner]), axis,
        ),
        BallJoint("upper", np.array([0.0, *upper_outer]), "upright", np.array([0.0, *upper_outer])),
        BallJoint("lower", np.array([0.0, *lower_outer]), "upright", np.array([0.0, *lower_outer])),
    )
    return constraints, state
