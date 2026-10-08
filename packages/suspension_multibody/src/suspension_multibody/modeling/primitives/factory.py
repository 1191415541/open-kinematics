"""Materialise physical declarations without interpreting subsystem roles."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .joints import (
    BallJoint,
    ConstantVelocityJoint,
    Constraint,
    CylindricalJoint,
    InPlaneJoint,
    PrismaticJoint,
    RevoluteJoint,
    RigidBody,
    UniversalJoint,
    WeldJoint,
)
from .spatial import SE3


def body_from_data(row: Mapping[str, Any]) -> RigidBody:
    """Create a rigid body from a resolved physical declaration."""
    return RigidBody(
        name=str(row["name"]),
        mass=float(row.get("mass", 0.0)),
        fixed=bool(row.get("fixed", False)),
        pose=SE3(
            np.asarray(row.get("position", (0.0, 0.0, 0.0)), dtype=float),
            np.asarray(row.get("quaternion", (1.0, 0.0, 0.0, 0.0)), dtype=float),
        ),
        inertia=np.asarray(row.get("inertia", np.eye(3)), dtype=float),
        center_of_mass=np.asarray(row.get("center_of_mass", (0.0, 0.0, 0.0)), dtype=float),
    )


def joint_from_data(row: Mapping[str, Any]) -> Constraint:
    """Create a joint from resolved body-local endpoints and axes."""
    kind = str(row["type"])
    constructors = {
        "spherical": BallJoint,
        "fixed": WeldJoint,
        "revolute": RevoluteJoint,
        "prismatic": PrismaticJoint,
        "universal": UniversalJoint,
        "cylindrical": CylindricalJoint,
        "inplane": InPlaneJoint,
        "convel": ConstantVelocityJoint,
    }
    try:
        constructor = constructors[kind]
    except KeyError as exc:
        raise ValueError(f"unsupported ideal joint kind {kind!r}") from exc
    values = {
        "name": str(row["name"]),
        "body_a": str(row["body_a"]),
        "point_a": np.asarray(row["point_a"], dtype=float),
        "body_b": str(row["body_b"]),
        "point_b": np.asarray(row["point_b"], dtype=float),
    }
    if kind not in {"spherical", "fixed"}:
        if "axis_a" not in row or (kind != "inplane" and "axis_b" not in row):
            raise ValueError(f"unsupported ideal joint kind {kind!r}")
        values["axis_a"] = np.asarray(row["axis_a"], dtype=float)
        if kind != "inplane":
            values["axis_b"] = np.asarray(row["axis_b"], dtype=float)
        if kind == "convel":
            values.update(axis_a_secondary=np.asarray(row["axis_a_secondary"], dtype=float),
                axis_b_secondary=np.asarray(row["axis_b_secondary"], dtype=float),
                angle_target=float(row.get("convel_angle_target", 0)))
    return constructor(**values)  # type: ignore[arg-type]
