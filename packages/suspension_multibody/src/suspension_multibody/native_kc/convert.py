"""
Convert a Python front-axle assembly into a native axle model.

This is the bridge the native K/C workflow stands on: the product's own front
axle assembly (ideal joints for K, compliant bushings for C) is expressed as an
``AxleDynamicsModel`` the kernel can solve, with identical geometry and units.

Two conversions are easy to get wrong and are therefore spelled out here:

* **lengths** -- the product model is millimetres, the kernel is SI, so every
  point and axis is scaled on the way in and the results are scaled back out;
* **driven targets** -- the kernel's driven-translation row is
  ``dot(dp, axis) - target``, i.e. the target is the *absolute* separation of the
  two points, not an increment from the assembling pose.  The design separation
  is computed here so a caller can pass a plain "wheel travel" like the Python
  K solver does.
"""

from __future__ import annotations

import numpy as np

from ..axle_dynamics import (
    AxleBody,
    AxleBushing,
    AxleDrivenCoordinate,
    AxleDynamicsModel,
    AxleJoint,
)
from ..core.constraints import BallJoint, PrismaticJoint, RevoluteJoint

MM = 1e-3
TRANSLATION_SCALE = 1e3   # N/mm      -> N/m
ROTATION_SCALE = 1e-3     # N*mm/rad  -> N*m/rad


class NativeKcError(RuntimeError):
    """Raised when an assembly cannot be expressed for the native kernel."""


def rotation_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Return the scalar-first quaternion for a rotation matrix (Shepperd)."""
    m = np.asarray(rotation, dtype=float)
    trace = float(m[0, 0] + m[1, 1] + m[2, 2])
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1] - m[1, 2]) / s, 0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2] - m[2, 0]) / s, (m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s
    else:
        s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0] - m[0, 1]) / s, (m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s
    quaternion = np.array([w, x, y, z], dtype=float)
    return tuple(float(v) for v in quaternion / np.linalg.norm(quaternion))


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    """Return the rotation matrix for a scalar-first quaternion."""
    w, x, y, z = (float(v) for v in quaternion)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _local_point(world: np.ndarray, body) -> np.ndarray:
    rotation = np.asarray(body.pose.rotation, dtype=float)
    offset = np.asarray(world, dtype=float) - np.asarray(body.pose.translation, dtype=float)
    return rotation.T @ offset


def _local_axis(world_axis: np.ndarray, body) -> np.ndarray:
    rotation = np.asarray(body.pose.rotation, dtype=float)
    axis = rotation.T @ np.asarray(world_axis, dtype=float)
    return axis / np.linalg.norm(axis)


def design_separation(assembly, body_name: str, point_local_mm, axis_local) -> float:
    """Signed separation of a driven coordinate at the assembling pose (SI)."""
    body = assembly.bodies[body_name]
    reaction = assembly.bodies["chassis"]
    rotation = np.asarray(body.pose.rotation, dtype=float)
    world_point = (
        np.asarray(body.pose.translation, dtype=float)
        + rotation @ np.asarray(point_local_mm, dtype=float)
    ) * MM
    reaction_rotation = np.asarray(reaction.pose.rotation, dtype=float)
    reaction_point = np.asarray(reaction.pose.translation, dtype=float) * MM
    axis_world = reaction_rotation @ np.asarray(axis_local, dtype=float)
    return float(np.dot(world_point - reaction_point, axis_world))


def collapse_spherical_pairs(constraints):
    """
    Fold two spherical joints on the same body pair into one revolute joint.

    Two coincident-point constraints between the same two rigid bodies remove the
    same five degrees of freedom as a revolute joint about the line through the
    two points, but with six rows of rank five.  The kernel requires independent
    constraint rows, so the compliant assembly's *rigid* reference -- which
    represents each arm mount that way -- is written in the equivalent,
    full-rank revolute form.  The solution manifold is unchanged (verified: the
    two forms agree to 1e-12 in the Python solver).
    """
    grouped: dict[tuple[str, str], list] = {}
    others = []
    for constraint in constraints:
        if isinstance(constraint, BallJoint):
            grouped.setdefault((constraint.body_a, constraint.body_b), []).append(constraint)
        else:
            others.append(constraint)
    joints = []
    for group in grouped.values():
        if len(group) == 1:
            joints.append(group[0])
            continue
        if len(group) != 2:
            raise NativeKcError(f"{len(group)} spherical joints share one body pair")
        first, second = group
        axis = np.asarray(second.point_a, dtype=float) - np.asarray(first.point_a, dtype=float)
        if np.linalg.norm(axis) <= 1e-9:
            raise NativeKcError("two mount points coincide; cannot form an axis")
        joints.append(
            RevoluteJoint(
                first.body_a, first.point_a, axis,
                first.body_b, first.point_b, axis,
                name=f"collapsed_{first.name}",
            )
        )
    return tuple(joints + others)


def _bodies(assembly) -> tuple[AxleBody, ...]:
    bodies = []
    for name, body in assembly.bodies.items():
        inertia = np.asarray(body.inertia, dtype=float) * MM**2
        if body.fixed:
            mass, inertia = 0.0, np.eye(3)
        else:
            # The quasi-static solve is load free; mass only conditions the
            # linear algebra, so a placeholder is used when the assembly does
            # not carry one (K-mode bodies are massless by design).
            mass = float(body.mass) if body.mass > 0.0 else 1.0
            if not np.all(np.isfinite(inertia)) or np.linalg.norm(inertia) < 1e-9:
                inertia = np.eye(3)
        bodies.append(
            AxleBody(
                name=name,
                mass_kg=mass,
                inertia_kg_m2=tuple(tuple(float(v) for v in row) for row in inertia),
                position_m=tuple(float(v) * MM for v in body.pose.translation),
                quaternion_body_to_world=rotation_to_quaternion(body.pose.rotation),
                fixed=bool(body.fixed),
            )
        )
    return tuple(bodies)


def _joints(assembly, constraints) -> tuple[AxleJoint, ...]:
    joints = []
    for constraint in constraints:
        body_a = assembly.bodies[constraint.body_a]
        body_b = assembly.bodies[constraint.body_b]
        point_a = _local_point(constraint.point_a, body_a) * MM
        point_b = _local_point(constraint.point_b, body_b) * MM
        if isinstance(constraint, BallJoint):
            kind, axis_a, axis_b = "spherical", (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)
        elif isinstance(constraint, RevoluteJoint):
            kind = "revolute"
            axis_a = tuple(float(v) for v in _local_axis(constraint.axis_a, body_a))
            axis_b = tuple(float(v) for v in _local_axis(constraint.axis_b, body_b))
        elif isinstance(constraint, PrismaticJoint):
            kind = "prismatic"
            axis_a = tuple(float(v) for v in _local_axis(constraint.axis_a, body_a))
            axis_b = tuple(float(v) for v in _local_axis(constraint.axis_b, body_b))
        else:
            raise NativeKcError(f"unsupported constraint {type(constraint).__name__}")
        joints.append(
            AxleJoint(
                name=constraint.name,
                kind=kind,
                body_a=constraint.body_a,
                body_b=constraint.body_b,
                point_a_m=tuple(float(v) for v in point_a),
                point_b_m=tuple(float(v) for v in point_b),
                axis_a=axis_a,
                axis_b=axis_b,
            )
        )
    return tuple(joints)


def _bushings(assembly) -> tuple[AxleBushing, ...]:
    bushings = []
    for element in assembly.bushings:
        stiffness = np.asarray(element.stiffness, dtype=float).copy()
        damping = np.asarray(element.damping, dtype=float).copy()
        for row in range(6):
            for column in range(6):
                # Each 3x3 block carries the unit of its own rows and columns.
                # A translational force per metre and a rotational moment per
                # radian convert by 1e3 and 1e-3; the two coupling blocks are a
                # force per radian and a moment per metre, which the kernel also
                # measures that way, so their factor is one.  Scaling them by
                # zero -- which is what this did -- silently deleted every
                # translation/rotation coupling term of every bushing.
                scale = (
                    TRANSLATION_SCALE
                    if row < 3 and column < 3
                    else ROTATION_SCALE
                    if row >= 3 and column >= 3
                    else 1.0
                )
                stiffness[row, column] *= scale
                damping[row, column] *= scale
        preload = np.asarray(element.preload, dtype=float).copy()
        preload[3:] *= ROTATION_SCALE
        bushings.append(
            AxleBushing(
                name=element.name,
                body_a=element.body_a,
                body_b=element.body_b,
                point_a_m=tuple(float(v) * MM for v in element.local_pose_a.translation),
                point_b_m=tuple(float(v) * MM for v in element.local_pose_b.translation),
                frame_a_to_body_quaternion=tuple(float(v) for v in element.local_pose_a.quaternion),
                frame_b_to_body_quaternion=tuple(float(v) for v in element.local_pose_b.quaternion),
                reference_translation_in_frame_a_m=(0.0, 0.0, 0.0),
                reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                stiffness=tuple(tuple(float(v) for v in row) for row in stiffness),
                damping=tuple(tuple(float(v) for v in row) for row in damping),
                preload_in_frame_a_n_n_m=tuple(float(v) for v in preload),
            )
        )
    return tuple(bushings)


def axle_drives(assembly, *, drive_wheels: bool) -> tuple[AxleDrivenCoordinate, ...]:
    """
    Wheel-centre and rack driven coordinates.

    ``drive_wheels`` is set for K (both wheel-centre z rows are prescribed); C
    mode prescribes the rack only, because the wheels are carried by compliance.
    """
    drives: list[AxleDrivenCoordinate] = []
    if drive_wheels:
        for side in ("L", "R"):
            drives.append(
                AxleDrivenCoordinate(
                    name=f"wheel_drive_{side}",
                    kind="translation",
                    body=f"upright_{side}",
                    reaction_body="chassis",
                    point_local_m=tuple(
                        float(v) * MM for v in assembly.point(f"upright_{side}", "wheel_center")
                    ),
                    reaction_point_local_m=(0.0, 0.0, 0.0),
                    axis_local=(0.0, 0.0, 1.0),
                )
            )
    drives.append(
        AxleDrivenCoordinate(
            name="rack_neutral" if not drive_wheels else "rack_drive",
            kind="translation",
            body="rack",
            reaction_body="chassis",
            point_local_m=tuple(float(v) * MM for v in assembly.point("rack", "center")),
            reaction_point_local_m=(0.0, 0.0, 0.0),
            axis_local=(0.0, 1.0, 0.0),
        )
    )
    return tuple(drives)


def to_native_model(assembly, *, name: str, drive_wheels: bool) -> AxleDynamicsModel:
    """
    Express ``assembly`` as a native model.

    K mode takes the ideal kinematic set; C mode takes ``assembly.constraints``,
    which leaves the arm mounts to the bushings -- using ``ideal_constraints``
    there would add rigid joints at the compliant mounts and stiffen the model.
    """
    # K drives its reference through the rigid kinematic set; a compliant
    # assembly carries that set with paired ball joints at the mounts, which is
    # written here in the equivalent full-rank revolute form.  C solves the
    # compliant set, where the mounts are carried by the bushings instead.
    joint_source = (
        collapse_spherical_pairs(assembly.ideal_constraints)
        if drive_wheels
        else assembly.constraints
    )
    uses_bushings = bool(assembly.bushings) and not drive_wheels
    return AxleDynamicsModel(
        name=name,
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=_bodies(assembly),
        joints=_joints(assembly, joint_source),
        bushings=_bushings(assembly) if uses_bushings else (),
        driven_coordinates=axle_drives(assembly, drive_wheels=drive_wheels),
    )
