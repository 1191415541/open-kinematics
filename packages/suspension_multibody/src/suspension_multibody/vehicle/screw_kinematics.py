"""
Instantaneous kinematics of an assembled mechanism: velocity screws and instant axes.

The input is *the assembled constraint set and the assembly's point table*, handed
in by the caller: a sequence of joint declarations plus a :class:`RigidBodyState`.
Nothing here assembles a model, looks a hardpoint up by a name string, or knows what
a "control arm" is.  A caller asks for the motion the assembly is allowed to have
under a drive it names, and gets back the generalized velocity, the per-body twists
and the *space instantaneous axis* of the motion of a named body relative to a named
reference.

**Where the residual rows come from.**  A joint declaration carries data only -- its
two bodies, their points and its axes (``modeling/primitives/joints.py``, whose
docstring says the residual belongs to the solving side).  The residual of each
joint type is therefore a kernel fact:
``packages/suspension_kernel/cpp/src/joint/registry.cpp`` binds each type to its
residual and Jacobian handler and ``kernel_model_constraint.cpp`` writes the rows.
This module transcribes exactly those rows, type for type, so that a Python-side
velocity solve is the *same* constraint system the kernel solves rather than a
second interpretation of the same declarations.  A joint type the kernel has no
handler for (``DistanceConstraint``, ``CoordinateDrive``) is refused by name instead
of being given a private residual here: adding one is a kernel question.

**Numerical differentiation.**  The constraint Jacobian is obtained by central
differences along the same local tangent retraction the kernel uses
(``RigidBodyState.retract``: a 6-vector ``(translation, rotation vector)`` per body,
composed on the right of the pose).  The step is ``NUMERICAL_STEP = 1e-6`` in the
project's internal unit (millimetre for a length, radian for a rotation).  For a
central difference the truncation error is ``O(h^2 f''')`` and the roundoff error is
``O(eps |f| / h)``.  ``|f|`` here is *not* the assembly's residual, which is zero at
an assembled configuration: it is the scale of the quantities the residual is built
from, i.e. the coordinates and point offsets it subtracts, of order ``1e3`` mm.
With ``eps = 2.2e-16`` that gives ``~2.2e-7`` of roundoff against ``~1.7e-13`` of
truncation, so the step is already roundoff-dominated and shrinking it only makes
things worse -- which the measured step sweep in this task's
``raw/numerics_and_comparison.md`` shows: relative to the ``h = 1e-5`` reference, the
error is ``2.3e-06`` at ``h = 1e-4`` (truncation-dominated) and ``1.0e-06`` / ``1.1e-05``
at ``h = 1e-7`` / ``h = 1e-8`` (roundoff, growing as ``1/h``); the smallest error of the
measured steps is ``1.1e-07`` at ``h = 1e-6``.  The kernel's reference Jacobian takes the identical step for
the identical reason (``constraint_jacobian_central_difference``,
``constexpr double h = 1e-6``, with the comment that a one-sided difference would
leave an ``O(1e-7)`` error in ``J``).

**The instant axis.**  A twist is reported as a :class:`Twist` -- the angular
velocity ``omega`` and the velocity ``velocity`` of the *world origin*, both in world
coordinates, so the velocity of a world point ``x`` is ``velocity + omega x x``.  Its
:class:`Screw` is the minimal representation of the same motion: ``direction`` is the
unit rotation axis, ``point`` is a point of that axis (the one closest to a
caller-chosen anchor), ``pitch`` is the advance along the axis per radian, and
``angular_speed`` is ``|omega|``.  The reconstruction identity is
``v(x) = omega_rel x (x - point) + pitch * omega_rel``, whose value at the axis point
``x = point`` is ``pitch * omega_rel`` (the advance rate of the axis itself); the four
numbers therefore describe the twist losslessly whenever ``omega != 0``.  A motion
with no rotation has no axis at all and is refused rather than reported as a zero
direction.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np

from ..modeling.primitives import (
    ConstantVelocityJoint,
    Constraint,
    CylindricalJoint,
    InPlaneJoint,
    PointCoincidence,
    PrismaticJoint,
    RevoluteJoint,
    RigidBodyState,
    UniversalJoint,
    WeldJoint,
)

__all__ = [
    "NUMERICAL_STEP",
    "RANK_TOLERANCE",
    "KinematicError",
    "PointDrive",
    "RigidMotion",
    "Screw",
    "TangentDrive",
    "Twist",
    "constraint_jacobian",
    "constraint_residual",
    "constraint_rows",
    "free_bodies",
    "solve_rigid_motion",
    "screw_from_twist",
]

#: Central-difference step for the constraint Jacobian, in the model's internal
#: unit.  Matched to the kernel's reference Jacobian; see the module docstring for
#: the truncation/roundoff trade-off that fixes it.
NUMERICAL_STEP = 1e-6

#: A singular value counts as zero when it is below this fraction of the largest one.
#: ``1e-9`` sits two to three orders above the roundoff floor of a Jacobian whose
#: entries reach ``~1e3`` and whose numerical error is ``~1e-10`` (module docstring),
#: and far below the smallest singular value an assembled mechanism produces
#: (measured ``1.9e-4`` for the double-wishbone axle, ``raw/engine_contract.md``).
RANK_TOLERANCE = 1e-9

#: Below this angular speed a twist is taken to have no rotation axis.
_ANGULAR_SPEED_FLOOR = 1e-12


class KinematicError(ValueError):
    """A request this engine cannot answer from the data it was handed."""


def _unsupported(constraint: Constraint) -> KinematicError:
    return KinematicError(
        f"constraint {constraint.name!r} of type {type(constraint).__name__} has no "
        "row in this engine: the native kernel's joint registry "
        "(cpp/src/joint/registry.cpp) binds a residual to the spherical, revolute, "
        "fixed, prismatic, universal, cylindrical, in-plane and CONVEL types, and "
        "inventing one here would make this solve a different constraint system than "
        "the one the kernel solves"
    )


def _rotation_log(rotation: np.ndarray) -> np.ndarray:
    """Return the principal rotation vector of a proper rotation matrix."""
    cosine = 0.5 * (float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2]) - 1.0)
    axis_sine = 0.5 * np.array(
        [
            rotation[2, 1] - rotation[1, 2],
            rotation[0, 2] - rotation[2, 0],
            rotation[1, 0] - rotation[0, 1],
        ]
    )
    sine = float(np.linalg.norm(axis_sine))
    if sine < 1e-12:
        # Small-rotation branch: the axis-sine vector is already the rotation vector
        # to O(theta^3), and theta/sin(theta) is 1 to O(theta^2) anyway.
        return axis_sine
    angle = float(np.arctan2(sine, max(-1.0, min(1.0, cosine))))
    return axis_sine * (angle / sine)


def _unit(vector: np.ndarray) -> np.ndarray:
    """Return a unit vector, refusing a degenerate one."""
    norm = float(np.linalg.norm(vector))
    if norm < 1e-30:
        raise KinematicError("a joint axis in this assembly is degenerate")
    return vector / norm


def _perpendicular_reference(axis: np.ndarray) -> np.ndarray:
    """
    Return the reference vector the kernel picks for a joint axis.

    ``perpendicular_reference`` in ``kernel_model_constraint.cpp``: the first
    coordinate axis far enough from parallel to the joint axis.  The revolute,
    prismatic and cylindrical residuals are stated in the pair of directions built
    from this choice, so the same choice has to be made here for the rows to be the
    same rows.
    """
    if abs(float(axis[0])) < 0.8:
        return np.array([1.0, 0.0, 0.0])
    return np.array([0.0, 1.0, 0.0])


def _axis_frame(
    state: RigidBodyState, constraint: Constraint
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the joint's world axis and the two directions perpendicular to it."""
    axis_local = np.asarray(constraint.axis_a, dtype=float)  # ty: ignore[unresolved-attribute]
    axis = _unit(state.pose(constraint.body_a).rotation @ axis_local)  # ty: ignore[unresolved-attribute]
    reference = _perpendicular_reference(axis)
    first = reference - axis * float(reference @ axis)
    first = first / float(np.linalg.norm(first))
    return axis, first, np.cross(axis, first)


def constraint_rows(constraint: Constraint, state: RigidBodyState) -> np.ndarray:
    """
    Return one constraint's rows of the position residual ``C(q)``.

    The rows and their order are the kernel's: for a joint whose type carries the
    shared translation block, the three components of ``point_a - point_b`` come
    first and the type's own rows follow.
    """
    # A joint's data lives on its concrete type; the base class declares no fields.
    pose_a = state.pose(constraint.body_a)  # ty: ignore[unresolved-attribute]
    pose_b = state.pose(constraint.body_b)  # ty: ignore[unresolved-attribute]
    point_a = pose_a.transform_point(np.asarray(constraint.point_a, dtype=float))  # ty: ignore[unresolved-attribute]
    point_b = pose_b.transform_point(np.asarray(constraint.point_b, dtype=float))  # ty: ignore[unresolved-attribute]
    separation = point_a - point_b

    if isinstance(constraint, PointCoincidence):
        # The spherical joint writes this block and nothing else.
        return np.asarray(separation, dtype=float)
    if isinstance(constraint, WeldJoint):
        return np.concatenate((separation, _rotation_log(pose_a.rotation.T @ pose_b.rotation)))
    if isinstance(constraint, (PrismaticJoint, CylindricalJoint)):
        _, first, second = _axis_frame(state, constraint)
        axis_b = _unit(pose_b.rotation @ np.asarray(constraint.axis_b, dtype=float))
        rows = [
            float(separation @ first),
            float(separation @ second),
            float(axis_b @ first),
            float(axis_b @ second),
        ]
        if isinstance(constraint, PrismaticJoint):
            relative = _rotation_log(pose_a.rotation.T @ pose_b.rotation)
            rows.append(float(_unit(np.asarray(constraint.axis_a, dtype=float)) @ relative))
        return np.array(rows)
    if isinstance(constraint, UniversalJoint):
        axis, _, _ = _axis_frame(state, constraint)
        axis_b = _unit(pose_b.rotation @ np.asarray(constraint.axis_b, dtype=float))
        return np.array([float(axis @ axis_b)])
    if isinstance(constraint, InPlaneJoint):
        axis, _, _ = _axis_frame(state, constraint)
        return np.array([float(separation @ axis)])
    if isinstance(constraint, ConstantVelocityJoint):
        x_a = _unit(pose_a.rotation @ np.asarray(constraint.axis_a, dtype=float))
        y_a = _unit(pose_a.rotation @ np.asarray(constraint.axis_a_secondary, dtype=float))
        y_b = _unit(pose_b.rotation @ np.asarray(constraint.axis_b, dtype=float))
        x_b = _unit(pose_b.rotation @ np.asarray(constraint.axis_b_secondary, dtype=float))
        return np.array([float(x_a @ y_b) + float(y_a @ x_b) - float(constraint.angle_target)])
    # The revolute joint is the last of the types that carry a translation block:
    # three point-coincidence rows, then the two that keep body B's axis
    # perpendicular to the two directions the axis frame fixes.
    if isinstance(constraint, RevoluteJoint):
        _, first, second = _axis_frame(state, constraint)
        axis_b = _unit(pose_b.rotation @ np.asarray(constraint.axis_b, dtype=float))
        return np.concatenate((separation, [float(axis_b @ first), float(axis_b @ second)]))
    raise _unsupported(constraint)


def constraint_residual(
    constraints: Sequence[Constraint], state: RigidBodyState
) -> np.ndarray:
    """Return the stacked position residual of every constraint in ``constraints``."""
    if not constraints:
        return np.zeros(0)
    return np.concatenate([constraint_rows(item, state) for item in constraints])


def free_bodies(
    constraints: Sequence[Constraint], state: RigidBodyState
) -> tuple[str, ...]:
    """
    Return the bodies a motion may move, in a deterministic order.

    A body is movable when it appears in ``constraints`` and is not declared fixed.
    A body outside the constraint set has no column: attaching one would add an
    unconstrained coordinate the caller never asked about.  The order is sorted by
    name, so the column layout does not depend on the order the assembly happened to
    produce its rows in.
    """
    names: set[str] = set()
    for constraint in constraints:
        # A joint names its two bodies on its concrete type.
        for body in (constraint.body_a, constraint.body_b):  # ty: ignore[unresolved-attribute]
            if not state.bodies[body].fixed:
                names.add(body)
    return tuple(sorted(names))


def constraint_jacobian(
    constraints: Sequence[Constraint],
    state: RigidBodyState,
    *,
    bodies: Sequence[str] | None = None,
    step: float = NUMERICAL_STEP,
) -> tuple[np.ndarray, tuple[str, ...]]:
    """
    Return ``(J, bodies)``: the residual derivative along the tangent motion.

    Column ``6 * i + k`` is the derivative along the ``k``-th local tangent
    coordinate of ``bodies[i]`` -- ``k < 3`` a translation, ``k >= 3`` a rotation
    vector component -- which is the retraction ``RigidBodyState.retract`` applies
    and the one the kernel differentiates along.  ``J`` has one row per residual row
    and six columns per movable body.
    """
    if not np.isfinite(step) or step <= 0.0:
        raise KinematicError(f"the differentiation step must be finite and positive, got {step!r}")
    moved = tuple(bodies) if bodies is not None else free_bodies(constraints, state)
    residual = constraint_residual(constraints, state)
    jacobian = np.zeros((residual.size, 6 * len(moved)))
    for index, body in enumerate(moved):
        for coordinate in range(6):
            increment = np.zeros(6)
            increment[coordinate] = step
            plus = constraint_residual(constraints, state.retract_unchecked({body: increment}))
            minus = constraint_residual(constraints, state.retract_unchecked({body: -increment}))
            jacobian[:, 6 * index + coordinate] = (plus - minus) / (2.0 * step)
    return jacobian, moved


@dataclass(frozen=True)
class Twist:
    """A rigid velocity: the angular velocity and the velocity of the world origin."""

    omega: np.ndarray
    velocity: np.ndarray

    def transform_point(self, point: np.ndarray) -> np.ndarray:
        """Return the velocity of the world point ``point``."""
        return self.velocity + np.cross(self.omega, np.asarray(point, dtype=float))


@dataclass(frozen=True)
class Screw:
    """
    A space instantaneous axis: one point of it, its direction and its pitch.

    ``pitch`` is the advance along ``direction`` per radian of rotation, so a pure
    rotation has ``pitch == 0``.  ``point`` is a point of the axis, not a fixed
    point: any point on the axis describes the same screw, which is why the engine
    reports the one closest to a caller-chosen anchor.
    """

    point: np.ndarray
    direction: np.ndarray
    pitch: float
    angular_speed: float


def screw_from_twist(twist: Twist, *, anchor: np.ndarray | None = None) -> Screw:
    """
    Return the space instantaneous axis of ``twist``.

    ``anchor`` picks which point of the axis is reported: the point closest to
    ``anchor``, or to the world origin when it is ``None``.  A motion with no
    rotation has no axis and is refused.
    """
    omega = np.asarray(twist.omega, dtype=float)
    speed = float(np.linalg.norm(omega))
    if not np.isfinite(speed) or speed <= _ANGULAR_SPEED_FLOOR:
        raise KinematicError(
            "this motion has no angular velocity, so it has no instantaneous axis: a "
            "pure translation is not a screw"
        )
    velocity = np.asarray(twist.velocity, dtype=float)
    point = np.cross(omega, velocity) / (speed * speed)
    direction = omega / speed
    if anchor is not None:
        offset = np.asarray(anchor, dtype=float) - point
        point = point + direction * float(offset @ direction)
    return Screw(
        point=point,
        direction=direction,
        pitch=float(omega @ velocity) / (speed * speed),
        angular_speed=speed,
    )


@dataclass(frozen=True)
class TangentDrive:
    """A prescribed coordinate rate of one body: tangent component ``component``."""

    body: str
    component: int
    rate: float = 1.0

    def __post_init__(self) -> None:
        if not 0 <= self.component < 6:
            raise KinematicError(f"a tangent drive names component 0..5, got {self.component!r}")
        if not np.isfinite(self.rate):
            raise KinematicError("a tangent drive rate must be finite")


@dataclass(frozen=True)
class PointDrive:
    """
    A prescribed velocity of an assembly point along a world direction.

    ``(body, label)`` is a key of the point table the caller handed in -- the same
    table the assembler fills -- and ``direction`` is a world vector.  The drive
    states that the point's velocity along the unit direction equals ``rate``.
    """

    body: str
    label: str
    direction: np.ndarray
    rate: float = 1.0

    def __post_init__(self) -> None:
        direction = np.asarray(self.direction, dtype=float)
        if direction.shape != (3,) or not np.all(np.isfinite(direction)):
            raise KinematicError("a point drive direction must contain three finite values")
        if float(np.linalg.norm(direction)) < 1e-30:
            raise KinematicError("a point drive direction must not be the zero vector")
        if not np.isfinite(self.rate):
            raise KinematicError("a point drive rate must be finite")


Drive = TangentDrive | PointDrive


@dataclass(frozen=True)
class RigidMotion:
    """
    The instantaneous motion an assembly admits under a set of drives.

    ``bodies`` is the column order of ``jacobian`` and of ``velocity``.
    ``constraint_residual`` is the infinity norm of ``J q_dot`` and
    ``drive_residual`` that of ``A q_dot - b``, so a caller can tell a solve that met
    the mechanism's constraints from one that did not.  ``rank`` is the rank of the
    stacked system and ``nullity`` the number of motion directions still free after
    the drives; a caller that needs a specific member of that family states more
    drives.

    ``omega_rel`` is the pivot body's angular velocity minus the reference body's,
    ``v_rel`` the velocity of ``pivot_point`` in the same relative sense, and
    ``screw`` their space instantaneous axis.
    """

    bodies: tuple[str, ...]
    jacobian: np.ndarray
    velocity: np.ndarray
    null_space: np.ndarray
    twists: dict[str, Twist]
    reference: str | None
    pivot_body: str
    pivot_point: np.ndarray
    omega_rel: np.ndarray
    v_rel: np.ndarray
    screw: Screw
    constraint_residual: float
    drive_residual: float
    rank: int
    nullity: int

    def twist_of(self, body: str) -> Twist:
        """Return one body's absolute twist as a component of this motion."""
        try:
            return self.twists[body]
        except KeyError as exc:
            raise KinematicError(f"body {body!r} is not part of this motion") from exc


def _body_twist(state: RigidBodyState, body: str, increment: np.ndarray) -> Twist:
    """Convert a body's tangent coordinates into a world twist."""
    pose = state.pose(body)
    omega = pose.rotation @ np.asarray(increment[3:], dtype=float)
    velocity = pose.rotation @ np.asarray(increment[:3], dtype=float) + np.cross(
        pose.translation, omega
    )
    return Twist(omega=omega, velocity=velocity)


def _point_row(
    state: RigidBodyState,
    bodies: tuple[str, ...],
    body: str,
    point_local: np.ndarray,
    direction: np.ndarray,
) -> np.ndarray:
    """
    Return the row whose datum is a material point's velocity along ``direction``.

    The point's world position is ``t + R p``, so its world velocity is
    ``R d_t + (R d_theta) x (R p)``.  Projecting that on the world direction ``u``
    and reading the coefficients of the local tangent coordinates gives ``R^T u``
    on the translation block and ``p x (R^T u)`` on the rotation block.
    """
    pose = state.pose(body)
    local_direction = pose.rotation.T @ np.asarray(direction, dtype=float)
    row = np.zeros(6 * len(bodies))
    index = 6 * bodies.index(body)
    row[index : index + 3] = local_direction
    row[index + 3 : index + 6] = np.cross(np.asarray(point_local, dtype=float), local_direction)
    return row


def _minimum_norm(
    matrix: np.ndarray, target: np.ndarray
) -> tuple[np.ndarray, np.ndarray, int]:
    """
    Return ``(x, null_space, rank)`` for the minimum-norm solution of ``A x = b``.

    ``full_matrices=True`` because it is the *null space*, not the solution, that
    needs the whole right factor: with a wide ``A`` the thin SVD carries only
    ``rows`` right singular vectors, so every free direction beyond that count
    would be lost and ``null_space`` would silently under-report the motion the
    drives leave free.
    """
    if matrix.shape[0] == 0:
        return np.zeros(matrix.shape[1]), np.eye(matrix.shape[1]), 0
    left, singular, right = np.linalg.svd(matrix, full_matrices=True)
    largest = float(singular[0]) if singular.size else 0.0
    if largest <= 0.0:
        return np.zeros(matrix.shape[1]), np.eye(matrix.shape[1]), 0
    keep = singular > RANK_TOLERANCE * largest
    rank = int(np.count_nonzero(keep))
    solution = np.zeros(matrix.shape[1])
    if rank:
        solution = right[:rank].T @ ((left[:, :rank].T @ target) / singular[:rank])
    return solution, right[rank:].T, rank


def solve_rigid_motion(
    constraints: Sequence[Constraint],
    state: RigidBodyState,
    drives: Sequence[Drive] = (),
    *,
    points: Mapping[tuple[str, str], np.ndarray] | None = None,
    pivot_body: str | None = None,
    pivot_point: tuple[str, str] | None = None,
    reference: str | None = None,
    bodies: Sequence[str] | None = None,
    step: float = NUMERICAL_STEP,
) -> RigidMotion:
    """
    Solve for the instantaneous motion of the mechanism under ``drives``.

    The motion returned satisfies the assembly's constraints (``J q_dot = 0``) and
    the drives (``A q_dot = b``); among those it is the one with the smallest norm in
    the local tangent coordinates, because the engine carries no inertia to weigh the
    alternatives with.  ``nullity`` reports how many directions were still free.

    ``pivot_body``/``pivot_point`` choose whose motion the screw describes and which
    of that body's points the axis is reported closest to.  ``pivot_point`` is a key
    of ``points``.  When ``pivot_body`` is omitted and exactly one body is driven,
    the driven body is used; with any other number of driven bodies the caller has to
    say.  ``reference`` is the body the motion is taken relative to: it defaults to
    the assembly's single fixed body, and a mechanism with no fixed body is taken
    relative to the world.
    """
    residual = constraint_residual(constraints, state)
    jacobian, columns = constraint_jacobian(constraints, state, bodies=bodies, step=step)

    drive_rows: list[np.ndarray] = []
    drive_targets: list[float] = []
    for drive in drives:
        if drive.body not in columns:
            raise KinematicError(
                f"drive names body {drive.body!r}, which is not movable in this "
                f"assembly; the movable bodies are {list(columns)}"
            )
        if isinstance(drive, TangentDrive):
            row = np.zeros(6 * len(columns))
            row[6 * columns.index(drive.body) + drive.component] = 1.0
        else:
            if points is None:
                raise KinematicError(
                    f"the drive on point {drive.body!r}/{drive.label!r} needs the "
                    "assembly's point table; pass points= as well"
                )
            try:
                point_local = np.asarray(points[(drive.body, drive.label)], dtype=float)
            except KeyError as exc:
                raise KinematicError(
                    f"the point table has no entry {drive.body!r}/{drive.label!r}"
                ) from exc
            direction = np.asarray(drive.direction, dtype=float)
            row = _point_row(
                state, columns, drive.body, point_local, direction / float(np.linalg.norm(direction))
            )
        drive_rows.append(row)
        drive_targets.append(float(drive.rate))

    if drive_rows:
        system = np.vstack([jacobian, *drive_rows])
        target = np.concatenate([np.zeros(residual.size), np.asarray(drive_targets, dtype=float)])
    else:
        system = jacobian
        target = np.zeros(residual.size)

    velocity, null_space, rank = _minimum_norm(system, target)
    nullity = int(system.shape[1] - rank)

    fixed = sorted(name for name, body in state.bodies.items() if body.fixed)
    if reference is None:
        reference = fixed[0] if fixed else None
    elif reference in columns:
        raise KinematicError(
            f"reference body {reference!r} is movable, so it has no motion for the "
            "others to be relative to; name a fixed body or leave reference=None"
        )
    elif reference not in state.bodies:
        raise KinematicError(f"unknown reference body {reference!r}")

    if pivot_body is None:
        driven = {drive.body for drive in drives}
        if len(driven) != 1:
            raise KinematicError(
                "this motion has no unique body to read a screw from: "
                f"{len(driven)} bodies are driven; name pivot_body= explicitly"
            )
        pivot_body = driven.pop()
    if pivot_body not in columns:
        raise KinematicError(
            f"pivot body {pivot_body!r} is not movable in this assembly; the movable "
            f"bodies are {list(columns)}"
        )

    twists = {
        body: _body_twist(state, body, velocity[6 * index : 6 * index + 6])
        for index, body in enumerate(columns)
    }
    reference_twist = _body_twist(state, reference, np.zeros(6)) if reference else None
    pivot_pose = state.pose(pivot_body)
    if pivot_point is None:
        anchor = pivot_pose.translation.copy()
    else:
        if points is None:
            raise KinematicError(
                f"pivot point {pivot_point!r} names an entry of the assembly's point "
                "table; pass points= as well"
            )
        try:
            anchor_local = np.asarray(points[pivot_point], dtype=float)
        except KeyError as exc:
            raise KinematicError(f"the point table has no entry {pivot_point!r}") from exc
        if pivot_point[0] != pivot_body:
            raise KinematicError(
                f"pivot point {pivot_point!r} belongs to {pivot_point[0]!r}, not to the "
                f"pivot body {pivot_body!r}"
            )
        anchor = pivot_pose.transform_point(anchor_local)

    pivot_twist = twists[pivot_body]
    omega_rel = (
        pivot_twist.omega - reference_twist.omega if reference_twist else pivot_twist.omega
    )
    velocity_rel = (
        pivot_twist.velocity - reference_twist.velocity
        if reference_twist
        else pivot_twist.velocity
    )
    relative = Twist(omega=omega_rel, velocity=velocity_rel)

    return RigidMotion(
        bodies=columns,
        jacobian=jacobian,
        velocity=velocity,
        null_space=null_space,
        twists=twists,
        reference=reference,
        pivot_body=pivot_body,
        pivot_point=anchor,
        omega_rel=omega_rel,
        v_rel=relative.transform_point(anchor),
        screw=screw_from_twist(relative, anchor=anchor),
        constraint_residual=(
            float(np.max(np.abs(jacobian @ velocity))) if residual.size else 0.0
        ),
        drive_residual=(
            float(np.max(np.abs(system @ velocity - target))) if system.shape[0] else 0.0
        ),
        rank=rank,
        nullity=nullity,
    )
