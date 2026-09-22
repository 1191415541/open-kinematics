"""
Joint residuals and analytic local Jacobians.

The joint declarations themselves are data and live in
``preparation.assembly.types``: a declaration carries its bodies, its points and
its axes, and nothing else.  What is left here is the *solving* half -- the
residual and Jacobian kernels over those declarations -- together with
``ConstraintSystem``, which stacks them into one residual vector and one
matrix.

This module is not on the live path: production authors a declaration and the
native kernel assembles and solves it.  The only remaining callers are
``core.reactions`` -- which has no production caller of its own, only
``tests/core/test_reactions.py`` -- and ``tests/core/test_constraints.py``.
The kernels move to the native ``mb_joint``/``mb_solve_*`` modules as part of
the epic; this file is deleted with the rest of ``core`` in 08.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

try:
    from numba import njit as _njit
except ImportError:  # pragma: no cover
    def _njit(*_args, **_kwargs):
        def _decorator(func):
            return func
        return _decorator

from ..preparation.assembly.types import (
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
from .rigid_body import RigidBodyState, point_jacobian
from .spatial import (
    Array,
    cross3,
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_rotation_vector,
    skew,
)

#: The declaration classes are re-exported for the retired callers that still
#: import them from here (``core/__init__``); the declarations themselves live
#: in ``preparation.assembly.types``.
__all__ = [
    "BallJoint",
    "ConstantVelocityJoint",
    "Constraint",
    "ConstraintSystem",
    "CoordinateDrive",
    "CylindricalJoint",
    "DistanceConstraint",
    "InPlaneJoint",
    "PointCoincidence",
    "PrismaticJoint",
    "RevoluteJoint",
    "UniversalJoint",
    "WeldJoint",
    "jacobian",
    "residual",
]


@_njit(nogil=True, fastmath=True)
def _point_coincidence_residual_numba(
    ta: Array, ra: Array, pa: Array, tb: Array, rb: Array, pb: Array
) -> Array:
    """Residual is point_world(a, pa) minus point_world(b, pb)."""
    pax = ta[0] + ra[0, 0] * pa[0] + ra[0, 1] * pa[1] + ra[0, 2] * pa[2]
    pay = ta[1] + ra[1, 0] * pa[0] + ra[1, 1] * pa[1] + ra[1, 2] * pa[2]
    paz = ta[2] + ra[2, 0] * pa[0] + ra[2, 1] * pa[1] + ra[2, 2] * pa[2]
    pbx = tb[0] + rb[0, 0] * pb[0] + rb[0, 1] * pb[1] + rb[0, 2] * pb[2]
    pby = tb[1] + rb[1, 0] * pb[0] + rb[1, 1] * pb[1] + rb[1, 2] * pb[2]
    pbz = tb[2] + rb[2, 0] * pb[0] + rb[2, 1] * pb[1] + rb[2, 2] * pb[2]
    result = np.empty(3)
    result[0] = pax - pbx
    result[1] = pay - pby
    result[2] = paz - pbz
    return result


@_njit(nogil=True, fastmath=True)
def _normalize3_numba(x: float, y: float, z: float) -> Array:
    norm_squared = x * x + y * y + z * z
    if norm_squared < 1e-24:
        result = np.empty(3)
        result[0] = 0.0
        result[1] = 0.0
        result[2] = 1.0
        return result
    inv = 1.0 / math.sqrt(norm_squared)
    result = np.empty(3)
    result[0] = x * inv
    result[1] = y * inv
    result[2] = z * inv
    return result


def _normalize3(vector: Array) -> Array:
    """Normalize a finite three-vector using scalar norm arithmetic."""
    value = np.asarray(vector, dtype=float)
    return _normalize3_numba(float(value[0]), float(value[1]), float(value[2]))


def _basis_perpendicular(axis: Array) -> Array:
    """Return two orthonormal rows perpendicular to a unit axis."""
    vector = _normalize3(axis)
    helper = (
        np.array([1.0, 0.0, 0.0]) if abs(vector[0]) < 0.8 else np.array([0.0, 1.0, 0.0])
    )
    first = _normalize3(cross3(vector, helper))
    second = cross3(vector, first)
    return np.vstack((first, second))


def _axis_frame_with_jacobian(
    rotation: Array, axis_local: Array
) -> tuple[Array, Array, Array, Array, Array]:
    """Return an axis frame and its analytic local-rotation derivatives."""
    axis_local_array = np.asarray(axis_local, dtype=float)
    axis_local_norm = float(np.linalg.norm(axis_local_array))
    if axis_local_norm <= 1e-12:
        raise ValueError("joint axis must be nonzero")
    axis_local_unit = axis_local_array / axis_local_norm
    axis = _normalize3(rotation @ axis_local_unit)
    helper = (
        np.array([1.0, 0.0, 0.0])
        if abs(axis[0]) < 0.8
        else np.array([0.0, 1.0, 0.0])
    )
    raw = helper - axis * float(helper @ axis)
    norm = float(np.linalg.norm(raw))
    e1 = raw / norm
    e2 = cross3(axis, e1)
    d_axis = np.hstack(
        (np.zeros((3, 3)), -rotation @ skew(axis_local_unit))
    )
    d_raw = -(
        np.eye(3) * float(helper @ axis) + np.outer(axis, helper)
    ) @ d_axis
    d_e1 = ((np.eye(3) - np.outer(e1, e1)) / norm) @ d_raw
    d_e2 = -skew(e1) @ d_axis + skew(axis) @ d_e1
    return axis, np.vstack((e1, e2)), d_axis, d_e1, d_e2


# --------------------------------------------------------------------------- #
# residual
# --------------------------------------------------------------------------- #


def residual(constraint: Constraint, state: RigidBodyState) -> Array:
    """Return the residual vector of one joint declaration."""
    if isinstance(constraint, PointCoincidence):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        return _point_coincidence_residual_numba(
            pose_a.translation, pose_a.rotation, np.asarray(constraint.point_a, dtype=float),
            pose_b.translation, pose_b.rotation, np.asarray(constraint.point_b, dtype=float),
        )
    if isinstance(constraint, WeldJoint):
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        relative = quaternion_multiply(
            quaternion_conjugate(state.pose(constraint.body_a).quaternion),
            state.pose(constraint.body_b).quaternion,
        )
        return np.concatenate((point, quaternion_to_rotation_vector(relative)))
    if isinstance(constraint, DistanceConstraint):
        delta = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        return np.array([np.linalg.norm(delta) - constraint.distance])
    if isinstance(constraint, RevoluteJoint):
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        axis_a = state.pose(constraint.body_a).rotation @ constraint.axis_a
        axis_b = state.pose(constraint.body_b).rotation @ constraint.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        basis = _basis_perpendicular(axis_a)
        return np.concatenate((point, basis @ cross3(axis_a, axis_b)))
    if isinstance(constraint, UniversalJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        axis_a = _normalize3(pose_a.rotation @ constraint.axis_a)
        axis_b = _normalize3(pose_b.rotation @ constraint.axis_b)
        return np.concatenate((point, [float(axis_a @ axis_b)]))
    if isinstance(constraint, ConstantVelocityJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        x_a = _normalize3(pose_a.rotation @ constraint.axis_a)
        y_a = _normalize3(pose_a.rotation @ constraint.axis_a_secondary)
        y_b = _normalize3(pose_b.rotation @ constraint.axis_b)
        x_b = _normalize3(pose_b.rotation @ constraint.axis_b_secondary)
        # 相位匹配需要两组交叉轴；只保留第一项会退化成普通万向约束。
        angle = float(x_a @ y_b + y_a @ x_b) - constraint.angle_target
        return np.concatenate((point, [angle]))
    if isinstance(constraint, CylindricalJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        axis_a, basis, _, _, _ = _axis_frame_with_jacobian(
            pose_a.rotation, constraint.axis_a
        )
        del axis_a
        axis_b = _normalize3(pose_b.rotation @ constraint.axis_b)
        return np.concatenate((basis @ point, basis @ axis_b))
    if isinstance(constraint, InPlaneJoint):
        pose_a = state.pose(constraint.body_a)
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        axis_a = _normalize3(pose_a.rotation @ constraint.axis_a)
        return np.array([float(point @ axis_a)])
    if isinstance(constraint, PrismaticJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        axis_a = pose_a.rotation @ constraint.axis_a
        axis_b = pose_b.rotation @ constraint.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        displacement = state.point_world(constraint.body_b, constraint.point_b) - state.point_world(
            constraint.body_a, constraint.point_a
        )
        relative_quaternion = quaternion_multiply(
            quaternion_conjugate(pose_a.quaternion), pose_b.quaternion
        )
        relative_vector = pose_a.rotation @ quaternion_to_rotation_vector(
            relative_quaternion
        )
        basis = _basis_perpendicular(axis_a)
        return np.concatenate(
            (
                basis @ displacement,
                basis @ cross3(axis_a, axis_b),
                [axis_a @ relative_vector],
            )
        )
    if isinstance(constraint, CoordinateDrive):
        axis = _normalize3(constraint.axis)
        return np.array(
            [axis @ state.point_world(constraint.body, constraint.point) - constraint.target]
        )
    raise TypeError(f"no residual for constraint {constraint!r}")


# --------------------------------------------------------------------------- #
# Jacobian
# --------------------------------------------------------------------------- #


def jacobian(constraint: Constraint, state: RigidBodyState) -> dict[str, Array]:
    """Return body-local Jacobian blocks of one joint declaration."""
    if isinstance(constraint, PointCoincidence):
        return {
            constraint.body_a: point_jacobian(state, constraint.body_a, constraint.point_a),
            constraint.body_b: -point_jacobian(state, constraint.body_b, constraint.point_b),
        }
    if isinstance(constraint, WeldJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        relative_rotation = pose_a.rotation.T @ pose_b.rotation
        rotation_a = np.hstack((np.zeros((3, 3)), -np.eye(3)))
        rotation_b = np.hstack((np.zeros((3, 3)), relative_rotation))
        return {
            constraint.body_a: np.vstack(
                (point_jacobian(state, constraint.body_a, constraint.point_a), rotation_a)
            ),
            constraint.body_b: np.vstack(
                (-point_jacobian(state, constraint.body_b, constraint.point_b), rotation_b)
            ),
        }
    if isinstance(constraint, DistanceConstraint):
        delta = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        norm = float(np.linalg.norm(delta))
        if norm < 1e-12:
            raise ValueError("distance constraint is singular at coincident points")
        direction = delta / norm
        return {
            constraint.body_a: direction[None, :]
            @ point_jacobian(state, constraint.body_a, constraint.point_a),
            constraint.body_b: -direction[None, :]
            @ point_jacobian(state, constraint.body_b, constraint.point_b),
        }
    if isinstance(constraint, RevoluteJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        axis_a = pose_a.rotation @ constraint.axis_a
        axis_b = pose_b.rotation @ constraint.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        basis = _basis_perpendicular(axis_a)
        point_a_jac = point_jacobian(state, constraint.body_a, constraint.point_a)
        point_b_jac = point_jacobian(state, constraint.body_b, constraint.point_b)
        d_axis_a = np.hstack((np.zeros((3, 3)), -pose_a.rotation @ skew(constraint.axis_a)))
        d_axis_b = np.hstack((np.zeros((3, 3)), -pose_b.rotation @ skew(constraint.axis_b)))
        d_cross_a = -skew(axis_b) @ d_axis_a
        d_cross_b = skew(axis_a) @ d_axis_b
        return {
            constraint.body_a: np.vstack((point_a_jac, basis @ d_cross_a)),
            constraint.body_b: np.vstack((-point_b_jac, basis @ d_cross_b)),
        }
    if isinstance(constraint, UniversalJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        axis_a, _, d_axis_a, _, _ = _axis_frame_with_jacobian(
            pose_a.rotation, constraint.axis_a
        )
        axis_b, _, d_axis_b, _, _ = _axis_frame_with_jacobian(
            pose_b.rotation, constraint.axis_b
        )
        return {
            constraint.body_a: np.vstack(
                (
                    point_jacobian(state, constraint.body_a, constraint.point_a),
                    (axis_b @ d_axis_a)[None, :],
                )
            ),
            constraint.body_b: np.vstack(
                (
                    -point_jacobian(state, constraint.body_b, constraint.point_b),
                    (axis_a @ d_axis_b)[None, :],
                )
            ),
        }
    if isinstance(constraint, ConstantVelocityJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        x_a, _, d_x_a, _, _ = _axis_frame_with_jacobian(
            pose_a.rotation, constraint.axis_a
        )
        y_a, _, d_y_a, _, _ = _axis_frame_with_jacobian(
            pose_a.rotation, constraint.axis_a_secondary
        )
        y_b, _, d_y_b, _, _ = _axis_frame_with_jacobian(
            pose_b.rotation, constraint.axis_b
        )
        x_b, _, d_x_b, _, _ = _axis_frame_with_jacobian(
            pose_b.rotation, constraint.axis_b_secondary
        )
        scalar_a = y_b @ d_x_a + x_b @ d_y_a
        scalar_b = x_a @ d_y_b + y_a @ d_x_b
        return {
            constraint.body_a: np.vstack(
                (
                    point_jacobian(state, constraint.body_a, constraint.point_a),
                    scalar_a[None, :],
                )
            ),
            constraint.body_b: np.vstack(
                (
                    -point_jacobian(state, constraint.body_b, constraint.point_b),
                    scalar_b[None, :],
                )
            ),
        }
    if isinstance(constraint, CylindricalJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        axis_a, basis, _, d_e1, d_e2 = _axis_frame_with_jacobian(
            pose_a.rotation, constraint.axis_a
        )
        axis_b = _normalize3(pose_b.rotation @ constraint.axis_b)
        _, _, d_axis_b, _, _ = _axis_frame_with_jacobian(
            pose_b.rotation, constraint.axis_b
        )
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        point_a_jac = point_jacobian(state, constraint.body_a, constraint.point_a)
        point_b_jac = point_jacobian(state, constraint.body_b, constraint.point_b)
        e1, e2 = basis
        return {
            constraint.body_a: np.vstack(
                (
                    e1 @ point_a_jac + (point @ d_e1)[None, :],
                    e2 @ point_a_jac + (point @ d_e2)[None, :],
                    (axis_b @ d_e1)[None, :],
                    (axis_b @ d_e2)[None, :],
                )
            ),
            constraint.body_b: np.vstack(
                (
                    -(e1 @ point_b_jac)[None, :],
                    -(e2 @ point_b_jac)[None, :],
                    (e1 @ d_axis_b)[None, :],
                    (e2 @ d_axis_b)[None, :],
                )
            ),
        }
    if isinstance(constraint, InPlaneJoint):
        pose_a = state.pose(constraint.body_a)
        point = state.point_world(constraint.body_a, constraint.point_a) - state.point_world(
            constraint.body_b, constraint.point_b
        )
        axis_a = _normalize3(pose_a.rotation @ constraint.axis_a)
        _, _, d_axis_a, _, _ = _axis_frame_with_jacobian(
            pose_a.rotation, constraint.axis_a
        )
        point_a_jac = point_jacobian(state, constraint.body_a, constraint.point_a)
        point_b_jac = point_jacobian(state, constraint.body_b, constraint.point_b)
        return {
            constraint.body_a: np.vstack(
                (
                    (axis_a @ point_a_jac + (point @ d_axis_a)[None, :]),
                )
            ),
            constraint.body_b: -(axis_a @ point_b_jac)[None, :],
        }
    if isinstance(constraint, PrismaticJoint):
        pose_a = state.pose(constraint.body_a)
        pose_b = state.pose(constraint.body_b)
        axis_a = pose_a.rotation @ constraint.axis_a
        axis_b = pose_b.rotation @ constraint.axis_b
        axis_a = _normalize3(axis_a)
        axis_b = _normalize3(axis_b)
        basis = _basis_perpendicular(axis_a)
        point_a_jac = point_jacobian(state, constraint.body_a, constraint.point_a)
        point_b_jac = point_jacobian(state, constraint.body_b, constraint.point_b)
        d_axis_a = np.hstack((np.zeros((3, 3)), -pose_a.rotation @ skew(constraint.axis_a)))
        d_axis_b = np.hstack((np.zeros((3, 3)), -pose_b.rotation @ skew(constraint.axis_b)))
        d_cross_a = -skew(axis_b) @ d_axis_a
        d_cross_b = skew(axis_a) @ d_axis_b
        rotation_a = np.hstack((np.zeros((3, 3)), -pose_a.rotation))
        rotation_b = np.hstack((np.zeros((3, 3)), pose_b.rotation))
        return {
            constraint.body_a: np.vstack(
                (
                    -basis @ point_a_jac,
                    basis @ d_cross_a,
                    axis_a @ rotation_a,
                )
            ),
            constraint.body_b: np.vstack(
                (
                    basis @ point_b_jac,
                    basis @ d_cross_b,
                    axis_a @ rotation_b,
                )
            ),
        }
    if isinstance(constraint, CoordinateDrive):
        axis = _normalize3(constraint.axis)
        return {
            constraint.body: axis[None, :]
            @ point_jacobian(state, constraint.body, constraint.point)
        }
    raise TypeError(f"no Jacobian for constraint {constraint!r}")


@dataclass(frozen=True)
class ConstraintSystem:
    """Stack constraints into a deterministic residual/Jacobian matrix."""

    constraints: tuple[Constraint, ...]
    _body_indices_cache: dict[str, int] = field(default_factory=dict, repr=False, compare=False)
    _n_bodies_cache: int = field(default=0, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not self._body_indices_cache and self.constraints:
            # Pre-compute body indices from the first constraint's bodies
            pass

    def _get_body_indices(self, body_order: tuple[str, ...]) -> dict[str, int]:
        if body_order and len(self._body_indices_cache) == len(body_order):
            cached = self._body_indices_cache
            if all(name in cached for name in body_order):
                return cached
        return {name: index for index, name in enumerate(body_order)}

    def residual(self, state: RigidBodyState) -> Array:
        if not self.constraints:
            return np.zeros(0)
        return np.concatenate(
            [residual(constraint, state) for constraint in self.constraints]
        )

    def jacobian(
        self, state: RigidBodyState, body_order: tuple[str, ...] | None = None
    ) -> Array:
        order = body_order or tuple(
            name for name, body in state.bodies.items() if not body.fixed
        )
        body_indices = self._get_body_indices(order)
        n_bodies = len(order)
        local_blocks = [jacobian(constraint, state) for constraint in self.constraints]
        total_rows = 0
        for local in local_blocks:
            first_block = next(iter(local.values()), None)
            total_rows += int(first_block.shape[0]) if first_block is not None else 0
        if total_rows == 0:
            return np.zeros((0, 6 * n_bodies))
        result = np.zeros((total_rows, 6 * n_bodies))
        row_offset = 0
        for local in local_blocks:
            first_block = next(iter(local.values()), None)
            row_count = int(first_block.shape[0]) if first_block is not None else 0
            if row_count:
                row_slice = slice(row_offset, row_offset + row_count)
                for name, block in local.items():
                    index = body_indices.get(name)
                    if index is not None:
                        result[row_slice, index * 6 : (index + 1) * 6] = block
            row_offset += row_count
        return result
