"""
Spatial algebra and SE(3) operations (the retired half of the legacy module).

The conversions preparation performs -- SE(3), the quaternion helpers, ``skew``,
``cross3`` and the wrench transform -- moved to ``preparation/geometry.py``,
which is the live implementation now.  This module re-exports them so the
retired Python solver modules that still run against ``core`` keep working, and
the state objects they build stay the *same* classes as the ones the authoring
layer builds; 08 deletes this file with the rest of ``core``.

What is left here is what has no live owner: the point-Jacobian kernel the
residual/Jacobian file still uses, and the twist and wrench tangents whose only
callers are ``tests/core/test_spatial.py``.
"""

from __future__ import annotations

import numpy as np

try:
    from numba import njit as _njit

    _HAVE_NUMBA = True
except ImportError:  # pragma: no cover - numba is an optional acceleration
    _HAVE_NUMBA = False

    def _njit(*_args, **_kwargs):  # type: ignore[misc]
        def _decorator(func):
            return func

        return _decorator

from ..preparation.geometry import (
    SE3,
    Array,
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
    "SE3",
    "cross3",
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


@_njit(nogil=True, fastmath=True)
def _point_jacobian_numba(rotation: Array, px: float, py: float, pz: float) -> Array:
    """Compute the 3x6 point Jacobian: [rotation | -rotation @ skew(point)]."""
    result = np.empty((3, 6))
    # Columns 0-2: rotation
    for i in range(3):
        for j in range(3):
            result[i, j] = rotation[i, j]
    # skew(point) columns: e0=[0,pz,-py], e1=[-pz,0,px], e2=[py,-px,0]
    # -R @ skew(point) = [-R@e0, -R@e1, -R@e2]
    # Column 3: -R @ [0, pz, -py]
    result[0, 3] = -(rotation[0, 1] * pz + rotation[0, 2] * (-py))
    result[1, 3] = -(rotation[1, 1] * pz + rotation[1, 2] * (-py))
    result[2, 3] = -(rotation[2, 1] * pz + rotation[2, 2] * (-py))
    # Column 4: -R @ [-pz, 0, px]
    result[0, 4] = -(rotation[0, 0] * (-pz) + rotation[0, 2] * px)
    result[1, 4] = -(rotation[1, 0] * (-pz) + rotation[1, 2] * px)
    result[2, 4] = -(rotation[2, 0] * (-pz) + rotation[2, 2] * px)
    # Column 5: -R @ [py, -px, 0]
    result[0, 5] = -(rotation[0, 0] * py + rotation[0, 1] * (-px))
    result[1, 5] = -(rotation[1, 0] * py + rotation[1, 1] * (-px))
    result[2, 5] = -(rotation[2, 0] * py + rotation[2, 1] * (-px))
    return result


def wrench_local_to_global(pose: SE3, wrench_local: Array) -> Array:
    """Transform a force/moment wrench from a body origin to global origin."""
    wrench = np.asarray(wrench_local, dtype=float)
    if wrench.shape != (6,):
        raise ValueError("wrench must contain six values")
    force = pose.rotation @ wrench[:3]
    moment = pose.rotation @ wrench[3:] + cross3(pose.translation, force)
    return np.concatenate((force, moment))


def twist_local_to_global(pose: SE3, twist_local: Array) -> Array:
    """Transform a body-origin twist to a global-origin twist."""
    twist = np.asarray(twist_local, dtype=float)
    if twist.shape != (6,):
        raise ValueError("twist must contain six values")
    angular = pose.rotation @ twist[3:]
    linear = pose.rotation @ twist[:3] + cross3(pose.translation, angular)
    return np.concatenate((linear, angular))


def wrench_matrix(pose: SE3) -> Array:
    """Return the matrix implementing :func:`wrench_local_to_global`."""
    rotation = pose.rotation
    return np.block(
        [[rotation, np.zeros((3, 3))], [skew(pose.translation) @ rotation, rotation]]
    )


def wrench_translation_tangent(force_global: Array) -> Array:
    """Return ``d(moment)/d(translation)`` for a fixed global force."""
    force = np.asarray(force_global, dtype=float)
    if force.shape != (3,):
        raise ValueError("force must contain three values")
    return np.vstack((np.zeros((3, 3)), -skew(force)))
