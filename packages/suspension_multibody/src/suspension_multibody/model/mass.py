"""Runtime mass-property helpers for dynamic analysis."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

try:
    from numba import njit
except ImportError:  # pragma: no cover
    def njit(*_args, **_kwargs):  # noqa: D103
        def _decorator(func):
            return func

        return _decorator

from ..core import RigidBody
from ..core.spatial import skew


@dataclass(frozen=True)
class BodyMassProperties:
    """Validated body mass properties and 6x6 spatial inertia."""

    body: str
    mass: float
    center_of_mass: np.ndarray
    inertia_about_com: np.ndarray
    spatial_inertia: np.ndarray


def body_mass_properties(body: RigidBody) -> BodyMassProperties:
    """Return validated mass properties for one runtime rigid body."""
    if body.fixed:
        spatial = np.zeros((6, 6))
        return BodyMassProperties(
            body.name, body.mass, body.center_of_mass.copy(), body.inertia.copy(), spatial
        )
    if body.mass <= 0:
        raise ValueError(f"dynamic body {body.name!r} requires positive mass")
    inertia = np.asarray(body.inertia, dtype=float)
    if not np.allclose(inertia, inertia.T, atol=1e-9):
        raise ValueError(f"dynamic body {body.name!r} inertia must be symmetric")
    if float(np.linalg.eigvalsh(inertia).min()) <= 0:
        raise ValueError(f"dynamic body {body.name!r} inertia must be positive definite")
    center = np.asarray(body.center_of_mass, dtype=float)
    cross = skew(center)
    spatial = np.block(
        [
            [body.mass * np.eye(3), -body.mass * cross],
            [body.mass * cross, inertia + body.mass * cross @ cross.T],
        ]
    )
    return BodyMassProperties(body.name, body.mass, center.copy(), inertia.copy(), spatial)


def mass_matrix(
    bodies: dict[str, RigidBody], body_order: tuple[str, ...] | None = None
) -> np.ndarray:
    """Assemble a block-diagonal spatial mass matrix for movable bodies."""
    order = body_order or tuple(name for name, body in bodies.items() if not body.fixed)
    blocks = [body_mass_properties(bodies[name]).spatial_inertia for name in order]
    if not blocks:
        return np.zeros((0, 0))
    matrix = np.zeros((6 * len(blocks), 6 * len(blocks)))
    for index, block in enumerate(blocks):
        start = 6 * index
        matrix[start : start + 6, start : start + 6] = block
    return matrix


@njit(nogil=True, fastmath=True)
def _spatial_bias_wrench_numba(spatial_inertia: np.ndarray, twist: np.ndarray) -> np.ndarray:
    """Return the Newton-Euler velocity bias for a body-frame twist."""
    momentum = spatial_inertia @ twist
    lm = momentum[:3]
    am = momentum[3:]
    lv = twist[:3]
    av = twist[3:]
    result = np.empty(6)
    result[0] = av[1] * lm[2] - av[2] * lm[1]
    result[1] = av[2] * lm[0] - av[0] * lm[2]
    result[2] = av[0] * lm[1] - av[1] * lm[0]
    result[3] = (lv[1] * lm[2] - lv[2] * lm[1]) + (av[1] * am[2] - av[2] * am[1])
    result[4] = (lv[2] * lm[0] - lv[0] * lm[2]) + (av[2] * am[0] - av[0] * am[2])
    result[5] = (lv[0] * lm[1] - lv[1] * lm[0]) + (av[0] * am[1] - av[1] * am[0])
    return result


def spatial_bias_wrench(spatial_inertia: np.ndarray, local_twist: np.ndarray) -> np.ndarray:
    """Return the Newton-Euler velocity bias for a body-frame twist."""
    return _spatial_bias_wrench_numba(
        np.asarray(spatial_inertia, dtype=float), np.asarray(local_twist, dtype=float)
    )
