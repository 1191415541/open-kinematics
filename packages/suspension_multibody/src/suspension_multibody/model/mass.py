"""Runtime mass-property helpers for dynamic analysis."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..core import RigidBody
from ..core.spatial import cross3, skew


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


def spatial_bias_wrench(spatial_inertia: np.ndarray, local_twist: np.ndarray) -> np.ndarray:
    """Return the Newton-Euler velocity bias for a body-frame twist."""
    inertia = np.asarray(spatial_inertia, dtype=float)
    twist = np.asarray(local_twist, dtype=float)
    if inertia.shape != (6, 6) or twist.shape != (6,):
        raise ValueError("spatial inertia and twist must have shapes (6, 6) and (6,)")
    momentum = inertia @ twist
    linear_momentum = momentum[:3]
    angular_momentum = momentum[3:]
    linear_velocity = twist[:3]
    angular_velocity = twist[3:]
    return np.concatenate(
        (
            cross3(angular_velocity, linear_momentum),
            cross3(linear_velocity, linear_momentum)
            + cross3(angular_velocity, angular_momentum),
        )
    )
