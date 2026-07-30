"""Compliance and secant/tangent calculations."""

from __future__ import annotations

import numpy as np


def validate_compliance(matrix: np.ndarray) -> np.ndarray:
    """Validate a finite symmetric positive-semidefinite 6x6 compliance."""
    value = np.asarray(matrix, dtype=float)
    if value.shape != (6, 6) or not np.all(np.isfinite(value)):
        raise ValueError("compliance must be a finite 6x6 matrix")
    if not np.allclose(value, value.T, atol=1e-10):
        raise ValueError("compliance must be symmetric")
    if np.linalg.eigvalsh(value).min() < -1e-10:
        raise ValueError("compliance must be positive semidefinite")
    return value.copy()


def tangent_compliance(stiffness: np.ndarray) -> np.ndarray:
    """Return the pseudo-inverse compliance of a possibly singular stiffness."""
    matrix = np.asarray(stiffness, dtype=float)
    if matrix.shape != (6, 6):
        raise ValueError("stiffness must be 6x6")
    return np.linalg.pinv(0.5 * (matrix + matrix.T), rcond=1e-12)


def secant_compliance(load: np.ndarray, displacement: np.ndarray) -> np.ndarray:
    """Return a rank-one secant compliance mapping for one path point."""
    force = np.asarray(load, dtype=float)
    delta = np.asarray(displacement, dtype=float)
    if force.shape != (6,) or delta.shape != (6,):
        raise ValueError("load and displacement must contain six values")
    denominator = float(force @ force)
    return (
        np.outer(delta, force) / denominator
        if denominator > 1e-24
        else np.zeros((6, 6))
    )
