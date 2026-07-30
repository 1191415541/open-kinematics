"""Constraint rank and mixed-unit scaling diagnostics."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class RankDiagnostic:
    """SVD-based rank status for a scaled constraint Jacobian."""

    rows: int
    columns: int
    rank: int
    singular_values: tuple[float, ...]
    condition_number: float
    underconstrained: bool
    overconstrained: bool


def scale_jacobian(
    jacobian: np.ndarray,
    *,
    translation_scale: float = 1.0,
    rotation_scale: float = 100.0,
) -> np.ndarray:
    """Scale alternating 6D body columns to common engineering units."""
    matrix = np.asarray(jacobian, dtype=float).copy()
    if matrix.ndim != 2:
        raise ValueError("Jacobian must be a two-dimensional matrix")
    if matrix.shape[1] % 6:
        return matrix
    scales = np.tile(
        [translation_scale] * 3 + [rotation_scale] * 3, matrix.shape[1] // 6
    )
    return matrix * scales


def diagnose_rank(jacobian: np.ndarray, *, rtol: float = 1e-10) -> RankDiagnostic:
    """Classify rank after explicit mixed-unit scaling."""
    scaled = scale_jacobian(jacobian)
    if not scaled.size:
        return RankDiagnostic(
            0, scaled.shape[1], 0, (), float("inf"), scaled.shape[1] > 0, False
        )
    singular = np.linalg.svd(scaled, compute_uv=False)
    threshold = singular[0] * rtol
    rank = int(np.count_nonzero(singular > threshold))
    condition = float(singular[0] / singular[rank - 1]) if rank else float("inf")
    return RankDiagnostic(
        rows=scaled.shape[0],
        columns=scaled.shape[1],
        rank=rank,
        singular_values=tuple(float(value) for value in singular),
        condition_number=condition,
        underconstrained=rank < scaled.shape[1],
        overconstrained=rank < scaled.shape[0],
    )
