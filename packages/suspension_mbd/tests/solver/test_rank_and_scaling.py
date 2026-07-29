"""Solver scaling and failure diagnostics tests."""

import numpy as np

from suspension_mbd.core import diagnose_rank, scale_jacobian


def test_mixed_unit_scaling_changes_rotation_columns_only() -> None:
    jacobian = np.eye(6)
    scaled = scale_jacobian(jacobian, translation_scale=1.0, rotation_scale=100.0)
    assert np.allclose(np.diag(scaled), [1, 1, 1, 100, 100, 100])
    assert diagnose_rank(jacobian).rank == 6
