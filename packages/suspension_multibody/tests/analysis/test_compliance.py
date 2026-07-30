"""Compliance tangent and secant tests."""

import numpy as np
import pytest

from suspension_multibody.analysis import secant_compliance, validate_compliance


def test_compliance_requires_symmetric_psd_matrix() -> None:
    matrix = np.eye(6)
    assert np.array_equal(validate_compliance(matrix), matrix)
    with pytest.raises(ValueError, match="symmetric"):
        validate_compliance(np.triu(np.ones((6, 6))))


def test_rank_one_secant_maps_path_load() -> None:
    load = np.array([2.0, 0, 0, 0, 0, 0])
    displacement = np.array([1.0, 0, 0, 0, 0, 0])
    result = secant_compliance(load, displacement)
    assert np.allclose(result @ load, displacement)
