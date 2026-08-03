"""Full-vehicle Adams acceptance matrix tests."""

from __future__ import annotations

import pytest

from suspension_multibody.adams import (
    HANDLING_CASES,
    RIDE_CASES,
    VehicleAcceptanceCase,
    default_vehicle_acceptance_matrix,
    validate_vehicle_acceptance_matrix,
)


def test_vehicle_acceptance_matrix_splits_handling_and_ride_cases() -> None:
    matrix = default_vehicle_acceptance_matrix()
    categories = {case.category for case in matrix}
    names = {case.name for case in matrix}

    assert categories == {"handling_stability", "ride"}
    assert set(HANDLING_CASES) <= names
    assert set(RIDE_CASES) <= names
    assert all(case.adams_template_source == "adams_builtin" for case in matrix)
    assert all(case.pac2002_source == "adams_builtin" for case in matrix)


def test_vehicle_acceptance_matrix_rejects_missing_ride_split() -> None:
    with pytest.raises(ValueError, match="handling and ride"):
        validate_vehicle_acceptance_matrix(
            (
                VehicleAcceptanceCase(
                    name="steady_state_circle",
                    category="handling_stability",
                    channels=("yaw_rate",),
                    tolerances=(),
                ),
            )
        )


def test_default_vehicle_acceptance_matrix_validates() -> None:
    validate_vehicle_acceptance_matrix(default_vehicle_acceptance_matrix())
