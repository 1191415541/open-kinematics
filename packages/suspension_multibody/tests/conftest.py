"""Shared fixtures for suspension_multibody tests."""

import pytest

from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3


@pytest.fixture
def minimal_model() -> FrontAxleModel:
    return FrontAxleModel(
        hardpoints={"LOWER_FRONT_LEFT": Vec3(x=100, y=-700, z=200)},
        mass=MassSpec(sprung_mass=1200),
    )
