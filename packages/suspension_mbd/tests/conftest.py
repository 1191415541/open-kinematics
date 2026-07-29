"""Shared fixtures for suspension_mbd tests."""

import pytest

from suspension_mbd.schema import FrontAxleModel, MassSpec, Vec3


@pytest.fixture
def minimal_model() -> FrontAxleModel:
    return FrontAxleModel(
        hardpoints={"LOWER_FRONT_LEFT": Vec3(x=100, y=-700, z=200)},
        mass=MassSpec(sprung_mass=1200),
    )
