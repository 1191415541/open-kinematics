"""Dynamic mass-property mapping tests."""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.core import RigidBody
from suspension_multibody.model import (
    body_mass_properties,
    build_front_axle,
    mass_matrix,
)
from suspension_multibody.schema import FrontAxleModel, MassSpec, RigidBodySpec, Vec3


def _hardpoints() -> dict[str, Vec3]:
    return {
        "UPPER_INBOARD_FRONT": Vec3(x=0, y=-300, z=300),
        "UPPER_INBOARD_REAR": Vec3(x=300, y=-300, z=300),
        "UPPER_OUTBOARD": Vec3(x=150, y=-700, z=250),
        "LOWER_INBOARD_FRONT": Vec3(x=0, y=-320, z=0),
        "LOWER_INBOARD_REAR": Vec3(x=320, y=-320, z=0),
        "LOWER_OUTBOARD": Vec3(x=150, y=-720, z=50),
        "TIE_ROD_INBOARD": Vec3(x=100, y=-250, z=100),
        "TIE_ROD_OUTBOARD": Vec3(x=180, y=-700, z=100),
        "WHEEL_CENTER": Vec3(x=160, y=-760, z=150),
        "RACK_CENTER": Vec3(x=100, y=0, z=100),
    }


def test_body_spec_maps_to_front_axle_runtime_body() -> None:
    model = FrontAxleModel(
        hardpoints=_hardpoints(),
        mass=MassSpec(sprung_mass=1000.0),
        bodies=(
            RigidBodySpec(
                name="upright_L",
                mass=38.0,
                center_of_mass=Vec3(x=1.0, y=2.0, z=3.0),
                inertia=((10.0, 0.0, 0.0), (0.0, 11.0, 0.0), (0.0, 0.0, 12.0)),
            ),
        ),
    )

    assembly = build_front_axle(model, "K")

    assert assembly.bodies["upright_L"].mass == pytest.approx(38.0)
    assert assembly.bodies["upright_L"].center_of_mass.tolist() == [1.0, 2.0, 3.0]


def test_mass_matrix_rejects_zero_mass_movable_body() -> None:
    bodies = {"body": RigidBody("body")}

    with pytest.raises(ValueError, match="positive mass"):
        mass_matrix(bodies)


def test_spatial_inertia_contains_mass_and_rotational_inertia() -> None:
    body = RigidBody(
        "body",
        mass=2.0,
        center_of_mass=np.array([0.0, 0.0, 0.0]),
        inertia=np.diag([3.0, 4.0, 5.0]),
    )

    properties = body_mass_properties(body)

    assert np.diag(properties.spatial_inertia).tolist() == [2.0, 2.0, 2.0, 3.0, 4.0, 5.0]
