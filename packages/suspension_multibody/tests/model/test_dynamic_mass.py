"""
Dynamic mass-property mapping: the schema body spec reaches the assembly body.

The mass-matrix and spatial-inertia assertions that used to live here were made
against ``model/mass.py``; they are native contract assertions now, in
``tests/axle_dynamics/test_solver_invariants.py``.
"""

from __future__ import annotations

import pytest

from suspension_multibody.preparation.assembly import build_front_axle
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
