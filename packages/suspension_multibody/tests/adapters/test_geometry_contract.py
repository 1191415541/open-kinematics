from __future__ import annotations

import pytest
from suspension_contracts import (
    SCHEMA_VERSION,
    CoordinateFrame,
    GeometryContract,
    Hardpoint,
    LengthUnit,
    MirrorAxis,
    Point3,
    RoleBinding,
    SourceSide,
    TopologyProfile,
)

from suspension_multibody.adapters import front_axle_model_from_contract
from suspension_multibody.schema import MassSpec

_ROLE_POSITIONS = {
    "upper_arm_inboard_front": (100.0, -500.0, 300.0),
    "upper_arm_inboard_rear": (-100.0, -500.0, 300.0),
    "upper_arm_outboard": (0.0, -900.0, 250.0),
    "lower_arm_inboard_front": (100.0, -500.0, 0.0),
    "lower_arm_inboard_rear": (-100.0, -500.0, 0.0),
    "lower_arm_outboard": (0.0, -900.0, 0.0),
    "tie_rod_inboard": (0.0, -250.0, 50.0),
    "tie_rod_outboard": (20.0, -850.0, 50.0),
    "wheel_center": (0.0, -1000.0, 100.0),
    "rack_center": (0.0, 0.0, 50.0),
}


def _contract(*, frame: CoordinateFrame = CoordinateFrame()) -> GeometryContract:
    hardpoints = tuple(
        Hardpoint(identifier=role, position=Point3(*position))
        for role, position in _ROLE_POSITIONS.items()
    )
    return GeometryContract(
        schema_version=SCHEMA_VERSION,
        name="contract_front_axle",
        topology=TopologyProfile.SYMMETRIC_FRONT_DOUBLE_WISHBONE,
        frame=frame,
        length_unit=LengthUnit.MILLIMETER,
        source_side=SourceSide.LEFT,
        mirror_axis=MirrorAxis.Y,
        hardpoints=hardpoints,
        role_bindings=tuple(
            RoleBinding(role=role, hardpoint_id=role) for role in _ROLE_POSITIONS
        ),
    )


def test_front_axle_model_uses_contract_hardpoint_roles() -> None:
    model = front_axle_model_from_contract(
        _contract(),
        mass=MassSpec(sprung_mass=1200.0),
    )

    assert model.name == "contract_front_axle"
    assert (
        model.hardpoints["upper_front"].as_tuple()
        == _ROLE_POSITIONS["upper_arm_inboard_front"]
    )
    assert (
        model.hardpoints["tierod_inner"].as_tuple()
        == _ROLE_POSITIONS["tie_rod_inboard"]
    )
    assert model.hardpoints["rack_center"].as_tuple() == _ROLE_POSITIONS["rack_center"]


def test_front_axle_model_rejects_a_non_vehicle_frame() -> None:
    contract = _contract(frame=CoordinateFrame(x_positive="forward"))

    with pytest.raises(ValueError, match="coordinate frame"):
        front_axle_model_from_contract(
            contract,
            mass=MassSpec(sprung_mass=1200.0),
        )
