from __future__ import annotations

import numpy as np
from suspension_contracts import GeometryContract

from suspension_kinematics.adapters import export_geometry_contract
from suspension_kinematics.core.enums import PointID
from suspension_kinematics.io.geometry_loader import load_geometry
from suspension_kinematics.suspensions.double_wishbone import DoubleWishboneSuspension

_ROLE_TO_POINT_ID = (
    ("upper_arm_inboard_front", PointID.UPPER_WISHBONE_INBOARD_FRONT),
    ("upper_arm_inboard_rear", PointID.UPPER_WISHBONE_INBOARD_REAR),
    ("upper_arm_outboard", PointID.UPPER_WISHBONE_OUTBOARD),
    ("lower_arm_inboard_front", PointID.LOWER_WISHBONE_INBOARD_FRONT),
    ("lower_arm_inboard_rear", PointID.LOWER_WISHBONE_INBOARD_REAR),
    ("lower_arm_outboard", PointID.LOWER_WISHBONE_OUTBOARD),
    ("tie_rod_inboard", PointID.TRACKROD_INBOARD),
    ("tie_rod_outboard", PointID.TRACKROD_OUTBOARD),
    ("wheel_center", PointID.WHEEL_CENTER),
)


def _positions_by_role(
    contract: GeometryContract,
) -> dict[str, tuple[float, float, float]]:
    hardpoints = {
        hardpoint.identifier: hardpoint.position for hardpoint in contract.hardpoints
    }
    return {
        binding.role: (
            hardpoints[binding.hardpoint_id].x,
            hardpoints[binding.hardpoint_id].y,
            hardpoints[binding.hardpoint_id].z,
        )
        for binding in contract.role_bindings
    }


def test_export_uses_the_design_state_and_round_trips(
    double_wishbone_geometry_file,
) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)
    assert isinstance(suspension, DoubleWishboneSuspension)

    contract = export_geometry_contract(suspension)
    state = suspension.initial_state()
    positions = _positions_by_role(contract)

    assert GeometryContract.from_json(contract.to_json()) == contract
    assert len(contract.hardpoints) == 10
    for role, point_id in _ROLE_TO_POINT_ID:
        np.testing.assert_allclose(positions[role], state.positions[point_id])

    tie_rod_inboard = state.positions[PointID.TRACKROD_INBOARD]
    np.testing.assert_allclose(
        positions["rack_center"],
        (tie_rod_inboard[0], 0.0, tie_rod_inboard[2]),
    )


def test_export_accepts_an_explicit_rack_center(double_wishbone_geometry_file) -> None:
    suspension = load_geometry(double_wishbone_geometry_file)

    contract = export_geometry_contract(
        suspension,
        rack_center=(10.0, 20.0, 30.0),
    )

    assert _positions_by_role(contract)["rack_center"] == (10.0, 20.0, 30.0)
