"""Export double-wishbone design geometry through Geometry Contract V1."""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np
from numpy.typing import NDArray
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

from suspension_kinematics.core.enums import PointID
from suspension_kinematics.suspensions.double_wishbone import DoubleWishboneSuspension

CoordinateValues = Sequence[float] | NDArray[np.floating[Any]]

_ROLE_TO_POINT_ID: tuple[tuple[str, PointID], ...] = (
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


def _point3(values: CoordinateValues) -> Point3:
    """Convert one solver coordinate vector into a contract point."""
    if len(values) != 3:
        raise ValueError("contract hardpoints must contain exactly three coordinates")
    return Point3(x=float(values[0]), y=float(values[1]), z=float(values[2]))


def export_geometry_contract(
    suspension: DoubleWishboneSuspension,
    *,
    name: str | None = None,
    rack_center: CoordinateValues | None = None,
) -> GeometryContract:
    """Export a left-side double-wishbone design as Geometry Contract V1.

    Geometry Contract V1 requires a rack-center point that the Kinematics design
    model does not store independently. When it is not supplied, this adapter
    uses the established rack-axis projection of the inner tie-rod point onto
    the vehicle center plane.
    """
    if not isinstance(suspension, DoubleWishboneSuspension):
        raise TypeError("Geometry Contract V1 export requires DoubleWishboneSuspension")

    state = suspension.initial_state()
    missing = [
        point_id.name
        for _, point_id in _ROLE_TO_POINT_ID
        if point_id not in state.positions
    ]
    if missing:
        raise ValueError(f"suspension state is missing contract points: {missing}")

    hardpoints = [
        Hardpoint(identifier=role, position=_point3(state.positions[point_id]))
        for role, point_id in _ROLE_TO_POINT_ID
    ]
    if rack_center is None:
        tie_rod_inboard = state.positions[PointID.TRACKROD_INBOARD]
        rack_center = (
            float(tie_rod_inboard[0]),
            0.0,
            float(tie_rod_inboard[2]),
        )
    hardpoints.append(
        Hardpoint(identifier="rack_center", position=_point3(rack_center))
    )

    return GeometryContract(
        schema_version=SCHEMA_VERSION,
        name=suspension.name if name is None else name,
        topology=TopologyProfile.SYMMETRIC_FRONT_DOUBLE_WISHBONE,
        frame=CoordinateFrame(),
        length_unit=LengthUnit.MILLIMETER,
        source_side=SourceSide.LEFT,
        mirror_axis=MirrorAxis.Y,
        hardpoints=tuple(hardpoints),
        role_bindings=tuple(
            RoleBinding(role=hardpoint.identifier, hardpoint_id=hardpoint.identifier)
            for hardpoint in hardpoints
        ),
    )
