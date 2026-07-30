"""Build Multibody front-axle geometry from Geometry Contract V1."""

from __future__ import annotations

from suspension_contracts import (
    CoordinateFrame,
    GeometryContract,
    LengthUnit,
    TopologyProfile,
)

from suspension_multibody.schema import FrontAxleModel, MassSpec, Vec3

_ROLE_TO_HARDPOINT: tuple[tuple[str, str], ...] = (
    ("upper_arm_inboard_front", "upper_front"),
    ("upper_arm_inboard_rear", "upper_rear"),
    ("upper_arm_outboard", "upper_outer"),
    ("lower_arm_inboard_front", "lower_front"),
    ("lower_arm_inboard_rear", "lower_rear"),
    ("lower_arm_outboard", "lower_outer"),
    ("tie_rod_inboard", "tierod_inner"),
    ("tie_rod_outboard", "tierod_outer"),
    ("wheel_center", "wheel_center"),
    ("rack_center", "rack_center"),
)


def _validate_contract(contract: GeometryContract) -> None:
    if contract.topology is not TopologyProfile.SYMMETRIC_FRONT_DOUBLE_WISHBONE:
        raise ValueError("Multibody adapter requires a symmetric front double-wishbone")
    if contract.length_unit is not LengthUnit.MILLIMETER:
        raise ValueError("Multibody adapter requires millimeter hardpoints")
    if contract.frame != CoordinateFrame():
        raise ValueError("Multibody adapter requires the V1 vehicle coordinate frame")


def front_axle_model_from_contract(
    contract: GeometryContract,
    *,
    mass: MassSpec,
    name: str | None = None,
) -> FrontAxleModel:
    """
    Create a geometry-only Multibody front-axle model from Contract V1.

    The Geometry Contract intentionally contains no mass or force-element data,
    so the caller supplies the Multibody mass specification explicitly.
    """
    if not isinstance(contract, GeometryContract):
        raise TypeError("contract must be a GeometryContract")
    if not isinstance(mass, MassSpec):
        raise TypeError("mass must be a MassSpec")
    _validate_contract(contract)

    points_by_id = {
        hardpoint.identifier: hardpoint.position for hardpoint in contract.hardpoints
    }
    bindings = {
        binding.role: binding.hardpoint_id for binding in contract.role_bindings
    }
    hardpoints: dict[str, Vec3] = {}
    for role, hardpoint_name in _ROLE_TO_HARDPOINT:
        try:
            position = points_by_id[bindings[role]]
        except KeyError as error:
            raise ValueError(
                f"contract is missing the {role!r} role binding"
            ) from error
        hardpoints[hardpoint_name] = Vec3(
            x=position.x,
            y=position.y,
            z=position.z,
        )

    return FrontAxleModel(
        name=contract.name if name is None else name,
        hardpoints=hardpoints,
        mass=mass,
    )
