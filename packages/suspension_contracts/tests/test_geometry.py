from __future__ import annotations

import pytest

from suspension_contracts import (
    GeometryContract,
    GeometryContractError,
    SchemaVersionError,
)


def _payload() -> dict[str, object]:
    roles = (
        "upper_arm_inboard_front",
        "upper_arm_inboard_rear",
        "upper_arm_outboard",
        "lower_arm_inboard_front",
        "lower_arm_inboard_rear",
        "lower_arm_outboard",
        "tie_rod_inboard",
        "tie_rod_outboard",
        "wheel_center",
        "rack_center",
    )
    hardpoints = [
        {
            "identifier": role,
            "position": {"x": float(index), "y": -10.0, "z": 100.0},
        }
        for index, role in enumerate(roles)
    ]
    return {
        "schema_version": "1.0",
        "name": "reference_front_axle",
        "topology": "symmetric_front_double_wishbone",
        "frame": {
            "name": "vehicle",
            "x_positive": "rearward",
            "y_positive": "right",
            "z_positive": "upward",
            "right_handed": True,
        },
        "length_unit": "mm",
        "source_side": "left",
        "mirror_axis": "y",
        "hardpoints": hardpoints,
        "role_bindings": [{"role": role, "hardpoint_id": role} for role in roles],
    }


def test_round_trip_preserves_the_closed_contract() -> None:
    contract = GeometryContract.from_dict(_payload())

    assert GeometryContract.from_json(contract.to_json()) == contract


def test_unknown_schema_major_is_rejected() -> None:
    payload = _payload()
    payload["schema_version"] = "2.0"

    with pytest.raises(SchemaVersionError, match="unsupported"):
        GeometryContract.from_dict(payload)


def test_duplicate_hardpoint_identifiers_are_rejected() -> None:
    payload = _payload()
    hardpoints = payload["hardpoints"]
    assert isinstance(hardpoints, list)
    duplicate = dict(hardpoints[0])
    hardpoints.append(duplicate)

    with pytest.raises(GeometryContractError, match="identifiers must be unique"):
        GeometryContract.from_dict(payload)


def test_missing_required_topology_role_is_rejected() -> None:
    payload = _payload()
    role_bindings = payload["role_bindings"]
    assert isinstance(role_bindings, list)
    role_bindings.pop()

    with pytest.raises(GeometryContractError, match="missing required roles"):
        GeometryContract.from_dict(payload)


def test_unknown_fields_are_rejected() -> None:
    payload = _payload()
    payload["unexpected"] = True

    with pytest.raises(GeometryContractError, match="unsupported fields"):
        GeometryContract.from_dict(payload)


def test_non_finite_coordinates_are_rejected() -> None:
    payload = _payload()
    hardpoints = payload["hardpoints"]
    assert isinstance(hardpoints, list)
    position = hardpoints[0]["position"]
    assert isinstance(position, dict)
    position["x"] = float("nan")

    with pytest.raises(GeometryContractError, match="finite"):
        GeometryContract.from_dict(payload)
