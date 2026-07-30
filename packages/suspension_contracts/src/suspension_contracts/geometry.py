"""JSON-compatible Geometry Contract V1 without solver dependencies."""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from enum import StrEnum
from typing import Mapping, Sequence, cast

SCHEMA_VERSION = "1.0"
_SUPPORTED_SCHEMA_MAJOR = 1
_IDENTIFIER = re.compile(r"^[a-z][a-z0-9_]*$")


class GeometryContractError(ValueError):
    """Raised when a Geometry Contract value is invalid."""


class SchemaVersionError(GeometryContractError):
    """Raised when a contract uses an unsupported schema major version."""


class LengthUnit(StrEnum):
    """Length units available in Geometry Contract V1."""

    MILLIMETER = "mm"


class TopologyProfile(StrEnum):
    """Topology profiles understood by Geometry Contract V1."""

    SYMMETRIC_FRONT_DOUBLE_WISHBONE = "symmetric_front_double_wishbone"


class SourceSide(StrEnum):
    """The side supplied explicitly before the symmetry transform."""

    LEFT = "left"


class MirrorAxis(StrEnum):
    """The vehicle axis used to mirror the source-side geometry."""

    Y = "y"


_REQUIRED_ROLES: dict[TopologyProfile, frozenset[str]] = {
    TopologyProfile.SYMMETRIC_FRONT_DOUBLE_WISHBONE: frozenset(
        {
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
        }
    )
}


def _nonempty_string(value: object, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise GeometryContractError(f"{label} must be a non-empty string")
    return value


def _identifier(value: object, label: str) -> str:
    text = _nonempty_string(value, label)
    if _IDENTIFIER.fullmatch(text) is None:
        raise GeometryContractError(f"{label} must be lower_snake_case, got {text!r}")
    return text


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, int | float):
        raise GeometryContractError(f"{label} must be a finite number")
    number = float(value)
    if not math.isfinite(number):
        raise GeometryContractError(f"{label} must be a finite number")
    return number


def _mapping(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise GeometryContractError(f"{label} must be an object")
    if not all(isinstance(key, str) for key in value):
        raise GeometryContractError(f"{label} keys must be strings")
    return cast(Mapping[str, object], value)


def _sequence(value: object, label: str) -> Sequence[object]:
    if isinstance(value, str | bytes) or not isinstance(value, Sequence):
        raise GeometryContractError(f"{label} must be an array")
    return value


def _closed_fields(
    data: Mapping[str, object], expected: frozenset[str], label: str
) -> None:
    missing = expected - set(data)
    unexpected = set(data) - expected
    if missing:
        raise GeometryContractError(f"{label} is missing fields: {sorted(missing)}")
    if unexpected:
        raise GeometryContractError(
            f"{label} has unsupported fields: {sorted(unexpected)}"
        )


def _schema_major(version: str) -> int:
    parts = version.split(".")
    if len(parts) != 2 or not all(part.isdecimal() for part in parts):
        raise SchemaVersionError("schema_version must use the '<major>.<minor>' form")
    return int(parts[0])


@dataclass(frozen=True, slots=True)
class Point3:
    """Finite position vector expressed in the contract coordinate frame."""

    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _finite_number(self.x, "x"))
        object.__setattr__(self, "y", _finite_number(self.y, "y"))
        object.__setattr__(self, "z", _finite_number(self.z, "z"))

    def to_dict(self) -> dict[str, float]:
        """Return a JSON-compatible representation."""
        return {"x": self.x, "y": self.y, "z": self.z}

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> Point3:
        """Create a point from a closed JSON-compatible object."""
        _closed_fields(payload, frozenset({"x", "y", "z"}), "point")
        return cls(
            x=_finite_number(payload["x"], "point.x"),
            y=_finite_number(payload["y"], "point.y"),
            z=_finite_number(payload["z"], "point.z"),
        )


@dataclass(frozen=True, slots=True)
class CoordinateFrame:
    """Explicit vehicle coordinate declaration carried by every contract."""

    name: str = "vehicle"
    x_positive: str = "rearward"
    y_positive: str = "right"
    z_positive: str = "upward"
    right_handed: bool = True

    def __post_init__(self) -> None:
        for field_name in ("name", "x_positive", "y_positive", "z_positive"):
            _nonempty_string(getattr(self, field_name), f"frame.{field_name}")
        if not isinstance(self.right_handed, bool):
            raise GeometryContractError("frame.right_handed must be a boolean")

    def to_dict(self) -> dict[str, str | bool]:
        """Return a JSON-compatible representation."""
        return {
            "name": self.name,
            "x_positive": self.x_positive,
            "y_positive": self.y_positive,
            "z_positive": self.z_positive,
            "right_handed": self.right_handed,
        }

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> CoordinateFrame:
        """Create a coordinate frame from a closed JSON-compatible object."""
        _closed_fields(
            payload,
            frozenset(
                {"name", "x_positive", "y_positive", "z_positive", "right_handed"}
            ),
            "frame",
        )
        right_handed = payload["right_handed"]
        if not isinstance(right_handed, bool):
            raise GeometryContractError("frame.right_handed must be a boolean")
        return cls(
            name=_nonempty_string(payload["name"], "frame.name"),
            x_positive=_nonempty_string(payload["x_positive"], "frame.x_positive"),
            y_positive=_nonempty_string(payload["y_positive"], "frame.y_positive"),
            z_positive=_nonempty_string(payload["z_positive"], "frame.z_positive"),
            right_handed=right_handed,
        )


@dataclass(frozen=True, slots=True)
class Hardpoint:
    """Stable hardpoint identifier and its design-condition position."""

    identifier: str
    position: Point3

    def __post_init__(self) -> None:
        object.__setattr__(
            self, "identifier", _identifier(self.identifier, "hardpoint")
        )
        if not isinstance(self.position, Point3):
            raise GeometryContractError("hardpoint.position must be a Point3")

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-compatible representation."""
        return {"identifier": self.identifier, "position": self.position.to_dict()}

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> Hardpoint:
        """Create a hardpoint from a closed JSON-compatible object."""
        _closed_fields(payload, frozenset({"identifier", "position"}), "hardpoint")
        return cls(
            identifier=_identifier(payload["identifier"], "hardpoint.identifier"),
            position=Point3.from_dict(
                _mapping(payload["position"], "hardpoint.position")
            ),
        )


@dataclass(frozen=True, slots=True)
class RoleBinding:
    """Bind a topology role to one stable hardpoint identifier."""

    role: str
    hardpoint_id: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "role", _identifier(self.role, "role"))
        object.__setattr__(
            self, "hardpoint_id", _identifier(self.hardpoint_id, "hardpoint_id")
        )

    def to_dict(self) -> dict[str, str]:
        """Return a JSON-compatible representation."""
        return {"role": self.role, "hardpoint_id": self.hardpoint_id}

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> RoleBinding:
        """Create a role binding from a closed JSON-compatible object."""
        _closed_fields(payload, frozenset({"role", "hardpoint_id"}), "role_binding")
        return cls(
            role=_identifier(payload["role"], "role_binding.role"),
            hardpoint_id=_identifier(
                payload["hardpoint_id"], "role_binding.hardpoint_id"
            ),
        )


@dataclass(frozen=True, slots=True)
class GeometryContract:
    """Closed V1 geometry interchange model for a symmetric front axle."""

    schema_version: str
    name: str
    topology: TopologyProfile
    frame: CoordinateFrame
    length_unit: LengthUnit
    source_side: SourceSide
    mirror_axis: MirrorAxis
    hardpoints: tuple[Hardpoint, ...]
    role_bindings: tuple[RoleBinding, ...]

    def __post_init__(self) -> None:
        major = _schema_major(_nonempty_string(self.schema_version, "schema_version"))
        if major != _SUPPORTED_SCHEMA_MAJOR:
            raise SchemaVersionError(
                f"unsupported Geometry Contract schema major {major}; "
                f"expected {_SUPPORTED_SCHEMA_MAJOR}"
            )
        _nonempty_string(self.name, "name")
        if not isinstance(self.topology, TopologyProfile):
            raise GeometryContractError("topology must be a TopologyProfile")
        if not isinstance(self.frame, CoordinateFrame):
            raise GeometryContractError("frame must be a CoordinateFrame")
        if not isinstance(self.length_unit, LengthUnit):
            raise GeometryContractError("length_unit must be a LengthUnit")
        if self.source_side is not SourceSide.LEFT:
            raise GeometryContractError(
                "Geometry Contract V1 requires left source-side data"
            )
        if self.mirror_axis is not MirrorAxis.Y:
            raise GeometryContractError(
                "Geometry Contract V1 requires mirroring about Y"
            )
        if not self.hardpoints:
            raise GeometryContractError("at least one hardpoint is required")

        hardpoint_ids = tuple(hardpoint.identifier for hardpoint in self.hardpoints)
        if len(hardpoint_ids) != len(set(hardpoint_ids)):
            raise GeometryContractError("hardpoint identifiers must be unique")
        if not all(isinstance(hardpoint, Hardpoint) for hardpoint in self.hardpoints):
            raise GeometryContractError("hardpoints must contain Hardpoint values")

        roles = tuple(binding.role for binding in self.role_bindings)
        if len(roles) != len(set(roles)):
            raise GeometryContractError("topology roles must be unique")
        if not all(isinstance(binding, RoleBinding) for binding in self.role_bindings):
            raise GeometryContractError("role_bindings must contain RoleBinding values")
        unknown_point_ids = {
            binding.hardpoint_id for binding in self.role_bindings
        } - set(hardpoint_ids)
        if unknown_point_ids:
            raise GeometryContractError(
                "role bindings reference unknown hardpoints: "
                f"{sorted(unknown_point_ids)}"
            )
        required_roles = _REQUIRED_ROLES[self.topology]
        missing_roles = required_roles - set(roles)
        if missing_roles:
            raise GeometryContractError(
                f"topology is missing required roles: {sorted(missing_roles)}"
            )

    def to_dict(self) -> dict[str, object]:
        """Return the closed JSON-compatible Geometry Contract representation."""
        return {
            "schema_version": self.schema_version,
            "name": self.name,
            "topology": self.topology.value,
            "frame": self.frame.to_dict(),
            "length_unit": self.length_unit.value,
            "source_side": self.source_side.value,
            "mirror_axis": self.mirror_axis.value,
            "hardpoints": [hardpoint.to_dict() for hardpoint in self.hardpoints],
            "role_bindings": [binding.to_dict() for binding in self.role_bindings],
        }

    def to_json(self) -> str:
        """Return canonical compact JSON suitable for file exchange and hashing."""
        return json.dumps(self.to_dict(), sort_keys=True, separators=(",", ":"))

    @classmethod
    def from_dict(cls, payload: Mapping[str, object]) -> GeometryContract:
        """Parse and validate a closed JSON-compatible Geometry Contract object."""
        _closed_fields(
            payload,
            frozenset(
                {
                    "schema_version",
                    "name",
                    "topology",
                    "frame",
                    "length_unit",
                    "source_side",
                    "mirror_axis",
                    "hardpoints",
                    "role_bindings",
                }
            ),
            "geometry contract",
        )
        try:
            topology = TopologyProfile(
                _nonempty_string(payload["topology"], "topology")
            )
            length_unit = LengthUnit(
                _nonempty_string(payload["length_unit"], "length_unit")
            )
            source_side = SourceSide(
                _nonempty_string(payload["source_side"], "source_side")
            )
            mirror_axis = MirrorAxis(
                _nonempty_string(payload["mirror_axis"], "mirror_axis")
            )
        except ValueError as error:
            raise GeometryContractError(str(error)) from error
        hardpoints = tuple(
            Hardpoint.from_dict(_mapping(value, "hardpoints item"))
            for value in _sequence(payload["hardpoints"], "hardpoints")
        )
        role_bindings = tuple(
            RoleBinding.from_dict(_mapping(value, "role_bindings item"))
            for value in _sequence(payload["role_bindings"], "role_bindings")
        )
        return cls(
            schema_version=_nonempty_string(
                payload["schema_version"], "schema_version"
            ),
            name=_nonempty_string(payload["name"], "name"),
            topology=topology,
            frame=CoordinateFrame.from_dict(_mapping(payload["frame"], "frame")),
            length_unit=length_unit,
            source_side=source_side,
            mirror_axis=mirror_axis,
            hardpoints=hardpoints,
            role_bindings=role_bindings,
        )

    @classmethod
    def from_json(cls, value: str) -> GeometryContract:
        """Parse a JSON document containing one Geometry Contract."""
        try:
            payload = json.loads(value)
        except json.JSONDecodeError as error:
            raise GeometryContractError(
                "geometry contract is not valid JSON"
            ) from error
        return cls.from_dict(_mapping(payload, "geometry contract"))
