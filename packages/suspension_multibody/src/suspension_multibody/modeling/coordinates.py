"""Immutable reports of declared drive coordinates and kernel axis bindings."""

from __future__ import annotations

from dataclasses import dataclass, field

WHEEL_COORDINATES = frozenset({"wheel_drive_L", "wheel_drive_R"})
RACK_COORDINATES = frozenset({"rack_drive", "rack_neutral"})
DRIVE_COORDINATE_NAMES = WHEEL_COORDINATES | RACK_COORDINATES
KERNEL_AXIS_GROUPS = {
    **{name: "wheel" for name in sorted(WHEEL_COORDINATES)},
    **{name: "rack" for name in sorted(RACK_COORDINATES)},
}


def kernel_axis(coordinate: str) -> str | None:
    """Return the declared protocol axis group, or None for an unswept drive."""
    return KERNEL_AXIS_GROUPS.get(coordinate)


class CapabilityError(ValueError):
    """A rig requested a coordinate the assembly does not declare."""


@dataclass(frozen=True)
class AssemblyCapabilities:
    """Coordinates supplied by declarations; body names are diagnostic only."""

    subsystems: frozenset[str]
    drive_coordinates: frozenset[str]
    body_names: frozenset[str] = field(default_factory=frozenset)

    def has(self, subsystem: str) -> bool:
        return subsystem in self.subsystems

    def provides(self, coordinate: str) -> bool:
        return coordinate in self.drive_coordinates

    def require(self, coordinate: str, *, rig: str) -> None:
        if self.provides(coordinate):
            return
        offered = ", ".join(sorted(self.drive_coordinates)) or "(none)"
        present = ", ".join(sorted(self.subsystems)) or "(none)"
        raise CapabilityError(
            f"rig {rig!r} requires drive coordinate {coordinate!r}, which this "
            f"assembly does not provide; it offers {offered} and carries "
            f"subsystems {present}"
        )
