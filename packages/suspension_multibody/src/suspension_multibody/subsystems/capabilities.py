"""
What an assembly can actually offer a rig.

A rig asks an assembly for coordinates to drive.  It used to get them by
inspecting the model -- "is there a body named rack?" -- which makes the rig
responsible for guessing, and makes an assembly that lacks a subsystem fail with
a `StopIteration` somewhere deep in the case layer rather than at the boundary.

An assembly therefore reports its capabilities: which subsystems it carries, and
which drive coordinates it can actually offer.  A rig consults this and shrinks
its interface to fit; it never probes for names.

The two namespaces here are deliberately different:

* `subsystems` uses the six role names (`suspension` .. `drive`);
* `drive_coordinates` uses the *document* coordinate names the model writes
  (`wheel_drive_L`, `rack_drive`), not the case layer's grouping keys
  (`wheel`, `rack`).  Mixing them is an interface error, so the constant below
  says which is which once.
"""

from __future__ import annotations

from dataclasses import dataclass, field

__all__ = [
    "ALL_SUBSYSTEMS",
    "AssemblyCapabilities",
    "DRIVE_COORDINATE_NAMES",
    "RACK_COORDINATES",
    "WHEEL_COORDINATES",
]

#: The six subsystem role names an assembly may carry.  Deliberately `wheel`
#: rather than `tire`, matching `templates.roles`.
ALL_SUBSYSTEMS: frozenset[str] = frozenset(
    {"suspension", "steering", "wheel", "chassis", "brake", "drive"}
)

#: The document-level names of the wheel travel coordinates.
WHEEL_COORDINATES: frozenset[str] = frozenset({"wheel_drive_L", "wheel_drive_R"})

#: The document-level names of the rack coordinate.  K drives the rack, C holds
#: it neutral; either way the name starts with `rack_`.
RACK_COORDINATES: frozenset[str] = frozenset({"rack_drive", "rack_neutral"})

#: Every drive coordinate name the axle families use, in the document's spelling.
DRIVE_COORDINATE_NAMES: frozenset[str] = WHEEL_COORDINATES | RACK_COORDINATES


@dataclass(frozen=True)
class AssemblyCapabilities:
    """
    The subsystems an assembly carries and the coordinates it can drive.

    Immutable, because a rig binds to it at preparation time and a capability
    that changed afterwards would silently invalidate that binding.

    The single rule a rig applies is
    `coordinate in capabilities.drive_coordinates`.  `body_names` is diagnostic
    only: it exists so an error message can say what *is* there, and nothing may
    branch on it.
    """

    #: Which of the six roles this assembly carries.
    subsystems: frozenset[str]
    #: The drive coordinates the assembly can actually offer, in the document's
    #: own spelling.
    drive_coordinates: frozenset[str]
    #: Diagnostic only: the bodies that ended up in the assembly.  Never a
    #: judgement -- see the module docstring.
    body_names: frozenset[str] = field(default_factory=frozenset)

    def has(self, subsystem: str) -> bool:
        """Return whether the assembly carries a subsystem role."""
        return subsystem in self.subsystems

    def provides(self, coordinate: str) -> bool:
        """Return whether the assembly can drive a coordinate."""
        return coordinate in self.drive_coordinates

    def require(self, coordinate: str, *, rig: str) -> None:
        """
        Raise unless the assembly can drive `coordinate`.

        Names the coordinate, the rig that wanted it, and what the assembly does
        offer, so a mismatch is diagnosable from the message alone.
        """
        if self.provides(coordinate):
            return
        offered = ", ".join(sorted(self.drive_coordinates)) or "(none)"
        present = ", ".join(sorted(self.subsystems)) or "(none)"
        raise CapabilityError(
            f"rig {rig!r} requires drive coordinate {coordinate!r}, which this "
            f"assembly does not provide; it offers {offered} and carries "
            f"subsystems {present}"
        )


class CapabilityError(ValueError):
    """A rig asked for something the assembly cannot provide."""


def capabilities_for(
    *,
    subsystems: frozenset[str],
    body_names: frozenset[str],
) -> AssemblyCapabilities:
    """
    Derive an assembly's capabilities from the subsystems it carries.

    The coordinate set is a *consequence* of the subsystem set, computed in one
    place: an assembly with wheels can drive wheel travel, and only one with a
    steering subsystem can drive the rack.  Computing it here rather than at each
    call site is what keeps the two assemblies consistent.
    """
    unknown = subsystems - ALL_SUBSYSTEMS
    if unknown:
        raise CapabilityError(f"unknown subsystem role(s) {sorted(unknown)}")

    coordinates: set[str] = set()
    if "suspension" in subsystems or "wheel" in subsystems:
        coordinates |= WHEEL_COORDINATES
    if "steering" in subsystems:
        coordinates |= RACK_COORDINATES

    return AssemblyCapabilities(
        subsystems=frozenset(subsystems),
        drive_coordinates=frozenset(coordinates),
        body_names=frozenset(body_names),
    )
