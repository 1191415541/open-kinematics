"""
Rigs: the test bench an assembly is run on, as a declaration.

A run used to be a `(assembly, family)` pair from an enumeration, which meant the
bench and the model were the same fact.  They are not: the bench is what *drives*
the assembly and what *measures* it, and the same suspension test bench can drive
a rigid axle or a compliant one, with or without steering.

A rig therefore declares three things and nothing else:

* the drive coordinates it wants, and how it moves them;
* the solver defaults it runs with;
* **its own outputs** -- the quantities the bench adds that the assembly cannot
  know about (a wheel load, a rig frame pose).

What it must *not* do is require everything it can imagine.  An assembly reports
what it can offer (`AssemblyCapabilities`), and the rig's interface **shrinks** to
fit: an assembly with no steering has no rack coordinate, so the rack axis and the
rack outputs are absent rather than zeroed, and the rig still runs.  That is the
difference between a bench and a checklist.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

__all__ = [
    "RIGS",
    "RIG_NAMES",
    "DriveSpec",
    "RigError",
    "RigSpec",
    "get_rig",
    "rig_names",
]

#: How a rig moves one coordinate.
DriveKind = Literal["displacement", "force", "none"]


class RigError(ValueError):
    """A rig is unknown, malformed, or incompatible with an assembly."""


@dataclass(frozen=True)
class DriveSpec:
    """
    One coordinate a rig drives, in the document's own spelling.

    The name is a *drive coordinate* -- `wheel_drive_L`, `rack_drive` -- and not
    the case layer's grouping key (`wheel`, `rack`).  The two namespaces are
    deliberately different and mixing them is an interface error, so the rig
    speaks the one an assembly reports in `AssemblyCapabilities.drive_coordinates`.
    """

    coordinate: str
    kind: DriveKind = "displacement"
    #: Coordinates this one moves together with, for a symmetric shorthand.
    coupled_with: tuple[str, ...] = ()
    #: Whether the assembly has to provide this coordinate, or the rig brings it
    #: itself.
    #:
    #: The distinction is what "which coordinate, provided by whom" turns on.  A
    #: wheel-travel or rack coordinate is the assembly's to offer, and one it
    #: cannot is dropped.  A road height or a steering-wheel angle is the *rig's own
    #: input*: it perturbs the assembly from outside, so requiring the assembly to
    #: declare it would make every dynamics bench unusable on every model.
    from_assembly: bool = True


@dataclass(frozen=True)
class RigSpec:
    """One test bench: what it drives, how it solves, and what it measures."""

    name: str
    #: The drive coordinates the rig would like to drive, in order.
    drives: tuple[DriveSpec, ...] = ()
    #: Minimum-unit outputs only this bench produces.
    outputs: tuple[str, ...] = ()
    #: The study this bench belongs to, or None for a bench that fits either.
    study: str | None = None
    #: Whether this bench supplies the wheels itself (a single-axle rig does).
    supplies_wheels: bool = False
    description: str = ""

    def coordinate_names(self) -> tuple[str, ...]:
        """Return the coordinates this rig drives, in declaration order."""
        return tuple(drive.coordinate for drive in self.drives)

    def wants(self, coordinate: str) -> bool:
        """Return whether this rig drives a coordinate."""
        return coordinate in self.coordinate_names()


#: The rigs the package ships.  Names match the families callers already use, so a
#: request that named a family keeps working while the two axes become separable.
RIGS: dict[str, RigSpec] = {
    "kc_quasi_static": RigSpec(
        name="kc_quasi_static",
        study="quasi_static",
        supplies_wheels=True,
        drives=(
            DriveSpec("wheel_drive_L", coupled_with=("wheel_drive_R",)),
            DriveSpec("wheel_drive_R"),
            DriveSpec("rack_drive"),
        ),
        outputs=("wheel_load", "rig_frame_pose"),
        description=(
            "The suspension K&C bench.  It supplies the wheels (a single-axle "
            "assembly builds no wheel body) and drives wheel travel, with the rack "
            "driven only when the assembly has steering to drive."
        ),
    ),
    "axle_dynamic": RigSpec(
        name="axle_dynamic",
        study="dynamic",
        supplies_wheels=True,
        drives=(DriveSpec("road_height", kind="displacement", from_assembly=False),),
        outputs=("wheel_load", "road_height"),
        description=(
            "The same bench, run in time: the road moves rather than the wheel "
            "centre, and the tire works in full."
        ),
    ),
    "vehicle_kc": RigSpec(
        name="vehicle_kc",
        study="quasi_static",
        drives=(
            DriveSpec("wheel_drive_L", coupled_with=("wheel_drive_R",)),
            DriveSpec("wheel_drive_R"),
            DriveSpec("rack_drive"),
        ),
        outputs=("wheel_load", "rig_frame_pose"),
        description="The full-vehicle K&C bench; the vehicle builds its own wheels.",
    ),
    "vehicle_dynamic": RigSpec(
        name="vehicle_dynamic",
        study="dynamic",
        drives=(DriveSpec("road_height", kind="displacement", from_assembly=False),),
        outputs=("wheel_load", "road_height"),
        description="The full-vehicle dynamics bench.",
    ),
    "handling": RigSpec(
        name="handling",
        study="dynamic",
        drives=(
            DriveSpec("steering_wheel_angle", kind="displacement", from_assembly=False),
            DriveSpec("road_height", kind="displacement", from_assembly=False),
        ),
        outputs=("wheel_load", "steering_output"),
        description="The handling bench: steering input plus road.",
    ),
    "ride_four_post": RigSpec(
        name="ride_four_post",
        study="dynamic",
        drives=(
            DriveSpec("road_height_fl", kind="displacement", from_assembly=False),
            DriveSpec("road_height_fr", kind="displacement", from_assembly=False),
            DriveSpec("road_height_rl", kind="displacement", from_assembly=False),
            DriveSpec("road_height_rr", kind="displacement", from_assembly=False),
        ),
        outputs=("wheel_load", "post_displacement"),
        description="The four-post ride bench: one vertical input per corner.",
    ),
    "ride_random_road": RigSpec(
        name="ride_random_road",
        study="dynamic",
        drives=(DriveSpec("road_height", kind="displacement", from_assembly=False),),
        outputs=("wheel_load", "road_height"),
        description="The random-road ride bench.",
    ),
}

#: The rig names in a stable order.
RIG_NAMES: tuple[str, ...] = tuple(RIGS)


def rig_names() -> tuple[str, ...]:
    """Return the rig names in a stable order."""
    return RIG_NAMES


def get_rig(name: str) -> RigSpec:
    """Return a rig by name, naming the unknown one if it is not registered."""
    try:
        return RIGS[str(name).strip().lower()]
    except KeyError as error:
        known = ", ".join(RIG_NAMES)
        raise RigError(f"unknown rig {name!r}; the registered rigs are {known}") from error
