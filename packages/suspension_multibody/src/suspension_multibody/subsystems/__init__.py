"""
Subsystems: the middle layer between a template and an assembly.

A *template* is what an expert writes.  A *subsystem* is one instance of it --
the left/right suspension pair, the steering, the wheels, the chassis, the brake,
the drive -- with concrete geometry and properties filled in.  An *assembly* is
some subsystems plus a rig.

This package holds the pieces that make that split real:

* `types.py` -- what a subsystem hands back (declarations, not runtime objects);
* `geometry.py` -- hardpoint lookup, mirroring and body-local conversion, shared
  with the assembly layer;
* `capabilities.py` -- what an assembly can offer a rig, so a rig never has to
  guess by inspecting body names;
* `assembly.py` -- the role assembly path, which turns a template instance into a
  subsystem contribution without ever asking which template it was;
* `suspension.py`, `steering.py`, `wheel.py`, `chassis.py`, `brake.py`,
  `drive.py` -- the six subsystems themselves.

The six are the user's decision, and this package is where that decision lives.
`brake` and `drive` are simplified providers: a template under their role that
owns no bodies and only emits a wheel torque.  They are replaceable -- a detailed
template registers under the same role, and `assembly.py` does not change.
"""

from . import brake, drive
from .assembly import assemble_from_template
from .brake import (
    SIMPLIFIED_BRAKE,
    SIMPLIFIED_BRAKE_NAME,
)
from .brake import (
    register_simplified as register_brake_simplified,
)
from .capabilities import (
    ALL_SUBSYSTEMS,
    DRIVE_COORDINATE_NAMES,
    RACK_COORDINATES,
    WHEEL_COORDINATES,
    AssemblyCapabilities,
    CapabilityError,
    capabilities_for,
)
from .drive import (
    SIMPLIFIED_DRIVE,
    SIMPLIFIED_DRIVE_NAME,
)
from .drive import (
    register_simplified as register_drive_simplified,
)
from .types import (
    DEFAULT_AXLE_SUBSYSTEMS,
    DEFAULT_VEHICLE_SUBSYSTEMS,
    ELEMENT_KINDS,
    MODES,
    SIDES,
    SUBSYSTEM_ROLES,
    WHEELS,
    AssemblyRequest,
    Connection,
    ResolvedElement,
    SubsystemContext,
    SubsystemOutput,
    merge_outputs,
)

__all__ = [
    "ALL_SUBSYSTEMS",
    "DEFAULT_AXLE_SUBSYSTEMS",
    "DEFAULT_VEHICLE_SUBSYSTEMS",
    "DRIVE_COORDINATE_NAMES",
    "ELEMENT_KINDS",
    "MODES",
    "RACK_COORDINATES",
    "SIDES",
    "SUBSYSTEM_ROLES",
    "WHEEL_COORDINATES",
    "WHEELS",
    "SIMPLIFIED_BRAKE",
    "SIMPLIFIED_BRAKE_NAME",
    "SIMPLIFIED_DRIVE",
    "SIMPLIFIED_DRIVE_NAME",
    "AssemblyCapabilities",
    "AssemblyRequest",
    "CapabilityError",
    "Connection",
    "ResolvedElement",
    "SubsystemContext",
    "SubsystemOutput",
    "assemble_from_template",
    "assemble_from_template",
    "brake",
    "capabilities_for",
    "drive",
    "merge_outputs",
    "register_brake_simplified",
    "register_drive_simplified",
]
