"""
Subsystems: the middle layer between a template and an assembly.

A *template* is what an expert writes.  A *subsystem* is one instance of it --
the left/right suspension pair, the steering, the wheels, the chassis, the brake,
the drive -- with concrete geometry and properties filled in.  An *assembly* is
some subsystems plus a rig.

This package holds the pieces that make that split real:

* `types.py` -- what a subsystem hands back (declarations, not runtime objects);
* `capabilities.py` -- what an assembly can offer a rig, so a rig never has to
  guess by inspecting body names;
* `suspension.py`, `steering.py`, `wheel.py`, `chassis.py`, `brake.py`,
  `drive.py` -- the six subsystems themselves.

The six are the user's decision, and this package is where that decision lives.
"""

from .capabilities import (
    ALL_SUBSYSTEMS,
    DRIVE_COORDINATE_NAMES,
    RACK_COORDINATES,
    WHEEL_COORDINATES,
    AssemblyCapabilities,
    CapabilityError,
    capabilities_for,
)
from .types import SubsystemOutput, merge_outputs

__all__ = [
    "ALL_SUBSYSTEMS",
    "DRIVE_COORDINATE_NAMES",
    "RACK_COORDINATES",
    "WHEEL_COORDINATES",
    "AssemblyCapabilities",
    "CapabilityError",
    "SubsystemOutput",
    "capabilities_for",
    "merge_outputs",
]
