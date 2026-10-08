"""Ordinary subsystem templates for test fixtures and their declared inputs."""

from .rig import RIG_NAMES, RIGS, DriveSpec, RigError, RigSpec, get_rig, rig_names
from .templates import generic_rig_subsystem, generic_rig_template

__all__ = [
    "RIGS", "RIG_NAMES", "DriveSpec", "RigError", "RigSpec", "get_rig",
    "rig_names", "generic_rig_subsystem", "generic_rig_template",
]
