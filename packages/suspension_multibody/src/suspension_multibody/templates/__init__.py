"""
Templates: the layer the experts write and the users never see.

A template declares a subsystem's parts, its connection points (each carrying a
joint column and a bushing column), the elastic elements and property slots it
needs, and the outputs it contributes.  It is one data structure for every kind
of subsystem -- suspension, steering, wheel, chassis, brake, drive -- because the
differences between them are content, not shape.  That is also how Adams Car
works: a `.tpl` is one file format and `MAJOR_ROLE` is a field inside it.

`roles.py` holds what each role promises; `model.py` holds the structure;
`registry.py` is how templates are named; `builtin.py` is the double-wishbone
template transcribed from the existing `symmetric_proxy` assembly.
"""

from .builtin import (
    BRAKE,
    CHASSIS,
    DEFAULT_MOUNT_STIFFNESS,
    DOUBLE_WISHBONE,
    DOUBLE_WISHBONE_NAME,
    DRIVE,
    RACK_HOUSING_MASS,
    STEERING,
    STEERING_GUIDED,
    VEHICLE_BODY,
    WHEEL,
    WHEEL_HUB_MASS,
    register_builtins,
)
from .instantiate import (
    ACTIVATED_MODES,
    MODES,
    SubsystemInstance,
    activated_column,
    instantiate,
    resolve_properties,
)
from .model import (
    ConnectionDefinition,
    OutputDeclaration,
    PartDefinition,
    PropertySlot,
    SlotValue,
    Template,
    TemplateError,
    template_dumps,
    template_from_json,
    template_loads,
    template_to_json,
)
from .registry import clear, get, names, register, registered
from .roles import ROLES, RoleSpec, RoleSpecError, get_role, role_names

#: Register the built-ins at import, so the registry is never empty in practice.
register_builtins()

__all__ = [
    "ACTIVATED_MODES",
    "BRAKE",
    "CHASSIS",
    "DEFAULT_MOUNT_STIFFNESS",
    "DOUBLE_WISHBONE",
    "DOUBLE_WISHBONE_NAME",
    "DRIVE",
    "MODES",
    "ROLES",
    "STEERING",
    "STEERING_GUIDED",
    "SubsystemInstance",
    "RACK_HOUSING_MASS",
    "VEHICLE_BODY",
    "WHEEL",
    "WHEEL_HUB_MASS",
    "ConnectionDefinition",
    "OutputDeclaration",
    "PartDefinition",
    "PropertySlot",
    "RoleSpec",
    "SlotValue",
    "RoleSpecError",
    "Template",
    "TemplateError",
    "activated_column",
    "clear",
    "instantiate",
    "resolve_properties",
    "get",
    "get_role",
    "names",
    "register",
    "register_builtins",
    "registered",
    "role_names",
    "template_dumps",
    "template_from_json",
    "template_loads",
    "template_to_json",
]
