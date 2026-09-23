"""
The one joint definition table.

Three places used to spell out the joint types independently, and they could
drift apart silently:

* the kernel's `contract_registry.cpp`, which names each type and its row count;
* `preparation/assembly/types.py`, which holds one dataclass per type;
* the authoring layer, which mapped a dataclass name to the document's `type`
  string -- and only did so for three of the eight real joints, rejecting the
  rest.

This package is the Python mirror of the first, and the single source of the
third.  The row counts are the kernel's: `mb_joint/types.hpp` is the C++ source
of truth and `contract_registry.cpp` restates it for the registry, so this table
mirrors the registry and a test pins the two together.

Two namespaces live here and must not be mixed:

* **joints** -- the eight real connection types (`spherical` .. `convel`);
* **driven coordinates** -- the two *prescribed* degrees of freedom
  (`driven_translation`, `driven_rotation`).

A driven coordinate is not a joint: it fixes a relative degree of freedom to a
signal rather than removing it.  It travels in the same document array because
that is where the kernel looks for it, but keeping it in the joint table would
make "eight joints" mean "ten things".
"""

from .table import (
    COMMON_FIELDS,
    DRIVEN_COORDINATES,
    DRIVEN_KINDS,
    JOINT_KINDS,
    JOINT_ROWS,
    JOINT_TYPES,
    SCHEMA_KINDS,
    JointDefinition,
    JointTableError,
    axis_fields_for_schema_kind,
    definition_for,
    document_type,
    document_type_for_schema_kind,
    kernel_row_count,
    required_axis_fields,
    rows_for,
)
from .validate import JointAvailabilityError, validate_joint_axes

__all__ = [
    "COMMON_FIELDS",
    "DRIVEN_COORDINATES",
    "DRIVEN_KINDS",
    "JOINT_KINDS",
    "JOINT_ROWS",
    "JOINT_TYPES",
    "SCHEMA_KINDS",
    "JointAvailabilityError",
    "JointDefinition",
    "JointTableError",
    "axis_fields_for_schema_kind",
    "definition_for",
    "document_type",
    "document_type_for_schema_kind",
    "kernel_row_count",
    "required_axis_fields",
    "rows_for",
    "validate_joint_axes",
]
