"""
The joint definition table itself.

Row counts mirror `packages/suspension_kernel/cpp/src/contract/contract_registry.cpp`
(which in turn mirrors `mb_joint/types.hpp`).  The authoring name is the dataclass
name in `preparation/assembly/types.py`; the kernel name is what the contract
document writes into a joint's `type` field.

`constant_velocity` is the one place the two names differ.  The document spelling
is `convel`, and that translation used to be written out twice -- in
`cases/axle_dynamic.py` and in `cases/vehicle_dynamic.py`.  It lives here now, so
there is exactly one place to change.
"""

from __future__ import annotations

from dataclasses import dataclass

__all__ = [
    "COMMON_FIELDS",
    "DRIVEN_COORDINATES",
    "DRIVEN_KINDS",
    "JOINT_KINDS",
    "JOINT_ROWS",
    "JOINT_TYPES",
    "JointDefinition",
    "JointTableError",
    "SCHEMA_KINDS",
    "axis_fields_for_schema_kind",
    "definition_for",
    "document_type",
    "document_type_for_schema_kind",
    "kernel_row_count",
    "required_axis_fields",
    "rows_for",
]


class JointTableError(ValueError):
    """A joint name is unknown, or the table itself is inconsistent."""


#: The document keys every joint entry carries regardless of type.
COMMON_FIELDS: tuple[str, ...] = (
    "name",
    "type",
    "body_a",
    "body_b",
    "point_a",
    "point_b",
)


@dataclass(frozen=True)
class JointDefinition:
    """
    One joint type: what it is called, how many rows it owns, what it needs.

    A joint is named twice in the authoring layer, and the table carries both so
    the translation lives in exactly one place:

    * `authoring_name` -- the dataclass in `preparation/assembly/types.py`, used
      by the kc authoring path;
    * `schema_kind` -- the `kind` literal in the axle/vehicle dynamic schemas,
      used by the dynamic authoring path.

    They agree for seven of the eight joints and differ for the eighth:
    `ConstantVelocityJoint` / `constant_velocity` both document as `convel`.

    `rows` is the constraint row count the kernel lays out.  `axis_fields` names
    the document keys beyond `COMMON_FIELDS` that the encoding must supply: a
    joint that needs an axis cannot be written without one, and the authoring
    layer has to say so rather than hand the kernel a degenerate axis for it to
    reject later.  The kernel treats every axis as optional with a default, so a
    missing axis is *silently* a wrong model -- which is why the check belongs
    here.
    """

    #: The dataclass name in `preparation/assembly/types.py` (empty if none).
    authoring_name: str
    #: The `type` string the contract document carries.
    kernel_name: str
    #: Constraint rows this type contributes.
    rows: int
    #: Document keys beyond `COMMON_FIELDS` this type needs.
    axis_fields: tuple[str, ...] = ()
    #: The `kind` literal the dynamic schemas use (empty for driven coordinates).
    schema_kind: str = ""


#: The eight real joints, in the kernel's enumeration order
#: (`mb_model/enums.hpp`: AXLE_SPHERICAL = 0 .. AXLE_CONVEL = 7).
#:
#: Row counts are `kJointTypeInfo` in `mb_joint/types.hpp`:
#: spherical 3, revolute 5, fixed 6, prismatic 5,
#: universal 4, cylindrical 4, inplane 1, convel 4.
#:
#: `axis_fields` mirrors the assembly dataclasses field for field: `InPlaneJoint`
#: carries only `axis_a` (the plane normal), `ConstantVelocityJoint` carries both
#: a primary and a secondary axis on each body.
JOINT_TYPES: tuple[JointDefinition, ...] = (
    JointDefinition("BallJoint", "spherical", 3, (), "spherical"),
    JointDefinition(
        "RevoluteJoint", "revolute", 5, ("axis_a", "axis_b"), "revolute"
    ),
    JointDefinition("WeldJoint", "fixed", 6, (), "fixed"),
    JointDefinition(
        "PrismaticJoint", "prismatic", 5, ("axis_a", "axis_b"), "prismatic"
    ),
    JointDefinition(
        "UniversalJoint", "universal", 4, ("axis_a", "axis_b"), "universal"
    ),
    JointDefinition(
        "CylindricalJoint", "cylindrical", 4, ("axis_a", "axis_b"), "cylindrical"
    ),
    JointDefinition("InPlaneJoint", "inplane", 1, ("axis_a",), "inplane"),
    JointDefinition(
        "ConstantVelocityJoint",
        "convel",
        4,
        ("axis_a", "axis_b", "axis_a_secondary", "axis_b_secondary"),
        "constant_velocity",
    ),
)

#: The two prescribed degrees of freedom.  A driven coordinate fixes one relative
#: degree of freedom to a signal; it is not a connection, so it is kept out of
#: `JOINT_TYPES` -- the two namespaces share the document array, not the meaning.
#: `authoring_name` is the `kind` string the axle/vehicle dynamic schemas use.
DRIVEN_COORDINATES: tuple[JointDefinition, ...] = (
    JointDefinition("translation", "driven_translation", 1, ("axis_b",)),
    JointDefinition("rotation", "driven_rotation", 1, ("axis_b",)),
)

#: Authoring dataclass name -> definition, for the eight joints.
JOINT_KINDS: dict[str, JointDefinition] = {
    definition.authoring_name: definition for definition in JOINT_TYPES
}

#: Dynamic schema `kind` -> definition, for the eight joints.  This is what lets
#: the axle/vehicle dynamic authoring paths drop their own `convel` translation.
SCHEMA_KINDS: dict[str, JointDefinition] = {
    definition.schema_kind: definition
    for definition in JOINT_TYPES
    if definition.schema_kind
}

#: Driven `kind` -> definition.
DRIVEN_KINDS: dict[str, JointDefinition] = {
    definition.authoring_name: definition for definition in DRIVEN_COORDINATES
}

#: Kernel name -> row count, for the eight joints.  This is the mirror the
#: consistency test compares against `contract_registry.cpp`.
JOINT_ROWS: dict[str, int] = {
    definition.kernel_name: definition.rows for definition in JOINT_TYPES
}


def _check_table() -> None:
    """Reject a table that cannot be trusted, where it is written."""
    if len(JOINT_TYPES) != 8:
        raise JointTableError(f"expected 8 real joints, found {len(JOINT_TYPES)}")
    if len(DRIVEN_COORDINATES) != 2:
        raise JointTableError(
            f"expected 2 driven coordinates, found {len(DRIVEN_COORDINATES)}"
        )
    names = [definition.kernel_name for definition in JOINT_TYPES]
    if len(set(names)) != len(names):
        raise JointTableError("joint kernel names must be unique")
    authors = [definition.authoring_name for definition in JOINT_TYPES]
    if len(set(authors)) != len(authors):
        raise JointTableError("joint authoring names must be unique")
    overlap = set(names) & {definition.kernel_name for definition in DRIVEN_COORDINATES}
    if overlap:
        raise JointTableError(
            f"driven coordinates must not appear in the joint table: {sorted(overlap)}"
        )
    for definition in JOINT_TYPES + DRIVEN_COORDINATES:
        if definition.rows < 1:
            raise JointTableError(
                f"joint {definition.kernel_name!r} must own at least one row"
            )
        unknown = set(definition.axis_fields) - {
            "axis_a",
            "axis_b",
            "axis_a_secondary",
            "axis_b_secondary",
        }
        if unknown:
            raise JointTableError(
                f"joint {definition.kernel_name!r} names unknown axis fields "
                f"{sorted(unknown)}"
            )


_check_table()


def definition_for(authoring_name: str) -> JointDefinition:
    """
    Return the definition for an assembly-layer constraint.

    Raises `JointTableError` naming the unknown type rather than a bare
    `KeyError`: the caller is an authoring path, and "unsupported joint X" is the
    message a user needs.
    """
    try:
        return JOINT_KINDS[authoring_name]
    except KeyError as exc:
        known = ", ".join(sorted(JOINT_KINDS))
        raise JointTableError(
            f"unsupported joint {authoring_name!r}; known joints are {known}"
        ) from exc


def document_type(authoring_name: str) -> str:
    """
    Translate an assembly-layer constraint name to the document's `type` string.

    This is the single home of the `constant_velocity -> convel` translation.
    """
    return definition_for(authoring_name).kernel_name


def document_type_for_schema_kind(schema_kind: str) -> str:
    """
    Translate a dynamic schema `kind` to the document's `type` string.

    The dynamic authoring paths name joints with the schema literal
    (`constant_velocity`), which is a different spelling from the kc path's
    dataclass name (`ConstantVelocityJoint`).  Both resolve here, so the
    `convel` translation exists once for the whole authoring layer.
    """
    try:
        return SCHEMA_KINDS[schema_kind].kernel_name
    except KeyError as exc:
        known = ", ".join(sorted(SCHEMA_KINDS))
        raise JointTableError(
            f"unsupported joint kind {schema_kind!r}; known kinds are {known}"
        ) from exc


def axis_fields_for_schema_kind(schema_kind: str) -> tuple[str, ...]:
    """Return the axis document keys a dynamic-schema joint kind needs."""
    try:
        return SCHEMA_KINDS[schema_kind].axis_fields
    except KeyError as exc:
        raise JointTableError(f"unsupported joint kind {schema_kind!r}") from exc


def rows_for(kernel_name: str) -> int:
    """Return the row count the kernel owns for a document `type` string."""
    try:
        return JOINT_ROWS[kernel_name]
    except KeyError as exc:
        known = ", ".join(sorted(JOINT_ROWS))
        raise JointTableError(
            f"unknown joint type {kernel_name!r}; known types are {known}"
        ) from exc


def required_axis_fields(kernel_name: str) -> tuple[str, ...]:
    """Return the axis document keys a joint type needs."""
    for definition in JOINT_TYPES + DRIVEN_COORDINATES:
        if definition.kernel_name == kernel_name:
            return definition.axis_fields
    raise JointTableError(f"unknown joint type {kernel_name!r}")


def kernel_row_count() -> int:
    """Return the total rows the eight real joints account for."""
    return sum(JOINT_ROWS.values())
