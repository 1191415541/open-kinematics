"""
Instantiation: turning a template into a subsystem, for one mode.

A template says what the subsystem *is*.  Instantiating it says which mode is
active and what its property slots hold, and the result is a `SubsystemInstance`
in the shapes 04's subsystem layer already builds from: the same declarations,
driven by the template rather than by a second copy of the logic.

Two rules make the K/C story real rather than rhetorical:

* **the placement is mode-independent.**  A connection's body, its point role and
  the identity of the objects it attaches never depend on the mode, so switching
  mode cannot move geometry;
* **the columns are the only thing the mode decides.**  A point that declares a
  joint column and no bushing column stays a joint in *both* modes -- the C-mode
  axle really does carry nine joints (four outboard ball joints, four tie rod
  ends, the rack guide), so "C activates bushings" must not be read as "C discards
  joints".
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field

import numpy as np

from .model import (
    ACTIVATED_MODES,
    MODES,
    ConnectionDefinition,
    SlotValue,
    Template,
    TemplateError,
)

__all__ = [
    "ACTIVATED_MODES",
    "MODES",
    "SubsystemInstance",
    "activated_column",
    "instantiate",
    "resolve_properties",
    "slot_element_type",
]


def activated_column(connection: ConnectionDefinition, mode: str) -> str | None:
    """
    Return the column `mode` activates at `connection`, or `None` for neither.

    Delegates to the connection's own declaration, which says which modes activate
    each column.  The rule it encodes, stated once:

    * both columns declared and both active -- the mode chooses (`joint` in K,
      `bushing` in C);
    * one column declared and active in this mode -- that column (the arm outer
      points and tie rod ends stay joints in C mode; discarding them would drop
      four of the nine joints the C-mode axle actually has);
    * neither active in this mode -- the point locates geometry and constrains
      nothing.  That is a real state, not an omission: in K mode the arm pivots on
      a single revolute whose axis runs to the inboard rear point, so the rear
      point carries no row of its own.
    """
    return connection.active_column(mode)


@dataclass(frozen=True)
class SubsystemInstance:
    """
    One template, instantiated for one mode.

    Carries no geometry of its own: `points` names the *roles* the assembly
    resolves against a concrete model.  That is deliberate -- it is what makes
    `with_mode` a change of activation rather than a second build, and it is what
    lets the same instantiation be checked against two different models.
    """

    template: Template
    mode: str
    #: Resolved constitutive values by slot name: supplied laws over the template's
    #: defaults.  A caller may still pass plain numbers; they are normalised here.
    properties: Mapping[str, SlotValue] = field(default_factory=dict)
    #: Template part names, in declaration order.
    bodies: tuple[str, ...] = ()
    #: `(connection name, hardpoint role)` in declaration order.
    points: tuple[tuple[str, str], ...] = ()
    #: Connection names that are ideal joints in this mode, in declaration order.
    joints: tuple[str, ...] = ()
    #: Connection names that are compliance elements in this mode, in order.
    bushings: tuple[str, ...] = ()
    #: Connection names that carry no constraint in this mode, in order.
    inert: tuple[str, ...] = ()

    def law_for(self, connection_name: str) -> SlotValue:
        """
        Return the resolved constitutive value the slot at `connection_name` holds.

        `stiffness_for` is the scalar view of this; a consumer that has to build
        the kernel's parameter block needs the whole law -- its element type, its
        model and its curve -- and reading it here is what keeps that consumer from
        reaching back into the property file a second time.
        """
        connection = self._connection(connection_name)
        if connection is None or connection.bushing is None:
            raise TemplateError(
                f"connection {connection_name!r} has no bushing column in mode "
                f"{self.mode!r}"
            )
        slot = self._slot_for(connection)
        if slot is None:
            raise TemplateError(
                f"connection {connection_name!r} names bushing "
                f"{connection.bushing!r}, which no property slot of template "
                f"{self.template.name!r} feeds; add it to that slot's "
                "`connections`"
            )
        if slot.name in self.properties:
            return self.properties[slot.name]
        if slot.default is None:
            raise TemplateError(
                f"slot {slot.name!r} used by connection {connection_name!r} has "
                "no default and no supplied value"
            )
        return SlotValue(
            element_type=slot_element_type(slot.name), scalar=float(slot.default)
        )

    def stiffness_for(self, connection_name: str) -> float:
        """
        Return the stiffness the bushing column at `connection_name` resolves to.

        The connection names the slot its bushing column refers to; the value is
        that slot's, which is how a template says "the mount bushing" without
        knowing what number an author will put there.
        """
        return float(self.law_for(connection_name))

    def bushing_stiffness(self) -> dict[str, float]:
        """Return the resolved stiffness of every active bushing, by connection."""
        return {name: self.stiffness_for(name) for name in self.bushings}

    def bushing_stiffness_matrix(self, connection_name: str) -> np.ndarray:
        """
        Return the 6x6 stiffness matrix for one active bushing.

        A file may state the table itself, in which case that *is* the mount and is
        returned as written -- the rotational diagonals a scalar cannot express are
        what makes a compliant assembly a mechanism or not.  A slot that holds only
        a number goes on the three translational diagonals, which mirrors how the
        model's own `Bushing6x6` entries are read.
        """
        law = self.law_for(connection_name)
        if law.matrix:
            return np.asarray(law.matrix, dtype=float)
        matrix = np.zeros((6, 6))
        for index in range(3):
            matrix[index, index] = float(law)
        return matrix

    def with_mode(self, mode: str) -> SubsystemInstance:
        """
        Return the same instance with the other mode activated.

        Nothing but the activation changes: same template, same properties, same
        bodies, same point roles in the same order.  `with_mode` is idempotent,
        and its result is bit-identical to instantiating the template afresh in
        that mode -- the assertion that keeps this from becoming a second
        implementation.
        """
        return instantiate(self.template, mode=mode, properties=dict(self.properties))

    def _slot_for(self, connection: ConnectionDefinition):
        """
        Return the property slot that feeds `connection`'s bushing column.

        Found through the slot's own `connections` list, not by matching names:
        the column carries the element name a bushing will get, while the slot
        carries the number, and one slot feeds all eight inboard points.
        """
        return next(
            (
                slot
                for slot in self.template.property_slots
                if connection.name in slot.connections
            ),
            None,
        )

    def _connection(self, name: str) -> ConnectionDefinition | None:
        return next(
            (
                connection
                for connection in self.template.connections
                if connection.name == name
            ),
            None,
        )


def instantiate(
    template: Template,
    *,
    mode: str,
    properties: Mapping[str, float | SlotValue] | None = None,
) -> SubsystemInstance:
    """
    Instantiate `template` for `mode`, activating the columns that mode selects.

    The role contract is re-checked here rather than only at registration: a
    template can be registered with placeholders and filled in later, and a
    failure has to name the missing mount or slot where someone tries to use it.

    Supplied values may be resolved laws or plain numbers; both arrive at the
    instance as resolved values, so a caller that had a number did not have to
    learn a second shape and a caller with a property file does not have to give
    up what the file said about the law.
    """
    if mode not in MODES:
        raise TemplateError(f"unknown mode {mode!r}; modes are K and C")
    template.check_role_contract()
    supplied = {name: _as_slot_value(name, value) for name, value in (properties or {}).items()}
    _check_property_names(template, supplied)
    merged = _resolve_defaults(template, supplied)
    template.check_filled(dict(merged))

    joints: list[str] = []
    bushings: list[str] = []
    inert: list[str] = []
    for connection in template.connections:
        column = activated_column(connection, mode)
        if column == "joint":
            joints.append(connection.name)
        elif column == "bushing":
            bushings.append(connection.name)
        else:
            inert.append(connection.name)

    return SubsystemInstance(
        template=template,
        mode=mode,
        properties=merged,
        bodies=tuple(part.name for part in template.parts),
        points=tuple(
            (connection.name, connection.role) for connection in template.connections
        ),
        joints=tuple(joints),
        bushings=tuple(bushings),
        inert=tuple(inert),
    )


def _as_slot_value(name: str, value: object) -> SlotValue:
    """
    Normalise one supplied value into a resolved slot value.

    A number is a linear law of whatever element the slot is named after, which is
    what a template's own default means and what a caller who passed a float means.
    Anything else is refused here rather than at the first arithmetic: a slot whose
    value is a string would otherwise reach the kernel's parameter block and fail
    there, naming the arithmetic instead of the file.
    """
    if isinstance(value, SlotValue):
        return value
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return SlotValue(element_type=slot_element_type(name), scalar=float(value))
    raise TemplateError(
        f"slot {name!r} was given {value!r} ({type(value).__name__}); a slot value "
        "is a resolved law or a number"
    )


def _check_property_names(template: Template, supplied: Mapping[str, SlotValue]) -> None:
    """Reject a property whose name the template does not declare."""
    declared = {slot.name for slot in template.property_slots}
    unknown = sorted(set(supplied) - declared)
    if unknown:
        raise TemplateError(
            f"template {template.name!r} does not declare property slot(s) {unknown}"
        )


def _resolve_defaults(
    template: Template, supplied: Mapping[str, SlotValue]
) -> dict[str, SlotValue]:
    """
    Merge supplied properties over the template's own defaults.

    A value the template carries is a real value of the simplified model, so it is
    kept in the instance's property map rather than looked up again at each use.
    A `None` default means "the author must supply this".
    """
    merged: dict[str, SlotValue] = {}
    for slot in template.property_slots:
        if slot.name in supplied:
            merged[slot.name] = supplied[slot.name]
        elif slot.default is not None:
            merged[slot.name] = SlotValue(
                element_type=slot_element_type(slot.name), scalar=float(slot.default)
            )
    return merged

#: Which entry field carries the scalar a slot reads, per entry kind.
#:
#: A damper's single number is its viscous damping; everything else's is a
#: stiffness.  Mapping it here keeps the template's slot vocabulary ("how stiff")
#: separate from the file's element vocabulary ("what kind of element").
_SLOT_SCALAR_FIELD: dict[str, str] = {
    "spring": "stiffness",
    "damper": "viscous_damping",
    "bushing6x6": "stiffness",
    "tire": "stiffness",
    "bump_stop": "stiffness",
}

#: Slot names that name an element kind; anything else is a plain number source.
_ELEMENT_SLOT_NAMES: frozenset[str] = frozenset(
    {"spring", "damper", "bump_stop", "bushing", "tire"}
)


def slot_element_type(name: str) -> str:
    """
    Return the element type a slot of this name declares.

    Named ``generic`` when the slot is not named after an element kind -- a mass
    or an inertia is a number too, and claiming it is a spring would be a worse
    answer than saying it is not a law.
    """
    return name if name in _ELEMENT_SLOT_NAMES else "generic"


def resolve_properties(
    template: Template, property_set: object
) -> dict[str, SlotValue]:
    """
    Bind a loaded properties file to a template's slots, producing slot values.

    A properties file is keyed by *property name*, and a template's slots are
    named the same way, so binding is a lookup rather than a translation.  A slot
    the file does not mention keeps the template's own default -- that is what
    "a template may also carry its numbers directly" means in practice, and it is
    why this is an added option rather than a replacement.

    The result has the same *shape* the element-property route produces: resolved
    values carrying the element kind the entry declares, not bare numbers.  The
    two routes into a slot -- this one and `authoring/properties.py` -- therefore
    hand the instance the same thing, which is what makes them one channel with
    two spellings rather than two channels that have to be kept in step.

    A slot the role requires, that the template gives no default for, and that the
    file does not supply, is an error that names the slot and the file: silently
    substituting zero would turn a missing stiffness into a floating linkage.
    """
    entries = getattr(property_set, "entries", None)
    if not isinstance(entries, dict):
        raise TemplateError(
            "resolve_properties expects a loaded properties file (a PropertySet "
            f"with an 'entries' mapping), found {type(property_set).__name__}"
        )
    path = getattr(property_set, "path", "<properties>")
    values: dict[str, SlotValue] = {}
    for slot in template.property_slots:
        entry = entries.get(slot.name)
        if entry is not None:
            kind = str(entry.get("kind"))
            field = _SLOT_SCALAR_FIELD.get(kind, "stiffness")
            value = entry.get(field)
            if isinstance(value, list):
                rows = tuple(tuple(float(item) for item in row) for row in value)
                values[slot.name] = SlotValue(
                    element_type=(
                        kind if kind in _ELEMENT_SLOT_NAMES else slot_element_type(slot.name)
                    ),
                    # The scalar view is kept beside the table: it is what a reader
                    # that only takes a number uses, and for the isotropic mounts the
                    # recorded baselines were solved with, the two agree exactly.
                    scalar=_scalar_from_matrix(path, slot.name, template.name, value),
                    matrix=rows,
                    source=str(path),
                )
                continue
            if not isinstance(value, (int, float)) or isinstance(value, bool):
                raise TemplateError(
                    f"{path}: property {slot.name!r} (kind {entry.get('kind')!r}) "
                    f"has no numeric {field!r} to fill slot {slot.name!r} of "
                    f"template {template.name!r}"
                )
            values[slot.name] = SlotValue(
                # The entry's own kind when it names an element, the slot's name
                # otherwise: a properties file spells a mount bushing
                # `bushing6x6`, which is the entry's vocabulary, while the slot and
                # the kernel speak of a `bushing` law, so the value is normalised to
                # the slot's name rather than carrying the file's spelling on.
                element_type=(
                    kind if kind in _ELEMENT_SLOT_NAMES else slot_element_type(slot.name)
                ),
                scalar=float(value),
                source=str(path),
            )
        elif slot.default is not None:
            values[slot.name] = SlotValue(
                element_type=slot_element_type(slot.name), scalar=float(slot.default)
            )
    required = set(template.role_spec.required_slots)
    missing = sorted(
        slot.name
        for slot in template.property_slots
        if slot.name in required
        and slot.name not in values
        and slot.default is None
    )
    if missing:
        raise TemplateError(
            f"{path}: properties file is missing required slot(s) {missing} for "
            f"template {template.name!r} (role {template.role!r})"
        )
    return values

def _scalar_from_matrix(
    path: object, slot_name: str, template_name: str, matrix: list
) -> float:
    """
    Reduce an isotropic 6x6 stiffness to the scalar a template slot holds.

    A slot is one number (a translational stiffness in N/m), while a properties
    entry may legitimately describe a full `bushing6x6`.  The two agree only when
    the matrix is isotropic on its translational diagonal; anything else cannot be
    represented by the slot, and saying so is better than silently keeping the
    first diagonal entry and dropping the rest of the description.
    """
    diagonal = []
    for index in range(3):
        row = matrix[index] if index < len(matrix) else []
        diagonal.append(float(row[index]) if index < len(row) else 0.0)
    if len({round(value, 12) for value in diagonal}) != 1:
        raise TemplateError(
            f"{path}: property {slot_name!r} has an anisotropic stiffness matrix, "
            f"but template {template_name!r} reads slot {slot_name!r} as a single "
            f"number; its translational diagonal is {diagonal}"
        )
    return diagonal[0]
