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

from dataclasses import dataclass, field

import numpy as np

from .model import ConnectionDefinition, Template, TemplateError

__all__ = [
    "ACTIVATED_MODES",
    "MODES",
    "SubsystemInstance",
    "activated_column",
    "instantiate",
]

#: The two modes a template can be instantiated for.
MODES: tuple[str, str] = ("K", "C")

#: Which column each mode activates, for a connection that declares both.
ACTIVATED_MODES: dict[str, str] = {"K": "joint", "C": "bushing"}


def activated_column(connection: ConnectionDefinition, mode: str) -> str | None:
    """
    Return the column `mode` activates at `connection`, or `None` for neither.

    The rule, stated once:

    * both columns declared -- the mode chooses (`joint` in K, `bushing` in C);
    * only one column declared -- that column, in *both* modes;
    * neither -- the point locates geometry and carries no constraint.

    The second bullet is the one that is easy to get wrong.  A tie rod end has no
    bushing column, so it is a joint even in C mode; discarding it would drop four
    of the nine joints the C-mode axle actually has.
    """
    if mode not in MODES:
        raise TemplateError(f"unknown mode {mode!r}; modes are K and C")
    columns = connection.columns()
    if not columns:
        return None
    if len(columns) == 1:
        return columns[0]
    return ACTIVATED_MODES[mode]


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
    #: Property values by slot name: supplied values over the template's defaults.
    properties: dict[str, float] = field(default_factory=dict)
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

    def stiffness_for(self, connection_name: str) -> float:
        """
        Return the stiffness the bushing column at `connection_name` resolves to.

        The connection names the slot its bushing column refers to; the value is
        that slot's, which is how a template says "the mount bushing" without
        knowing what number an author will put there.
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
        return slot.default

    def bushing_stiffness(self) -> dict[str, float]:
        """Return the resolved stiffness of every active bushing, by connection."""
        return {name: self.stiffness_for(name) for name in self.bushings}

    def bushing_stiffness_matrix(self, connection_name: str) -> np.ndarray:
        """
        Return the 6x6 stiffness matrix for one active bushing.

        A slot value is a translational stiffness, so it goes on the three
        translational diagonals; the rotational diagonals stay zero unless a
        template says otherwise.  This mirrors how the model's own `Bushing6x6`
        entries are read.
        """
        value = self.stiffness_for(connection_name)
        matrix = np.zeros((6, 6))
        for index in range(3):
            matrix[index, index] = value
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
    properties: dict[str, float] | None = None,
) -> SubsystemInstance:
    """
    Instantiate `template` for `mode`, activating the columns that mode selects.

    The role contract is re-checked here rather than only at registration: a
    template can be registered with placeholders and filled in later, and a
    failure has to name the missing mount or slot where someone tries to use it.
    """
    if mode not in MODES:
        raise TemplateError(f"unknown mode {mode!r}; modes are K and C")
    template.check_role_contract()
    supplied: dict[str, float] = dict(properties or {})
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


def _check_property_names(template: Template, supplied: dict[str, float]) -> None:
    """Reject a property whose name the template does not declare."""
    declared = {slot.name for slot in template.property_slots}
    unknown = sorted(set(supplied) - declared)
    if unknown:
        raise TemplateError(
            f"template {template.name!r} does not declare property slot(s) {unknown}"
        )


def _resolve_defaults(
    template: Template, supplied: dict[str, float]
) -> dict[str, float]:
    """
    Merge supplied properties over the template's own defaults.

    A value the template carries is a real value of the simplified model, so it is
    kept in the instance's property map rather than looked up again at each use.
    A `None` default means "the author must supply this".
    """
    merged: dict[str, float] = {}
    for slot in template.property_slots:
        if slot.name in supplied:
            merged[slot.name] = float(supplied[slot.name])
        elif slot.default is not None:
            merged[slot.name] = float(slot.default)
    return merged
