"""
What a template offers and what it needs, in the template's own vocabulary.

A template declares ports by *local* name and a role; the assembly turns those
into :class:`~suspension_multibody.modeling.ports.GeometryPort` values once it
knows which entity owns them and what its local frame is.  Keeping the
declaration local is what lets one template be instantiated twice -- the left and
right arms of the same axle are the same template at two mountings, and their
ports must not collide.

A port declaration deliberately carries no coordinates.  Where a port sits is a
property of the *model*, not of the template: a hardpoint moves and the port
moves with it, without the template being edited.  That is exactly the property
``A4`` tests by perturbing hardpoints and comparing the derived attachment
against an independently computed transform.

``PortNeed`` is the other direction: what this template requires a neighbour to
provide.  ``required=False`` marks a branch that may vanish as a whole, and
``bound_outputs`` names the outputs that vanish with it -- recording them here is
what stops a shrink from leaving an output pointing at a branch that is gone.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ..modeling.ports import PortError, PortRequirement

__all__ = [
    "PortDeclaration",
    "PortNeed",
    "declaration_to_port",
    "need_to_requirement",
]


@dataclass(frozen=True)
class PortDeclaration:
    """
    One port a template offers, before it knows where it is mounted.

    ``role`` is the semantic name a requirement matches on.  ``owner`` names the
    template part the port hangs on, or is empty when the port hangs on the
    instance itself (a whole-subsystem channel such as a torque input).
    """

    name: str
    role: str
    #: Template part name the port is attached to; empty means the instance.
    owner: str = ""
    #: Which entity kind this port is: a physical attachment or a channel.
    kind: str = "geometry"
    #: Capabilities this port offers, for capability-based matching.
    capabilities: frozenset[str] = field(default_factory=frozenset)
    #: Side/axle tags, kept exact during matching so L never meets R.
    labels: frozenset[str] = field(default_factory=frozenset)
    #: ``one``, ``many`` or ``none``; see ``modeling.ports``.
    cardinality: str = "one"
    #: Compatibility family for geometry ports: ports in one family may be
    #: substituted for each other.
    family: str = ""
    #: Channel units, for ``kind="channel"``.
    units: str = ""
    #: Channel direction in the owner's local frame, for ``kind="channel"``.
    direction: tuple[float, float, float] | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise PortError("a port declaration needs a name")
        if not self.role:
            raise PortError(f"port declaration {self.name!r} needs a role")
        if self.kind not in ("geometry", "channel"):
            raise PortError(
                f"port declaration {self.name!r} has unknown kind {self.kind!r}"
            )
        if self.cardinality not in ("one", "many", "none"):
            raise PortError(
                f"port declaration {self.name!r} has unknown cardinality "
                f"{self.cardinality!r}"
            )


@dataclass(frozen=True)
class PortNeed:
    """One thing a template needs from its neighbours."""

    role: str
    requires_capabilities: frozenset[str] = field(default_factory=frozenset)
    match_labels: frozenset[str] = field(default_factory=frozenset)
    count: int = 1
    required: bool = True
    #: Outputs that disappear together with an optional branch.
    bound_outputs: tuple[str, ...] = ()
    note: str = ""

    def __post_init__(self) -> None:
        if not self.role:
            raise PortError("a port need requires a role")
        if self.count < 1:
            raise PortError(f"port need count must be >= 1, got {self.count}")


def declaration_to_port(
    declaration: PortDeclaration,
    *,
    instance: tuple[str, ...],
    owner_local: str | None = None,
    pose=None,
):
    """
    Turn a declaration into a concrete port for one mounting.

    ``instance`` is the path the template is mounted at, so two instances of one
    template produce different ids from the same declaration.  ``owner_local``
    overrides the declaration's owner when the caller knows the qualified name
    (a builder that generated the body under a different name).
    """
    from ..modeling.identity import EntityId
    from ..modeling.ports import ChannelPort, GeometryPort

    owner = owner_local if owner_local is not None else declaration.owner
    owner_id = EntityId(instance, owner) if owner else EntityId(instance, declaration.name)
    port_id = EntityId(instance, declaration.name)
    if declaration.kind == "channel":
        return ChannelPort(
            id=port_id,
            owner=owner_id,
            role=declaration.role,
            capabilities=frozenset(declaration.capabilities),
            labels=frozenset(declaration.labels),
            cardinality=declaration.cardinality,  # type: ignore[arg-type]
            units=declaration.units,
            direction=declaration.direction,
        )
    return GeometryPort(
        id=port_id,
        owner=owner_id,
        role=declaration.role,
        capabilities=frozenset(declaration.capabilities),
        labels=frozenset(declaration.labels),
        cardinality=declaration.cardinality,  # type: ignore[arg-type]
        pose=pose if pose is not None else _identity_pose(),
        family=declaration.family,
    )


def _identity_pose():
    """Return the identity pose the spatial layer uses."""
    from ..modeling.primitives.spatial import SE3

    return SE3.identity()


def need_to_requirement(need: PortNeed) -> PortRequirement:
    """Turn a template's need into the requirement the matcher consumes."""
    return PortRequirement(
        role=need.role,
        requires_capabilities=frozenset(need.requires_capabilities),
        match_labels=frozenset(need.match_labels),
        count=need.count,
        required=need.required,
        bound_outputs=tuple(need.bound_outputs),
        note=need.note,
    )
