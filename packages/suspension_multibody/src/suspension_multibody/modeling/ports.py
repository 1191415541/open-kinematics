"""
Ports: what an instance can be connected to, and what it needs connected.

A port is a *semantic* attachment point, not a name to be matched by string
similarity.  It carries the entity it sits on, a role, a local frame, optional
labels and the capacities it offers; a requirement carries the role it wants and
how many links it will accept.  Matching those two is a real decision that can
fail loudly, which is the point: silently picking a port by name proximity is
what produced rigs that bind to whatever happens to be there.

Two port kinds are deliberately distinct types rather than one type with a flag:

* ``GeometryPort`` -- a physical attachment (a suspension pickup, a rig mount).
  It has a local frame and a compatibility family, because two geometric ports
  are only interchangeable when the geometry agrees.
* ``ChannelPort`` -- a signal or torque channel (a drive input, a measurement
  output).  It has units and no frame; forcing it to carry a pose would invent
  geometry that does not exist.

Ports hang on the body that owns them and hold *local* translation/rotation; the
world pose is derived from the owner's pose at binding time. That is what keeps
a rig definition valid when hardpoints move: the owner moves, the local offset
does not.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

from .identity import EntityId
from .primitives.spatial import SE3

__all__ = [
    "ChannelPort",
    "GeometryPort",
    "PortError",
    "PortRequirement",
    "PortSpec",
    "RoadPort",
    "SpinPort",
    "link_counts_agree",
]

#: How many links a port or a requirement will accept.
#: ``ONE`` -- exactly one (a wheel centre belongs to one rig).
#: ``MANY`` -- any number (a shared ground reference).
#: ``NONE`` -- must stay unconnected (a diagnostic-only port).
Cardinality = Literal["one", "many", "none"]

PortKind = Literal["geometry", "channel", "road", "spin"]


class PortError(ValueError):
    """A port declaration or a requirement is not usable as stated."""


@dataclass(frozen=True)
class PortSpec:
    """
    The parts every port has, regardless of kind.

    Kept as a base so ``requires`` matching can read role/labels/cardinality
    without knowing which kind it is looking at.
    """

    #: Stable identity of this port: instance path plus local name.
    id: EntityId
    #: The entity that owns the port -- a body, or the instance itself.
    owner: EntityId
    #: Semantic role, e.g. ``wheel_centre``, ``rack_input``, ``load_output``.
    role: str
    #: What this port can be matched on, beyond its role.
    capabilities: frozenset[str] = field(default_factory=frozenset)
    #: Side/axle style tags (``L``/``R``/``front``), used to keep a left port
    #: from binding to a right requirement.
    labels: frozenset[str] = field(default_factory=frozenset)
    #: How many links this port accepts.
    cardinality: Cardinality = "one"

    def __post_init__(self) -> None:
        if not self.role:
            raise PortError("a port needs a non-empty role")
        if self.cardinality not in ("one", "many", "none"):
            raise PortError(f"unknown cardinality {self.cardinality!r}")


@dataclass(frozen=True)
class GeometryPort(PortSpec):
    """
    A physical attachment point.

    ``pose`` is the port frame *in the owner's local coordinates*.  Two ports are
    interchangeable only when their frames and compatibility families agree, so
    the family is part of the declaration rather than inferred from the role.
    """

    pose: SE3 = field(default_factory=SE3.identity)
    #: Ports in the same family may be substituted for one another.
    family: str = ""

    kind: Literal["geometry"] = field(default="geometry", init=False)


@dataclass(frozen=True)
class ChannelPort(PortSpec):
    """
    A signal or torque channel.

    No frame: a drive input has a direction and units, not a pose.  Inventing an
    identity transform for it would let a geometric match succeed on a channel
    that has no geometry to match.
    """

    #: Units of the channel's value, e.g. ``N``, ``N*m``.
    units: str = ""
    #: The direction the channel acts along, in the owner's local frame.
    direction: tuple[float, float, float] | None = None

    kind: Literal["channel"] = field(default="channel", init=False)

    def __post_init__(self) -> None:
        super(ChannelPort, self).__post_init__()  # type: ignore[misc]
        if self.direction is not None:
            vector = np.asarray(self.direction, dtype=float)
            if vector.shape != (3,) or not np.all(np.isfinite(vector)):
                raise PortError("a channel direction must be three finite values")
            norm = float(np.linalg.norm(vector))
            if norm == 0.0:
                raise PortError("a channel direction must not be the zero vector")


@dataclass(frozen=True)
class RoadPort(PortSpec):
    """A declared road resource binding without a rigid attachment."""

    resource: str = ""
    kind: Literal["road"] = field(default="road", init=False)


@dataclass(frozen=True)
class SpinPort(PortSpec):
    """A rotational coordinate of one explicitly declared bearing joint."""

    coordinate: str = ""
    units: Literal["rad"] = field(default="rad", init=False)
    kind: Literal["spin"] = field(default="spin", init=False)

    def __post_init__(self) -> None:
        super(SpinPort, self).__post_init__()
        if not self.coordinate:
            raise PortError("a spin port requires a joint coordinate")


@dataclass(frozen=True)
class PortRequirement:
    """
    What one instance needs from another.

    ``required=False`` marks a branch that may vanish as a whole -- a rig's rack
    drive on an assembly with no steering.  Only explicitly optional branches may
    disappear; a required one that cannot be met is an error, never a silent
    downgrade.
    """

    #: Role the counterpart port must carry.
    role: str
    #: Capabilities the counterpart must offer, all of them.
    requires_capabilities: frozenset[str] = field(default_factory=frozenset)
    #: Tags that must match exactly (``L`` must meet ``L``).
    match_labels: frozenset[str] = field(default_factory=frozenset)
    #: How many counterpart ports this requirement wants.
    count: int = 1
    required: bool = True
    #: Ids of the outputs/excitations that disappear with an optional branch.
    #: Recorded here so a shrink cannot leave an output pointing at a branch
    #: that is gone.
    bound_outputs: tuple[str, ...] = ()
    #: Why this branch is optional, for the error message.
    note: str = ""
    kind: PortKind | None = None
    units: str = ""
    family: str = ""
    name: str = ""

    @property
    def key(self) -> str:
        """Return the declaration identity used by explicit bindings."""
        return self.name or self.role

    def __post_init__(self) -> None:
        if not self.role:
            raise PortError("a requirement needs a non-empty role")
        if self.count < 1:
            raise PortError(f"requirement count must be >= 1, got {self.count}")
        if self.kind is not None and self.kind not in {"geometry", "channel", "road", "spin"}:
            raise PortError(f"unknown requirement kind {self.kind!r}")

    def accepts(self, port: PortSpec) -> bool:
        """Return whether ``port`` satisfies this requirement's role and tags."""
        if port.role != self.role:
            return False
        if self.kind is not None and getattr(port, "kind", None) != self.kind:
            return False
        if self.units and (not isinstance(port, (ChannelPort, SpinPort)) or port.units != self.units):
            return False
        if self.family and (not isinstance(port, GeometryPort) or port.family != self.family):
            return False
        if not self.requires_capabilities <= port.capabilities:
            return False
        if self.match_labels and port.labels != self.match_labels:
            return False
        return port.cardinality != "none"


def link_counts_agree(port: PortSpec, requirement: PortRequirement) -> bool:
    """Return whether a port's cardinality can serve a requirement's count."""
    if port.cardinality == "none":
        return False
    if port.cardinality == "one":
        return requirement.count == 1
    return True
