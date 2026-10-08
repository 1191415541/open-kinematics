"""
A model fragment: the entities one instance contributes, before they are joined.

The point of this type is that a declaration and its builder produce *the same
thing*.  A template that fills in a dictionary of JSON produces a fragment; a
template whose builder walks a loop and creates a variable number of arms also
produces a fragment.  Downstream code then sees one shape, so adding a topology
never means adding a branch to the central assembly code.

Everything here is immutable data.  A fragment does not know how to solve, does
not evaluate a residual and does not reach for a template: those belong to the
kernel and to the compiler.  It carries bodies, their local points, joints,
drives, force elements, ports and requirements, plus the *provenance* of each --
which template, which revision, which properties produced it -- because
``A5`` has to trace an entity back to its source, and a flat merged model cannot
do that after the fact.

Units are SI throughout.  The millimetre inputs the schema accepts are converted
once at the boundary, before a fragment exists; nothing downstream converts.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, replace
from typing import Any, Literal

import numpy as np

from .identity import EntityId
from .ports import PortRequirement, PortSpec

__all__ = [
    "EntityConflictError",
    "FragmentProvenance",
    "ModelFragment",
    "Connection",
    "ResolvedElement",
]


class EntityConflictError(ValueError):
    """Two contributions claim the same entity identity."""


@dataclass(frozen=True)
class Connection:
    """Stable physical connection identifier."""

    name: str
    kind: Literal["ideal", "bushing"]
    body_a: str
    body_b: str
    point_a: str
    point_b: str


@dataclass(frozen=True)
class ResolvedElement:
    """A force element declaration with resolved attachment geometry."""

    kind: str
    name: str
    spec: object
    body_a: str | None = None
    point_a: np.ndarray | None = None
    body_b: str | None = None
    point_b: np.ndarray | None = None
    local_pose_a: object | None = None
    local_pose_b: object | None = None


@dataclass(frozen=True)
class FragmentProvenance:
    """
    Where a fragment's entities came from.

    Recorded per fragment rather than per entity: a fragment is the output of one
    build, so every entity in it shares the template, revision and property
    fingerprint that produced it.  ``A5`` traces an entity to its origin by
    walking the fragments that contributed to an assembly, which is why this is
    kept instead of being flattened away.
    """

    #: Template that produced the fragment, e.g. ``front_double_wishbone``.
    template: str
    #: Template revision, so a changed build is visible in a fingerprint.
    revision: str = ""
    #: The instance path this fragment was built under.
    instance: tuple[str, ...] = ()
    #: Fingerprint of the resolved properties/parameters that drove the build.
    properties_fingerprint: str = ""
    #: Free-form note for synthetic fixtures: they must say so.
    note: str = ""


@dataclass(frozen=True)
class ModelFragment:
    """
    The immutable entities one instance contributes to an assembly.

    Fields are keyed by local name within the fragment; ``qualified`` turns them
    into globally stable :class:`EntityId` values once the fragment knows its
    instance path.  Keeping local names here and qualifying at the boundary means
    a builder never has to know where it will be mounted -- which is what lets the
    same template instance be used twice in one model.
    """

    #: Rigid bodies by local name. SI: metres, kilograms.
    bodies: Mapping[str, Any] = field(default_factory=dict)
    #: Body-local points by ``(body, label)``. SI: metres.
    points: Mapping[tuple[str, str], Any] = field(default_factory=dict)
    #: Joint declarations, keyed by local name.
    joints: Mapping[str, Any] = field(default_factory=dict)
    #: The ideal-joint column, keyed by local name.
    #:
    #: Separate from ``joints`` because a C-mode assembly carries both columns:
    #: ``joints`` is the active one and this is the ideal set a K reading of the
    #: same model would use.  Keeping only the active column made a composition
    #: unable to answer "what would this model's K joints be", which is what a
    #: study switch asks.
    ideal_constraints: Mapping[str, Any] = field(default_factory=dict)
    #: The compliance column, keyed by local name.
    bushings: Mapping[str, Any] = field(default_factory=dict)
    #: Coordinate drives, keyed by local name.
    drives: Mapping[str, Any] = field(default_factory=dict)
    #: Force elements (springs, bushings, bars), keyed by local name.
    forces: Mapping[str, Any] = field(default_factory=dict)
    #: Tire references, keyed by local name. The tire owns its own mass.
    tires: Mapping[str, Any] = field(default_factory=dict)
    #: Connection rows, keyed by local name.  Accounting only: no document reads
    #: them, but an assembly's connection list is part of what it produces.
    connections: Mapping[str, Any] = field(default_factory=dict)
    #: Minimum output declarations, keyed by local name.
    outputs: Mapping[str, Any] = field(default_factory=dict)
    #: Ports this fragment offers, keyed by local name.
    ports: Mapping[str, PortSpec] = field(default_factory=dict)
    #: What this fragment needs from its neighbours.
    requirements: tuple[PortRequirement, ...] = ()
    #: Where this fragment came from.
    provenance: FragmentProvenance | None = None
    #: Instance path this fragment has been mounted at, if any.  Empty means the
    #: fragment is still detached and its ids are local.
    instance: tuple[str, ...] = ()

    # -- conflict detection -------------------------------------------------

    def __post_init__(self) -> None:
        for name, mapping in (
            ("bodies", self.bodies),
            ("points", self.points),
            ("joints", self.joints),
            ("ideal_constraints", self.ideal_constraints),
            ("bushings", self.bushings),
            ("drives", self.drives),
            ("forces", self.forces),
            ("tires", self.tires),
            ("connections", self.connections),
            ("outputs", self.outputs),
            ("ports", self.ports),
        ):
            keys = list(mapping)
            if len(keys) != len(set(keys)):
                duplicates = sorted({str(k) for k in keys if keys.count(k) > 1})
                raise EntityConflictError(
                    f"fragment declares {name} more than once: {duplicates}"
                )
        # A point's owning body is deliberately *not* required to be in this
        # fragment.  A fragment is partial by definition -- a suspension
        # contributes points on the chassis and on the tie rods, which other
        # contributions own -- so "every point resolves" is an assembly-level
        # property, checked once the contributions are merged.  Checking it here
        # would make a legal cross-subsystem attachment look like a dangling
        # reference.  `unresolved_point_bodies` reports what is still open so the
        # assembly can close it.
        # A port's owner is relaxed for the same reason as a point's: a
        # contribution may attach a port to a body another contribution owns (a
        # bench's wheel-centre port hangs on the carrier, but a rig may attach to
        # a body of the assembly).  Only the *local* name is recorded here, so
        # this level cannot decide the question; `unresolved_port_owners` reports
        # it and the assembly closes it.

    # -- completeness (closed by the owning assembly) -----------------------

    def unresolved_point_bodies(self) -> tuple[str, ...]:
        """
        Return the point-owning body names this fragment does not declare.

        Empty for a self-contained fragment.  Non-empty for a *partial* one --
        which is the normal case during composition, and the reason this is a
        query rather than an error.
        """
        return tuple(sorted({body for body, _ in self.points} - set(self.bodies)))

    def unresolved_port_owners(self) -> tuple[str, ...]:
        """Return the port-owner names this fragment does not declare as bodies."""
        return tuple(
            sorted(
                {
                    port.owner.local
                    for name, port in self.ports.items()
                    if port.owner.local not in self.bodies and port.owner.local != name
                }
            )
        )

    # -- identity -----------------------------------------------------------

    def mounted(self, at: tuple[str, ...]) -> ModelFragment:
        """Return this fragment as mounted at ``at``, ids qualified by the path."""
        return replace(self, instance=at)

    def entity(self, local: str) -> EntityId:
        """Identify an entity of this fragment, given its mounting path."""
        return EntityId(self.instance, local)

    def body_id(self, name: str) -> EntityId:
        """Identify a body of this fragment."""
        if name not in self.bodies:
            raise KeyError(f"unknown body {name!r} in fragment")
        return self.entity(name)

    # -- composition --------------------------------------------------------

    def merged_with(self, other: ModelFragment) -> ModelFragment:
        """
        Join two fragments, refusing any collision.

        Refusing rather than overwriting is the whole point: two subsystems
        claiming ``wheel`` is a modelling error, and the version that silently
        keeps the last write is the one that produces a model nobody can explain.
        Ports are the exception in spirit -- two fragments may both offer a port
        with the same local name only if the ids differ after qualification, and
        since qualified ids include the instance path they do.
        """
        if self.instance != other.instance:
            raise EntityConflictError(
                "cannot merge fragments mounted at different paths: "
                f"{self.instance} and {other.instance}"
            )
        return ModelFragment(
            bodies={**self.bodies, **self._unique("bodies", other.bodies)},
            points={**self.points, **self._unique("points", other.points)},
            joints={**self.joints, **self._unique("joints", other.joints)},
            ideal_constraints={
                **self.ideal_constraints,
                **self._unique("ideal_constraints", other.ideal_constraints),
            },
            bushings={**self.bushings, **self._unique("bushings", other.bushings)},
            drives={**self.drives, **self._unique("drives", other.drives)},
            forces={**self.forces, **self._unique("forces", other.forces)},
            tires={**self.tires, **self._unique("tires", other.tires)},
            connections={
                **self.connections,
                **self._unique("connections", other.connections),
            },
            outputs={**self.outputs, **self._unique("outputs", other.outputs)},
            ports={**self.ports, **self._unique("ports", other.ports)},
            requirements=(*self.requirements, *other.requirements),
            provenance=self.provenance,
            instance=self.instance,
        )

    def _unique(self, field_name: str, incoming: Mapping) -> Mapping:
        current = getattr(self, field_name)
        clashes = sorted(set(current) & set(incoming))
        if clashes:
            raise EntityConflictError(
                f"duplicate {field_name} between fragments: {clashes}"
            )
        return incoming

    # -- inspection ---------------------------------------------------------

    @property
    def entity_count(self) -> int:
        """Total number of declared entities, for a quick structural check."""
        return sum(
            len(mapping)
            for mapping in (
                self.bodies,
                self.points,
                self.joints,
                self.ideal_constraints,
                self.bushings,
                self.drives,
                self.forces,
                self.tires,
                self.connections,
                self.outputs,
                self.ports,
            )
        )

    def is_empty(self) -> bool:
        """Return whether this fragment declares nothing at all."""
        return self.entity_count == 0 and not self.requirements

    def local_names(self) -> Iterable[str]:
        """Every local name this fragment declares, across all kinds."""
        for mapping in (
            self.bodies,
            self.joints,
            self.ideal_constraints,
            self.bushings,
            self.drives,
            self.forces,
            self.tires,
            self.connections,
            self.outputs,
            self.ports,
        ):
            yield from mapping
