"""
Assemblies: a connected fragment, and that fragment joined to a rig.

Two levels, because they answer different questions:

* :class:`Assembly` -- the *device under test*.  Suspension, chassis, optional
  steering.  It has external ports, it may nest (an axle inside a vehicle), and
  it does **not** include wheels for a single-axle model: the rig supplies those.
* :class:`SimulationAssembly` -- the assembly with a rig attached and every
  connection resolved.  This is a complete physical model, so every required
  reference is satisfied and the global rules have passed.  It is still not a
  *run*: a study and a case are needed to execute it, because the assembly is a
  physical model and not a solver setting.

Both are immutable.  Rebuilding after a hardpoint change produces a new
SimulationAssembly rather than mutating one, which is what makes "the rig does
not change its coordinates when hardpoints move" checkable: the rig's own
definition is not touched, only the derived attachment poses move.

Nesting keeps its sources: an axle mounted inside a vehicle keeps the fragment
that produced it, so a later question -- which template made this body, and with
which properties -- is answerable without re-running the build.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
from typing import Any

from .identity import EntityId
from .instance import FragmentProvenance, ModelFragment
from .ports import PortRequirement, PortSpec

__all__ = [
    "Assembly",
    "AssemblyError",
    "NestedInstance",
    "SimulationAssembly",
]


class AssemblyError(ValueError):
    """An assembly is not usable as stated."""


@dataclass(frozen=True)
class NestedInstance:
    """
    A named child assembly inside a parent.

    The child keeps its own fragment and ports, with ids qualified by the
    instance name.  A vehicle's front axle is a nested instance; the parent does
    not flatten it away, because flattening is what loses the ability to say
    which template produced a body.
    """

    name: str
    assembly: Assembly

    def __post_init__(self) -> None:
        if not self.name:
            raise AssemblyError("a nested instance needs a name")

    @property
    def path(self) -> tuple[str, ...]:
        """The instance path this child occupies within its parent."""
        return (self.name,)


@dataclass(frozen=True)
class Assembly:
    """
    A connected fragment with external ports: the device under test.

    ``fragment`` holds the merged entities of this level only; children live in
    ``children`` and are *not* flattened into it.  That separation is what lets
    a rule be applied per root category -- a single-axle model forbids a brake
    subsystem at its own level while a vehicle requires one -- without the rule
    having to guess which nesting level it is looking at.
    """

    name: str
    fragment: ModelFragment
    #: Ports this assembly exposes to a rig, keyed by local name.
    ports: Mapping[str, PortSpec] = field(default_factory=dict)
    #: What this assembly needs from a rig.
    requirements: tuple[PortRequirement, ...] = ()
    #: Named child assemblies, unflattened.
    children: tuple[NestedInstance, ...] = ()
    #: Which of the six subsystem roles this level carries.
    subsystems: frozenset[str] = field(default_factory=frozenset)
    #: The category of the *root* assembly (``axle`` or ``vehicle``).  Lives here
    #: so a nested axle can be checked against the root's rules rather than its
    #: own, which is what stops a single-axle rule from being applied twice.
    root_kind: str = ""
    #: The physical build this level was composed from, when there is one.
    #:
    #: A composition carries identity and ports; a document needs masses, the
    #: built elements and the K/C column the mode selects.  Keeping the build
    #: here is what lets a compiler read one model without a second assembly
    #: path existing beside the composition -- the two would drift a line at a
    #: time and the drift would be invisible in every result.
    physical: Any = None
    #: Provenance of this level's own fragment.
    provenance: FragmentProvenance | None = None

    def __post_init__(self) -> None:
        if not self.name:
            raise AssemblyError("an assembly needs a name")
        names = [child.name for child in self.children]
        if len(names) != len(set(names)):
            raise AssemblyError(f"duplicate child instance names: {sorted(names)}")
        clashes = sorted(set(self.ports) & set(self.fragment.ports))
        if clashes:
            raise AssemblyError(
                "this level's ports and its fragment's ports collide: "
                f"{clashes}; an assembly port is declared in exactly one place"
            )

    # -- identity -----------------------------------------------------------

    def entity(self, local: str) -> EntityId:
        """Identify a top-level entity of this assembly."""
        return EntityId((self.name,), local)

    def all_ports(self) -> dict[str, PortSpec]:
        """Every port reachable from this assembly, keyed by qualified id."""
        found: dict[str, PortSpec] = {}
        for name, port in {**self.fragment.ports, **self.ports}.items():
            found[str(EntityId((self.name,), name))] = port
        for child in self.children:
            for key, port in child.assembly.all_ports().items():
                found[f"{self.name}/{key}"] = port
        return found

    def walk(self) -> Iterable[Assembly]:
        """Yield this assembly and every nested one, depth first."""
        yield self
        for child in self.children:
            yield from child.assembly.walk()


@dataclass(frozen=True)
class SimulationAssembly:
    """
    An assembly with a rig, fully connected and policy-checked.

    ``assembly`` and ``rig`` are both kept rather than merged into one fragment,
    because their roles stay distinct: the rig is not part of the vehicle, it
    loads the vehicle.  Connections between them are *generated* entities owned
    by this level, and their ids derive from the binding they came from, so the
    same pair always produces the same id.

    ``fingerprint`` is the structural identity A6 compares: the same
    SimulationAssembly run as a quasi-static and as a dynamic study must report
    the same fingerprint, because the study changes the solve and not the model.
    """

    name: str
    assembly: Assembly
    rig: Assembly
    #: Connections generated between the two, owned here.
    connections: tuple[Any, ...] = ()
    #: Generated connection and cross-instance entities, keyed by stable id.
    generated: Mapping[str, Any] = field(default_factory=dict)
    #: Structural fingerprint: identical for two studies of the same model.
    fingerprint: str = ""
    #: Recorded bindings, so a match can be audited rather than guessed at.
    bindings: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.name:
            raise AssemblyError("a simulation assembly needs a name")
        # This is where the completeness question is settled: a composition is a
        # *complete* model, so every point and every port must resolve against
        # the bodies the composition as a whole declares.  A single fragment
        # cannot answer that -- its references legitimately reach into other
        # contributions -- so it is asked here, once, after the merge.
        self._check_references_resolve()

    def _check_references_resolve(self) -> None:
        """
        Refuse a composition with a dangling point or port reference.

        Only the *merged* level is checked.  A child level is one contribution's
        partial view -- its points legitimately reach into bodies other
        contributions own -- so requiring it to be self-contained is exactly the
        mistake this check exists to avoid.  The merged fragment is where every
        reference has to land.
        """
        fragment = self.assembly.fragment
        dangling = [
            *(f"point on undeclared body {body!r}" for body in fragment.unresolved_point_bodies()),
            *(f"port owned by undeclared body {owner!r}" for owner in fragment.unresolved_port_owners()),
        ]
        if dangling:
            raise AssemblyError(
                "a composed simulation assembly must resolve every reference; "
                "unresolved: " + "; ".join(sorted(dangling))
            )

    @property
    def root_kind(self) -> str:
        """The category the global rules are applied to."""
        return self.assembly.root_kind

    def walk(self) -> Iterable[Assembly]:
        """Yield the device under test and the rig, with their nested parts."""
        yield from self.assembly.walk()
        yield from self.rig.walk()

    def entity_ids(self) -> set[EntityId]:
        """Every entity identity reachable from this simulation assembly."""
        found: set[EntityId] = set()
        for level in self.walk():
            path = (level.name,)
            found.update(
                EntityId(path, local) for local in level.fragment.local_names()
            )
            found.update(EntityId(path, local) for local in level.fragment.points)
        return found
