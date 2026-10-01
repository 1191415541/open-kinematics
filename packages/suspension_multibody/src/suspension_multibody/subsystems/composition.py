"""
One SI simulation assembly, composed from subsystems through their ports.

This is the step that turns "subsystems are composable" from a property of the
authoring functions into a property of a *value*.  Before it, six subsystems were
each a function that mutated a shared context in a fixed order, and the assembly
existed only as the side effect of running them in that order: build the chassis,
then the suspension (so the uprights exist), then steering (whose tie rods reach
those uprights).  Any new combination meant another ordered sequence, and getting
the order wrong produced a half-built model rather than an error.

Here the contributions are collected as fragments and *connected* by port, so:

* the order the subsystems are asked for their entities stops being load-bearing;
* a subsystem that cannot satisfy another's requirement fails by name, at
  composition time, instead of producing a model with a body missing;
* the result is a
  :class:`~suspension_multibody.modeling.assembly.SimulationAssembly` that can be
  inspected, fingerprinted and compared -- which is what lets ``06`` check the new
  path against the old one item by item rather than by eyeballing two results.

The composition is deliberately *additive*: it does not reimplement any subsystem.
Each subsystem's existing contribution function is called and its output converted
into a fragment, so the new path cannot drift from the old one without one of them
being edited.
"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
from typing import TYPE_CHECKING, Any

from ..modeling.assembly import Assembly, NestedInstance, SimulationAssembly
from ..modeling.identity import EntityId
from ..modeling.instance import FragmentProvenance, ModelFragment
from ..modeling.ports import GeometryPort, PortRequirement
from .capabilities import AssemblyCapabilities, capabilities_for
from .types import SubsystemOutput

if TYPE_CHECKING:
    from ..connections.links import LinkSpec

__all__ = [
    "SI_ASSEMBLY_NAME",
    "SUBSYSTEM_ROLES",
    "CompositionError",
    "SubsystemContribution",
    "compose_simulation_assembly",
    "fingerprint_assembly",
]

#: The name the composed single-axle SI assembly carries.
SI_ASSEMBLY_NAME = "axle"

#: The subsystem roles, in the order the *recorded* body list has always used.
#:
#: Order still matters for the output document -- the contract lists bodies in a
#: sequence -- but it no longer decides whether the build succeeds: a subsystem
#: may now be asked for its entities in any order, because requirements are
#: resolved by port after all contributions are in hand.
#:
#: `anti_roll_bar` is appended rather than slotted in, so the recorded order of
#: the six roles that were already here is unchanged.
SUBSYSTEM_ROLES: tuple[str, ...] = (
    "chassis",
    "suspension",
    "steering",
    "wheel",
    "brake",
    "drive",
    "anti_roll_bar",
)


class CompositionError(ValueError):
    """The contributions cannot be composed into one assembly."""


@dataclass(frozen=True)
class SubsystemContribution:
    """
    One subsystem's entities, with the identity and ports it contributes them under.

    ``output`` is the existing :class:`~suspension_multibody.subsystems.types.SubsystemOutput`
    verbatim: converting it here rather than rebuilding the subsystem logic is
    what keeps the composed path comparable to the historical one.
    """

    role: str
    output: SubsystemOutput
    #: Ports this contribution offers, keyed by local name.
    ports: Mapping[str, GeometryPort]
    #: What this contribution needs from its neighbours.
    needs: tuple[PortRequirement, ...] = ()
    #: Which sides this contribution covers, for per-side port qualification.
    sides: tuple[str, ...] = ()
    #: How this contribution's matched ports are joined, if it has an opinion.
    #:
    #: Empty means "record the binding and build nothing", which is where every
    #: existing caller stands: the composition stays a record of *which* port met
    #: *which* requirement until somebody states how the two are joined.  See
    #: :mod:`suspension_multibody.connections.links`.
    links: tuple["LinkSpec", ...] = ()
    #: Free-form note recorded in the provenance.
    note: str = ""
    def __post_init__(self) -> None:
        if self.role not in SUBSYSTEM_ROLES:
            raise CompositionError(
                f"unknown subsystem role {self.role!r}; known roles are "
                f"{', '.join(SUBSYSTEM_ROLES)}"
            )


def _fragment_from_output(
    contribution: SubsystemContribution, *, instance: tuple[str, ...]
) -> ModelFragment:
    """
    Convert one subsystem's output into a fragment.

    The mapping is intentionally mechanical -- bodies to bodies, points to points,
    constraints to joints, element rows to forces -- because anything clever here
    would be a second implementation of the assembly, which is exactly what the
    architecture is trying to stop having.

    Both constraint columns travel: ``joints`` carries the active column and
    ``ideal_constraints`` the ideal one, so a C-mode composition can still answer
    what its K joints would be.  Bushings, tire rows and connection rows travel
    too, because a contract emitter reads all three and a fragment that dropped
    them forced the composition to keep the historical build alive beside it.
    """
    joints: dict[str, object] = {}
    ideal: dict[str, object] = {}
    for constraint in contribution.output.constraints:
        name = getattr(constraint, "name", None) or type(constraint).__name__
        joints[str(name)] = {
            "kind": type(constraint).__name__,
            "constraint": constraint,
        }
    for constraint in contribution.output.ideal_constraints:
        name = getattr(constraint, "name", None) or type(constraint).__name__
        ideal[str(name)] = {
            "kind": type(constraint).__name__,
            "constraint": constraint,
        }

    forces: dict[str, object] = {}
    for row in contribution.output.elements:
        forces[str(row.name)] = {"kind": row.kind, "row": row}

    bushings: dict[str, object] = {}
    for row in contribution.output.bushings:
        bushings[str(row.name)] = {"kind": row.kind, "row": row}

    tires: dict[str, object] = {}
    for name, row in forces.items():
        if row.get("kind") == "tire":
            tires[name] = row

    connections: dict[str, object] = {}
    for connection in contribution.output.connections:
        connections[str(connection.name)] = connection

    bodies = {name: body for name, body in contribution.output.bodies.items()}
    points = {
        (body, label): value for (body, label), value in contribution.output.points.items()
    }
    # Ports are declared on the *assembly* level, not inside the fragment: the
    # fragment describes entities, the assembly describes what a neighbour can
    # attach to.  Declaring them in both places is the collision the Assembly
    # constructor refuses, and refusing it is right -- two sources for "which
    # ports does this offer" is how the two drift apart.
    return ModelFragment(
        bodies=bodies,
        points=points,
        joints=joints,
        ideal_constraints=ideal,
        bushings=bushings,
        forces=forces,
        tires=tires,
        connections=connections,
        requirements=tuple(contribution.needs),
        provenance=FragmentProvenance(
            template=f"subsystem:{contribution.role}",
            revision=contribution.role,
            instance=instance,
            note=contribution.note,
        ),
    ).mounted(instance)


def _port_id(instance: tuple[str, ...], local: str) -> str:
    """Render a port id the way the entity layer does, for matching."""
    return str(EntityId(instance, local))


def compose_simulation_assembly(
    contributions: Sequence[SubsystemContribution],
    *,
    capabilities: AssemblyCapabilities | None = None,
    name: str = SI_ASSEMBLY_NAME,
    root_kind: str | None = None,
    pairings: Mapping[str, str] | None = None,
    explicit_bindings: Mapping[str, str] | None = None,
    body_order: Sequence[str] | None = None,
    physical: Any = None,
    rig: Assembly | None = None,
    rig_name: str = "none",
) -> SimulationAssembly:
    """
    Join subsystem contributions into one SI simulation assembly.

    Requirements are resolved after every contribution is in hand, so the order
    the callers list them in decides nothing but the recorded body sequence.  A
    requirement with no candidate raises unless it was declared optional, in which
    case it disappears together with the outputs bound to it -- the same rule the
    interface matcher applies, applied once here rather than re-invented.

    ``capabilities`` is what the caller knows about the whole assembly (which
    roles it carries).  It is optional because the composition itself does not
    need it: the ports carry the facts.  It is *used* when supplied, so a caller
    that has already derived capabilities does not derive them twice.

    ``rig`` is the bench the assembly is run on.  A run is one assembly plus one
    rig, and the two are joined here rather than by the caller, because the
    *pair* is what a simulation assembly is: without a bench the assembled model
    would be a device under test with nothing driving it.  The bench is kept as
    its own level -- it is not merged into the device's fragment -- because the
    rig is not part of the model it loads; a merged fragment could no longer say
    which entities came from the bench, and the two roles would stop being
    separable.
    """
    from ..connections.links import build_links, explicit_bindings_from_pairings
    from ..connections.matcher import match_requirements

    if not contributions:
        raise CompositionError("a simulation assembly needs at least one contribution")

    roles = [contribution.role for contribution in contributions]
    duplicates = sorted({role for role in roles if roles.count(role) > 1})
    if duplicates:
        raise CompositionError(
            "a subsystem role is contributed twice: "
            + ", ".join(duplicates)
            + "; one role is one contribution, so the assembly knows where an "
            "entity came from"
        )

    instance = (name,)
    fragments: list[ModelFragment] = []
    child_assemblies: list[Assembly] = []
    all_ports: dict[str, GeometryPort] = {}
    all_requirements = []

    for contribution in contributions:
        fragment = _fragment_from_output(contribution, instance=instance)
        fragments.append(fragment)
        for local, port in contribution.ports.items():
            all_ports[_port_id(instance, local)] = port
        all_requirements.extend(contribution.needs)
        child_assemblies.append(
            Assembly(
                name=contribution.role,
                fragment=fragment,
                ports=dict(contribution.ports),
                requirements=tuple(contribution.needs),
                subsystems=frozenset({contribution.role}),
                provenance=fragment.provenance,
            )
        )

    # ``pairings`` is the document's spelling -- a requirement role and a port
    # *name* -- and ``explicit_bindings`` the code's, a role and a port id.  Both
    # end in one mapping, so there is still exactly one matching channel, and a
    # pairing stated for a role overrides a binding stated for that same role.
    explicit = dict(explicit_bindings or {})
    if pairings:
        explicit.update(
            explicit_bindings_from_pairings(pairings, all_ports, instance=instance)
        )
    report = match_requirements(all_requirements, all_ports, explicit=explicit)

    # The bindings become entities here.  The match says which two ports meet and
    # the contributions say what that meeting *is*; a composition whose
    # contributions state no link produces no rows, so it is what it was.
    link_rows = build_links(
        report.bindings,
        all_ports,
        [spec for contribution in contributions for spec in contribution.links],
        instance=instance,
    )

    merged = ModelFragment(instance=instance)
    for fragment in fragments:
        merged = merged.merged_with(fragment)
    if body_order is not None:
        known = set(merged.bodies)
        unknown = [item for item in body_order if item not in known]
        if unknown:
            raise CompositionError(
                "body_order names bodies no contribution produced: "
                + ", ".join(sorted(unknown))
            )
        ordered = {item: merged.bodies[item] for item in body_order}
        for item, body in merged.bodies.items():
            ordered.setdefault(item, body)
        merged = replace(merged, bodies=ordered)
    if link_rows.rows:
        # Merged after the body order is applied: a link adds no body, and the
        # bodies the caller already ordered keep their places.
        merged = merged.merged_with(link_rows.fragment)

    resolved_capabilities = capabilities
    if resolved_capabilities is None:
        resolved_capabilities = capabilities_for(
            subsystems=frozenset(roles) & set(SUBSYSTEM_ROLES),
            body_names=frozenset(merged.bodies),
        )

    assembly = Assembly(
        name=name,
        fragment=merged,
        ports=dict(all_ports),
        requirements=tuple(all_requirements),
        children=tuple(
            NestedInstance(role, child) for role, child in zip(roles, child_assemblies)
        ),
        subsystems=frozenset(contribution.role for contribution in contributions),
        root_kind=root_kind if root_kind is not None else "",
        physical=physical,
        provenance=FragmentProvenance(
            template="simulation_assembly",
            revision="-".join(roles),
            instance=instance,
        ),
    )
    bindings = {
        binding.requirement.role: ",".join(binding.port_ids)
        for binding in report.bindings
    }
    generated: dict[str, Any] = {
        "disappeared": report.disappeared,
        "dropped_outputs": report.dropped_outputs,
        # The generated rows travel with the assembly, so a caller can audit what
        # the pairings *built* rather than only which ports they named.
        "links": link_rows.rows,
    }
    if rig is not None:
        # The bench's own entities are kept on their own level, and the
        # cross-level connections are generated here, exactly like a nested
        # instance: the pair is the simulation assembly, and a caller that asks
        # for a body must be able to tell which side it came from.
        bindings = {**bindings, **_bind_rig(assembly, rig)}
    return SimulationAssembly(
        name=name,
        assembly=assembly,
        rig=rig if rig is not None else Assembly(name="none", fragment=ModelFragment()),
        generated=generated,
        # No capabilities: the fingerprint is a property of the *model*, and a
        # caller asking for it later -- with or without a capability report --
        # must get the same value.  Passing them here made the same assembly
        # answer differently depending on who asked.
        fingerprint=fingerprint_assembly(assembly),
        bindings=bindings,
    )


def _bind_rig(assembly: Assembly, rig: Assembly) -> dict[str, str]:
    """
    Resolve the device's requirements against the bench's ports.

    The device reports what it needs; the bench reports what it offers.  Matching
    them is a real decision that can fail, so it goes through the same matcher the
    subsystem level uses rather than a name comparison -- a bench bound by name
    proximity is what produced rigs attached to whatever happened to be there.

    Only *required* requirements raise: a bench that cannot reach an optional
    branch contributes no measurement for it, which is the same rule the ports
    already encode.
    """
    from ..connections.matcher import match_requirements

    if not assembly.requirements:
        return {}
    offered = {
        str(EntityId((rig.name,), local)): port
        for local, port in rig.ports.items()
    }
    report = match_requirements(assembly.requirements, offered)
    return {
        binding.requirement.role: ",".join(binding.port_ids)
        for binding in report.bindings
    }


def fingerprint_assembly(
    assembly: Assembly, *, capabilities: AssemblyCapabilities | None = None
) -> str:
    """
    Fingerprint an assembly's *structure*, so two studies can be compared.

    Structural, not numerical: the point of the fingerprint is that the same
    assembly read by a quasi-static and by a dynamic study reports the same value
    because the study changes the solve and not the model.  Coordinates are
    deliberately excluded -- a study may legitimately evaluate the model at
    different states, and folding those in would make the fingerprint a result
    hash rather than an identity.
    """
    parts: list[str] = [assembly.name, assembly.root_kind]
    # Sorted by level name so the fingerprint does not depend on the order the
    # contributions happened to be collected in: two compositions of the same
    # model must agree even if the caller listed the subsystems differently.
    for level in sorted(assembly.walk(), key=lambda item: item.name):
        fragment = level.fragment
        parts.append(f"level:{level.name}")
        parts.append("bodies:" + ",".join(sorted(fragment.bodies)))
        parts.append("joints:" + ",".join(sorted(fragment.joints)))
        parts.append("forces:" + ",".join(sorted(fragment.forces)))
        parts.append("tires:" + ",".join(sorted(fragment.tires)))
        parts.append("ports:" + ",".join(sorted(fragment.ports)))
        # Geometry *is* part of the identity: two models whose hardpoints differ
        # are different models, and a fingerprint that ignored the coordinates
        # would report them as the same one.  Points are rounded to a
        # representable precision so that the same model built twice -- where
        # floating-point arithmetic could differ in the last bit -- still agrees.
        parts.append(
            "points:"
            + ",".join(
                f"{body}.{label}="
                + " ".join(f"{float(value):.12g}" for value in coordinates)
                for (body, label), coordinates in sorted(fragment.points.items())
            )
        )
        parts.append("mass:" + ",".join(
            f"{name}={_body_mass(body)!r}" for name, body in sorted(fragment.bodies.items())
        ))
    if capabilities is not None:
        parts.append("roles:" + ",".join(sorted(capabilities.subsystems)))
        parts.append("coords:" + ",".join(sorted(capabilities.drive_coordinates)))
    return hashlib.sha256("|".join(parts).encode("utf-8")).hexdigest()[:16]


def _body_mass(body: object) -> object:
    """
    Return a body's mass, or ``None`` for a body that carries none.

    Bodies reach the fragment as the subsystem's own objects during composition
    and as the modeling layer's values afterwards, so this reads whichever
    attribute is present rather than assuming one shape.  A missing mass is
    reported as ``None`` rather than zero: "no mass declared" and "massless" are
    different claims about a model.
    """
    return getattr(body, "mass", None)
