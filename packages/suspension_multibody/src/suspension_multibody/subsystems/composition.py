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
from typing import Any

from ..modeling.assembly import Assembly, NestedInstance, SimulationAssembly
from ..modeling.identity import EntityId
from ..modeling.instance import FragmentProvenance, ModelFragment
from ..modeling.ports import GeometryPort, PortRequirement
from .capabilities import AssemblyCapabilities, capabilities_for
from .types import SubsystemOutput

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

#: The six roles, in the order the *recorded* body list has always used.
#:
#: Order still matters for the output document -- the contract lists bodies in a
#: sequence -- but it no longer decides whether the build succeeds: a subsystem
#: may now be asked for its entities in any order, because requirements are
#: resolved by port after all contributions are in hand.
SUBSYSTEM_ROLES: tuple[str, ...] = (
    "chassis",
    "suspension",
    "steering",
    "wheel",
    "brake",
    "drive",
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
    """
    joints: dict[str, object] = {}
    for constraint in contribution.output.constraints:
        name = getattr(constraint, "name", None) or type(constraint).__name__
        joints[str(name)] = {
            "kind": type(constraint).__name__,
            "constraint": constraint,
        }

    forces: dict[str, object] = {}
    for row in contribution.output.elements:
        forces[str(row.name)] = {"kind": row.kind, "row": row}

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
        forces=forces,
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
    explicit_bindings: Mapping[str, str] | None = None,
    body_order: Sequence[str] | None = None,
    physical: Any = None,
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
    """
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

    report = match_requirements(
        all_requirements, all_ports, explicit=dict(explicit_bindings or {})
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

    resolved_capabilities = capabilities
    if resolved_capabilities is None:
        resolved_capabilities = capabilities_for(
            subsystems=frozenset(roles) & {"suspension", "steering", "wheel", "chassis", "brake", "drive"},
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
    return SimulationAssembly(
        name=name,
        assembly=assembly,
        rig=Assembly(name="none", fragment=ModelFragment()),
        generated={"disappeared": report.disappeared, "dropped_outputs": report.dropped_outputs},
        # No capabilities: the fingerprint is a property of the *model*, and a
        # caller asking for it later -- with or without a capability report --
        # must get the same value.  Passing them here made the same assembly
        # answer differently depending on who asked.
        fingerprint=fingerprint_assembly(assembly),
        bindings=bindings,
    )


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
