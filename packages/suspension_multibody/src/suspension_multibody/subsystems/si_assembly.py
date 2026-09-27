"""
The SI assembly entry point: one composition, from the subsystems that exist.

``composition.py`` knows how to join contributions; this module is what produces
them from the axle the package already builds.  It exists as a separate step
because the *point* of subtask 06 is that the new path and the old one agree, and
that is only checkable if the new path consumes the old one's outputs rather than
re-deriving them.  Every contribution here is the verbatim output of the existing
subsystem function, wrapped with the ports and needs that describe it.

Two things this module deliberately does **not** do:

* it does not reorder or filter the subsystems' entities -- the recorded body
  list has a sequence the contract depends on, and the composition preserves it;
* it does not call the solver or emit a contract document.  ``06`` produces and
  verifies the assembly value; switching the production path over to it is ``07``,
  and claiming otherwise here would be the "half-migrated, two live paths"
  failure the epic warns about.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import replace
from typing import Literal

from ..connections.policy import check_root
from ..modeling.assembly import Assembly, SimulationAssembly
from ..modeling.identity import EntityId
from ..modeling.instance import FragmentProvenance, ModelFragment
from ..modeling.ports import GeometryPort, PortRequirement
from ..schema import FrontAxleModel
from . import chassis as chassis_subsystem
from . import steering as steering_subsystem
from . import suspension as suspension_subsystem
from . import wheel as wheel_subsystem
from .capabilities import AssemblyCapabilities
from .composition import (
    SI_ASSEMBLY_NAME,
    CompositionError,
    SubsystemContribution,
    compose_simulation_assembly,
    fingerprint_assembly,
)
from .element_build import element_rows
from .explicit import build_explicit_runtime
from .runtime import RuntimeOrder, SubsystemRuntime, runtime_from_outputs
from .types import AssemblyRequest, SubsystemContext, SubsystemOutput

__all__ = [
    "contributions_for_axle",
    "si_assembly_for_axle",
]

#: The sides a symmetric axle has, typed as the geometry helpers require.
_SIDES: tuple[Literal["L", "R"], Literal["L", "R"]] = ("L", "R")


def _port(instance: tuple[str, ...], local: str, role: str, side: str | None) -> GeometryPort:
    """Build one geometric port for a subsystem's own entity."""
    labels = frozenset({side}) if side else frozenset()
    return GeometryPort(
        id=EntityId(instance, local),
        owner=EntityId(instance, local),
        role=role,
        capabilities=frozenset({"geometry"}),
        labels=labels,
    )


def _ports_for_bodies(
    instance: tuple[str, ...], bodies: Mapping[str, object]
) -> dict[str, GeometryPort]:
    """
    Offer one port per body, named after the body.

    A body is what a neighbour can attach to, and the port's role says so.  This
    is what lets the composition resolve requirements by entity rather than by a
    hand-maintained list: the ports are derived from what the subsystem actually
    emitted, so a subsystem that adds a body offers the port for it automatically.

    A body that carries a wheel centre offers a second, *semantic* port for it.
    The two are different claims -- "here is a body you may attach to" and "here
    is the wheel centre you need" -- and a bench supplying wheels binds to the
    second.  Deriving it from the body's own points is what keeps the claim true:
    a body that stops carrying a wheel centre stops offering the port.
    """
    ports: dict[str, GeometryPort] = {}
    for name in bodies:
        side = name.rsplit("_", 1)[-1] if name.endswith(("_L", "_R")) else None
        ports[name] = _port(instance, name, "body", side)
        if name.startswith("upright_"):
            centre = f"wheel_centre_{side}"
            ports[centre] = GeometryPort(
                id=EntityId(instance, centre),
                owner=EntityId(instance, name),
                role="wheel_centre",
                capabilities=frozenset({"wheel", "load"}),
                labels=frozenset({side}) if side else frozenset(),
            )
    return ports


def _wheel_centre_needs(
    instance: tuple[str, ...], bodies: Mapping[str, object]
) -> tuple[PortRequirement, ...]:
    """
    Declare that this assembly needs a wheel centre per side it carries.

    The single-axle assembly builds no wheel body (D9): the bench supplies the
    wheel, so somebody has to name the attachment, and the assembly is the party
    that knows which sides it has.  The requirement is *optional* because an axle
    with no upright at all is a legal assembly -- it simply has no wheel to load,
    and a required port there would refuse a model that is not wrong.
    """
    needs: list[PortRequirement] = []
    for side in _SIDES:
        if f"upright_{side}" not in bodies:
            continue
        needs.append(
            PortRequirement(
                role="wheel_centre",
                count=1,
                required=False,
                match_labels=frozenset({side}),
                note=(
                    "the bench supplies the wheel on this side; a bench that "
                    "cannot reach it contributes no wheel load there"
                ),
            )
        )
    return tuple(needs)


def contributions_for_axle(
    model: FrontAxleModel,
    *,
    request: AssemblyRequest | None = None,
    name: str = SI_ASSEMBLY_NAME,
) -> tuple[SubsystemContribution, ...]:
    """Return the contributions the axle's subsystem set implies."""
    contributions, _ = axle_contributions_and_order(
        model, request=request, name=name
    )
    return contributions


def axle_contributions_and_order(
    model: FrontAxleModel,
    *,
    request: AssemblyRequest | None = None,
    name: str = SI_ASSEMBLY_NAME,
) -> tuple[tuple[SubsystemContribution, ...], RuntimeOrder]:
    """
    Produce the contributions the axle's subsystem set implies, and its order.

    The set comes from ``request.subsystems``, so an assembly without steering
    simply has no steering contribution -- and, because the composition resolves
    requirements afterwards, the tie rods and rack are *absent* rather than
    present-but-degenerate.  That is the same rule the historical build applies,
    reached by construction instead of by a conditional in an ordered sequence.

    The context is threaded exactly as the composition entry threads it, because
    the subsystems read shared state: the suspension's uprights have to exist
    before steering can resolve its tie rod points against them.  What changes is
    that this ordering is now an implementation detail of *producing* the
    contributions, and no longer decides whether the assembly is correct.

    The returned :class:`~.runtime.RuntimeOrder` is the sequence the **document**
    records, which is not the order the contributions are listed in: the recorded
    rows interleave the two subsystems by side (left suspension, left steering,
    right suspension, right steering) while the contributions are grouped by role.
    Stating it here, where both subsystems' per-side content is in hand, is what
    keeps the document's order from depending on how the contributions happened
    to be filed.
    """
    from .geometry import side_hardpoints

    if request is None:
        request = AssemblyRequest(mode="K")
    mode = request.mode
    if mode not in ("K", "C"):
        raise CompositionError(f"unknown mode {mode!r}; modes are K and C")

    context = SubsystemContext(
        model=model,
        request=request,
        hardpoints=dict(model.hardpoints),
        side_schema={side: side_hardpoints(model.hardpoints, side) for side in _SIDES},
    )
    instance = (name,)
    contributions: list[SubsystemContribution] = []

    chassis_output = chassis_subsystem.build(context)
    context.bodies.update(chassis_output.bodies)
    contributions.append(
        SubsystemContribution(
            role="chassis",
            output=chassis_output,
            ports=_ports_for_bodies(instance, chassis_output.bodies),
            note="fixed body and its connection",
        )
    )

    side_bodies: dict[str, dict[str, object]] = {}
    for side in _SIDES:
        side_bodies[side] = suspension_subsystem.side_bodies(context, side)
        context.bodies.update(side_bodies[side])

    steering_bodies = (
        steering_subsystem.bodies(context) if request.carries("steering") else {}
    )

    # The recorded order: chassis, rack, then per side arm/arm/upright/tie rod.
    suspension_outputs: list[SubsystemOutput] = []
    for side in _SIDES:
        for key, point in side_hardpoints(model.hardpoints, side).items():
            values = point.as_array()
            context.hardpoints[f"{key}__{side}"] = type(point)(
                x=values[0], y=values[1], z=values[2]
            )
        suspension_outputs.append(suspension_subsystem.side_content(context, side))
        context.points.update(suspension_outputs[-1].points)

    suspension_bodies: dict[str, object] = {}
    for side in _SIDES:
        suspension_bodies.update(side_bodies[side])

    # The two subsystems are contributed *separately* so that every entity keeps
    # its provenance -- which subsystem produced it is a question a caller asks.
    # The recorded order interleaves them by side, and that order is applied
    # afterwards by `compose_simulation_assembly`'s sequence arguments, because
    # reproducing it here would mean filing steering's rows under suspension.
    contributions.append(
        SubsystemContribution(
            role="suspension",
            output=SubsystemOutput(
                bodies=suspension_bodies,
                points={
                    key: value
                    for output in suspension_outputs
                    for key, value in output.points.items()
                },
                connections=[
                    connection for output in suspension_outputs for connection in output.connections
                ],
                constraints=[
                    constraint for output in suspension_outputs for constraint in output.constraints
                ],
                ideal_constraints=[
                    constraint
                    for output in suspension_outputs
                    for constraint in output.ideal_constraints
                ],
                bushings=[
                    row for output in suspension_outputs for row in output.bushings
                ],
                # The elastic elements come from the subsystem hooks that have
                # always owned them (springs, dampers, bump stops, the anti-roll
                # bar, the tires), in their recorded order.  They are *not* part of
                # any subsystem output above -- those carry bodies, points,
                # constraints and compliance slots -- so a composition that stopped
                # at them would describe a model with no springs at all: a model
                # that solves and is wrong.
                #
                # The C-mode slot placeholders are passed as empty here because
                # `bushings` already carries them and the runtime appends that
                # column after the elements, which is exactly the recorded
                # sequence.  Passing them twice would duplicate four rows.
                elements=list(element_rows(model, mode, context, ())),
            ),
            ports=_ports_for_bodies(instance, suspension_bodies),
            needs=_wheel_centre_needs(instance, suspension_bodies),
            note=f"left/right suspension pair, mode {mode}",
        )
    )

    steering_outputs: list[SubsystemOutput] = []
    guide = SubsystemOutput()
    if steering_bodies:
        steering_outputs = [
            steering_subsystem.side_content(context, side) for side in _SIDES
        ]
        guide = steering_subsystem.guide(context)
        context.points.update(
            {key: value for output in steering_outputs for key, value in output.points.items()}
        )
        context.points.update(guide.points)
        context.hardpoints.update(guide.hardpoints)  # type: ignore[arg-type]
        contributions.append(
            SubsystemContribution(
                role="steering",
                output=SubsystemOutput(
                    bodies=steering_bodies,
                    points={
                        key: value
                        for output in (*steering_outputs, guide)
                        for key, value in output.points.items()
                    },
                    connections=[
                        connection
                        for output in (*steering_outputs, guide)
                        for connection in output.connections
                    ],
                    constraints=[
                        constraint
                        for output in (*steering_outputs, guide)
                        for constraint in output.constraints
                    ],
                    ideal_constraints=[
                        constraint
                        for output in (*steering_outputs, guide)
                        for constraint in output.ideal_constraints
                    ],
                    bushings=[row for output in (*steering_outputs, guide) for row in output.bushings],
                    elements=[row for output in (*steering_outputs, guide) for row in output.elements],
                ),
                ports=_ports_for_bodies(instance, steering_bodies),
                note="rack and tie rods; absent when the assembly carries no steering",
            )
        )

    if request.carries("wheel"):
        wheel_output = wheel_subsystem.build(context)
        contributions.append(
            SubsystemContribution(
                role="wheel",
                output=wheel_output,
                ports=_ports_for_bodies(instance, wheel_output.bodies),
                note="wheel and tire references",
            )
        )

    # The recorded sequence: per side, the suspension rows and then that side's
    # steering rows, with the rack guide last.  Derived from the per-side content
    # rather than written out, so a change to what a subsystem emits on a side is
    # reflected in the document order instead of being silently dropped from it.
    recorded_constraints: list[str] = [
        getattr(constraint, "name", "")
        for constraint in chassis_output.constraints
    ]
    recorded_ideal: list[str] = [
        getattr(constraint, "name", "") for constraint in chassis_output.ideal_constraints
    ]
    recorded_connections: list[str] = [
        connection.name for connection in chassis_output.connections
    ]
    recorded_points: list[str] = []
    for index, side in enumerate(_SIDES):
        suspension_output = suspension_outputs[index]
        steering_output = steering_outputs[index] if steering_outputs else None
        for part in (suspension_output, steering_output):
            if part is None:
                continue
            recorded_constraints.extend(
                getattr(constraint, "name", "") for constraint in part.constraints
            )
            recorded_ideal.extend(
                getattr(constraint, "name", "") for constraint in part.ideal_constraints
            )
            recorded_connections.extend(
                connection.name for connection in part.connections
            )
            recorded_points.extend(f"{body}.{label}" for body, label in part.points)
    if steering_bodies:
        recorded_constraints.extend(
            getattr(constraint, "name", "") for constraint in guide.constraints
        )
        recorded_ideal.extend(
            getattr(constraint, "name", "") for constraint in guide.ideal_constraints
        )
        recorded_connections.extend(
            connection.name for connection in guide.connections
        )
    order = RuntimeOrder(
        bodies=(),
        constraints=tuple(recorded_constraints),
        ideal_constraints=tuple(recorded_ideal),
        connections=tuple(recorded_connections),
    )
    # The hardpoint table travels on the chassis contribution, and it is attached
    # *here* rather than where that contribution was built: `context.hardpoints` is
    # still growing at that point (the per-side mirrors are added while the
    # suspension outputs are produced, and steering adds its own generated
    # entries), so attaching it early would publish a partial table.
    #
    # It matters that it travels at all: every subsystem's geometry is *named* in
    # this table, so a composition that dropped it would describe an assembly whose
    # points have no definitions -- which is what made the frozen-snapshot test
    # fail with an empty `hardpoints` before this.
    contributions[0] = replace(
        contributions[0],
        output=replace(
            contributions[0].output, hardpoints=dict(context.hardpoints)
        ),
    )
    return tuple(contributions), order


def si_assembly_for_axle(
    model: FrontAxleModel,
    *,
    request: AssemblyRequest | None = None,
    name: str = SI_ASSEMBLY_NAME,
    rig: str | None = None,
):
    """
    Compose the SI simulation assembly for an axle, from its subsystems.

    Returns a :class:`~suspension_multibody.modeling.assembly.SimulationAssembly`
    carrying a structural fingerprint, so the same model read by two studies can
    be shown to be one model.

    The composition carries the **runtime face** it amounts to, not the
    historical assembly: a document is authored from the runtime, so requiring an
    assembly here would make the composition a shell around the path it is meant
    to replace.

    An explicit model takes the other topology: it states its own parts and
    joints, so there is nothing to compose from subsystem roles and the runtime is
    built directly.  Both routes return the same value shape, which is what lets
    a caller hold one object without knowing which topology produced it.

    ``rig`` names the bench to join, when the caller wants a *simulation*
    assembly rather than a bare device under test.  It is optional because
    composing and running are separate questions: a caller that only wants the
    model must not be forced to name a bench for it.
    """
    resolved = request if request is not None else AssemblyRequest(mode="K")
    if model.topology == "explicit":
        return _explicit_simulation_assembly(model, resolved, name=name, rig=rig)

    # The global rules are applied here, once, to the root category -- not spread
    # across the builders that happen to know that an axle has no brake.  This is
    # the only place a composition learns whether it is the thing it claims to be.
    check_root("axle", resolved.subsystems)
    contributions, recorded = axle_contributions_and_order(
        model, request=resolved, name=name
    )
    # The document's body sequence comes from the template's own part declaration,
    # with steering's bodies where the template puts them.  It used to be a list
    # written here (`chassis, rack, upper_arm, lower_arm, upright, tie_rod` per
    # side), which made the order a fact about *this module*: a template declaring
    # its parts differently would have been silently reordered to match the list, so
    # "choose a template" could not change the model's entities.
    #
    # The built-in template declares exactly that sequence, so deriving it leaves
    # every existing document unchanged -- asserted in the test suite, because that
    # equality is what makes this a refactor rather than a behaviour change.
    body_order: list[str] = list(suspension_subsystem.side_body_order(resolved))
    runtime = runtime_from_outputs(
        (contribution.output for contribution in contributions),
        mode=resolved.mode,
        roles=frozenset(contribution.role for contribution in contributions),
        order=replace(recorded, bodies=tuple(body_order)),
        # The model's ground travels with the runtime because the document is authored
        # from the runtime alone: the model itself is consumed while composing, so a
        # surface the model declared would otherwise be lost before the document is
        # written.  `None` is the ordinary case -- most readings drive the wheel centre
        # and need no ground.
        road=model.road,
    )
    reordered = _reorder_bodies(runtime, body_order)
    bench = _rig_assembly(rig, mode=resolved.mode, capabilities=reordered.capabilities)
    if bench is not None:
        # D1: the bench's wheel joins the model it loads.  The attachment is built
        # here, where both the assembly and the bench are in hand, because it needs
        # the assembly's points to find the wheel centre's body.
        #
        # The carrier is free and welded to that body, which reproduces the
        # historical structure exactly: two more bodies, two more six-row welds, and
        # the same unconstrained directions the model already had (measured
        # constraint rows against columns: 64 over 66 in K, 42 over 66 in C, no
        # redundancy in either).
        from .rig_link import link_wheel_supplying_rig, merge_rig_link

        link = link_wheel_supplying_rig(reordered, bench, mode=resolved.mode)
        reordered = merge_rig_link(reordered, link)
    return compose_simulation_assembly(
        contributions,
        name=name,
        root_kind="axle",
        body_order=tuple(body_order),
        physical=reordered,
        rig=bench,
    )


def _rig_assembly(
    rig: str | None,
    *,
    mode: str,
    capabilities: AssemblyCapabilities | None,
) -> Assembly | None:
    """
    Return the bench's own level, entities included, or ``None``.

    The bench is a real contributor (``rigs/bench.py``), not only a declaration
    of what it drives: a wheel-supplying bench owns the wheel body and the tire
    force element, and the assembly deliberately builds neither (D9).  Without
    its entities the model has no wheel to load and the run is a different
    question than the one that was asked.

    ``SUSPENSION_MULTIBODY_RIG_ENTITIES=0`` restores the declaration-only bench,
    which is the pre-D1 behaviour: it exists so that the change can be *measured*
    against the state it replaced rather than only asserted.
    """
    if rig is None:
        return None
    if not _rig_entities_enabled():
        return None
    from ..rigs.bench import build_rig_assembly

    return build_rig_assembly(rig, capabilities=capabilities, mode=mode)


#: The switch that decides whether a bench's entities join the model.
RIG_ENTITIES_SWITCH = "SUSPENSION_MULTIBODY_RIG_ENTITIES"


def _rig_entities_enabled() -> bool:
    """
    Return whether the bench contributes entities.

    On by default: an assembly without its bench is not a simulation assembly.
    A leading ``0`` turns it off, which is the historical behaviour and the
    baseline the change is measured against.
    """
    import os

    value = os.environ.get(RIG_ENTITIES_SWITCH)
    if value is None or value == "":
        return True
    return not value.startswith("0")


def _reorder_bodies(runtime: SubsystemRuntime, order: list[str]) -> SubsystemRuntime:
    """
    Return the runtime with its bodies in the recorded document order.

    The composition records the order too, but the runtime is built here and a
    document reads *it*; ordering only the composition would let the two
    disagree, and the disagreement would show up as a reordered document rather
    than as an error.
    """
    known = set(runtime.bodies)
    ordered: dict[str, object] = {}
    for item in order:
        if item in known:
            ordered[item] = runtime.bodies[item]
    for item, body in runtime.bodies.items():
        ordered.setdefault(item, body)
    return replace(runtime, bodies=ordered)


def _explicit_simulation_assembly(
    model: FrontAxleModel,
    request: AssemblyRequest,
    *,
    name: str,
    rig: str | None = None,
):
    """
    Wrap an explicit model's runtime as a simulation assembly.

    There is nothing to compose: an explicit model declares its own parts and
    joints, so the runtime is built directly and wrapped in the same value shape
    the composed route returns.  The fragment still travels, because it is what
    carries the entity identity and provenance a caller may ask for.
    """
    runtime = build_explicit_runtime(model, request.mode)
    fragment = ModelFragment(
        bodies=dict(runtime.bodies),
        points=dict(runtime.points),
        joints={
            getattr(constraint, "name", f"joint_{index}"): {
                "kind": type(constraint).__name__,
                "constraint": constraint,
            }
            for index, constraint in enumerate(runtime.constraints)
        },
        ideal_constraints={
            getattr(constraint, "name", f"joint_{index}"): {
                "kind": type(constraint).__name__,
                "constraint": constraint,
            }
            for index, constraint in enumerate(runtime.ideal_constraints)
        },
        bushings={
            getattr(element, "name", f"bushing_{index}"): {
                "kind": type(element).__name__,
                "element": element,
            }
            for index, element in enumerate(runtime.bushings)
        },
        forces={
            getattr(element, "name", f"element_{index}"): {
                "kind": type(element).__name__,
                "element": element,
            }
            for index, element in enumerate(runtime.elements)
        },
        connections={
            str(connection.name): connection for connection in runtime.connections
        },
        provenance=FragmentProvenance(
            template="explicit_topology",
            revision=request.mode,
            instance=(name,),
        ),
    ).mounted((name,))
    assembly = Assembly(
        name=name,
        fragment=fragment,
        subsystems=frozenset(runtime.capabilities.subsystems)
        if runtime.capabilities is not None
        else frozenset(),
        root_kind="axle",
        physical=runtime,
        provenance=fragment.provenance,
    )
    bench = _rig_assembly(rig, mode=request.mode, capabilities=runtime.capabilities)
    return SimulationAssembly(
        name=name,
        assembly=assembly,
        rig=bench if bench is not None else Assembly(name="none", fragment=ModelFragment()),
        fingerprint=fingerprint_assembly(assembly),
    )
