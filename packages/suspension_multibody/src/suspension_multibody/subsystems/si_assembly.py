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
from typing import Literal

from ..modeling.identity import EntityId
from ..modeling.ports import GeometryPort
from ..preparation.assembly import build_front_axle
from ..schema import FrontAxleModel
from . import chassis as chassis_subsystem
from . import steering as steering_subsystem
from . import suspension as suspension_subsystem
from . import wheel as wheel_subsystem
from .composition import (
    SI_ASSEMBLY_NAME,
    CompositionError,
    SubsystemContribution,
    compose_simulation_assembly,
)
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
    """
    ports: dict[str, GeometryPort] = {}
    for name in bodies:
        side = name.rsplit("_", 1)[-1] if name.endswith(("_L", "_R")) else None
        ports[name] = _port(instance, name, "body", side)
    return ports


def contributions_for_axle(
    model: FrontAxleModel,
    *,
    request: AssemblyRequest | None = None,
    name: str = SI_ASSEMBLY_NAME,
) -> tuple[SubsystemContribution, ...]:
    """
    Produce the six (or fewer) contributions the axle's subsystem set implies.

    The set comes from ``request.subsystems``, so an assembly without steering
    simply has no steering contribution -- and, because the composition resolves
    requirements afterwards, the tie rods and rack are *absent* rather than
    present-but-degenerate.  That is the same rule the historical build applies,
    reached by construction instead of by a conditional in an ordered sequence.

    The context is threaded exactly as ``build_front_axle`` threads it, because
    the subsystems read shared state: the suspension's uprights have to exist
    before steering can resolve its tie rod points against them.  What changes is
    that this ordering is now an implementation detail of *producing* the
    contributions, and no longer decides whether the assembly is correct.
    """
    from ..preparation.assembly.front_axle import side_hardpoints

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
                elements=[
                    row for output in suspension_outputs for row in output.elements
                ],
            ),
            ports=_ports_for_bodies(instance, suspension_bodies),
            note=f"left/right suspension pair, mode {mode}",
        )
    )

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

    return tuple(contributions)


def si_assembly_for_axle(
    model: FrontAxleModel,
    *,
    request: AssemblyRequest | None = None,
    name: str = SI_ASSEMBLY_NAME,
):
    """
    Compose the SI simulation assembly for an axle, from its subsystems.

    Returns a :class:`~suspension_multibody.modeling.assembly.SimulationAssembly`
    carrying a structural fingerprint, so the same model read by two studies can
    be shown to be one model.
    """
    resolved = request if request is not None else AssemblyRequest(mode="K")
    contributions = contributions_for_axle(model, request=resolved, name=name)
    # The recorded order the contract document follows: chassis, rack, then per
    # side arm / arm / upright / tie rod.  Stated here rather than left to the
    # merge order, because the document lists bodies in sequence and the
    # historical build's sequence is part of what it produces.
    order: list[str] = ["chassis"]
    if resolved.carries("steering"):
        order.append("rack")
    for side in _SIDES:
        order.extend([f"upper_arm_{side}", f"lower_arm_{side}", f"upright_{side}"])
        if resolved.carries("steering"):
            order.append(f"tie_rod_{side}")
    return compose_simulation_assembly(
        contributions,
        name=name,
        root_kind="axle",
        body_order=tuple(order),
        physical=build_front_axle(model, resolved.mode, resolved),
    )
