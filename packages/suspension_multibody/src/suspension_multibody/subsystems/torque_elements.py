"""
The brake and drive torque elements, attached to a vehicle's wheel ends.

Until this module existed the two roles were *registered* and never *called*: a
vehicle composition produced an `elements` tuple with no torque element in it at
all, and the only callers of `brake.wheel_torque_element` /
`drive.wheel_torque_element` were those modules' own tests.  A model's braking
and driving therefore reached the solver through the preparation layer's
pre-sampled per-wheel torque tables -- the mechanism the road map's 2.1 section
retires.  This is the step that puts the element on the assembly.

Where the two bodies come from
------------------------------
A couple acts between the wheel it slows and the member that reacts it.  Neither
body is *named* here:

* the wheel is the wheel-end the assembly attached, read from
  ``runtime.wheel_body_names`` -- the same table the wheel subsystem filled when
  it built the wheel end;
* the reaction member is the wheel end's own carrier, read from
  ``runtime.wheel_centers[wheel][0]``, which the same build recorded as the
  ``upright`` the wheel centre hangs on.

Those two facts are published as ports and matched by
:func:`~suspension_multibody.connections.matcher.match_requirements`, and the
element's ends are read off the matched port's owner -- exactly the rule
:mod:`~suspension_multibody.compilation.element_blocks` applies to any other
torque pairing.  That is what lets a caliper bracket or a subframe meet the
requirement without this module, or the element, knowing either name: a body
name appearing as a literal here would be the identity guess the assembly layer
forbids.

Why a model gets none by default
--------------------------------
The elements are produced only for the wheels the model *declares a driver
demand for* (``DrivelineSpec.torque_demand``).  A model that declares none keeps
the pre-sampled wheel-torque tables it always had, byte for byte -- which is what
makes this additive rather than a behaviour change to every recorded result.

The two units the slots and the kernel each use
-----------------------------------------------
The standardized template slots are the recorded Adams simple-brake numbers, so
they are in the model's own engineering units: `piston_area` in mm^2 and
`effective_radius` in mm, which makes ``brake_amplitude`` an **N*mm** figure.
The kernel's law works in SI -- `stiffness`, `max_torque` and the couple it
applies are all N*m -- and it scales a document's moment *parameters* by the
document's length unit (``contract_model.cpp``'s ``stiffness *= length_scale_``).
So the value this module hands the element is the amplitude converted to N*m,
and the conversion is `MM_TO_M`, named rather than inlined: a bare ``/ 1000.0``
here would be the kind of magic the assembly layer does not get to have.

The share is the recorded split, not a new one
----------------------------------------------
``_brake_share`` reproduces the retired builder's allocation exactly: the front
bias is split **among the braked wheels of that axle** rather than given whole
to each (``preparation/vehicle_dynamic.py``'s ``_build_wheel_torque_signals``
divided by ``len(front_braked)``), so a 60/40 car brakes its two front wheels at
0.3 each.  The share multiplies the element's gain, which is the same statement
as scaling the demand: ``min((gain*share)*demand, cap*share)``.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from ..modeling.identity import EntityId
from ..modeling.ports import GeometryPort, PortRequirement

if TYPE_CHECKING:  # pragma: no cover - annotations only, keeps the import small
    from ..schema import VehicleModel
    from .vehicle_assembly import VehicleRuntime

__all__ = [
    "BRAKE_DEMAND",
    "BRAKE_SOURCE",
    "DRIVE_DEMAND",
    "DRIVE_SOURCE",
    "MM_TO_M",
    "REACTION_ROLE",
    "rotational_torque_rows",
    "wheel_demand_wheels",
]

#: The driver demand a wheel's brake follows, as the model spells it.
BRAKE_DEMAND = "brake_torque"
#: The driver demand a wheel's drive follows, as the model spells it.
DRIVE_DEMAND = "wheel_torque"

#: The kernel's `TorqueDemandSource` codes (`mb_model/enums.hpp`).
#:
#: They travel on the element's integer block, so the numbers are the kernel's
#: and not this module's invention: 0 is the unit demand, 1 the case's
#: per-tire `wheel_torque` column and 2 its `brake_torque` column.  The columns
#: a *normalized* demand uses are the two new roles, which is why the case
#: emitter writes `brake_pressure`/`throttle_demand` for these elements.
BRAKE_SOURCE = 2
DRIVE_SOURCE = 1

#: Millimetres per metre: the template's slot units to the kernel's SI ones.
MM_TO_M = 1.0e-3

#: The requirement role each end of a couple is matched through.  A *role*, not
#: a body name: what a vehicle declares is that these wheels need something to
#: react against, and which member offers that is a property of the topology.
REACTION_ROLE = "torque_reaction"


def wheel_demand_wheels(model: VehicleModel, demand: str) -> tuple[str, ...]:
    """
    Return the wheels a model declares a driver demand for, in wheel order.

    The answer reads the model's own declaration rather than assuming every
    wheel is braked: a template that states one driven corner is a different
    vehicle from one that states four, and the elements must be one per declared
    wheel or the count would be a guess.

    ``DrivelineSpec.torque_demand`` is the switch the whole mechanism hangs on,
    and its default is ``"none"`` -- the state every existing model is in.  A
    model that states none produces no elements here and keeps the pre-sampled
    tables its results were recorded from.
    """
    declared = getattr(model.driveline, "torque_demand", "none")
    if declared == "none":
        return ()
    wanted = "drive" if demand == DRIVE_DEMAND else "brake"
    if declared != wanted and declared != "both":
        return ()
    if demand == DRIVE_DEMAND:
        return tuple(model.driveline.driven_wheels)
    return tuple(wheel.name for wheel in model.wheels if wheel.braked)


def _wheel_reaction_body(runtime: VehicleRuntime, wheel: str) -> str:
    """
    Return the member a wheel's couple must react on, from the topology alone.

    This is the one decision in this module that a wrong answer makes silently
    wrong rather than absent, so it is worth stating what the right answer is.

    A wheel is joined to its carrier in two steps: the wheel body is **welded**
    into a wheel-hub body (``vehicle_parts.py`` builds that weld whenever the
    mount it landed on is a hub), and that rigid pair turns in the carrier
    through the spin **revolute** joint (``suspension.py``'s
    ``wheel_spin_joint_*``, whose axis is the wheel's own spin axis).  A couple
    applied between the wheel body and the hub would therefore act *inside* one
    rigid component -- the weld transmits it whole, the pair's relative rate is
    identically zero, and the law's "a stopped pair gets no couple" branch makes
    the element do nothing while every interface still looks correct.

    So the reaction end is the far side of the spin joint: the member the welded
    wheel-plus-hub rotates *in*.  The wheel body stays the driven end, and the
    couple travels through the weld into the hub the way a real brake's does.

    The walk is by *constraint kind and axis*, never by name:

    * the rigid component is the connected set of bodies joined by welds;
    * a candidate is a revolute joint with exactly one end in that component;
    * it is *this wheel's* spin joint when its axis, taken on the component's
      side, is parallel to the wheel's own spin axis.

    Zero candidates and several candidates are both refused by name rather than
    guessed at.  A topology that cannot say which joint is the spin joint cannot
    say which member reacts the couple either, and picking one would be the
    identity guess the assembly layer forbids.
    """
    from ..modeling.primitives.joints import RevoluteJoint, WeldJoint

    start = runtime.wheel_body_names.get(wheel)
    if start is None:
        raise ValueError(
            f"the wheel {wheel!r} has no wheel end in this assembly, so nothing "
            "says which member reacts its couple"
        )

    component = {start}
    changed = True
    while changed:
        changed = False
        for constraint in runtime.constraints:
            if not isinstance(constraint, WeldJoint):
                continue
            ends = {constraint.body_a, constraint.body_b}
            if ends & component and not ends <= component:
                component |= ends
                changed = True

    wheel_spec = runtime.wheel_specs.get(wheel)
    if wheel_spec is None:
        raise ValueError(f"the wheel {wheel!r} has no wheel spec in this assembly")
    spin_axis = np.asarray(wheel_spec.spin_axis.as_array(), dtype=float)
    spin_axis = spin_axis / np.linalg.norm(spin_axis)

    candidates: list[str] = []
    for constraint in runtime.constraints:
        if not isinstance(constraint, RevoluteJoint):
            continue
        ends = {constraint.body_a, constraint.body_b}
        inside = ends & component
        if len(inside) != 1:
            # A joint wholly inside the rigid component is not a joint at all
            # after the weld, and one wholly outside does not touch this wheel.
            continue
        outer = next(iter(ends - component))
        # The joint's axis is stated in each body's frame; the side inside the
        # component is the one whose axis is the wheel's own spin axis.  A small
        # tolerance compares directions only, so the two bodies' frames may
        # differ by a pose without turning this into a geometry rebuild.
        axis_inside = (
            constraint.axis_a if constraint.body_a in component else constraint.axis_b
        )
        axis = np.asarray(axis_inside, dtype=float)
        norm = np.linalg.norm(axis)
        if norm <= 1e-12:
            continue
        axis = axis / norm
        if float(np.dot(axis, spin_axis)) > 1.0 - 1e-6:
            candidates.append(outer)
        elif float(np.dot(axis, -spin_axis)) > 1.0 - 1e-6:
            # An axis stated the other way round describes the same joint.
            candidates.append(outer)

    if len(candidates) != 1:
        raise ValueError(
            f"the wheel {wheel!r} has {len(candidates)} members it spins in "
            f"({sorted(candidates)}); a couple's reaction end needs exactly one, "
            "and which one is meant is a property of the topology rather than "
            "something to guess"
        )
    return candidates[0]


def _reaction_ports(
    instance: tuple[str, ...], runtime: VehicleRuntime
) -> dict[str, GeometryPort]:
    """
    Offer one reaction port per wheel end, on the member that reacts its couple.

    The owner comes from :func:`_wheel_reaction_body`, so a topology whose wheel
    turns in a strut offers the port there and a double-wishbone's offers it on
    its upright -- without this module asking which topology it is looking at.
    """
    ports: dict[str, GeometryPort] = {}
    for wheel in runtime.wheel_centers:
        ports[f"reaction_{wheel}"] = GeometryPort(
            id=EntityId(instance, f"reaction_{wheel}"),
            owner=EntityId(instance, _wheel_reaction_body(runtime, wheel)),
            role=REACTION_ROLE,
            capabilities=frozenset({"geometry"}),
            labels=frozenset({wheel}),
        )
    return ports


def rotational_torque_rows(
    model: VehicleModel,
    runtime: VehicleRuntime,
    *,
    instance: tuple[str, ...] = ("vehicle",),
) -> tuple[Any, ...]:
    """
    Return the torque element rows a model's declarations imply.

    One row per declared wheel per demand.  Each row's two bodies come from a
    match between the wheel's own requirement and the carrier's offered port, so
    the whole path from "which member reacts this couple" to the built element
    has no step that names a body.
    """
    from ..connections.matcher import match_requirements

    brakes = wheel_demand_wheels(model, BRAKE_DEMAND)
    drives = wheel_demand_wheels(model, DRIVE_DEMAND)
    if not brakes and not drives:
        return ()

    from . import brake as brake_subsystem
    from . import drive as drive_subsystem

    ports = _reaction_ports(instance, runtime)
    rows: list[Any] = []
    roles = (
        (BRAKE_DEMAND, brakes, brake_subsystem),
        (DRIVE_DEMAND, drives, drive_subsystem),
    )
    for demand, wheels, subsystem in roles:
        for wheel in wheels:
            own_body = runtime.wheel_body_names.get(wheel)
            if own_body is None:
                raise ValueError(
                    f"the model declares a {demand} demand for wheel {wheel!r}, "
                    "which this assembly does not carry"
                )
            requirements = (
                PortRequirement(
                    role=REACTION_ROLE,
                    count=1,
                    required=True,
                    match_labels=frozenset({wheel}),
                    note=f"the member that reacts {wheel}'s {demand}",
                ),
            )
            report = match_requirements(requirements, ports)
            element = _element_for(
                subsystem,
                model,
                wheel=wheel,
                own_body=own_body,
                report=report,
                ports=ports,
                demand=demand,
            )
            rows.append(element)
    return tuple(rows)


def _element_for(
    subsystem: Any,
    model: VehicleModel,
    *,
    wheel: str,
    own_body: str,
    report: Any,
    ports: dict[str, GeometryPort],
    demand: str,
) -> Any:
    """
    Build one wheel's element through its own subsystem.

    The slot values are the role's own defaults, which is what "the parameter
    comes from the template's property slot" means for a model that states no
    properties file: the simplified template carries its numbers, and a caller
    that wants others states them on the template.  The gain is converted to
    SI, because the slots are in the model's engineering units and the kernel's
    couple is in newton-metres -- see the module docstring.
    """
    from ..templates.builtin import BRAKE, DRIVE

    if demand == BRAKE_DEMAND:
        slots = _slots(BRAKE)
        return subsystem.wheel_torque_element(
            slots,
            wheel=wheel,
            own_body=own_body,
            report=report,
            ports=ports,
            demand=1.0,
            share=brake_share(model, wheel),
            gain_scale=MM_TO_M,
            reaction_role=REACTION_ROLE,
            demand_source=BRAKE_SOURCE,
            demand_tire=_tire_index(model, wheel),
        )
    slots = _slots(DRIVE)
    return subsystem.wheel_torque_element(
        model.driveline,
        slots=slots,
        wheel=wheel,
        own_body=own_body,
        report=report,
        ports=ports,
        drive=1.0,
        gain_scale=MM_TO_M,
        reaction_role=REACTION_ROLE,
        demand_source=DRIVE_SOURCE,
        demand_tire=_tire_index(model, wheel),
    )


def _slots(template: Any) -> dict[str, float]:
    """Return a template's property slots at their declared defaults."""
    return {
        slot.name: float(slot.default)
        for slot in template.property_slots
        if slot.default is not None
    }


def brake_share(model: VehicleModel, wheel: str) -> float:
    """
    Return this wheel's part of one brake demand, the recorded way.

    The retired builder split the bias among the braked wheels of the wheel's
    own axle, so a 60/40 car brakes each front wheel at 0.3 and each rear at
    0.2 -- reproducing that here is what keeps the elements' allocation the one
    the recorded results were produced with.  A wheel that is not a front wheel
    takes the rear share; a model that brakes only one axle gives that axle's
    wheels the whole demand, which is what "split among the braked wheels"
    means when there is one of them.
    """
    bias = float(model.driveline.front_brake_bias)
    front_wheels = sorted(
        name
        for name, spec in ((item.name, item) for item in model.wheels)
        if spec.braked and name.startswith("front_")
    )
    rear_wheels = sorted(
        name
        for name, spec in ((item.name, item) for item in model.wheels)
        if spec.braked and name.startswith("rear_")
    )
    if wheel.startswith("front_"):
        return bias / len(front_wheels) if front_wheels else 0.0
    return (1.0 - bias) / len(rear_wheels) if rear_wheels else 0.0


def _tire_index(model: VehicleModel, wheel: str) -> int:
    """Return the wheel's index in the case's per-tire tables."""
    names = tuple(item.name for item in model.wheels)
    try:
        return names.index(wheel)
    except ValueError as error:  # pragma: no cover - the caller checked first
        raise ValueError(f"unknown wheel {wheel!r}") from error
