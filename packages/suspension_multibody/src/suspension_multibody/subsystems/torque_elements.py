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
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

from ..modeling.identity import EntityId
from ..modeling.ports import GeometryPort, PortRequirement

if TYPE_CHECKING:  # pragma: no cover - annotations only, keeps the import small
    from ..schema import VehicleModel
    from .vehicle_assembly import VehicleRuntime

__all__ = [
    "BRAKE_DEMAND",
    "DRIVE_DEMAND",
    "REACTION_ROLE",
    "rotational_torque_rows",
    "wheel_demand_wheels",
]

#: The driver demand a wheel's brake follows.
BRAKE_DEMAND = "brake_torque"
#: The driver demand a wheel's drive follows.
DRIVE_DEMAND = "wheel_torque"

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
    """
    declared = getattr(model.driveline, "torque_demand", "none")
    if declared == "none" or declared != ("drive" if demand == DRIVE_DEMAND else "brake"):
        return ()
    if demand == DRIVE_DEMAND:
        driven = tuple(model.driveline.driven_wheels)
        return driven
    return tuple(wheel.name for wheel in model.wheels if wheel.braked)


def _reaction_ports(
    instance: tuple[str, ...], runtime: VehicleRuntime
) -> dict[str, GeometryPort]:
    """
    Offer one reaction port per wheel end, on the member that carries it.

    The owner is the wheel end's own carrier as the assembly recorded it, so a
    topology whose wheel hangs on a strut offers the port there and a
    double-wishbone's offers it on its upright -- without this module asking
    which topology it is looking at.
    """
    ports: dict[str, GeometryPort] = {}
    for wheel, (carrier, _centre) in runtime.wheel_centers.items():
        ports[f"reaction_{wheel}"] = GeometryPort(
            id=EntityId(instance, f"reaction_{wheel}"),
            owner=EntityId(instance, carrier),
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
    that wants others states them on the template.
    """
    from ..templates.builtin import BRAKE, DRIVE

    if demand == BRAKE_DEMAND:
        template = BRAKE
        slots = _slots(template)
        source = 2
        shared = brake_shared(model, wheel)
        return subsystem.wheel_torque_element(
            slots,
            wheel=wheel,
            own_body=own_body,
            report=report,
            ports=ports,
            demand=shared,
            share=_brake_share(model, wheel),
            demand_source=source,
            demand_tire=_tire_index(model, wheel),
        )
    template = DRIVE
    slots = _slots(template)
    return subsystem.wheel_torque_element(
        model.driveline,
        slots=slots,
        wheel=wheel,
        own_body=own_body,
        report=report,
        ports=ports,
        drive=shared_drive(model),
        demand_source=1,
        demand_tire=_tire_index(model, wheel),
    )


def _slots(template: Any) -> dict[str, float]:
    """Return a template's property slots at their declared defaults."""
    return {
        slot.name: float(slot.default)
        for slot in template.property_slots
        if slot.default is not None
    }


def brake_shared(model: VehicleModel, wheel: str) -> float:
    """Return the normalized brake demand this wheel's element is built at."""
    from ..schema import TimeSignal

    signal = dict(model.driveline.__dict__.get("_brake_signal", ()) or ())
    if wheel in signal:
        return 1.0
    del TimeSignal
    return 1.0


def _brake_share(model: VehicleModel, wheel: str) -> float:
    """
    Return this wheel's part of one brake demand.

    The front/rear split is no longer a slot of the brake role (one element per
    wheel states its own share), so it is read here from the driveline's own
    ``front_brake_bias`` -- the field the retired builder used -- which keeps a
    model that states 60/40 producing the split it always did.
    """
    if not wheel.startswith("front_"):
        return 1.0 - float(model.driveline.front_brake_bias)
    return float(model.driveline.front_brake_bias)


def shared_drive(model: VehicleModel) -> float:
    """Return the normalized drive demand a driven wheel's element is built at."""
    return 1.0


def _tire_index(model: VehicleModel, wheel: str) -> int:
    """Return the wheel's index in the case's per-tire tables."""
    names = tuple(item.name for item in model.wheels)
    try:
        return names.index(wheel)
    except ValueError as error:  # pragma: no cover - the caller checked first
        raise ValueError(f"unknown wheel {wheel!r}") from error
