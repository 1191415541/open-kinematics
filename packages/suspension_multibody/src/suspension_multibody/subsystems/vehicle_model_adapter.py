"""
The compatibility adapter: a ``VehicleModel`` in, an entry list out.

``VehicleModel`` is the historical shape -- exactly two axles, which the model
itself calls ``front_axle`` and ``rear_axle``.  The assembler in
``subsystems/assembler.py`` does not speak that shape: it is handed a list of
:class:`~suspension_multibody.subsystems.assembler.AxleEntry`, each saying where
its axle sits and which of its bodies the vehicle takes over.  This module is the
one place that translates between the two, and therefore the one place left in
the assembly path that assumes a vehicle has a front axle and a rear one.

It is deliberately a module of its own rather than a section of
``subsystems/vehicle_assembly.py``.  The entry point there is the historical name
every existing caller uses, and the reason it can stay is that the two-axle
assumption has somewhere else to live: reading it out of the entry point's body
would put it back on the path a document-driven caller travels, which is exactly
what the split removes.

Nothing here decides anything about the model.  Which wheel end belongs to which
placement is read off the wheel's own name (a ``VehicleModel`` carries one flat
wheel tuple, so nothing else says it), and ``replace_bodies`` writes down what
used to be a name rule -- "a body called ``chassis`` or ``ground`` becomes the
vehicle's chassis" -- as a declaration this model makes about its own two axles.
"""

from __future__ import annotations

from ..schema import VehicleModel, WheelSpec
from .assembler import AxleEntry, VehicleFacts, vehicle_facts

__all__ = ["axle_entries_for_model", "vehicle_facts_for"]


def _wheels_for(model: VehicleModel, placement: str) -> tuple[WheelSpec, ...]:
    """
    List the wheel ends the model carries under one placement.

    The split is by name because the model states no other one: its wheel tuple
    is flat.  Keeping that convention here, where the model's shape is known,
    is what lets the assembler stop caring about it.
    """
    return tuple(
        wheel for wheel in model.wheels if wheel.name.startswith(f"{placement}_")
    )


def axle_entries_for_model(model: VehicleModel) -> tuple[AxleEntry, ...]:
    """
    Return the entries a ``VehicleModel`` amounts to.

    The two placements and their prefixes are this model's own declaration -- the
    historical shape names them ``front_axle`` and ``rear_axle`` -- and a document
    driven caller builds its entries itself rather than coming through here.

    ``replace_bodies`` is the declaration that took over the old name rule: both
    axles declare a ``chassis`` body (and one declares a ``ground``), and a whole
    vehicle has exactly one chassis body, so those two axle bodies are handed over
    to the model's own.  Stating it per entry means the assembler applies no rule
    of its own to a name.
    """
    replaces = {"chassis": model.chassis.name, "ground": model.chassis.name}
    return (
        AxleEntry(
            placement="front",
            prefix="front_",
            axle=model.front_axle,
            replace_bodies=replaces,
            wheels=_wheels_for(model, "front"),
        ),
        AxleEntry(
            placement="rear",
            prefix="rear_",
            axle=model.rear_axle,
            replace_bodies=replaces,
            wheels=_wheels_for(model, "rear"),
        ),
    )


def vehicle_facts_for(model: VehicleModel) -> VehicleFacts:
    """
    Return what a reader needs to know about the assembly this model amounts to.

    The facts come from the same entry list ``compose_vehicle_runtime`` builds
    from, so "which axles, which of them is compliant, which racks are bolted" has
    one answer, and a reader asking it does not have to walk a named axle field to
    find out.
    """
    return vehicle_facts(axle_entries_for_model(model))
