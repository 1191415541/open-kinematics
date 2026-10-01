"""
The full vehicle, built by the composition layer.

A ``VehicleModel`` names an axle here and an axle there under one chassis.  It used
to be assembled by the retired vehicle builder calling the historical axle build
twice and merging the results by hand; this module reaches the same model by
composing each axle through ``subsystems/si_assembly.py`` and joining them under a
local chassis.

Two decisions are worth stating, because both are load-bearing:

**This module is the entry point, not the assumption.**  It composes what
``subsystems/vehicle_model_adapter.py`` hands it -- an entry list -- and the
two-axle assumption lives there.  Reading it out of this file would put it back on
every path that reaches this name, which is what the split exists to avoid.

**The wheel ends and the weld handling are the same code.**  They are subtle -- a
fixed wheel is condensed into its mount body by composite mass properties, and a
weld may be fused or sent as a constraint depending on an environment switch --
and a second implementation of either would drift from the first without any
result showing it.  So the assembler *calls* the existing helpers rather than
re-deriving them.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from typing import Literal

import numpy as np

from ..modeling.primitives import (
    Constraint,
    RigidBody,
    RigidBodyState,
)
from ..schema import VehicleModel, WheelSpec
from .assembler import compose_entries_runtime
from .capabilities import AssemblyCapabilities
from .runtime import SubsystemRuntime
from .types import (
    AssemblyRequest,
    Connection,
)
from .vehicle_model_adapter import axle_entries_for_model

__all__ = [
    "VehicleRuntime",
    "compose_vehicle_runtime",
]


@dataclass(frozen=True)
class VehicleRuntime:
    """
    The runtime face of one assembled vehicle.

    The fields are the historical assembly's, in the same order and with the same
    meaning, so that the post-processing helpers (weld condensation, isolated-body
    dropping) and every existing reader work on this value unchanged.  ``state``
    is built from ``bodies``; the four wheel tables are keyed by wheel name and
    answer the three different questions the vehicle layer asks about a wheel:
    where its centre is, which body carries it, and how its reference frame sits
    in that body.
    """

    mode: Literal["K", "C"]
    bodies: dict[str, RigidBody]
    state: RigidBodyState
    points: dict[tuple[str, str], np.ndarray]
    constraints: tuple[Constraint, ...]
    ideal_constraints: tuple[Constraint, ...]
    elements: tuple[object, ...]
    connections: tuple[Connection, ...]
    wheel_specs: dict[str, WheelSpec]
    wheel_centers: dict[str, tuple[str, np.ndarray]]
    wheel_body_names: dict[str, str]
    wheel_rotations_local: dict[str, np.ndarray]
    #: The per-axle runtime this vehicle was merged from, keyed by the placement
    #: its entry declared.  A SubsystemRuntime exposes the same ``points`` mapping
    #: the historical axle assembly did, which is what the steering branch of the
    #: vehicle layer reads.
    axle_assemblies: dict[str, SubsystemRuntime]
    body_aliases: dict[str, str] = field(default_factory=dict)
    capabilities: AssemblyCapabilities | None = None
    #: The body the assembly named as the vehicle's own chassis.  The two
    #: post-processing rules that used to ask whether a body is *called* chassis
    #: read it here instead: a fused weld component is named after something, and
    #: the chassis is never an isolated body.  Empty means the assembly named none,
    #: which is the honest answer for a runtime built without a vehicle body.
    chassis_name: str = ""

    @property
    def component_ids(self) -> tuple[str, ...]:
        """Return deterministic body identifiers."""
        return tuple(self.bodies)

    @property
    def wheel_ids(self) -> tuple[str, ...]:
        """Return deterministic corner identifiers."""
        return tuple(self.wheel_specs)

    @property
    def element_ids(self) -> tuple[str, ...]:
        """Return deterministic force-element identifiers."""
        return tuple(
            getattr(element, "name", f"element_{index}")
            for index, element in enumerate(self.elements)
        )

    @property
    def total_mass(self) -> float:
        """Return the sum of all movable and fixed body masses."""
        return float(sum(body.mass for body in self.bodies.values()))

    def wheel_center_local(self, wheel: str) -> np.ndarray:
        """Return the wheel-center point on its upright body."""
        try:
            return self.wheel_centers[wheel][1].copy()
        except KeyError as exc:
            raise KeyError(f"unknown wheel {wheel!r}") from exc


def compose_vehicle_runtime(
    model: VehicleModel,
    mode: Literal["K", "C"] = "K",
    request: AssemblyRequest | None = None,
) -> VehicleRuntime:
    """
    Compose suspension, wheel ends and chassis into one vehicle runtime.

    This is the historical entry point and it keeps its name and its signature:
    everything that builds a vehicle from a :class:`VehicleModel` calls it.  What
    changed is where the work is.  A ``VehicleModel`` says "an axle here and an
    axle there"; the assembler in ``subsystems/assembler.py`` wants an entry list
    and knows nothing about which axle is which, so the adapter in
    ``subsystems/vehicle_model_adapter.py`` writes that list and this function
    hands it on.

    ``request`` names the subsystems the vehicle carries.  It defaults to the full
    set, so a caller that says nothing gets exactly the vehicle it always got, and
    a caller that resolved its roles from an assembly file gets that assembly
    instead of a role list this function decided on its own.
    """
    from .vehicle_parts import _body_from_spec

    runtime = compose_entries_runtime(
        axle_entries_for_model(model),
        chassis_name=model.chassis.name,
        chassis_body=_body_from_spec(model.chassis),
        mode=mode,
        request=request,
    )
    return _with_torque_elements(model, runtime)


def _with_torque_elements(
    model: VehicleModel, runtime: VehicleRuntime
) -> VehicleRuntime:
    """
    Return ``runtime`` with the model's brake and drive torque elements added.

    The elements are built *after* the assembly rather than inside it because a
    couple's reaction member is the wheel end's own carrier, and that table only
    exists once every entry has handed its wheels over.  Building them here also
    keeps the entry list free of a role the vehicle owns: the brake and drive
    are the vehicle's subsystem, exactly as ``DEFAULT_VEHICLE_SUBSYSTEMS`` says.

    A model that declares no driver demand
    (``DrivelineSpec.torque_demand == "none"``, the default) gets a runtime whose
    ``elements`` are the ones the assembly produced, untouched -- which is what
    keeps every recorded result exactly what it was.
    """
    from .element_build import build_element
    from .torque_elements import rotational_torque_rows

    rows = rotational_torque_rows(model, runtime)
    if not rows:
        return runtime
    return replace(runtime, elements=runtime.elements + tuple(build_element(row) for row in rows))
