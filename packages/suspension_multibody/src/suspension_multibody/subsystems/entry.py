"""
The two assemblies, built by composing subsystems.

This is the composition layer's own door.  It exists because the composition had
no short form: ``si_assembly_for_axle(model, request=...).assembly.physical`` says
what it does, but a caller who just wants an axle had to know the internal
structure of a ``SimulationAssembly`` to reach it.

Both signatures deliberately match the historical entries so they are
interchangeable at every call site.  What changed is *which* builds them: these
compose subsystems, the old ones were hand-written paths beside the composition.
Those paths are gone; these replaced them.
"""

from __future__ import annotations

from typing import Literal

from ..schema import FrontAxleModel, VehicleModel
from .runtime import SubsystemRuntime
from .si_assembly import si_assembly_for_axle
from .types import AssemblyRequest
from .vehicle_assembly import VehicleRuntime, compose_vehicle_runtime

__all__ = ["compose_axle", "compose_vehicle"]


def compose_axle(
    model: FrontAxleModel,
    mode: Literal["K", "C"] | None = None,
    request: AssemblyRequest | None = None,
) -> SubsystemRuntime:
    """
    Compose one axle from its subsystems and return the runtime it carries.

    ``mode`` and ``request`` say the same thing twice, so either alone is enough and
    passing both is accepted only when they agree -- the same contract the
    historical entry had, kept so a caller does not have to learn a second one.

    Omitting ``request`` asks for the subsystem set the axle has always carried, so
    a caller that wants the default does not have to say so.
    """
    if mode is not None and mode not in ("K", "C"):
        raise ValueError("mode must be K or C")
    if request is None:
        request = AssemblyRequest(mode=mode or "K")
    elif mode is not None and request.mode != mode:
        raise ValueError(
            f"mode {mode!r} disagrees with request.mode {request.mode!r}; "
            "pass one or the other, or make them agree"
        )
    return si_assembly_for_axle(model, request=request).assembly.physical


def compose_vehicle(
    model: VehicleModel,
    mode: Literal["K", "C"] = "K",
    request: AssemblyRequest | None = None,
) -> VehicleRuntime:
    """
    Compose one vehicle: two composed axles merged under one chassis.

    The vehicle is a *user* of the composed axle rather than a second assembly path
    beside it, so a change to how an axle is composed reaches the vehicle without
    this function being touched.  That is the property the two hand-written paths
    could never have.

    ``request`` names the subsystems the vehicle carries, which is how an assembly
    file's own role set reaches the composition; omitting it asks for the full
    vehicle this entry has always built.
    """
    return compose_vehicle_runtime(model, mode=mode, request=request)
