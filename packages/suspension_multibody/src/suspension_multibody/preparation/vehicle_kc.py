"""
Preparation for the ``vehicle`` ``vehicle_kc`` family.

A vehicle-level K/C request is a vehicle model, the wheel corners the sweep
drives and the wheel/rack travels it prescribes.  Preparing it assembles the
vehicle once and hands the compiler the inputs its document authoring needs:
the vehicle model document pair, the assembly the driven coordinates are read
from, and the corners they are read for.

The driven coordinates belong to the sweep rather than to the vehicle model --
a K sweep prescribes them, a ride or handling case does not -- so they are not
part of the prepared model document and are added by the family's compiler,
which is where that document authoring already lives.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ..axle_dynamics.schema import AxleSolverSettings
from ..cases.vehicle_kc import VehicleKcCorner
from ..schema import VehicleDynamicCase, VehicleModel
from ..simulation.preparation import PreparedSimulation
from ..simulation.request import SimulationRequest

ASSEMBLY = "vehicle"
FAMILY = "vehicle_kc"


@dataclass(frozen=True)
class VehicleKcSweep:
    """One vehicle K/C sweep: the vehicle, the corners and the travels it drives."""

    vehicle_case: VehicleDynamicCase
    wheel_values_mm: tuple[float, ...] = (0.0,)
    rack_values_mm: tuple[float, ...] = (0.0,)
    left_right_mode: str = "symmetric"
    corners: tuple[VehicleKcCorner, ...] = ()
    #: The sweep's own output grid.  A K/C window is the ramp the driven
    #: coordinates are raised over, so it belongs to the sweep rather than to
    #: the vehicle case; the vehicle run's grid is used when it is not stated.
    times_s: tuple[float, ...] | None = None
    name: str = "vehicle-kc"
    settings: AxleSolverSettings | None = None


@dataclass(frozen=True)
class VehicleKcPrepared:
    """The assembly, the driven corners and the sweep's prepared inputs."""

    #: Typed loosely because the prepared vehicle run exposes it as ``object``:
    #: the vehicle assembly the driven coordinates are read from.
    assembly: object
    wheels: tuple[VehicleKcCorner, ...]
    model_document_pair: tuple[dict[str, Any], bytes]
    case_document: dict[str, Any]


def _validate_identity(request: SimulationRequest) -> None:
    """Reject a request this family does not own."""
    if request.assembly != ASSEMBLY or request.family != FAMILY:
        raise ValueError(
            f"vehicle K/C preparation expects {ASSEMBLY}/{FAMILY}, "
            f"got {request.assembly}/{request.family}"
        )


def prepare_request(request: SimulationRequest) -> PreparedSimulation:
    """Assemble the vehicle and prepare the sweep's inputs and case document."""
    _validate_identity(request)
    model = request.model
    sweep = request.case
    if not isinstance(model, VehicleModel):
        raise TypeError(
            f"vehicle K/C preparation requires a VehicleModel, got {type(model).__name__}"
        )
    if not isinstance(sweep, VehicleKcSweep):
        raise TypeError(
            "vehicle K/C preparation requires a VehicleKcSweep, got "
            f"{type(sweep).__name__}"
        )

    from ..cases.vehicle_kc import case_document, vehicle_model_document
    from .vehicle_dynamic import prepare_vehicle_run

    vehicle = prepare_vehicle_run(model, sweep.vehicle_case)
    times = (
        tuple(float(value) for value in vehicle.times)
        if sweep.times_s is None
        else sweep.times_s
    )
    name = request.name or sweep.name
    model_pair = vehicle_model_document(model, vehicle)
    case_emitted = case_document(
        name=name,
        wheel_values_mm=sweep.wheel_values_mm,
        rack_values_mm=sweep.rack_values_mm,
        times_s=times,
        settings=(
            sweep.settings if sweep.settings is not None else AxleSolverSettings()
        ),
        left_right_mode=sweep.left_right_mode,
        corners=sweep.corners,
    )
    prepared = VehicleKcPrepared(
        assembly=vehicle.assembly,
        wheels=sweep.corners,
        model_document_pair=model_pair,
        case_document=case_emitted,
    )
    return PreparedSimulation(
        request=request,
        value=prepared,
        context={
            "model_document_pair": model_pair,
            "vehicle_assembly": vehicle.assembly,
            "wheels": sweep.corners,
            "case_document": case_emitted,
        },
        metadata={"assembly": ASSEMBLY, "family": FAMILY},
    )


__all__ = [
    "ASSEMBLY",
    "FAMILY",
    "VehicleKcPrepared",
    "VehicleKcSweep",
    "prepare_request",
]
