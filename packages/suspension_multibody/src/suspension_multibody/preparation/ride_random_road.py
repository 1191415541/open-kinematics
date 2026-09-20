"""
Preparation for the ``vehicle`` ``ride_random_road`` family.

A random-road request is a vehicle model plus the road profile each wheel
follows and the speed it is traversed at.  Preparing it assembles the vehicle
once -- the model document carries the steering actuators and tire parameter
blocks the run needs -- and then writes the model document and the road's case
document.

The profile travels as spatial components rather than as sampled heights: the
kernel turns each wavelength into a temporal harmonic, and the case document is
the declaration of those components.  The case's time grid is the vehicle run's
output grid so both documents describe the same window.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from suspension_contracts import pack_container

from ..axle_dynamics.schema import AxleSolverSettings
from ..cases.ride_random_road import RandomRoadWheel
from ..schema import VehicleDynamicCase, VehicleModel
from ..simulation.preparation import PreparedSimulation
from ..simulation.request import SimulationRequest

ASSEMBLY = "vehicle"
FAMILY = "ride_random_road"


@dataclass(frozen=True)
class RandomRoadRide:
    """One random-road run: the vehicle it excites, its profiles and its speed."""

    vehicle_case: VehicleDynamicCase
    wheels: tuple[RandomRoadWheel, ...]
    speed_mps: float
    name: str = "ride-random-road"
    settings: AxleSolverSettings | None = None


@dataclass(frozen=True)
class RideRandomRoadPrepared:
    """The vehicle model document and the random-road case document."""

    model_document: dict[str, Any]
    model_payload: bytes
    case_document: dict[str, Any]


def _validate_identity(request: SimulationRequest) -> None:
    """Reject a request this family does not own."""
    if request.assembly != ASSEMBLY or request.family != FAMILY:
        raise ValueError(
            f"random-road preparation expects {ASSEMBLY}/{FAMILY}, "
            f"got {request.assembly}/{request.family}"
        )


def prepare_request(request: SimulationRequest) -> PreparedSimulation:
    """Assemble the vehicle and author the random-road documents for one request."""
    _validate_identity(request)
    model = request.model
    ride = request.case
    if not isinstance(model, VehicleModel):
        raise TypeError(
            "random-road preparation requires a VehicleModel, got "
            f"{type(model).__name__}"
        )
    if not isinstance(ride, RandomRoadRide):
        raise TypeError(
            f"random-road preparation requires a RandomRoadRide, got "
            f"{type(ride).__name__}"
        )

    from ..cases.ride_random_road import case_document
    from ..cases.vehicle_dynamic import model_document
    from .vehicle_dynamic import prepare_vehicle_run

    vehicle = prepare_vehicle_run(model, ride.vehicle_case)
    name = request.name or ride.name
    model_emitted, model_blob = model_document(model, vehicle, name=name)
    case_emitted = case_document(
        name=name,
        wheels=ride.wheels,
        speed_mps=ride.speed_mps,
        times_s=tuple(float(value) for value in vehicle.times),
        settings=ride.settings if ride.settings is not None else AxleSolverSettings(),
    )
    prepared = RideRandomRoadPrepared(
        model_document=model_emitted,
        model_payload=pack_container(model_emitted, model_blob),
        case_document=case_emitted,
    )
    return PreparedSimulation(
        request=request,
        value=prepared,
        context={
            "model_document": model_emitted,
            "model_payload": prepared.model_payload,
            "case_document": case_emitted,
        },
        metadata={"assembly": ASSEMBLY, "family": FAMILY},
    )


__all__ = [
    "ASSEMBLY",
    "FAMILY",
    "RandomRoadRide",
    "RideRandomRoadPrepared",
    "prepare_request",
]
