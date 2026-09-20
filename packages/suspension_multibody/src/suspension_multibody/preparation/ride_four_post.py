"""
Preparation for the ``vehicle`` ``ride_four_post`` family.

A four-post request is a vehicle model plus the pad shapes the rig drives.
Preparing it assembles the vehicle once -- the model document carries the
steering actuators and tire parameter blocks the run needs -- and then writes
the model document and the rig's case document.

The pads are declared as shapes rather than sampled tables: the kernel expands
them, and the case document is the declaration.  The case's time grid is the
vehicle run's output grid so both documents describe the same window.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from suspension_contracts import pack_container

from ..axle_dynamics.schema import AxleSolverSettings
from ..cases.ride_four_post import FourPostCorner
from ..schema import VehicleDynamicCase, VehicleModel
from ..simulation.preparation import PreparedSimulation
from ..simulation.request import SimulationRequest

ASSEMBLY = "vehicle"
FAMILY = "ride_four_post"


@dataclass(frozen=True)
class FourPostRide:
    """One four-post run: the vehicle it excites and the pad shapes it drives."""

    vehicle_case: VehicleDynamicCase
    corners: tuple[FourPostCorner, ...]
    name: str = "ride-four-post"
    settings: AxleSolverSettings | None = None


@dataclass(frozen=True)
class RideFourPostPrepared:
    """The vehicle model document and the four-post case document."""

    model_document: dict[str, Any]
    model_payload: bytes
    case_document: dict[str, Any]


def _validate_identity(request: SimulationRequest) -> None:
    """Reject a request this family does not own."""
    if request.assembly != ASSEMBLY or request.family != FAMILY:
        raise ValueError(
            f"four-post preparation expects {ASSEMBLY}/{FAMILY}, "
            f"got {request.assembly}/{request.family}"
        )


def prepare_request(request: SimulationRequest) -> PreparedSimulation:
    """Assemble the vehicle and author the four-post documents for one request."""
    _validate_identity(request)
    model = request.model
    ride = request.case
    if not isinstance(model, VehicleModel):
        raise TypeError(
            "four-post preparation requires a VehicleModel, got "
            f"{type(model).__name__}"
        )
    if not isinstance(ride, FourPostRide):
        raise TypeError(
            f"four-post preparation requires a FourPostRide, got {type(ride).__name__}"
        )

    from ..cases.ride_four_post import case_document
    from ..cases.vehicle_dynamic import model_document
    from .vehicle_dynamic import prepare_vehicle_run

    vehicle = prepare_vehicle_run(model, ride.vehicle_case)
    name = request.name or ride.name
    model_emitted, model_blob = model_document(model, vehicle, name=name)
    case_emitted = case_document(
        name=name,
        corners=ride.corners,
        times_s=tuple(float(value) for value in vehicle.times),
        settings=ride.settings if ride.settings is not None else AxleSolverSettings(),
    )
    prepared = RideFourPostPrepared(
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
    "FourPostRide",
    "RideFourPostPrepared",
    "prepare_request",
]
