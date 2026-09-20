"""
Preparation for the ``vehicle`` ``handling`` family.

A handling request is a vehicle model plus the steering shapes the manoeuvre
drives.  Preparing it assembles the vehicle once -- the model document carries
the steering actuators the case refers to, so it cannot be authored without the
vehicle run -- and then writes the two contract documents the compiler frames.

The manoeuvre's own time grid is the vehicle run's output grid, so the case
document and the model document describe the same run; the family's solver
settings are its own, because a handling case states the solver it was
integrated with rather than inheriting the vehicle case's.

The family's documents are already SI, and the model document's payload carries
the tire parameter blocks, so the prepared context carries the packed payload
the compiler submits.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from suspension_contracts import pack_container

from ..axle_dynamics.schema import AxleSolverSettings
from ..cases.handling import SteeringShape
from ..schema import VehicleDynamicCase, VehicleModel
from ..simulation.preparation import PreparedSimulation
from ..simulation.request import SimulationRequest

ASSEMBLY = "vehicle"
FAMILY = "handling"


@dataclass(frozen=True)
class HandlingManoeuvre:
    """One handling manoeuvre: the vehicle it runs on and its steering shapes."""

    vehicle_case: VehicleDynamicCase
    shapes: tuple[SteeringShape, ...]
    name: str = "handling"
    settings: AxleSolverSettings | None = None


@dataclass(frozen=True)
class HandlingPrepared:
    """The vehicle model document and the manoeuvre case document."""

    model_document: dict[str, Any]
    model_payload: bytes
    case_document: dict[str, Any]


def _validate_identity(request: SimulationRequest) -> None:
    """Reject a request this family does not own."""
    if request.assembly != ASSEMBLY or request.family != FAMILY:
        raise ValueError(
            f"handling preparation expects {ASSEMBLY}/{FAMILY}, "
            f"got {request.assembly}/{request.family}"
        )


def prepare_request(request: SimulationRequest) -> PreparedSimulation:
    """Assemble the vehicle and author the handling documents for one request."""
    _validate_identity(request)
    model = request.model
    manoeuvre = request.case
    if not isinstance(model, VehicleModel):
        raise TypeError(
            f"handling preparation requires a VehicleModel, got {type(model).__name__}"
        )
    if not isinstance(manoeuvre, HandlingManoeuvre):
        raise TypeError(
            "handling preparation requires a HandlingManoeuvre, got "
            f"{type(manoeuvre).__name__}"
        )

    from ..cases.handling import case_document
    from ..cases.vehicle_dynamic import model_document
    from .vehicle_dynamic import prepare_vehicle_run

    vehicle = prepare_vehicle_run(model, manoeuvre.vehicle_case)
    name = request.name or manoeuvre.name
    model_emitted, model_blob = model_document(model, vehicle, name=name)
    case_emitted = case_document(
        name=name,
        shapes=manoeuvre.shapes,
        times_s=tuple(float(value) for value in vehicle.times),
        settings=(
            manoeuvre.settings if manoeuvre.settings is not None else AxleSolverSettings()
        ),
    )
    prepared = HandlingPrepared(
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
    "HandlingManoeuvre",
    "HandlingPrepared",
    "prepare_request",
]
