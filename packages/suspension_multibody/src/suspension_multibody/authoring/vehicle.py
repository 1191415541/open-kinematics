"""
The vehicle-level numbers a full-vehicle assembly states, and how they round-trip.

A `full_vehicle` assembly names six subsystems, and until now that was the whole
of what a file could say about a vehicle: the chassis mass, the four wheel ends,
the steering ratio and the driveline were `VehicleDeclaration` data with no file
spelling, so "assemble a vehicle from files" stopped at the role set plus two file
axles.  The `vehicle` section closes that: it carries the four objects the
subsystems do not, and this module converts both ways.

Two boundaries are deliberate and stated here rather than left to be inferred:

* the section is validated structurally by the assembly schema and *semantically*
  by the model's own classes, so a bound such as "steering ratio must be positive"
  has one home.  The schema states that the objects are objects; the classes state
  what may be in them.
* the two axles are **not** in the section.  An axle is a subsystem, and an
  assembly already refers to its subsystems by file; stating the axles here as
  well would give one model two descriptions that could disagree.  Both the
  template-based conversion and this one take their axles from `file_axles_from`.
"""

from __future__ import annotations

import json
from typing import Any

from pydantic import ValidationError

from ..schema import (
    DrivelineSpec,
    RigidBodySpec,
    SteeringChannelSpec,
    SteeringSystemSpec,
    WheelSpec,
)
from ..schema.common import StrictModel
from ..schema.vehicle import VehicleDeclaration
from .bridge import axle_declaration_from
from .documents import AssemblyDocument, SimulationAssembly
from .errors import AuthoringError

__all__ = ["vehicle_declaration_from", "vehicle_document_from"]


def vehicle_document_from(model: VehicleDeclaration) -> dict[str, Any]:
    """
    Export one vehicle model's vehicle-level numbers to the file format.

    This is the other direction of `vehicle_declaration_from`, and it is what makes the
    `vehicle` section a *faithful* spelling of a vehicle rather than a subset of
    one.  The two axles are left out because they are subsystems: the assembly
    document refers to them by file, and writing them here as well would be a
    second description of the same axle.

    `steering_channels` is written only when the model declares any of them, so a
    one-channel vehicle exports the four keys it always did, byte for byte.
    """
    section = {
        "chassis": _stated(model.chassis),
        "wheels": [_stated(wheel) for wheel in model.wheels],
        "steering": _stated(model.steering),
        "driveline": _stated(model.driveline),
    }
    if model.steering_channels:
        section["steering_channels"] = [
            _stated(channel) for channel in model.steering_channels
        ]
    return section


def _stated(value: Any) -> dict[str, Any]:
    """
    Dump one vehicle-level object, including the fields its own dump hides.

    A handful of fields in the vehicle schema are excluded from `model_dump` on
    purpose: `api.py` hashes that dump into `Provenance.model_hash`, so a
    dump-visible field would move every recorded full-vehicle hash.  The file is a
    description of the *model* rather than of its hash, so it states them -- an
    export that silently dropped a brake coefficient would be lossy, and the loss
    would be invisible because the dropped value equals its default.
    """
    payload = value.model_dump(mode="json")
    for name, field in type(value).model_fields.items():
        if field.exclude and name not in payload:
            payload[name] = json.loads(json.dumps(getattr(value, name)))
    return payload


def vehicle_declaration_from(
    document: AssemblyDocument | SimulationAssembly,
    *,
    name: str | None = None,
) -> VehicleDeclaration:
    """
    Build a full-vehicle model from one assembly document.

    The document has to be a `full_vehicle` and has to carry a `vehicle` section.
    Both refusals are the same rule seen twice: a vehicle model whose chassis mass,
    wheel geometry or steering ratio was invented here would be a model nobody
    described, and the run that used it would look like a working vehicle.

    Everything is the file's -- the four corners, their poses and axes, the
    steering actuator and the driveline -- except the axles, which are the
    document's suspension subsystems.
    """
    assembly = document.assembly if isinstance(document, SimulationAssembly) else document
    if assembly.assembly_kind != "full_vehicle":
        raise AuthoringError(
            f"{assembly.where}: a vehicle model needs a full_vehicle assembly; "
            f"this document is a {assembly.assembly_kind!r}"
        )
    section = assembly.payload.get("vehicle")
    if section is None:
        raise AuthoringError(
            f"{assembly.where}: a full-vehicle assembly needs a 'vehicle' section "
            "stating the chassis, the four wheels and the steering system; this "
            "document states none"
        )
    axles = {entry.placement_role: axle_declaration_from(entry.effective(),
        name=f"{entry.placement_role}_{assembly.name}") for entry in assembly.entries
        if entry.functional_role == "suspension"}
    missing = sorted({"front", "rear"}-set(axles))
    if missing:
        raise AuthoringError(f"{assembly.where}: a vehicle model needs a suspension at {missing}; "
            f"this document places them at {sorted(axles)}")
    # The axles come back exactly as their own subsystem files describe them,
    # including whether each rack is bolted to the chassis.  A vehicle may declare
    # several steering channels (subtask p2-06), so "the file states one steering
    # system" no longer means one *steered* axle, and bolting the rear rack down
    # here would silently refuse the second channel a document asked for.  Whether
    # a rack is bolted is a fact about the axle its subsystem file describes, and
    # that file is the one place that decides it.
    # Each part is validated on its own so a failure names the part: a nested
    # `model_validate` reports its own field (`ratio`) and loses which object the
    # field was in, and "ratio must be greater than 0" is a worse report than
    # "vehicle.steering: ratio must be greater than 0".
    chassis = _validated(assembly.where, "chassis", RigidBodySpec, section["chassis"])
    steering = _validated(
        assembly.where, "steering", SteeringSystemSpec, section["steering"]
    )
    driveline = _validated(
        assembly.where, "driveline", DrivelineSpec, section.get("driveline", {})
    )
    wheels = tuple(
        _validated(assembly.where, f"wheels[{index}]", WheelSpec, row)
        for index, row in enumerate(section["wheels"])
    )
    channels = tuple(
        _validated(
            assembly.where, f"steering_channels[{index}]", SteeringChannelSpec, row
        )
        for index, row in enumerate(section.get("steering_channels", []))
    )
    try:
        return VehicleDeclaration(
            name=name or assembly.name,
            chassis=chassis,
            front_axle=axles["front"],
            rear_axle=axles["rear"],
            wheels=wheels,
            steering=steering,
            steering_channels=channels,
            driveline=driveline,
        )
    except ValidationError as exc:
        # What is left to fail here is the vehicle's own rules rather than one
        # part's bounds: four corners, one chassis body, wheels the driveline
        # refers to, an axle that names every body it needs.
        raise AuthoringError(
            f"{assembly.where}: the 'vehicle' section is not a vehicle: "
            f"{_first_problem(exc)}"
        ) from exc


def _validated(
    where: str, kind: str, model_class: type[StrictModel], payload: Any
) -> Any:
    """Validate one part of the vehicle section, naming the part on failure."""
    try:
        return model_class.model_validate(payload)
    except ValidationError as exc:
        raise AuthoringError(f"{where}: vehicle.{kind}: {_first_problem(exc)}") from exc


def _first_problem(exc: ValidationError) -> str:
    """Return the first validation problem, located by field path."""
    errors = exc.errors()
    if not errors:
        return str(exc)
    first = errors[0]
    location = ".".join(str(part) for part in first.get("loc", ()))
    return f"{location}: {first.get('msg', 'invalid')}"
