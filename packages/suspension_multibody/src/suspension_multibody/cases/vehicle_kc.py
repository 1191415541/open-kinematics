"""
Emit and run the `vehicle_kc` contract documents.

A vehicle-level K/C sweep prescribes the driven coordinates of a whole vehicle --
four wheel-centre travels and the rack -- and measures what the body does.  The
case layer's expansion is the same cartesian grid the axle family uses; what
this module adds is the *model* side: a vehicle assembly does not declare wheel
drives, so they are written here from the assembly's own attachment points.

Putting them here rather than in the vehicle model is deliberate.  A wheel-centre
drive is a property of the case being run -- a K sweep prescribes it, a ride or
handling case does not -- and a model that always carried one would pin the
suspension in every other family.

**This family asks for no steering actuator, rather than removing one.**  A K/C
sweep drives the rack itself, and a prescribed steering actuator on the same
degree of freedom would be a second row on one coordinate.  The answer is to ask
the model document for a vehicle without steering elements -- the same
"add, never remove" boundary subtask 05 of the assembly-layer epic applied to
the test rig, where a bench stopped stripping a declaration it did not want and
started not requesting it.  The two are the same rule read from two sides and
they belong to *different* epics: that one owns the rig files
(``subsystems/rig_link.py``, ``rigs/**``) and this one owns this family's model
document, so neither repeats the other's change.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..axle_dynamics.schema import AxleSolverSettings
from .vehicle_dynamic import _solver_block
from .vehicle_dynamic import model_document as _vehicle_model_document

__all__ = [
    "VehicleKcCorner",
    "case_document",
    "driven_joints",
    "model_document",
]


@dataclass(frozen=True)
class VehicleKcCorner:
    """One driven wheel: the name it is addressed by and the body it moves."""

    name: str
    body: str


def _default_corners() -> tuple[VehicleKcCorner, ...]:
    return (
        VehicleKcCorner("front_left", "front_upright_L"),
        VehicleKcCorner("front_right", "front_upright_R"),
        VehicleKcCorner("rear_left", "rear_upright_L"),
        VehicleKcCorner("rear_right", "rear_upright_R"),
    )


def driven_joints(
    assembly, corners: tuple[VehicleKcCorner, ...] = (), *, rack_body: str = "front_rack"
) -> list[dict[str, Any]]:
    """
    Return the driven-coordinate joints a vehicle K/C sweep prescribes.

    Each wheel drive is a translation along the chassis vertical between the
    wheel centre and the chassis, and the rack drive is a translation along the
    chassis lateral axis.  The axis is written in the reaction body's frame
    because that is the frame the relative separation is measured in.
    """
    corners = corners or _default_corners()
    joints: list[dict[str, Any]] = []

    def add(name: str, body: str, point_local_mm, axis) -> None:
        joints.append(
            {
                "name": name,
                "type": "driven_translation",
                "body_a": body,
                "body_b": "chassis",
                "point_a": [float(value) / 1000.0 for value in np.asarray(point_local_mm)[:3]],
                "point_b": [0.0, 0.0, 0.0],
                "axis_b": [float(value) for value in np.asarray(axis)[:3]],
                "reference_quaternion": [1.0, 0.0, 0.0, 0.0],
                "target": name,
            }
        )

    def point(body: str, label: str):
        # The vehicle assembly stores its attachment points in a flat table
        # keyed by (body, label); the axle assembly wraps the same table in a
        # `point()` accessor.  Going through the table works for both.
        try:
            return assembly.points[(body, label)]
        except KeyError as error:
            raise ValueError(f"the assembly has no {label!r} point on {body!r}") from error

    for corner in corners:
        body, center_local = _driven_wheel_centre(assembly, corner, point)
        add(f"wheel_drive_{corner.name}", body, center_local, (0.0, 0.0, 1.0))
    add("rack_drive", rack_body, point(rack_body, "center"), (0.0, 1.0, 0.0))
    return joints


def _driven_wheel_centre(assembly, corner: VehicleKcCorner, point):
    """
    Return the body one corner's wheel-centre drive acts through, and its centre.

    The centre belongs to whichever body *declares* it: `方式 A` gives the wheel a hub
    of its own, so the driven point is the hub's, while a topology that hangs the wheel
    straight on the upright leaves it there.  The vehicle build states both already --
    `wheel_centers` holds every wheel's centre on the body that carries it -- so the
    drive reads them from there rather than from the upright's name: a corner names a
    *wheel*, not the part that happens to carry it.
    """
    centers = getattr(assembly, "wheel_centers", {})
    if corner.name in centers:
        body, center = centers[corner.name]
        return body, np.asarray(center, dtype=float)
    return corner.body, point(corner.body, "wheel_center")


def model_document(
    model_document_pair: tuple[dict[str, Any], bytes],
    *,
    wheels: tuple[VehicleKcCorner, ...],
    assembly,
) -> tuple[dict[str, Any], bytes]:
    """
    Add the sweep's driven coordinates to a vehicle model document.

    Only additions: the base document is asked for a vehicle *without* steering
    actuators and the sweep's own driven coordinates are appended to its joints.
    The earlier form built the document with the actuators and then deleted every
    ``steering_actuator`` element from it, which is why this function's contract
    is now stated as a request rather than as a correction -- a driver that is
    "the K sweep drives the rack itself" belongs in the request, and a deletion
    would have to be repeated once per steering channel now that a vehicle may
    declare several.

    The payload is unchanged: driven coordinates are topology, not data.
    """
    document, blob = model_document_pair
    document = dict(document)
    document["elements"] = list(document["elements"])
    document["joints"] = list(document["joints"]) + driven_joints(assembly, wheels)
    return document, blob


def case_document(
    *,
    name: str,
    wheel_values_mm: tuple[float, ...],
    rack_values_mm: tuple[float, ...],
    times_s: tuple[float, ...],
    settings: AxleSolverSettings,
    left_right_mode: str = "symmetric",
    corners: tuple[VehicleKcCorner, ...] = (),
) -> dict[str, Any]:
    """Describe a vehicle K/C sweep as a case document."""
    times = np.asarray(times_s, dtype=float)
    if times.size < 2:
        raise ValueError("a vehicle K/C case needs at least two sample times")
    steps = np.diff(times)
    if not np.allclose(steps, steps[0], rtol=0.0, atol=1e-15):
        raise ValueError(
            "the contract time grid is a start/end/step, so the samples must be uniform"
        )
    corners = corners or _default_corners()
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "vehicle_kc",
        "name": name,
        "time": {
            "start_s": float(times[0]),
            "end_s": float(times[-1]),
            "step_s": float(steps[0]),
        },
        "solver": _solver_block(settings),
        "k": {
            "wheel_values_mm": [float(value) for value in wheel_values_mm],
            "rack_values_mm": [float(value) for value in rack_values_mm],
            # The travel is reached over the whole window rather than applied at
            # the first sample: a static trim starts from the assembling pose and
            # cannot take a full-travel step in one Newton step.
            "ramp_s": float(times[-1] - times[0]),
            "drive": "wheel_center",
            "left_right_mode": left_right_mode,
            "axis_map": {
                "wheel": [f"wheel_drive_{corner.name}" for corner in corners],
                "rack": "rack_drive",
            },
        },
    }


def vehicle_model_document(model, prepared) -> tuple[dict[str, Any], bytes]:
    """
    Return the vehicle model document this family starts from.

    Steering actuators are left out at the source (subtask p2-06): this family
    prescribes the rack through its own driven coordinate, and two rows on one
    degree of freedom is a rank-deficient constraint set.  Asking the document
    builder for a vehicle without them is the boundary; deleting them afterwards
    was the correction.
    """
    return _vehicle_model_document(model, prepared, include_steering_actuators=False)
