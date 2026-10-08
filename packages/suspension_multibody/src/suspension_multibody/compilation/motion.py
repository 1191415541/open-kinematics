"""Lower declared joint coordinates to native scalar motion rows."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..modeling.primitives.spatial import (
    quaternion_conjugate,
    quaternion_multiply,
    quaternion_to_matrix,
    rotation_vector_to_quaternion,
)
from ..modeling.resolved import ResolvedModel, ResolvedSolvePlan


@dataclass(frozen=True)
class MotionResolution:
    model: ResolvedModel
    targets: dict[str, tuple[float, ...]]
    rates: dict[str, tuple[float, ...]]


def resolve_motion_boundaries(model: ResolvedModel, plan: ResolvedSolvePlan) -> MotionResolution:
    """Keep bearings intact; each controlled coordinate adds exactly one row."""
    document = model.to_document()
    run = plan.to_document()
    coordinates = {row["name"]: row for row in document.get("coordinates", ())}
    ports = {row["name"]: row for row in document.get("ports", ())}
    joints = {row["name"]: row for row in document["joints"]}
    frames = {row["name"]: row for row in document["frames"]}
    bodies = {row["name"]: row for row in document["bodies"]}
    signals = {row["name"]: row for row in run["inputs"]}
    targets, rates = {}, {}
    controlled = set()
    for boundary in run["boundaries"]:
        name = boundary["coordinate"]
        if name in ports:
            name = ports[name].get("coordinate", "")
        coordinate = coordinates.get(name)
        if coordinate is None:
            raise ValueError(f"boundary {boundary['name']!r}: unknown joint coordinate")
        if name in controlled:
            raise ValueError(f"duplicate motion boundary for {name!r}")
        controlled.add(name)
        if boundary["mode"] == "free":
            continue
        rotation = coordinate["kind"] == "rotation"
        unit = "rad" if rotation else "m"
        if boundary.get("units", unit) != unit:
            raise ValueError(f"motion boundary requires {unit}")
        if boundary["mode"] == "prescribed_angle" and not rotation:
            raise ValueError("prescribed_angle requires a rotational coordinate")
        if boundary["mode"] == "prescribed_displacement" and rotation:
            raise ValueError("prescribed_displacement requires a translational coordinate")
        source = joints[coordinate["source_joint_id"]]
        action, reaction = source["body_b"], source["body_a"]
        if action not in bodies or bodies[action].get("fixed", False):
            raise ValueError("motion coordinate action body must be movable")
        frame = frames[coordinate["frame_a"]]
        axis = quaternion_to_matrix(np.asarray(frame["quaternion"])) @ coordinate["axis"]
        qa = np.asarray(bodies[action].get("quaternion", [1., 0., 0., 0.]))
        qb = np.asarray(bodies.get(reaction, {}).get("quaternion", [1., 0., 0., 0.]))
        relative = quaternion_multiply(quaternion_conjugate(qb), qa)
        reference = quaternion_multiply(rotation_vector_to_quaternion(-axis*coordinate["reference"]), relative)
        signal = boundary["name"]
        if signal in joints:
            raise ValueError(f"motion boundary repeats entity ID {signal!r}")
        if boundary["mode"] == "locked":
            values = [float(boundary["value"])]*len(run["samples"])
            derivatives = [0.]*len(values)
        else:
            program = signals.get(boundary["program"])
            if program is None or "values" not in program or "rates" not in program:
                raise ValueError("prescribed motion requires sampled values and analytic rates")
            values, derivatives = program["values"], program["rates"]
            if len(values) != len(run["samples"]) or len(derivatives) != len(values):
                raise ValueError("motion program samples disagree with solve plan")
        if not np.isfinite(values).all() or not np.isfinite(derivatives).all():
            raise ValueError("motion program must contain finite values and rates")
        row = {"name": signal, "type": "driven_rotation" if rotation else "driven_translation", "body_a": action,
            "body_b": reaction, "point_a": source.get("point_b", [0, 0, 0]),
            "point_b": source.get("point_a", [0, 0, 0]), "axis_b": axis.tolist(),
            "target": signal}
        if rotation:
            row["reference_quaternion"] = reference.tolist()
        document["joints"].append(row)
        targets[signal] = tuple(values) if rotation else tuple(value-coordinate["reference"] for value in values)
        rates[signal] = tuple(derivatives)
    return MotionResolution(ResolvedModel(document, model.resource_payload), targets, rates)
