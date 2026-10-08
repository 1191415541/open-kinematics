"""Solver-independent SI entity graph and run-plan validation."""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

from .multibody import (
    ContractError,
    canonical_json_bytes,
    validate,
    validate_case,
    validate_model,
)


def _indexed(rows: Any, label: str) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for row in rows:
        name = row.get("name")
        if not isinstance(name, str) or not name:
            raise ContractError(f"{label}: stable name is required")
        if name in result:
            raise ContractError(f"{label}: duplicate ID {name!r}")
        result[name] = row
    return result


def _inertia(matrix: Any, label: str) -> None:
    if not isinstance(matrix, (list, tuple)) or len(matrix) != 3 or any(
        not isinstance(row, (list, tuple)) or len(row) != 3 for row in matrix
    ):
        raise ContractError(f"{label}: inertia must be 3x3")
    if any(not isinstance(value, (int, float)) or not math.isfinite(value) for row in matrix for value in row):
        raise ContractError(f"{label}: inertia must be finite")
    scale = max(abs(value) for row in matrix for value in row)
    if scale == 0:
        return
    values = [[value / scale for value in row] for row in matrix]
    if any(abs(values[i][j] - values[j][i]) > 1e-12 for i in range(3) for j in range(3)):
        raise ContractError(f"{label}: inertia must be symmetric")
    half_trace = sum(values[i][i] for i in range(3)) / 2
    moment = [[(half_trace if i == j else 0) - values[i][j] for j in range(3)] for i in range(3)]
    a, b, c = (moment[i][i] for i in range(3))
    d, e, f = moment[0][1], moment[0][2], moment[1][2]
    minors = (a, b, c, a*b-d*d, a*c-e*e, b*c-f*f, a*b*c+2*d*e*f-a*f*f-b*e*e-c*d*d)
    if min(minors) < -1e-12:
        raise ContractError(f"{label}: inertia must be physically realizable")


def _vector(value: Any, size: int, label: str) -> None:
    if not isinstance(value, (list, tuple)) or len(value) != size or any(
        not isinstance(item, (int, float)) or not math.isfinite(item) for item in value
    ):
        raise ContractError(f"{label}: expected {size} finite values")


def _rotate(quaternion: Any, vector: Any) -> list[float]:
    w, x, y, z = quaternion
    a, b, c = vector
    return [
        (1-2*(y*y+z*z))*a + 2*(x*y-w*z)*b + 2*(x*z+w*y)*c,
        2*(x*y+w*z)*a + (1-2*(x*x+z*z))*b + 2*(y*z-w*x)*c,
        2*(x*z-w*y)*a + 2*(y*z+w*x)*b + (1-2*(x*x+y*y))*c,
    ]


def validate_resolved_model(document: Mapping[str, Any]) -> None:
    """Validate SI units, physical inertia and every explicit entity reference."""
    validate(document, "resolved_model")
    canonical_json_bytes(document)
    bodies = _indexed(document["bodies"], "bodies")
    frames = _indexed(document["frames"], "frames")
    joints = _indexed(document["joints"], "joints")
    elements = _indexed(document["elements"], "elements")
    tires = _indexed(document.get("tires", ()), "tires")
    ports = _indexed(document.get("ports", ()), "ports")
    coordinates = _indexed(document.get("coordinates", ()), "coordinates")
    couplers = _indexed(document.get("couplers", ()), "couplers")
    for name, row in couplers.items():
        for end in ("a", "b"):
            source = joints.get(row.get("joint_" + end))
            if source is None:
                raise ContractError(f"coupler {name!r}: unknown joint_{end}")
            allowed = {"rotation": {"revolute", "cylindrical"}, "translation": {"prismatic", "cylindrical"}}
            if source["type"] not in allowed.get(row.get("coordinate_" + end, "rotation"), set()):
                raise ContractError(f"coupler {name!r}: incompatible coordinate_{end}")
    for row in document.get("gauges", ()):
        if row.get("body") not in bodies:
            raise ContractError("rotation gauge: unknown body")
        _vector(row.get("axis_local"), 3, "rotation gauge")
        if sum(value*value for value in row["axis_local"]) <= 1e-12:
            raise ContractError("rotation gauge: axis must be nonzero")
    _indexed(document.get("signals", ()), "signals")
    measurements = _indexed(document.get("measurements", ()), "measurements")
    owners = set(bodies) | {"ground"}
    for name, row in measurements.items():
        kind = row.get("type")
        units = {"toe": "rad", "camber": "rad", "frame_position": "m", "tire_load": "N", "roll_center": "m"}.get(kind)
        if units is None or row.get("units") != units:
            raise ContractError(f"measurement {name!r}: unknown primitive or incompatible units")
        references = [row.get("reference", "world")]
        if kind in {"toe", "camber", "frame_position"}:
            references.append(row.get("frame"))
        if kind in {"toe", "camber"}:
            _vector(row.get("axis"), 3, name)
            if abs(sum(value*value for value in row["axis"])-1) > 1e-10:
                raise ContractError(f"measurement {name!r}: axis must be normalized")
            if row.get("sign", 1) not in (-1, 1):
                raise ContractError(f"measurement {name!r}: sign must be -1 or 1")
        if kind == "tire_load" and row.get("element") not in tires:
            raise ContractError(f"measurement {name!r}: unknown tire")
        if kind == "roll_center":
            for key in ("contact_frames", "drive_frames"):
                if len(row.get(key, ())) != 2:
                    raise ContractError(f"measurement {name!r}: {key} must name two frames")
                references.extend(row[key])
            if not row.get("constraints") or any(value not in joints for value in row["constraints"]):
                raise ContractError(f"measurement {name!r}: unknown or missing constraint IDs")
        if any(value != "world" and value not in frames for value in references):
            raise ContractError(f"measurement {name!r}: unknown frame")
    native_bodies = []
    for name, body in bodies.items():
        _inertia(body["inertia"], name)
        _vector(body.get("center_of_mass", [0., 0., 0.]), 3, name)
        if "quaternion" in body and abs(sum(value*value for value in body["quaternion"]) - 1) > 1e-10:
            raise ContractError(f"body {name!r}: quaternion must be normalized")
        native_bodies.append({key: value for key, value in body.items() if key != "center_of_mass"})
    for name, frame in frames.items():
        if frame["body"] not in owners:
            raise ContractError(f"frame {name!r}: unknown body {frame['body']!r}")
        norm = sum(value*value for value in frame["quaternion"])
        if abs(norm - 1) > 1e-10:
            raise ContractError(f"frame {name!r}: quaternion must be normalized")
    for rows in (joints, elements):
        for name, row in rows.items():
            for key in ("body_a", "body_b"):
                if key in row and row[key] not in owners:
                    raise ContractError(f"{name!r}: unknown {key} {row[key]!r}")
            if row["type"] in {"force", "torque", "wrench"}:
                for key in ("action", "reaction", "reference"):
                    marker = row.get("parameters", {}).get(key)
                    if marker is None or marker.get("body") not in owners:
                        raise ContractError(f"{name!r}: unknown {key} frame owner")
    for name, tire in tires.items():
        if tire["body"] not in bodies:
            raise ContractError(f"tire {name!r}: unknown body")
    for name, coordinate in coordinates.items():
        source = joints.get(coordinate["source_joint_id"])
        if source is None:
            raise ContractError(f"coordinate {name!r}: unknown source joint")
        expected = "rad" if coordinate["kind"] == "rotation" else "m"
        joint_type = "revolute" if coordinate["kind"] == "rotation" else "prismatic"
        if source["type"] != joint_type:
            raise ContractError(f"coordinate {name!r}: source must be a {joint_type} joint")
        if coordinate["units"] != expected:
            raise ContractError(f"coordinate {name!r}: incompatible units")
        for suffix in ("a", "b"):
            frame = frames.get(coordinate[f"frame_{suffix}"])
            if frame is None or frame["body"] != source[f"body_{suffix}"]:
                raise ContractError(f"coordinate {name!r}: frame/joint owner mismatch")
        if abs(sum(value*value for value in coordinate["axis"]) - 1) > 1e-10:
            raise ContractError(f"coordinate {name!r}: axis must be normalized")
        axis = _rotate(frames[coordinate["frame_a"]]["quaternion"], coordinate["axis"])
        source_axis = source.get("axis_a", [0., 0., 1.])
        norm = math.sqrt(sum(value*value for value in source_axis))
        if norm == 0 or sum(axis[i]*source_axis[i] for i in range(3))/norm < 1-1e-10:
            raise ContractError(f"coordinate {name!r}: axis must follow source joint's positive axis")
    for name, port in ports.items():
        if port["kind"] == "geometry" and port.get("frame") not in frames:
            raise ContractError(f"port {name!r}: unknown frame")
        if port["kind"] == "geometry" and frames[port["frame"]]["body"] != port["owner"]:
            raise ContractError(f"port {name!r}: frame/port owner mismatch")
        if port["kind"] == "spin":
            coordinate = coordinates.get(port.get("coordinate"))
            if coordinate is None or coordinate["kind"] != "rotation":
                raise ContractError(f"port {name!r}: unknown rotational coordinate")
            if port.get("units") != "rad":
                raise ContractError(f"port {name!r}: spin units must be rad")
            joint = joints[coordinate["source_joint_id"]]
            if port["owner"] not in {joint["body_a"], joint["body_b"]}:
                raise ContractError(f"port {name!r}: spin owner must be a bearing endpoint")

    # Fixed mounts may put a tire on a wheel proxy. Trace the declared bearing
    # through that component, so its contact frame cannot inherit relative spin.
    fixed_links: dict[str, set[str]] = {}
    for joint in joints.values():
        if joint["type"] == "fixed":
            for end, other in (("a", "b"), ("b", "a")):
                fixed_links.setdefault(joint["body_"+end], set()).add(joint["body_"+other])

    def fixed_members(owner: str) -> set[str]:
        members, pending = {owner}, [owner]
        while pending:
            for other in fixed_links.get(pending.pop(), ()):
                if other not in members:
                    members.add(other)
                    pending.append(other)
        return members

    for name, tire in tires.items():
        wheel_members = fixed_members(tire["body"])
        frame_body = tire.get("parameters", {}).get("frame_body", tire["body"])
        for port in ports.values():
            if port["kind"] != "spin" or port["owner"] not in wheel_members:
                continue
            source = joints[coordinates[port["coordinate"]]["source_joint_id"]]
            carrier = source["body_b"] if source["body_a"] in wheel_members else source["body_a"]
            if frame_body in wheel_members or frame_body not in fixed_members(carrier):
                raise ContractError(f"tire {name!r}: contact frame must exclude the declared bearing's relative spin")
    native = {
        "contract": "multibody-model", "contract_version": 1, "kind": "model",
        "name": document["name"], "units": {"length": "m", "mass": "kg", "time": "s", "angle": "rad"},
        "bodies": native_bodies, "joints": document["joints"], "elements": document["elements"],
    }
    for key in ("tires", "function_programs", "gravity", "road", "blobs", "couplers", "gauges", "capabilities"):
        if key in document:
            native[key] = document[key]
    validate_model(native)


def validate_solve_plan(document: Mapping[str, Any]) -> None:
    """Validate one run's facts; model entities cannot be stored on the plan."""
    validate(document, "solve_plan")
    canonical_json_bytes(document)
    _indexed(document["boundaries"], "boundaries")
    _indexed(document["inputs"], "inputs")
    if "excitation" in document:
        forbidden = {"contract", "contract_version", "kind", "name", "family", "solver", "time", "bodies", "joints", "elements", "tires"} & document["excitation"].keys()
        if forbidden:
            raise ContractError(f"solve plan excitation cannot redefine {sorted(forbidden)}")
        validate_case({"contract": "multibody-case", "contract_version": 1, "kind": "case",
            "name": document["name"], "family": document.get("protocol", "axle_dynamic"),
            "solver": document["solver"], **document["excitation"]})
    if any(b <= a for a, b in zip(document["samples"], document["samples"][1:])):
        raise ContractError("solve plan samples must strictly increase")
    controlled: set[str] = set()
    for boundary in document["boundaries"]:
        coordinate = boundary["coordinate"]
        if boundary["mode"] == "prescribed_speed":
            raise ContractError("prescribed_speed is not supported by this backend")
        if boundary["mode"] == "free" and any(key in boundary for key in ("value", "program")):
            raise ContractError("free boundary cannot prescribe motion")
        if coordinate in controlled:
            raise ContractError(f"duplicate motion boundary for {coordinate!r}")
        controlled.add(coordinate)
        if boundary["mode"] == "locked" and "value" not in boundary:
            raise ContractError("locked boundary requires a reference value")
        if boundary["mode"].startswith("prescribed") and "program" not in boundary:
            raise ContractError("prescribed boundary requires a function program")
