"""Compile one immutable SI graph and one run plan, independently of assembly roles."""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any

import numpy as np
from suspension_contracts import (
    CONTRACT_VERSION,
    pack_container,
    validate_case,
    validate_model,
)

from ..modeling.resolved import ResolvedModel, ResolvedSolvePlan
from ..simulation.request import CompiledSimulation, SimulationRequest
from .motion import resolve_motion_boundaries

_IDENTITY = {"contract", "contract_version", "kind", "name", "family", "solver", "time"}
_GRID_PROTOCOLS = {"kc_quasi_static", "vehicle_kc"}
_TABLE_PROTOCOLS = {"axle_dynamic", "vehicle_dynamic"}


def plan_from_case(case: Mapping[str, Any], input_payload: bytes = b"") -> ResolvedSolvePlan:
    """Retain native excitation declarations without copying or deriving a model."""
    validate_case(case)
    time = case.get("time", {})
    if "samples" in time:
        descriptor = next((row for row in case.get("blobs", ()) if row["name"] == time["samples"]), None)
        if descriptor is None:
            raise ValueError("case time names a missing sample table")
        start, length = descriptor["offset"], descriptor["length"]
        if start < 0 or length < 0 or start+length > len(input_payload):
            raise ValueError("case sample table falls outside input payload")
        samples = np.frombuffer(input_payload[start:start+length], dtype="<f8").tolist()
    else:
        start, end, step = (float(time.get(key, default)) for key, default in
            (("start_s", 0), ("end_s", .001), ("step_s", .001)))
        if not np.isfinite([start, end, step]).all() or step <= 0 or end <= start:
            raise ValueError("case time requires a finite positive interval and step")
        samples = np.linspace(start, end, int(round((end-start)/step))+1).tolist()
    return ResolvedSolvePlan({"schema_version": 1, "name": case["name"],
        "study": "quasi_static" if case["family"] in _GRID_PROTOCOLS else "dynamic",
        "protocol": case["family"], "samples": samples, "solver": dict(case.get("solver", {})),
        "excitation": {key: value for key, value in case.items() if key not in _IDENTITY},
        "boundaries": [], "inputs": [], "outputs": []}, input_payload)


def native_model_document(model: ResolvedModel) -> dict[str, Any]:
    """Emit entities once; study and protocol never select bodies or force laws."""
    graph = model.to_document()
    for body in graph["bodies"]:
        center = body.pop("center_of_mass", [0, 0, 0])
        if any(value != 0 for value in center):
            raise ValueError("native body frame must be located at its declared center of mass")
    document = {"contract": "multibody-model", "contract_version": CONTRACT_VERSION,
        "kind": "model", "name": model.name,
        "units": {"length": "m", "mass": "kg", "time": "s", "angle": "rad"},
        "bodies": graph["bodies"], "joints": graph["joints"], "elements": graph["elements"],
        "markers": [{"name": row["name"], "body": row["body"], "point": row["point"]} for row in graph["frames"]]}
    for key in ("tires", "gravity", "road", "blobs", "function_programs", "couplers", "gauges", "capabilities"):
        if key in graph:
            document[key] = graph[key]
    # The existing native flag enables advanced force primitives, independent
    # of the assembly's authoring role or the selected study.
    if any(row["model"] != "native_brush" for row in document.get("tires", ())) or any(
        row["type"] in {"steering_actuator", "aerodynamic_drag"}
        for row in document["elements"]
    ):
        document["capabilities"] = sorted(set(document.get("capabilities", ())) | {"vehicle"})
    validate_model(document)
    return document


def _check_input_references(model: dict[str, Any], case: dict[str, Any], payload: bytes) -> None:
    bodies = {row["name"] for row in model["bodies"]}
    tires = {row["name"] for row in model.get("tires", ())}
    joints = {row["name"]: row for row in model["joints"]}
    driven = {row.get("target", row["name"]) for row in joints.values() if row["type"].startswith("driven_")}
    actuators = {
        row.get("target", row["name"]) for row in model.get("elements", ())
        if row.get("type") == "steering_actuator"
    }
    for row in case.get("blobs", ()):
        start, length = row.get("offset", 0), row.get("length", 0)
        if start < 0 or length < 0 or start+length > len(payload):
            raise ValueError(f"input {row.get('name')!r}: blob range falls outside payload")
        for field, known in (("body", bodies), ("tire", tires), ("coordinate", driven), ("actuator", actuators)):
            if field in row and row[field] not in known:
                raise ValueError(f"input {row.get('name')!r}: unknown {field} {row[field]!r}")
    family = case["family"]
    if family == "handling":
        rows = case.get("handling", {}).get("steering", ())
        if not rows or any(row.get("actuator") not in actuators for row in rows):
            raise ValueError("handling requires explicitly declared steering actuators")
    for section, key in (("four_post", "corners"), ("ride_random_road", "wheels")):
        if section in case:
            rows = case[section].get(key, ())
            if not rows or any(row.get("tire") not in tires for row in rows):
                raise ValueError(f"{section} requires explicitly declared tire IDs")
    for axis in case.get("k", {}).get("axes", ()):
        if axis["coordinate"] not in driven:
            raise ValueError(f"unknown grid coordinate {axis['coordinate']!r}")
    axis_map = case.get("k", {}).get("axis_map", {})
    names = [*axis_map.get("wheel", ())]
    if "rack" in axis_map:
        names.append(axis_map["rack"])
    unknown = [name for name in names if name not in driven]
    if unknown:
        raise ValueError(f"grid axis_map names unknown driven coordinates: {unknown}")


def compile_resolved(model: ResolvedModel, plan: ResolvedSolvePlan, *, request: SimulationRequest | None = None) -> CompiledSimulation:
    """Emit one native submission from the resolved model and run plan."""
    if not isinstance(model, ResolvedModel) or not isinstance(plan, ResolvedSolvePlan):
        raise TypeError("compiler requires ResolvedModel and ResolvedSolvePlan")
    run = plan.to_document()
    family = run.get("protocol", "vehicle_dynamic" if run["study"] == "dynamic" else "kc_quasi_static")
    if family in _GRID_PROTOCOLS and run["study"] == "dynamic":
        raise ValueError("grid protocol cannot perform a dynamic study")
    if family not in _GRID_PROTOCOLS and run["study"] != "dynamic":
        raise ValueError("time-history protocol requires a dynamic study")
    motion = resolve_motion_boundaries(model, plan)
    document = native_model_document(motion.model)
    elements = {row["name"] for key in ("elements", "tires") for row in document.get(key, ())}
    activation = {}
    for row in run.get("element_activation", ()):
        entity = row["entity"]
        if entity not in elements:
            raise ValueError(f"activation names an unknown force element {entity!r}")
        if entity in activation:
            raise ValueError(f"duplicate activation for force element {entity!r}")
        activation[entity] = row["active"]
    for key in ("elements", "tires"):
        if key in document:
            document[key] = [row for row in document[key] if activation.get(row["name"], True)]
    for row in document.get("blobs", ()):
        if row["offset"] < 0 or row["offset"]+row["length"] > len(model.resource_payload):
            raise ValueError(f"model resource {row['name']!r} falls outside pinned payload")
    case = {"contract": "multibody-case", "contract_version": CONTRACT_VERSION,
        "kind": "case", "family": family, "name": run["name"], "solver": run["solver"],
        **run.get("excitation", {})}
    case["outputs"] = {"constraint_reaction": True, "element_wrench": True}
    if any(row["type"] == "roll_center" for row in model.to_document().get("measurements", ())):
        case["outputs"]["constraint_jacobian"] = True
    payload = bytearray(plan.input_payload)
    descriptors = case.setdefault("blobs", [])
    occupied = {row["name"] for row in descriptors if "name" in row}

    def append(name: str, role: str, values: Any, **target: str) -> None:
        if name in occupied:
            raise ValueError(f"duplicate input ID {name!r}")
        array = np.asarray(values, dtype="<f8")
        if not np.isfinite(array).all():
            raise ValueError(f"input {name!r} contains nonfinite values")
        descriptors.append({"name": name, "role": role, **target, "offset": len(payload),
            "length": array.nbytes, "shape": list(array.shape), "dtype": "float64", "order": "C"})
        occupied.add(name)
        payload.extend(array.tobytes())

    samples = np.asarray(run["samples"], dtype=float)
    if len(samples) < 2:
        raise ValueError("native run requires at least two output samples")
    differences = np.diff(samples)
    if np.allclose(differences, differences[0], atol=1e-14, rtol=1e-12):
        case["time"] = {"start_s": float(samples[0]), "end_s": float(samples[-1]), "step_s": float(differences[0])}
    else:
        append("solve-plan-samples", "sample_times", samples)
        case["time"] = {"samples": "solve-plan-samples"}
    if motion.targets:
        inputs = case.setdefault("inputs", {})
        if "motion" in inputs:
            raise ValueError("case excitation and boundaries both declare motion")
        inputs["motion"] = []
        for name, values in motion.targets.items():
            value_id, rate_id = f"{name}:motion_target", f"{name}:motion_rate"
            append(value_id, "motion_target", values, coordinate=name)
            append(rate_id, "motion_rate", motion.rates[name], coordinate=name)
            inputs["motion"].append({"coordinate": name, "blob": value_id, "rate_blob": rate_id})
    programs = {row.get("program") for row in run["boundaries"]}
    for row in run["inputs"]:
        if row["name"] in programs:
            continue
        if family not in _TABLE_PROTOCOLS:
            raise ValueError(f"protocol {family!r} does not support sampled input tables")
        if "values" not in row:
            raise ValueError(f"input {row['name']!r} requires sampled values")
        values = np.asarray(row["values"], dtype=float)
        role = row["role"]
        expected_shape = (len(samples), 6) if role == "body_wrench" else (len(samples),)
        if values.shape != expected_shape:
            raise ValueError(f"input {row['name']!r} must have shape {expected_shape}")
        append(row["name"], role, values, **{key: row[key] for key in ("body", "tire", "coordinate", "actuator", "quantity") if key in row})
    if not descriptors:
        case.pop("blobs")
    state = run.get("initial_state", {})
    body_rows = {row["name"]: row for row in document["bodies"]}
    for name, given in state.items():
        if name not in body_rows or set(given)-{"position", "quaternion", "velocity", "omega"}:
            raise ValueError(f"initial state names an unknown body or field: {name!r}")
        body_rows[name].update(given)
    if family in _GRID_PROTOCOLS and "c" in case:
        markers = {row["name"]: row["body"] for row in document["markers"]}
        names = [case["c"][key] for key in ("load_marker", "mirror_marker") if key in case["c"]]
        if any(name not in markers for name in names):
            raise ValueError("load marker is not a declared model frame")
        if len({markers[name] for name in names}) != len(names):
            raise ValueError("native loads support one application marker per body")
        document["body_wrench_markers"] = names
    validate_model(document)
    validate_case(case)
    _check_input_references(document, case, bytes(payload))
    from ..kernel.capabilities import (
        kernel_capability_document,
        require_function_program_version,
    )

    if document.get("function_programs"):
        require_function_program_version(1)
    if motion.targets and 1 not in kernel_capability_document().get("joint_coordinate_motion_versions", ()):
        raise ValueError("loaded kernel does not support joint coordinate motion version 1")
    if family not in kernel_capability_document().get("case_families", ()):
        raise ValueError(f"loaded kernel does not support case protocol {family!r}")
    if 1 not in kernel_capability_document().get("constraint_channel_versions", ()):
        raise ValueError("loaded kernel does not support constraint channel version 1")
    active = request or SimulationRequest(assembly="generic", family=family, study=run["study"], model=model, case=plan, name=run["name"])
    if active.family != family:
        raise ValueError("request family disagrees with compiled protocol")
    return CompiledSimulation(request=active, model_document=document, case_document=case,
        model_payload=pack_container(document, model.resource_payload), case_payload=pack_container(case, bytes(payload)),
        layout={"document_order": ["model", "case"], "payload_order": ["model", "case"]},
        metadata={"compiler": "ResolvedModelCompiler", "payload_schema": "contract_document_pair",
            "model_fingerprint": model.fingerprint, "resolved_fingerprint": motion.model.fingerprint,
            "provenance": model.to_document().get("provenance", {}),
            "resource_manifest": model.to_document().get("resource_manifest", []),
            "solve_plan": run, "family": family, "study": run["study"]})
