"""
Emit and run the `vehicle_dynamic` contract documents.

This is the full-vehicle counterpart of the axle family.  The model document
carries the assembled vehicle -- two suspensions, four wheels, a chassis, the
steering actuator and anything the model declares (springs, bushings, anti-roll
bars, aerodynamic drag) -- and the case document carries the excitation: the
steering history, the road, brake and wheel torque, body wrenches and the
static-trim gauge.

The setup below is the same sequence `vehicle_dynamics.run_vehicle_dynamics`
performs before it marshals anything, and it calls the same builders.  What
changes is the destination: instead of filling a ctypes structure, the model
becomes a document and the kernel reads it.  The two paths are held together by
a bit-exact comparison test rather than by discipline.

The document declares metres.  The native vehicle model is already SI, so every
conversion factor is one and the arithmetic is exact; a millimetre round trip
would cost an ulp per value and the frozen hashes would not survive it.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..axle_dynamics.schema import (
    PAC2002_PARAMETER_DEFAULTS,
    PAC2002_PARAMETER_NAMES,
)
from ..kernel.solver import solver_settings_document
from ..schema import VehicleDynamicCase, VehicleModel
from ..vehicle_dynamics import prepare_vehicle_run

__all__ = [
    "case_document",
    "model_document",
    "prepare_vehicle_run",
]

_UNITS = {"length": "m", "mass": "kg", "time": "s", "angle": "rad"}

#: The ABI's steering actuator type, as the document spells it.
_ACTUATOR_NAMES = {
    0: "translation",
    1: "rotation",
    2: "prescribed_rotation",
    3: "prescribed_translation",
}

#: The kernel's tire kind, as the contract's `model` spells it.  The two PAC2002
#: variants differ in where their coefficients came from, which is a document
#: field of its own rather than a distinct model name.
TIRE_MODEL_NAMES = {0: "native_brush", 1: "pac2002", 2: "pac2002", 3: "fiala"}

#: The ABI's road kind, as the document spells it.
_ROAD_NAMES = {
    0: "plane",
    1: "plane",
    2: "sine",
    3: "bump",
    4: "random_fourier",
    5: "four_post",
}


def _vec3(values) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=float).reshape(-1)[:3]]


def _quaternion(values) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=float).reshape(-1)[:4]]


def _joint_entry(joint) -> dict[str, Any]:
    entry = {
        "name": joint.name,
        "type": "convel" if joint.kind == "constant_velocity" else joint.kind,
        "body_a": joint.body_a,
        "body_b": joint.body_b,
        "point_a": _vec3(joint.point_a_m),
        "point_b": _vec3(joint.point_b_m),
        "axis_a": _vec3(joint.axis_a),
        "axis_b": _vec3(joint.axis_b),
    }
    if joint.kind == "constant_velocity":
        entry.update(
            {
                "axis_a_secondary": _vec3(joint.axis_a_secondary),
                "axis_b_secondary": _vec3(joint.axis_b_secondary),
                "convel_angle_target": float(joint.constant_velocity_angle_target),
            }
        )
    return entry


def _spring_element(spring) -> dict[str, Any]:
    parameters: dict[str, Any] = {
        "point_a": _vec3(spring.point_a_m),
        "point_b": _vec3(spring.point_b_m),
        "stiffness": float(spring.stiffness_n_per_m),
        "compression_damping": float(spring.compression_damping_n_s_per_m),
        "rebound_damping": float(spring.rebound_damping_n_s_per_m),
        "free_length": float(spring.free_length_m),
        "compression_stop_stiffness": float(spring.compression_stop_stiffness_n_per_m),
        "compression_stop_damping": float(spring.compression_stop_damping_n_s_per_m),
        "rebound_stop_stiffness": float(spring.rebound_stop_stiffness_n_per_m),
        "rebound_stop_damping": float(spring.rebound_stop_damping_n_s_per_m),
    }
    if spring.minimum_length_m is not None:
        parameters["minimum_length"] = float(spring.minimum_length_m)
    if spring.maximum_length_m is not None:
        parameters["maximum_length"] = float(spring.maximum_length_m)
    if spring.damper_curve_velocity_m_per_s:
        parameters["damper_curve"] = [
            [float(velocity), float(force)]
            for velocity, force in zip(
                spring.damper_curve_velocity_m_per_s, spring.damper_curve_force_n
            )
        ]
    if spring.elastic_curve_deflection_m:
        parameters["elastic_curve"] = [
            [float(deflection), float(force)]
            for deflection, force in zip(
                spring.elastic_curve_deflection_m, spring.elastic_curve_force_n
            )
        ]
    if spring.compression_stop_curve_penetration_m:
        parameters["compression_stop_curve"] = [
            [float(penetration), float(force)]
            for penetration, force in zip(
                spring.compression_stop_curve_penetration_m,
                spring.compression_stop_curve_force_n,
            )
        ]
    if spring.rebound_stop_curve_penetration_m:
        parameters["rebound_stop_curve"] = [
            [float(penetration), float(force)]
            for penetration, force in zip(
                spring.rebound_stop_curve_penetration_m,
                spring.rebound_stop_curve_force_n,
            )
        ]
    return {
        "name": spring.name,
        "type": "spring_damper",
        "body_a": spring.body_a,
        "body_b": spring.body_b,
        "parameters": parameters,
    }


def _bushing_element(bushing) -> dict[str, Any]:
    parameters: dict[str, Any] = {
            "point_a": _vec3(bushing.point_a_m),
            "point_b": _vec3(bushing.point_b_m),
            "frame_a_quaternion": _quaternion(bushing.frame_a_to_body_quaternion),
            "frame_b_quaternion": _quaternion(bushing.frame_b_to_body_quaternion),
            "reference_translation": _vec3(bushing.reference_translation_in_frame_a_m),
            "reference_quaternion": _quaternion(bushing.reference_quaternion_a_to_b),
            "rotation_coordinates": bushing.rotation_coordinates,
            "stiffness": [
                [float(value) for value in row]
                for row in np.asarray(bushing.stiffness, dtype=float)
            ],
            "damping": [
                [float(value) for value in row]
                for row in np.asarray(bushing.damping, dtype=float)
            ],
            # A bushing preload is six components -- three forces and three
            # moments -- and truncating it to three reads as a malformed
            # element on the kernel side rather than as a silent loss.
            "preload": [
                float(value)
                for value in np.asarray(bushing.preload_in_frame_a_n_n_m, dtype=float)[:6]
            ],
    }
    if bushing.force_curves:
        parameters["force_curves"] = [
            [[float(x), float(y)] for x, y in curve]
            for curve in bushing.force_curves
        ]
        parameters["force_curve_interpolation"] = bushing.force_curve_interpolation
    return {
        "name": bushing.name,
        "type": "bushing",
        "body_a": bushing.body_a,
        "body_b": bushing.body_b,
        "parameters": parameters,
    }


def _is_right_tire_name(name: str) -> bool:
    """Return whether a tire name denotes the right-hand side of a vehicle."""
    normalized = name.strip().lower().replace("-", "_")
    return normalized.endswith(("_right", "_r", "right"))


def tire_model_arrays(
    model,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return the per-tire model kind, parameter block and mirror flag."""
    tire_model_kind = np.ascontiguousarray(
        [
            (
                (
                    2
                    if (
                        tire.model_kind == "pac2002_pure_slip"
                        and getattr(tire, "pac2002_parameter_source", "user")
                        == "adams_builtin"
                    )
                    else 1
                    if tire.model_kind == "pac2002_pure_slip"
                    else 3
                    if tire.model_kind == "fiala"
                    else 0
                )
            )
            for tire in model.tires
        ],
        dtype=np.int32,
    )
    tire_pac2002_parameters = np.ascontiguousarray(
        [
            [
                float(
                    tire.pac2002_coefficients.get(
                        name, PAC2002_PARAMETER_DEFAULTS[name]
                    )
                )
                for name in PAC2002_PARAMETER_NAMES
            ]
            for tire in model.tires
        ],
        dtype=np.float64,
    )
    for i, tire in enumerate(model.tires):
        if tire.model_kind == "fiala":
            fiala = tire.fiala_parameters
            fiala_values = (
                fiala.get("CSLIP", 1000.0),
                fiala.get("CALPHA", 800.0),
                fiala.get("CGAMMA", 0.0),
                0.0,
                fiala.get("USE_MODE", 2.0),
                fiala.get("UMIN", 0.9),
                fiala.get("UMAX", 1.0),
                fiala.get("RELAX_LENGTH_X", tire.longitudinal_relaxation_length_m),
                fiala.get("RELAX_LENGTH_Y", tire.lateral_relaxation_length_m),
                fiala.get("WIDTH", 0.235),
                fiala.get("ROLLING_RESISTANCE", 0.0),
                fiala.get("LOW_SPEED_THRESHOLD", 1.0e-3),
                0.0,
                0.0,
            )
            tire_pac2002_parameters[i, : len(fiala_values)] = fiala_values
    tire_pac2002_mirror = np.ascontiguousarray(
        [
            int(
                tire.model_kind == "pac2002_pure_slip"
                and getattr(tire, "pac2002_parameter_source", "user")
                == "adams_builtin"
                and (
                    (
                        tire.pac2002_mirror
                        if tire.pac2002_mirror is not None
                        else _is_right_tire_name(tire.name)
                    )
                    or tire.pac2002_coefficients.get("USE_MODE", 14.0) < 0.0
                )
            )
            for tire in model.tires
        ],
        dtype=np.int32,
    )
    return tire_model_kind, tire_pac2002_parameters, tire_pac2002_mirror


def _tire_entry(tire, index: int, kind: int, mirror: int) -> dict[str, Any]:
    model = TIRE_MODEL_NAMES.get(int(kind))
    if model is None:
        raise NotImplementedError(f"unknown native tire model kind {kind!r}")
    parameters: dict[str, Any] = {
        "center_local": _vec3(tire.center_local_m),
        "spin_axis_local": _vec3(tire.spin_axis_local),
        "forward_axis_local": _vec3(tire.forward_axis_local),
        "unloaded_radius": float(tire.unloaded_radius_m),
        "maximum_compression": float(tire.maximum_compression_m),
        "vertical_stiffness": float(tire.vertical_stiffness_n_per_m),
        "vertical_damping": float(tire.vertical_damping_n_s_per_m),
        "longitudinal_friction_coefficient": float(tire.longitudinal_friction_coefficient),
        "lateral_friction_coefficient": float(tire.lateral_friction_coefficient),
        "longitudinal_brush_stiffness": float(tire.longitudinal_brush_stiffness_n_per_m),
        "lateral_brush_stiffness": float(tire.lateral_brush_stiffness_n_per_m),
        "longitudinal_relaxation_length": float(tire.longitudinal_relaxation_length_m),
        "lateral_relaxation_length": float(tire.lateral_relaxation_length_m),
        "detached_relaxation_s": float(tire.detached_relaxation_s),
    }
    # The frame is what the tire's axes are expressed in, and it is not always
    # the body its force acts on: a wheel spins on a carrier that does not.
    if tire.frame_body is not None:
        parameters["frame_body"] = tire.frame_body
        parameters["frame_center_local"] = _vec3(tire.frame_center_local_m)
    else:
        parameters["frame_center_local"] = parameters["center_local"]
    if tire.drive_torque_body is not None:
        parameters["drive_torque_body"] = tire.drive_torque_body
        if tire.drive_torque_reaction_body is not None:
            parameters["drive_torque_reaction_body"] = tire.drive_torque_reaction_body
        parameters["drive_torque_axis_local"] = _vec3(tire.drive_torque_axis_local)
    # The parameter vector is big and its slots have no useful individual
    # names, so it travels in the model's payload; the tire names the
    # descriptor that describes it.  The same holds for a measured vertical
    # table, which is a list of pairs rather than a scalar.
    parameters["blob"] = f"tire-parameters-{index}"
    parameters["parameter_source"] = "adams_builtin" if int(kind) == 2 else "user"
    parameters["mirror"] = bool(mirror)
    tables = {
        "deflection_curve": tire.pac2002_tables.get("deflection_load_curve"),
        "bottoming_curve": tire.pac2002_tables.get("bottoming_curve"),
    }
    for field, rows in tables.items():
        if rows:
            parameters[field] = f"tire-{field.replace('_', '-')}-{index}"
    return {"name": tire.name, "model": model, "body": tire.body, "parameters": parameters}


def _steering_element(steering, index: int, body_names: tuple[str, ...]) -> dict[str, Any]:
    parameters: dict[str, Any] = {
        "type": _ACTUATOR_NAMES[int(steering.actuator_type[index])],
        "body": body_names[int(steering.body[index])],
        "point_local": _vec3(steering.point_local[index]),
        "reaction_point_local": _vec3(steering.reaction_point_local[index]),
        "axis_local": _vec3(steering.axis_local[index]),
        "reference_quaternion": _quaternion(steering.reference_quaternion[index]),
        "stiffness": float(steering.stiffness[index]),
        "damping": float(steering.damping[index]),
    }
    # A negative reaction body is the documented "no reaction body" case, so it
    # is absent from the document rather than a name that does not exist.
    if int(steering.reaction_body[index]) >= 0:
        parameters["reaction_body"] = body_names[int(steering.reaction_body[index])]
    return {
        "name": f"steering_{steering.names[index]}",
        "type": "steering_actuator",
        "target": steering.names[index],
        "parameters": parameters,
    }


def _append_tire_table(
    blob: bytearray, descriptors: list[dict[str, Any]], name: str, values
) -> None:
    """Append one tire-side table and describe where it landed."""
    array = np.ascontiguousarray(values, dtype=np.float64)
    offset = len(blob)
    blob.extend(array.tobytes())
    descriptors.append(
        {
            "name": name,
            "offset": offset,
            "length": array.size * array.dtype.itemsize,
            "dtype": "float64",
            "shape": [int(extent) for extent in array.shape],
            "order": "C",
        }
    )


def model_document(
    model: VehicleModel, prepared, *, name: str | None = None
) -> tuple[dict[str, Any], bytes]:
    """
    Describe the assembled vehicle as a multibody model document.

    Returns the document and the payload its own descriptors point into.  A tire
    model with a parameter vector needs the second half: the vector is far too
    big for the JSON, and the contract already has the vocabulary for it.
    """
    native = prepared.native_model
    kinds, parameter_blocks, mirrors = tire_model_arrays(native)
    blob = bytearray()
    descriptors: list[dict[str, Any]] = []
    for index in range(len(native.tires)):
        block = np.ascontiguousarray(parameter_blocks[index], dtype=np.float64)
        _append_tire_table(blob, descriptors, f"tire-parameters-{index}", block)
        tire = native.tires[index]
        for field, key in (
            ("deflection-curve", "deflection_load_curve"),
            ("bottoming-curve", "bottoming_curve"),
        ):
            rows = tire.pac2002_tables.get(key)
            if rows:
                _append_tire_table(
                    blob,
                    descriptors,
                    f"tire-{field}-{index}",
                    np.asarray(rows, dtype=np.float64).reshape(-1, 2),
                )
    elements = [_spring_element(spring) for spring in native.springs]
    elements.extend(_bushing_element(bushing) for bushing in native.bushings)
    elements.extend(
        {
            "name": f"aero_{index}",
            "type": "aerodynamic_drag",
            "body_a": drag.body,
            "parameters": {
                "application_point": _vec3(drag.application_point_m),
                "forward_axis": _vec3(drag.forward_axis_local),
                "coefficient": float(drag.coefficient_n_s2_per_m2),
            },
        }
        for index, drag in enumerate(native.aerodynamic_drags)
    )
    elements.extend(
        _steering_element(prepared.steering, index, prepared.body_names)
        for index in range(len(prepared.steering.names))
    )
    document: dict[str, Any] = {
        "contract": "multibody-model",
        "contract_version": 1,
        "kind": "model",
        "name": name or native.name,
        "units": dict(_UNITS),
        "gravity": [float(value) for value in native.gravity_m_per_s2],
        "capabilities": ["vehicle"],
        "bodies": [
            {
                "name": body.name,
                "mass": float(body.mass_kg),
                "inertia": [
                    [float(value) for value in row]
                    for row in np.asarray(body.inertia_kg_m2, dtype=float)
                ],
                "fixed": bool(body.fixed),
                "position": _vec3(body.position_m),
                "quaternion": _quaternion(body.quaternion_body_to_world),
                "velocity": _vec3(body.linear_velocity_m_per_s),
                "omega": _vec3(body.angular_velocity_rad_per_s),
            }
            for body in native.bodies
        ],
        "joints": [_joint_entry(joint) for joint in native.joints],
        "elements": elements,
        # A tire is not an element: it carries its own model kind and parameter
        # block, which is the one place the contract has a distinct vocabulary
        # for it.
        "tires": [
            _tire_entry(tire, index, int(kinds[index]), int(mirrors[index]))
            for index, tire in enumerate(native.tires)
        ],
        "blobs": descriptors,
    }
    if native.coordinate_couplers:
        document["couplers"] = [
            {
                "name": coupler.name,
                "joint_a": coupler.joint_a,
                "coordinate_a": coupler.coordinate_a,
                "scale_a": float(coupler.scale_a),
                "joint_b": coupler.joint_b,
                "coordinate_b": coupler.coordinate_b,
                "scale_b": float(coupler.scale_b),
            }
            for coupler in native.coordinate_couplers
        ]
    if prepared.static_rotation_gauges:
        document["gauges"] = [
            {"body": body, "axis_local": _vec3(axis)}
            for body, axis in prepared.static_rotation_gauges
        ]
    return document, bytes(blob)


def _append_table(
    blob: bytearray, tables: list[dict[str, Any]], values, *, role: str, **extra: Any
) -> None:
    array = np.ascontiguousarray(values, dtype=np.float64)
    offset = len(blob)
    blob.extend(array.tobytes())
    tables.append(
        {
            "name": role,
            "role": role,
            "offset": offset,
            "length": array.size * array.dtype.itemsize,
            "dtype": "float64",
            "shape": [int(extent) for extent in array.shape],
            **extra,
        }
    )


def _solver_block(settings) -> dict[str, Any]:
    """Compatibility wrapper for the shared neutral solver serializer."""
    return solver_settings_document(settings)


def case_document(
    model: VehicleModel, case: VehicleDynamicCase, prepared, *, name: str | None = None
) -> tuple[dict[str, Any], bytes]:
    """Describe one excitation history, returning the document and its blob."""
    times = np.asarray(prepared.times, dtype=float)
    if times.size < 2:
        raise ValueError("a dynamic case needs at least two sample times")
    steps = np.diff(times)
    uniform = bool(np.allclose(steps, steps[0], rtol=0.0, atol=1e-15))

    blob = bytearray()
    tables: list[dict[str, Any]] = []
    time_entry: dict[str, Any] = {
        "start_s": float(times[0]),
        "end_s": float(times[-1]),
        # Uniform grids are rules; irregular histories carry their exact sample
        # instants in the case payload so the kernel does not re-derive them.
        "step_s": float(steps[0] if uniform else np.min(steps)),
    }
    if not uniform:
        _append_table(blob, tables, times, role="sample_times")
        time_entry["samples"] = "sample_times"
    for tire in prepared.native_model.tires:
        height = prepared.road_height.get(tire.name)
        if height is not None:
            _append_table(blob, tables, height, role="road_height", tire=tire.name)
        velocity = prepared.road_velocity.get(tire.name)
        if velocity is not None:
            _append_table(blob, tables, velocity, role="road_velocity", tire=tire.name)
        torque = prepared.wheel_torque.get(tire.name)
        if torque is not None:
            _append_table(blob, tables, torque, role="wheel_torque", tire=tire.name)
        brake = prepared.brake_torque.get(tire.name)
        if brake is not None:
            _append_table(blob, tables, brake, role="brake_torque", tire=tire.name)
    # The steering buffers are flat `sample_count * actuator_count` tables in
    # the ABI, so a column is a stride through them rather than a slice.
    actuator_count = len(prepared.steering.names)
    sample_count = times.size
    target_columns = np.asarray(prepared.steering.target, dtype=float).reshape(
        sample_count, actuator_count
    )
    rate_columns = np.asarray(prepared.steering.target_rate, dtype=float).reshape(
        sample_count, actuator_count
    )
    for index, actuator in enumerate(prepared.steering.names):
        _append_table(
            blob,
            tables,
            target_columns[:, index],
            role="steering_target",
            actuator=actuator,
            quantity="translation"
            if int(prepared.steering.actuator_type[index]) in (0, 3)
            else "rotation",
        )
        _append_table(
            blob,
            tables,
            rate_columns[:, index],
            role="steering_rate",
            actuator=actuator,
            quantity="translation"
            if int(prepared.steering.actuator_type[index]) in (0, 3)
            else "rotation",
        )

    inputs: dict[str, Any] = {
        "initial_state_angle_tolerance": prepared.initial_state_angle_tolerance,
        "road": {
            "kind": _ROAD_NAMES[prepared.road.kind],
            "parameters": {
                "origin_x": float(prepared.road.origin_x),
                "origin_z": float(prepared.road.origin_z),
                "amplitude": float(prepared.road.amplitude),
                "wavelength": float(prepared.road.wavelength),
                "phase": float(prepared.road.phase),
                "bump_start": float(prepared.road.bump_start),
                "bump_length": float(prepared.road.bump_length),
                "corner_scale": [float(value) for value in prepared.road.corner_scale[:4]],
            },
        },
    }
    if prepared.static_gauge_body is not None:
        inputs["static_gauge"] = {
            "body": prepared.static_gauge_body,
            "dof_mask": int(prepared.static_gauge_dof_mask),
            "trim_then_release": bool(prepared.trim_then_release),
        }

    document: dict[str, Any] = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "vehicle_dynamic",
        "name": name or case.name,
        "time": time_entry,
        "solver": _solver_block(prepared.solver),
        "inputs": inputs,
        "blobs": tables,
    }
    return document, bytes(blob)
