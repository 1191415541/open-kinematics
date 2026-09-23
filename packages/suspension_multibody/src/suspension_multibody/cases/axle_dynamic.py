"""
Emit and run the `axle_dynamic` contract documents.

The dynamic axle family is a single time history rather than a sweep, so the
model document carries the full element set -- bodies, joints, springs,
bushings and tires -- and the case document carries the sampled inputs the
kernel interpolates.  Those inputs are large, so they travel in the container's
blob and the document describes them by role, target and byte range.

The family's model and case objects are already SI, so the document declares
metres and nothing is converted.  That is a deliberate choice rather than
laziness: a length written in millimetres and read back is a multiply and a
divide by a number that is not a power of two, and the round trip costs an ulp.
A run that has to reproduce a frozen byte-for-byte hash cannot afford that, and
a unit conversion that is not needed is a rounding error waiting to happen.  The
kernel honours the declared unit -- a document that says `m` scales by one.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..axle_dynamics.schema import AxleDynamicsCase, AxleDynamicsModel
from ..joints import document_type_for_schema_kind as _joint_document_type
from ..kernel.solver import solver_settings_document

__all__ = [
    "case_document",
    "model_document",
]

#: The document is written in the units the objects already use, so every
#: conversion factor below is one and the arithmetic is exact.  They are named
#: rather than inlined because the kernel decomposes a millimetre document the
#: same way, and keeping the shape makes that correspondence visible.
_MM = 1.0
_INV_MM = 1.0
_INERTIA = 1.0

_UNITS = {"length": "m", "mass": "kg", "time": "s", "angle": "rad"}


def _vec3(values) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=float)[:3]]


def _vec3_mm(values) -> list[float]:
    return [float(value) * _INV_MM for value in np.asarray(values, dtype=float)[:3]]


def _quaternion(values) -> list[float]:
    return [float(value) for value in np.asarray(values, dtype=float)[:4]]


def _matrix(values) -> list[list[float]]:
    return [[float(value) for value in row] for row in np.asarray(values, dtype=float)]


def _spring_element(spring) -> dict[str, Any]:
    """One spring/damper, with its force-per-length coefficients in N/mm."""
    parameters: dict[str, Any] = {
        "point_a": _vec3_mm(spring.point_a_m),
        "point_b": _vec3_mm(spring.point_b_m),
        "stiffness": float(spring.stiffness_n_per_m) * _MM,
        "compression_damping": float(spring.compression_damping_n_s_per_m) * _MM,
        "rebound_damping": float(spring.rebound_damping_n_s_per_m) * _MM,
        "free_length": float(spring.free_length_m) * _INV_MM,
        "compression_stop_stiffness": float(spring.compression_stop_stiffness_n_per_m) * _MM,
        "compression_stop_damping": float(spring.compression_stop_damping_n_s_per_m) * _MM,
        "rebound_stop_stiffness": float(spring.rebound_stop_stiffness_n_per_m) * _MM,
        "rebound_stop_damping": float(spring.rebound_stop_damping_n_s_per_m) * _MM,
    }
    # A limit that is not set is genuinely absent, not zero: zero would clamp
    # the spring solid.
    if spring.minimum_length_m is not None:
        parameters["minimum_length"] = float(spring.minimum_length_m) * _INV_MM
    if spring.maximum_length_m is not None:
        parameters["maximum_length"] = float(spring.maximum_length_m) * _INV_MM
    if spring.damper_curve_velocity_m_per_s:
        parameters["damper_curve"] = [
            [float(velocity) * _INV_MM, float(force)]
            for velocity, force in zip(
                spring.damper_curve_velocity_m_per_s, spring.damper_curve_force_n
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
    """
    One bushing, with its 6x6 coefficient blocks written in the document units.

    The kernel stores a translational coefficient per metre and a rotational one
    per radian, so putting them back into a millimetre document means dividing
    the translational block and multiplying the rotational one.  The two
    coupling blocks already read as force per radian and moment per metre, so
    they do not move.
    """
    stiffness = np.asarray(bushing.stiffness, dtype=float).copy()
    damping = np.asarray(bushing.damping, dtype=float).copy()
    for row in range(6):
        for column in range(6):
            if row < 3 and column < 3:
                scale = _MM
            elif row >= 3 and column >= 3:
                scale = _INV_MM
            else:
                scale = 1.0
            stiffness[row, column] *= scale
            damping[row, column] *= scale
    preload = np.asarray(bushing.preload_in_frame_a_n_n_m, dtype=float).copy()
    preload[3:] *= _INV_MM
    return {
        "name": bushing.name,
        "type": "bushing",
        "body_a": bushing.body_a,
        "body_b": bushing.body_b,
        "parameters": {
            "point_a": _vec3_mm(bushing.point_a_m),
            "point_b": _vec3_mm(bushing.point_b_m),
            "frame_a_quaternion": _quaternion(bushing.frame_a_to_body_quaternion),
            "frame_b_quaternion": _quaternion(bushing.frame_b_to_body_quaternion),
            "reference_translation": _vec3_mm(bushing.reference_translation_in_frame_a_m),
            "reference_quaternion": _quaternion(bushing.reference_quaternion_a_to_b),
            "stiffness": _matrix(stiffness),
            "damping": _matrix(damping),
            "preload": [float(value) for value in preload[:6]],
        },
    }


def _anti_roll_element(bar) -> dict[str, Any]:
    """One anti-roll bar, as the element the kernel couples two bodies with."""
    return {
        "name": bar.name,
        "type": "anti_roll_bar",
        "body_a": bar.body_a,
        "body_b": bar.body_b,
        "parameters": {
            "axis_a": _vec3(bar.axis_a),
            "reference_quaternion": _quaternion(bar.reference_quaternion_a_to_b),
            "stiffness": float(bar.stiffness_n_m_per_rad) * _MM,
            "damping": float(bar.damping_n_m_s_per_rad) * _MM,
        },
    }


def _aerodynamic_element(drag, index: int) -> dict[str, Any]:
    """One quadratic drag element, applied at a point of its own body."""
    return {
        "name": f"aero_{index}",
        "type": "aerodynamic_drag",
        "body_a": drag.body,
        "parameters": {
            "application_point": _vec3_mm(drag.application_point_m),
            "forward_axis": _vec3(drag.forward_axis_local),
            "coefficient": float(drag.coefficient_n_s2_per_m2),
        },
    }


def _tire_entry(tire, index: int, kind: int, mirror: int) -> dict[str, Any]:
    """
    One tire: its geometry, its model kind and the block its parameters travel in.

    A brush tire's coefficients fit in the JSON.  A PAC2002 or Fiala parameter
    vector does not, and the contract already has the vocabulary for that: the
    tire names a descriptor and the numbers ride in the model's payload, exactly
    as the vehicle family writes them.  The parameter vector is the same one
    either way, which is why both families call `tire_model_arrays`.
    """
    from .vehicle_dynamic import TIRE_MODEL_NAMES

    model = TIRE_MODEL_NAMES.get(int(kind))
    if model is None:
        raise NotImplementedError(f"unknown native tire model kind {kind!r}")
    parameters: dict[str, Any] = {
        "center_local": _vec3_mm(tire.center_local_m),
        "spin_axis_local": _vec3(tire.spin_axis_local),
        "forward_axis_local": _vec3(tire.forward_axis_local),
        "unloaded_radius": float(tire.unloaded_radius_m) * _INV_MM,
        "maximum_compression": float(tire.maximum_compression_m) * _INV_MM,
        "vertical_stiffness": float(tire.vertical_stiffness_n_per_m) * _MM,
        "vertical_damping": float(tire.vertical_damping_n_s_per_m) * _MM,
        "longitudinal_friction_coefficient": float(
            tire.longitudinal_friction_coefficient
        ),
        "lateral_friction_coefficient": float(tire.lateral_friction_coefficient),
        "longitudinal_brush_stiffness": float(
            tire.longitudinal_brush_stiffness_n_per_m
        )
        * _MM,
        "lateral_brush_stiffness": float(tire.lateral_brush_stiffness_n_per_m) * _MM,
        "longitudinal_relaxation_length": float(tire.longitudinal_relaxation_length_m)
        * _INV_MM,
        "lateral_relaxation_length": float(tire.lateral_relaxation_length_m) * _INV_MM,
        "detached_relaxation_s": float(tire.detached_relaxation_s),
    }
    # The frame is what the tire's axes are expressed in, and it is not always
    # the body its force acts on: a wheel spins on a carrier that does not.
    if tire.frame_body is not None:
        parameters["frame_body"] = tire.frame_body
        parameters["frame_center_local"] = _vec3_mm(tire.frame_center_local_m)
    else:
        parameters["frame_center_local"] = parameters["center_local"]
    if tire.drive_torque_body is not None:
        parameters["drive_torque_body"] = tire.drive_torque_body
        if tire.drive_torque_reaction_body is not None:
            parameters["drive_torque_reaction_body"] = tire.drive_torque_reaction_body
        parameters["drive_torque_axis_local"] = _vec3(tire.drive_torque_axis_local)
    # The parameter vector travels in the model's payload; the tire names the
    # descriptor that describes it.  The same holds for a measured vertical
    # table, which is a list of pairs rather than a scalar.
    parameters["blob"] = f"tire-parameters-{index}"
    parameters["parameter_source"] = "adams_builtin" if int(kind) == 2 else "user"
    parameters["mirror"] = bool(mirror)
    for field, key in (
        ("deflection_curve", "deflection_load_curve"),
        ("bottoming_curve", "bottoming_curve"),
    ):
        if tire.pac2002_tables.get(key):
            parameters[field] = f"tire-{field.replace('_', '-')}-{index}"
    entry: dict[str, Any] = {
        "name": tire.name,
        "model": model,
        "body": tire.body,
        "parameters": parameters,
    }
    # The tire's own mass, when the model declares one (decision D2).  The field is
    # optional in the contract, so a tire that carries none emits nothing and the
    # document is byte-identical to what it was before the field existed -- which
    # is what keeps every recorded baseline valid.  The inertia rides in a 3x3 the
    # contract already accepts; without it the mass alone is still the right
    # statement, because the solver adds both to the carrying body.
    if float(tire.mass_kg) > 0.0:
        entry["mass"] = float(tire.mass_kg)
        inertia = tire.inertia_kg_m2
        if inertia is not None:
            entry["inertia"] = [
                [float(value) for value in row] for row in inertia
            ]
    return entry


def model_document(
    model: AxleDynamicsModel, *, name: str | None = None
) -> tuple[dict[str, Any], bytes]:
    """
    Describe an axle dynamics model as a multibody model document.

    Returns the document and the payload its own descriptors point into.  A tire
    whose law needs a parameter vector names a descriptor instead of spelling the
    vector out: it is far too big for the JSON, and the contract already has the
    vocabulary for it.
    """
    from .vehicle_dynamic import tire_model_arrays

    # One encoding of "how the kernel understands a tire model", shared with the
    # vehicle family: the kind number, the parameter order and the mirror rule
    # cannot drift between the two boundaries if there is only one of them.
    kinds, parameter_blocks, mirrors = tire_model_arrays(model)
    blob = bytearray()
    tables: list[dict[str, Any]] = []
    for index, tire in enumerate(model.tires):
        _append_model_table(
            blob, tables, f"tire-parameters-{index}", parameter_blocks[index]
        )
        # A measured vertical table replaces the stiffness polynomial, so a tire
        # that has one says so by naming its descriptor.
        for field, key in (
            ("deflection-curve", "deflection_load_curve"),
            ("bottoming-curve", "bottoming_curve"),
        ):
            rows = tire.pac2002_tables.get(key)
            if rows:
                _append_model_table(
                    blob,
                    tables,
                    f"tire-{field}-{index}",
                    np.asarray(rows, dtype=float).reshape(-1, 2),
                )
    bodies = [
        {
            "name": body.name,
            "mass": float(body.mass_kg),
            "inertia": _matrix(np.asarray(body.inertia_kg_m2, dtype=float) * _INERTIA),
            "fixed": bool(body.fixed),
            "position": _vec3_mm(body.position_m),
            "quaternion": _quaternion(body.quaternion_body_to_world),
            "velocity": _vec3(body.linear_velocity_m_per_s),
            "omega": _vec3(body.angular_velocity_rad_per_s),
        }
        for body in model.bodies
    ]
    joints = []
    for joint in model.joints:
        entry: dict[str, Any] = {
            "name": joint.name,
            "type": _joint_document_type(joint.kind),
            "body_a": joint.body_a,
            "body_b": joint.body_b,
            "point_a": _vec3_mm(joint.point_a_m),
            "point_b": _vec3_mm(joint.point_b_m),
            "axis_a": _vec3(joint.axis_a),
            "axis_b": _vec3(joint.axis_b),
        }
        joints.append(entry)
    for driven in model.driven_coordinates:
        # A driven coordinate is not a joint *row*: it prescribes one relative
        # degree of freedom and names the signal that drives it.  The reader
        # keeps it in a separate table (``driven_type_`` and friends), which is
        # why it is spelled with ``driven_`` types rather than as a prismatic or
        # revolute joint.  The axis goes in ``axis_b`` because the row measures
        # the separation along the axis in the *reaction* body's frame.
        joints.append(
            {
                "name": driven.name,
                "type": (
                    "driven_translation"
                    if driven.kind == "translation"
                    else "driven_rotation"
                ),
                "body_a": driven.body,
                "body_b": driven.reaction_body,
                "point_a": _vec3_mm(driven.point_local_m),
                "point_b": _vec3_mm(driven.reaction_point_local_m),
                "axis_b": _vec3(driven.axis_local),
                "reference_quaternion": _quaternion(driven.reference_quaternion),
                "target": driven.name,
            }
        )
    elements = [_spring_element(spring) for spring in model.springs]
    elements.extend(_bushing_element(bushing) for bushing in model.bushings)
    elements.extend(_anti_roll_element(bar) for bar in model.anti_roll_bars)
    elements.extend(
        _aerodynamic_element(drag, index)
        for index, drag in enumerate(model.aerodynamic_drags)
    )
    document: dict[str, Any] = {
        "contract": "multibody-model",
        "contract_version": 1,
        "kind": "model",
        "name": name or model.name,
        "units": dict(_UNITS),
        "gravity": [float(value) * _INV_MM for value in model.gravity_m_per_s2],
        "bodies": bodies,
        "joints": joints,
        "elements": elements,
        "tires": [
            _tire_entry(tire, index, int(kinds[index]), int(mirrors[index]))
            for index, tire in enumerate(model.tires)
        ],
    }
    if model.aerodynamic_drags or any(int(kind) != 0 for kind in kinds):
        # Two things are registered by the vehicle-level stage rather than by the
        # axle stage: every tire law other than the brush model (with its
        # parameter block, its measured curves and its contact frame) and the
        # aerodynamic drag elements.  Saying so is what makes the kernel read
        # them; leaving it out would silently run a model the document did not
        # describe.
        document["capabilities"] = ["vehicle"]
    if tables:
        document["blobs"] = tables
    return document, bytes(blob)


def _append_table(
    blob: bytearray, tables: list[dict[str, Any]], values, *, role: str, **extra: str
) -> None:
    """Append one contiguous float64 table and describe where it landed."""
    array = np.ascontiguousarray(values, dtype=np.float64)
    offset = len(blob)
    blob.extend(array.tobytes())
    tables.append(
        {
            "role": role,
            "offset": offset,
            "length": array.size * array.dtype.itemsize,
            "dtype": "float64",
            "shape": [int(extent) for extent in array.shape],
            **extra,
        }
    )


def _append_model_table(
    blob: bytearray, tables: list[dict[str, Any]], name: str, values
) -> None:
    """
    Append one table the *model* document points at.

    A model descriptor is found by name rather than by role: the reader looks up
    `tire-parameters-3`, not "the fourth parameters table".  The shape travels
    with it because the reader checks a parameter block's length and a curve's
    row count against it.
    """
    array = np.ascontiguousarray(values, dtype=np.float64)
    offset = len(blob)
    blob.extend(array.tobytes())
    tables.append(
        {
            "name": name,
            "offset": offset,
            "length": array.size * array.dtype.itemsize,
            "dtype": "float64",
            "shape": [int(extent) for extent in array.shape],
            "order": "C",
        }
    )


def case_document(
    model: AxleDynamicsModel, case: AxleDynamicsCase, *, name: str | None = None
) -> tuple[dict[str, Any], bytes]:
    """
    Describe one time history, returning the document and its payload blob.

    The blob is returned separately because the container is built by the
    boundary, not here: this module decides what the numbers are, and the shell
    decides how they are framed.
    """
    times = np.asarray(case.times_s, dtype=float)
    if times.size < 2:
        raise ValueError("a dynamic case needs at least two sample times")
    steps = np.diff(times)
    uniform = bool(np.allclose(steps, steps[0], rtol=0.0, atol=1e-15))

    blob = bytearray()
    tables: list[dict[str, Any]] = []
    time_entry: dict[str, Any] = {
        "start_s": float(times[0]),
        "end_s": float(times[-1]),
        # A uniform grid is a rule and the kernel reproduces it exactly.  An
        # irregular one is data: the average step is only a scale hint for
        # reporting, and the instants themselves travel in the payload so the
        # kernel reads them back rather than re-deriving them.
        "step_s": float(steps[0] if uniform else np.min(steps)),
    }
    if not uniform:
        _append_table(blob, tables, times, role="sample_times", name="sample_times")
        time_entry["samples"] = "sample_times"
    for tire in model.tires:
        height = case.road_height_m.get(tire.name)
        if height is not None:
            _append_table(
                blob,
                tables,
                np.asarray(height, dtype=float) * _INV_MM,
                role="road_height",
                tire=tire.name,
            )
        velocity = case.road_velocity_m_per_s.get(tire.name)
        if velocity is not None:
            _append_table(
                blob,
                tables,
                np.asarray(velocity, dtype=float) * _INV_MM,
                role="road_velocity",
                tire=tire.name,
            )
        torque = case.wheel_torque_n_m.get(tire.name)
        if torque is not None:
            _append_table(
                blob,
                tables,
                np.asarray(torque, dtype=float) * _INV_MM,
                role="wheel_torque",
                tire=tire.name,
            )
    for body in model.bodies:
        wrench = case.body_wrench_n_n_m.get(body.name)
        if wrench is None:
            continue
        values = np.asarray(wrench, dtype=float).reshape(-1, 6).copy()
        # A moment is a force times the document's length unit; a force is not.
        values[:, 3:] *= _INV_MM
        _append_table(blob, tables, values, role="body_wrench", body=body.name)

    for driven in model.driven_coordinates:
        # The case states an *offset* from the separation the model was
        # assembled with -- an Adams joint MOTION -- and the reader adds the
        # separation back, so the geometry is not recomputed on this side.  A
        # translation is a length and is written in the document's unit; a
        # rotation is an angle and is not.
        translation = driven.kind == "translation"
        target_values = case.driven_target_m.get(driven.name)
        rate_values = case.driven_target_rate.get(driven.name)
        if target_values is None and rate_values is None:
            # A coordinate with neither is an error rather than a silent zero:
            # the row would hold it at the assembling pose and the run would
            # look like a plausible answer to a different question.
            raise ValueError(
                f"driven coordinate {driven.name!r} has neither a target nor a rate"
            )
        if target_values is None:
            # A rate-only request: the displacement is the integral of the rate
            # from the assembling pose, by the trapezoid rule so that the
            # displacement matches the sampled rate rather than a rectangle
            # approximation of it.
            rate = np.asarray(rate_values, dtype=float)
            target = np.concatenate(
                ([0.0], np.cumsum(0.5 * (rate[1:] + rate[:-1]) * np.diff(times)))
            )
        else:
            target = np.asarray(target_values, dtype=float)
            if rate_values is None:
                # Every velocity- and acceleration-level row needs the explicit
                # time derivative of the target, so it is derived once here
                # rather than asked of every caller.
                rate = np.gradient(target, times) if times.size > 1 else np.zeros(1)
            else:
                rate = np.asarray(rate_values, dtype=float)
        for role, signal_values in (
            ("driven_offset", target),
            ("driven_offset_rate", rate),
        ):
            _append_table(
                blob,
                tables,
                signal_values * _INV_MM if translation else signal_values,
                role=role,
                coordinate=driven.name,
            )

    solver = case.solver
    document: dict[str, Any] = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "axle_dynamic",
        "name": name or case.name,
        "time": time_entry,
        "solver": solver_settings_document(solver),
        "blobs": tables,
    }
    return document, bytes(blob)
