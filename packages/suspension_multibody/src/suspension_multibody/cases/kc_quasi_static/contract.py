"""
Emit the contract documents the kernel reads.

This is the Python half of the document boundary, and it is the *authoring*
half: it describes the model and the case, and it stops there.  Nothing here
computes a residual, a constitutive law or a Jacobian, and nothing here knows
the kernel's ctypes layout.

Two conventions are load-bearing and are therefore stated once, here:

* **Everything is body-local.**  A joint point is local to its first body, a
  joint axis to the body it is written against, a marker to its body.  The
  assembly already carries global and local forms, so the conversion happens
  once, where the geometry is.
* **Everything is millimetres.**  The document's ``units`` block says so, and
  the kernel scales on the way in.  Writing metres here would silently produce
  a model a thousand times too small.

The two families the native K/C workflow runs need *different models*, and the
difference is physical rather than cosmetic:

* K drives the wheel centres through the rigid kinematic set, so the model
  carries ``ideal_constraints`` with the paired ball joints folded into their
  equivalent revolutes, and no bushings;
* C loads the compliant set, where the arm mounts are carried by bushings
  instead, so the model carries ``constraints`` and the bushings.
"""

from __future__ import annotations

import numpy as np

from ...joints import (
    DRIVEN_KINDS,
    JOINT_KINDS,
    JointTableError,
    definition_for,
    validate_joint_axes,
)
from ...kernel.solver import solver_settings_document as _solver_settings_document
from ...preparation.assembly.types import (
    ConstantVelocityJoint,
    CylindricalJoint,
    InPlaneJoint,
    PrismaticJoint,
    RevoluteJoint,
    UniversalJoint,
)
from .convert import (
    NativeKcError,
    _local_axis,
    _local_point,
    collapse_spherical_pairs,
    rotation_to_quaternion,
)

UNITS = {"length": "mm", "mass": "kg", "time": "s", "angle": "rad"}
#: The joint table lives in `joints/`; this module no longer keeps its own copy.
#: `_JOINT_KINDS` used to map only three of the eight types and reject the rest,
#: which is what made "any assembly can use any joint" false in practice.
_JOINT_KINDS = JOINT_KINDS
#: Distinct from the joint table: a driven coordinate is a *prescribed* degree
#: of freedom, not a joint in the assembly's sense, but it travels in the same
#: list because that is where the kernel expects to find it.
_DRIVEN_KINDS = {
    kind: definition.kernel_name for kind, definition in DRIVEN_KINDS.items()
}


def _constraint_axes(
    constraint, body_a, body_b
) -> dict[str, list[float]]:
    """
    Return the axis fields one assembly constraint carries, in body-local form.

    Each branch mirrors the dataclass it reads: `InPlaneJoint` has only `axis_a`
    (the plane normal), `ConstantVelocityJoint` carries a primary and a secondary
    axis on each body, and the rest carry one axis per body.  A type with no axis
    returns an empty mapping -- the caller decides which of them are required.
    """
    if isinstance(constraint, InPlaneJoint):
        return {"axis_a": _unit_axis(_local_axis(constraint.axis_a, body_a))}
    if isinstance(constraint, ConstantVelocityJoint):
        return {
            "axis_a": _unit_axis(_local_axis(constraint.axis_a, body_a)),
            "axis_b": _unit_axis(_local_axis(constraint.axis_b, body_b)),
            "axis_a_secondary": _unit_axis(
                _local_axis(constraint.axis_a_secondary, body_a)
            ),
            "axis_b_secondary": _unit_axis(
                _local_axis(constraint.axis_b_secondary, body_b)
            ),
        }
    if isinstance(
        constraint, (RevoluteJoint, PrismaticJoint, UniversalJoint, CylindricalJoint)
    ):
        return {
            "axis_a": _unit_axis(_local_axis(constraint.axis_a, body_a)),
            "axis_b": _unit_axis(_local_axis(constraint.axis_b, body_b)),
        }
    return {}
UNITS = {"length": "mm", "mass": "kg", "time": "s", "angle": "rad"}
#: The joint table lives in `joints/`; this module no longer keeps its own copy.
#: `_JOINT_KINDS` used to map only three of the eight types and reject the rest,
#: which is what made "any assembly can use any joint" false in practice.
_JOINT_KINDS = JOINT_KINDS
#: Distinct from the joint table: a driven coordinate is a *prescribed* degree
#: of freedom, not a joint in the assembly's sense, but it travels in the same
#: list because that is where the kernel expects to find it.
_DRIVEN_KINDS = {kind: definition.kernel_name for kind, definition in DRIVEN_KINDS.items()}


def _vec3(values) -> list[float]:
    return [float(v) for v in np.asarray(values, dtype=float)[:3]]


def _unit_axis(values) -> list[float]:
    axis = np.asarray(values, dtype=float)[:3]
    return [float(v) for v in axis / np.linalg.norm(axis)]


def model_document(assembly, *, name: str = "front-axle", drive_wheels: bool = True) -> dict[str, object]:
    """
    Describe one axle assembly as a multibody model document.

    ``drive_wheels`` selects which physical model the document describes: the
    rigid kinematic set of the K family, or the compliant set of the C family.
    It is not a solver setting, which is why it lives here rather than in the
    case document.
    """
    bodies = []
    for body_name, body in assembly.bodies.items():
        inertia = np.asarray(body.inertia, dtype=float)
        if body.fixed:
            mass, inertia = 0.0, np.eye(3)
        else:
            mass = float(body.mass) if body.mass > 0.0 else 1.0
            if not np.all(np.isfinite(inertia)) or np.linalg.norm(inertia) < 1e-9:
                inertia = np.eye(3)
        bodies.append(
            {
                "name": body_name,
                "mass": mass,
                "inertia": [[float(v) for v in row] for row in inertia],
                "fixed": bool(body.fixed),
                "position": _vec3(body.pose.translation),
                "quaternion": list(rotation_to_quaternion(body.pose.rotation)),
            }
        )

    joint_source = (
        collapse_spherical_pairs(assembly.ideal_constraints)
        if drive_wheels
        else assembly.constraints
    )
    joints = []
    joints = []
    for constraint in joint_source:
        try:
            definition = definition_for(type(constraint).__name__)
        except JointTableError as exc:
            # The joint table owns the message; this keeps the kc layer's own
            # error type so callers that catch `NativeKcError` still work.
            raise NativeKcError(str(exc)) from exc
        body_a = assembly.bodies[constraint.body_a]
        body_b = assembly.bodies[constraint.body_b]
        entry: dict[str, object] = {
            "name": constraint.name,
            "type": definition.kernel_name,
            "body_a": constraint.body_a,
            "body_b": constraint.body_b,
            "point_a": _vec3(_local_point(constraint.point_a, body_a)),
            "point_b": _vec3(_local_point(constraint.point_b, body_b)),
        }
        # The axes each type needs come from the table, so a joint added to the
        # assembly layer cannot be encoded with the wrong field set.  The
        # defaults the kernel substitutes for a missing axis are plausible
        # numbers, which is why an omission has to fail here instead.
        axes = _constraint_axes(constraint, body_a, body_b)
        validate_joint_axes(
            joint_name=constraint.name,
            kernel_name=definition.kernel_name,
            axes=axes,
        )
        entry.update(axes)
        if isinstance(constraint, ConstantVelocityJoint):
            entry["convel_angle_target"] = float(constraint.angle_target)
        joints.append(entry)

    elements = []
    if not drive_wheels:
        elements = [_bushing_element(bushing) for bushing in assembly.bushings]

    markers = [
        {
            "name": f"wheel_center_{side}",
            "body": f"upright_{side}",
            "point": _vec3(assembly.point(f"upright_{side}", "wheel_center")),
        }
        for side in ("L", "R")
        if f"upright_{side}" in assembly.bodies
    ]

    _, driven_joints = _driven_coordinates(assembly, drive_wheels=drive_wheels)
    document: dict[str, object] = {
        "contract": "multibody-model",
        "contract_version": 1,
        "kind": "model",
        "name": name,
        "units": dict(UNITS),
        "gravity": [0.0, 0.0, 0.0],
        "bodies": bodies,
        "joints": joints + driven_joints,
        "elements": elements,
        "markers": markers,
        # A `c` path loads the wheel centre, which is a point of the upright
        # rather than the upright's origin.  Naming the marker here is what
        # makes the kernel take the lever arm at the pose it solves for;
        # pre-transferring it at the reference pose freezes the arm and loses
        # the first-order geometry of the swept upright.
        "body_wrench_markers": [marker["name"] for marker in markers],
    }
    return document


def _bushing_element(bushing) -> dict[str, object]:
    """One bushing, with its stiffness blocks in their own units."""
    return {
        "name": bushing.name,
        "type": "bushing",
        "body_a": bushing.body_a,
        "body_b": bushing.body_b,
        "parameters": {
            "point_a": _vec3(bushing.local_pose_a.translation),
            "point_b": _vec3(bushing.local_pose_b.translation),
            "frame_a_quaternion": [float(v) for v in bushing.local_pose_a.quaternion],
            "frame_b_quaternion": [float(v) for v in bushing.local_pose_b.quaternion],
            "stiffness": [
                [float(v) for v in row] for row in np.asarray(bushing.stiffness, dtype=float)
            ],
            "damping": [
                [float(v) for v in row] for row in np.asarray(bushing.damping, dtype=float)
            ],
            "preload": [float(v) for v in np.asarray(bushing.preload, dtype=float)[:6]],
        },
    }


def _driven_coordinates(assembly, *, drive_wheels: bool):
    """
    Describe the prescribed degrees of freedom, as joints plus their signal names.

    A driven coordinate names its target signal rather than carrying targets:
    how far the wheel travels is a *case* input, and the case layer turns the
    travel into the absolute separation the kernel's constraint row measures.
    The axis is written in the reaction body's frame because that is the frame
    the relative separation is measured in.
    """
    coordinates: list[tuple[str, dict[str, object]]] = []

    def add(name: str, kind: str, body: str, point_local_mm, axis_local) -> None:
        entry = {
            "name": name,
            "type": _DRIVEN_KINDS[kind],
            "body_a": body,
            "body_b": "chassis",
            "point_a": _vec3(point_local_mm),
            "point_b": [0.0, 0.0, 0.0],
            "axis_b": _unit_axis(axis_local),
            "reference_quaternion": [1.0, 0.0, 0.0, 0.0],
            "target": name,
        }
        coordinates.append((name, entry))

    if drive_wheels:
        for side in ("L", "R"):
            add(
                f"wheel_drive_{side}",
                "translation",
                f"upright_{side}",
                assembly.point(f"upright_{side}", "wheel_center"),
                (0.0, 0.0, 1.0),
            )
        rack_name = "rack_drive"
    else:
        rack_name = "rack_neutral"
    add(
        rack_name,
        "translation",
        "rack",
        assembly.point("rack", "center"),
        (0.0, 1.0, 0.0),
    )
    return coordinates, [entry for _, entry in coordinates]


def solver_settings_document(settings, *, times_s=None) -> dict[str, object]:
    """Compatibility wrapper for the shared neutral solver serializer."""
    del times_s
    return _solver_settings_document(settings)


def time_document(times_s) -> dict[str, object]:
    """Return the output grid as a start/end/step the case layer can re-derive."""
    values = [float(value) for value in times_s]
    if len(values) < 2:
        raise NativeKcError("a case needs at least two sample times")
    step = values[1] - values[0]
    for left, right in zip(values, values[1:]):
        if abs((right - left) - step) > 1e-15:
            raise NativeKcError("the case layer only expands a uniform time grid")
    return {"start_s": values[0], "end_s": values[-1], "step_s": step}


def case_document(
    assembly,
    *,
    family: str,
    name: str,
    wheel_values_mm: tuple[float, ...] = (),
    rack_values_mm: tuple[float, ...] = (),
    drive: str = "wheel_center",
    left_right_mode: str = "symmetric",
    paths: tuple[str, ...] = (),
    levels: int = 11,
    maximum: float = 1.0,
    side_mode: str = "single",
    times_s: tuple[float, ...] = (),
    settings=None,
    drive_wheels: bool | None = None,
) -> dict[str, object]:
    """Describe a ``kc_quasi_static`` request as a case document."""
    if family not in ("kc_quasi_static",):
        raise NativeKcError(f"unsupported case family {family!r}")
    document: dict[str, object] = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": family,
        "name": name,
    }
    if drive_wheels is None:
        drive_wheels = bool(wheel_values_mm)
    driven, _ = _driven_coordinates(assembly, drive_wheels=drive_wheels)
    driven_names = [coordinate for coordinate, _ in driven]
    if times_s:
        document["time"] = time_document(times_s)
    if settings is not None:
        document["solver"] = solver_settings_document(settings, times_s=times_s)
    if wheel_values_mm or rack_values_mm:
        document["k"] = {
            "wheel_values_mm": [float(v) for v in wheel_values_mm],
            "rack_values_mm": [float(v) for v in rack_values_mm],
            "drive": drive,
            "left_right_mode": left_right_mode,
            "axis_map": {
                "wheel": [
                    value for value in driven_names if value.startswith("wheel_drive_")
                ],
                "rack": next(
                    value for value in driven_names if value.startswith("rack_")
                ),
            },
        }
    if paths:
        document["c"] = {
            "paths": [str(p) for p in paths],
            "levels": int(levels),
            "maximum": float(maximum),
            "side_mode": side_mode,
            "load_marker": "wheel_center_L",
        }
    return document
