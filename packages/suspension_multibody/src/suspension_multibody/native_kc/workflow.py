"""
Native quasi-static K/C workflow.

Runs the product's front-axle assembly through the native kernel and returns the
same canonical state records the Python solvers produced, so the two can be
compared field by field:

* **K** -- the wheel-travel x rack grid, solved with the wheel-centre drives;
* **C** -- the standard six load paths at the neutral K point, solved with the
  rack held and a wheel-centre wrench applied at the upright.

Metrics follow ``analysis/metrics.py`` (upright pose, lateral axis) and the C
deformation follows ``analysis/c_mode._wheel_response`` (wheel-centre
translation plus the reference-framed rotation vector).
"""

from __future__ import annotations

import numpy as np

from ..axle_dynamics import AxleDynamicsCase, AxleSolverSettings, run_axle_dynamics
from ..core import quaternion_to_rotation_vector
from .convert import (
    MM,
    NativeKcError,
    quaternion_to_rotation,
    rotation_to_quaternion,
    to_native_model,
)

SIDES = ("L", "R")
AXIS_ORDER = ("fx", "fy", "fz", "mx", "my", "mz")
DEFAULT_TIMES = tuple(float(t) for t in np.linspace(0.0, 2e-3, 9))
DEFAULT_SETTINGS = AxleSolverSettings(internal_step_s=2.5e-4)


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of two scalar-first quaternions."""
    aw, ax, ay, az = (float(v) for v in a)
    bw, bx, by, bz = (float(v) for v in b)
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def quaternion_conjugate(q: np.ndarray) -> np.ndarray:
    """Conjugate (inverse, for unit quaternions) of a scalar-first quaternion."""
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def wheel_center_world(assembly, side: str, position_m, quaternion) -> np.ndarray:
    """World position of the wheel-centre marker for one side (metres)."""
    rotation = quaternion_to_rotation(quaternion)
    local = np.asarray(assembly.point(f"upright_{side}", "wheel_center"), dtype=float) * MM
    return np.asarray(position_m, dtype=float) + rotation @ local


def _drive_target(_assembly, name: str, displacement_m: float) -> tuple[str, float]:
    """
    Return the signal name and its offset from the assembling separation.

    The case states an *offset* -- how far the coordinate moves away from the
    pose the model was assembled in, exactly like an Adams joint MOTION -- and
    the kernel adds the design separation back from the model document's own
    geometry.  Adding it here as well would double every travel.
    """
    return name, displacement_m


def _side_fields(assembly, side: str, position_m, quaternion) -> dict[str, float]:
    rotation = quaternion_to_rotation(quaternion)
    local = np.asarray(assembly.point(f"upright_{side}", "wheel_center"), dtype=float)
    center = np.asarray(position_m, dtype=float) / MM + rotation @ local
    outward = -1.0 if side == "L" else 1.0
    lateral = float(rotation[1, 1])
    name = "left" if side == "L" else "right"
    return {
        f"{name}_wheel_center_x_mm": float(center[0]),
        f"{name}_wheel_center_y_mm": float(center[1]),
        f"{name}_wheel_center_z_mm": float(center[2]),
        f"{name}_camber_deg": float(
            -outward * np.degrees(np.arctan2(float(rotation[2, 1]), lateral))
        ),
        f"{name}_toe_deg": float(
            -outward * np.degrees(np.arctan2(float(rotation[0, 1]), lateral))
        ),
    }


def run_k_grid(
    assembly,
    *,
    wheel_values_mm: tuple[float, ...] = (-10.0, 0.0, 10.0),
    rack_values_mm: tuple[float, ...] = (-5.0, 0.0, 5.0),
    times_s: tuple[float, ...] = DEFAULT_TIMES,
    settings: AxleSolverSettings = DEFAULT_SETTINGS,
) -> list[dict[str, object]]:
    """Solve the K grid natively and return canonical K state records."""
    model = to_native_model(assembly, name="native-k", drive_wheels=True)
    states: list[dict[str, object]] = []
    for wheel in wheel_values_mm:
        for rack in rack_values_mm:
            targets = {
                "wheel_drive_L": _drive_target(assembly, "wheel_drive_L", wheel * MM)[1],
                "wheel_drive_R": _drive_target(assembly, "wheel_drive_R", wheel * MM)[1],
                "rack_drive": _drive_target(assembly, "rack_drive", rack * MM)[1],
            }
            case = AxleDynamicsCase(
                name=f"k-w{wheel:+.0f}-r{rack:+.0f}",
                times_s=times_s,
                driven_target_m={k: tuple(v for _ in times_s) for k, v in targets.items()},
                solver=settings,
            )
            result = run_axle_dynamics(model, case)
            record: dict[str, object] = {
                "case_id": f"k-w{wheel:+.0f}-r{rack:+.0f}",
                "wheel_travel_mm": float(wheel),
                "rack_displacement_mm": float(rack),
            }
            for side in SIDES:
                state = result.body_state(f"upright_{side}")[-1]
                record.update(_side_fields(assembly, side, state[:3], state[3:7]))
            states.append(record)
    return states


def k_reference_pose(assembly, *, times_s=DEFAULT_TIMES, settings=DEFAULT_SETTINGS):
    """Solve the neutral K point natively and return each upright's pose."""
    model = to_native_model(assembly, name="native-k-reference", drive_wheels=True)
    targets = {
        name: _drive_target(assembly, name, 0.0)[1]
        for name in ("wheel_drive_L", "wheel_drive_R", "rack_drive")
    }
    case = AxleDynamicsCase(
        name="k-reference",
        times_s=times_s,
        driven_target_m={k: tuple(v for _ in times_s) for k, v in targets.items()},
        solver=settings,
    )
    result = run_axle_dynamics(model, case)
    return {
        side: (
            result.body_state(f"upright_{side}")[-1, :3].copy(),
            result.body_state(f"upright_{side}")[-1, 3:7].copy(),
        )
        for side in SIDES
    }


def neutral_k_metrics(assembly) -> dict[str, dict[str, float]]:
    """
    Return the K metrics at the assembling pose, per side.

    The assembling pose is the neutral K solution: every driven target sits at
    its design separation there, so the residual is zero and the state is the
    one the model was built with.  The C family measures its deformations
    against it, and the C-minus-K deltas are differences of these metrics.
    """
    metrics: dict[str, dict[str, float]] = {}
    for side in SIDES:
        body = assembly.bodies[f"upright_{side}"]
        position = np.asarray(body.pose.translation, dtype=float) * MM
        quaternion = np.asarray(rotation_to_quaternion(body.pose.rotation), dtype=float)
        metrics["left" if side == "L" else "right"] = _side_fields(
            assembly, side, position, quaternion
        )
    return metrics


def origin_wrench(assembly, side, reference_pose, load: np.ndarray) -> tuple[float, ...]:
    """
    Translate a wheel-centre wrench to the upright's origin (ABI convention).

    The ABI applies ``body_wrench`` at the body origin, so a wrench given at the
    wheel centre needs ``M_origin = M_point + r x F``.  The Python C path works
    in millimetres, hence the moment scaling on the way in.
    """
    position_m, quaternion = reference_pose
    centre = wheel_center_world(assembly, side, position_m, quaternion)
    arm = centre - np.asarray(position_m, dtype=float)
    force = np.asarray(load[:3], dtype=float)
    moment = np.asarray(load[3:], dtype=float) * 1e-3 + np.cross(arm, force)
    return tuple(float(v) for v in np.concatenate((force, moment)))


def run_c_paths(
    assembly,
    *,
    paths=(("fx", "fy", "fz", "mx", "my", "mz")),
    levels: int = 11,
    maximum: float = 1.0,
    times_s: tuple[float, ...] = DEFAULT_TIMES,
    settings: AxleSolverSettings = DEFAULT_SETTINGS,
) -> list[dict[str, object]]:
    """Solve the C load paths natively and return canonical C state records."""
    model = to_native_model(assembly, name="native-c", drive_wheels=False)
    rack_target = _drive_target(assembly, "rack_neutral", 0.0)[1]
    reference = k_reference_pose(assembly, times_s=times_s, settings=settings)
    states: list[dict[str, object]] = []
    axes = paths[0] if paths and isinstance(paths[0], tuple) else paths
    for axis in axes:
        if axis not in AXIS_ORDER:
            raise NativeKcError(f"unknown load axis {axis!r}")
        for level in np.linspace(-maximum, maximum, levels):
            load = np.zeros(6)
            load[AXIS_ORDER.index(axis)] = float(level)
            wrench = origin_wrench(assembly, "L", reference["L"], load)
            case = AxleDynamicsCase(
                name=f"c-{axis}-{level:+.2f}",
                times_s=times_s,
                driven_target_m={"rack_neutral": tuple(rack_target for _ in times_s)},
                body_wrench_n_n_m={
                    "upright_L": tuple(wrench for _ in times_s),
                    "upright_R": tuple((0.0,) * 6 for _ in times_s),
                },
                solver=settings,
            )
            result = run_axle_dynamics(model, case)
            record: dict[str, object] = {
                "case_id": f"c-{axis}-{level:+.2f}",
                "path": axis,
                "level": float(level),
                "side_mode": "single",
                "load_left": [float(v) for v in load],
                "load_right": [0.0] * 6,
            }
            metrics: dict[str, dict[str, float]] = {}
            for side, key in (("L", "deformation_left"), ("R", "deformation_right")):
                state = result.body_state(f"upright_{side}")[-1]
                metrics["left" if side == "L" else "right"] = _side_fields(
                    assembly, side, state[:3], state[3:7]
                )
                ref_position, ref_quaternion = reference[side]
                centre = wheel_center_world(assembly, side, state[:3], state[3:7])
                ref_centre = wheel_center_world(
                    assembly, side, ref_position, ref_quaternion
                )
                relative = quaternion_multiply(
                    quaternion_conjugate(np.asarray(ref_quaternion, dtype=float)),
                    np.asarray(state[3:7], dtype=float),
                )
                rotation = quaternion_to_rotation(ref_quaternion) @ (
                    quaternion_to_rotation_vector(relative)
                )
                record[key] = [
                    float(v) for v in np.concatenate(((centre - ref_centre) / MM, rotation))
                ]
            # The C state's own K metrics travel with it: a caller that wants the
            # C-minus-K deltas should not have to re-derive camber and toe from
            # a pose it no longer has.
            record["metrics"] = metrics
            states.append(record)
    return states


def compliance_matrix(loads: np.ndarray, deformations: np.ndarray) -> np.ndarray:
    """
    Least-squares 6x6 compliance from matched load/deformation rows.

    Mirrors ``analysis/compliance``: the tangent compliance is the
    pseudo-inverse of the stiffness implied by the sampled pairs.
    """
    loads = np.asarray(loads, dtype=float)
    deformations = np.asarray(deformations, dtype=float)
    if loads.shape != deformations.shape or loads.ndim != 2:
        raise NativeKcError("compliance needs matched (n, 6) load and response rows")
    stiffness, *_ = np.linalg.lstsq(deformations, loads, rcond=None)
    return np.linalg.pinv(stiffness)


# --- the contract boundary -------------------------------------------------
#
# The two runners below are the same orchestration as the two above, with one
# difference: they never touch a ctypes structure.  They build the model and
# case documents and hand them to the single kernel entry point, which expands
# the grid, solves every case and returns the body states.  Turning those states
# into the canonical records stays here, because a wheel-centre position and a
# camber angle are reporting, not physics.


def _contract_body_states(assembly, run, *, sides):
    """Return the final sample of each named upright as (position_m, quaternion)."""
    states = run.block("body_state")
    bodies = list(run.document.get("manifest", {}).get("bodies", []))
    resolved = {}
    for side in sides:
        name = f"upright_{side}"
        if name not in bodies:
            raise NativeKcError(f"the contract result has no body {name!r}")
        index = bodies.index(name)
        last = states.shape[0] - 1
        resolved[side] = (states[last, index, :3].copy(), states[last, index, 3:7].copy())
    return resolved


def _assembling_pose(model_document, side):
    """Return the pose the model document declares for one upright."""
    for body in model_document["bodies"]:
        if body["name"] == f"upright_{side}":
            position = np.asarray(body["position"], dtype=float) * MM
            quaternion = np.asarray(body["quaternion"], dtype=float)
            return position, quaternion
    raise NativeKcError(f"the model document has no upright_{side}")


def run_k_grid_contract(
    assembly,
    *,
    wheel_values_mm: tuple[float, ...] = (-10.0, 0.0, 10.0),
    rack_values_mm: tuple[float, ...] = (-5.0, 0.0, 5.0),
    times_s: tuple[float, ...] = DEFAULT_TIMES,
    settings: AxleSolverSettings = DEFAULT_SETTINGS,
) -> list[dict[str, object]]:
    """Solve the K grid through the contract boundary."""
    from ..kernel import run_contract
    from .contract import case_document, model_document

    model = model_document(assembly, name="native-k", drive_wheels=True)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="kc-k",
        wheel_values_mm=wheel_values_mm,
        rack_values_mm=rack_values_mm,
        times_s=times_s,
        settings=settings,
        drive_wheels=True,
    )
    run = run_contract(model, case)
    states = run.block("body_state")
    bodies = list(run.document["manifest"]["bodies"])
    left = bodies.index("upright_L")
    right = bodies.index("upright_R")

    records: list[dict[str, object]] = []
    for index, entry in enumerate(run.cases()):
        wheel = wheel_values_mm[index // len(rack_values_mm)]
        rack = rack_values_mm[index % len(rack_values_mm)]
        case_id = f"k-w{wheel:+.0f}-r{rack:+.0f}"
        # The case layer expands the grid in document order; checking the name
        # it reported turns a silent reordering into a failure.
        if str(entry["name"]) != case_id:
            raise NativeKcError(
                f"the kernel expanded {entry['name']!r} where {case_id!r} was expected"
            )
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        record: dict[str, object] = {
            "case_id": case_id,
            "wheel_travel_mm": float(wheel),
            "rack_displacement_mm": float(rack),
        }
        record.update(_side_fields(assembly, "L", states[last, left, :3], states[last, left, 3:7]))
        record.update(_side_fields(assembly, "R", states[last, right, :3], states[last, right, 3:7]))
        records.append(record)
    return records


def run_c_paths_contract(
    assembly,
    *,
    paths: tuple[str, ...] = AXIS_ORDER,
    levels: int = 11,
    maximum: float = 1.0,
    times_s: tuple[float, ...] = DEFAULT_TIMES,
    settings: AxleSolverSettings = DEFAULT_SETTINGS,
) -> list[dict[str, object]]:
    """Solve the C load paths through the contract boundary."""
    from ..kernel import run_contract
    from .contract import case_document, model_document

    model = model_document(assembly, name="native-c", drive_wheels=False)
    case = case_document(
        assembly,
        family="kc_quasi_static",
        name="kc-c",
        paths=tuple(paths),
        levels=levels,
        maximum=maximum,
        side_mode="single",
        times_s=times_s,
        settings=settings,
        drive_wheels=False,
    )
    run = run_contract(model, case)
    states = run.block("body_state")
    bodies = list(run.document["manifest"]["bodies"])
    # The C deformation is measured against the neutral K pose, which is the
    # assembling pose: at the design separation every driven target has zero
    # residual, so the reference is the model document's own initial state.
    reference = {side: _assembling_pose(model, side) for side in SIDES}
    # The K reference the C response is measured from.  The driven case's zero
    # target resolves to the separation the model was assembled with, so the
    # assembling pose *is* the K reference -- the same thing the Python solver's
    # `KReferenceCache` solves for, reached without solving it again.
    reference_metrics = {
        ("left" if side == "L" else "right"): _side_fields(
            assembly, side, reference[side][0], reference[side][1]
        )
        for side in SIDES
    }

    records: list[dict[str, object]] = []
    for index, entry in enumerate(run.cases()):
        axis = paths[index // levels]
        position_in_path = index % levels
        if position_in_path == 0:
            level = -maximum
        elif position_in_path == levels - 1:
            level = maximum
        else:
            level = -maximum + position_in_path * (2.0 * maximum / (levels - 1))
        case_id = f"c-{axis}-{level:+.2f}"
        if str(entry["name"]) != case_id:
            raise NativeKcError(
                f"the kernel expanded {entry['name']!r} where {case_id!r} was expected"
            )
        load = [0.0] * 6
        load[AXIS_ORDER.index(axis)] = float(level)
        record = {
            "case_id": case_id,
            "path": axis,
            "level": float(level),
            "side_mode": "single",
            "load_left": list(load),
            "load_right": [0.0] * 6,
        }
        last = int(entry["sample_offset"]) + int(entry["sample_count"]) - 1
        metrics: dict[str, dict[str, float]] = {}
        for side, key in (("L", "deformation_left"), ("R", "deformation_right")):
            body = bodies.index(f"upright_{side}")
            state = states[last, body]
            metrics["left" if side == "L" else "right"] = _side_fields(
                assembly, side, state[:3], state[3:7]
            )
            ref_position, ref_quaternion = reference[side]
            centre = wheel_center_world(assembly, side, state[:3], state[3:7])
            ref_centre = wheel_center_world(assembly, side, ref_position, ref_quaternion)
            relative = quaternion_multiply(
                quaternion_conjugate(np.asarray(ref_quaternion, dtype=float)),
                np.asarray(state[3:7], dtype=float),
            )
            rotation = quaternion_to_rotation(ref_quaternion) @ quaternion_to_rotation_vector(
                relative
            )
            record[key] = [
                float(value) for value in np.concatenate(((centre - ref_centre) / MM, rotation))
            ]
        record["metrics"] = metrics
        record["c_minus_k"] = {
            key: float(value - reference_metrics[side][key])
            for side in ("left", "right")
            for key, value in metrics[side].items()
        }
        records.append(record)
    return records
