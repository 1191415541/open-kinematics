"""
True time derivatives of dynamic geometry, from the state the solver already stores.

The distinction this module exists to preserve is between three different things that
all produce "a number per sample":

* **A true time derivative.**  Toe rate is how fast the toe angle changes with respect
  to real time.  The native state stores each body's velocity and angular velocity
  alongside its pose, so the rate follows from the geometry's own definition by the
  chain rule -- no differencing, no trajectory ratio.
* **An input partial derivative.**  How a geometry value would change if a *driving
  input* changed, holding time fixed.  This is not a time derivative and must not be
  reported as one; the two differ by the input's own rate.
* **A difference quotient over samples.**  ``(v[i+1] - v[i]) / (t[i+1] - t[i])`` is a
  property of the *sample grid*, not of the motion.  It divides by the output interval,
  so it changes when the requested samples change even though the physics did not.
  The measurement layer already refuses this for solver responses ("a finite difference
  belongs in a test oracle"); this module is the other half of that refusal, supplying
  the derivative that does not need one.

Deriving the rate analytically also settles the two cases that a difference quotient
gets wrong in a way that looks right:

* **A moving reference frame.**  Toe and camber are measured *relative* to a reference
  frame.  Both ``omega_body x axis`` and ``omega_ref x axis`` terms belong in the rate;
  dropping the second gives a rate that is correct only while the reference is still,
  which is exactly the case a stationary test fixture cannot distinguish.
* **A stopped body.**  A stopped wheel has a zero toe rate, and that is a fact rather
  than a missing value.  What must not happen is using a zero travel speed to produce a
  number that is then presented as an input partial derivative -- the substitution the
  contract names explicitly.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..characteristics.measurements import _as_vector, _axis_in_reference
from ..characteristics.values import (
    CharacteristicsError,
    MeasurementValue,
    VectorValue,
)

__all__ = [
    "DYNAMIC_GEOMETRIES",
    "GeometryRate",
    "geometry_rate_request",
    "pose_rates",
]

#: The dynamic geometries whose true time derivative this module defines.
DYNAMIC_GEOMETRIES = ("toe", "camber", "wheel_center")

#: Below this magnitude a quantity is treated as exactly zero.  A rate that is zero
#: because nothing moved is a fact; a rate that is tiny because two large numbers
#: cancelled is not, and this floor is the boundary between them.
ZERO_FLOOR = 1e-12

#: The native body-state layout, named so the offsets are not scattered as literals.
_STATE_POSITION = slice(0, 3)
_STATE_QUATERNION = slice(3, 7)
_STATE_VELOCITY = slice(7, 10)
_STATE_OMEGA = slice(10, 13)
_STATE_WIDTH = 13


@dataclass(frozen=True)
class GeometryRate:
    """
    One geometry's value and its true time derivative over a sample sequence.

    ``rate`` is ``d(value)/dt``, computed from stored velocities.  Angular geometries
    (toe, camber) are scalars and carry a ``MeasurementValue``; a wheel centre is a
    position, so its value and rate are ``VectorValue`` with one row per sample.  Both
    are the contract's own types, so a consumer reads a status per sample either way
    and cannot mistake a missing component for a zero one.

    ``driver`` records what the rate depends on, so a consumer can tell a true time
    derivative from an input partial derivative without reading the call site -- the two
    are the same shape and would otherwise be interchangeable.
    """

    kind: str
    value: MeasurementValue | VectorValue
    rate: MeasurementValue | VectorValue
    driver: str
    reference_moving: bool


def _rotation_matrices(quaternions: np.ndarray) -> np.ndarray:
    """
    Return rotation matrices for scalar-first quaternions, shape ``(n, 3, 3)``.

    Written out rather than imported: this is the query layer, and reaching into the
    modelling primitives would invert the layering the architecture gate enforces.
    """
    q = np.asarray(quaternions, dtype=float)
    if q.ndim != 2 or q.shape[1] != 4:
        raise CharacteristicsError("a quaternion sequence must have shape (samples, 4)")
    norm = np.linalg.norm(q, axis=1)
    if np.any(norm <= 0.0):
        raise CharacteristicsError("a stored quaternion has zero norm")
    w, x, y, z = (q[:, index] / norm for index in range(4))
    matrices = np.empty((q.shape[0], 3, 3), dtype=float)
    matrices[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    matrices[:, 0, 1] = 2.0 * (x * y - z * w)
    matrices[:, 0, 2] = 2.0 * (x * z + y * w)
    matrices[:, 1, 0] = 2.0 * (x * y + z * w)
    matrices[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    matrices[:, 1, 2] = 2.0 * (y * z - x * w)
    matrices[:, 2, 0] = 2.0 * (x * z - y * w)
    matrices[:, 2, 1] = 2.0 * (y * z + x * w)
    matrices[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return matrices


def _as_states(states: Any, label: str) -> np.ndarray:
    """Validate a stored body-state sequence and return it as a float array."""
    array = np.asarray(states, dtype=float)
    if array.ndim != 2 or array.shape[1] != _STATE_WIDTH:
        raise CharacteristicsError(
            f"{label} must be a (samples, {_STATE_WIDTH}) body state -- position, "
            "quaternion, velocity and omega; a pose alone has no rate"
        )
    if array.shape[0] == 0:
        raise CharacteristicsError(f"{label} carries no samples")
    return array


def _poses_from_states(states: np.ndarray) -> np.ndarray:
    """
    Return homogeneous world poses, shape ``(n, 4, 4)``, from stored body states.

    The characteristic measurement helpers take 4x4 poses (a rotation and a
    translation together), while the native state stores position and quaternion in
    separate columns; this is the one place that joins them back, so the join cannot
    drift apart from the offset constants above.
    """
    poses = np.zeros((states.shape[0], 4, 4), dtype=float)
    poses[:, :3, :3] = _rotation_matrices(states[:, _STATE_QUATERNION])
    poses[:, :3, 3] = states[:, _STATE_POSITION]
    poses[:, 3, 3] = 1.0
    return poses


def pose_rates(
    states: Any,
    *,
    reference_states: Any | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return ``(velocity, omega)`` per sample, relative to a reference body if given.

    Velocity and angular velocity are *stored* in the native state, not reconstructed,
    which is the whole reason a true time derivative is available here.

    With a reference, the returned velocity is the rate of this body's origin as seen
    from the reference body: the reference's own linear and angular motion is removed
    by ``v_rel = v - (v_ref + omega_ref x r)``.  Omitting that carried term is the
    classic moving-frame error -- the result still looks like a rate of this body and
    is actually the sum of both bodies' motion.
    """
    absolute = _as_states(states, "body states")
    velocity = absolute[:, _STATE_VELOCITY]
    omega = absolute[:, _STATE_OMEGA]
    if reference_states is None:
        return velocity, omega
    reference = _as_states(reference_states, "reference states")
    if reference.shape != absolute.shape:
        raise CharacteristicsError("a body and its reference must share a sample grid")
    lever = absolute[:, _STATE_POSITION] - reference[:, _STATE_POSITION]
    carried = np.cross(reference[:, _STATE_OMEGA], lever)
    return (
        velocity - (reference[:, _STATE_VELOCITY] + carried),
        omega - reference[:, _STATE_OMEGA],
    )


def _reference_frames(
    absolute: np.ndarray, reference_states: Any | None
) -> tuple[np.ndarray, np.ndarray, np.ndarray | None]:
    """
    Return ``(reference_poses, reference_omega, reference_absolute)``.

    With no reference the world frame is used: identity rotation, zero rate, and
    ``None`` for the states so the wheel-centre branch knows there is nothing to
    subtract.
    """
    if reference_states is None:
        identity = np.broadcast_to(np.eye(4), (absolute.shape[0], 4, 4))
        return identity, np.zeros_like(absolute[:, _STATE_OMEGA]), None
    reference = _as_states(reference_states, "reference states")
    if reference.shape != absolute.shape:
        raise CharacteristicsError("a body and its reference must share a sample grid")
    return (
        _poses_from_states(reference),
        reference[:, _STATE_OMEGA],
        reference,
    )


def _axis_rate(
    poses: np.ndarray,
    omega: np.ndarray,
    reference_poses: np.ndarray,
    reference_omega: np.ndarray,
    axis: np.ndarray,
) -> np.ndarray:
    """
    Return ``d/dt`` of a body-local axis resolved into the reference frame.

    The resolved axis is ``R_ref^T R_body a``, so its rate is
    ``R_ref^T (omega_body x R_body a) - omega_ref x (R_ref^T R_body a)``.  Both terms
    are required: the first is the axis turning with its own body, the second is the
    observer turning with the reference.  Dropping the second is invisible whenever the
    reference is stationary and wrong as soon as it is not.
    """
    world = np.einsum("nij,j->ni", poses[:, :3, :3], axis)
    resolved = np.einsum("nji,nj->ni", reference_poses[:, :3, :3], world)
    turning = np.cross(omega, world)
    return (
        np.einsum("nji,nj->ni", reference_poses[:, :3, :3], turning)
        - np.cross(reference_omega, resolved)
    )


def _toe_value_rate(
    resolved: np.ndarray, resolved_rate: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return ``(toe, toe_dot)`` from the resolved axis and its rate.

    Toe is ``atan2(-v_x, v_y)``; its exact derivative is
    ``(v_x dv_y - v_y dv_x) / (v_x^2 + v_y^2)``.  The denominator is the squared
    horizontal projection, so a vertical axis (no horizontal projection) makes the rate
    undefined and is refused rather than reported as a large finite number.
    """
    horizontal_squared = resolved[:, 0] ** 2 + resolved[:, 1] ** 2
    degenerate = horizontal_squared <= ZERO_FLOOR
    if degenerate.any():
        raise CharacteristicsError(
            f"toe rate is undefined at {int(degenerate.sum())} sample(s): the wheel axis "
            "has no horizontal projection, so toe has no defined value to differentiate"
        )
    value = np.arctan2(-resolved[:, 0], resolved[:, 1])
    rate = (
        resolved[:, 0] * resolved_rate[:, 1] - resolved[:, 1] * resolved_rate[:, 0]
    ) / horizontal_squared
    return value, rate


def _camber_value_rate(
    resolved: np.ndarray, resolved_rate: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """
    Return ``(camber, camber_dot)`` from the resolved axis and its rate.

    Camber is ``atan2(v_z, h)`` with ``h = hypot(v_x, v_y)``, so
    ``dh/dt = (v_x dv_x + v_y dv_y) / h`` and the rate is
    ``(dv_z h - v_z dh/dt) / (h^2 + v_z^2)``.  A zero-length axis is refused; a vertical
    axis is not, because camber is then legitimately +/-pi/2.
    """
    length_squared = np.einsum("ni,ni->n", resolved, resolved)
    degenerate = length_squared <= ZERO_FLOOR
    if degenerate.any():
        raise CharacteristicsError(
            f"camber rate is undefined at {int(degenerate.sum())} sample(s): the wheel "
            "axis has no length"
        )
    horizontal = np.hypot(resolved[:, 0], resolved[:, 1])
    safe = np.where(horizontal <= ZERO_FLOOR, 1.0, horizontal)
    horizontal_rate = (
        resolved[:, 0] * resolved_rate[:, 0] + resolved[:, 1] * resolved_rate[:, 1]
    ) / safe
    value = np.arctan2(resolved[:, 2], horizontal)
    rate = (
        resolved_rate[:, 2] * horizontal - resolved[:, 2] * horizontal_rate
    ) / length_squared
    return value, rate


def geometry_rate_request(
    states: Any,
    *,
    kind: str,
    axis: Any = None,
    sign: int = 1,
    reference_states: Any | None = None,
    origin: Any = None,
    input_rate: Any = None,
    frame_id: str = "world",
) -> GeometryRate:
    """
    Answer one dynamic-geometry rate request over a stored sample sequence.

    ``kind`` is one of :data:`DYNAMIC_GEOMETRIES`.  Both the value and the rate are
    returned as ``MeasurementValue``, so a sample whose rate is undefined is reported
    with a reason instead of a number.  ``driver`` is ``"pose_rate"`` throughout: the
    rate comes from the stored pose rate by the chain rule, never from differencing
    samples.

    ``input_rate`` is the *driving input's* rate when a caller has one.  It is accepted
    only to support the input partial derivative, and its presence is recorded rather
    than folded into the time derivative: a stopped input cannot separate the geometry's
    dependence on it from the rest of the motion, so that case is refused by name.
    """
    if kind not in DYNAMIC_GEOMETRIES:
        raise CharacteristicsError(
            f"unknown dynamic geometry {kind!r}; the contract defines "
            f"{', '.join(DYNAMIC_GEOMETRIES)}"
        )
    absolute = _as_states(states, "body states")
    poses = _poses_from_states(absolute)
    omega = absolute[:, _STATE_OMEGA]
    reference_poses, reference_omega, reference_absolute = _reference_frames(
        absolute, reference_states
    )
    reference_moving = bool(
        reference_absolute is not None
        and (
            np.any(np.abs(reference_absolute[:, _STATE_VELOCITY]) > ZERO_FLOOR)
            or np.any(np.abs(reference_omega) > ZERO_FLOOR)
        )
    )

    if kind == "wheel_center":
        local = _as_vector(origin, "origin")
        world = poses[:, :3, 3] + np.einsum("nij,j->ni", poses[:, :3, :3], local)
        lever = world - absolute[:, _STATE_POSITION]
        world_rate = absolute[:, _STATE_VELOCITY] + np.cross(omega, lever)
        if reference_absolute is None:
            value, rate = world, world_rate
        else:
            reference_world = reference_poses[:, :3, 3] + np.einsum(
                "nij,j->ni", reference_poses[:, :3, :3], local
            )
            reference_lever = reference_world - reference_absolute[:, _STATE_POSITION]
            reference_rate = reference_absolute[
                :, _STATE_VELOCITY
            ] + np.cross(reference_omega, reference_lever)
            value, rate = world - reference_world, world_rate - reference_rate
        name = "dynamic_wheel_center"
        driver = "pose_rate"
        if input_rate is not None:
            _check_input_rate(input_rate, rate)
            driver = "pose_rate+input_rate"
        return GeometryRate(
            kind=kind,
            value=VectorValue.all_valid(
                name, value, ("m", "m", "m"), frame_id
            ),
            rate=VectorValue.all_valid(
                f"{name}_rate", rate, ("m/s", "m/s", "m/s"), frame_id
            ),
            driver=driver,
            reference_moving=reference_moving,
        )

    vector = _as_vector(axis, "axis")
    resolved = _axis_in_reference(poses, reference_poses, vector)
    resolved_rate = _axis_rate(poses, omega, reference_poses, reference_omega, vector)
    compute = _toe_value_rate if kind == "toe" else _camber_value_rate
    value, rate = compute(resolved, resolved_rate)
    value, rate = value * sign, rate * sign

    name = f"dynamic_{kind}"
    driver = "pose_rate"
    if input_rate is not None:
        _check_input_rate(input_rate, rate)
        driver = "pose_rate+input_rate"
    return GeometryRate(
        kind=kind,
        value=MeasurementValue.all_valid(name, value, "rad", frame_id),
        rate=MeasurementValue.all_valid(f"{name}_rate", rate, "rad/s", frame_id),
        driver=driver,
        reference_moving=reference_moving,
    )


def _check_input_rate(input_rate: Any, rate: np.ndarray) -> None:
    """
    Validate a driving input's rate before it is used to form a partial derivative.

    A stopped input is refused by name: with the input not moving, the geometry's
    dependence on it cannot be separated from the rest of the motion, and the time
    derivative is not a substitute for the partial one.  Reporting one under the
    other's label is the substitution the contract names explicitly.
    """
    driving = np.asarray(input_rate, dtype=float)
    if driving.ndim != 1 or driving.shape[0] != rate.shape[0]:
        raise CharacteristicsError(
            "input rates must carry one value per sample to form a partial derivative"
        )
    stopped = int(np.sum(np.abs(driving) <= ZERO_FLOOR))
    if stopped:
        raise CharacteristicsError(
            f"the input partial derivative is undefined at {stopped} sample(s): a "
            "stopped input cannot separate the geometry's dependence on it from the "
            "rest of the motion, and the time derivative is not a substitute"
        )
