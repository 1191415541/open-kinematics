"""Physical 33-channel export from the native axle dynamics result."""

from __future__ import annotations

import math
from collections.abc import Sequence

import numpy as np

from ..axle_dynamics import (
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleDynamicsResult,
)
from .axle_contract import (
    AxleChannelBindings,
    load_axle_channel_contract,
    validate_axle_channel_bindings,
)
from .time_domain import TimeHistory

_POSITION = slice(0, 3)
_QUATERNION = slice(3, 7)
_VELOCITY = slice(7, 10)
_OMEGA = slice(10, 13)
_ACCELERATION = slice(13, 16)
_ALPHA = slice(16, 19)


def axle_history_from_result(
    model: AxleDynamicsModel,
    result: AxleDynamicsResult,
    bindings: AxleChannelBindings,
    *,
    case: AxleDynamicsCase | None = None,
) -> TimeHistory:
    """
    Evaluate every frozen channel from explicit model roles and SI outputs.

    A runtime case enables the common momentum-balance reconstruction for the
    fixture wrench.  The raw joint multipliers are retained for initialization
    evidence, but redundant ideal constraints can make their individual
    reaction split solver-dependent.
    """
    validate_axle_channel_bindings(model, bindings)
    _validate_result_layout(model, result)
    sprung = result.body_state(bindings.sprung_body)
    sprung_rotation = _rotation_matrices(sprung[:, _QUATERNION])
    sprung_rotation_vector = _relative_rotation_vectors(
        sprung[:, _QUATERNION],
        sprung[0, _QUATERNION],
    )

    left_marker = bindings.left_wheel_center_marker
    right_marker = bindings.right_wheel_center_marker
    left_position, left_velocity, left_acceleration = _marker_kinematics(
        result.body_state(left_marker.body),
        left_marker.point_local_m,
    )
    right_position, right_velocity, right_acceleration = _marker_kinematics(
        result.body_state(right_marker.body),
        right_marker.point_local_m,
    )
    left_deflection = _suspension_deflection(
        sprung,
        sprung_rotation,
        left_position,
    )
    right_deflection = _suspension_deflection(
        sprung,
        sprung_rotation,
        right_position,
    )

    left_tire = result.tire_state(bindings.left_tire)
    right_tire = result.tire_state(bindings.right_tire)
    left_spring = result.spring_state(bindings.left_spring)
    right_spring = result.spring_state(bindings.right_spring)
    left_damper = result.spring_state(bindings.left_damper)
    right_damper = result.spring_state(bindings.right_damper)
    fixture_wrench = _fixture_wrench(model, result, bindings, case=case)

    channels: dict[str, tuple[float, ...]] = {
        "sprung_body.heave": _tuple(sprung[:, 2] - sprung[0, 2]),
        "sprung_body.pitch": _tuple(sprung_rotation_vector[:, 1]),
        "sprung_body.roll": _tuple(sprung_rotation_vector[:, 0]),
        "sprung_body.heave_velocity": _tuple(sprung[:, 9]),
        "sprung_body.pitch_rate": _tuple(sprung[:, 11]),
        "sprung_body.roll_rate": _tuple(sprung[:, 10]),
        "sprung_body.heave_acceleration": _tuple(sprung[:, 15]),
        "left.wheel_center_z": _tuple(left_position[:, 2] - left_position[0, 2]),
        "right.wheel_center_z": _tuple(
            right_position[:, 2] - right_position[0, 2]
        ),
        "left.wheel_center_z_velocity": _tuple(left_velocity[:, 2]),
        "right.wheel_center_z_velocity": _tuple(right_velocity[:, 2]),
        "left.wheel_center_z_acceleration": _tuple(left_acceleration[:, 2]),
        "right.wheel_center_z_acceleration": _tuple(right_acceleration[:, 2]),
        "left.suspension_deflection": _tuple(left_deflection),
        "right.suspension_deflection": _tuple(right_deflection),
        "left.tire_normal_force": _tuple(left_tire[:, 4]),
        "right.tire_normal_force": _tuple(right_tire[:, 4]),
        "left.tire_longitudinal_force": _tuple(left_tire[:, 5]),
        "right.tire_longitudinal_force": _tuple(right_tire[:, 5]),
        "left.tire_lateral_force": _tuple(left_tire[:, 6]),
        "right.tire_lateral_force": _tuple(right_tire[:, 6]),
        "left.spring_force": _tuple(_conservative_axial_force(left_spring)),
        "right.spring_force": _tuple(_conservative_axial_force(right_spring)),
        "left.damper_force": _tuple(_dissipative_axial_force(left_damper)),
        "right.damper_force": _tuple(_dissipative_axial_force(right_damper)),
        "left.wheel_spin": _tuple(
            _wheel_spin(
                model,
                result,
                left_marker.body,
                bindings.left_wheel_spin_joint,
            )
        ),
        "right.wheel_spin": _tuple(
            _wheel_spin(
                model,
                result,
                right_marker.body,
                bindings.right_wheel_spin_joint,
            )
        ),
        "fixture.force_x": _tuple(fixture_wrench[:, 0]),
        "fixture.force_y": _tuple(fixture_wrench[:, 1]),
        "fixture.force_z": _tuple(fixture_wrench[:, 2]),
        "fixture.moment_x": _tuple(fixture_wrench[:, 3]),
        "fixture.moment_y": _tuple(fixture_wrench[:, 4]),
        "fixture.moment_z": _tuple(fixture_wrench[:, 5]),
    }
    contract = load_axle_channel_contract()
    expected = tuple(contract["channels"])
    if tuple(channels) != expected:
        raise RuntimeError("native axle channel order differs from frozen contract")
    units = {
        name: str(contract["channels"][name]["unit"])
        for name in expected
    }
    return TimeHistory(
        time=_tuple(result.times_s),
        channels=channels,
        units=units,
    )


def _validate_result_layout(
    model: AxleDynamicsModel,
    result: AxleDynamicsResult,
) -> None:
    expected = {
        "body": tuple(body.name for body in model.bodies),
        "constraint": tuple(joint.name for joint in model.joints),
        "spring": tuple(spring.name for spring in model.springs),
        "bushing": tuple(bushing.name for bushing in model.bushings),
        "anti-roll bar": tuple(bar.name for bar in model.anti_roll_bars),
        "tire": tuple(tire.name for tire in model.tires),
    }
    actual = {
        "body": result.body_names,
        "constraint": result.constraint_names,
        "spring": result.spring_names,
        "bushing": result.bushing_names,
        "anti-roll bar": result.anti_roll_bar_names,
        "tire": result.tire_names,
    }
    for label in expected:
        if actual[label] != expected[label]:
            raise ValueError(
                f"result {label} layout does not match manifest model order"
            )
    if not np.all(np.isfinite(np.asarray(result.times_s, dtype=float))):
        raise ValueError("result time grid contains non-finite values")


def _marker_kinematics(
    state: np.ndarray,
    point_local_m: Sequence[float],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    rotation = _rotation_matrices(state[:, _QUATERNION])
    local = np.asarray(point_local_m, dtype=float)
    offset = np.einsum("tij,j->ti", rotation, local)
    position = state[:, _POSITION] + offset
    velocity = state[:, _VELOCITY] + np.cross(state[:, _OMEGA], offset)
    acceleration = (
        state[:, _ACCELERATION]
        + np.cross(state[:, _ALPHA], offset)
        + np.cross(state[:, _OMEGA], np.cross(state[:, _OMEGA], offset))
    )
    return position, velocity, acceleration


def _suspension_deflection(
    sprung_state: np.ndarray,
    sprung_rotation: np.ndarray,
    wheel_position: np.ndarray,
) -> np.ndarray:
    trim_offset_world = wheel_position[0] - sprung_state[0, _POSITION]
    reference_local = sprung_rotation[0].T @ trim_offset_world
    reference_offset = np.einsum(
        "tij,j->ti",
        sprung_rotation,
        reference_local,
    )
    reference_position = sprung_state[:, _POSITION] + reference_offset
    trim_relative = wheel_position[0] - reference_position[0]
    return (wheel_position - reference_position - trim_relative)[:, 2]


def _wheel_spin(
    model: AxleDynamicsModel,
    result: AxleDynamicsResult,
    wheel_body: str,
    joint_name: str,
) -> np.ndarray:
    joint = next(joint for joint in model.joints if joint.name == joint_name)
    if joint.body_a == wheel_body:
        upright_body = joint.body_b
        axis_local = joint.axis_a
    else:
        upright_body = joint.body_a
        axis_local = joint.axis_b
    wheel = result.body_state(wheel_body)
    upright = result.body_state(upright_body)
    rotation = _rotation_matrices(wheel[:, _QUATERNION])
    axis_world = np.einsum("tij,j->ti", rotation, np.asarray(axis_local))
    axis_norm = np.linalg.norm(axis_world, axis=1)
    if np.any(axis_norm <= 1e-12):
        raise ValueError(f"spin joint {joint_name!r} has a zero transformed axis")
    axis_world /= axis_norm[:, None]
    return np.einsum(
        "ti,ti->t",
        wheel[:, _OMEGA] - upright[:, _OMEGA],
        axis_world,
    )


def _conservative_axial_force(spring_state: np.ndarray) -> np.ndarray:
    return spring_state[:, 2] + spring_state[:, 4] + spring_state[:, 5]


def _dissipative_axial_force(spring_state: np.ndarray) -> np.ndarray:
    return spring_state[:, 6] - _conservative_axial_force(spring_state)


def _fixture_wrench(
    model: AxleDynamicsModel,
    result: AxleDynamicsResult,
    bindings: AxleChannelBindings,
    *,
    case: AxleDynamicsCase | None = None,
) -> np.ndarray:
    if case is not None:
        return _fixture_wrench_from_momentum_balance(model, result, case, bindings)
    return _fixture_wrench_from_constraint_multipliers(model, result, bindings)


def _fixture_wrench_from_constraint_multipliers(
    model: AxleDynamicsModel,
    result: AxleDynamicsResult,
    bindings: AxleChannelBindings,
) -> np.ndarray:
    """Fallback for result-only callers that do not carry the input case."""
    marker = bindings.fixture_reference_marker
    fixture_body = marker.body
    reference_position, _, _ = _marker_kinematics(
        result.body_state(fixture_body),
        marker.point_local_m,
    )
    wrench = np.zeros((len(result.times_s), 6), dtype=float)

    for joint in model.joints:
        if fixture_body not in {joint.body_a, joint.body_b}:
            continue
        on_body_b = result.joint_wrench_on_body_b(joint.name)
        if joint.body_b == fixture_body:
            body_force = on_body_b[:, :3]
            body_moment = on_body_b[:, 3:]
            point_local = joint.point_b_m
        else:
            body_force = -on_body_b[:, :3]
            body_moment = -on_body_b[:, 3:]
            point_local = joint.point_a_m
        point, _, _ = _marker_kinematics(
            result.body_state(fixture_body),
            point_local,
        )
        _accumulate_wrench(
            wrench,
            body_force,
            body_moment,
            point,
            reference_position,
        )

    for spring in model.springs:
        if fixture_body not in {spring.body_a, spring.body_b}:
            continue
        point_a, _, _ = _marker_kinematics(
            result.body_state(spring.body_a),
            spring.point_a_m,
        )
        point_b, _, _ = _marker_kinematics(
            result.body_state(spring.body_b),
            spring.point_b_m,
        )
        delta = point_b - point_a
        length = np.linalg.norm(delta, axis=1)
        if np.any(length <= 1e-12):
            raise ValueError(f"spring {spring.name!r} has zero endpoint distance")
        direction = delta / length[:, None]
        force_on_b = direction * result.spring_state(spring.name)[:, 6, None]
        if spring.body_b == fixture_body:
            force = force_on_b
            point = point_b
        else:
            force = -force_on_b
            point = point_a
        _accumulate_wrench(
            wrench,
            force,
            np.zeros_like(force),
            point,
            reference_position,
        )

    for bushing in model.bushings:
        if fixture_body not in {bushing.body_a, bushing.body_b}:
            continue
        body_a = result.body_state(bushing.body_a)
        rotation_a = _rotation_matrices(body_a[:, _QUATERNION])
        frame_a = _quaternion_matrix(
            np.asarray(bushing.frame_a_to_body_quaternion, dtype=float)
        )
        frame_world = np.einsum("tij,jk->tik", rotation_a, frame_a)
        local_wrench = result.bushing_state(bushing.name)[:, 6:12]
        force_on_b = np.einsum(
            "tij,tj->ti",
            frame_world,
            local_wrench[:, :3],
        )
        moment_on_b = np.einsum(
            "tij,tj->ti",
            frame_world,
            local_wrench[:, 3:],
        )
        if bushing.body_b == fixture_body:
            force = force_on_b
            moment = moment_on_b
            point_local = bushing.point_b_m
        else:
            force = -force_on_b
            moment = -moment_on_b
            point_local = bushing.point_a_m
        point, _, _ = _marker_kinematics(
            result.body_state(fixture_body),
            point_local,
        )
        _accumulate_wrench(
            wrench,
            force,
            moment,
            point,
            reference_position,
        )

    for bar in model.anti_roll_bars:
        if fixture_body not in {bar.body_a, bar.body_b}:
            continue
        body_a = result.body_state(bar.body_a)
        rotation_a = _rotation_matrices(body_a[:, _QUATERNION])
        axis_world = np.einsum(
            "tij,j->ti",
            rotation_a,
            np.asarray(bar.axis_a, dtype=float),
        )
        axis_world /= np.linalg.norm(axis_world, axis=1)[:, None]
        torque_on_b = (
            axis_world
            * result.anti_roll_bar_state(bar.name)[:, 2, None]
        )
        torque = torque_on_b if bar.body_b == fixture_body else -torque_on_b
        wrench[:, 3:] += torque
    return wrench


def _fixture_wrench_from_momentum_balance(
    model: AxleDynamicsModel,
    result: AxleDynamicsResult,
    case: AxleDynamicsCase,
    bindings: AxleChannelBindings,
) -> np.ndarray:
    """
    Reconstruct the fixture wrench from a common discrete balance.

    The result is the wrench *on* the fixed fixture.  For the moving bodies,
    ``dP/dt`` and ``dH_ref/dt`` equal the sum of gravity, prescribed inputs,
    tire forces, and the opposite fixture wrench.  Using the accepted public
    states makes the reconstruction identical for native and Adams and avoids
    non-unique multiplier splits from redundant ideal constraints.
    """
    time = np.asarray(result.times_s, dtype=float)
    case_time = np.asarray(case.times_s, dtype=float)
    if time.shape != case_time.shape or not np.allclose(
        time, case_time, rtol=0.0, atol=1e-12
    ):
        raise ValueError(
            "fixture momentum balance requires the common case output grid"
        )
    if time.size < 2:
        raise ValueError("fixture momentum balance requires at least two samples")

    marker = bindings.fixture_reference_marker
    reference_position, _, _ = _marker_kinematics(
        result.body_state(marker.body), marker.point_local_m
    )
    result_body_index = {
        name: index for index, name in enumerate(result.body_names)
    }
    moving = [
        body
        for body in model.bodies
        if not body.fixed
    ]
    masses = np.asarray([body.mass_kg for body in moving], dtype=float)
    linear_momentum = np.zeros((time.size, 3), dtype=float)
    angular_momentum = np.zeros((time.size, 3), dtype=float)
    external_force = np.zeros((time.size, 3), dtype=float)
    external_moment = np.zeros((time.size, 3), dtype=float)
    gravity = np.asarray(model.gravity_m_per_s2, dtype=float)

    for body_offset, body in enumerate(moving):
        index = result_body_index[body.name]
        state = result.states[:, index, :]
        position = state[:, _POSITION]
        velocity = state[:, _VELOCITY]
        omega = state[:, _OMEGA]
        rotation = _rotation_matrices(state[:, _QUATERNION])
        inertia_body = np.asarray(body.inertia_kg_m2, dtype=float)
        inertia_world = np.einsum(
            "tij,jk,tlk->til", rotation, inertia_body, rotation
        )
        linear_momentum += masses[body_offset] * velocity
        angular_momentum += np.cross(
            position - reference_position,
            masses[body_offset] * velocity,
        )
        angular_momentum += np.einsum(
            "tij,tj->ti", inertia_world, omega
        )

        body_gravity = masses[body_offset] * gravity
        external_force += body_gravity
        external_moment += np.cross(
            position - reference_position, body_gravity
        )

        body_wrench = _case_signal(
            case.body_wrench_n_n_m.get(body.name), case_time, time, 6
        )
        body_force = body_wrench[:, :3]
        external_force += body_force
        external_moment += np.cross(
            position - reference_position, body_force
        ) + body_wrench[:, 3:]

    for tire_index, tire in enumerate(model.tires):
        body_index_value = result_body_index[tire.body]
        state = result.states[:, body_index_value, :]
        rotation = _rotation_matrices(state[:, _QUATERNION])
        center = state[:, _POSITION] + np.einsum(
            "tij,j->ti", rotation, np.asarray(tire.center_local_m, dtype=float)
        )
        forward = np.einsum(
            "tij,j->ti", rotation, np.asarray(tire.forward_axis_local, dtype=float)
        )
        forward[:, 2] = 0.0
        forward_norm = np.linalg.norm(forward, axis=1)
        if np.any(forward_norm <= 1e-12):
            raise ValueError(f"tire {tire.name!r} has an invalid world forward axis")
        forward /= forward_norm[:, None]
        normal = np.zeros_like(forward)
        normal[:, 2] = 1.0
        lateral = np.cross(normal, forward)
        lateral /= np.linalg.norm(lateral, axis=1)[:, None]
        tire_state = result.tire_state(tire.name)
        force = (
            forward * tire_state[:, 5, None]
            + lateral * tire_state[:, 6, None]
            + normal * tire_state[:, 4, None]
        )
        contact_point = center.copy()
        contact_point[:, 2] -= tire.unloaded_radius_m
        external_force += force
        external_moment += np.cross(
            contact_point - reference_position, force
        )

        drive = _case_signal(
            case.wheel_torque_n_m.get(tire.name), case_time, time, 1
        )[:, 0]
        spin_axis = np.einsum(
            "tij,j->ti", rotation, np.asarray(tire.spin_axis_local, dtype=float)
        )
        spin_axis /= np.linalg.norm(spin_axis, axis=1)[:, None]
        external_moment += spin_axis * drive[:, None]

    momentum_rate = _time_derivative(linear_momentum, time)
    angular_momentum_rate = _time_derivative(angular_momentum, time)
    return np.column_stack(
        (external_force - momentum_rate, external_moment - angular_momentum_rate)
    )


def _case_signal(
    values: Sequence[Sequence[float]] | Sequence[float] | None,
    case_time: np.ndarray,
    result_time: np.ndarray,
    width: int,
) -> np.ndarray:
    if values is None:
        return np.zeros((result_time.size, width), dtype=float)
    array = np.asarray(values, dtype=float)
    if width == 1 and array.ndim == 1:
        array = array[:, None]
    if array.ndim != 2 or array.shape != (case_time.size, width):
        raise ValueError("case signal shape does not match the public time grid")
    if np.allclose(case_time, result_time, rtol=0.0, atol=1e-12):
        return array.copy()
    return np.column_stack(
        [
            np.interp(result_time, case_time, array[:, column])
            for column in range(width)
        ]
    )


def _time_derivative(values: np.ndarray, time: np.ndarray) -> np.ndarray:
    if values.shape[0] < 3:
        return np.gradient(values, time, axis=0, edge_order=1)
    return np.gradient(values, time, axis=0, edge_order=2)


def _accumulate_wrench(
    total: np.ndarray,
    force: np.ndarray,
    moment_at_point: np.ndarray,
    point: np.ndarray,
    reference: np.ndarray,
) -> None:
    total[:, :3] += force
    total[:, 3:] += moment_at_point + np.cross(point - reference, force)


def _relative_rotation_vectors(
    quaternions: np.ndarray,
    trim_quaternion: np.ndarray,
) -> np.ndarray:
    trim_conjugate = np.asarray(trim_quaternion, dtype=float).copy()
    trim_conjugate[1:] *= -1.0
    return np.asarray(
        [
            _quaternion_log(_quaternion_product(quaternion, trim_conjugate))
            for quaternion in quaternions
        ],
        dtype=float,
    )


def _rotation_matrices(quaternions: np.ndarray) -> np.ndarray:
    return np.asarray(
        [_quaternion_matrix(quaternion) for quaternion in quaternions],
        dtype=float,
    )


def _quaternion_matrix(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("result contains an invalid quaternion")
    w, x, y, z = q / norm
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _quaternion_product(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = np.asarray(left, dtype=float)
    rw, rx, ry, rz = np.asarray(right, dtype=float)
    return np.asarray(
        (
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ),
        dtype=float,
    )


def _quaternion_log(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    q /= np.linalg.norm(q)
    if q[0] < 0.0:
        q = -q
    vector_norm = float(np.linalg.norm(q[1:]))
    if vector_norm <= 1e-14:
        return 2.0 * q[1:]
    angle = 2.0 * math.atan2(vector_norm, float(q[0]))
    return q[1:] * (angle / vector_norm)


def _tuple(values: np.ndarray) -> tuple[float, ...]:
    return tuple(float(value) for value in np.asarray(values, dtype=float))
