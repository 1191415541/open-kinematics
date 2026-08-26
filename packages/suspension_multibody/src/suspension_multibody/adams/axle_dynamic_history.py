"""Convert generated Adams/Solver results to the canonical axle history."""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..axle_dynamics import AxleDynamicsCase, AxleDynamicsModel
from .axle_adams_model import AxleAdamsDataset
from .axle_channels import axle_history_from_result
from .axle_contract import AxleChannelBindings
from .time_domain import AdamsResultChannel, TimeHistory, parse_adams_result_history

ADAMS_AXLE_HISTORY_CONTRACT = "dynamic-axle-adams-canonical-history-v1"

_BODY_POSE_COMPONENTS = ("X", "Y", "Z", "PSI", "THETA", "PHI")
_BODY_RATE_COMPONENTS = ("VX", "VY", "VZ", "WX", "WY", "WZ")
_BODY_ACCELERATION_COMPONENTS = (
    "ACCX",
    "ACCY",
    "ACCZ",
    "WDX",
    "WDY",
    "WDZ",
)
_JOINT_COMPONENTS = ("FX", "FY", "FZ", "TX", "TY", "TZ")
_SPRING_COMPONENTS = ("FX", "FY", "FZ")
_TIRE_VARIABLES = (
    "penetration",
    "penetration_rate",
    "normal_force",
    "longitudinal_force",
    "lateral_force",
    "longitudinal_slip",
    "lateral_slip",
    "friction_utilization",
    "brush_x",
    "brush_y",
)


@dataclass(frozen=True)
class AdamsAxleResult:
    """The result protocol consumed by :func:`axle_history_from_result`."""

    times_s: np.ndarray
    body_names: tuple[str, ...]
    constraint_names: tuple[str, ...]
    spring_names: tuple[str, ...]
    bushing_names: tuple[str, ...]
    anti_roll_bar_names: tuple[str, ...]
    tire_names: tuple[str, ...]
    states: np.ndarray
    constraint_wrench: np.ndarray
    spring_output: np.ndarray
    bushing_output: np.ndarray
    anti_roll_output: np.ndarray
    tire_output: np.ndarray

    def body_state(self, body: str) -> np.ndarray:
        return self.states[:, self.body_names.index(body), :]

    def tire_state(self, tire: str) -> np.ndarray:
        return self.tire_output[:, self.tire_names.index(tire), :]

    def joint_wrench_on_body_b(self, joint: str) -> np.ndarray:
        return self.constraint_wrench[:, self.constraint_names.index(joint), :]

    def spring_state(self, spring: str) -> np.ndarray:
        return self.spring_output[:, self.spring_names.index(spring), :]

    def bushing_state(self, bushing: str) -> np.ndarray:
        return self.bushing_output[:, self.bushing_names.index(bushing), :]

    def anti_roll_bar_state(self, bar: str) -> np.ndarray:
        return self.anti_roll_output[:, self.anti_roll_bar_names.index(bar), :]


def adams_axle_raw_channel_map(
    model: AxleDynamicsModel,
    dataset: AxleAdamsDataset | Mapping[str, object],
) -> dict[str, AdamsResultChannel]:
    """
    Return standard Adams result entities needed for canonical export.

    The generated REQUEST blocks are intentionally not used here: Adams 2024.1
    retains their time columns but may omit expression values in a ``.res``
    step.  Body states are emitted as explicit CM-marker ``VARIABLE`` entities;
    ``PART_XFORM`` is not a CM state and is therefore not used for canonical
    body kinematics.
    """
    ids = _entity_ids(dataset)
    channels: dict[str, AdamsResultChannel] = {}
    for body in model.bodies:
        if body.fixed:
            continue
        for component in (
            *_BODY_POSE_COMPONENTS,
            *_BODY_RATE_COMPONENTS,
            *_BODY_ACCELERATION_COMPONENTS,
        ):
            variable_id = _entity_id(ids, f"body:{body.name}:state:{component}")
            channels[f"body:{body.name}:{component}"] = AdamsResultChannel(
                f"VARIABLE_{variable_id}", "Q"
            )

    for joint in model.joints:
        joint_id = _entity_id(ids, f"joint:{joint.name}")
        entity = f"JPRIM_{joint_id}" if joint.kind == "inplane" else f"JOINT_{joint_id}"
        for component in _JOINT_COMPONENTS:
            channels[f"joint:{joint.name}:{component}"] = AdamsResultChannel(
                entity, component
            )

    for spring in model.springs:
        spring_id = _entity_id(ids, f"spring:{spring.name}")
        entity = f"SFORCE_{spring_id}"
        for component in _SPRING_COMPONENTS:
            channels[f"sforce:{spring.name}:{component}"] = AdamsResultChannel(
                entity, component
            )

    for tire in model.tires:
        for variable in _TIRE_VARIABLES:
            variable_id = _entity_id(ids, f"tire:{tire.name}:{variable}")
            entity_prefix = "DIFF" if variable in {"brush_x", "brush_y"} else "VARIABLE"
            channels[f"tire:{tire.name}:{variable}"] = AdamsResultChannel(
                f"{entity_prefix}_{variable_id}", "Q"
            )
    return channels


def adams_axle_history_from_result(
    model: AxleDynamicsModel,
    bindings: AxleChannelBindings,
    dataset: AxleAdamsDataset | Mapping[str, object],
    result_path: str | Path,
    *,
    case: AxleDynamicsCase | None = None,
) -> TimeHistory:
    """Parse one real Adams result and evaluate all frozen axle channels."""
    result = adams_axle_result_from_result(model, dataset, result_path)
    return axle_history_from_result(model, result, bindings, case=case)


def adams_axle_result_from_result(
    model: AxleDynamicsModel,
    dataset: AxleAdamsDataset | Mapping[str, object],
    result_path: str | Path,
) -> AdamsAxleResult:
    """Parse one real Adams result into the raw canonical element layout."""
    raw = parse_adams_result_history(
        result_path,
        adams_axle_raw_channel_map(model, dataset),
    )
    return _build_result(model, dataset, raw)


def _build_result(
    model: AxleDynamicsModel,
    dataset: AxleAdamsDataset | Mapping[str, object],
    raw: TimeHistory,
) -> AdamsAxleResult:
    time = np.asarray(raw.time, dtype=float)
    body_names = tuple(body.name for body in model.bodies)
    body_index = {name: index for index, name in enumerate(body_names)}
    states = np.zeros((len(time), len(body_names), 19), dtype=float)

    for index, body in enumerate(model.bodies):
        if body.fixed:
            states[:, index, :3] = np.asarray(body.position_m, dtype=float)
            states[:, index, 3:7] = np.asarray(
                body.quaternion_body_to_world, dtype=float
            )
            continue
        def values(component: str) -> np.ndarray:
            return _raw_values(raw, f"body:{body.name}:{component}")

        states[:, index, :3] = np.column_stack(
            [values(component) for component in ("X", "Y", "Z")]
        )
        states[:, index, 3:7] = np.asarray(
            [
                _quaternion_from_euler_313(psi, theta, phi)
                for psi, theta, phi in zip(
                    values("PSI"), values("THETA"), values("PHI")
                )
            ],
            dtype=float,
        )
        if time.size and abs(float(time[0])) <= 1.0e-12:
            # The shared manifest is exact at t=0. Adams formatted Euler
            # output is only a finite-precision representation of that state.
            states[0, index, 3:7] = np.asarray(
                body.quaternion_body_to_world,
                dtype=float,
            )
        states[:, index, 7:10] = np.column_stack(
            [values(component) for component in ("VX", "VY", "VZ")]
        )
        states[:, index, 10:13] = np.column_stack(
            [values(component) for component in ("WX", "WY", "WZ")]
        )
        states[:, index, 13:16] = np.column_stack(
            [values(component) for component in ("ACCX", "ACCY", "ACCZ")]
        )
        states[:, index, 16:19] = np.column_stack(
            [values(component) for component in ("WDX", "WDY", "WDZ")]
        )

    constraint_names = tuple(joint.name for joint in model.joints)
    constraint_wrench = np.zeros(
        (len(time), len(constraint_names), len(_JOINT_COMPONENTS)), dtype=float
    )
    for index, joint in enumerate(model.joints):
        constraint_wrench[:, index, :] = np.column_stack(
            [_raw_values(raw, f"joint:{joint.name}:{component}") for component in _JOINT_COMPONENTS]
        )

    spring_names = tuple(spring.name for spring in model.springs)
    spring_output = _spring_outputs(model, raw, states, body_index)
    tire_names = tuple(tire.name for tire in model.tires)
    tire_output = _tire_outputs(model, raw)
    return AdamsAxleResult(
        times_s=time,
        body_names=body_names,
        constraint_names=constraint_names,
        spring_names=spring_names,
        bushing_names=tuple(bushing.name for bushing in model.bushings),
        anti_roll_bar_names=tuple(bar.name for bar in model.anti_roll_bars),
        tire_names=tire_names,
        states=states,
        constraint_wrench=constraint_wrench,
        spring_output=spring_output,
        bushing_output=np.zeros((len(time), len(model.bushings), 12), dtype=float),
        anti_roll_output=np.zeros(
            (len(time), len(model.anti_roll_bars), 3), dtype=float
        ),
        tire_output=tire_output,
    )


def _spring_outputs(
    model: AxleDynamicsModel,
    raw: TimeHistory,
    states: np.ndarray,
    body_index: Mapping[str, int],
) -> np.ndarray:
    output = np.zeros((len(raw.time), len(model.springs), 7), dtype=float)
    for index, spring in enumerate(model.springs):
        position_a, velocity_a = _point_kinematics(
            states, body_index[spring.body_a], spring.point_a_m
        )
        position_b, velocity_b = _point_kinematics(
            states, body_index[spring.body_b], spring.point_b_m
        )
        delta = position_b - position_a
        length = np.linalg.norm(delta, axis=1)
        if np.any(length <= 1e-12):
            raise ValueError(f"spring {spring.name!r} has zero endpoint distance")
        direction = delta / length[:, None]
        length_rate = np.einsum("ij,ij->i", velocity_b - velocity_a, direction)
        elastic = spring.stiffness_n_per_m * (spring.free_length_m - length)
        compression_stop = np.zeros_like(length)
        if spring.minimum_length_m is not None:
            compression_stop = spring.compression_stop_stiffness_n_per_m * np.maximum(
                0.0, spring.minimum_length_m - length
            )
        rebound_stop = np.zeros_like(length)
        if spring.maximum_length_m is not None:
            rebound_stop = -spring.rebound_stop_stiffness_n_per_m * np.maximum(
                0.0, length - spring.maximum_length_m
            )
        force = np.column_stack(
            [
                _raw_values(raw, f"sforce:{spring.name}:{component}")
                for component in _SPRING_COMPONENTS
            ]
        )
        total = np.einsum("ij,ij->i", force, direction)
        damping = total - elastic - compression_stop - rebound_stop
        output[:, index, :] = np.column_stack(
            (
                length,
                length_rate,
                elastic,
                damping,
                compression_stop,
                rebound_stop,
                total,
            )
        )
    return output


def _tire_outputs(model: AxleDynamicsModel, raw: TimeHistory) -> np.ndarray:
    output = np.zeros((len(raw.time), len(model.tires), 12), dtype=float)
    for index, tire in enumerate(model.tires):
        def values(name: str) -> np.ndarray:
            return _raw_values(raw, f"tire:{tire.name}:{name}")

        penetration = values("penetration")
        normal_force = values("normal_force")
        output[:, index, :] = np.column_stack(
            (
                (normal_force > 0.0).astype(float),
                -penetration,
                np.maximum(0.0, penetration),
                -values("penetration_rate"),
                normal_force,
                values("longitudinal_force"),
                values("lateral_force"),
                values("longitudinal_slip"),
                values("lateral_slip"),
                values("friction_utilization"),
                values("brush_x"),
                values("brush_y"),
            )
        )
    return output


def _point_kinematics(
    states: np.ndarray,
    body_index: int,
    point_local: tuple[float, float, float],
) -> tuple[np.ndarray, np.ndarray]:
    state = states[:, body_index, :]
    local = np.asarray(point_local, dtype=float)
    offset = np.asarray(
        [_rotation_matrix(quaternion) @ local for quaternion in state[:, 3:7]],
        dtype=float,
    )
    position = state[:, :3] + offset
    velocity = state[:, 7:10] + np.cross(state[:, 10:13], offset)
    return position, velocity


def _quaternion_from_euler_313(psi: float, theta: float, phi: float) -> np.ndarray:
    c_psi, s_psi = math.cos(psi), math.sin(psi)
    c_theta, s_theta = math.cos(theta), math.sin(theta)
    c_phi, s_phi = math.cos(phi), math.sin(phi)
    matrix = np.asarray(
        (
            (
                c_psi * c_phi - s_psi * c_theta * s_phi,
                -c_psi * s_phi - s_psi * c_theta * c_phi,
                s_psi * s_theta,
            ),
            (
                s_psi * c_phi + c_psi * c_theta * s_phi,
                -s_psi * s_phi + c_psi * c_theta * c_phi,
                -c_psi * s_theta,
            ),
            (s_theta * s_phi, s_theta * c_phi, c_theta),
        ),
        dtype=float,
    )
    return _quaternion_from_matrix(matrix)


def _quaternion_from_matrix(matrix: np.ndarray) -> np.ndarray:
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quaternion = np.asarray(
            (
                0.25 * scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ),
            dtype=float,
        )
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = 2.0 * math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
        quaternion = np.asarray(
            (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            ),
            dtype=float,
        )
    elif matrix[1, 1] > matrix[2, 2]:
        scale = 2.0 * math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
        quaternion = np.asarray(
            (
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            ),
            dtype=float,
        )
    else:
        scale = 2.0 * math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
        quaternion = np.asarray(
            (
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            ),
            dtype=float,
        )
    norm = float(np.linalg.norm(quaternion))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Adams result contains an invalid orientation")
    return quaternion / norm


def _rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(q))
    if not math.isfinite(norm) or norm <= 1e-12:
        raise ValueError("Adams result contains an invalid quaternion")
    w, x, y, z = q / norm
    return np.asarray(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)),
            (2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)),
            (2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def _raw_values(history: TimeHistory, name: str) -> np.ndarray:
    try:
        return np.asarray(history.channels[name], dtype=float)
    except KeyError as exc:
        raise ValueError(f"Adams raw channel is missing: {name}") from exc


def _entity_ids(
    dataset: AxleAdamsDataset | Mapping[str, object],
) -> Mapping[str, object]:
    if isinstance(dataset, AxleAdamsDataset):
        return dataset.entity_ids
    entity_ids = dataset.get("entity_ids")
    if not isinstance(entity_ids, Mapping):
        raise ValueError("Adams dataset entity_ids must be a mapping")
    return entity_ids


def _entity_id(ids: Mapping[str, object], key: str) -> int:
    value = ids.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError(f"Adams dataset entity id is invalid: {key}")
    return value
