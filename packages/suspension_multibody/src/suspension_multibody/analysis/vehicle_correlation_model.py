"""Independent 14/15-DOF full-vehicle model used for Adams correlation."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass, fields
from pathlib import Path
from typing import TYPE_CHECKING, Literal, cast

import numpy as np

from ..dynamics import TireKinematics, TireModel, tire_model_from_spec
from ..schema import TireModelSpec

if TYPE_CHECKING:
    from ..adams.time_domain import TimeHistory

VehicleDof = Literal[14, 15]

_DURATION_S = {
    "steady_state_circle": 17.0,
    "step_steer": 5.0,
    "sine_steer": 6.0,
    "double_lane_change": 12.0,
    "single_wheel_bump": 4.0,
    "double_wheel_bump": 4.0,
    "random_road": 8.0,
    "four_post_rig": 4.0,
}
_HANDLING = frozenset(
    ("steady_state_circle", "step_steer", "sine_steer", "double_lane_change")
)
_FOUR_POST_CORNERS = (
    "jms_post_pad_vertical_lf",
    "jms_post_pad_vertical_rf",
    "jms_post_pad_vertical_lr",
    "jms_post_pad_vertical_rr",
)
_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_STEP_DIFFERENCE = re.compile(
    rf"^STEP\(TIME,({_NUMBER}),({_NUMBER}),({_NUMBER}),({_NUMBER})\)"
    rf"-STEP\(TIME,({_NUMBER}),({_NUMBER}),({_NUMBER}),({_NUMBER})\)$"
)
_SINE_DEGREES = re.compile(rf"^({_NUMBER})\*sin\(({_NUMBER})d\*TIME\)$")


@dataclass(frozen=True)
class Vehicle14DofParameters:
    """SI parameters for 6 body, 4 wheel-vertical, and 4 wheel-spin coordinates."""

    mass: float = 1_527.680888
    sprung_mass: float = 1_295.680888
    unsprung_mass: float = 58.0
    inertia_roll: float = 298.8515286
    inertia_pitch: float = 1_162.028448
    inertia_yaw: float = 1_340.386004
    wheelbase: float = 2.56
    front_axle_to_cg: float = 1.481397767
    rear_axle_to_cg: float = 1.078602233
    front_track: float = 1.52
    rear_track: float = 1.594
    center_of_mass_height: float = 0.55
    suspension_stiffness: float = 30_000.0
    suspension_damping: float = 3_000.0
    tire_vertical_stiffness: float = 180_000.0
    roll_stiffness: float = 55_000.0
    roll_damping: float = 5_500.0
    pitch_stiffness: float = 70_000.0
    pitch_damping: float = 6_500.0
    steering_ratio: float = 27.6
    steering_time_constant: float = 0.01
    wheel_radius: float = 0.31
    front_cornering_stiffness: float = 70_000.0
    rear_cornering_stiffness: float = 100_000.0
    tire_friction_coefficient: float = 0.9

    def __post_init__(self) -> None:
        if min(
            self.mass,
            self.sprung_mass,
            self.unsprung_mass,
            self.inertia_roll,
            self.inertia_pitch,
            self.inertia_yaw,
            self.wheelbase,
            self.front_axle_to_cg,
            self.rear_axle_to_cg,
            self.front_track,
            self.rear_track,
            self.suspension_stiffness,
            self.tire_vertical_stiffness,
            self.wheel_radius,
            self.front_cornering_stiffness,
            self.rear_cornering_stiffness,
            self.tire_friction_coefficient,
        ) <= 0.0:
            raise ValueError("vehicle correlation parameters must be positive")
        if self.sprung_mass + 4.0 * self.unsprung_mass > self.mass:
            raise ValueError("sprung and unsprung masses exceed vehicle mass")
        if not math.isclose(
            self.front_axle_to_cg + self.rear_axle_to_cg,
            self.wheelbase,
            abs_tol=1e-9,
        ):
            raise ValueError("front and rear axle distances must equal wheelbase")


@dataclass(frozen=True)
class VehicleCorrelationRun:
    """Response plus auditable proof that only input-manifest data was consumed."""

    case: str
    degrees_of_freedom: VehicleDof
    history: TimeHistory
    input_manifest_hash: str
    source_access_audit: tuple[str, ...]


def simulate_vehicle_correlation_case(
    case: str,
    *,
    degrees_of_freedom: VehicleDof = 15,
    tire_model: Literal["fiala", "pac2002"] = "pac2002",
    parameters: Vehicle14DofParameters = Vehicle14DofParameters(),
    input_manifest: str | Path | Mapping[str, object] | None = None,
) -> VehicleCorrelationRun:
    """Simulate one frozen maneuver without opening Adams response files."""
    from ..adams.time_domain import TimeHistory

    if case not in _DURATION_S:
        raise ValueError(f"unsupported vehicle correlation case: {case}")
    manifest, manifest_hash, audit = _load_manifest(case, input_manifest)
    four_post_functions = _four_post_functions(manifest)
    duration = _manifest_float(manifest, "duration_s", _DURATION_S[case])
    step = _manifest_float(manifest, "output_step_s", 0.01)
    if duration <= 0.0:
        raise ValueError("input-manifest duration must be positive")
    if not math.isclose(step, 0.01, abs_tol=1e-12):
        raise ValueError("input-manifest output step differs from the frozen case")
    front_tire = tire_model_from_spec(
        TireModelSpec(
            kind=tire_model,
            parameter_source="adams_builtin" if tire_model == "pac2002" else "user",
            cornering_stiffness=parameters.front_cornering_stiffness,
            longitudinal_stiffness=120_000.0,
            friction_coefficient=parameters.tire_friction_coefficient,
            vertical_stiffness=parameters.tire_vertical_stiffness,
        )
    )
    rear_tire = tire_model_from_spec(
        TireModelSpec(
            kind=tire_model,
            parameter_source="adams_builtin" if tire_model == "pac2002" else "user",
            cornering_stiffness=parameters.rear_cornering_stiffness,
            longitudinal_stiffness=120_000.0,
            friction_coefficient=parameters.tire_friction_coefficient,
            vertical_stiffness=parameters.tire_vertical_stiffness,
        )
    )
    count = int(round(duration / step)) + 1
    time = tuple(index * step for index in range(count))
    steering_input = _steering_input(case, manifest, count, step)
    parameters = _vehicle_parameters(case, manifest, parameters)
    q = np.zeros(int(degrees_of_freedom), dtype=float)
    dq = np.zeros_like(q)
    channels = {name: [0.0] * count for name in _channel_names(case)}
    for index, current_time in enumerate(time):
        output, ddq = _evaluate(
            case,
            current_time,
            q,
            dq,
            degrees_of_freedom,
            parameters,
            front_tire,
            rear_tire,
            four_post_functions,
            steering_input,
        )
        for name, value in output.items():
            channels[name][index] = value
        if index + 1 < count:
            _step(
                case,
                current_time,
                step,
                q,
                dq,
                ddq,
                degrees_of_freedom,
                parameters,
                steering_input,
            )
    return VehicleCorrelationRun(
        case=case,
        degrees_of_freedom=degrees_of_freedom,
        history=TimeHistory(
            time=time,
            channels={name: tuple(values) for name, values in channels.items()},
            units=_units(case),
        ),
        input_manifest_hash=manifest_hash,
        source_access_audit=audit,
    )


def _evaluate(
    case: str,
    time: float,
    q: np.ndarray,
    dq: np.ndarray,
    degrees_of_freedom: VehicleDof,
    parameters: Vehicle14DofParameters,
    front_tire: TireModel,
    rear_tire: TireModel,
    four_post_functions: tuple[str, str, str, str] | None,
    steering_input: tuple[float, ...] | None,
) -> tuple[dict[str, float], np.ndarray]:
    ddq = np.zeros_like(q)
    speed = _speed(case, time)
    driver_steering = _driver_steering(case, time, parameters, steering_input)
    steering = q[14] if degrees_of_freedom == 15 else driver_steering
    road_wheel_angle = steering / parameters.steering_ratio
    lateral_velocity, yaw_rate = dq[1], dq[5]
    front_static_load = (
        parameters.mass
        * 9.81
        * parameters.rear_axle_to_cg
        / parameters.wheelbase
        / 2.0
    )
    rear_static_load = (
        parameters.mass
        * 9.81
        * parameters.front_axle_to_cg
        / parameters.wheelbase
        / 2.0
    )
    front_slip = (
        math.atan2(
            lateral_velocity + parameters.front_axle_to_cg * yaw_rate, speed
        )
        - road_wheel_angle
    )
    rear_slip = math.atan2(
        lateral_velocity - parameters.rear_axle_to_cg * yaw_rate, speed
    )
    normal_loads = (
        front_static_load - parameters.tire_vertical_stiffness * q[6],
        front_static_load - parameters.tire_vertical_stiffness * q[7],
        rear_static_load - parameters.tire_vertical_stiffness * q[8],
        rear_static_load - parameters.tire_vertical_stiffness * q[9],
    )
    front_force = sum(
        front_tire.evaluate(
            TireKinematics(normal_load=normal_load, slip_angle=front_slip)
        ).fy
        for normal_load in normal_loads[:2]
    )
    rear_force = sum(
        rear_tire.evaluate(
            TireKinematics(normal_load=normal_load, slip_angle=rear_slip)
        ).fy
        for normal_load in normal_loads[2:]
    )
    lateral_force = front_force + rear_force
    ddq[1] = lateral_force / parameters.mass - speed * yaw_rate
    ddq[5] = (
        parameters.front_axle_to_cg * front_force
        - parameters.rear_axle_to_cg * rear_force
    ) / parameters.inertia_yaw
    vertical_force, roll_moment, pitch_moment = _vertical_loads(case, time, q, dq, parameters)
    ddq[2] = vertical_force / parameters.sprung_mass
    ddq[3] = (
        -parameters.center_of_mass_height * lateral_force
        + roll_moment
        - parameters.roll_stiffness * q[3]
        - parameters.roll_damping * dq[3]
    ) / parameters.inertia_roll
    ddq[4] = (
        pitch_moment
        - parameters.pitch_stiffness * q[4]
        - parameters.pitch_damping * dq[4]
    ) / parameters.inertia_pitch
    for index in range(4):
        ddq[6 + index] = _wheel_vertical_acceleration(
            case,
            time,
            index,
            q,
            dq,
            parameters,
            four_post_functions,
        )
        ddq[10 + index] = (speed / parameters.wheel_radius - dq[10 + index]) / 0.02
    output = (
        {
            "steering_angle": driver_steering,
            "lateral_acceleration": ddq[1] + speed * yaw_rate,
            "yaw_rate": yaw_rate,
            "body_roll": q[3],
        }
        if case in _HANDLING
        else {
            "body_accel_z": ddq[2],
            "body_heave": q[2],
            "body_pitch": q[4],
            "body_roll": q[3],
        }
    )
    return output, ddq


def _step(
    case: str,
    time: float,
    step: float,
    q: np.ndarray,
    dq: np.ndarray,
    ddq: np.ndarray,
    degrees_of_freedom: VehicleDof,
    parameters: Vehicle14DofParameters,
    steering_input: tuple[float, ...] | None,
) -> None:
    dq += ddq * step
    q += dq * step
    dq[0] = _speed(case, time)
    q[0] += dq[0] * step
    if degrees_of_freedom == 15:
        target = _driver_steering(case, time + step, parameters, steering_input)
        dq[14] = (target - q[14]) / parameters.steering_time_constant
        q[14] += dq[14] * step


def _vertical_loads(
    case: str, time: float, q: np.ndarray, dq: np.ndarray, parameters: Vehicle14DofParameters
) -> tuple[float, float, float]:
    total = roll = pitch = 0.0
    for index, (x, y) in enumerate(_corners(parameters)):
        body_z = q[2] + y * q[3] - x * q[4]
        body_dz = dq[2] + y * dq[3] - x * dq[4]
        force = -parameters.suspension_stiffness * (body_z - q[6 + index])
        force -= parameters.suspension_damping * (body_dz - dq[6 + index])
        total += force
        roll += y * force
        pitch -= x * force
    return total, roll, pitch


def _wheel_vertical_acceleration(
    case: str,
    time: float,
    index: int,
    q: np.ndarray,
    dq: np.ndarray,
    parameters: Vehicle14DofParameters,
    four_post_functions: tuple[str, str, str, str] | None,
) -> float:
    x, y = _corners(parameters)[index]
    body_z = q[2] + y * q[3] - x * q[4]
    body_dz = dq[2] + y * dq[3] - x * dq[4]
    force_on_body = -parameters.suspension_stiffness * (body_z - q[6 + index])
    force_on_body -= parameters.suspension_damping * (body_dz - dq[6 + index])
    tire_force = parameters.tire_vertical_stiffness * (
        _road(case, time, index, four_post_functions) - q[6 + index]
    )
    return (-force_on_body + tire_force) / parameters.unsprung_mass


def _corners(parameters: Vehicle14DofParameters) -> tuple[tuple[float, float], ...]:
    front, rear = parameters.front_axle_to_cg, -parameters.rear_axle_to_cg
    return (
        (front, 0.5 * parameters.front_track),
        (front, -0.5 * parameters.front_track),
        (rear, 0.5 * parameters.rear_track),
        (rear, -0.5 * parameters.rear_track),
    )


def _speed(case: str, time: float) -> float:
    if case == "steady_state_circle":
        return 16.666 if time < 3.8 else min(27.777, 16.666 + 0.855 * (time - 3.8))
    return 16.667 if case in _HANDLING else 12.0


def _driver_steering(
    case: str,
    time: float,
    parameters: Vehicle14DofParameters,
    steering_input: tuple[float, ...] | None,
) -> float:
    if steering_input is not None:
        return _sampled_value(steering_input, time)
    if case == "step_steer":
        return 1.57 * _ramp(time, 1.0, 2.0)
    if case == "sine_steer":
        return (
            0.03 * math.sin(math.pi * (time - 0.5))
            if 0.5 <= time <= 2.5
            else 0.0
        )
    if case == "steady_state_circle":
        return parameters.steering_ratio * math.atan(parameters.wheelbase / 80.0) if time >= 3.8 else 0.0
    if case == "double_lane_change":
        return 0.42 * (_smooth(time, 3.5, 5.4) - 2 * _smooth(time, 5.4, 6.9) + _smooth(time, 6.9, 8.5))
    return 0.0


def _sampled_value(samples: tuple[float, ...], time: float) -> float:
    position = max(0.0, time / 0.01)
    lower = min(int(math.floor(position)), len(samples) - 1)
    upper = min(lower + 1, len(samples) - 1)
    fraction = position - lower
    return samples[lower] + fraction * (samples[upper] - samples[lower])


def _road(
    case: str,
    time: float,
    index: int,
    four_post_functions: tuple[str, str, str, str] | None = None,
) -> float:
    if four_post_functions is not None:
        return _adams_four_post_displacement(four_post_functions[index], time)
    bump = 0.03 * (_ramp(time, 0.0, 0.05) - _ramp(time, 0.15, 0.2))
    if case == "single_wheel_bump":
        return bump if index == 0 else 0.0
    if case == "double_wheel_bump":
        return bump if index < 2 else 0.0
    if case == "four_post_rig":
        return 0.01 * math.sin(math.pi * time)
    if case == "random_road":
        phase = (0.0, 0.7, 1.3, 2.1)[index]
        return 0.004 * (math.sin(2 * math.pi * 1.3 * time + phase) + 0.6 * math.sin(2 * math.pi * 3.7 * time + 1.7 * phase))
    return 0.0


def _four_post_functions(
    manifest: Mapping[str, object],
) -> tuple[str, str, str, str] | None:
    value = manifest.get("four_post_functions")
    if value is None:
        return None
    if not isinstance(value, Mapping):
        raise ValueError("four_post_functions must be an object")
    value = cast(Mapping[str, object], value)
    functions: list[str] = []
    for corner in _FOUR_POST_CORNERS:
        function = value.get(corner)
        if not isinstance(function, str):
            raise ValueError(f"four_post_functions.{corner} must be a string")
        functions.append(function)
    return cast(tuple[str, str, str, str], tuple(functions))


def _steering_input(
    case: str,
    manifest: Mapping[str, object],
    count: int,
    step: float,
) -> tuple[float, ...] | None:
    value = manifest.get("steering_input")
    if value is None:
        if case in _HANDLING and manifest.get("schema") == "vehicle-adams-case-input-v1":
            raise ValueError("handling input manifest is missing steering_input")
        return None
    if case not in _HANDLING:
        raise ValueError("steering_input is only valid for handling cases")
    if not isinstance(value, Mapping):
        raise ValueError("steering_input must be an object")
    value = cast(Mapping[str, object], value)
    if value.get("kind") != "sampled_driver_demand":
        raise ValueError("steering_input.kind must be sampled_driver_demand")
    sample_period = value.get("sample_period_s")
    if not isinstance(sample_period, (float, int)) or not math.isclose(
        float(sample_period), step, abs_tol=1e-12
    ):
        raise ValueError("steering_input.sample_period_s differs from output_step_s")
    samples = value.get("angle_rad")
    if not isinstance(samples, list) or len(samples) != count:
        raise ValueError("steering_input.angle_rad does not match the frozen time grid")
    if not all(isinstance(sample, (float, int)) and math.isfinite(sample) for sample in samples):
        raise ValueError("steering_input.angle_rad must contain finite numeric values")
    return tuple(float(sample) for sample in samples)


def _vehicle_parameters(
    case: str,
    manifest: Mapping[str, object],
    fallback: Vehicle14DofParameters,
) -> Vehicle14DofParameters:
    value = manifest.get("vehicle_model_parameters")
    if value is None:
        if case in _HANDLING and manifest.get("schema") == "vehicle-adams-case-input-v1":
            raise ValueError("handling input manifest is missing vehicle_model_parameters")
        return fallback
    if not isinstance(value, Mapping):
        raise ValueError("vehicle_model_parameters must be an object")
    value = cast(Mapping[str, object], value)
    expected = {field.name for field in fields(Vehicle14DofParameters)}
    if set(value) != expected:
        raise ValueError("vehicle_model_parameters fields differ from the 14/15-DOF contract")
    if not all(
        isinstance(item, (float, int)) and math.isfinite(item)
        for item in value.values()
    ):
        raise ValueError("vehicle_model_parameters must contain finite numeric values")
    return Vehicle14DofParameters(
        **{str(name): float(item) for name, item in value.items()}
    )


def _adams_four_post_displacement(function: str, time: float) -> float:
    """
    Evaluate retained Adams four-post expressions as SI pad displacement.

    The captured ACF expressions are in the Adams/Car native millimetre system;
    their result is converted to metres at this boundary.
    """
    expression = function.replace(" ", "")
    if expression in {"0", "0.0"}:
        return 0.0
    match = _STEP_DIFFERENCE.fullmatch(expression)
    if match is not None:
        values = tuple(float(value) for value in match.groups())
        return 1e-3 * (
            _step_expression(time, *values[:4])
            - _step_expression(time, *values[4:])
        )
    match = _SINE_DEGREES.fullmatch(expression)
    if match is not None:
        amplitude_mm, frequency_deg_per_second = (
            float(value) for value in match.groups()
        )
        return 1e-3 * amplitude_mm * math.sin(
            math.radians(frequency_deg_per_second) * time
        )
    raise ValueError(f"unsupported Adams four-post function: {function!r}")


def _step_expression(
    time: float,
    start_time: float,
    start_value: float,
    end_time: float,
    end_value: float,
) -> float:
    return start_value + (end_value - start_value) * _ramp(
        time, start_time, end_time
    )


def _channel_names(case: str) -> tuple[str, ...]:
    return (
        ("steering_angle", "lateral_acceleration", "yaw_rate", "body_roll")
        if case in _HANDLING
        else ("body_accel_z", "body_heave", "body_pitch", "body_roll")
    )


def _units(case: str) -> dict[str, str]:
    return (
        {"steering_angle": "rad", "lateral_acceleration": "m/s^2", "yaw_rate": "rad/s", "body_roll": "rad"}
        if case in _HANDLING
        else {"body_accel_z": "m/s^2", "body_heave": "m", "body_pitch": "rad", "body_roll": "rad"}
    )


def _load_manifest(
    case: str, source: str | Path | Mapping[str, object] | None
) -> tuple[Mapping[str, object], str, tuple[str, ...]]:
    if source is None:
        manifest: Mapping[str, object] = {"case": case, "duration_s": _DURATION_S[case], "output_step_s": 0.01}
        return manifest, _hash(manifest), ("no_adams_reference_history_access",)
    if isinstance(source, Mapping):
        manifest = cast(Mapping[str, object], source)
        audit = ("mapping_input_only", "no_adams_reference_history_access")
    else:
        path = Path(source)
        if "reference_bundle" in path.name:
            raise ValueError("package simulation requires an input manifest, not reference data")
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, Mapping):
            raise ValueError("vehicle input manifest root must be an object")
        manifest = cast(Mapping[str, object], payload)
        audit = (f"input_manifest={path.name}", "no_adams_reference_history_access")
    if manifest.get("case") != case:
        raise ValueError("vehicle input manifest case does not match requested case")
    return manifest, _hash(manifest), audit


def _manifest_float(manifest: Mapping[str, object], name: str, default: float) -> float:
    value = manifest.get(name, default)
    if not isinstance(value, (float, int)):
        raise ValueError(f"vehicle input manifest {name} must be numeric")
    return float(value)


def _hash(manifest: Mapping[str, object]) -> str:
    return hashlib.sha256(json.dumps(manifest, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def _ramp(time: float, start: float, end: float) -> float:
    return 0.0 if time <= start else 1.0 if time >= end else (time - start) / (end - start)


def _smooth(time: float, start: float, end: float) -> float:
    value = _ramp(time, start, end)
    return value * value * (3.0 - 2.0 * value)
