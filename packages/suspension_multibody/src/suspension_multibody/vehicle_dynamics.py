"""原生整车多体动力学入口."""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from . import __version__
from .axle_dynamics.native import (
    _run_native,
    _VehicleRoadBuffers,
    _VehicleSteeringBuffers,
)
from .axle_dynamics.result import AxleDynamicsResult
from .axle_dynamics.schema import (
    AxleAerodynamicDrag,
    AxleBody,
    AxleBushing,
    AxleCoordinateCoupler,
    AxleDynamicsCase,
    AxleJoint,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
)
from .core import (
    BallJoint,
    ConstantVelocityJoint,
    CoordinateDrive,
    CylindricalJoint,
    DistanceConstraint,
    InPlaneJoint,
    PointCoincidence,
    PrismaticJoint,
    RevoluteJoint,
    UniversalJoint,
    WeldJoint,
)
from .elements import (
    AntiRollBarElement,
    BumpStopElement,
    BushingElement,
    LinearSpringElement,
    StaticDamperElement,
    VerticalTireElement,
)
from .io import canonical_hash
from .model import VehicleAssembly, build_vehicle
from .schema import (
    DynamicSolverSettings,
    RoadSurfaceSpec,
    SteeringSystemSpec,
    UnitSystem,
    VehicleDynamicCase,
    VehicleModel,
    WheelSpec,
)

_WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")
_ROAD_KIND = {
    "plane": 1,
    "sine": 2,
    "bump": 3,
    "random_fourier": 4,
    "four_post": 5,
}


@dataclass(frozen=True)
class _NativeVehicleModel:
    """允许整车底盘自由运动的 native model 视图."""

    name: str
    bodies: tuple[AxleBody, ...]
    joints: tuple[AxleJoint, ...]
    coordinate_couplers: tuple[AxleCoordinateCoupler, ...]
    springs: tuple[AxleSpringDamper, ...]
    bushings: tuple[AxleBushing, ...]
    anti_roll_bars: tuple[object, ...]
    tires: tuple[AxleTire, ...]
    aerodynamic_drags: tuple[AxleAerodynamicDrag, ...]
    gravity_m_per_s2: tuple[float, float, float]


@dataclass(frozen=True)
class _BodyFrame:
    """Reference pose and COM offset used to map global attachment points."""

    rotation: np.ndarray
    origin_m: np.ndarray
    center_of_mass_m: np.ndarray


def _select_assembly_mode(model: VehicleModel, requested: str) -> str:
    """Select a topology without silently replacing physical connections."""
    has_physical_bushings = bool(
        model.front_axle.bushings or model.rear_axle.bushings
    )
    if requested == "auto":
        return "C" if has_physical_bushings else "K"
    if requested == "K" and has_physical_bushings:
        raise ValueError(
            "suspension_mode=K cannot represent declared physical bushings; "
            "use suspension_mode=C"
        )
    if requested == "C" and not has_physical_bushings:
        raise ValueError(
            "suspension_mode=C requires physical bushing data; "
            "use suspension_mode=K or auto"
        )
    return requested


def _validate_steering_topology(model: VehicleModel) -> None:
    """Reject an unactuated rear steering rack instead of freezing it."""
    if not model.rear_axle.rack_fixed_to_chassis:
        raise ValueError(
            "native vehicle dynamics supports the front rack only; "
            "rear_axle.rack_fixed_to_chassis must be true"
        )


@dataclass(frozen=True)
class VehicleDynamicsResult:
    """整车运行结果；底层状态和诊断保持 native axle 结果协议."""

    axle: AxleDynamicsResult
    steering_names: tuple[str, ...] = ()
    steering_output: np.ndarray | None = None

    @property
    def times_s(self) -> np.ndarray:
        return self.axle.times_s

    @property
    def body_names(self) -> tuple[str, ...]:
        return self.axle.body_names

    @property
    def tire_names(self) -> tuple[str, ...]:
        return self.axle.tire_names

    @property
    def states(self) -> np.ndarray:
        """返回所有刚体的状态数组."""
        return self.axle.states

    @property
    def diagnostics(self):
        return self.axle.diagnostics

    @property
    def performance(self):
        return self.axle.performance

    def body_state(self, body: str) -> np.ndarray:
        return self.axle.body_state(body)

    def tire_state(self, tire: str) -> np.ndarray:
        return self.axle.tire_state(tire)

    def steering_state(self, actuator: str) -> np.ndarray:
        if self.steering_output is None:
            raise KeyError("this run has no steering actuator output")
        try:
            index = self.steering_names.index(actuator)
        except ValueError as exc:
            raise KeyError(f"unknown steering actuator {actuator!r}") from exc
        return self.steering_output[:, index, :]

    def joint_wrench(self, joint: str) -> np.ndarray:
        """返回一个约束在 body_b 上的世界坐标系力和力矩."""
        return self.axle.joint_wrench_on_body_b(joint)

    def spring_state(self, spring: str) -> np.ndarray:
        """返回一个弹簧阻尼器的长度、速度和力分量."""
        return self.axle.spring_state(spring)

    def bushing_state(self, bushing: str) -> np.ndarray:
        """返回一个衬套的局部变形和力."""
        return self.axle.bushing_state(bushing)


def run_vehicle_dynamics(
    model: VehicleModel, case: VehicleDynamicCase
) -> VehicleDynamicsResult:
    """运行一个真实前后悬架、车身和轮端的 native 整车动力学算例."""
    if case.vehicle is not model and case.vehicle.model_dump() != model.model_dump():
        raise ValueError("case.vehicle must describe the supplied VehicleModel")
    length_scale = _length_scale(model.units)
    _validate_units(model, case)
    _validate_steering_topology(model)
    assembly_mode = _select_assembly_mode(model, case.suspension_mode)
    assembly = build_vehicle(model, mode=assembly_mode)  # type: ignore[arg-type]
    times = _output_times(case.solver)
    body_state, body_frames = _initial_body_state(assembly, case, length_scale)
    body_names = tuple(assembly.bodies)
    body_index = {name: index for index, name in enumerate(body_names)}
    steering = _build_steering(
        model.steering,
        case.steering_input,
        model,
        assembly,
        body_state,
        body_index,
        body_frames,
        times,
        length_scale,
    )
    road, road_height, road_velocity = _build_road(
        case.road, times, length_scale
    )
    wheel_torque, brake_torque = _build_wheel_torque_signals(
        model, case, times, length_scale
    )
    solver = _native_solver_settings(case.solver, case.static_equilibrium, length_scale)
    springs, bushings = _build_elements(assembly, body_frames, length_scale)
    static_rotation_gauges = _build_static_rotation_gauges(model, assembly)
    native_model = _NativeVehicleModel(
        name=model.name,
        bodies=body_state,
        joints=_build_joints(assembly, body_frames, length_scale),
        coordinate_couplers=_build_coordinate_couplers(model, length_scale),
        springs=springs,
        bushings=bushings,
        anti_roll_bars=(),
        tires=_build_tires(
            model.wheels,
            assembly,
            body_frames,
            length_scale,
            case.road.friction_coefficient,
        ),
        aerodynamic_drags=_build_aerodynamic_drags(
            model, assembly, body_frames, length_scale
        ),
        gravity_m_per_s2=tuple(
            float(value * length_scale)
            for value in case.solver.gravity.as_tuple()
        ),
    )
    axle_case = AxleDynamicsCase(
        name=case.name,
        times_s=tuple(float(value) for value in times),
        road_height_m=road_height,
        road_velocity_m_per_s=road_velocity,
        wheel_torque_n_m=wheel_torque,
        solver=solver,
    )
    native_run = _run_native(
        native_model,
        axle_case,
        steering=steering,
        road=road,
        brake_torque=brake_torque,
        static_gauge_body=(
            model.chassis.name
            if _uses_horizontal_static_gauge(case, road)
            and not assembly.bodies[model.chassis.name].fixed
            else None
        ),
        static_gauge_dof_mask=(
            (1 << 0) | (1 << 1) | (1 << 5)
            if _uses_horizontal_static_gauge(case, road)
            and not assembly.bodies[model.chassis.name].fixed
            else 0
        ),
        static_trim_then_release=(
            case.static_equilibrium and case.initial_forward_speed_mps > 0.0
        ),
        static_rotation_gauges=static_rotation_gauges,
        initial_state_angle_tolerance_rad=(
            case.solver.initial_state_angle_tolerance_rad
            or case.solver.constraint_tolerance
        ),
    )
    return VehicleDynamicsResult(
        axle=native_run.result,
        steering_names=() if steering is None else steering.names,
        steering_output=native_run.steering_output,
    )


def write_vehicle_dynamics_artifact(
    result: VehicleDynamicsResult | None,
    model: VehicleModel,
    case: VehicleDynamicCase,
    output_dir: str | Path,
    *,
    failure: Exception | None = None,
) -> Path:
    """写入整车原始数组和可复现性清单."""
    destination = Path(output_dir)
    destination.mkdir(parents=True, exist_ok=True)
    from .axle_dynamics.native import native_build_metadata
    from .axle_dynamics.result import (
        ANTI_ROLL_OUTPUT_COLUMNS,
        BODY_STATE_COLUMNS,
        BUSHING_OUTPUT_COLUMNS,
        CONSTRAINT_WRENCH_COLUMNS,
        DIAGNOSTIC_COLUMNS,
        ENERGY_COLUMNS,
        PERFORMANCE_COLUMNS,
        SPRING_OUTPUT_COLUMNS,
        TIRE_OUTPUT_COLUMNS,
    )

    model_payload = model.model_dump(mode="json")
    case_payload = case.model_dump(mode="json")
    axle = None if result is None else result.axle
    failure_row = getattr(failure, "failure_diagnostics", None)
    manifest: dict[str, Any] = {
        "schema_version": 1,
        "artifact_type": "vehicle_dynamics_result",
        "status": "failed" if failure is not None else "success",
        "package_version": __version__,
        "model_name": model.name,
        "case_name": case.name,
        "model_sha256": canonical_hash(model_payload),
        "case_sha256": canonical_hash(case_payload),
        "model": model_payload,
        "case": case_payload,
        "native_assembly_mode": _select_assembly_mode(
            model, case.suspension_mode
        ),
        "native_build": native_build_metadata(),
        "completed_sample_count": 0 if axle is None else len(axle.times_s),
        "failed_sample_index": getattr(failure, "failed_sample_index", None),
        "failed_time_s": getattr(failure, "failed_time_s", None),
        "native_status": getattr(failure, "status", 0),
        "performance": None if axle is None else asdict(axle.performance),
        "error": None if failure is None else str(failure),
        "failure_diagnostics": (
            None
            if failure_row is None
            else {
                name: float(value)
                for name, value in zip(DIAGNOSTIC_COLUMNS, failure_row)
            }
        ),
        "steering_names": None if result is None else result.steering_names,
        "layouts": {
            "body_state": BODY_STATE_COLUMNS,
            "constraint_wrench": CONSTRAINT_WRENCH_COLUMNS,
            "spring_output": SPRING_OUTPUT_COLUMNS,
            "bushing_output": BUSHING_OUTPUT_COLUMNS,
            "anti_roll_output": ANTI_ROLL_OUTPUT_COLUMNS,
            "diagnostics": DIAGNOSTIC_COLUMNS,
            "tire_output": TIRE_OUTPUT_COLUMNS,
            "energy": ENERGY_COLUMNS,
            "performance": PERFORMANCE_COLUMNS,
            "steering_output": (
                "coordinate_m_or_angle_rad",
                "rate_per_s",
                "target_m_or_angle_rad",
                "actuator_force_or_torque",
            ),
        },
        "arrays_file": "arrays.npz" if result is not None else None,
    }
    if result is not None:
        diagnostics = result.diagnostics
        np.savez_compressed(
            destination / "arrays.npz",
            times_s=result.times_s,
            body_names=np.asarray(result.body_names),
            constraint_names=np.asarray(result.axle.constraint_names),
            spring_names=np.asarray(result.axle.spring_names),
            bushing_names=np.asarray(result.axle.bushing_names),
            anti_roll_bar_names=np.asarray(result.axle.anti_roll_bar_names),
            tire_names=np.asarray(result.tire_names),
            steering_names=np.asarray(result.steering_names),
            states=result.states,
            constraint_wrench=result.axle.constraint_wrench,
            spring_output=result.axle.spring_output,
            bushing_output=result.axle.bushing_output,
            anti_roll_output=result.axle.anti_roll_output,
            diagnostics=np.column_stack(
                tuple(
                    getattr(diagnostics, field)
                    for field in (
                        "accepted",
                        "internal_steps",
                        "rejected_attempts",
                        "newton_iterations",
                        "minimum_accepted_step_s",
                        "maximum_accepted_step_s",
                        "last_accepted_step_s",
                        "position_residual",
                        "velocity_residual",
                        "dynamics_residual",
                        "active_contacts",
                        "contact_events",
                        "local_error_ratio",
                        "energy_residual",
                        "failure_code",
                        "pinned_null_directions",
                    )
                )
            ),
            tire_output=result.axle.tire_output,
            energy=result.axle.energy,
            steering_output=(
                result.steering_output
                if result.steering_output is not None
                else np.empty((len(result.times_s), 0, 4), dtype=np.float64)
            ),
        )
    manifest_path = destination / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True),
        encoding="utf-8",
    )
    return manifest_path


def _length_scale(units: UnitSystem) -> float:
    return 1.0e-3 if units == UnitSystem.ENGINEERING else 1.0


def _validate_units(model: VehicleModel, case: VehicleDynamicCase) -> None:
    if model.coordinate_system.value != "vehicle":
        raise ValueError("native vehicle dynamics requires vehicle coordinates")
    if model.units == UnitSystem.SI and "gravity" not in case.solver.model_fields_set:
        raise ValueError(
            "SI vehicle dynamics requires gravity to be specified explicitly in m/s^2"
        )
    for name, axle in (("front", model.front_axle), ("rear", model.rear_axle)):
        if axle.units != model.units:
            raise ValueError(f"{name} axle units must match VehicleModel.units")


def _build_static_rotation_gauges(
    model: VehicleModel, assembly: VehicleAssembly
) -> tuple[tuple[str, tuple[float, float, float]], ...]:
    """Map declared static-only axes to the composed vehicle body names."""
    gauges: list[tuple[str, tuple[float, float, float]]] = []

    def add(body: str, axis) -> None:
        if axis is None:
            return
        if body not in assembly.bodies:
            raise ValueError(
                f"static rotation gauge references unknown body {body!r}"
            )
        if any(existing_body == body for existing_body, _ in gauges):
            raise ValueError(
                f"multiple static rotation gauges reference body {body!r}"
            )
        gauges.append((body, tuple(float(value) for value in axis.as_tuple())))

    add(model.chassis.name, model.chassis.static_rotation_axis_local)
    for axle_name, axle_model in (
        ("front", model.front_axle),
        ("rear", model.rear_axle),
    ):
        prefix = f"{axle_name}_"
        for body in axle_model.bodies:
            name = model.chassis.name if body.name == "chassis" else f"{prefix}{body.name}"
            add(name, body.static_rotation_axis_local)
    for wheel in model.wheels:
        add(
            assembly.wheel_body_names[wheel.name],
            wheel.static_rotation_axis_local,
        )
    return tuple(gauges)


def _uses_horizontal_static_gauge(
    case: VehicleDynamicCase, road: _VehicleRoadBuffers
) -> bool:
    """Return whether static equations have the flat-road global gauge."""
    if not case.static_equilibrium:
        return False
    # A plane and sampled four-post profile depend only on vertical contact
    # heights.  Sine, bump, and Fourier profiles depend on x and must retain
    # their physical position/orientation equations during trim.
    return road.kind == _ROAD_KIND["plane"] or (
        road.kind == 0 and case.road.corner_height_signals is not None
    )


def _output_times(settings: DynamicSolverSettings) -> np.ndarray:
    step = settings.output_step or settings.step_size
    values = [float(settings.start_time)]
    while values[-1] + step < settings.end_time - 1e-12:
        values.append(values[-1] + step)
    if values[-1] < settings.end_time - 1e-12:
        values.append(float(settings.end_time))
    elif abs(values[-1] - settings.end_time) <= 1e-12:
        values[-1] = float(settings.end_time)
    return np.ascontiguousarray(values, dtype=np.float64)


def _initial_body_state(
    assembly: VehicleAssembly,
    case: VehicleDynamicCase,
    length_scale: float,
) -> tuple[tuple[AxleBody, ...], dict[str, _BodyFrame]]:
    positions: dict[str, np.ndarray] = {}
    orientations: dict[str, np.ndarray] = {}
    velocities: dict[str, np.ndarray] = {}
    omegas: dict[str, np.ndarray] = {}
    fixed: dict[str, bool] = {}
    masses: dict[str, float] = {}
    inertias: dict[str, np.ndarray] = {}
    body_frames: dict[str, _BodyFrame] = {}
    body_aliases = {
        wheel.body: assembly.wheel_body_names[wheel.name]
        for wheel in assembly.wheel_specs.values()
        if wheel.body != assembly.wheel_body_names[wheel.name]
    }
    for name, body in assembly.bodies.items():
        rotation = body.pose.rotation
        origin_m = body.pose.translation * length_scale
        center_of_mass_m = np.asarray(body.center_of_mass, dtype=float) * length_scale
        body_frames[name] = _BodyFrame(
            rotation=rotation.copy(),
            origin_m=origin_m.copy(),
            center_of_mass_m=center_of_mass_m.copy(),
        )
        positions[name] = origin_m + rotation @ center_of_mass_m
        orientations[name] = body.pose.quaternion.copy()
        velocities[name] = np.array(
            [
                case.initial_velocity_sign * case.initial_forward_speed_mps,
                0.0,
                0.0,
            ],
            dtype=float,
        )
        omegas[name] = np.zeros(3, dtype=float)
        fixed[name] = bool(body.fixed)
        masses[name] = float(body.mass)
        inertia = np.asarray(body.inertia, dtype=float) * length_scale**2
        inertias[name] = inertia
        if not body.fixed and body.mass <= 0.0:
            raise ValueError(
                f"native vehicle dynamics requires positive mass for free body {name!r}"
            )

    body_names = tuple(assembly.bodies)
    for initial in case.initial_states:
        name = _resolve_vehicle_body(initial.body, body_names, body_aliases)
        pose = initial.pose
        quaternion = np.asarray(pose.rotation.as_tuple(), dtype=float)
        rotation = _rotation_from_quaternion(quaternion)
        positions[name] = (
            pose.translation.as_array() * length_scale
            + rotation @ body_frames[name].center_of_mass_m
        )
        orientations[name] = quaternion
        velocity = initial.velocity.as_array()
        linear = velocity[:3] * length_scale
        omega = velocity[3:]
        velocities[name] = linear + np.cross(
            omega, rotation @ body_frames[name].center_of_mass_m
        )
        omegas[name] = omega.copy()

    initial_wheel_speeds = dict(case.initial_wheel_speeds)
    explicit_state_bodies = {
        _resolve_vehicle_body(initial.body, body_names, body_aliases)
        for initial in case.initial_states
    }
    for wheel_name, wheel in assembly.wheel_specs.items():
        body_name = assembly.wheel_body_names[wheel_name]
        rotation = _rotation_from_quaternion(orientations[body_name])
        wheel_to_body = assembly.wheel_rotations_local[wheel_name]
        spin_local = wheel_to_body @ wheel.spin_axis.as_array()
        spin_local /= np.linalg.norm(spin_local)
        spin_world = rotation @ spin_local
        if wheel_name in initial_wheel_speeds:
            # Adams 源结果已经给出主轴刚体的完整角速度。标量轮速只用于
            # 没有显式刚体状态的普通算例，不能覆盖源状态中的进动分量。
            if body_name in explicit_state_bodies:
                continue
            omegas[body_name] = spin_world * initial_wheel_speeds[wheel_name]
            continue
        if case.initial_forward_speed_mps <= 0.0:
            continue
        forward_local = _wheel_forward_local(
            wheel,
            rotation,
            wheel_to_body=wheel_to_body,
        )
        forward_world = rotation @ forward_local
        coefficient = float(
            np.dot(np.cross(spin_world, np.array([0.0, 0.0, -1.0])), forward_world)
            * wheel.tire.unloaded_radius
            * length_scale
        )
        if abs(coefficient) <= 1e-12:
            raise ValueError(f"wheel {wheel_name!r} has no valid rolling axis")
        omegas[body_name] = spin_world * (
            -(
                case.initial_velocity_sign * case.initial_forward_speed_mps
            )
            / coefficient
        )

    bodies: list[AxleBody] = []
    for name in body_names:
        bodies.append(
            AxleBody(
                name=name,
                mass_kg=masses[name],
                inertia_kg_m2=_matrix3(inertias[name]),
                position_m=_tuple3(positions[name]),
                quaternion_body_to_world=_tuple4(orientations[name]),
                linear_velocity_m_per_s=_tuple3(velocities[name]),
                angular_velocity_rad_per_s=_tuple3(omegas[name]),
                fixed=fixed[name],
            )
        )
    return tuple(bodies), body_frames


def _resolve_vehicle_body(
    name: str,
    body_names: tuple[str, ...],
    aliases: dict[str, str] | None = None,
) -> str:
    if aliases is not None and name in aliases:
        return aliases[name]
    if name in body_names:
        return name
    candidates = tuple(
        candidate
        for candidate in (f"front_{name}", f"rear_{name}")
        if candidate in body_names
    )
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise ValueError(f"unknown vehicle body {name!r}")
    raise ValueError(
        f"vehicle body {name!r} is ambiguous; use a front_ or rear_ name"
    )


def _rotation_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _tuple3(value: np.ndarray) -> tuple[float, float, float]:
    return tuple(float(item) for item in value)  # type: ignore[return-value]


def _tuple4(value: np.ndarray) -> tuple[float, float, float, float]:
    return tuple(float(item) for item in value)  # type: ignore[return-value]


def _matrix3(value: np.ndarray) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(float(item) for item in row) for row in value)


def _shift_point(
    body: str, point: np.ndarray, body_frames: dict[str, _BodyFrame], scale: float
) -> tuple[float, float, float]:
    frame = body_frames[body]
    point_m = np.asarray(point, dtype=float) * scale
    # Assembly attachment points are already local to the body reference pose.
    # The native kernel stores the body state at its COM, so only this local
    # offset must be removed here.
    point_com_m = point_m - frame.center_of_mass_m
    return _tuple3(point_com_m)


def _build_aerodynamic_drags(
    model: VehicleModel,
    assembly: VehicleAssembly,
    body_frames: dict[str, _BodyFrame],
    scale: float,
) -> tuple[AxleAerodynamicDrag, ...]:
    drag = model.aerodynamic_drag
    if drag is None:
        return ()
    body = model.chassis.name
    if body not in assembly.bodies:
        raise ValueError("aerodynamic drag references an unknown chassis")
    axis = drag.forward_axis.as_array()
    axis /= np.linalg.norm(axis)
    return (
        AxleAerodynamicDrag(
            body=body,
            application_point_m=_shift_point(
                body, drag.application_point.as_array(), body_frames, scale
            ),
            forward_axis_local=_tuple3(axis),
            coefficient_n_s2_per_m2=(
                0.5
                * drag.air_density
                * drag.drag_coefficient
                * drag.frontal_area
            ),
        ),
    )


def _build_joints(
    assembly: VehicleAssembly,
    body_frames: dict[str, _BodyFrame],
    scale: float,
) -> tuple[AxleJoint, ...]:
    joints: list[AxleJoint] = []
    for constraint in assembly.constraints:
        if isinstance(constraint, (BallJoint, PointCoincidence)):
            kind = "spherical"
        elif isinstance(constraint, WeldJoint):
            kind = "fixed"
        elif isinstance(constraint, RevoluteJoint):
            kind = "revolute"
        elif isinstance(constraint, PrismaticJoint):
            kind = "prismatic"
        elif isinstance(constraint, UniversalJoint):
            kind = "universal"
        elif isinstance(constraint, ConstantVelocityJoint):
            kind = "constant_velocity"
        elif isinstance(constraint, CylindricalJoint):
            kind = "cylindrical"
        elif isinstance(constraint, InPlaneJoint):
            kind = "inplane"
        elif isinstance(constraint, (DistanceConstraint, CoordinateDrive)):
            raise ValueError(
                f"native vehicle dynamics does not support constraint {constraint.name!r}"
            )
        else:
            raise ValueError(
                f"native vehicle dynamics does not support {type(constraint).__name__}"
            )
        joints.append(
            AxleJoint(
                name=constraint.name,
                kind=kind,
                body_a=constraint.body_a,
                body_b=constraint.body_b,
                point_a_m=_shift_point(
                    constraint.body_a,
                    constraint.point_a,
                    body_frames,
                    scale,
                ),
                point_b_m=_shift_point(
                    constraint.body_b,
                    constraint.point_b,
                    body_frames,
                    scale,
                ),
                axis_a=_tuple3(np.asarray(getattr(constraint, "axis_a", (0, 0, 1)))),
                axis_b=_tuple3(np.asarray(getattr(constraint, "axis_b", (0, 0, 1)))),
                axis_a_secondary=_tuple3(
                    np.asarray(getattr(constraint, "axis_a_secondary", (0, 1, 0)))
                ),
                axis_b_secondary=_tuple3(
                    np.asarray(getattr(constraint, "axis_b_secondary", (1, 0, 0)))
                ),
                constant_velocity_angle_target=float(
                    getattr(constraint, "angle_target", 0.0)
                ),
            )
        )
    return tuple(joints)


def _build_coordinate_couplers(
    model: VehicleModel,
    scale: float,
) -> tuple[AxleCoordinateCoupler, ...]:
    result: list[AxleCoordinateCoupler] = []
    for coupler in model.coordinate_couplers:
        payload = coupler.model_dump()
        if coupler.coordinate_a == "translation":
            payload["scale_a"] = coupler.scale_a / scale
        if coupler.coordinate_b == "translation":
            payload["scale_b"] = coupler.scale_b / scale
        result.append(AxleCoordinateCoupler(**payload))
    return tuple(result)


def _build_elements(
    assembly: VehicleAssembly,
    body_frames: dict[str, _BodyFrame],
    scale: float,
) -> tuple[tuple[AxleSpringDamper, ...], tuple[AxleBushing, ...]]:
    springs: list[AxleSpringDamper] = []
    bushings: list[AxleBushing] = []
    force_scale = np.diag([1.0, 1.0, 1.0, scale, scale, scale])
    coordinate_inverse = np.diag([1.0 / scale, 1.0 / scale, 1.0 / scale, 1.0, 1.0, 1.0])

    for element in assembly.elements:
        if isinstance(element, LinearSpringElement):
            reference = (
                element.free_length
                if element.free_length is not None
                else element.reference_length
            )
            if reference is None:
                raise ValueError(f"spring {element.name!r} has no reference length")
            free_length = reference + element.preload / element.stiffness
            if free_length < 0.0:
                raise ValueError(f"spring {element.name!r} has a negative effective free length")
            elastic_curve_deflection, elastic_curve_force = _spring_force_curve(
                element.force_curve, scale
            )
            springs.append(
                AxleSpringDamper(
                    name=element.name,
                    body_a=element.body_a,
                    body_b=element.body_b,
                    point_a_m=_shift_point(
                        element.body_a, element.point_a, body_frames, scale
                    ),
                    point_b_m=_shift_point(
                        element.body_b, element.point_b, body_frames, scale
                    ),
                    stiffness_n_per_m=element.stiffness / scale,
                    compression_damping_n_s_per_m=0.0,
                    rebound_damping_n_s_per_m=0.0,
                    free_length_m=free_length * scale,
                    elastic_curve_deflection_m=elastic_curve_deflection,
                    elastic_curve_force_n=elastic_curve_force,
                )
            )
        elif isinstance(element, StaticDamperElement):
            gas_stiffness = element.gas_stiffness / scale
            offset = (
                element.gas_reference_force
                + element.preload
                + element.friction * math.copysign(1.0, element.extension_sign)
            )
            if gas_stiffness > 0.0:
                if element.gas_reference_length is None:
                    raise ValueError(f"damper {element.name!r} has no gas reference length")
                free_length = (
                    element.gas_reference_length
                    - offset / element.gas_stiffness
                )
                if free_length < 0.0:
                    raise ValueError(f"damper {element.name!r} has a negative effective free length")
            else:
                if abs(offset) > 1e-12:
                    raise ValueError(
                        f"damper {element.name!r} has a constant axial load that the native ABI cannot represent"
                    )
                free_length = 0.0
            curve_velocity, curve_force = _damper_curve(element.force_curve, scale)
            springs.append(
                AxleSpringDamper(
                    name=element.name,
                    body_a=element.body_a,
                    body_b=element.body_b,
                    point_a_m=_shift_point(
                        element.body_a, element.point_a, body_frames, scale
                    ),
                    point_b_m=_shift_point(
                        element.body_b, element.point_b, body_frames, scale
                    ),
                    stiffness_n_per_m=gas_stiffness,
                    compression_damping_n_s_per_m=element.viscous_damping / scale,
                    rebound_damping_n_s_per_m=element.viscous_damping / scale,
                    free_length_m=free_length * scale,
                    damper_curve_velocity_m_per_s=curve_velocity,
                    damper_curve_force_n=curve_force,
                )
            )
        elif isinstance(element, BumpStopElement):
            minimum = element.clearance * scale if element.direction == "bump" else None
            maximum = element.clearance * scale if element.direction == "rebound" else None
            stop_curve_penetration, stop_curve_force = _length_force_curve(
                element.force_curve, scale
            )
            springs.append(
                AxleSpringDamper(
                    name=element.name,
                    body_a=element.body_a,
                    body_b=element.body_b,
                    point_a_m=_shift_point(
                        element.body_a, element.point_a, body_frames, scale
                    ),
                    point_b_m=_shift_point(
                        element.body_b, element.point_b, body_frames, scale
                    ),
                    stiffness_n_per_m=0.0,
                    compression_damping_n_s_per_m=0.0,
                    rebound_damping_n_s_per_m=0.0,
                    free_length_m=0.0,
                    minimum_length_m=minimum,
                    maximum_length_m=maximum,
                    compression_stop_stiffness_n_per_m=(
                        element.stiffness / scale if minimum is not None else 0.0
                    ),
                    rebound_stop_stiffness_n_per_m=(
                        element.stiffness / scale if maximum is not None else 0.0
                    ),
                    compression_stop_curve_penetration_m=(
                        stop_curve_penetration
                        if minimum is not None
                        else ()
                    ),
                    compression_stop_curve_force_n=(
                        stop_curve_force if minimum is not None else ()
                    ),
                    rebound_stop_curve_penetration_m=(
                        stop_curve_penetration
                        if maximum is not None
                        else ()
                    ),
                    rebound_stop_curve_force_n=(
                        stop_curve_force if maximum is not None else ()
                    ),
                )
            )
        elif isinstance(element, BushingElement):
            stiffness = force_scale @ np.asarray(element.stiffness) @ coordinate_inverse
            damping = force_scale @ np.asarray(element.damping) @ coordinate_inverse
            preload = force_scale @ np.asarray(element.preload)
            bushings.append(
                AxleBushing(
                    name=element.name,
                    body_a=element.body_a,
                    body_b=element.body_b,
                    point_a_m=_shift_point(
                        element.body_a,
                        element.local_pose_a.translation,
                        body_frames,
                        scale,
                    ),
                    point_b_m=_shift_point(
                        element.body_b,
                        element.local_pose_b.translation,
                        body_frames,
                        scale,
                    ),
                    frame_a_to_body_quaternion=_tuple4(element.local_pose_a.quaternion),
                    frame_b_to_body_quaternion=_tuple4(element.local_pose_b.quaternion),
                    reference_translation_in_frame_a_m=_tuple3(
                        element.local_pose_b.translation * 0.0
                    ),
                    reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                    stiffness=tuple(tuple(float(item) for item in row) for row in stiffness),
                    damping=tuple(tuple(float(item) for item in row) for row in damping),
                    preload_in_frame_a_n_n_m=_tuple6(preload),
                    force_curves=_bushing_force_curves(element.force_curves, scale),
                    force_curve_interpolation=element.force_curve_interpolation,
                    rotation_coordinates=element.rotation_coordinates,
                )
            )
        elif isinstance(element, AntiRollBarElement):
            raise ValueError(
                f"anti-roll element {element.name!r} is a link anti-roll law; "
                "the native torsional anti-roll ABI is not equivalent"
            )
        elif isinstance(element, VerticalTireElement):
            raise ValueError(
                f"vertical tire element {element.name!r} must be represented by the native tire ABI"
            )
        else:
            raise ValueError(
                f"native vehicle dynamics does not support element {type(element).__name__}"
            )
    return tuple(springs), tuple(bushings)


def _damper_curve(
    curve: tuple[tuple[float, float], ...], scale: float
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if not curve:
        return (), ()
    return (
        tuple(float(velocity * scale) for velocity, _ in curve),
        tuple(float(force) for _, force in curve),
    )


def _length_force_curve(
    curve: tuple[tuple[float, float], ...], scale: float
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """将源文件的长度/力曲线转换为 native 的米/牛顿数组."""
    if not curve:
        return (), ()
    return (
        tuple(float(length * scale) for length, _ in curve),
        tuple(float(force) for _, force in curve),
    )


def _bushing_force_curves(
    curves: tuple[tuple[tuple[float, float], ...], ...], scale: float
) -> tuple[tuple[tuple[float, float], ...], ...]:
    """将工程单位衬套曲线转换为 SI 平移/转动单位."""
    if not curves:
        return ()
    if len(curves) != 6:
        raise ValueError("bushing force_curves must contain six axis curves")
    converted: list[tuple[tuple[float, float], ...]] = []
    for axis, curve in enumerate(curves):
        coordinate_scale = scale if axis < 3 else 1.0
        force_scale = 1.0 if axis < 3 else scale
        converted.append(
            tuple(
                (float(coordinate * coordinate_scale), float(force * force_scale))
                for coordinate, force in curve
            )
        )
    return tuple(converted)


def _spring_force_curve(
    curve: tuple[tuple[float, float], ...], scale: float
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """将带符号伸长曲线转换为压缩挠度和正压缩力曲线."""
    if not curve:
        return (), ()
    compression_curve = tuple(
        (-extension, -force) for extension, force in reversed(curve)
    )
    return _length_force_curve(compression_curve, scale)


def _tuple6(value: np.ndarray) -> tuple[float, float, float, float, float, float]:
    return tuple(float(item) for item in value)  # type: ignore[return-value]


def _wheel_forward_local(
    wheel: WheelSpec,
    rotation: np.ndarray,
    *,
    wheel_to_body: np.ndarray | None = None,
) -> np.ndarray:
    spin = wheel.spin_axis.as_array()
    if wheel_to_body is not None:
        spin = wheel_to_body @ spin
    spin /= np.linalg.norm(spin)
    if wheel.forward_axis is not None:
        forward = wheel.forward_axis.as_array()
        if wheel_to_body is not None:
            forward = wheel_to_body @ forward
    elif wheel_to_body is None:
        forward = rotation.T @ np.array([1.0, 0.0, 0.0])
    else:
        forward = wheel_to_body @ (
            _rotation_from_quaternion(
                np.asarray(wheel.pose.rotation.as_tuple(), dtype=float)
            ).T
            @ np.array([1.0, 0.0, 0.0])
        )
    forward -= spin * np.dot(spin, forward)
    norm = np.linalg.norm(forward)
    if norm <= 1e-12:
        raise ValueError(f"wheel {wheel.name!r} has no forward axis perpendicular to spin")
    return forward / norm


def _build_tires(
    wheels: tuple[WheelSpec, ...],
    assembly: VehicleAssembly,
    body_frames: dict[str, _BodyFrame],
    scale: float,
    road_friction: float,
) -> tuple[AxleTire, ...]:
    tires: list[AxleTire] = []
    for wheel in wheels:
        spec = wheel.tire
        if spec.kind not in {"native_brush", "pac2002"}:
            raise ValueError(
                f"wheel {wheel.name!r} uses tire kind {spec.kind!r}; "
                "native vehicle dynamics supports native_brush and pac2002"
            )
        if spec.kind == "native_brush" and spec.relaxation_length <= 0.0:
            raise ValueError(
                f"wheel {wheel.name!r} requires a positive relaxation_length for native_brush"
            )
        # PAC 纯滑移分支不使用刷胎状态计算力，但底层隐式状态布局仍需
        # 一个正的衰减长度；没有显式值时使用轮胎半径作为数值辅助量。
        relaxation_length = spec.relaxation_length
        if relaxation_length <= 0.0:
            relaxation_length = spec.unloaded_radius
        body_name = assembly.wheel_body_names[wheel.name]
        body = assembly.bodies[body_name]
        radius = spec.unloaded_radius * scale
        maximum = (
            spec.maximum_compression * scale
            if spec.maximum_compression is not None
            else 0.99 * radius
        )
        rotation = body.pose.rotation
        wheel_to_body = assembly.wheel_rotations_local[wheel.name]
        spin_local = wheel_to_body @ wheel.spin_axis.as_array()
        spin_local /= np.linalg.norm(spin_local)
        forward = _wheel_forward_local(
            wheel,
            rotation,
            wheel_to_body=wheel_to_body,
        )
        spin_world = rotation @ spin_local
        frame_body_name = assembly.wheel_centers[wheel.name][0]
        frame_rotation = assembly.bodies[frame_body_name].pose.rotation
        spin = frame_rotation.T @ spin_world
        spin /= np.linalg.norm(spin)
        forward_world = rotation @ forward
        forward = frame_rotation.T @ forward_world
        forward /= np.linalg.norm(forward)
        pac2002_mirror: bool | None = None
        if spec.kind == "pac2002" and spec.parameter_source == "adams_builtin":
            # Adams tire input arrays encode the physical side explicitly:
            # left=0, right=1. Vehicle travel direction does not change it.
            pac2002_mirror = wheel.name.endswith("right")
        drive_torque_body: str | None = None
        drive_torque_reaction_body: str | None = None
        if wheel.drive_torque_body is not None:
            prefix = "rear_" if wheel.name.startswith("rear_") else "front_"

            def runtime_body_name(name: str) -> str:
                if name in assembly.bodies:
                    return name
                candidate = f"{prefix}{name}"
                if candidate in assembly.bodies:
                    return candidate
                raise ValueError(
                    f"wheel {wheel.name!r} drive torque references unknown body {name!r}"
                )

            drive_torque_body = runtime_body_name(wheel.drive_torque_body)
            if wheel.drive_torque_reaction_body is not None:
                drive_torque_reaction_body = runtime_body_name(
                    wheel.drive_torque_reaction_body
                )
        mu = spec.friction_coefficient * road_friction
        tires.append(
            AxleTire(
                name=wheel.name,
                body=body_name,
                center_local_m=_shift_point(
                    body_name,
                    assembly.points[(body_name, "center")],
                    body_frames,
                    scale,
                ),
                frame_body=assembly.wheel_centers[wheel.name][0],
                frame_center_local_m=_shift_point(
                    assembly.wheel_centers[wheel.name][0],
                    assembly.wheel_centers[wheel.name][1],
                    body_frames,
                    scale,
                ),
                drive_torque_body=drive_torque_body,
                drive_torque_reaction_body=drive_torque_reaction_body,
                drive_torque_axis_local=(
                    None
                    if wheel.drive_torque_axis_local is None
                    else _tuple3(wheel.drive_torque_axis_local.as_array())
                ),
                spin_axis_local=_tuple3(spin),
                forward_axis_local=_tuple3(forward),
                unloaded_radius_m=radius,
                maximum_compression_m=maximum,
                vertical_stiffness_n_per_m=spec.vertical_stiffness / scale,
                vertical_damping_n_s_per_m=spec.vertical_damping / scale,
                longitudinal_friction_coefficient=mu,
                lateral_friction_coefficient=mu,
                longitudinal_brush_stiffness_n_per_m=(
                    spec.longitudinal_stiffness / (relaxation_length * scale)
                ),
                lateral_brush_stiffness_n_per_m=(
                    spec.cornering_stiffness / (relaxation_length * scale)
                ),
                longitudinal_relaxation_length_m=relaxation_length * scale,
                lateral_relaxation_length_m=relaxation_length * scale,
                detached_relaxation_s=spec.detached_relaxation_s,
                model_kind=(
                    "pac2002_pure_slip"
                    if spec.kind == "pac2002"
                    else "native_brush"
                ),
                pac2002_parameter_source=spec.parameter_source,
                pac2002_mirror=pac2002_mirror,
                pac2002_coefficients=(
                    dict(spec.pac2002_coefficients)
                    if spec.kind == "pac2002"
                    else {}
                ),
            )
        )
    return tuple(tires)


def _build_steering(
    steering_spec: SteeringSystemSpec,
    steering_input,
    model: VehicleModel,
    assembly: VehicleAssembly,
    initial_bodies: tuple[AxleBody, ...],
    body_index: dict[str, int],
    body_frames: dict[str, _BodyFrame],
    times: np.ndarray,
    scale: float,
) -> _VehicleSteeringBuffers:
    body_names = tuple(assembly.bodies)
    chassis = model.chassis.name
    if steering_spec.actuator_mode == "prescribed_rotation":
        if steering_spec.actuator_body is None:
            raise ValueError(
                "prescribed steering requires steering_spec.actuator_body"
            )
        actuator_body = _resolve_named_body(
            steering_spec.actuator_body, body_names, "steering actuator"
        )
        reaction_body = _resolve_named_body(
            steering_spec.actuator_reaction_body or chassis,
            body_names,
            "steering reaction",
        )
        if assembly.bodies[actuator_body].fixed:
            raise ValueError("the prescribed steering body must be free")
        raw_target = np.asarray(
            [float(steering_input.value_at(float(time))) for time in times],
            dtype=float,
        )
        raw_rate = np.asarray(
            [float(steering_input.derivative_at(float(time))) for time in times],
            dtype=float,
        )
        if np.any(np.abs(raw_target) > steering_spec.max_steering_angle + 1e-12):
            raise ValueError("steering input exceeds max_steering_angle")
        axis = steering_spec.actuator_axis_local.as_array()
        axis /= np.linalg.norm(axis)
        return _VehicleSteeringBuffers(
            names=("steering_input",),
            actuator_type=np.ascontiguousarray([2], dtype=np.int32),
            body=np.ascontiguousarray([body_index[actuator_body]], dtype=np.int32),
            reaction_body=np.ascontiguousarray(
                [body_index[reaction_body]], dtype=np.int32
            ),
            point_local=np.zeros((1, 3), dtype=np.float64),
            reaction_point_local=np.zeros((1, 3), dtype=np.float64),
            axis_local=np.ascontiguousarray([axis], dtype=np.float64),
            reference_quaternion=np.ascontiguousarray(
                [steering_spec.actuator_reference_rotation.as_tuple()],
                dtype=np.float64,
            ),
            target=np.ascontiguousarray(raw_target, dtype=np.float64),
            target_rate=np.ascontiguousarray(raw_rate, dtype=np.float64),
            stiffness=np.ascontiguousarray(
                [steering_spec.rack_stiffness], dtype=np.float64
            ),
            damping=np.ascontiguousarray(
                [steering_spec.rack_damping], dtype=np.float64
            ),
            output=np.full((len(times), 1, 4), np.nan, dtype=np.float64),
        )
    rack = _resolve_steering_rack(steering_spec.rack_body, body_names)
    if assembly.bodies[rack].fixed:
        raise ValueError("the steering rack must be a free body for native actuation")
    reaction_body = _resolve_named_body(
        steering_spec.actuator_reaction_body or chassis,
        body_names,
        "steering reaction",
    )
    if steering_spec.actuator_reaction_body is None:
        try:
            rack_point = assembly.points[(rack, "center")]
            # Both axle assemblies expose a chassis rack marker.  The merged
            # point map can only retain one of them, so use the front assembly
            # explicitly for the front steering actuator.
            reaction_point = assembly.axle_assemblies["front"].points[
                ("chassis", "rack_center")
            ]
        except KeyError as exc:
            raise ValueError("vehicle steering rack markers are incomplete") from exc
        axis = np.asarray(model.front_axle.rack_axis.as_tuple(), dtype=float)
    else:
        rack_joints = tuple(
            joint
            for joint in assembly.constraints
            if isinstance(joint, PrismaticJoint)
            and {joint.body_a, joint.body_b} == {rack, reaction_body}
        )
        if len(rack_joints) != 1:
            raise ValueError(
                "prescribed rack translation requires one prismatic joint "
                "between the rack and its reaction body"
            )
        rack_joint = rack_joints[0]
        if rack_joint.body_a == rack:
            rack_point = rack_joint.point_a
            reaction_point = rack_joint.point_b
            axis = np.asarray(rack_joint.axis_b, dtype=float)
        else:
            rack_point = rack_joint.point_b
            reaction_point = rack_joint.point_a
            axis = np.asarray(rack_joint.axis_a, dtype=float)
    raw_target = np.asarray(
        [
            _steering_target_value(steering_spec, steering_input, time)
            for time in times
        ],
        dtype=float,
    )
    raw_rate = np.asarray(
        [
            _steering_target_rate(steering_spec, steering_input, time)
            for time in times
        ],
        dtype=float,
    )
    max_displacement = steering_spec.max_rack_displacement
    if np.any(np.abs(raw_target) > max_displacement + 1e-12):
        raise ValueError("steering input exceeds max_rack_displacement")
    target = np.ascontiguousarray((raw_target * scale)[:, None])
    target_rate = np.ascontiguousarray((raw_rate * scale)[:, None])
    axis /= np.linalg.norm(axis)
    rack_point_local = _shift_point(rack, rack_point, body_frames, scale)
    reaction_point_local = _shift_point(
        reaction_body, reaction_point, body_frames, scale
    )
    if steering_spec.actuator_mode == "prescribed_translation":
        rack_initial = initial_bodies[body_index[rack]]
        reaction_initial = initial_bodies[body_index[reaction_body]]
        rack_rotation = _rotation_from_quaternion(
            np.asarray(rack_initial.quaternion_body_to_world, dtype=float)
        )
        reaction_rotation = _rotation_from_quaternion(
            np.asarray(reaction_initial.quaternion_body_to_world, dtype=float)
        )
        axis_world = reaction_rotation @ axis
        rack_world = (
            np.asarray(rack_initial.position_m, dtype=float)
            + rack_rotation @ rack_point_local
        )
        reaction_world = (
            np.asarray(reaction_initial.position_m, dtype=float)
            + reaction_rotation @ reaction_point_local
        )
        reference_offset = float(axis_world @ (rack_world-reaction_world)) - float(
            target[0, 0]
        )
        reaction_point_local = reaction_point_local + axis * reference_offset
    return _VehicleSteeringBuffers(
        names=("front_rack",),
        actuator_type=np.ascontiguousarray(
            [3 if steering_spec.actuator_mode == "prescribed_translation" else 0],
            dtype=np.int32,
        ),
        body=np.ascontiguousarray([body_index[rack]], dtype=np.int32),
        reaction_body=np.ascontiguousarray(
            [body_index[reaction_body]], dtype=np.int32
        ),
        point_local=np.ascontiguousarray(
            [rack_point_local],
            dtype=np.float64,
        ),
        reaction_point_local=np.ascontiguousarray(
            [reaction_point_local],
            dtype=np.float64,
        ),
        axis_local=np.ascontiguousarray([axis], dtype=np.float64),
        reference_quaternion=np.ascontiguousarray(
            [[1.0, 0.0, 0.0, 0.0]], dtype=np.float64
        ),
        target=np.ascontiguousarray(target.reshape(-1), dtype=np.float64),
        target_rate=np.ascontiguousarray(target_rate.reshape(-1), dtype=np.float64),
        stiffness=np.ascontiguousarray(
            [steering_spec.rack_stiffness / scale], dtype=np.float64
        ),
        damping=np.ascontiguousarray(
            [steering_spec.rack_damping / scale], dtype=np.float64
        ),
        output=np.full((len(times), 1, 4), np.nan, dtype=np.float64),
    )


def _resolve_named_body(name: str, body_names: tuple[str, ...], role: str) -> str:
    if name in body_names:
        return name
    candidates = tuple(
        candidate
        for candidate in (f"front_{name}", f"rear_{name}")
        if candidate in body_names
    )
    if len(candidates) == 1:
        return candidates[0]
    raise ValueError(f"{role} body {name!r} is not uniquely resolvable")


def _resolve_steering_rack(name: str, body_names: tuple[str, ...]) -> str:
    if name in body_names:
        return name
    if f"front_{name}" in body_names:
        return f"front_{name}"
    candidates = tuple(
        candidate
        for candidate in (f"front_{name}", f"rear_{name}")
        if candidate in body_names
    )
    if len(candidates) == 1:
        return candidates[0]
    raise ValueError(f"steering rack body {name!r} is not uniquely resolvable")


def _steering_target_value(
    steering: SteeringSystemSpec, signal, time: float
) -> float:
    value = signal.value_at(float(time))
    if steering.input == "rack_displacement":
        return value
    conversion = steering.rack_displacement_per_steering_wheel_angle or steering.ratio
    return conversion * value


def _steering_target_rate(
    steering: SteeringSystemSpec, signal, time: float
) -> float:
    value = signal.derivative_at(float(time))
    if steering.input == "rack_displacement":
        return value
    conversion = steering.rack_displacement_per_steering_wheel_angle or steering.ratio
    return conversion * value


def _build_road(
    road: RoadSurfaceSpec,
    times: np.ndarray,
    scale: float,
) -> tuple[_VehicleRoadBuffers, dict[str, tuple[float, ...]], dict[str, tuple[float, ...]]]:
    normal = road.normal.as_array()
    normal /= np.linalg.norm(normal)
    if not np.allclose(normal, np.array([0.0, 0.0, 1.0]), atol=1e-12, rtol=0.0):
        raise ValueError("native road profile currently requires a horizontal road")
    if abs(road.origin.y) > 1e-12:
        raise ValueError("native road profile does not support a nonzero road origin y")
    height: dict[str, tuple[float, ...]] = {}
    velocity: dict[str, tuple[float, ...]] = {}
    road_kind = _ROAD_KIND[road.kind]
    if road.corner_height_signals is not None:
        for index, signal in enumerate(road.corner_height_signals):
            name = _WHEEL_NAMES[index]
            height[name] = tuple(
                (road.origin.z + signal.value_at(float(time))) * scale
                for time in times
            )
            velocity[name] = tuple(
                signal.derivative_at(float(time)) * scale for time in times
            )
        # Sampled corner signals are the complete road height at each wheel.
        # Keeping the analytic profile disabled here avoids adding the same
        # four-post excitation twice in the native contact evaluator.
        road_kind = 0
    buffers = _VehicleRoadBuffers(
        kind=road_kind,
        origin_x=road.origin.x * scale,
        origin_z=road.origin.z * scale,
        amplitude=road.amplitude * scale,
        wavelength=road.wavelength * scale,
        phase=road.phase,
        bump_start=road.bump_start * scale,
        bump_length=road.bump_length * scale,
        corner_scale=np.ascontiguousarray(road.corner_scales, dtype=np.float64),
    )
    return buffers, height, velocity


def _build_wheel_torque_signals(
    model: VehicleModel,
    case: VehicleDynamicCase,
    times: np.ndarray,
    scale: float,
) -> tuple[dict[str, tuple[float, ...]], dict[str, tuple[float, ...]]]:
    direct_drive = dict(case.wheel_drive_torque)
    direct_brake = dict(case.wheel_brake_torque)
    driveline = model.driveline
    if not direct_drive:
        shares = dict(zip(_WHEEL_NAMES, driveline.drive_split))
        driven = set(driveline.driven_wheels)
        if driveline.maximum_drive_torque > 0.0 and not driven:
            raise ValueError("drive torque requires driveline.driven_wheels")
        if any(name in driven and shares[name] <= 0.0 for name in driven):
            raise ValueError("each driven wheel requires a positive drive_split")
    else:
        shares = {}
        driven = set()
    if not direct_brake:
        braked = {wheel.name for wheel in model.wheels if wheel.braked}
        front_braked = sorted(name for name in braked if name.startswith("front_"))
        rear_braked = sorted(name for name in braked if name.startswith("rear_"))
    else:
        front_braked = []
        rear_braked = []
    drive_results: dict[str, tuple[float, ...]] = {}
    brake_results: dict[str, tuple[float, ...]] = {}
    for name in _WHEEL_NAMES:
        drive_values: list[float] = []
        brake_values: list[float] = []
        drive_signal = direct_drive.get(name)
        brake_signal = direct_brake.get(name)
        if not direct_brake:
            if name in front_braked:
                brake_share = driveline.front_brake_bias / len(front_braked)
            elif name in rear_braked:
                brake_share = (1.0 - driveline.front_brake_bias) / len(rear_braked)
            else:
                brake_share = 0.0
        for time in times:
            if direct_drive:
                drive_values.append(
                    scale * (drive_signal.value_at(float(time)) if drive_signal else 0.0)
                )
            else:
                drive = case.drive_input.value_at(float(time))
                if drive < -1.0 or drive > 1.0:
                    raise ValueError("drive_input must be normalized to [-1, 1]")
                drive_values.append(
                    scale
                    * (
                        driveline.maximum_drive_torque * shares[name] * drive
                        if name in driven
                        else 0.0
                    )
                )
            if direct_brake:
                brake_values.append(
                    scale * (brake_signal.value_at(float(time)) if brake_signal else 0.0)
                )
            else:
                brake = case.brake_input.value_at(float(time))
                if brake < 0.0 or brake > 1.0:
                    raise ValueError("brake_input must be normalized to [0, 1]")
                brake_values.append(
                    scale
                    * driveline.maximum_brake_torque
                    * brake_share
                    * brake
                )
        drive_results[name] = tuple(drive_values)
        brake_results[name] = tuple(brake_values)
    return drive_results, brake_results


def _native_solver_settings(
    settings: DynamicSolverSettings,
    static_equilibrium: bool,
    scale: float,
) -> AxleSolverSettings:
    if settings.integrator != "generalized_alpha":
        raise ValueError(
            "native vehicle dynamics requires the generalized_alpha integrator"
        )
    if settings.global_velocity_damping != 0.0:
        raise ValueError(
            "global_velocity_damping is not part of the native vehicle ABI"
        )
    if settings.velocity_recovery_enabled:
        raise ValueError(
            "velocity recovery is not part of the native vehicle ABI"
        )
    if settings.min_internal_step_size > settings.internal_step_size:
        raise ValueError("min_internal_step_size must not exceed internal_step_size")
    tolerance = settings.constraint_tolerance
    integration_tolerance = (
        settings.integration_error_tolerance
        if settings.integration_error_tolerance is not None
        else tolerance
    )
    velocity_tolerance = settings.velocity_tolerance
    return AxleSolverSettings(
        integrator="ggl_generalized_alpha",
        rho_inf=settings.generalized_alpha_rho_inf,
        initialization_mode=(
            "static_equilibrium" if static_equilibrium else "provided_consistent_state"
        ),
        adaptive_step=settings.adaptive_substepping,
        internal_step_s=settings.internal_step_size,
        minimum_step_s=settings.min_internal_step_size,
        maximum_step_s=max(settings.step_size, settings.internal_step_size),
        local_relative_tolerance=integration_tolerance,
        # These absolute floors belong to the time-integration error norm.
        # Keep them independent from the stricter Newton/constraint residual
        # tolerances below; Adams' integration error is a separate setting.
        local_position_tolerance_m=integration_tolerance * scale,
        local_angle_tolerance_rad=integration_tolerance,
        local_velocity_tolerance_m_per_s=integration_tolerance * scale,
        local_angular_velocity_tolerance_rad_per_s=integration_tolerance,
        local_brush_tolerance_m=integration_tolerance * scale,
        contact_event_tolerance_s=settings.event_tolerance,
        max_newton_iterations=settings.projection_max_iterations,
        max_line_search_iterations=settings.projection_backtracking,
        position_tolerance_m=tolerance * scale,
        velocity_tolerance_m_per_s=velocity_tolerance * scale,
        dynamics_tolerance=tolerance,
        increment_tolerance=tolerance * scale,
    )
