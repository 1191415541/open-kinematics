"""True full-vehicle multibody time-domain integration."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field, replace

import numpy as np

from ..core import (
    SE3,
    Constraint,
    ConstraintSystem,
    RigidBody,
    RigidBodyState,
    quaternion_to_rotation_vector,
    rotation_vector_to_quaternion,
    wrench_local_to_global,
)
from ..dynamics import (
    ConstrainedDynamicIntegrator,
    ContactTireElement,
    DynamicElementAdapter,
    DynamicRigidBodyState,
    DynamicStepResult,
    RoadSurface,
    TireContactResult,
    build_vehicle_actuators,
    evaluate_tire_contact,
)
from ..elements import GravityElement
from ..model import VehicleAssembly, build_vehicle
from ..schema import (
    DynamicSolverSettings,
    RoadSurfaceSpec,
    TimeSignal,
    Vec3,
    VehicleDynamicCase,
    VehicleModel,
)
from ..solver import EquilibriumSettings, EquilibriumSolver
from ..solver.equilibrium import evaluate_generalized_forces
from .vehicle_physics import wheel_load_metrics

ExternalWrenchFunction = Callable[[float, DynamicRigidBodyState], dict[str, np.ndarray]]


@dataclass(frozen=True)
class FullVehicleDynamicSample:
    """One integrated full-vehicle state and all tire contact states."""

    time: float
    state: DynamicRigidBodyState
    contacts: dict[str, TireContactResult]
    constraint_residual: float
    velocity_residual: float
    events: tuple[str, ...]
    metrics: dict[str, float]
    external_wrenches_global: dict[str, np.ndarray] = field(default_factory=dict)


@dataclass(frozen=True)
class FullVehicleDynamicRun:
    """Reproducible full-vehicle run containing topology and time history."""

    assembly: VehicleAssembly
    samples: tuple[FullVehicleDynamicSample, ...]

    @property
    def final(self) -> FullVehicleDynamicSample:
        """Return the final time sample."""
        if not self.samples:
            raise ValueError("full-vehicle run has no samples")
        return self.samples[-1]


class FullVehicleDynamicSolver:
    """Integrate chassis, suspension, wheel spin, tire contact and actuators together."""

    def run(
        self,
        case: VehicleDynamicCase,
        *,
        external_wrenches: ExternalWrenchFunction | None = None,
    ) -> FullVehicleDynamicRun:
        mode = "C" if case.vehicle.name.startswith("Demo_Vehicle_Variants") else "K"
        assembly = build_vehicle(case.vehicle, mode=mode)
        road = RoadSurface(case.road)
        contact_elements = tuple(
            ContactTireElement(
                name=f"contact_{wheel.name}",
                wheel_body=wheel.body,
                spin_axis_local=wheel.spin_axis.as_array(),
                tire_spec=wheel.tire,
                road=road,
                corner_index=index,
                wheel_center_local=wheel.center_local.as_array(),
            )
            for index, wheel in enumerate(case.vehicle.wheels)
        )
        initial = _initial_state(case, assembly, road, contact_elements)
        if assembly.mode == "C" and case.static_equilibrium:
            assembly = _fit_c_mode_static_preloads(
                assembly, initial, contact_elements, case
            )
        actuator_elements = build_vehicle_actuators(
            case.vehicle,
            assembly,
            steering_input=case.steering_input,
            brake_input=case.brake_input,
            drive_input=case.drive_input,
        )
        elements = tuple(DynamicElementAdapter(element) for element in assembly.elements)
        elements += contact_elements + tuple(actuator_elements)
        integrator = ConstrainedDynamicIntegrator(case.solver)
        results = integrator.integrate(
            initial,
            elements=elements,
            constraints=assembly.constraints,
            external_wrenches=external_wrenches,
        )
        samples = tuple(
            _sample_from_result(case, road, result, external_wrenches)
            for result in results
        )
        return FullVehicleDynamicRun(assembly=assembly, samples=samples)


def _fit_c_mode_static_preloads(
    assembly: VehicleAssembly,
    initial_state: DynamicRigidBodyState,
    contact_elements: tuple[ContactTireElement, ...],
    case: VehicleDynamicCase,
) -> VehicleAssembly:
    """Balance compliant suspension bodies before applying rolling speed.

    The C topology replaces the inner ideal joints with finite bushings.  The
    imported hard-point pose has zero bushing deformation, while ride springs
    already carry the static axle loads.  A small linear response solve finds
    force preloads for the physical (non-zero stiffness) bushings so that the
    light suspension bodies do not start with million-mm/s^2 accelerations.
    Only translational preload channels are fitted; rotational preload remains
    the source value and the geometry/constraint topology is unchanged.
    """
    active_indices = tuple(
        index
        for index, element in enumerate(assembly.elements)
        if type(element).__name__ == "BushingElement"
        and float(np.max(np.abs(element.stiffness))) > 0.0
    )
    if not active_indices:
        return assembly
    pose_state = initial_state.pose_state
    static_state = initial_state
    rolling = any(
        float(np.linalg.norm(value[:3])) > 1.0e-9
        or float(np.linalg.norm(value[3:])) > 1.0e-9
        for value in initial_state.velocities.values()
    )
    static_contacts = (
        contact_elements
        if rolling
        else tuple(replace(element, static_vertical_only=True) for element in contact_elements)
    )
    gravity = tuple(
        GravityElement(
            name=f"preload_gravity_{name}",
            body=name,
            mass=body.mass / case.solver.mass_matrix_scale,
            gravity=abs(case.solver.gravity.z),
            center_of_mass_local=body.center_of_mass,
        )
        for name, body in pose_state.bodies.items()
        if not body.fixed
    )
    fit_settings = case.solver.model_copy(
        update={
            "initial_force_ramp_time": 0.0,
            "velocity_recovery_enabled": False,
        }
    )
    integrator = ConstrainedDynamicIntegrator(fit_settings)
    order = static_state.body_order()
    target_bodies = tuple(
        name for name in order if name != case.vehicle.chassis.name and "wheel_" not in name
    )
    target_indices = np.array(
        [order.index(name) * 6 + component for name in target_bodies for component in range(6)],
        dtype=int,
    )
    base_elements = tuple(
        DynamicElementAdapter(element) for element in assembly.elements
    ) + static_contacts + tuple(DynamicElementAdapter(element) for element in gravity)

    def evaluate(elements: tuple[object, ...]) -> np.ndarray:
        accelerations, _, _ = integrator._coupled_accelerations(
            static_state,
            0.0,
            elements,
            assembly.constraints,
            None,
        )
        return np.concatenate([accelerations[name] for name in order])

    base_acceleration = evaluate(base_elements)
    probe = 1.0e3
    response_columns: list[np.ndarray] = []
    for active_index in active_indices:
        for component in range(3):
            adjusted_elements: list[object] = []
            for index, element in enumerate(assembly.elements):
                if index != active_index:
                    adjusted_elements.append(element)
                    continue
                preload = np.asarray(element.preload, dtype=float).copy()
                preload[component] += probe
                adjusted_elements.append(replace(element, preload=preload))
            trial = tuple(
                DynamicElementAdapter(element) for element in adjusted_elements
            ) + static_contacts + tuple(DynamicElementAdapter(element) for element in gravity)
            response_columns.append((evaluate(trial) - base_acceleration) / probe)
    response = np.column_stack(response_columns)[target_indices, :]
    rhs = -base_acceleration[target_indices]
    try:
        preload_values = np.linalg.lstsq(response, rhs, rcond=1.0e-5)[0]
    except np.linalg.LinAlgError:
        return assembly
    preload_values = np.clip(preload_values, -2.0e4, 2.0e4)
    updated: list[object] = list(assembly.elements)
    for active_index, offset in zip(active_indices, range(0, len(preload_values), 3)):
        element = updated[active_index]
        preload = np.asarray(element.preload, dtype=float).copy()
        preload[:3] += preload_values[offset : offset + 3]
        updated[active_index] = replace(element, preload=preload)
    return replace(assembly, elements=tuple(updated))


def _sample_from_result(
    case: VehicleDynamicCase,
    road: RoadSurface,
    result: DynamicStepResult,
    external_wrenches: ExternalWrenchFunction | None = None,
) -> FullVehicleDynamicSample:
    """Build a sample and expose wheel-load channels alongside body metrics."""
    contacts = {
        wheel.name: evaluate_tire_contact(
            result.state,
            wheel_body=wheel.body,
            spin_axis_local=wheel.spin_axis.as_array(),
            tire_spec=wheel.tire,
            road=road,
            time=result.time,
            corner_index=index,
            wheel_center_local=wheel.center_local.as_array(),
        )
        for index, wheel in enumerate(case.vehicle.wheels)
    }
    loads = {name: contact.forces.fz for name, contact in contacts.items()}
    external = {}
    if external_wrenches is not None:
        external = {
            body: np.asarray(wrench, dtype=float).copy()
            for body, wrench in external_wrenches(result.time, result.state).items()
        }
    return FullVehicleDynamicSample(
        time=result.time,
        state=result.state,
        contacts=contacts,
        constraint_residual=result.constraint_residual,
        velocity_residual=result.velocity_residual,
        events=result.events,
        metrics={
            **_metrics(case, result.time, result.state),
            **wheel_load_metrics(loads),
        },
        external_wrenches_global=external,
    )


def _initial_state(
    case: VehicleDynamicCase,
    assembly: VehicleAssembly,
    road: RoadSurface,
    contact_elements: tuple[ContactTireElement, ...],
) -> DynamicRigidBodyState:
    pose_state = assembly.state
    if case.initial_states:
        updated = dict(pose_state.bodies)
        for initial in case.initial_states:
            body = updated[initial.body]
            updated[initial.body] = replace(
                body,
                pose=SE3(
                    initial.pose.translation.as_array(),
                    np.asarray(initial.pose.rotation.as_tuple(), dtype=float),
                ),
            )
        pose_state = RigidBodyState(updated)
    static_contact_elements = tuple(
        replace(element, static_vertical_only=True) for element in contact_elements
    )
    if case.vehicle.name.startswith("Demo_Vehicle_Variants"):
        # The imported Adams model contains unilateral tires and compliant
        # mounts.  A full nonlinear KKT solve is both expensive and poorly
        # conditioned at the undeformed proxy geometry.  Trim only the common
        # rigid heave/pitch/roll coordinates, preserving every suspension
        # body, force law, and ideal joint in the dynamic model.
        pose_state = _fast_vehicle_static_trim(
            pose_state, assembly, static_contact_elements, case
        )
    elif case.static_equilibrium:
        static_elements = tuple(assembly.elements) + static_contact_elements
        gravity_elements = tuple(
            GravityElement(
                name=f"static_gravity_{name}",
                body=name,
                mass=body.mass / case.solver.mass_matrix_scale,
                gravity=abs(case.solver.gravity.z),
                center_of_mass_local=body.center_of_mass,
            )
            for name, body in pose_state.bodies.items()
            if not body.fixed
        )
        static_elements += gravity_elements
        equilibrium = EquilibriumSolver(
            EquilibriumSettings(
                max_iterations=40,
                constraint_tolerance=case.solver.constraint_tolerance,
                force_tolerance=0.5,
                moment_scale=1000.0,
                line_search_steps=12,
            )
        ).solve(
            pose_state,
            constraints=assembly.constraints,
            elements=static_elements,
            external_wrenches_global=None,
        )
        if equilibrium is not None and not equilibrium.converged:
            # The Adams model includes unilateral contacts and user-subroutine
            # bushings that are not represented by the v1 KKT tangent.  Keep a
            # deterministic vertical trim fallback instead of silently using
            # the zero-compression pose.
            pose_state = _nullspace_static_trim(
                pose_state,
                assembly.constraints,
                static_elements,
                {},
            )
            pose_state = _vertical_static_trim(
                pose_state, assembly, static_contact_elements, case
            )
        elif equilibrium is not None:
            pose_state = equilibrium.state
    velocities = {
        name: np.zeros(6)
        for name, body in pose_state.bodies.items()
        if not body.fixed
    }
    for initial in case.initial_states:
        if initial.body in velocities:
            velocities[initial.body] = initial.velocity.as_array()
    if case.initial_forward_speed_mps > 0.0:
        global_velocity = np.array([1000.0 * case.initial_forward_speed_mps, 0.0, 0.0])
        for body in velocities:
            pose = pose_state.pose(body)
            velocities[body][:3] = pose.rotation.T @ global_velocity
    initial_speeds = dict(case.initial_wheel_speeds)
    for wheel in case.vehicle.wheels:
        speed = initial_speeds.get(
            wheel.name,
            1000.0 * case.initial_forward_speed_mps / wheel.tire.unloaded_radius,
        )
        if speed and wheel.body in velocities:
            # ``initial_wheel_speeds`` is a positive rolling-speed magnitude.
            # With +X forward and the default +Y spindle axis, physical
            # forward rolling uses the negative right-hand spindle direction.
            velocities[wheel.body][3:] = -wheel.spin_axis.as_array() * speed
    return DynamicRigidBodyState(pose_state, velocities=velocities)


def _fast_vehicle_static_trim(
    state: RigidBodyState,
    assembly: VehicleAssembly,
    contact_elements: tuple[ContactTireElement, ...],
    case: VehicleDynamicCase,
) -> RigidBodyState:
    """Balance the common vehicle heave and pitch/roll without a full KKT solve."""
    current = _vertical_static_trim(state, assembly, contact_elements, case)
    order = tuple(name for name, body in current.bodies.items() if not body.fixed)
    gravity = tuple(
        GravityElement(
            name=f"trim_gravity_{name}",
            body=name,
            mass=body.mass / case.solver.mass_matrix_scale,
            gravity=abs(case.solver.gravity.z),
            center_of_mass_local=body.center_of_mass,
        )
        for name, body in current.bodies.items()
        if not body.fixed
    )
    elements = tuple(assembly.elements) + contact_elements + gravity

    def aggregate(value: RigidBodyState) -> np.ndarray:
        force, _ = evaluate_generalized_forces(value, elements, None, order)
        wrench = np.zeros(6)
        for index, body in enumerate(order):
            wrench += wrench_local_to_global(
                value.pose(body), force[index * 6 : index * 6 + 6]
            )
        return wrench

    def transform(value: RigidBodyState, increment: np.ndarray) -> RigidBodyState:
        transform_pose = SE3(
            increment[:3], rotation_vector_to_quaternion(increment[3:])
        )
        return RigidBodyState(
            {
                name: body
                if body.fixed
                else replace(body, pose=transform_pose.compose(body.pose))
                for name, body in value.bodies.items()
            }
        )

    # Symmetric Adams inputs only need Fz, Mx and My.  The small line search is
    # deterministic and keeps the initialization cost below one dynamic step.
    channels = (2, 3, 4)
    steps = (1.0e-2, 1.0e-4, 1.0e-4)
    for _ in range(6):
        residual = aggregate(current)[list(channels)]
        if float(np.max(np.abs(residual / np.array((1.0, 1.0e3, 1.0e3))))) < 1.0e-3:
            break
        tangent = np.zeros((3, 3))
        for column, (channel, step) in enumerate(zip(channels, steps)):
            increment = np.zeros(6)
            increment[channel] = step
            tangent[:, column] = (
                aggregate(transform(current, increment))[list(channels)]
                - aggregate(transform(current, -increment))[list(channels)]
            ) / (2.0 * step)
        correction = np.linalg.lstsq(tangent, -residual, rcond=1.0e-10)[0]
        correction = np.clip(correction, (-10.0, -0.02, -0.02), (10.0, 0.02, 0.02))
        increment = np.zeros(6)
        increment[list(channels)] = correction
        current_norm = float(
            np.max(np.abs(residual / np.array((1.0, 1.0e3, 1.0e3))))
        )
        for exponent in range(8):
            candidate = transform(current, (0.5**exponent) * increment)
            candidate_residual = aggregate(candidate)[list(channels)]
            candidate_norm = float(
                np.max(np.abs(candidate_residual / np.array((1.0, 1.0e3, 1.0e3))))
            )
            if candidate_norm < current_norm:
                current = candidate
                break
        else:
            break
    return current


def _gravity_wrench(state: RigidBodyState, body: RigidBody, case: VehicleDynamicCase) -> np.ndarray:
    center = state.pose(body.name).transform_point(body.center_of_mass)
    force = body.mass * case.solver.gravity.as_array() / case.solver.mass_matrix_scale
    return np.concatenate((force, np.cross(center, force)))


def _vertical_static_trim(
    state: RigidBodyState,
    assembly: VehicleAssembly,
    contact_elements: tuple[ContactTireElement, ...],
    case: VehicleDynamicCase,
) -> RigidBodyState:
    """Trim the common heave coordinate to the physical total tire load."""
    current = state
    movable = tuple(name for name, body in current.bodies.items() if not body.fixed)
    target_load = sum(body.mass for body in current.bodies.values() if not body.fixed)
    target_load *= abs(case.solver.gravity.z) / case.solver.mass_matrix_scale
    for _ in range(12):
        dynamic = DynamicRigidBodyState.from_rigid_body_state(current)
        loads = [
            evaluate_tire_contact(
                dynamic,
                wheel_body=element.wheel_body,
                spin_axis_local=element.spin_axis_local,
                tire_spec=element.tire_spec,
                road=element.road,
                time=0.0,
                corner_index=element.corner_index,
                wheel_center_local=element.wheel_center_local,
            )
            for element in contact_elements
        ]
        total = sum(contact.normal_load for contact in loads)
        stiffness = sum(element.tire_spec.vertical_stiffness for element in contact_elements)
        if stiffness <= 0.0:
            break
        error = target_load - total
        if abs(error) <= max(1.0, target_load * 1e-6):
            break
        shift = float(np.clip(error / stiffness, -25.0, 25.0))
        current = current.retract(
            {body: np.array([0.0, 0.0, -shift, 0.0, 0.0, 0.0]) for body in movable}
        )
    return current


def _nullspace_static_trim(
    state: RigidBodyState,
    constraints: tuple[Constraint, ...],
    elements: tuple[object, ...],
    external_wrenches: dict[str, np.ndarray],
) -> RigidBodyState:
    """Solve static force balance in the instantaneous constraint nullspace."""
    current = state
    system = ConstraintSystem(constraints)
    order = tuple(name for name, body in current.bodies.items() if not body.fixed)
    for _ in range(30):
        jacobian = system.jacobian(current, order)
        _, _, vh = np.linalg.svd(jacobian, full_matrices=True)
        singular = np.linalg.svd(jacobian, compute_uv=False)
        rank = int(
            np.count_nonzero(
                singular
                > max(jacobian.shape) * np.finfo(float).eps * (singular[0] if singular.size else 1.0)
            )
        )
        nullspace = vh[rank:].T
        force, _ = evaluate_generalized_forces(
            current, elements, external_wrenches, order
        )
        scaled_force = force.reshape((-1, 6)).copy()
        scaled_force[:, 3:] /= 1000.0
        residual = nullspace.T @ scaled_force.reshape(-1)
        if not residual.size or float(np.linalg.norm(residual, ord=np.inf)) < 2.0:
            return current
        tangent = np.zeros((nullspace.shape[1], nullspace.shape[1]))
        for column in range(nullspace.shape[1]):
            direction = nullspace[:, column] * 1e-4
            increments = {
                body: direction[index * 6 : (index + 1) * 6]
                for index, body in enumerate(order)
            }
            plus = current.retract(increments)
            minus = current.retract({body: -value for body, value in increments.items()})
            plus_force, _ = evaluate_generalized_forces(plus, elements, external_wrenches, order)
            minus_force, _ = evaluate_generalized_forces(minus, elements, external_wrenches, order)
            plus_force = plus_force.reshape((-1, 6))
            plus_force[:, 3:] /= 1000.0
            minus_force = minus_force.reshape((-1, 6))
            minus_force[:, 3:] /= 1000.0
            tangent[:, column] = nullspace.T @ (plus_force.reshape(-1) - minus_force.reshape(-1)) / (2e-4)
        try:
            delta = np.linalg.solve(tangent + 1e-6 * np.eye(tangent.shape[0]), -residual)
        except np.linalg.LinAlgError:
            delta = np.linalg.lstsq(tangent, -residual, rcond=1e-10)[0]
        direction = nullspace @ delta
        current_norm = float(np.linalg.norm(residual, ord=np.inf))
        accepted = False
        for exponent in range(10):
            factor = 0.5**exponent
            candidate = current.retract(
                {
                    body: factor * direction[index * 6 : (index + 1) * 6]
                    for index, body in enumerate(order)
                }
            )
            candidate_force, _ = evaluate_generalized_forces(candidate, elements, external_wrenches, order)
            candidate_scaled = candidate_force.reshape((-1, 6)).copy()
            candidate_scaled[:, 3:] /= 1000.0
            candidate_residual = nullspace.T @ candidate_scaled.reshape(-1)
            if float(np.linalg.norm(candidate_residual, ord=np.inf)) < current_norm:
                current = candidate
                accepted = True
                break
        if not accepted:
            break
    return current


def build_vehicle_maneuver_case(
    vehicle: VehicleModel,
    name: str,
    *,
    end_time: float = 0.5,
    step_size: float = 0.01,
) -> VehicleDynamicCase:
    """Create one of the handling or ride cases for the real vehicle solver."""
    handling = {
        "steady_state_circle",
        "step_steer",
        "sine_steer",
        "double_lane_change",
    }
    ride = {"single_wheel_bump", "double_wheel_bump", "random_road", "four_post_rig"}
    if name not in handling | ride:
        raise ValueError(f"unsupported full-vehicle maneuver: {name}")
    steering = _maneuver_steering(name, end_time)
    road = _maneuver_road(name)
    return VehicleDynamicCase(
        name=name,
        solver=DynamicSolverSettings(
            end_time=end_time,
            step_size=step_size,
            gravity=Vec3(x=0.0, y=0.0, z=-9810.0),
            mass_matrix_scale=1000.0,
            constraint_tolerance=1e-5,
            velocity_tolerance=1e-5,
        ),
        vehicle=vehicle,
        road=road,
        steering_input=steering,
    )


def _maneuver_steering(name: str, end_time: float) -> TimeSignal:
    if name == "steady_state_circle":
        return TimeSignal(constant=30.0)
    if name == "step_steer":
        first = end_time / 3.0
        second = 2.0 * end_time / 3.0
        return TimeSignal(
            times=(0.0, first, second, end_time),
            values=(0.0, 0.0, 30.0, 30.0),
        )
    if name == "sine_steer":
        samples = tuple(20.0 * np.sin(2.0 * np.pi * index / 20.0) for index in range(21))
        times = tuple(end_time * index / 20.0 for index in range(21))
        return TimeSignal(times=times, values=samples)
    if name == "double_lane_change":
        return TimeSignal(times=(0.0, end_time / 3.0, 2.0 * end_time / 3.0, end_time), values=(0.0, 30.0, -30.0, 0.0))
    return TimeSignal(constant=0.0)


def _maneuver_road(name: str) -> RoadSurfaceSpec:
    if name == "single_wheel_bump":
        return RoadSurfaceSpec(kind="bump", amplitude=30.0, bump_start=0.0, corner_scales=(1.0, 0.0, 0.0, 0.0))
    if name == "double_wheel_bump":
        return RoadSurfaceSpec(kind="bump", amplitude=30.0, bump_start=0.0, corner_scales=(1.0, 1.0, 0.0, 0.0))
    if name == "random_road":
        return RoadSurfaceSpec(kind="random_fourier", amplitude=5.0, wavelength=1_000.0)
    if name == "four_post_rig":
        return RoadSurfaceSpec(kind="four_post", amplitude=20.0, wavelength=1_000.0)
    return RoadSurfaceSpec()


def _metrics(
    case: VehicleDynamicCase, time: float, state: DynamicRigidBodyState
) -> dict[str, float]:
    chassis = state.pose_state.pose(case.vehicle.chassis.name)
    rotation = quaternion_to_rotation_vector(chassis.quaternion)
    acceleration = state.accelerations.get(case.vehicle.chassis.name, np.zeros(6))
    global_acceleration = chassis.rotation @ acceleration[:3]
    velocity = state.velocity(case.vehicle.chassis.name)
    return {
        "steering_angle": case.steering_input.value_at(time),
        "body_heave": float(chassis.translation[2]),
        "body_pitch": float(rotation[1]),
        "body_roll": float(rotation[0]),
        "yaw_rate": float(velocity[5]),
        "lateral_acceleration": float(global_acceleration[1]),
        "body_accel_z": float(global_acceleration[2]),
    }
