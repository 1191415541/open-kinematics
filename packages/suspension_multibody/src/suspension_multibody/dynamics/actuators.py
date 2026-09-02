"""Steering, brake and drive torque actuators for the full vehicle."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..elements.elastic import _point_wrench
from ..model import VehicleAssembly
from ..schema import (
    DrivelineSpec,
    SteeringSystemSpec,
    TimeSignal,
    VehicleModel,
    WheelSpec,
)
from .forces import DynamicForceEvaluation
from .state import DynamicRigidBodyState


@dataclass(frozen=True)
class RackDriveElement:
    """Relative rack displacement servo with a reaction on the chassis."""

    name: str
    rack_body: str
    rack_point_local: np.ndarray
    chassis_body: str
    chassis_point_local: np.ndarray
    axis_global: np.ndarray
    target: TimeSignal
    target_scale: float = 1.0
    stiffness: float = 20_000.0
    damping: float = 500.0

    def evaluate_dynamic(self, state: DynamicRigidBodyState, time: float) -> DynamicForceEvaluation:
        rack_point = state.pose_state.point_world(self.rack_body, self.rack_point_local)
        chassis_point = state.pose_state.point_world(self.chassis_body, self.chassis_point_local)
        axis = self.axis_global / np.linalg.norm(self.axis_global)
        reference = float(axis @ (rack_point - chassis_point))
        relative_velocity = float(
            axis
            @ (
                state.point_velocity_global(self.rack_body, self.rack_point_local)
                - state.point_velocity_global(self.chassis_body, self.chassis_point_local)
            )
        )
        target = self.target_scale * self.target.value_at(time)
        scalar = self.stiffness * (target - reference) - self.damping * relative_velocity
        force = scalar * axis
        return DynamicForceEvaluation(
            name=self.name,
            energy=0.5 * self.stiffness * (target - reference) ** 2,
            power=float(force @ (state.point_velocity_global(self.rack_body, self.rack_point_local) - state.point_velocity_global(self.chassis_body, self.chassis_point_local))),
            body_wrenches_global={
                self.rack_body: _point_wrench(rack_point, force),
                self.chassis_body: _point_wrench(chassis_point, -force),
            },
        )


@dataclass(frozen=True)
class WheelTorqueActuator:
    """Drive and brake torque pair with equal reaction on the wheel carrier."""

    name: str
    wheel: str
    wheel_body: str
    reaction_body: str
    spin_axis_local: np.ndarray
    drive_signal: TimeSignal
    brake_signal: TimeSignal
    maximum_drive_torque: float
    maximum_brake_torque: float
    drive_share: float = 0.0
    brake_share: float = 1.0

    def evaluate_dynamic(self, state: DynamicRigidBodyState, time: float) -> DynamicForceEvaluation:
        pose = state.pose_state.pose(self.wheel_body)
        axis = pose.rotation @ self.spin_axis_local
        axis /= np.linalg.norm(axis)
        drive = _clip(self.drive_signal.value_at(time), -1.0, 1.0)
        brake = _clip(self.brake_signal.value_at(time), 0.0, 1.0)
        torque = (
            self.maximum_drive_torque * self.drive_share * drive
            - self.maximum_brake_torque * self.brake_share * brake * np.sign(
                float(
                    (pose.rotation @ self.spin_axis_local)
                    @ (pose.rotation @ state.velocities[self.wheel_body][3:])
                )
            )
        )
        if abs(torque) < 1e-12:
            return DynamicForceEvaluation(name=self.name, energy=0.0, active=False)
        moment = torque * axis
        return DynamicForceEvaluation(
            name=self.name,
            energy=0.0,
            power=float(moment @ (pose.rotation @ state.velocities[self.wheel_body][3:])),
            body_wrenches_global={
                self.wheel_body: np.concatenate((np.zeros(3), moment)),
                self.reaction_body: np.concatenate((np.zeros(3), -moment)),
            },
        )


def build_vehicle_actuators(
    model: VehicleModel,
    assembly: VehicleAssembly,
    *,
    steering_input: TimeSignal,
    brake_input: TimeSignal,
    drive_input: TimeSignal,
) -> tuple[object, ...]:
    """Build one steering drive and four independent wheel torque channels."""
    steering = _build_steering(model.steering, assembly, steering_input)
    return (
        steering,
        *(
            _build_wheel_actuator(
                wheel,
                model.driveline,
                assembly,
                brake_input,
                drive_input,
            )
            for wheel in model.wheels
        ),
    )


def _build_steering(
    spec: SteeringSystemSpec, assembly: VehicleAssembly, signal: TimeSignal
) -> RackDriveElement:
    rack = spec.rack_body
    if rack not in assembly.bodies:
        rack = f"front_{rack}"
    if rack not in assembly.bodies:
        raise ValueError(f"steering rack body {spec.rack_body!r} is not in the vehicle")
    chassis = next(name for name, body in assembly.bodies.items() if name == "chassis" or body.fixed)
    point = assembly.points.get((rack, "center"), np.zeros(3))
    chassis_point = assembly.points.get((chassis, "rack_center"), np.zeros(3))
    return RackDriveElement(
        name="steering_rack_drive",
        rack_body=rack,
        rack_point_local=point,
        chassis_body=chassis,
        chassis_point_local=chassis_point,
        axis_global=np.array([0.0, 1.0, 0.0]),
        target=signal,
        target_scale=(
            spec.rack_displacement_per_steering_wheel_angle or spec.ratio
            if spec.input == "steering_wheel_angle"
            else 1.0
        ),
        stiffness=20_000.0 / spec.ratio,
        damping=500.0 / spec.ratio,
    )


def _build_wheel_actuator(
    wheel: WheelSpec,
    driveline: DrivelineSpec,
    assembly: VehicleAssembly,
    brake_signal: TimeSignal,
    drive_signal: TimeSignal,
) -> WheelTorqueActuator:
    reaction_body = assembly.wheel_centers[wheel.name][0]
    split_index = ("front_left", "front_right", "rear_left", "rear_right").index(wheel.name)
    drive_share = driveline.drive_split[split_index]
    if wheel.name not in driveline.driven_wheels:
        drive_share = 0.0
    brake_share = driveline.front_brake_bias / 2.0 if wheel.name.startswith("front") else (1.0 - driveline.front_brake_bias) / 2.0
    return WheelTorqueActuator(
        name=f"torque_{wheel.name}",
        wheel=wheel.name,
        wheel_body=assembly.wheel_body_names[wheel.name],
        reaction_body=reaction_body,
        spin_axis_local=(
            assembly.wheel_rotations_local[wheel.name]
            @ wheel.spin_axis.as_array()
        ),
        drive_signal=drive_signal,
        brake_signal=brake_signal,
        maximum_drive_torque=driveline.maximum_drive_torque,
        maximum_brake_torque=driveline.maximum_brake_torque,
        drive_share=drive_share,
        brake_share=brake_share if wheel.braked else 0.0,
    )


def _clip(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))
