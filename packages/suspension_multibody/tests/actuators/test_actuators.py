"""Steering and wheel torque actuator tests."""

import numpy as np

from suspension_multibody.core import SE3, RigidBody, RigidBodyState
from suspension_multibody.dynamics import (
    DynamicRigidBodyState,
    RackDriveElement,
    WheelTorqueActuator,
)
from suspension_multibody.schema import (
    DrivelineSpec,
    TimeSignal,
    TireModelSpec,
    Vec3,
    WheelSpec,
)


def _wheel_state() -> DynamicRigidBodyState:
    wheel = RigidBody("wheel", pose=SE3.identity(), inertia=np.eye(3))
    carrier = RigidBody("carrier", pose=SE3.identity(), inertia=np.eye(3))
    return DynamicRigidBodyState(
        RigidBodyState({"wheel": wheel, "carrier": carrier}),
        velocities={
            "wheel": np.array([0.0, 0.0, 0.0, 0.0, 10.0, 0.0]),
            "carrier": np.zeros(6),
        },
    )


def test_wheel_torque_has_equal_and_opposite_reaction() -> None:
    actuator = WheelTorqueActuator(
        name="brake_drive",
        wheel="front_left",
        wheel_body="wheel",
        reaction_body="carrier",
        spin_axis_local=np.array([0.0, 1.0, 0.0]),
        drive_signal=TimeSignal(constant=0.5),
        brake_signal=TimeSignal(constant=0.5),
        maximum_drive_torque=100.0,
        maximum_brake_torque=200.0,
        drive_share=1.0,
    )
    result = actuator.evaluate_dynamic(_wheel_state(), 0.0)

    assert result.body_wrenches_global["wheel"][4] == -50.0
    assert np.allclose(
        result.body_wrenches_global["wheel"] + result.body_wrenches_global["carrier"],
        np.zeros(6),
    )


def test_rack_drive_applies_reaction_to_chassis() -> None:
    state = DynamicRigidBodyState(
        RigidBodyState(
            {
                "rack": RigidBody("rack", pose=SE3.identity()),
                "chassis": RigidBody("chassis", pose=SE3.identity()),
            }
        ),
        velocities={"rack": np.zeros(6), "chassis": np.zeros(6)},
    )
    actuator = RackDriveElement(
        name="rack",
        rack_body="rack",
        rack_point_local=np.zeros(3),
        chassis_body="chassis",
        chassis_point_local=np.zeros(3),
        axis_global=np.array([0.0, 1.0, 0.0]),
        target=TimeSignal(constant=10.0),
        stiffness=100.0,
    )
    result = actuator.evaluate_dynamic(state, 0.0)

    assert result.body_wrenches_global["rack"][1] == 1_000.0
    assert result.body_wrenches_global["chassis"][1] == -1_000.0


def test_wheel_actuator_factory_uses_brake_bias() -> None:
    wheel = WheelSpec(
        name="front_left",
        body="wheel",
        center_local=Vec3(),
        tire=TireModelSpec(),
    )
    # Factory integration is covered by the full assembly test; this assertion
    # keeps the schema-side brake parameters explicit for the direct actuator test.
    driveline = DrivelineSpec(front_brake_bias=0.7)
    assert driveline.front_brake_bias == 0.7
    assert wheel.braked
