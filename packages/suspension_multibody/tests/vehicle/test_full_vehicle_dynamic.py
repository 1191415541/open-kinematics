"""End-to-end full-vehicle multibody integration tests."""

import numpy as np

from suspension_multibody.analysis import FullVehicleDynamicSolver
from suspension_multibody.schema import (
    DynamicSolverSettings,
    FrontAxleModel,
    MassSpec,
    RigidBodySpec,
    SteeringSystemSpec,
    TireModelSpec,
    Vec3,
    VehicleDynamicCase,
    VehicleModel,
    WheelSpec,
)


def _axle(name: str, x: float) -> FrontAxleModel:
    return FrontAxleModel(
        name=name,
        hardpoints={
            "UPPER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=500.0),
            "UPPER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=500.0),
            "UPPER_OUTBOARD": Vec3(x=x, y=-750.0, z=350.0),
            "LOWER_INBOARD_FRONT": Vec3(x=x, y=-500.0, z=100.0),
            "LOWER_INBOARD_REAR": Vec3(x=x + 150.0, y=-500.0, z=100.0),
            "LOWER_OUTBOARD": Vec3(x=x, y=-750.0, z=100.0),
            "TIE_ROD_INBOARD": Vec3(x=x, y=-450.0, z=250.0),
            "TIE_ROD_OUTBOARD": Vec3(x=x, y=-750.0, z=250.0),
            "WHEEL_CENTER": Vec3(x=x, y=-750.0, z=250.0),
            "RACK_CENTER": Vec3(x=x, y=0.0, z=250.0),
        },
        mass=MassSpec(sprung_mass=600.0),
        bodies=tuple(
            RigidBodySpec(name=body, mass=10.0)
            for body in (
                "rack", "upper_arm_L", "upper_arm_R", "lower_arm_L", "lower_arm_R",
                "upright_L", "upright_R", "tie_rod_L", "tie_rod_R",
            )
        ),
    )


def _case() -> VehicleDynamicCase:
    vehicle = VehicleModel(
        chassis=RigidBodySpec(name="chassis", mass=1200.0),
        front_axle=_axle("front", 1_400.0),
        rear_axle=_axle("rear", -1_400.0),
        wheels=tuple(
            WheelSpec(
                name=name,
                body=f"wheel_{name}",
                center_local=Vec3(),
                mass=20.0,
                axial_inertia=2.0,
                tire=TireModelSpec(
                    kind="fiala",
                    unloaded_radius=300.0,
                    vertical_stiffness=20.0,
                    vertical_damping=1.0,
                ),
            )
            for name in ("front_left", "front_right", "rear_left", "rear_right")
        ),
        steering=SteeringSystemSpec(ratio=16.0),
    )
    return VehicleDynamicCase(
        solver=DynamicSolverSettings(
            end_time=0.002,
            step_size=0.001,
            gravity=Vec3(),
            constraint_tolerance=1e-5,
            velocity_tolerance=1e-5,
        ),
        vehicle=vehicle,
        initial_wheel_speeds=(("front_left", 10.0),),
    )


def test_full_vehicle_solver_integrates_all_bodies_and_contacts() -> None:
    run = FullVehicleDynamicSolver().run(_case())

    assert len(run.assembly.component_ids) >= 15
    assert len(run.samples) == 3
    assert set(run.final.contacts) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    assert any(result.active for result in run.final.contacts.values())
    assert np.isfinite(run.final.constraint_residual)
    assert run.final.state.multipliers.size > 0
    assert run.final.metrics["normal_load_total"] > 0.0
    assert np.isfinite(run.final.metrics["load_transfer_front_minus_rear"])
    assert np.isfinite(run.final.metrics["load_transfer_right_minus_left"])
