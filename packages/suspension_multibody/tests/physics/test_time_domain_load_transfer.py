"""Time-domain load-transfer diagnostics for the full-vehicle solver."""

import numpy as np

from suspension_multibody.analysis import (
    FullVehicleDynamicSolver,
    diagnose_dynamic_load_transfer,
)
from suspension_multibody.schema import (
    DynamicSolverSettings,
    Vec3,
    VehicleDynamicCase,
)


def _preloaded_vehicle(full_vehicle_model):
    """Set wheel compression so the first dynamic sample is near static load."""
    wheel_center_z = 120.9675

    def update_axle(axle):
        hardpoints = dict(axle.hardpoints)
        hardpoints["WHEEL_CENTER"] = hardpoints["WHEEL_CENTER"].model_copy(
            update={"z": wheel_center_z}
        )
        return axle.model_copy(update={"hardpoints": hardpoints})

    wheels = tuple(
        wheel.model_copy(
            update={
                "tire": wheel.tire.model_copy(
                    update={
                        "vertical_stiffness": 20_000.0,
                        "vertical_damping": 200.0,
                        "friction_coefficient": 1e-8,
                    }
                )
            }
        )
        for wheel in full_vehicle_model.wheels
    )
    return full_vehicle_model.model_copy(
        update={
            "front_axle": update_axle(full_vehicle_model.front_axle),
            "rear_axle": update_axle(full_vehicle_model.rear_axle),
            "wheels": wheels,
        }
    )


def _run_with_road_force(full_vehicle_model, force: np.ndarray):
    vehicle = _preloaded_vehicle(full_vehicle_model)
    case = VehicleDynamicCase(
        name="road_force_load_transfer",
        solver=DynamicSolverSettings(
            end_time=0.005,
            step_size=0.0005,
            gravity=Vec3(x=0.0, y=0.0, z=-9810.0),
            constraint_tolerance=1e-5,
            velocity_tolerance=1e-5,
        ),
        vehicle=vehicle,
    )

    def external_wrenches(_time, state):
        chassis_origin = state.pose_state.pose("chassis").translation
        return {
            "chassis": np.concatenate((force, np.cross(chassis_origin, force)))
        }

    return FullVehicleDynamicSolver().run(
        case,
        external_wrenches=external_wrenches,
    )


def test_time_domain_longitudinal_load_transfer_is_balanced(full_vehicle_model) -> None:
    run = _run_with_road_force(
        full_vehicle_model,
        np.array([1_460_000.0, 0.0, 0.0]),
    )
    diagnostic = diagnose_dynamic_load_transfer(run, ignore_before=0.0005)

    assert diagnostic.balanced_sample_count == len(run.samples) - 1
    assert diagnostic.final.summary.front_rear_delta < 0.0
    np.testing.assert_allclose(
        diagnostic.final.summary.right_left_delta,
        0.0,
        atol=1.0,
    )
    assert diagnostic.max_force_balance_residual < 1e-3 * 1460.0 * 9810.0
    assert diagnostic.max_moment_balance_residual < 1e-4 * 1460.0 * 9810.0 * 2800.0


def test_time_domain_lateral_load_transfer_is_balanced(full_vehicle_model) -> None:
    run = _run_with_road_force(
        full_vehicle_model,
        np.array([0.0, 1_460_000.0, 0.0]),
    )
    diagnostic = diagnose_dynamic_load_transfer(run, ignore_before=0.0005)

    assert diagnostic.final.summary.right_left_delta < 0.0
    np.testing.assert_allclose(
        diagnostic.final.summary.front_rear_delta,
        0.0,
        atol=1.0,
    )
    assert diagnostic.max_force_balance_residual < 1e-3 * 1460.0 * 9810.0
    assert diagnostic.max_moment_balance_residual < 1e-4 * 1460.0 * 9810.0 * 3000.0
