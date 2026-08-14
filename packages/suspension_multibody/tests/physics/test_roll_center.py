"""Consistency checks for geometric and effective dynamic roll centers."""

import numpy as np

from suspension_multibody.analysis import (
    FullVehicleDynamicSolver,
    compute_vehicle_roll_centers,
    diagnose_dynamic_roll_centers,
)
from suspension_multibody.schema import DynamicSolverSettings, Vec3, VehicleDynamicCase


def _preloaded_vehicle(full_vehicle_model):
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


def _run_with_wheel_lateral_force(
    full_vehicle_model,
    *,
    end_time: float = 0.005,
    step_size: float = 0.0005,
):
    vehicle = _preloaded_vehicle(full_vehicle_model)
    case = VehicleDynamicCase(
        name="roll_center_lateral_force",
        solver=DynamicSolverSettings(
            end_time=end_time,
            step_size=step_size,
            gravity=Vec3(x=0.0, y=0.0, z=-9810.0),
            constraint_tolerance=1e-5,
            velocity_tolerance=1e-5,
        ),
        vehicle=vehicle,
    )
    force = np.array([0.0, 100_000.0, 0.0])

    def external_wrenches(_time, state):
        result = {}
        for wheel in vehicle.wheels:
            center = state.pose_state.point_world(
                wheel.body,
                wheel.center_local.as_array(),
            )
            point = center.copy()
            point[2] = 0.0
            result[wheel.body] = np.concatenate((force, np.cross(point, force)))
        return result

    return FullVehicleDynamicSolver().run(case, external_wrenches=external_wrenches), vehicle


def test_preloaded_geometry_uses_road_contact_plane(full_vehicle_model) -> None:
    vehicle = _preloaded_vehicle(full_vehicle_model)
    base = compute_vehicle_roll_centers(full_vehicle_model)
    preloaded = compute_vehicle_roll_centers(vehicle)

    for axle in ("front", "rear"):
        np.testing.assert_allclose(preloaded[axle].center, base[axle].center)


def test_dynamic_roll_center_reports_effective_height_and_source(full_vehicle_model) -> None:
    run, vehicle = _run_with_wheel_lateral_force(full_vehicle_model)
    result = diagnose_dynamic_roll_centers(run, vehicle)

    assert result.valid_sample_count == 2 * len(run.samples)
    assert result.invalid_sample_count == 0
    assert result.ignored_sample_count == 0
    for axle in ("front", "rear"):
        samples = result.samples_for_axle(axle)
        assert all(sample.valid for sample in samples)
        assert all(sample.force_source == "wheel_external" for sample in samples)
        assert all(np.isfinite(sample.effective_height) for sample in samples)
        assert samples[-1].effective_height < 0.0
        assert all(np.isfinite(sample.height_difference) for sample in samples)
        np.testing.assert_allclose(
            samples[0].geometric_height,
            result.geometric_centers[axle].center[1],
        )


def test_dynamic_roll_center_marks_small_force_as_unidentifiable(full_vehicle_model) -> None:
    run, vehicle = _run_with_wheel_lateral_force(full_vehicle_model)
    result = diagnose_dynamic_roll_centers(
        run,
        vehicle,
        lateral_force_epsilon=1e9,
    )

    assert result.valid_sample_count == 0
    assert result.invalid_sample_count == 2 * len(run.samples)
    assert result.ignored_sample_count == 0
    assert {sample.reason for sample in result.samples} == {"lateral_force_too_small"}


def test_dynamic_roll_center_final_value_is_time_step_converged(full_vehicle_model) -> None:
    coarse_run, coarse_vehicle = _run_with_wheel_lateral_force(
        full_vehicle_model,
        step_size=0.001,
    )
    fine_run, fine_vehicle = _run_with_wheel_lateral_force(
        full_vehicle_model,
        step_size=0.00025,
    )
    coarse = diagnose_dynamic_roll_centers(coarse_run, coarse_vehicle)
    fine = diagnose_dynamic_roll_centers(fine_run, fine_vehicle)

    coarse_final = coarse.samples_for_axle("front")[-1].effective_height
    fine_final = fine.samples_for_axle("front")[-1].effective_height
    np.testing.assert_allclose(coarse_final, fine_final, rtol=0.05, atol=0.02)
