from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from suspension_multibody.axle_dynamics import (
    BODY_STATE_COLUMNS,
    DIAGNOSTIC_COLUMNS,
    ENERGY_COLUMNS,
    TIRE_OUTPUT_COLUMNS,
    AxleAntiRollBar,
    AxleBody,
    AxleBushing,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
    load_axle_dynamics_case,
    load_axle_dynamics_model,
    run_axle_dynamics,
    write_axle_dynamics_artifact,
)


def _vertical_slider_model() -> AxleDynamicsModel:
    mass = 10.0
    stiffness = 10_000.0
    equilibrium_length = 0.25 - mass * 9.80665 / stiffness
    return AxleDynamicsModel(
        name="vertical-slider",
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="slider",
                mass_kg=mass,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                position_m=(0.0, 0.0, equilibrium_length),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="spring",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=stiffness,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.25,
            ),
        ),
    )


def test_native_solver_preserves_static_equilibrium() -> None:
    model = _vertical_slider_model()
    case = AxleDynamicsCase(
        name="static-equilibrium",
        times_s=(0.0, 0.001, 0.002),
        solver=AxleSolverSettings(internal_step_s=0.00025),
    )

    result = run_axle_dynamics(model, case)
    slider = result.body_state("slider")

    expected_z = 0.25 - 10.0 * 9.80665 / 10_000.0
    np.testing.assert_allclose(slider[:, 2], expected_z, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(slider[:, 7:], 0.0, atol=1e-9, rtol=0.0)
    spring = result.spring_state("spring")
    np.testing.assert_allclose(spring[:, 0], expected_z, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(spring[:, 1], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(
        spring[:, 2], 10.0 * 9.80665, atol=1e-6, rtol=1e-10
    )
    np.testing.assert_allclose(spring[:, 3:6], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(
        spring[:, 6], 10.0 * 9.80665, atol=1e-6, rtol=1e-10
    )
    assert np.all(result.diagnostics.accepted)
    assert np.max(result.diagnostics.position_residual) <= 1e-8
    assert np.max(result.diagnostics.velocity_residual) <= 1e-7


def test_tire_contact_is_at_patch_and_reports_physical_loads() -> None:
    model = AxleDynamicsModel(
        name="tire-slider",
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="wheel",
                mass_kg=10.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                position_m=(0.0, 0.0, 0.31),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="ground",
                body_b="wheel",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
        tires=(
            AxleTire(
                name="wheel_tire",
                body="wheel",
                unloaded_radius_m=0.3,
                maximum_compression_m=0.05,
                vertical_stiffness_n_per_m=10_000.0,
                vertical_damping_n_s_per_m=100.0,
                longitudinal_friction_coefficient=1.0,
                lateral_friction_coefficient=1.0,
                longitudinal_brush_stiffness_n_per_m=100_000.0,
                lateral_brush_stiffness_n_per_m=100_000.0,
                longitudinal_relaxation_length_m=0.2,
                lateral_relaxation_length_m=0.2,
                detached_relaxation_s=0.02,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="tire-static",
            times_s=(0.0, 0.001),
            solver=AxleSolverSettings(internal_step_s=0.00025),
        ),
    )

    tire = result.tire_output[:, 0, :]
    np.testing.assert_allclose(tire[:, 4], 10.0 * 9.80665, atol=1e-5, rtol=1e-8)
    assert np.all(tire[:, 0] == 1.0)
    assert np.all(tire[:, 9] <= 1.0 + 1e-12)
    assert np.all(np.isfinite(result.energy))


def test_bushing_reference_and_static_force_balance() -> None:
    model = AxleDynamicsModel(
        name="bushing-slider",
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=10.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                position_m=(0.0, 0.0, 0.3),
            ),
        ),
        joints=(),
        bushings=(
            AxleBushing(
                name="mount",
                body_a="ground",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                reference_translation_in_frame_a_m=(0.0, 0.0, 0.3),
                reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                stiffness=(
                    (1000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 1000.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 10_000.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1000.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 0.0, 1000.0, 0.0),
                    (0.0, 0.0, 0.0, 0.0, 0.0, 1000.0),
                ),
                damping=((0.0,) * 6,) * 6,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="bushing-static",
            times_s=(0.0, 0.001),
            solver=AxleSolverSettings(internal_step_s=0.00025),
        ),
    )

    expected_z = 0.3 - 10.0 * 9.80665 / 10_000.0
    np.testing.assert_allclose(
        result.body_state("body")[:, 2], expected_z, atol=1e-6, rtol=0.0
    )
    bushing = result.bushing_state("mount")
    np.testing.assert_allclose(
        bushing[:, 2], -10.0 * 9.80665 / 10_000.0, atol=1e-6, rtol=0.0
    )
    np.testing.assert_allclose(bushing[:, :2], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(bushing[:, 3:6], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(
        bushing[:, 8], 10.0 * 9.80665, atol=1e-5, rtol=1e-8
    )
    np.testing.assert_allclose(bushing[:, [6, 7, 9, 10, 11]], 0.0, atol=1e-9)


def test_anti_roll_bar_reports_physical_angle_rate_and_torque() -> None:
    angle = 0.04
    angular_rate = 0.2
    stiffness = 5000.0
    damping = 50.0
    model = AxleDynamicsModel(
        name="anti-roll-output",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="arm",
                mass_kg=10.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                quaternion_body_to_world=(
                    math.cos(0.5 * angle),
                    0.0,
                    0.0,
                    math.sin(0.5 * angle),
                ),
                angular_velocity_rad_per_s=(0.0, 0.0, angular_rate),
            ),
        ),
        joints=(),
        anti_roll_bars=(
            AxleAntiRollBar(
                name="bar",
                body_a="ground",
                body_b="arm",
                axis_a=(0.0, 0.0, 1.0),
                reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                stiffness_n_m_per_rad=stiffness,
                damping_n_m_s_per_rad=damping,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="anti-roll-output",
            times_s=(0.0, 0.0001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.0001,
            ),
        ),
    )

    output = result.anti_roll_bar_state("bar")[0]
    np.testing.assert_allclose(output[0], angle, atol=1e-12, rtol=0.0)
    np.testing.assert_allclose(output[1], angular_rate, atol=1e-12, rtol=0.0)
    np.testing.assert_allclose(
        output[2], -stiffness * angle - damping * angular_rate, atol=1e-10
    )


def test_axle_schema_loader_and_result_artifact_are_self_describing(
    tmp_path: Path,
) -> None:
    model = _vertical_slider_model()
    case = AxleDynamicsCase(
        name="artifact",
        times_s=(0.0, 0.001),
        solver=AxleSolverSettings(internal_step_s=0.00025),
    )
    model_path = tmp_path / "model.json"
    case_path = tmp_path / "case.yaml"
    model_path.write_text(
        json.dumps(model.model_dump(mode="json")),
        encoding="utf-8",
    )
    import yaml

    case_path.write_text(
        yaml.safe_dump(case.model_dump(mode="json")),
        encoding="utf-8",
    )
    loaded_model = load_axle_dynamics_model(model_path)
    loaded_case = load_axle_dynamics_case(case_path)
    result = run_axle_dynamics(loaded_model, loaded_case)
    manifest_path = write_axle_dynamics_artifact(
        result, loaded_model, loaded_case, tmp_path / "artifact"
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "success"
    assert len(manifest["model_sha256"]) == 64
    assert len(manifest["case_sha256"]) == 64
    assert manifest["layouts"]["body_state"] == list(BODY_STATE_COLUMNS)
    assert manifest["layouts"]["diagnostics"] == list(DIAGNOSTIC_COLUMNS)
    assert manifest["layouts"]["energy"] == list(ENERGY_COLUMNS)
    assert manifest["layouts"]["tire_output"] == list(TIRE_OUTPUT_COLUMNS)
    assert TIRE_OUTPUT_COLUMNS[-3:] == (
        "overturning_moment_n_m",
        "rolling_resistance_moment_n_m",
        "aligning_moment_n_m",
    )
    assert len(TIRE_OUTPUT_COLUMNS) == 15
    with np.load(manifest_path.parent / "arrays.npz") as arrays:
        np.testing.assert_allclose(arrays["states"], result.states)
        np.testing.assert_allclose(arrays["energy"], result.energy)
