from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    AxleAerodynamicDrag,
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleTire,
    NativeAxleError,
    run_axle_dynamics,
)
from suspension_multibody.axle_dynamics.native import (
    _run_native,
    _VehicleRoadBuffers,
)

_I = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
_ZERO_I = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def _tire(
    *,
    body: str = "wheel",
    maximum_compression: float = 0.05,
) -> AxleTire:
    return AxleTire(
        name="tire",
        body=body,
        unloaded_radius_m=0.3,
        maximum_compression_m=maximum_compression,
        vertical_stiffness_n_per_m=10_000.0,
        vertical_damping_n_s_per_m=100.0,
        longitudinal_friction_coefficient=0.2,
        lateral_friction_coefficient=0.3,
        longitudinal_brush_stiffness_n_per_m=100_000.0,
        lateral_brush_stiffness_n_per_m=80_000.0,
        longitudinal_relaxation_length_m=0.2,
        lateral_relaxation_length_m=0.25,
        detached_relaxation_s=0.02,
    )


def _drop_model(
    *,
    height: float = 0.31,
    vertical_velocity: float = -1.0,
    maximum_compression: float = 0.05,
) -> AxleDynamicsModel:
    return AxleDynamicsModel(
        name="drop",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="wheel",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, height),
                linear_velocity_m_per_s=(0.0, 0.0, vertical_velocity),
            ),
        ),
        joints=(),
        tires=(
            _tire(maximum_compression=maximum_compression),
        ),
    )


def test_drop_locates_contact_and_limits_penetration() -> None:
    result = run_axle_dynamics(
        _drop_model(),
        AxleDynamicsCase(
            name="drop",
            times_s=(0.0, 0.005, 0.01, 0.015, 0.02),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=True,
                internal_step_s=0.004,
                maximum_step_s=0.004,
                local_relative_tolerance=1e-4,
                local_position_tolerance_m=1e-7,
                local_velocity_tolerance_m_per_s=1e-6,
            ),
        ),
    )
    tire = result.tire_state("tire")
    assert tire[2, 0] == 0.0
    assert tire[2, 2] == pytest.approx(0.0, abs=1e-8)
    assert np.any(tire[3:, 0] == 1.0)
    assert np.all(tire[:, 2] <= 0.05 + 1e-9)
    assert np.all(tire[:, 9] <= 1.0 + 1e-12)


def test_initial_tire_compression_limit_is_a_hard_physical_failure() -> None:
    with pytest.raises(NativeAxleError, match="maximum compression"):
        run_axle_dynamics(
            _drop_model(height=0.249, vertical_velocity=0.0),
            AxleDynamicsCase(
                name="over-compressed",
                times_s=(0.0, 0.001),
                solver=AxleSolverSettings(
                    initialization_mode="provided_consistent_state"
                ),
            ),
        )


def test_vehicle_aerodynamic_drag_uses_current_body_velocity() -> None:
    model = AxleDynamicsModel(
        name="aerodynamic-drag",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                linear_velocity_m_per_s=(10.0, 0.0, 0.0),
            ),
        ),
        joints=(),
        aerodynamic_drags=(
            AxleAerodynamicDrag(
                body="body",
                coefficient_n_s2_per_m2=0.5,
            ),
        ),
    )
    result = _run_native(
        model,
        AxleDynamicsCase(
            name="aerodynamic-drag",
            times_s=(0.0, 0.00001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00001,
                maximum_step_s=0.00001,
            ),
        ),
        road=_VehicleRoadBuffers(
            kind=1,
            origin_x=0.0,
            origin_z=0.0,
            amplitude=0.0,
            wavelength=1.0,
            phase=0.0,
            bump_start=0.0,
            bump_length=1.0,
            corner_scale=np.ones(4, dtype=float),
        ),
        initial_state_angle_tolerance_rad=1.0e-8,
    ).result

    assert result.body_state("body")[0, 13] == pytest.approx(-5.0)


def test_pac2002_cambered_contact_applies_normal_force_at_ground_intersection() -> None:
    camber = 0.1
    center_height = 0.25
    model = AxleDynamicsModel(
        name="cambered-pac-contact",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="wheel",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, center_height),
                quaternion_body_to_world=(
                    float(np.cos(0.5 * camber)),
                    float(np.sin(0.5 * camber)),
                    0.0,
                    0.0,
                ),
            ),
        ),
        joints=(),
        tires=(
            _tire().model_copy(
                update={
                    "model_kind": "pac2002_pure_slip",
                    "pac2002_parameter_source": "adams_builtin",
                    "vertical_damping_n_s_per_m": 0.0,
                }
            ),
        ),
    )
    result = _run_native(
        model,
        AxleDynamicsCase(
            name="cambered-pac-contact",
            times_s=(0.0, 0.00001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=True,
                internal_step_s=0.00001,
                maximum_step_s=0.00001,
            ),
        ),
        road=_VehicleRoadBuffers(
            kind=1,
            origin_x=0.0,
            origin_z=0.0,
            amplitude=0.0,
            wavelength=1.0,
            phase=0.0,
            bump_start=0.0,
            bump_length=1.0,
            corner_scale=np.ones(4, dtype=float),
        ),
        initial_state_angle_tolerance_rad=1.0e-8,
    ).result

    normal_force = float(result.tire_state("tire")[0, 4])
    expected_roll_acceleration = center_height * np.tan(camber) * normal_force
    assert float(result.body_state("wheel")[0, 16]) == pytest.approx(
        expected_roll_acceleration, rel=1e-10, abs=1e-10
    )


def test_brush_force_opposes_longitudinal_slip_and_respects_ellipse() -> None:
    model = AxleDynamicsModel(
        name="rolling-brush",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="fixture",
                mass_kg=0.0,
                inertia_kg_m2=_ZERO_I,
                fixed=True,
            ),
            AxleBody(
                name="wheel",
                mass_kg=10.0,
                inertia_kg_m2=_I,
                position_m=(0.0, 0.0, 0.29),
                linear_velocity_m_per_s=(1.0, 0.0, 0.0),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="fixture",
                body_b="wheel",
                point_a_m=(0.0, 0.0, 0.29),
                point_b_m=(0.0, 0.0, 0.0),
                axis_a=(1.0, 0.0, 0.0),
                axis_b=(1.0, 0.0, 0.0),
            ),
        ),
        tires=(_tire(),),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="rolling-brush",
            times_s=(0.0, 0.001, 0.002, 0.005),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.00025,
            ),
        ),
    )
    tire = result.tire_state("tire")
    assert np.all(tire[1:, 5] <= 1e-12)
    assert np.all(tire[:, 9] <= 1.0 + 1e-12)


def test_fixed_step_localizes_non_grid_contact_as_an_explicit_split() -> None:
    event_time = 0.0073
    result = run_axle_dynamics(
        _drop_model(height=0.3 + event_time),
        AxleDynamicsCase(
            name="non-grid-contact",
            times_s=(0.0, 0.008, 0.012),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.004,
                minimum_step_s=1e-6,
                maximum_step_s=0.004,
                contact_event_tolerance_s=1e-7,
            ),
        ),
    )

    tire = result.tire_state("tire")
    assert tire[0, 0] == 0.0
    assert tire[1, 0] == 1.0
    assert tire[1, 2] > 0.0
    assert result.diagnostics.internal_steps[1] == 3
    assert result.diagnostics.internal_steps[2] == 1
    assert len(result.contact_events) == 1
    event = result.contact_events[0]
    assert event.tire == "tire"
    assert event.transition == "enter"
    assert event.time_s == pytest.approx(event_time, abs=1e-7)


def test_contact_releases_when_raw_normal_force_reaches_zero() -> None:
    result = run_axle_dynamics(
        _drop_model(height=0.29, vertical_velocity=0.2),
        AxleDynamicsCase(
            name="damped-release",
            times_s=(0.0, 0.02, 0.03, 0.04),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.004,
                maximum_step_s=0.004,
                contact_event_tolerance_s=1e-7,
            ),
        ),
    )

    tire = result.tire_state("tire")
    assert tire[0, 0] == 1.0
    assert np.any(tire[1:, 0] == 0.0)
    first_detached = int(np.flatnonzero(tire[:, 0] == 0.0)[0])
    assert tire[first_detached, 4] == 0.0
    assert tire[first_detached, 2] >= 0.0
    assert any(event.transition == "exit" for event in result.contact_events)
