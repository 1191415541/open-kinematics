"""Full-vehicle solver to Adams time-history contract tests."""

from suspension_multibody.adams import (
    TimeHistory,
    VehicleReferenceBundle,
    audit_full_vehicle_pairing,
    compare_full_vehicle_mbd_case,
    full_vehicle_time_history,
)
from suspension_multibody.analysis import (
    FullVehicleDynamicSolver,
    build_vehicle_maneuver_case,
)


def _handling_reference(case: str) -> VehicleReferenceBundle:
    return VehicleReferenceBundle(
        case=case,
        category="handling_stability",
        history=TimeHistory(
            time=(0.0, 0.001, 0.002),
            channels={
                "steering_angle": (0.0, 0.0, 0.0),
                "lateral_acceleration": (0.0, 0.0, 0.0),
                "yaw_rate": (0.0, 0.0, 0.0),
                "body_roll": (0.0, 0.0, 0.0),
            },
            units={
                "steering_angle": "rad",
                "lateral_acceleration": "m/s^2",
                "yaw_rate": "rad/s",
                "body_roll": "rad",
            },
        ),
        response_channels=("lateral_acceleration", "yaw_rate", "body_roll"),
        input_manifest={
            "analysis_mode": "full_vehicle_sdi_dynamic",
            "assembly": "Demo_Vehicle_Variants_pac2002.asy",
            "tire_model": "adams_builtin_pac2002",
        },
        input_manifest_hash="fixture",
        raw_artifacts={},
        producer={"name": "msc.adams-car", "version": "2024.1"},
    )


def test_real_solver_exports_all_handling_and_ride_channel_families(
    full_vehicle_model,
) -> None:
    for name, category in (
        ("steady_state_circle", "handling_stability"),
        ("step_steer", "handling_stability"),
        ("sine_steer", "handling_stability"),
        ("double_lane_change", "handling_stability"),
        ("single_wheel_bump", "ride"),
        ("double_wheel_bump", "ride"),
        ("random_road", "ride"),
        ("four_post_rig", "ride"),
    ):
        case = build_vehicle_maneuver_case(
            full_vehicle_model,
            name,
            end_time=0.002,
            step_size=0.001,
        )
        run = FullVehicleDynamicSolver().run(case)
        history = full_vehicle_time_history(run, category)
        assert len(history.time) == 3
        assert history.channels


def test_full_vehicle_pairing_gate_rejects_simplified_or_incomplete_inputs(
    full_vehicle_model,
) -> None:
    case = build_vehicle_maneuver_case(
        full_vehicle_model,
        "step_steer",
        end_time=0.002,
        step_size=0.001,
    )
    audit = audit_full_vehicle_pairing(_handling_reference("step_steer"), case)

    assert audit.status == "BLOCKED"
    assert "chassis_mass_com_inertia_hash" in audit.missing_or_mismatched_fields
    assert "solver_static_equilibrium_state" in audit.missing_or_mismatched_fields
    assert "solver_initial_forward_velocity" in audit.missing_or_mismatched_fields


def test_blocked_full_vehicle_comparison_does_not_calculate_channel_errors(
    full_vehicle_model,
) -> None:
    case = build_vehicle_maneuver_case(
        full_vehicle_model,
        "step_steer",
        end_time=0.002,
        step_size=0.001,
    )
    report = compare_full_vehicle_mbd_case(_handling_reference("step_steer"), case)

    assert report["status"] == "BLOCKED"
    assert "comparison" not in report
