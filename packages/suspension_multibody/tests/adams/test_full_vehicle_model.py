"""Real Adams source-input importer tests."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.adams import (
    AdamsResultChannel,
    adams_contact_patch_plane_height_m,
    adams_rack_displacement_signal_from_result,
    build_adams_vehicle_case,
    build_adams_vehicle_model,
    build_native_rack_steering_model,
    direct_wheel_torque_signals_from_adams_result,
    load_adams_full_vehicle_input,
    parse_adams_result_history,
    steering_signal_from_manifest,
)
from suspension_multibody.adams.full_vehicle_mbd_comparison import (
    _audit_source_equivalence,
)
from suspension_multibody.adams.full_vehicle_model import (
    AdamsBushingAssembly,
    _adams_reuler_matrix,
    _parse_adm_couplers,
    _parse_adm_fields,
    _parse_adm_joints,
    _parse_adm_markers,
    _parse_adm_parts,
    _parse_bushing_sources,
    _parse_initial_speed,
    _parse_subsystem,
    _parse_tire,
    _rotation_matrix_from_quaternion,
    _source_axle_part_ids,
    _source_bushing_frame,
    _source_bushing_side,
    _source_native_body_part_ids,
    build_adams_source_vehicle_model,
)
from suspension_multibody.schema import TimeSignal
from suspension_multibody.vehicle_dynamics import run_vehicle_dynamics

_CASE = Path("artifacts/adams/correlation-reference-real-si/handling-pac2002-v1/step_steer")
_SOURCE_CASE = Path("artifacts/adams-full-source/step_steer")


def test_source_bushing_frame_uses_right_handed_xp_zp_and_side_mirror() -> None:
    assembly = AdamsBushingAssembly(
        subsystem_path=Path("rear.sub"),
        usage="lwr_strut",
        symmetry="left/right",
        property_key="bushing.bus",
        property_path=Path("bushing.bus"),
        orientation_zp=(0.9922015565, -0.1240251946, 0.0124025195),
        orientation_xp=(0.0124990236, 0.0, -0.9999218842),
        preload=(0.0,) * 6,
        force_scaling=(1.0,) * 6,
        damping_force_scaling=(1.0,) * 6,
    )
    left = _source_bushing_frame(assembly, side="L")
    right = _source_bushing_frame(assembly, side="R")
    reflection = np.diag((1.0, -1.0, 1.0))

    np.testing.assert_allclose(left.T @ left, np.eye(3), atol=1e-12)
    np.testing.assert_allclose(right.T @ right, np.eye(3), atol=1e-12)
    assert np.linalg.det(left) == pytest.approx(1.0, abs=1e-12)
    assert np.linalg.det(right) == pytest.approx(1.0, abs=1e-12)
    np.testing.assert_allclose(right[:, 0], reflection @ left[:, 0], atol=1e-12)
    np.testing.assert_allclose(right[:, 1], -reflection @ left[:, 1], atol=1e-12)
    np.testing.assert_allclose(right[:, 2], reflection @ left[:, 2], atol=1e-12)
    assert _source_bushing_side("TR_Rear_Suspension.bkr_lwr_strut.field") == "R"
    assert _source_bushing_side("TR_Rear_Suspension.bkl_lwr_strut.field") == "L"


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_model_keeps_rack_housing_support_topology() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_adams_source_vehicle_model(data)

    assert _source_native_body_part_ids(data, include_drivetrain=True) == {
        part_id for part_id, part in data.compiled_parts.items() if part.mass > 0.0
    }
    assert {
        "adams_part_81",
        "adams_part_82",
        "adams_part_83",
        "adams_part_84",
        "adams_part_121",
        "adams_part_122",
    } <= {body.name for body in model.rear_axle.bodies}
    assert 92 in _source_axle_part_ids(data, rear=False)
    assert 123 not in _source_axle_part_ids(data, rear=False)
    assert 123 in _source_axle_part_ids(data, rear=True)
    assert "rack_housing" in {body.name for body in model.front_axle.bodies}
    assert "powertrain" in {body.name for body in model.rear_axle.bodies}
    assert {f"adams_field_{field_id}" for field_id in range(41, 45)} <= {
        bushing.name for bushing in model.rear_axle.bushings
    }
    convel_targets = {
        joint.name: joint.constant_velocity_angle_target
        for joint in model.rear_axle.joints
        if joint.kind == "constant_velocity"
    }
    assert abs(convel_targets["TR_Rear_Suspension.jolcon_tierod_inner"]) < 1.0e-12
    assert abs(convel_targets["TR_Rear_Suspension.jorcon_tierod_inner"]) < 1.0e-12
    rack_joints = {
        (joint.kind, joint.body_a, joint.body_b)
        for joint in model.front_axle.joints
        if "rack" in joint.name.lower()
    }
    assert ("prismatic", "rack", "rack_housing") in rack_joints
    rack_bushings = {
        (bushing.body_a, bushing.body_b)
        for bushing in model.front_axle.bushings
        if bushing.name in {"adams_field_38", "adams_field_39"}
    }
    assert rack_bushings == {("chassis", "rack_housing")}
    assert "rack_guide" not in {joint.name for joint in model.front_axle.joints}
    rear_lwr_strut = next(
        bushing
        for bushing in model.rear_axle.bushings
        if bushing.name == "adams_field_29"
    )
    assert (rear_lwr_strut.body_a, rear_lwr_strut.body_b) == (
        "adams_part_59",
        "adams_part_73",
    )
    assert rear_lwr_strut.rotation_coordinates == "cardan_xyz"


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_contact_patch_height_is_loaded_from_adams_result() -> None:
    result_path = (
        _SOURCE_CASE / "adams_raw" / "handling_step_steer_dynamic.res"
    )

    assert adams_contact_patch_plane_height_m(result_path) == pytest.approx(
        0.012893098,
        abs=1.0e-9,
    )


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_native_source_case_uses_explicit_rack_displacement_input() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    source_model = build_adams_source_vehicle_model(data)

    model = build_native_rack_steering_model(source_model)
    rack_displacement = adams_rack_displacement_signal_from_result(
        _SOURCE_CASE / "adams_raw" / "handling_step_steer_dynamic.res"
    )
    replay_displacement = TimeSignal(
        times=(0.0, 0.005, 0.02),
        values=(
            rack_displacement.values[0],
            rack_displacement.values[0],
            rack_displacement.values[0] + 0.2,
        ),
    )
    case = build_adams_vehicle_case(
        data,
        model,
        case_name="source_rack_input_only",
        steering_input=replay_displacement,
        end_time=0.02,
        step_size=0.01,
    )

    assert model.steering.input == "rack_displacement"
    assert model.steering.actuator_mode == "prescribed_translation"
    assert model.steering.actuator_body is None
    assert model.steering.actuator_reaction_body == "rack_housing"
    assert len(source_model.coordinate_couplers) == 2
    assert len(model.coordinate_couplers) == 1
    assert all(
        coupler.coordinate_a == coupler.coordinate_b == "rotation"
        for coupler in model.coordinate_couplers
    )
    assert case.vehicle.steering.actuator_mode == "prescribed_translation"
    assert rack_displacement.value_at(2.0) == pytest.approx(8.973816419494435)
    result = run_vehicle_dynamics(model, case)
    assert bool(np.all(result.diagnostics.accepted))
    steering = result.steering_state("front_rack")
    np.testing.assert_allclose(steering[:, 0], steering[:, 2], atol=1.0e-8)


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_native_comparison_converts_contact_height_to_model_units() -> None:
    script_path = Path(__file__).parents[2] / "scripts" / "run_full_native_three_model_comparison.py"
    spec = importlib.util.spec_from_file_location("native_three_model_comparison", script_path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_native_rack_steering_model(
        build_adams_source_vehicle_model(data, tire_kind="pac2002")
    )
    result_path = _SOURCE_CASE / "adams_raw" / "handling_step_steer_dynamic.res"
    rack_displacement = adams_rack_displacement_signal_from_result(result_path)
    case = module._native_case(
        data,
        model,
        rack_displacement,
        tire_kind="pac2002",
        end_time=0.02,
        output_step=0.01,
        internal_step=2.5e-4,
        road_origin_z_m=adams_contact_patch_plane_height_m(result_path),
        source_drive_brake_result_path=result_path,
    )

    assert case.road.origin.z == pytest.approx(12.89309847)
    assert set(dict(case.wheel_drive_torque)) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    assert dict(case.wheel_drive_torque)["rear_left"].values[0] == pytest.approx(
        50377.23486693174
    )
    assert set(dict(case.wheel_brake_torque)) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    assert all(
        channel.entity.endswith("_wheel_tire_forces")
        for channel in module.ADAMS_TIRE_CHANNELS.values()
    )


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_manifest_records_couplers_and_user_function_entities() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)

    assert tuple(
        (item.coupler_id, item.joint_ids, item.kind, item.scales)
        for item in data.source_couplers
    ) == (
        (3, (110, 112), "R:R", (-1.0, 1.0)),
        (4, (111, 107), "R:T", (1.0, 0.1745329252)),
    )
    user_counts = {
        entity_type: sum(
            item.entity_type == entity_type for item in data.source_user_functions
        )
        for entity_type in {item.entity_type for item in data.source_user_functions}
    }
    assert user_counts == {
        "CBKSUB": 1,
        "DIFF": 2,
        "FIELD": 35,
        "GFORCE": 4,
        "GSE": 6,
        "REQUEST": 183,
        "SENSOR": 18,
        "VARIABLE": 88,
    }
    manifest = data.pairing_manifest()
    assert manifest["adams_source_topology"]["unresolved_joint_kinds"] == ()
    assert manifest["adams_source_topology"]["unresolved_coupler_ids"] == ()
    assert manifest["adams_user_function_inventory"]["entity_counts"] == user_counts
    assert manifest["adams_user_function_inventory"]["solver_active_count"] == 154
    assert manifest["native_suspension_implementation"][
        "source_suspension_force_mapping"
    ] == {
        "source_suspension_force_ids": {
            "spring": (7, 8, 17, 18),
            "damper": (9, 10, 19, 20),
            "bumpstop": (15, 16, 25, 26),
        },
        "mapped_suspension_force_ids": (
            7,
            8,
            9,
            10,
            15,
            16,
            17,
            18,
            19,
            20,
            25,
            26,
        ),
        "unmapped_suspension_force_ids": (),
        "spring": "source_AKISPL_curve_and_marker_pair_mapped",
        "damper": "source_AKISPL_curve_and_marker_pair_mapped",
        "bumpstop": "source_AKISPL_curve_and_clearance_mapped",
    }
    source_model = build_adams_source_vehicle_model(data)
    source_force_names = {
        "LinearSpring": tuple(
            element.name
            for element in source_model.front_axle.springs
            + source_model.rear_axle.springs
        ),
        "StaticDamper": tuple(
            element.name
            for element in source_model.front_axle.dampers
            + source_model.rear_axle.dampers
        ),
        "BumpStop": tuple(
            element.name
            for element in source_model.front_axle.stops
            + source_model.rear_axle.stops
        ),
    }
    assert source_force_names == {
        "LinearSpring": (
            "adams_sforce_7",
            "adams_sforce_8",
            "adams_sforce_17",
            "adams_sforce_18",
        ),
        "StaticDamper": (
            "adams_sforce_9",
            "adams_sforce_10",
            "adams_sforce_19",
            "adams_sforce_20",
        ),
        "BumpStop": (
            "adams_sforce_15",
            "adams_sforce_16",
            "adams_sforce_25",
            "adams_sforce_26",
        ),
    }


@pytest.mark.skipif(not _CASE.is_dir(), reason="real Adams reference artifacts are unavailable")
def test_importer_uses_adams_source_files_and_builds_full_model() -> None:
    data = load_adams_full_vehicle_input(_CASE)
    model = build_adams_vehicle_model(data)

    assert data.initial_forward_speed_mps == pytest.approx(16.667)
    assert data.source_units["tire"]["length"] == "meter"
    assert data.source_units["spring"]["length"] == "mm"
    assert data.pac2002_coefficients["PCY1"] == pytest.approx(1.3507)
    assert model.name.startswith("Demo_Vehicle_Variants")
    assert model.chassis.mass == pytest.approx(1399.735175708)
    assert model.wheels[0].tire.unloaded_radius == pytest.approx(344.0)
    assert model.wheels[0].tire.pac2002_coefficients["PDY1"] == pytest.approx(1.0489)
    assert data.spring_curve[0] == pytest.approx((-100.0, -12_500.0))
    assert data.damper_curve[0] == pytest.approx((-1270.0, -1495.5))
    assert data.bumpstop_curve[-1] == pytest.approx((54.0, 31_050.0))
    assert len(data.front_bushing_assemblies) == 9
    assert len(data.rear_bushing_assemblies) == 9
    assert len(data.compiled_markers) == 1039
    assert len(data.source_joints) == 61
    assert data.source_couplers == (
        data.source_couplers[0].__class__(
            coupler_id=3,
            joint_ids=(110, 112),
            kind="R:R",
            scales=(-1.0, 1.0),
            adams_name="TR_Steering.grsred_steering_wheel_column_lock",
        ),
        data.source_couplers[0].__class__(
            coupler_id=4,
            joint_ids=(111, 107),
            kind="R:T",
            scales=(1.0, 0.1745329252),
            adams_name="TR_Steering.grsred_pinion_to_rack",
        ),
    )
    assert data.source_prescribed_joint_ids == (17, 18, 19, 20, 61, 62, 63, 64)
    assert len(data.source_fields) == 35
    assert {joint.kind for joint in data.source_joints} >= {
        "CONVEL",
        "HOOKE",
    }
    lca = data.bushing_properties["bushings.tbl/mdi_lwr_control_arm.bus"]
    assert lca.force_curves[0][0] == pytest.approx((-3.5, -25_000.0))
    assert lca.force_curves[3][0] == pytest.approx(
        (-np.pi / 6.0, -780_000.0)
    )
    assert model.front_axle.bodies[1].inertia[0][0] > 1_000.0
    assert model.rear_axle.rack_fixed_to_chassis
    assert model.wheels[0].tire.cornering_stiffness == pytest.approx(21.92 * 4_850.0)
    assert data.unsupported_user_functions
    user_counts = {
        entity_type: sum(
            item.entity_type == entity_type for item in data.source_user_functions
        )
        for entity_type in {item.entity_type for item in data.source_user_functions}
    }
    assert user_counts == {
        "CBKSUB": 1,
        "DIFF": 2,
        "FIELD": 35,
        "GFORCE": 4,
        "GSE": 6,
        "REQUEST": 183,
        "SENSOR": 18,
        "VARIABLE": 88,
    }
    manifest = data.pairing_manifest()
    assert manifest["adams_bushing_sources"]["status"] == (
        "source_curves_and_application_frames_mapped_to_explicit_topology"
    )
    suspension_manifest = manifest["native_suspension_implementation"]
    assert suspension_manifest["proxy_model"] == (
        "ideal_K_without_unresolved_adams_bushing_curves"
    )
    assert suspension_manifest["source_explicit_model"] == (
        "explicit_C_with_source_field_bushing_curves"
    )
    assert suspension_manifest["source_field_mapping"]["unmapped_field_ids"] == ()
    assert manifest["adams_user_function_inventory"][
        "source_explicit_model_mapped_field_ids"
    ] == tuple(sorted(field.field_id for field in data.source_fields))
    assert manifest["adams_source_topology"]["joint_count"] == 61
    assert manifest["adams_source_topology"]["prescribed_joint_ids"] == (
        17,
        18,
        19,
        20,
        61,
        62,
        63,
        64,
    )
    assert manifest["adams_source_topology"]["field_count"] == 35
    assert manifest["adams_source_topology"]["unresolved_joint_kinds"] == ()
    assert manifest["adams_source_topology"]["unresolved_coupler_ids"] == ()
    assert manifest["adams_user_function_inventory"]["entity_counts"] == user_counts
    assert manifest["adams_user_function_inventory"]["solver_active_count"] == 154
    assert manifest["unit_normalization"]["status"] == "complete"
    assert (
        manifest["native_tire_implementation"]
        == "pac2002_selected_combined_slip_with_relaxation_source_offsets"
    )
    assert "selected_combined_slip_coefficients" in manifest["native_tire_model_scope"]["implemented"]
    assert manifest["adams_model_reduction"]["omitted_part_ids"]
    assert "unsupported" in manifest["adams_force_law_mapping"]["spring"]
    source_inputs = manifest["source_drive_brake_input_contract"]
    assert source_inputs["source"]["drive"]["status"] == (
        "state_dependent_source_force"
    )
    assert source_inputs["source"]["drive"]["force_ids"] == (27, 29)
    assert source_inputs["source"]["brake"]["status"] == (
        "nonzero_source_force"
    )
    assert source_inputs["source"]["brake"]["constant_nonzero_force_ids"] == (
        31,
        34,
    )
    assert source_inputs["native_mapping"] == {"brake": "zero", "drive": "zero"}


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_wheel_spin_axes_use_positive_forward_rolling_convention() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_adams_source_vehicle_model(data)

    for wheel in model.wheels:
        quaternion = np.asarray(wheel.pose.rotation.as_tuple(), dtype=float)
        w, x, y, z = quaternion
        wheel_rotation = np.array(
            (
                (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
                (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
                (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
            ),
            dtype=float,
        )
        axis_world = wheel_rotation @ wheel.spin_axis.as_array()
        axis_world /= np.linalg.norm(axis_world)
        assert wheel.forward_axis is not None
        forward_world = wheel_rotation @ wheel.forward_axis.as_array()
        forward_world /= np.linalg.norm(forward_world)
        source_forward_world = np.array(
            [float(data.initial_velocity_sign), 0.0, 0.0], dtype=float
        )
        source_forward_world -= axis_world * float(source_forward_world @ axis_world)
        source_forward_world /= np.linalg.norm(source_forward_world)
        rolling_velocity = np.cross(axis_world, np.array([0.0, 0.0, -1.0]))

        np.testing.assert_allclose(forward_world, source_forward_world, atol=1.0e-12)
        assert float(rolling_velocity @ source_forward_world) < 0.0


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_initial_conditions_map_to_complete_native_vehicle_state() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_adams_source_vehicle_model(data)
    manifest = json.loads(
        (_SOURCE_CASE / "adams_reference_bundle.json").read_text(encoding="utf-8")
    )["input_manifest"]

    case = build_adams_vehicle_case(
        data,
        model,
        case_name="step_steer",
        steering_input=steering_signal_from_manifest(manifest),
        end_time=0.02,
        step_size=0.01,
    )

    assert len(data.initial_part_states) == 66
    assert len(case.initial_states) == 53
    assert model.steering.actuator_mode == "prescribed_rotation"
    assert model.steering.actuator_body == "adams_part_90"
    assert model.aerodynamic_drag is not None
    assert model.aerodynamic_drag.air_density == pytest.approx(1.22)
    assert model.aerodynamic_drag.drag_coefficient == pytest.approx(0.36)
    assert model.aerodynamic_drag.frontal_area == pytest.approx(1.8)
    assert model.aerodynamic_drag.application_point.as_tuple() == pytest.approx(
        (0.0, 0.0, 12.5)
    )
    assert model.aerodynamic_drag.forward_axis.as_tuple() == pytest.approx(
        (1.0, 0.0, 0.0), abs=1.0e-12
    )
    assert len(model.coordinate_couplers) == 2
    assert "rack_housing" in {body.name for body in model.front_axle.bodies}
    source_manifest = data.pairing_manifest()
    assert source_manifest["native_suspension_implementation"][
        "source_field_mapping"
    ]["unmapped_field_ids"] == ()
    assert len(source_manifest["native_suspension_implementation"][
        "source_field_mapping"
    ]["mapped_field_ids"]) == len(data.source_fields)
    assert len({state.body for state in case.initial_states}) == 53
    assert "rear_powertrain" in {state.body for state in case.initial_states}
    assert case.static_equilibrium is False
    assert case.initial_velocity_sign == -1
    assert dict(case.initial_wheel_speeds) == pytest.approx(
        {
            "front_left": 49.779388911125146,
            "front_right": 49.777989677562395,
            "rear_left": 49.95511239530034,
            "rear_right": 49.954282284642495,
        }
    )
    assert data.pairing_manifest()["solver_initial_state_source"] == (
        "adams_initialConditions_001"
    )
    assert data.pairing_manifest()["adams_model_reduction"]["omitted_part_ids"] == ()


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_result_maps_direct_drive_and_brake_channels() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_adams_source_vehicle_model(data)
    manifest = json.loads(
        (_SOURCE_CASE / "adams_reference_bundle.json").read_text(encoding="utf-8")
    )["input_manifest"]
    result_path = (
        _SOURCE_CASE / "adams_raw" / "handling_step_steer_dynamic.res"
    )
    drive, brake = direct_wheel_torque_signals_from_adams_result(
        result_path
    )

    assert set(drive) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    assert set(brake) == {
        "front_left",
        "front_right",
        "rear_left",
        "rear_right",
    }
    assert drive["rear_left"].times[0] == pytest.approx(0.0)
    assert max(abs(value) for value in drive["front_left"].values) == 0.0
    assert max(abs(value) for value in drive["front_right"].values) == 0.0
    assert drive["rear_left"].values[0] == pytest.approx(50377.23486693174)
    assert drive["rear_right"].values[-1] == pytest.approx(124711.39225572975)
    assert max(
        max(abs(value) for value in signal.values)
        for signal in brake.values()
    ) < 1.0e-10

    case = build_adams_vehicle_case(
        data,
        model,
        case_name="step_steer_direct_torque_inputs",
        steering_input=steering_signal_from_manifest(manifest),
        end_time=0.02,
        step_size=0.01,
        wheel_drive_torque=drive,
        wheel_brake_torque=brake,
    )
    replay_case = build_adams_vehicle_case(
        data,
        model,
        case_name="step_steer_direct_torque_result",
        steering_input=steering_signal_from_manifest(manifest),
        end_time=0.02,
        step_size=0.01,
        source_drive_brake_result_path=result_path,
    )
    replay_manifest = data.pairing_manifest(
        source_drive_brake_result_path=result_path
    )
    verified: list[str] = []
    missing: list[str] = []
    _audit_source_equivalence(
        replay_manifest, verified, missing, [], case=replay_case
    )

    assert dict(case.wheel_drive_torque) == drive
    assert dict(case.wheel_brake_torque) == brake
    assert case.drive_input.constant == 0.0
    assert case.brake_input.constant == 0.0
    assert dict(replay_case.wheel_drive_torque) == drive
    assert dict(replay_case.wheel_brake_torque) == brake
    initial_wheel_speeds = dict(replay_case.initial_wheel_speeds)
    assert drive["rear_left"].values[0] * initial_wheel_speeds["rear_left"] > 0.0
    assert drive["rear_right"].values[0] * initial_wheel_speeds["rear_right"] > 0.0
    assert "source_drive_brake_input_equivalence" in verified
    assert "source_drive_brake_input_equivalence" not in missing
    result = run_vehicle_dynamics(model, replay_case)
    assert bool(np.all(result.diagnostics.accepted))


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_drive_torque_preserves_adams_rotational_force_pair() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_adams_source_vehicle_model(data, tire_kind="pac2002")
    wheels = {wheel.name: wheel for wheel in model.wheels}
    body_specs = {body.name: body for body in model.rear_axle.bodies}

    for name in ("front_left", "front_right"):
        assert wheels[name].drive_torque_body is None
        assert wheels[name].drive_torque_reaction_body is None
        assert wheels[name].drive_torque_axis_local is None

    for name, drive_body in (
        ("rear_left", "adams_part_121"),
        ("rear_right", "adams_part_122"),
    ):
        wheel = wheels[name]
        assert wheel.drive_torque_body == drive_body
        assert wheel.drive_torque_reaction_body == "powertrain"
        assert wheel.drive_torque_axis_local is not None
        rotation = _rotation_matrix_from_quaternion(
            np.asarray(body_specs[drive_body].pose.rotation.as_tuple(), dtype=float)
        )
        axis_world = rotation @ wheel.drive_torque_axis_local.as_array()
        np.testing.assert_allclose(axis_world, (0.0, -1.0, 0.0), atol=1e-12)


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_nonlinear_bushings_preserve_adams_akima_interpolation() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_adams_source_vehicle_model(data, tire_kind="pac2002")
    bushings = (*model.front_axle.bushings, *model.rear_axle.bushings)

    assert bushings
    assert all(bushing.force_curves for bushing in bushings)
    assert all(
        bushing.force_curve_interpolation == "akima" for bushing in bushings
    )


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_drive_torque_matches_adams_output_acceleration() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_native_rack_steering_model(
        build_adams_source_vehicle_model(data, tire_kind="pac2002")
    )
    result_path = _SOURCE_CASE / "adams_raw" / "handling_step_steer_dynamic.res"
    case = build_adams_vehicle_case(
        data,
        model,
        case_name="source_drive_torque_acceleration",
        steering_input=adams_rack_displacement_signal_from_result(result_path),
        end_time=0.001,
        step_size=0.001,
        source_drive_brake_result_path=result_path,
    )
    case = case.model_copy(
        update={
            "solver": case.solver.model_copy(
                update={
                    "internal_step_size": 2.5e-4,
                    "min_internal_step_size": 1.0e-4,
                }
            ),
            "road": case.road.model_copy(
                update={
                    "origin": case.road.origin.model_copy(
                        update={
                            "z": 1.0e3
                            * adams_contact_patch_plane_height_m(result_path)
                        }
                    )
                }
            ),
        }
    )
    result = run_vehicle_dynamics(model, case)
    adams = parse_adams_result_history(
        result_path,
        {
            "rear_left": AdamsResultChannel(
                "jfl_output_torque_data", "acceleration_rear"
            ),
            "rear_right": AdamsResultChannel(
                "jfr_output_torque_data", "acceleration_rear"
            ),
        },
    )
    reaction_alpha_y = result.body_state("rear_powertrain")[0, 17]
    for wheel, body in (
        ("rear_left", "rear_adams_part_121"),
        ("rear_right", "rear_adams_part_122"),
    ):
        relative_alpha = -(
            result.body_state(body)[0, 17] - reaction_alpha_y
        )
        assert relative_alpha == pytest.approx(
            adams.channels[wheel][0], abs=2.0
        )


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_pac2002_initial_longitudinal_slip_matches_adams() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_native_rack_steering_model(
        build_adams_source_vehicle_model(data, tire_kind="pac2002")
    )
    result_path = (
        _SOURCE_CASE / "adams_raw" / "handling_step_steer_dynamic.res"
    )
    case = build_adams_vehicle_case(
        data,
        model,
        case_name="source_pac2002_initial_slip",
        steering_input=adams_rack_displacement_signal_from_result(result_path),
        end_time=0.001,
        step_size=0.001,
        source_drive_brake_result_path=result_path,
    )
    case = case.model_copy(
        update={
            "solver": case.solver.model_copy(
                update={
                    "internal_step_size": 2.5e-4,
                    "min_internal_step_size": 1.0e-4,
                }
            ),
            "road": case.road.model_copy(
                update={
                    "origin": case.road.origin.model_copy(
                        update={
                            "z": 1.0e3
                            * adams_contact_patch_plane_height_m(result_path)
                        }
                    )
                }
            ),
        }
    )
    result = run_vehicle_dynamics(model, case)
    adams_tire = parse_adams_result_history(
        result_path,
        {
            "front_left": AdamsResultChannel(
                "til_wheel_tire_kinematics", "longitudinal_slip_front"
            ),
            "front_left.lateral_slip": AdamsResultChannel(
                "til_wheel_tire_kinematics", "lateral_slip_front"
            ),
            "front_left.longitudinal_force": AdamsResultChannel(
                "til_wheel_tire_forces", "longitudinal_front"
            ),
            "front_left.lateral_force": AdamsResultChannel(
                "til_wheel_tire_forces", "lateral_front"
            ),
            "front_left.overturning_moment": AdamsResultChannel(
                "til_wheel_tire_forces", "overturning_moment_front"
            ),
            "front_left.rolling_resistance_moment": AdamsResultChannel(
                "til_wheel_tire_forces", "rolling_resistance_front"
            ),
            "front_left.aligning_moment": AdamsResultChannel(
                "til_wheel_tire_forces", "aligning_torque_front"
            ),
            "front_right": AdamsResultChannel(
                "tir_wheel_tire_kinematics", "longitudinal_slip_front"
            ),
            "front_right.lateral_slip": AdamsResultChannel(
                "tir_wheel_tire_kinematics", "lateral_slip_front"
            ),
            "front_right.longitudinal_force": AdamsResultChannel(
                "tir_wheel_tire_forces", "longitudinal_front"
            ),
            "front_right.lateral_force": AdamsResultChannel(
                "tir_wheel_tire_forces", "lateral_front"
            ),
            "front_right.overturning_moment": AdamsResultChannel(
                "tir_wheel_tire_forces", "overturning_moment_front"
            ),
            "front_right.rolling_resistance_moment": AdamsResultChannel(
                "tir_wheel_tire_forces", "rolling_resistance_front"
            ),
            "front_right.aligning_moment": AdamsResultChannel(
                "tir_wheel_tire_forces", "aligning_torque_front"
            ),
            "rear_left": AdamsResultChannel(
                "til_wheel_tire_kinematics", "longitudinal_slip_rear"
            ),
            "rear_left.lateral_slip": AdamsResultChannel(
                "til_wheel_tire_kinematics", "lateral_slip_rear"
            ),
            "rear_left.longitudinal_force": AdamsResultChannel(
                "til_wheel_tire_forces", "longitudinal_rear"
            ),
            "rear_left.lateral_force": AdamsResultChannel(
                "til_wheel_tire_forces", "lateral_rear"
            ),
            "rear_left.overturning_moment": AdamsResultChannel(
                "til_wheel_tire_forces", "overturning_moment_rear"
            ),
            "rear_left.rolling_resistance_moment": AdamsResultChannel(
                "til_wheel_tire_forces", "rolling_resistance_rear"
            ),
            "rear_left.aligning_moment": AdamsResultChannel(
                "til_wheel_tire_forces", "aligning_torque_rear"
            ),
            "rear_right": AdamsResultChannel(
                "tir_wheel_tire_kinematics", "longitudinal_slip_rear"
            ),
            "rear_right.lateral_slip": AdamsResultChannel(
                "tir_wheel_tire_kinematics", "lateral_slip_rear"
            ),
            "rear_right.longitudinal_force": AdamsResultChannel(
                "tir_wheel_tire_forces", "longitudinal_rear"
            ),
            "rear_right.lateral_force": AdamsResultChannel(
                "tir_wheel_tire_forces", "lateral_rear"
            ),
            "rear_right.overturning_moment": AdamsResultChannel(
                "tir_wheel_tire_forces", "overturning_moment_rear"
            ),
            "rear_right.rolling_resistance_moment": AdamsResultChannel(
                "tir_wheel_tire_forces", "rolling_resistance_rear"
            ),
            "rear_right.aligning_moment": AdamsResultChannel(
                "tir_wheel_tire_forces", "aligning_torque_rear"
            ),
        },
    )

    assert bool(np.all(result.diagnostics.accepted))
    for wheel in result.tire_names:
        tire = result.tire_state(wheel)[0]
        assert tire[10] == pytest.approx(
            adams_tire.channels[wheel][0] * 0.01,
            abs=3.5e-5,
        )
        assert tire[11] == pytest.approx(
            adams_tire.channels[f"{wheel}.lateral_slip"][0],
            abs=3.0e-5,
        )
        assert tire[5] == pytest.approx(
            adams_tire.channels[f"{wheel}.longitudinal_force"][0],
            abs=5.0,
        )
        assert tire[6] == pytest.approx(
            adams_tire.channels[f"{wheel}.lateral_force"][0],
            abs=5.0,
        )
        assert tire[12] == pytest.approx(
            adams_tire.channels[f"{wheel}.overturning_moment"][0] * 1.0e-3,
            abs=0.1,
        )
        assert tire[13] == pytest.approx(
            adams_tire.channels[f"{wheel}.rolling_resistance_moment"][0] * 1.0e-3,
            abs=0.1,
        )
        assert tire[14] == pytest.approx(
            adams_tire.channels[f"{wheel}.aligning_moment"][0] * 1.0e-3,
            abs=0.5,
        )


@pytest.mark.skipif(
    not _SOURCE_CASE.is_dir(), reason="strict Adams source artifacts are unavailable"
)
def test_source_prescribed_steering_reports_rate_in_actuator_coordinates() -> None:
    data = load_adams_full_vehicle_input(_SOURCE_CASE)
    model = build_adams_source_vehicle_model(data)
    manifest = json.loads(
        (_SOURCE_CASE / "adams_reference_bundle.json").read_text(encoding="utf-8")
    )["input_manifest"]
    case = build_adams_vehicle_case(
        data,
        model,
        case_name="source_prescribed_steering_rate",
        steering_input=steering_signal_from_manifest(manifest),
        end_time=1.01,
        step_size=0.01,
    )

    result = run_vehicle_dynamics(model, case)
    assert len(case.initial_states) == 53
    assert bool(np.all(result.diagnostics.accepted))
    assert float(np.max(result.diagnostics.velocity_residual)) <= 1.0e-4
    steering = result.steering_state("steering_input")
    target_rate = (steering[-1, 2] - steering[-2, 2]) / (
        result.times_s[-1] - result.times_s[-2]
    )

    assert steering[-1, 1] == pytest.approx(target_rate, abs=5e-4)


def test_adm_part_com_uses_local_cm_marker_and_part_orientation(tmp_path: Path) -> None:
    adm = tmp_path / "fixture.adm"
    adm.write_text(
        "\n".join(
            (
                "PART/7",
                ", QG = 100, 200, 300",
                ", REULER = 0D, 90D, 0D",
                ", MASS = 2",
                ", CM = 12",
                ", IM = 13",
                ", IP = 1, 2, 3, 0.1, 0.2",
                ", 0.3",
                "!",
                "MARKER/12",
                ", PART = 7",
                ", QP = 1, 2, 3",
                "MARKER/13",
                ", PART = 7",
                ", QP = 4, 5, 6",
                ", REULER = 45D, 45D, 45D",
            )
        ),
        encoding="ascii",
    )

    parts = _parse_adm_parts(adm)

    assert parts[7].center_of_mass == pytest.approx((101.0, 197.0, 302.0))
    assert parts[7].inertia_products == pytest.approx((0.1, 0.2, 0.3))


def test_compiled_adm_topology_preserves_marker_frames_and_source_laws(
    tmp_path: Path,
) -> None:
    adm = tmp_path / "fixture.adm"
    adm.write_text(
        "\n".join(
            (
                "UNITS/",
                ", LENGTH = MILLIMETER",
                "PART/3",
                "!",
                "!                 adams_view_name='fixture_marker'",
                "MARKER/7",
                ", PART = 3",
                ", QP = 1, -2, 3",
                ", REULER = 10D, 20D, 30D",
                "!",
                "!                 adams_view_name='fixture_convel'",
                "JOINT/9",
                ", CONVEL",
                ", I = 7",
                ", J = 8",
                "!",
                "!                 adams_view_name='fixture_coupler'",
                "COUPLER/2",
                ", JOINTS = 9, 10",
                ", TYPE = R:T",
                ", SCALES = -1, 0.25",
                "!",
                "!                 adams_view_name='fixture_field'",
                "FIELD/11",
                ", I = 7",
                ", J = 8",
                ", FORMULATION = LINEAR",
                ", FUNCTION = USER(910, 2, 3)\\",
                ", ROUTINE = abgFDM::fie910",
            )
        ),
        encoding="ascii",
    )

    markers = _parse_adm_markers(adm)
    joints = _parse_adm_joints(adm)
    couplers = _parse_adm_couplers(adm)
    fields = _parse_adm_fields(adm)

    assert markers[7].part_id == 3
    assert markers[7].local_position == pytest.approx((1.0, -2.0, 3.0))
    np.testing.assert_allclose(
        markers[7].local_orientation,
        _adams_reuler_matrix((10.0, 20.0, 30.0)),
    )
    assert markers[7].adams_name == "fixture_marker"
    assert joints == (
        joints[0].__class__(
            joint_id=9,
            kind="CONVEL",
            marker_i=7,
            marker_j=8,
            adams_name="fixture_convel",
        ),
    )
    assert fields == (
        fields[0].__class__(
            field_id=11,
            marker_i=7,
            marker_j=8,
            formulation="LINEAR",
            function="USER(910, 2, 3)",
            routine="abgFDM::fie910",
            adams_name="fixture_field",
        ),
    )
    assert couplers == (
        couplers[0].__class__(
            coupler_id=2,
            joint_ids=(9, 10),
            kind="R:T",
            scales=(-1.0, 0.25),
            adams_name="fixture_coupler",
        ),
    )


def test_source_units_are_normalized_to_runtime_units(tmp_path: Path) -> None:
    subsystem = tmp_path / "fixture.sub"
    subsystem.write_text(
        "\n".join(
            (
                "[UNITS]",
                " LENGTH = 'meter'",
                " FORCE = 'newton'",
                " ANGLE = 'radian'",
                " MASS = 'kilogram'",
                " TIME = 'second'",
                "[HARDPOINT]",
                " 'wheel_center'  'left/right'  1.0  2.0  3.0",
                "[PART_ASSEMBLY]",
                " USAGE = 'subframe'",
                " MASS = 2.0",
            )
        ),
        encoding="ascii",
    )
    tire = tmp_path / "fixture.tir"
    tire.write_text(
        "\n".join(
            (
                "[UNITS]",
                "LENGTH = 'meter'",
                "FORCE = 'newton'",
                "ANGLE = 'radian'",
                "MASS = 'kilogram'",
                "TIME = 'second'",
                "[MODEL]",
                "UNLOADED_RADIUS = 0.344",
                "FNOMIN = 4850.0",
                "VERTICAL_STIFFNESS = 210000.0",
                "VERTICAL_DAMPING = 50.0",
            )
        ),
        encoding="ascii",
    )
    dcf = tmp_path / "fixture.dcf"
    dcf.write_text(
        "\n".join(
            (
                "[UNITS]",
                "LENGTH = 'millimeter'",
                "FORCE = 'newton'",
                "ANGLE = 'radian'",
                "MASS = 'kilogram'",
                "TIME = 'millisecond'",
                "[EXPERIMENT]",
                "INITIAL_SPEED = 1.0",
            )
        ),
        encoding="ascii",
    )

    hardpoints, parts = _parse_subsystem(subsystem)
    values = _parse_tire(tire)

    assert hardpoints["WHEEL_CENTER"] == pytest.approx((1000.0, 2000.0, 3000.0))
    assert parts["subframe"] == pytest.approx(2.0)
    assert values["UNLOADED_RADIUS_MM"] == pytest.approx(344.0)
    assert values["FNOMIN_N"] == pytest.approx(4850.0)
    assert values["VERTICAL_STIFFNESS_N_MM"] == pytest.approx(210.0)
    assert values["VERTICAL_DAMPING_N_S_MM"] == pytest.approx(0.05)
    assert _parse_initial_speed(dcf) == pytest.approx(1.0)


def test_bushing_source_parser_preserves_curves_and_assembly_scaling(
    tmp_path: Path,
) -> None:
    database = tmp_path / "shared_car_database.cdb"
    properties = database / "bushings.tbl"
    properties.mkdir(parents=True)
    (properties / "fixture.bus").write_text(
        "\n".join(
            (
                "[UNITS]",
                "LENGTH = 'mm'",
                "FORCE = 'newton'",
                "ANGLE = 'degrees'",
                "MASS = 'kg'",
                "TIME = 'second'",
                "[DAMPING]",
                "FX_DAMPING = 10",
                "TX_DAMPING = 20",
                "[FX_CURVE]",
                "0 0",
                "1 100",
                "[TX_CURVE]",
                "0 0",
                "10 200",
            )
        ),
        encoding="ascii",
    )
    subsystem = tmp_path / "fixture.sub"
    subsystem.write_text(
        "\n".join(
            (
                "[UNITS]",
                "LENGTH = 'mm'",
                "FORCE = 'newton'",
                "ANGLE = 'degrees'",
                "MASS = 'kg'",
                "TIME = 'second'",
                "[BUSHING_ASSEMBLY]",
                "USAGE = 'lca_front'",
                "SYMMETRY = 'left/right'",
                "T_PRELOAD_X = 3",
                "R_PRELOAD_X = 4",
                "FX_SCALING_FACTOR = 2",
                "TX_DAMPING_FORCE_SCALE = 3",
                "PROPERTY_FILE = 'mdids://acar_shared/bushings.tbl/fixture.bus'",
            )
        ),
        encoding="ascii",
    )

    assemblies, parsed = _parse_bushing_sources(subsystem, database)

    assert assemblies[0].preload == pytest.approx((3.0, 0.0, 0.0, 4.0, 0.0, 0.0))
    assert assemblies[0].force_scaling == pytest.approx((2.0, 1.0, 1.0, 1.0, 1.0, 1.0))
    assert assemblies[0].damping_force_scaling == pytest.approx((3.0, 1.0, 1.0, 1.0, 1.0, 1.0))
    property_data = parsed["bushings.tbl/fixture.bus"]
    np.testing.assert_allclose(
        property_data.force_curves[0], ((0.0, 0.0), (1.0, 100.0))
    )
    np.testing.assert_allclose(
        property_data.force_curves[3], ((0.0, 0.0), (np.pi / 18.0, 200.0))
    )
    assert property_data.damping[0] == pytest.approx(10.0)
    assert property_data.damping[3] == pytest.approx(20.0 * 180.0 / np.pi)


def test_source_equivalence_gate_rejects_reduced_adams_mapping() -> None:
    missing: list[str] = []
    notes: list[str] = []
    _audit_source_equivalence(
        {
            "unsupported_adams_user_functions": ("wheel_force",),
            "adams_force_law_mapping": {
                "spring": "source_curve",
                "user_subroutine": "unsupported_explicit_approximation",
            },
            "adams_model_reduction": {
                "status": "partial_runtime_body_mapping",
                "mass_treatment": "omitted_mass_lumped_into_chassis",
                "omitted_part_ids": (12,),
            },
            "native_tire_implementation": "pac2002_selected_combined_slip_with_relaxation_source_offsets",
        },
        [],
        missing,
        notes,
    )

    assert set(missing) == {
        "adams_unit_conversion",
        "unsupported_adams_user_functions",
        "exact_adams_force_law_mapping",
        "complete_adams_body_mapping",
        "source_steering_topology_equivalence",
        "exact_native_pac2002_tire",
        "source_drive_brake_input_contract",
    }
    assert notes


def test_source_equivalence_gate_accepts_condensed_mass_mapping() -> None:
    verified: list[str] = []
    missing: list[str] = []
    _audit_source_equivalence(
        {
            "unit_normalization": {"status": "complete"},
            "unsupported_adams_user_functions": (),
            "adams_force_law_mapping": {"spring": "source_curve"},
            "adams_model_reduction": {
                "status": "exact_part_mapping",
                "mass_treatment": "exact_with_fixed_wheel_mass_condensation",
                "omitted_part_ids": (),
                "steering_internal_treatment": "exact_source_topology",
            },
            "native_tire_implementation": "exact_pac2002",
            "source_drive_brake_input_contract": {
                "source": {
                    "drive": {"status": "zero"},
                    "brake": {"status": "zero"},
                },
                "native_mapping": {"drive": "zero", "brake": "zero"},
            },
        },
        verified,
        missing,
        [],
    )

    assert missing == []
    assert "source_drive_brake_input_contract" in verified
