import math

import numpy as np

from kinematics.steering.workbench import (
    SteeringCurve,
    available_steering_outputs,
    copy_hardpoint_rows,
    curve_specs_for_plot,
    default_steering_project,
    hardpoint_rows_from_csv,
    hardpoints_from_rows,
    input_angle_slider_limits,
    load_steering_project,
    optimize_steering_hardpoints,
    parse_float_entry,
    pitman_angle_slider_limits,
    pitman_arm_x_length,
    pitman_x_position,
    save_hardpoint_rows_csv,
    save_steering_project,
    set_pitman_arm_x_length,
    set_pitman_x_position,
    solve_steering_project,
    sweep_steering_project,
)


def test_project_rows_convert_to_mirrored_hardpoints():
    project = default_steering_project()
    project.hardpoints[2].y = -530.0

    hardpoints = hardpoints_from_rows(project.hardpoints)

    np.testing.assert_allclose(
        hardpoints.left_wheel.wheel_center,
        np.array([60.0, -530.0, 320.0]),
    )
    np.testing.assert_allclose(
        hardpoints.right_wheel.wheel_center,
        np.array([60.0, 530.0, 320.0]),
    )


def test_csv_import_returns_editable_hardpoint_rows(tmp_path):
    csv_path = tmp_path / "hardpoints.csv"
    csv_path.write_text(
        "\n".join(
            [
                "category,name,x,y,z",
                "symmetric,wheel_kingpin_lower,0,-500,280",
                "symmetric,wheel_kingpin_upper,0,-500,340",
                "symmetric,wheel_center,60,-520,320",
                "symmetric,wheel_tie_rod_pickup,-180,-420,280",
                "symmetric,pitman_output,-350,-120,285",
                "center,pitman_pivot,-350,0,300",
            ]
        ),
        encoding="utf-8",
    )

    rows = hardpoint_rows_from_csv(csv_path)

    assert [row.name for row in rows] == [
        "wheel_kingpin_lower",
        "wheel_kingpin_upper",
        "wheel_center",
        "wheel_tie_rod_pickup",
        "pitman_output",
        "pitman_pivot",
    ]
    assert rows[0].y < 0.0


def test_hardpoint_rows_can_be_exported_to_csv(tmp_path):
    path = tmp_path / "exported_hardpoints.csv"
    project = default_steering_project()
    project.hardpoints[2].x = 72.5
    project.hardpoints[2].y = -535.0

    save_hardpoint_rows_csv(project.hardpoints, path)
    loaded = hardpoint_rows_from_csv(path)

    assert path.read_text(encoding="utf-8").splitlines()[0] == "category,name,x,y,z"
    assert [row.name for row in loaded] == [row.name for row in project.hardpoints]
    assert loaded[2].x == 72.5
    assert loaded[2].y == -535.0


def test_copy_hardpoint_rows_returns_independent_rows():
    project = default_steering_project()

    copied = copy_hardpoint_rows(project.hardpoints)
    copied[0].x = 99.0

    assert project.hardpoints[0].x == 0.0
    assert copied[0].x == 99.0


def test_pitman_x_position_moves_pivot_and_outputs_and_updates_tie_rod_length():
    project = default_steering_project()
    set_pitman_arm_x_length(project.hardpoints, 80.0)
    before = hardpoints_from_rows(project.hardpoints).to_2d_geometry()
    wheel_center_x = project.hardpoints[2].x

    set_pitman_x_position(project.hardpoints, -420.0)
    after = hardpoints_from_rows(project.hardpoints).to_2d_geometry()

    assert pitman_x_position(project.hardpoints) == -420.0
    assert pitman_arm_x_length(project.hardpoints) == 80.0
    assert project.hardpoints[2].x == wheel_center_x
    assert after.pitman.pivot[0] == -420.0
    assert after.pitman.left_output[0] == -340.0
    assert after.pitman.right_output[0] == -340.0
    assert after.left_tie_rod_length != before.left_tie_rod_length
    assert after.right_tie_rod_length != before.right_tie_rod_length


def test_pitman_arm_x_length_updates_left_and_right_output_x_from_pivot():
    project = default_steering_project()

    set_pitman_arm_x_length(project.hardpoints, 80.0)
    geometry = hardpoints_from_rows(project.hardpoints).to_2d_geometry()

    assert pitman_x_position(project.hardpoints) == -350.0
    assert pitman_arm_x_length(project.hardpoints) == 80.0
    assert geometry.pitman.pivot[0] == -350.0
    assert geometry.pitman.left_output[0] == -270.0
    assert geometry.pitman.right_output[0] == -270.0


def test_project_can_be_saved_and_loaded(tmp_path):
    path = tmp_path / "steering_project.json"
    project = default_steering_project()
    project.name = "demo steering"
    project.input_mode = "left_wheel_angle"
    project.input_value = 12.5
    project.wheel_radius = 285.0
    project.wheel_width = 205.0
    project.wheelbase = 2800.0
    project.curves.append(
        SteeringCurve(
            x_output="pitman_angle_deg",
            y_output="left_wheel_angle_deg",
            label="left sweep",
        )
    )

    save_steering_project(project, path)
    loaded = load_steering_project(path)

    assert loaded.name == "demo steering"
    assert loaded.input_mode == "left_wheel_angle"
    assert loaded.input_value == 12.5
    assert loaded.wheel_radius == 285.0
    assert loaded.wheel_width == 205.0
    assert loaded.wheelbase == 2800.0
    assert loaded.curves[0].label == "left sweep"
    assert len(loaded.hardpoints) == len(project.hardpoints)


def test_solve_project_supports_all_input_modes():
    project = default_steering_project()
    project.input_mode = "pitman_angle"
    project.input_value = 8.0
    _, outputs = solve_steering_project(project)

    left_angle = outputs["left_wheel_angle_deg"]
    right_angle = outputs["right_wheel_angle_deg"]
    assert left_angle > 0.0
    assert right_angle > 0.0

    project.input_mode = "left_wheel_angle"
    project.input_value = left_angle
    _, left_outputs = solve_steering_project(project)
    np.testing.assert_allclose(left_outputs["pitman_angle_deg"], 8.0, atol=1e-8)

    project.input_mode = "right_wheel_angle"
    project.input_value = right_angle
    _, right_outputs = solve_steering_project(project)
    np.testing.assert_allclose(right_outputs["pitman_angle_deg"], 8.0, atol=1e-8)


def test_solve_project_outputs_current_geometry_steering_limits():
    project = default_steering_project()

    _, outputs = solve_steering_project(project)

    expected_names = {
        "max_left_turn_left_wheel_angle_deg",
        "max_left_turn_right_wheel_angle_deg",
        "max_right_turn_left_wheel_angle_deg",
        "max_right_turn_right_wheel_angle_deg",
    }
    assert expected_names.issubset(outputs)
    assert expected_names.issubset(available_steering_outputs())
    assert outputs["max_left_turn_left_wheel_angle_deg"] > 0.0
    assert outputs["max_left_turn_right_wheel_angle_deg"] > 0.0
    assert outputs["max_right_turn_left_wheel_angle_deg"] < 0.0
    assert outputs["max_right_turn_right_wheel_angle_deg"] < 0.0


def test_solve_project_outputs_ackermann_rate_from_wheelbase():
    project = default_steering_project()
    project.input_mode = "right_wheel_angle"
    project.input_value = 10.0
    project.wheelbase = 2800.0

    _, outputs = solve_steering_project(project)

    actual_ackerman = (
        outputs["right_wheel_angle_deg"] - outputs["left_wheel_angle_deg"]
    )
    inner_angle_deg = max(
        abs(outputs["left_wheel_angle_deg"]),
        abs(outputs["right_wheel_angle_deg"]),
    )
    track = abs(outputs["right_wheel_center_y"] - outputs["left_wheel_center_y"])
    radius_to_inner = project.wheelbase / math.tan(math.radians(inner_angle_deg))
    ideal_outer_angle_deg = math.degrees(
        math.atan2(project.wheelbase, radius_to_inner + track)
    )
    ideal_ackerman = math.copysign(
        inner_angle_deg - ideal_outer_angle_deg,
        actual_ackerman,
    )

    assert "ackermann_rate_pct" in available_steering_outputs()
    np.testing.assert_allclose(
        outputs["ackermann_rate_pct"],
        100.0 * actual_ackerman / ideal_ackerman,
    )


def test_pitman_angle_slider_limits_follow_reachable_geometry_limits():
    project = default_steering_project()

    limits = pitman_angle_slider_limits(project.hardpoints)

    assert limits.minimum < 0.0
    assert limits.maximum > 0.0
    np.testing.assert_allclose(limits.minimum, -15.414004385471344)
    np.testing.assert_allclose(limits.maximum, 15.414004385471344)


def test_input_angle_slider_limits_follow_selected_input_mode():
    project = default_steering_project()

    left_limits = input_angle_slider_limits(project.hardpoints, "left_wheel_angle")
    right_limits = input_angle_slider_limits(project.hardpoints, "right_wheel_angle")

    np.testing.assert_allclose(left_limits.minimum, -21.2185729573261)
    np.testing.assert_allclose(left_limits.maximum, 4.3944304333923725)
    np.testing.assert_allclose(right_limits.minimum, -4.3944304333923725)
    np.testing.assert_allclose(right_limits.maximum, 21.2185729573261)


def test_sweep_project_outputs_selected_variables():
    project = default_steering_project()
    project.input_mode = "pitman_angle"
    project.sweep_min = -8.0
    project.sweep_max = 8.0
    project.sweep_step = 8.0

    rows = sweep_steering_project(project)

    assert [row["pitman_angle_deg"] for row in rows] == [-8.0, 0.0, 8.0]
    assert "left_minus_right_deg" in available_steering_outputs()
    assert rows[0]["left_wheel_angle_deg"] < rows[-1]["left_wheel_angle_deg"]


def test_sweep_project_can_skip_unreachable_wheel_angle_samples():
    project = default_steering_project()
    project.hardpoints[-1].x = -450.0
    project.input_mode = "left_wheel_angle"
    project.sweep_min = -20.0
    project.sweep_max = 20.0
    project.sweep_step = 2.0

    rows = sweep_steering_project(project, skip_unreachable=True)

    assert rows
    assert rows[-1]["input_value"] == 0.0
    assert all(row["input_value"] <= 0.0 for row in rows)


def test_optimize_steering_hardpoints_matches_target_wheel_angle_delta():
    project = default_steering_project()

    result = optimize_steering_hardpoints(
        project.hardpoints,
        inner_wheel="right",
        inner_wheel_angle_deg=10.0,
        target_left_minus_right_deg=-4.0,
        variable_names=(
            "pitman_x",
            "pitman_arm_x_length",
            "tie_rod_outer_y",
            "tie_rod_inner_y",
        ),
        variable_delta_limit=40.0,
    )

    assert abs(result.final_error_deg) < 1e-3
    assert result.initial_error_deg > result.final_error_deg
    assert result.applied_values["pitman_x"] != -350.0
    assert result.applied_values["pitman_arm_x_length"] != 0.0
    assert result.applied_values["tie_rod_outer_y"] != -420.0
    assert result.applied_values["tie_rod_inner_y"] != -120.0


def test_parse_float_entry_preserves_previous_value_during_partial_edits():
    previous = 8.0

    for text in ("", " ", "+", "-", ".", "+.", "-."):
        parsed = parse_float_entry(text, previous)

        assert parsed.value == previous
        assert parsed.is_valid
        assert not parsed.is_complete


def test_parse_float_entry_accepts_numbers_and_flags_invalid_text():
    parsed = parse_float_entry(" 12.5 ", previous=8.0)

    assert parsed.value == 12.5
    assert parsed.is_valid
    assert parsed.is_complete

    invalid = parse_float_entry("abc", previous=8.0)
    assert invalid.value == 8.0
    assert not invalid.is_valid
    assert not invalid.is_complete


def test_curve_specs_preview_selected_outputs_when_no_curves_are_saved():
    specs = curve_specs_for_plot(
        curves=[],
        selected_x_output="pitman_angle_deg",
        selected_y_output="left_wheel_angle_deg",
        selected_label="",
    )

    assert specs == [
        ("pitman_angle_deg", "left_wheel_angle_deg", "left_wheel_angle_deg preview")
    ]


def test_curve_specs_use_saved_curves_for_managed_plotting():
    curves = [
        SteeringCurve(
            x_output="pitman_angle_deg",
            y_output="right_wheel_angle_deg",
            label="right curve",
        )
    ]

    specs = curve_specs_for_plot(
        curves=curves,
        selected_x_output="input_value",
        selected_y_output="left_wheel_angle_deg",
        selected_label="",
    )

    assert specs == [("pitman_angle_deg", "right_wheel_angle_deg", "right curve")]
