import numpy as np
import pytest

from suspension_kinematics.steering import (
    PitmanArmGeometry2D,
    PitmanArmHardpoints3D,
    SteeringCoordinateSystem,
    TwoSegmentSteeringAnalyticComparison,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    WheelSteeringGeometry2D,
    WheelSteeringHardpoints3D,
    compare_two_segment_2d_and_3d,
    compare_two_segment_3d_analytic_and_numeric,
    pinion_angle_from_rack_displacement,
    project_kingpin_axis_to_steering_top_view,
    project_point_to_steering_top_view,
    rack_displacement_from_pinion_angle,
    solve_two_segment_from_left_wheel_angle,
    solve_two_segment_from_left_wheel_angle_3d,
    solve_two_segment_from_left_wheel_angle_3d_analytic,
    solve_two_segment_from_right_wheel_angle,
    solve_two_segment_rack_and_pinion_3d_analytic,
    solve_two_segment_steering,
    solve_two_segment_steering_3d_analytic,
    sweep_two_segment_steering,
)


def symmetric_geometry() -> TwoSegmentSteeringGeometry:
    return TwoSegmentSteeringGeometry(
        left_wheel=WheelSteeringGeometry2D(
            kingpin=np.array([0.0, -500.0]),
            wheel_center=np.array([60.0, -520.0]),
            tie_rod_pickup=np.array([-180.0, -420.0]),
        ),
        right_wheel=WheelSteeringGeometry2D(
            kingpin=np.array([0.0, 500.0]),
            wheel_center=np.array([60.0, 520.0]),
            tie_rod_pickup=np.array([-180.0, 420.0]),
        ),
        pitman=PitmanArmGeometry2D(
            pivot=np.array([-350.0, 0.0]),
            left_output=np.array([-350.0, -120.0]),
            right_output=np.array([-350.0, 120.0]),
        ),
    )


def symmetric_hardpoints_3d() -> TwoSegmentSteeringHardpoints3D:
    return TwoSegmentSteeringHardpoints3D(
        left_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([0.0, -500.0, 280.0]),
            kingpin_upper=np.array([0.0, -500.0, 340.0]),
            wheel_center=np.array([60.0, -520.0, 320.0]),
            tie_rod_pickup=np.array([-180.0, -420.0, 280.0]),
        ),
        right_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([0.0, 500.0, 280.0]),
            kingpin_upper=np.array([0.0, 500.0, 340.0]),
            wheel_center=np.array([60.0, 520.0, 319.0]),
            tie_rod_pickup=np.array([-180.0, 420.0, 281.0]),
        ),
        pitman=PitmanArmHardpoints3D(
            pivot=np.array([-350.0, 0.0, 300.0]),
            left_output=np.array([-350.0, -120.0, 285.0]),
            right_output=np.array([-350.0, 120.0, 286.0]),
        ),
    )


def inclined_hardpoints_3d() -> TwoSegmentSteeringHardpoints3D:
    return TwoSegmentSteeringHardpoints3D(
        left_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([10.0, -500.0, 280.0]),
            kingpin_upper=np.array([50.0, -560.0, 340.0]),
            wheel_center=np.array([60.0, -520.0, 320.0]),
            tie_rod_pickup=np.array([-180.0, -420.0, 280.0]),
        ),
        right_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([10.0, 500.0, 280.0]),
            kingpin_upper=np.array([50.0, 560.0, 340.0]),
            wheel_center=np.array([60.0, 520.0, 320.0]),
            tie_rod_pickup=np.array([-180.0, 420.0, 280.0]),
        ),
        pitman=PitmanArmHardpoints3D(
            pivot=np.array([-350.0, 0.0, 300.0]),
            left_output=np.array([-350.0, -120.0, 285.0]),
            right_output=np.array([-350.0, 120.0, 285.0]),
        ),
    )


def unreachable_left_wheel_target_hardpoints_3d() -> TwoSegmentSteeringHardpoints3D:
    return TwoSegmentSteeringHardpoints3D(
        left_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([0.0, -500.0, 280.0]),
            kingpin_upper=np.array([0.0, -500.0, 340.0]),
            wheel_center=np.array([60.0, -520.0, 320.0]),
            tie_rod_pickup=np.array([-180.0, -420.0, 280.0]),
        ),
        right_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([0.0, 500.0, 280.0]),
            kingpin_upper=np.array([0.0, 500.0, 340.0]),
            wheel_center=np.array([60.0, 520.0, 320.0]),
            tie_rod_pickup=np.array([-180.0, 420.0, 281.0]),
        ),
        pitman=PitmanArmHardpoints3D(
            pivot=np.array([-450.0, 0.0, 300.0]),
            left_output=np.array([-450.0, -120.0, 285.0]),
            right_output=np.array([-450.0, 120.0, 286.0]),
        ),
    )


def test_steering_coordinate_system_is_rear_right_up():
    np.testing.assert_allclose(
        SteeringCoordinateSystem.X_REAR,
        np.array([1.0, 0.0, 0.0]),
    )
    np.testing.assert_allclose(
        SteeringCoordinateSystem.Y_RIGHT,
        np.array([0.0, 1.0, 0.0]),
    )
    np.testing.assert_allclose(
        SteeringCoordinateSystem.Z_UP,
        np.array([0.0, 0.0, 1.0]),
    )
    np.testing.assert_allclose(
        np.cross(SteeringCoordinateSystem.X_REAR, SteeringCoordinateSystem.Y_RIGHT),
        SteeringCoordinateSystem.Z_UP,
    )


def test_steering_top_view_places_left_side_at_negative_y():
    geometry = symmetric_geometry()

    assert geometry.left_wheel.kingpin[1] < 0.0
    assert geometry.right_wheel.kingpin[1] > 0.0


def test_project_point_to_steering_top_view_uses_x_rear_and_y_right():
    point = np.array([12.0, -34.0, 56.0])

    projected = project_point_to_steering_top_view(point)

    np.testing.assert_allclose(projected, np.array([12.0, -34.0]))


def test_project_kingpin_axis_uses_wheel_center_height():
    lower = np.array([0.0, -500.0, 280.0])
    upper = np.array([30.0, -560.0, 340.0])

    projected = project_kingpin_axis_to_steering_top_view(
        lower,
        upper,
        reference_z=320.0,
    )

    np.testing.assert_allclose(projected, np.array([20.0, -540.0]))


def test_three_dimensional_hardpoints_map_to_two_dimensional_geometry():
    geometry = symmetric_hardpoints_3d().to_2d_geometry()

    np.testing.assert_allclose(geometry.left_wheel.kingpin, np.array([0.0, -500.0]))
    np.testing.assert_allclose(geometry.right_wheel.kingpin, np.array([0.0, 500.0]))
    np.testing.assert_allclose(geometry.pitman.left_output, np.array([-350.0, -120.0]))
    np.testing.assert_allclose(geometry.pitman.right_output, np.array([-350.0, 120.0]))


def test_three_dimensional_hardpoints_can_drive_solver_directly():
    solution = solve_two_segment_steering(
        symmetric_hardpoints_3d(),
        pitman_angle_deg=8.0,
    )

    assert solution.converged
    assert solution.left_wheel_angle_deg > 0.0
    assert solution.right_wheel_angle_deg > 0.0


def test_zero_pitman_angle_keeps_both_wheels_at_design_angle():
    solution = solve_two_segment_steering(symmetric_geometry(), pitman_angle_deg=0.0)

    assert solution.converged
    np.testing.assert_allclose(solution.left_wheel_angle_deg, 0.0, atol=1e-10)
    np.testing.assert_allclose(solution.right_wheel_angle_deg, 0.0, atol=1e-10)
    assert solution.max_abs_tie_rod_residual < 1e-9


def test_pinion_angle_and_rack_displacement_are_inverse_operations():
    pinion_pitch_radius_mm = 15.0
    pinion_angle_deg = 12.0

    displacement = rack_displacement_from_pinion_angle(
        pinion_angle_deg,
        pinion_pitch_radius_mm,
    )

    np.testing.assert_allclose(
        pinion_angle_from_rack_displacement(
            displacement,
            pinion_pitch_radius_mm,
        ),
        pinion_angle_deg,
        atol=1e-12,
    )

    with pytest.raises(ValueError, match="Pinion pitch radius must be positive"):
        rack_displacement_from_pinion_angle(1.0, 0.0)
    with pytest.raises(ValueError, match="Pinion pitch radius must be positive"):
        pinion_angle_from_rack_displacement(1.0, -1.0)


def test_rack_and_pinion_zero_travel_preserves_design_state():
    hardpoints = symmetric_hardpoints_3d()

    solution = solve_two_segment_rack_and_pinion_3d_analytic(
        hardpoints,
        rack_displacement_mm=0.0,
    )

    assert solution.converged
    np.testing.assert_allclose(solution.left_wheel_angle_deg, 0.0, atol=1e-10)
    np.testing.assert_allclose(solution.right_wheel_angle_deg, 0.0, atol=1e-10)
    np.testing.assert_allclose(
        solution.pitman_left_output_3d,
        hardpoints.pitman.left_output,
    )
    np.testing.assert_allclose(
        solution.pitman_right_output_3d,
        hardpoints.pitman.right_output,
    )


def test_rack_and_pinion_translates_both_inner_tie_rod_joints():
    hardpoints = symmetric_hardpoints_3d()
    rack_displacement_mm = 2.0

    solution = solve_two_segment_rack_and_pinion_3d_analytic(
        hardpoints,
        rack_displacement_mm=rack_displacement_mm,
    )

    expected_translation = np.array([0.0, rack_displacement_mm, 0.0])
    assert solution.converged
    assert solution.max_abs_tie_rod_residual < 1e-6
    np.testing.assert_allclose(
        solution.pitman_left_output_3d,
        hardpoints.pitman.left_output + expected_translation,
    )
    np.testing.assert_allclose(
        solution.pitman_right_output_3d,
        hardpoints.pitman.right_output + expected_translation,
    )


def test_pitman_angle_solves_left_and_right_wheel_angles():
    solution = solve_two_segment_steering(symmetric_geometry(), pitman_angle_deg=8.0)

    assert solution.converged
    assert solution.left_wheel_angle_deg > 0.0
    assert solution.right_wheel_angle_deg > 0.0
    assert solution.left_tie_rod_pickup.shape == (2,)
    assert solution.right_tie_rod_pickup.shape == (2,)
    assert solution.left_wheel_center.shape == (2,)
    assert solution.right_wheel_center.shape == (2,)
    assert solution.max_abs_tie_rod_residual < 1e-6


def test_wheel_centers_rotate_about_kingpins():
    geometry = symmetric_geometry()
    solution = solve_two_segment_steering(geometry, pitman_angle_deg=8.0)

    angle = np.deg2rad(solution.left_wheel_angle_deg)
    offset = geometry.left_wheel.wheel_center - geometry.left_wheel.kingpin
    expected = geometry.left_wheel.kingpin + np.array(
        [
            offset[0] * np.cos(angle) - offset[1] * np.sin(angle),
            offset[0] * np.sin(angle) + offset[1] * np.cos(angle),
        ]
    )

    np.testing.assert_allclose(solution.left_wheel_center, expected, atol=1e-8)
    assert not np.allclose(solution.left_wheel_center, geometry.left_wheel.kingpin)


def test_wheel_center_radius_is_preserved_about_kingpin():
    geometry = symmetric_geometry()
    solution = solve_two_segment_steering(geometry, pitman_angle_deg=8.0)

    design_radius = np.linalg.norm(
        geometry.left_wheel.wheel_center - geometry.left_wheel.kingpin
    )
    solved_radius = np.linalg.norm(
        solution.left_wheel_center - geometry.left_wheel.kingpin
    )

    np.testing.assert_allclose(solved_radius, design_radius, atol=1e-8)


def test_symmetric_geometry_produces_equal_and_opposite_sweep_directions():
    positive = solve_two_segment_steering(symmetric_geometry(), pitman_angle_deg=8.0)
    negative = solve_two_segment_steering(symmetric_geometry(), pitman_angle_deg=-8.0)

    np.testing.assert_allclose(
        positive.left_wheel_angle_deg,
        -negative.right_wheel_angle_deg,
        atol=1e-8,
    )
    np.testing.assert_allclose(
        positive.right_wheel_angle_deg,
        -negative.left_wheel_angle_deg,
        atol=1e-8,
    )


def test_tie_rod_lengths_are_preserved_after_solving():
    geometry = symmetric_geometry()
    solution = solve_two_segment_steering(geometry, pitman_angle_deg=12.0)

    design_left = np.linalg.norm(
        geometry.left_wheel.tie_rod_pickup - geometry.pitman.left_output
    )
    design_right = np.linalg.norm(
        geometry.right_wheel.tie_rod_pickup - geometry.pitman.right_output
    )
    solved_left = np.linalg.norm(
        solution.left_tie_rod_pickup - solution.pitman_left_output
    )
    solved_right = np.linalg.norm(
        solution.right_tie_rod_pickup - solution.pitman_right_output
    )

    np.testing.assert_allclose(solved_left, design_left, atol=1e-6)
    np.testing.assert_allclose(solved_right, design_right, atol=1e-6)


def test_sweep_uses_previous_solution_as_next_initial_guess():
    angles = [-8.0, -4.0, 0.0, 4.0, 8.0]

    solutions = sweep_two_segment_steering(symmetric_geometry(), angles)

    assert [s.pitman_angle_deg for s in solutions] == angles
    assert len(solutions) == len(angles)
    assert all(s.converged for s in solutions)
    assert solutions[0].left_wheel_angle_deg < solutions[-1].left_wheel_angle_deg


def test_left_wheel_angle_can_drive_two_segment_steering():
    geometry = symmetric_geometry()
    reference = solve_two_segment_steering(geometry, pitman_angle_deg=8.0)

    solution = solve_two_segment_from_left_wheel_angle(
        geometry,
        left_wheel_angle_deg=reference.left_wheel_angle_deg,
    )

    np.testing.assert_allclose(solution.pitman_angle_deg, 8.0, atol=1e-8)
    np.testing.assert_allclose(
        solution.left_wheel_angle_deg,
        reference.left_wheel_angle_deg,
        atol=1e-8,
    )
    np.testing.assert_allclose(
        solution.right_wheel_angle_deg,
        reference.right_wheel_angle_deg,
        atol=1e-8,
    )
    assert solution.converged


def test_three_dimensional_solver_matches_two_dimensional_projection_when_vertical():
    hardpoints = symmetric_hardpoints_3d()

    comparison = compare_two_segment_2d_and_3d(
        hardpoints,
        pitman_angle_deg=8.0,
    )

    assert comparison.solve_2d.converged
    assert comparison.solve_3d.converged
    np.testing.assert_allclose(comparison.left_wheel_angle_delta_deg, 0.0, atol=1e-8)
    np.testing.assert_allclose(comparison.right_wheel_angle_delta_deg, 0.0, atol=1e-8)


def test_three_dimensional_solver_preserves_tie_rod_lengths():
    hardpoints = inclined_hardpoints_3d()

    comparison = compare_two_segment_2d_and_3d(
        hardpoints,
        pitman_angle_deg=8.0,
    )

    design_left = np.linalg.norm(
        hardpoints.left_wheel.tie_rod_pickup - hardpoints.pitman.left_output
    )
    design_right = np.linalg.norm(
        hardpoints.right_wheel.tie_rod_pickup - hardpoints.pitman.right_output
    )
    solved_left = np.linalg.norm(
        comparison.solve_3d.left_tie_rod_pickup_3d
        - comparison.solve_3d.pitman_left_output_3d
    )
    solved_right = np.linalg.norm(
        comparison.solve_3d.right_tie_rod_pickup_3d
        - comparison.solve_3d.pitman_right_output_3d
    )

    np.testing.assert_allclose(solved_left, design_left, atol=1e-6)
    np.testing.assert_allclose(solved_right, design_right, atol=1e-6)


def test_three_dimensional_analytic_solver_preserves_tie_rod_lengths():
    hardpoints = inclined_hardpoints_3d()

    solution = solve_two_segment_steering_3d_analytic(
        hardpoints,
        pitman_angle_deg=8.0,
    )

    design_left = np.linalg.norm(
        hardpoints.left_wheel.tie_rod_pickup - hardpoints.pitman.left_output
    )
    design_right = np.linalg.norm(
        hardpoints.right_wheel.tie_rod_pickup - hardpoints.pitman.right_output
    )
    solved_left = np.linalg.norm(
        solution.left_tie_rod_pickup_3d - solution.pitman_left_output_3d
    )
    solved_right = np.linalg.norm(
        solution.right_tie_rod_pickup_3d - solution.pitman_right_output_3d
    )

    np.testing.assert_allclose(solved_left, design_left, atol=1e-6)
    np.testing.assert_allclose(solved_right, design_right, atol=1e-6)


def test_three_dimensional_analytic_solver_matches_numeric_forward_solution():
    hardpoints = inclined_hardpoints_3d()

    comparison = compare_two_segment_3d_analytic_and_numeric(
        hardpoints,
        pitman_angle_deg=8.0,
    )

    assert isinstance(comparison, TwoSegmentSteeringAnalyticComparison)
    assert comparison.solve_numeric.converged
    assert comparison.solve_analytic.converged
    assert comparison.max_abs_wheel_angle_delta_deg < 1e-6
    assert comparison.max_abs_tie_rod_residual_delta < 1e-6


def test_three_dimensional_analytic_inverse_matches_numeric_solution():
    hardpoints = inclined_hardpoints_3d()
    forward = solve_two_segment_steering_3d_analytic(
        hardpoints,
        pitman_angle_deg=8.0,
    )
    reference = solve_two_segment_from_left_wheel_angle_3d(
        hardpoints,
        left_wheel_angle_deg=forward.left_wheel_angle_deg,
    )

    analytic = solve_two_segment_from_left_wheel_angle_3d_analytic(
        hardpoints,
        left_wheel_angle_deg=forward.left_wheel_angle_deg,
    )

    np.testing.assert_allclose(
        analytic.pitman_angle_deg,
        reference.pitman_angle_deg,
        atol=1e-6,
    )
    np.testing.assert_allclose(
        analytic.right_wheel_angle_deg,
        reference.right_wheel_angle_deg,
        atol=1e-6,
    )
    assert analytic.max_abs_tie_rod_residual < 1e-6


def test_three_dimensional_solver_differs_from_projection_for_inclined_kingpin():
    comparison = compare_two_segment_2d_and_3d(
        inclined_hardpoints_3d(),
        pitman_angle_deg=8.0,
    )

    assert comparison.solve_2d.converged
    assert comparison.solve_3d.converged
    assert abs(comparison.left_wheel_angle_delta_deg) > 1e-3
    assert abs(comparison.right_wheel_angle_delta_deg) > 1e-3


def test_right_wheel_angle_can_drive_two_segment_steering():
    geometry = symmetric_geometry()
    reference = solve_two_segment_steering(geometry, pitman_angle_deg=-8.0)

    solution = solve_two_segment_from_right_wheel_angle(
        geometry,
        right_wheel_angle_deg=reference.right_wheel_angle_deg,
    )

    np.testing.assert_allclose(solution.pitman_angle_deg, -8.0, atol=1e-8)
    np.testing.assert_allclose(
        solution.right_wheel_angle_deg,
        reference.right_wheel_angle_deg,
        atol=1e-8,
    )
    np.testing.assert_allclose(
        solution.left_wheel_angle_deg,
        reference.left_wheel_angle_deg,
        atol=1e-8,
    )
    assert solution.converged


def test_three_dimensional_left_wheel_inverse_rejects_unreachable_target() -> None:
    hardpoints = unreachable_left_wheel_target_hardpoints_3d()

    with pytest.raises(ValueError, match="No valid pitman arm position"):
        solve_two_segment_from_left_wheel_angle_3d(
            hardpoints,
            left_wheel_angle_deg=20.0,
        )
