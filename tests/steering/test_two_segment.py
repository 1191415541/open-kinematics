import numpy as np

from kinematics.steering import (
    PitmanArmGeometry2D,
    PitmanArmHardpoints3D,
    SteeringCoordinateSystem,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    WheelSteeringGeometry2D,
    WheelSteeringHardpoints3D,
    project_kingpin_axis_to_steering_top_view,
    project_point_to_steering_top_view,
    solve_two_segment_from_left_wheel_angle,
    solve_two_segment_from_right_wheel_angle,
    solve_two_segment_steering,
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
