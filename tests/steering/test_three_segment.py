import numpy as np

from kinematics.steering import (
    BellcrankGeometry2D,
    ThreeSegmentSteeringGeometry,
    WheelSteeringGeometry2D,
    solve_three_segment_from_left_wheel_angle,
    solve_three_segment_steering,
    sweep_three_segment_steering,
)


def symmetric_three_segment_geometry() -> ThreeSegmentSteeringGeometry:
    return ThreeSegmentSteeringGeometry(
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
        left_bellcrank=BellcrankGeometry2D(
            pivot=np.array([-260.0, -320.0]),
            center_link_pickup=np.array([-460.0, -300.0]),
            tie_rod_pickup=np.array([-300.0, -300.0]),
        ),
        right_bellcrank=BellcrankGeometry2D(
            pivot=np.array([-260.0, 320.0]),
            center_link_pickup=np.array([-460.0, 300.0]),
            tie_rod_pickup=np.array([-300.0, 300.0]),
        ),
    )


def test_zero_left_bellcrank_angle_keeps_three_segment_design_state():
    solution = solve_three_segment_steering(
        symmetric_three_segment_geometry(),
        left_bellcrank_angle_deg=0.0,
    )

    assert solution.converged
    np.testing.assert_allclose(solution.left_bellcrank_angle_deg, 0.0)
    np.testing.assert_allclose(solution.right_bellcrank_angle_deg, 0.0, atol=1e-10)
    np.testing.assert_allclose(solution.left_wheel_angle_deg, 0.0, atol=1e-10)
    np.testing.assert_allclose(solution.right_wheel_angle_deg, 0.0, atol=1e-10)
    assert solution.max_abs_link_residual < 1e-8


def test_left_bellcrank_angle_solves_three_segment_linkage():
    geometry = symmetric_three_segment_geometry()
    solution = solve_three_segment_steering(
        geometry,
        left_bellcrank_angle_deg=8.0,
    )

    assert solution.converged
    assert solution.right_bellcrank_angle_deg > 0.0
    assert solution.left_wheel_angle_deg > 0.0
    assert solution.right_wheel_angle_deg > 0.0
    assert solution.left_bellcrank_center_link_pickup.shape == (2,)
    assert solution.right_bellcrank_center_link_pickup.shape == (2,)
    assert solution.left_bellcrank_tie_rod_pickup.shape == (2,)
    assert solution.right_bellcrank_tie_rod_pickup.shape == (2,)
    assert solution.left_wheel_center.shape == (2,)
    assert solution.right_wheel_center.shape == (2,)
    assert solution.max_abs_link_residual < 1e-6

    np.testing.assert_allclose(
        np.linalg.norm(
            solution.left_bellcrank_center_link_pickup
            - solution.right_bellcrank_center_link_pickup
        ),
        geometry.center_link_length,
        atol=1e-6,
    )
    np.testing.assert_allclose(
        np.linalg.norm(
            solution.left_tie_rod_pickup
            - solution.left_bellcrank_tie_rod_pickup
        ),
        geometry.left_tie_rod_length,
        atol=1e-6,
    )
    np.testing.assert_allclose(
        np.linalg.norm(
            solution.right_tie_rod_pickup
            - solution.right_bellcrank_tie_rod_pickup
        ),
        geometry.right_tie_rod_length,
        atol=1e-6,
    )


def test_left_bellcrank_angle_uses_closed_form_geometry():
    solution = solve_three_segment_steering(
        symmetric_three_segment_geometry(),
        left_bellcrank_angle_deg=8.0,
    )

    assert solution.converged
    assert solution.nfev == 0
    assert solution.max_abs_link_residual < 1e-8


def test_left_wheel_angle_inverse_prefers_nearest_bellcrank_branch():
    geometry = symmetric_three_segment_geometry()

    solution = solve_three_segment_from_left_wheel_angle(
        geometry,
        left_wheel_angle_deg=2.3,
        initial_left_bellcrank_guess_deg=9.3,
    )

    np.testing.assert_allclose(solution.left_wheel_angle_deg, 2.3, atol=1e-6)
    assert abs(solution.left_bellcrank_angle_deg - 9.3) < 2.0


def test_symmetric_three_segment_sweep_uses_continuation():
    angles = [-8.0, -4.0, 0.0, 4.0, 8.0]

    solutions = sweep_three_segment_steering(
        symmetric_three_segment_geometry(),
        angles,
    )

    assert [s.left_bellcrank_angle_deg for s in solutions] == angles
    assert all(s.converged for s in solutions)
    assert solutions[0].left_wheel_angle_deg < solutions[-1].left_wheel_angle_deg
