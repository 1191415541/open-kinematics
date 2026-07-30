import numpy as np

from suspension_kinematics.steering import (
    BellcrankGeometry2D,
    BellcrankHardpoints3D,
    ThreeSegmentSteeringAnalyticComparison,
    ThreeSegmentSteeringGeometry,
    ThreeSegmentSteeringHardpoints3D,
    WheelSteeringGeometry2D,
    WheelSteeringHardpoints3D,
    compare_three_segment_3d_analytic_and_semi_analytic,
    solve_three_segment_from_left_wheel_angle,
    solve_three_segment_steering,
    solve_three_segment_steering_3d,
    solve_three_segment_steering_3d_analytic,
    solve_three_segment_steering_3d_semi_analytic,
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


def symmetric_three_segment_hardpoints_3d() -> ThreeSegmentSteeringHardpoints3D:
    return ThreeSegmentSteeringHardpoints3D(
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
        left_bellcrank=BellcrankHardpoints3D(
            pivot=np.array([-260.0, -320.0, 300.0]),
            center_link_pickup=np.array([-460.0, -300.0, 302.0]),
            tie_rod_pickup=np.array([-300.0, -300.0, 295.0]),
        ),
        right_bellcrank=BellcrankHardpoints3D(
            pivot=np.array([-260.0, 320.0, 301.0]),
            center_link_pickup=np.array([-460.0, 300.0, 303.0]),
            tie_rod_pickup=np.array([-300.0, 300.0, 296.0]),
        ),
    )


def inclined_three_segment_hardpoints_3d() -> ThreeSegmentSteeringHardpoints3D:
    return ThreeSegmentSteeringHardpoints3D(
        left_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([10.0, -500.0, 280.0]),
            kingpin_upper=np.array([45.0, -545.0, 342.0]),
            wheel_center=np.array([60.0, -520.0, 320.0]),
            tie_rod_pickup=np.array([-180.0, -420.0, 281.0]),
        ),
        right_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([10.0, 500.0, 280.0]),
            kingpin_upper=np.array([45.0, 545.0, 341.0]),
            wheel_center=np.array([60.0, 520.0, 319.0]),
            tie_rod_pickup=np.array([-180.0, 420.0, 282.0]),
        ),
        left_bellcrank=BellcrankHardpoints3D(
            pivot=np.array([-260.0, -320.0, 300.0]),
            center_link_pickup=np.array([-460.0, -300.0, 302.0]),
            tie_rod_pickup=np.array([-300.0, -300.0, 295.0]),
            axis=np.array([0.08, -0.03, 1.0]),
        ),
        right_bellcrank=BellcrankHardpoints3D(
            pivot=np.array([-260.0, 320.0, 301.0]),
            center_link_pickup=np.array([-460.0, 300.0, 303.0]),
            tie_rod_pickup=np.array([-300.0, 300.0, 296.0]),
            axis=np.array([-0.08, -0.03, 1.0]),
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
            solution.left_tie_rod_pickup - solution.left_bellcrank_tie_rod_pickup
        ),
        geometry.left_tie_rod_length,
        atol=1e-6,
    )
    np.testing.assert_allclose(
        np.linalg.norm(
            solution.right_tie_rod_pickup - solution.right_bellcrank_tie_rod_pickup
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


def test_three_dimensional_hardpoints_project_to_two_dimensional_geometry():
    geometry = symmetric_three_segment_hardpoints_3d().to_2d_geometry()

    np.testing.assert_allclose(geometry.left_wheel.kingpin, np.array([0.0, -500.0]))
    np.testing.assert_allclose(geometry.right_wheel.kingpin, np.array([0.0, 500.0]))
    np.testing.assert_allclose(
        geometry.left_bellcrank.center_link_pickup,
        np.array([-460.0, -300.0]),
    )
    np.testing.assert_allclose(
        geometry.right_bellcrank.tie_rod_pickup,
        np.array([-300.0, 300.0]),
    )


def test_zero_left_bellcrank_angle_keeps_three_segment_3d_design_state():
    solution = solve_three_segment_steering_3d_analytic(
        symmetric_three_segment_hardpoints_3d(),
        left_bellcrank_angle_deg=0.0,
    )

    assert solution.converged
    assert solution.has_3d_state
    np.testing.assert_allclose(solution.left_bellcrank_angle_deg, 0.0)
    np.testing.assert_allclose(solution.right_bellcrank_angle_deg, 0.0, atol=1e-10)
    np.testing.assert_allclose(solution.left_wheel_angle_deg, 0.0, atol=1e-10)
    np.testing.assert_allclose(solution.right_wheel_angle_deg, 0.0, atol=1e-10)
    assert solution.max_abs_link_residual < 1e-10


def test_three_segment_3d_default_solver_uses_analytic_solution():
    hardpoints = symmetric_three_segment_hardpoints_3d()

    default_solution = solve_three_segment_steering_3d(
        hardpoints,
        left_bellcrank_angle_deg=8.0,
    )
    analytic_solution = solve_three_segment_steering_3d_analytic(
        hardpoints,
        left_bellcrank_angle_deg=8.0,
    )

    assert default_solution.nfev == 0
    np.testing.assert_allclose(
        default_solution.right_bellcrank_angle_deg,
        analytic_solution.right_bellcrank_angle_deg,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        default_solution.left_wheel_angle_deg,
        analytic_solution.left_wheel_angle_deg,
        atol=1e-12,
    )
    np.testing.assert_allclose(
        default_solution.right_wheel_angle_deg,
        analytic_solution.right_wheel_angle_deg,
        atol=1e-12,
    )


def test_three_segment_3d_analytic_and_semi_analytic_match():
    comparison = compare_three_segment_3d_analytic_and_semi_analytic(
        symmetric_three_segment_hardpoints_3d(),
        left_bellcrank_angle_deg=8.0,
    )

    assert isinstance(comparison, ThreeSegmentSteeringAnalyticComparison)
    assert comparison.solve_analytic.converged
    assert comparison.solve_semi_analytic.converged
    assert comparison.solve_analytic.nfev == 0
    assert comparison.solve_semi_analytic.nfev > 0
    assert comparison.max_abs_angle_delta_deg < 1e-10
    assert comparison.max_abs_link_residual_delta < 1e-10


def test_three_segment_3d_sweep_stays_on_continuation_branch():
    hardpoints = symmetric_three_segment_hardpoints_3d()
    angles = [-8.0, -4.0, 0.0, 4.0, 8.0]

    guess = (0.0, 0.0, 0.0)
    solutions = []
    for angle in angles:
        solution = solve_three_segment_steering_3d_analytic(
            hardpoints,
            left_bellcrank_angle_deg=angle,
            initial_guess_deg=guess,
        )
        solutions.append(solution)
        guess = (
            solution.right_bellcrank_angle_deg,
            solution.left_wheel_angle_deg,
            solution.right_wheel_angle_deg,
        )

    assert all(solution.converged for solution in solutions)
    assert all(solution.has_3d_state for solution in solutions)
    assert solutions[0].left_wheel_angle_deg < solutions[-1].left_wheel_angle_deg
    assert solutions[0].right_wheel_angle_deg < solutions[-1].right_wheel_angle_deg
    deltas = np.diff([solution.right_bellcrank_angle_deg for solution in solutions])
    assert np.all(np.abs(deltas) < 10.0)


def test_three_segment_3d_hardpoints_work_with_projected_2d_solver():
    solution = solve_three_segment_steering(
        symmetric_three_segment_hardpoints_3d(),
        left_bellcrank_angle_deg=8.0,
    )

    assert solution.converged
    assert solution.left_wheel_center.shape == (2,)
    assert solution.right_wheel_center.shape == (2,)


def test_three_segment_3d_semi_analytic_matches_analytic_over_sweep():
    hardpoints = symmetric_three_segment_hardpoints_3d()

    guess = (0.0, 0.0, 0.0)
    for angle in np.linspace(-8.0, 8.0, 9):
        analytic = solve_three_segment_steering_3d_analytic(
            hardpoints,
            left_bellcrank_angle_deg=float(angle),
            initial_guess_deg=guess,
        )
        semi_analytic = solve_three_segment_steering_3d_semi_analytic(
            hardpoints,
            left_bellcrank_angle_deg=float(angle),
            initial_guess_deg=guess,
        )
        np.testing.assert_allclose(
            semi_analytic.right_bellcrank_angle_deg,
            analytic.right_bellcrank_angle_deg,
            atol=1e-10,
        )
        np.testing.assert_allclose(
            semi_analytic.left_wheel_angle_deg,
            analytic.left_wheel_angle_deg,
            atol=1e-10,
        )
        np.testing.assert_allclose(
            semi_analytic.right_wheel_angle_deg,
            analytic.right_wheel_angle_deg,
            atol=1e-10,
        )
        assert analytic.max_abs_link_residual < 1e-10
        assert semi_analytic.max_abs_link_residual < 1e-10
        guess = (
            analytic.right_bellcrank_angle_deg,
            analytic.left_wheel_angle_deg,
            analytic.right_wheel_angle_deg,
        )


def test_inclined_three_segment_3d_analytic_and_semi_analytic_match():
    hardpoints = inclined_three_segment_hardpoints_3d()

    guess = (0.0, 0.0, 0.0)
    for angle in [0.0, 2.0, 4.0, 8.0]:
        analytic = solve_three_segment_steering_3d_analytic(
            hardpoints,
            left_bellcrank_angle_deg=float(angle),
            initial_guess_deg=guess,
        )
        semi_analytic = solve_three_segment_steering_3d_semi_analytic(
            hardpoints,
            left_bellcrank_angle_deg=float(angle),
            initial_guess_deg=guess,
        )
        assert analytic.converged
        assert semi_analytic.converged
        np.testing.assert_allclose(
            semi_analytic.right_bellcrank_angle_deg,
            analytic.right_bellcrank_angle_deg,
            atol=1e-10,
        )
        np.testing.assert_allclose(
            semi_analytic.left_wheel_angle_deg,
            analytic.left_wheel_angle_deg,
            atol=1e-10,
        )
        np.testing.assert_allclose(
            semi_analytic.right_wheel_angle_deg,
            analytic.right_wheel_angle_deg,
            atol=1e-10,
        )
        guess = (
            analytic.right_bellcrank_angle_deg,
            analytic.left_wheel_angle_deg,
            analytic.right_wheel_angle_deg,
        )
