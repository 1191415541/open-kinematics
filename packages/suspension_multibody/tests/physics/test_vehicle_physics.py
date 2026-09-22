"""Self-consistency checks for full-vehicle loads and roll-center geometry."""

import numpy as np

from suspension_multibody.analysis import (
    compute_static_wheel_loads,
    compute_vehicle_roll_centers,
)


def test_static_wheel_loads_balance_weight_and_moments(full_vehicle_model) -> None:
    result = compute_static_wheel_loads(full_vehicle_model)

    assert result.rank == 3
    assert result.residual < 1e-8
    assert all(value > 0.0 for value in result.wheel_loads.values())
    assert np.isclose(
        result.summary.total,
        result.total_mass * 9810.0,
        rtol=0.0,
        atol=1e-8,
    )
    assert np.isclose(result.summary.left_side, result.summary.right_side)
    assert np.isclose(result.summary.front_axle, result.summary.rear_axle)


def test_longitudinal_acceleration_transfers_load_rearward(full_vehicle_model) -> None:
    static = compute_static_wheel_loads(full_vehicle_model)
    accelerated = compute_static_wheel_loads(
        full_vehicle_model,
        acceleration=np.array([1_000.0, 0.0, 0.0]),
    )

    assert accelerated.summary.front_axle < static.summary.front_axle
    assert accelerated.summary.rear_axle > static.summary.rear_axle


def test_positive_lateral_acceleration_transfers_load_to_negative_y_side(
    full_vehicle_model,
) -> None:
    static = compute_static_wheel_loads(full_vehicle_model)
    accelerated = compute_static_wheel_loads(
        full_vehicle_model,
        acceleration=np.array([0.0, 1_000.0, 0.0]),
    )

    assert accelerated.summary.left_side > static.summary.left_side
    assert accelerated.summary.right_side < static.summary.right_side
    assert accelerated.summary.right_left_delta < 0.0


def test_front_and_rear_roll_centers_are_finite_and_symmetric(full_vehicle_model) -> None:
    centers = compute_vehicle_roll_centers(full_vehicle_model)

    assert set(centers) == {"front", "rear"}
    for result in centers.values():
        assert np.all(np.isfinite(result.center))
        assert np.isclose(result.center[0], 0.0, atol=1e-8)
        assert np.isclose(
            result.left_instant_center[0], -result.right_instant_center[0]
        )


def test_static_wheel_loads_are_the_minimum_norm_solution(full_vehicle_model) -> None:
    """
    The retained Python solver must still return the *minimum norm* load split.

    The four vertical reactions are underdetermined (three balance equations),
    so the algorithm -- not the physics alone -- picks one of a family of valid
    answers.  An equal-front-rear, equal-left-right layout has a symmetric
    minimum norm split; a solver that returned any other balanced solution (or
    an unconstrained least-squares fit) would fail this.  This pins the
    algorithmic choice that 2026-09-22 decision A2 keeps in Python.
    """
    result = compute_static_wheel_loads(full_vehicle_model)

    assert result.rank == 3
    loads = result.wheel_loads
    quarter = result.summary.total / 4.0
    for name, value in loads.items():
        assert np.isclose(value, quarter, rtol=1e-9, atol=1e-8), name
    # The minimum-norm member of the balanced family is the uniform split, so
    # equality with the quarter load is the algorithm's fingerprint: a solver
    # returning any other balanced solution would satisfy the balance checks
    # above but fail here.
