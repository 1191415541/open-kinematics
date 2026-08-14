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
