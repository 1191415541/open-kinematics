from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.adams.full_vehicle_correlation import (
    full_vehicle_time_history,
)


class _NativeRun:
    times_s = np.asarray((0.0, 0.01), dtype=float)

    def __init__(self, forward_velocity: float) -> None:
        self._chassis = np.zeros((2, 19), dtype=float)
        self._chassis[:, 3] = 1.0
        self._chassis[:, 7] = forward_velocity
        self._chassis[:, 14] = 2.0

    def body_state(self, name: str) -> np.ndarray:
        if name != "chassis":
            raise KeyError(name)
        return self._chassis

    def steering_state(self, name: str) -> np.ndarray:
        if name != "front_rack":
            raise KeyError(name)
        return np.zeros((2, 4), dtype=float)


@pytest.mark.parametrize(
    ("forward_velocity", "expected_lateral_acceleration"),
    ((10.0, 2_000.0), (-10.0, -2_000.0)),
)
def test_native_handling_lateral_axis_follows_vehicle_forward_direction(
    forward_velocity: float, expected_lateral_acceleration: float
) -> None:
    history = full_vehicle_time_history(
        _NativeRun(forward_velocity),
        "handling_stability",
        steering_ratio_m_per_rad=1.0,
    )

    assert history.channels["lateral_acceleration"] == pytest.approx(
        (expected_lateral_acceleration, expected_lateral_acceleration)
    )
