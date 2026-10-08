from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.adams.full_vehicle_correlation import (
    full_vehicle_time_history,
)


class _NativeRun:
    times_s = np.asarray((0.0, 0.01), dtype=float)

    def __init__(
        self,
        forward_velocity: float,
        *,
        channel: str = "front_rack",
        steering_target: float = 0.0,
    ) -> None:
        self._chassis = np.zeros((2, 19), dtype=float)
        self._chassis[:, 3] = 1.0
        self._chassis[:, 7] = forward_velocity
        self._chassis[:, 14] = 2.0
        self.steering_names = (channel,)
        self._steering = np.zeros((2, 4), dtype=float)
        self._steering[:, 2] = steering_target

    def body_state(self, name: str) -> np.ndarray:
        if name != "body.chassis":
            raise KeyError(name)
        return self._chassis

    def element_state(self, name: str) -> np.ndarray:
        if name != self.steering_names[0]:
            raise KeyError(name)
        return self._steering


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
        chassis_body_id="body.chassis",
        steering_ratio_m_per_rad=1.0,
        steering_channel="front_rack",
    )

    assert history.channels["lateral_acceleration"] == pytest.approx(
        (expected_lateral_acceleration, expected_lateral_acceleration)
    )


def test_native_handling_scales_the_rack_displacement_by_the_ratio() -> None:
    """A rack translation is read back through the declared ratio."""
    history = full_vehicle_time_history(
        _NativeRun(10.0, steering_target=0.0552),
        "handling_stability",
        chassis_body_id="body.chassis",
        steering_ratio_m_per_rad=0.0276,
        steering_channel="front_rack",
        steering_actuator_mode="prescribed_translation",
    )

    assert history.channels["steering_angle"] == pytest.approx((2.0, 2.0), rel=1e-12)
    assert history.units["steering_angle"] == "rad"


def test_native_handling_reads_a_prescribed_rotation_as_an_angle() -> None:
    """
    The declared mode, not the channel name, decides the conversion.

    Since p2-06 a prescribed rotation is addressed by its declared
    ``channel_name`` -- the same name the rack path uses -- so a name lookup can
    no longer tell the two apart.  Reading this angle through the rack ratio
    would divide it by 0.0276 and understate the steering input by ~36x.
    """
    history = full_vehicle_time_history(
        _NativeRun(10.0, steering_target=0.0552),
        "handling_stability",
        chassis_body_id="body.chassis",
        steering_ratio_m_per_rad=0.0276,
        steering_channel="front_rack",
        steering_actuator_mode="prescribed_rotation",
    )

    assert history.channels["steering_angle"] == pytest.approx(
        (0.0552, 0.0552), rel=1e-12
    )
    assert history.units["steering_angle"] == "rad"


def test_native_handling_uses_the_declared_channel_name() -> None:
    """A renamed channel is addressed by that name, not by a literal."""
    history = full_vehicle_time_history(
        _NativeRun(10.0, channel="rear_rack", steering_target=0.005),
        "handling_stability",
        chassis_body_id="body.chassis",
        steering_ratio_m_per_rad=0.5,
        steering_channel="rear_rack",
        steering_actuator_mode="prescribed_translation",
    )

    assert history.channels["steering_angle"] == pytest.approx((0.01, 0.01), rel=1e-12)


def test_native_handling_rejects_an_undeclared_channel_name() -> None:
    """A run that does not report the declared channel is refused, not guessed."""
    with pytest.raises(ValueError, match="no steering channel 'steering_input'"):
        full_vehicle_time_history(
            _NativeRun(10.0, channel="front_rack"),
            "handling_stability",
        chassis_body_id="body.chassis",
            steering_ratio_m_per_rad=1.0,
            steering_channel="steering_input",
        )


def test_native_handling_rejects_a_missing_channel_name() -> None:
    with pytest.raises(ValueError, match="requires the declared steering channel"):
        full_vehicle_time_history(
            _NativeRun(10.0),
            "handling_stability",
        chassis_body_id="body.chassis",
            steering_ratio_m_per_rad=1.0,
        )


def test_native_handling_rejects_a_non_positive_ratio_for_a_rack_translation() -> None:
    with pytest.raises(ValueError, match="positive steering ratio"):
        full_vehicle_time_history(
            _NativeRun(10.0, steering_target=0.005),
            "handling_stability",
        chassis_body_id="body.chassis",
            steering_ratio_m_per_rad=0.0,
            steering_channel="front_rack",
            steering_actuator_mode="prescribed_translation",
        )
