"""
The steering allocator: one driver input, several correctly angled channels.

Subtask p2-06 adds distributed steering channels, and with more than one channel
there is a question the vehicle has never had to answer: the driver commands one
thing and the solver drives several.  This module answers it with arithmetic, so
the tests here are numerical rather than "it ran": every law's formula is written
out again in the test and the allocator's output is compared with it, because a
law that is only checked against itself is not checked at all.

The four laws are the ones the roadmap names: `direct` (the historical
behaviour), Ackermann, four-wheel steer with a speed-dependent rear gain, and
multi-axle follow.
"""

from __future__ import annotations

import math

import pytest

from suspension_multibody.authoring.steering_allocator import (
    FOUR_WHEEL_STEER_HIGH_SPEED_GAIN,
    FOUR_WHEEL_STEER_LOW_SPEED_GAIN,
    AllocationChannel,
    allocate,
)

#: A front axle of an ordinary car: 2.8 m wheelbase, 1.6 m track.
FRONT = AllocationChannel(
    name="front_rack",
    placement="front",
    wheelbase_mm=2800.0,
    track_mm=1600.0,
    distance_to_reference_mm=0.0,
)
#: The same car's rear axle, 2.8 m behind the front one.
REAR = AllocationChannel(
    name="rear_rack",
    placement="rear",
    wheelbase_mm=2800.0,
    track_mm=1600.0,
    distance_to_reference_mm=2800.0,
)


def test_direct_passes_the_input_through_on_one_channel() -> None:
    """The historical case: one channel takes the driver's angle unchanged."""
    result = allocate("direct", [FRONT], steer_input_rad=0.2, speed_mps=0.0)

    assert result.names == ("front_rack",)
    assert result.angles_rad == (0.2,)
    assert result.law == "direct"
    assert result.steer_input_rad == 0.2


def test_direct_passes_the_input_through_on_every_channel() -> None:
    """Two channels under `direct` are both driven by the same signal."""
    result = allocate("direct", [FRONT, REAR], steer_input_rad=-0.17, speed_mps=12.0)

    assert result.names == ("front_rack", "rear_rack")
    assert result.angles_rad == (-0.17, -0.17)


def test_ackermann_splits_the_reference_angle_about_the_turn_centre() -> None:
    """
    The inner wheel turns further than the reference; the outer one less.

    The expected values are the formula written a second time, independently of
    the allocator: with `L = 2800 mm`, `t = 1600 mm` and `dc = 0.2 rad`, the turn
    radius at the rear axle is `R = L / tan(dc)`, and the two ends of the track
    sit half a track either side of it.
    """
    inner = AllocationChannel(
        name="front_left",
        placement="front_left",
        wheelbase_mm=2800.0,
        track_mm=1600.0,
        distance_to_reference_mm=-800.0,
    )
    outer = AllocationChannel(
        name="front_right",
        placement="front_right",
        wheelbase_mm=2800.0,
        track_mm=1600.0,
        distance_to_reference_mm=800.0,
    )
    dc = 0.2
    length, track = 2800.0, 1600.0
    radius = length / math.tan(abs(dc))
    expected_inner = math.atan(length / (radius - track / 2.0))
    expected_outer = math.atan(length / (radius + track / 2.0))

    result = allocate(
        "ackermann", [FRONT, inner, outer], steer_input_rad=dc, speed_mps=0.0
    )

    assert result.angles_rad[0] == dc
    assert result.angles_rad[1] == pytest.approx(expected_inner, rel=1e-12)
    assert result.angles_rad[2] == pytest.approx(expected_outer, rel=1e-12)
    # And the ordering the geometry requires, stated on its own so a sign or
    # track-width slip cannot hide behind the formula agreeing with itself.
    assert abs(result.angles_rad[1]) > abs(result.angles_rad[0]) > abs(result.angles_rad[2])


def test_ackermann_at_zero_input_is_zero_rather_than_a_division() -> None:
    """Straight ahead every end of the axle points the same way: zero."""
    result = allocate(
        "ackermann", [FRONT, REAR], steer_input_rad=1e-15, speed_mps=0.0
    )

    assert result.angles_rad == (0.0, 0.0)


def test_ackermann_rejects_a_geometrically_unreachable_pair() -> None:
    """A track wider than twice the turn radius puts the inner wheel past the centre."""
    with pytest.raises(ValueError, match="geometrically unreachable"):
        allocate("ackermann", [FRONT, REAR], steer_input_rad=1.4, speed_mps=0.0)


def test_ackermann_with_one_channel_returns_the_reference_angle() -> None:
    """One channel is the whole axle the driver asked for, not half of it."""
    result = allocate("ackermann", [FRONT], steer_input_rad=0.2, speed_mps=0.0)

    assert result.angles_rad == (0.2,)


def test_four_wheel_steer_turns_the_rear_wheels_against_the_front_at_low_speed() -> None:
    """
    Below the crossover the rear gain is negative: a smaller turning circle.

    The default gain is stated in the module so the number under test is the one
    a caller gets without passing anything.
    """
    result = allocate(
        "four_wheel_steer", [FRONT, REAR], steer_input_rad=0.1, speed_mps=2.0
    )

    assert FOUR_WHEEL_STEER_LOW_SPEED_GAIN == -0.25
    assert result.angles_rad[0] == 0.1
    assert result.angles_rad[1] == pytest.approx(-0.25 * 0.1, rel=1e-12)
    assert math.copysign(1.0, result.angles_rad[1]) != math.copysign(
        1.0, result.angles_rad[0]
    )


def test_four_wheel_steer_turns_the_rear_wheels_with_the_front_at_high_speed() -> None:
    """At and above the crossover the rear gain is positive: stability."""
    result = allocate(
        "four_wheel_steer", [FRONT, REAR], steer_input_rad=0.1, speed_mps=20.0
    )

    assert FOUR_WHEEL_STEER_HIGH_SPEED_GAIN == 0.15
    assert result.angles_rad[0] == 0.1
    assert result.angles_rad[1] == pytest.approx(0.15 * 0.1, rel=1e-12)
    assert math.copysign(1.0, result.angles_rad[1]) == math.copysign(
        1.0, result.angles_rad[0]
    )


def test_the_four_wheel_steer_crossover_is_the_caller_s_threshold() -> None:
    """The threshold is a declaration: the same speed can be either side of it."""
    low = allocate(
        "four_wheel_steer",
        [FRONT, REAR],
        steer_input_rad=0.1,
        speed_mps=4.9,
        low_speed_threshold_mps=5.0,
    )
    high = allocate(
        "four_wheel_steer",
        [FRONT, REAR],
        steer_input_rad=0.1,
        speed_mps=5.0,
        low_speed_threshold_mps=5.0,
    )

    assert low.angles_rad[1] == pytest.approx(-0.025, rel=1e-12)
    assert high.angles_rad[1] == pytest.approx(0.015, rel=1e-12)


def test_four_wheel_steer_refuses_two_gains_of_the_same_sign() -> None:
    """A schedule whose two ends agree is not a schedule, it is a pair of typos."""
    with pytest.raises(ValueError, match="rear_gain_low < 0 < rear_gain_high"):
        allocate(
            "four_wheel_steer",
            [FRONT, REAR],
            steer_input_rad=0.1,
            speed_mps=2.0,
            rear_gain_low=0.2,
            rear_gain_high=0.15,
        )


def test_multi_axle_follow_places_every_axle_on_the_reference_circle() -> None:
    """
    A three-axle train: axle two and axle three trace the first axle's circle.

    The expected values are the formula written a second time --
    `atan((Li / Lref) * tan(dref))` -- with the distances the caller states.
    """
    second = AllocationChannel(
        name="middle_rack",
        placement="middle",
        wheelbase_mm=4200.0,
        track_mm=1800.0,
        distance_to_reference_mm=4200.0,
    )
    third = AllocationChannel(
        name="third_rack",
        placement="third",
        wheelbase_mm=4200.0,
        track_mm=1800.0,
        distance_to_reference_mm=5600.0,
    )
    reference = AllocationChannel(
        name="front_rack",
        placement="front",
        wheelbase_mm=2800.0,
        track_mm=1800.0,
        distance_to_reference_mm=0.0,
    )
    dref = 0.1

    result = allocate(
        "multi_axle_follow", [reference, second, third], steer_input_rad=dref, speed_mps=0.0
    )

    assert result.names == ("front_rack", "middle_rack", "third_rack")
    assert result.angles_rad[0] == dref
    assert result.angles_rad[1] == pytest.approx(
        math.atan(4200.0 / 2800.0 * math.tan(dref)), rel=1e-12
    )
    assert result.angles_rad[2] == pytest.approx(
        math.atan(5600.0 / 2800.0 * math.tan(dref)), rel=1e-12
    )


def test_multi_axle_follow_refuses_an_axle_that_is_not_behind_the_reference() -> None:
    """A distance of zero names no following axle, so it is refused by name."""
    with pytest.raises(ValueError, match="middle_rack"):
        allocate(
            "multi_axle_follow",
            [
                FRONT,
                AllocationChannel("middle_rack", "middle", 4200.0, 1800.0, 0.0),
            ],
            steer_input_rad=0.1,
            speed_mps=0.0,
        )


def test_multi_axle_follow_refuses_a_negative_distance_by_name() -> None:
    """The same refusal for a distance behind the reference axle."""
    with pytest.raises(ValueError, match="middle_rack"):
        allocate(
            "multi_axle_follow",
            [
                FRONT,
                AllocationChannel("middle_rack", "middle", 4200.0, 1800.0, -100.0),
            ],
            steer_input_rad=0.1,
            speed_mps=0.0,
        )


def test_no_channels_allocate_nothing() -> None:
    """A vehicle that steers nothing gets an empty, complete answer."""
    result = allocate("ackermann", [], steer_input_rad=0.3, speed_mps=7.0)

    assert result.names == ()
    assert result.angles_rad == ()
    assert result.law == "ackermann"
    assert result.speed_mps == 7.0
    assert result.steer_input_rad == 0.3


def test_an_unknown_law_is_refused_by_name() -> None:
    """An unstated law would be a vehicle that silently steers by some third rule."""
    with pytest.raises(ValueError, match="crab_steer"):
        allocate("crab_steer", [FRONT, REAR], steer_input_rad=0.1, speed_mps=1.0)


@pytest.mark.parametrize("law", ["direct", "ackermann", "four_wheel_steer", "multi_axle_follow"])
def test_a_non_finite_input_is_refused(law: str) -> None:
    """A target the solver cannot integrate is refused at the boundary."""
    for bad in (math.inf, -math.inf, math.nan):
        with pytest.raises(ValueError, match="steer_input_rad must be finite"):
            allocate(law, [FRONT, REAR], steer_input_rad=bad, speed_mps=1.0)


@pytest.mark.parametrize("law", ["direct", "ackermann", "four_wheel_steer", "multi_axle_follow"])
def test_a_non_finite_speed_is_refused(law: str) -> None:
    """The speed selects a gain, so it has to be a number."""
    with pytest.raises(ValueError, match="speed_mps must be finite"):
        allocate(law, [FRONT, REAR], steer_input_rad=0.1, speed_mps=math.nan)


def test_a_non_finite_channel_geometry_is_refused() -> None:
    """A channel whose wheelbase is not a number cannot be placed on a circle."""
    with pytest.raises(ValueError, match="front_rack"):
        allocate(
            "direct",
            [
                AllocationChannel(
                    "front_rack", "front", math.inf, 1600.0, 0.0
                )
            ],
            steer_input_rad=0.1,
            speed_mps=1.0,
        )
