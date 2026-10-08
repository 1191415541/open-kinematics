"""
Turn one driver input into the per-channel steering angles (subtask p2-06).

A vehicle with more than one steering rack has one thing the driver actually
commands -- the steering wheel, or the rack it turns -- and several things the
solver actually drives.  This module is the one place that converts between
them, and it does so as pure arithmetic on numbers the caller has already
resolved: an angle, a speed, and per-channel geometry.

Two boundaries are deliberate:

* it computes *angles*, not rack travels and not forces.  A channel's own ratio
  turns the angle it is handed into the displacement its rack is prescribed, so
  the conversion stays with the channel that declares the ratio;
* it reads nothing off a model, an assembly or a document.  The geometry it
  needs -- wheelbase, track, how far an axle sits from the reference -- is what
  the caller passes in, because a module that went looking for it would be a
  second opinion about which axle a channel steers.

Four laws are implemented, and each is stated as a formula rather than as a
table:

``direct``
    every channel takes the input unchanged.  This is the historical behaviour:
    the case's own steering signal drives each channel, so a one-channel
    vehicle is exactly what it was before channels existed.

``ackermann``
    the reference channel takes the input and the two steered ends of its axle
    split it about the turn centre: with ``L`` the wheelbase, ``t`` the track
    and ``R = L / tan(|δc|)`` the turn radius measured at the rear axle,
    ``|δin| = atan(L / (R - t/2))`` and ``|δout| = atan(L / (R + t/2))``, both
    carrying the sign of ``δc``.  With one channel declared the reference *is*
    the whole axle, so it takes the input exactly.

``four_wheel_steer``
    the reference channel takes the input and every other channel follows it
    with one gain: ``δr = gain * δf``.  The gain is chosen by speed -- the rear
    wheels turn against the front at low speed (a smaller turning circle) and
    with them at high speed (stability) -- and the crossover is the caller's
    ``low_speed_threshold_mps``.

``multi_axle_follow``
    each following axle traces the circle the reference axle turned onto:
    ``δi = atan((Li / Lref) * tan(δref))``, with ``Li`` the distance from that
    axle to the reference one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal, Sequence

__all__ = [
    "FOUR_WHEEL_STEER_HIGH_SPEED_GAIN",
    "FOUR_WHEEL_STEER_LOW_SPEED_GAIN",
    "AllocationChannel",
    "AllocationLaw",
    "SteeringAllocation",
    "allocate",
]

#: The rear-to-front gain used below :data:`DEFAULT_LOW_SPEED_THRESHOLD_MPS`.
#:
#: Negative on purpose: below the crossover the rear wheels turn *against* the
#: front ones, which is what shortens the turning circle.
FOUR_WHEEL_STEER_LOW_SPEED_GAIN = -0.25
#: The rear-to-front gain used at and above the crossover.
#:
#: Positive: the rear wheels turn *with* the front ones, which is the
#: high-speed part of a four-wheel-steer schedule.
FOUR_WHEEL_STEER_HIGH_SPEED_GAIN = 0.15
#: Where a four-wheel-steer schedule switches between its two gains, in m/s.
#:
#: A default rather than a physical law: which speed a given car crosses over at
#: is a tuning decision, so a caller states its own.
DEFAULT_LOW_SPEED_THRESHOLD_MPS = 5.0

#: The gains are applied below and at the threshold respectively, so a negative
#: low-speed gain is the declaration "the rear wheels oppose the front ones in a
#: parking manoeuvre" and cannot be confused with a sign error.
AllocationLaw = Literal["direct", "ackermann", "four_wheel_steer", "multi_axle_follow"]


@dataclass(frozen=True)
class AllocationChannel:
    """
    One steering channel, reduced to the geometry an allocation law needs.

    ``distance_to_reference_mm`` is the axle's distance to the *reference*
    channel's axle and is stated by the caller: a module that derived it from a
    wheelbase would be guessing which axles a truck has.
    """

    name: str
    placement: str
    wheelbase_mm: float
    track_mm: float
    distance_to_reference_mm: float


@dataclass(frozen=True)
class SteeringAllocation:
    """The angles one driver input allocates, channel by channel."""

    #: The channels, in the order they were handed in.
    names: tuple[str, ...]
    #: The angle each channel's rack is to be steered by, in radians.
    angles_rad: tuple[float, ...]
    #: The law that produced them.
    law: str
    #: The speed the law was evaluated at, in m/s.
    speed_mps: float
    #: The reference input the angles were derived from, in radians.
    steer_input_rad: float


def allocate(
    law: str,
    channels: Sequence[AllocationChannel],
    *,
    steer_input_rad: float,
    speed_mps: float,
    low_speed_threshold_mps: float = DEFAULT_LOW_SPEED_THRESHOLD_MPS,
    rear_gain_low: float = FOUR_WHEEL_STEER_LOW_SPEED_GAIN,
    rear_gain_high: float = FOUR_WHEEL_STEER_HIGH_SPEED_GAIN,
) -> SteeringAllocation:
    """
    Return the per-channel angles one driver input allocates.

    ``channels[0]`` is the reference channel for every law that has one: it is
    the channel the driver's input lands on.  A caller that wants a different
    reference orders its channels accordingly rather than naming an index here,
    because "which channel the driver turns" is a property of the vehicle, not
    of this function.

    The laws are pure functions of their arguments: same inputs, same numbers,
    on every platform and in every order of calls.
    """
    if not math.isfinite(steer_input_rad):
        raise ValueError("steer_input_rad must be finite")
    if not math.isfinite(speed_mps):
        raise ValueError("speed_mps must be finite")
    if not math.isfinite(low_speed_threshold_mps):
        raise ValueError("low_speed_threshold_mps must be finite")
    if not math.isfinite(rear_gain_low) or not math.isfinite(rear_gain_high):
        raise ValueError("the four-wheel-steer gains must be finite")
    if channels:
        for channel in channels:
            for label, value in (
                ("wheelbase_mm", channel.wheelbase_mm),
                ("track_mm", channel.track_mm),
                ("distance_to_reference_mm", channel.distance_to_reference_mm),
            ):
                if not math.isfinite(value):
                    raise ValueError(
                        f"steering channel {channel.name!r} {label} must be finite"
                    )
    if not channels:
        # An empty vehicle steers nothing, and that is a complete answer rather
        # than a failure: the laws below all need a reference channel.
        return SteeringAllocation(
            names=(),
            angles_rad=(),
            law=str(law),
            speed_mps=float(speed_mps),
            steer_input_rad=float(steer_input_rad),
        )
    if law == "direct":
        angles = _direct(channels, steer_input_rad)
    elif law == "ackermann":
        angles = _ackermann(channels, steer_input_rad)
    elif law == "four_wheel_steer":
        angles = _four_wheel_steer(
            channels,
            steer_input_rad,
            speed_mps=speed_mps,
            low_speed_threshold_mps=low_speed_threshold_mps,
            rear_gain_low=rear_gain_low,
            rear_gain_high=rear_gain_high,
        )
    elif law == "multi_axle_follow":
        angles = _multi_axle_follow(channels, steer_input_rad)
    else:
        raise ValueError(
            f"unknown steering allocation law {law!r}; the laws are "
            "\"direct\", \"ackermann\", \"four_wheel_steer\" and \"multi_axle_follow\""
        )
    return SteeringAllocation(
        names=tuple(channel.name for channel in channels),
        angles_rad=angles,
        law=str(law),
        speed_mps=float(speed_mps),
        steer_input_rad=float(steer_input_rad),
    )


def _direct(
    channels: Sequence[AllocationChannel], steer_input_rad: float
) -> tuple[float, ...]:
    """Every channel takes the input unchanged."""
    return tuple(float(steer_input_rad) for _ in channels)


def _ackermann(
    channels: Sequence[AllocationChannel], steer_input_rad: float
) -> tuple[float, ...]:
    """
    Split the reference angle about the turn centre.

    ``channels[0]`` takes the input as stated -- it is the angle the driver
    asked for, and re-deriving it from its own split would move it -- and the
    remaining channels are placed on the same circle: the turn radius measured
    at the *rear* axle is ``R = L / tan(|δc|)``, the inner end of the track is
    half a track closer to the centre than that and the outer end half a track
    further, so their angles are ``atan(L / (R ∓ t/2))``.
    """
    reference = channels[0]
    if not math.isfinite(reference.wheelbase_mm) or reference.wheelbase_mm <= 0.0:
        raise ValueError(
            f"ackermann needs a positive wheelbase on {reference.name!r}; "
            f"found {reference.wheelbase_mm}"
        )
    magnitude = abs(float(steer_input_rad))
    if magnitude < 1e-12:
        # Straight ahead: the turn centre is at infinity and every angle on the
        # circle is zero.  Split here rather than let the division by tan(0)
        # decide, because "0/0" is not a steering angle.
        return tuple(0.0 for _ in channels)
    length = reference.wheelbase_mm
    half_track = reference.track_mm / 2.0
    radius = length / math.tan(magnitude)
    if radius - half_track <= 0.0:
        raise ValueError(
            f"ackermann is geometrically unreachable on {reference.name!r}: "
            f"wheelbase {length} and track {reference.track_mm} put the inner "
            f"wheel past the turn centre (radius {radius}, half track {half_track})"
        )
    sign = math.copysign(1.0, float(steer_input_rad))
    inner = math.atan(length / (radius - half_track))
    outer = math.atan(length / (radius + half_track))
    angles = [float(steer_input_rad)]
    for channel in channels[1:]:
        # Which end of the track a following channel steers is what its
        # distance to the reference *declares*: a negative distance puts it on
        # the inside of the turn, a positive one on the outside.  Zero declares
        # neither, so it is refused by name rather than resolved to a side.
        if channel.distance_to_reference_mm < 0.0:
            angles.append(sign * inner)
        elif channel.distance_to_reference_mm > 0.0:
            angles.append(sign * outer)
        else:
            raise ValueError(
                f"ackermann cannot place steering channel {channel.name!r}: a "
                "distance to the reference of 0 does not say which end of the "
                "track it steers"
            )
    return tuple(angles)


def _four_wheel_steer(
    channels: Sequence[AllocationChannel],
    steer_input_rad: float,
    *,
    speed_mps: float,
    low_speed_threshold_mps: float,
    rear_gain_low: float,
    rear_gain_high: float,
) -> tuple[float, ...]:
    """
    Drive the reference channel and let the others follow it with one gain.

    The gain is a schedule rather than a number: below the crossover the rear
    wheels oppose the front ones, at and above it they follow them.  The two
    signs are the contract, so a caller that hands in two gains of the same sign
    is refused by name instead of getting a car that steers wrong at one end.
    """
    if not rear_gain_low < 0.0 < rear_gain_high:
        raise ValueError(
            "four-wheel steer needs rear_gain_low < 0 < rear_gain_high "
            "(the rear wheels oppose the front ones at low speed and follow them "
            f"at high speed); found {rear_gain_low} and {rear_gain_high}"
        )
    gain = rear_gain_low if speed_mps < low_speed_threshold_mps else rear_gain_high
    front = float(steer_input_rad)
    return tuple(front if index == 0 else gain * front for index in range(len(channels)))


def _multi_axle_follow(
    channels: Sequence[AllocationChannel], steer_input_rad: float
) -> tuple[float, ...]:
    """
    Place every following axle on the circle the reference axle turned onto.

    ``δi = atan((Li / Lref) * tan(δref))`` with ``Li`` the axle's distance to
    the reference one.  Both lengths are the caller's, and both have to be
    positive: an axle at the reference or behind it is not a following axle, and
    a zero wheelbase would make the ratio meaningless rather than zero.
    """
    reference = channels[0]
    if not math.isfinite(reference.wheelbase_mm) or reference.wheelbase_mm <= 0.0:
        raise ValueError(
            f"multi axle follow needs a positive wheelbase on {reference.name!r}; "
            f"found {reference.wheelbase_mm}"
        )
    angles = [float(steer_input_rad)]
    tangent = math.tan(float(steer_input_rad))
    for channel in channels[1:]:
        if not math.isfinite(channel.distance_to_reference_mm) or (
            channel.distance_to_reference_mm <= 0.0
        ):
            raise ValueError(
                f"multi axle follow needs a positive distance to the reference "
                f"axle on {channel.name!r}; found {channel.distance_to_reference_mm}"
            )
        angles.append(
            math.atan(
                (channel.distance_to_reference_mm / reference.wheelbase_mm) * tangent
            )
        )
    return tuple(angles)
