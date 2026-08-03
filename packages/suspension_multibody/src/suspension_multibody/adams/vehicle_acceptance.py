"""Full-vehicle Adams acceptance matrix."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal

AcceptanceCategory = Literal["handling_stability", "ride"]


@dataclass(frozen=True)
class EngineeringTolerance:
    """Engineering-level time-history acceptance tolerance."""

    metric: str
    relative_percent: float | None = None
    absolute: float | None = None
    phase_ms: float | None = None
    magnitude_db: float | None = None
    phase_deg: float | None = None


@dataclass(frozen=True)
class VehicleAcceptanceCase:
    """One full-vehicle Adams acceptance case."""

    name: str
    category: AcceptanceCategory
    channels: tuple[str, ...]
    tolerances: tuple[EngineeringTolerance, ...]
    adams_template_source: str = "adams_builtin"
    pac2002_source: str = "adams_builtin"


HANDLING_CASES = (
    "steady_state_circle",
    "step_steer",
    "sine_steer",
    "double_lane_change",
)

RIDE_CASES = (
    "single_wheel_bump",
    "double_wheel_bump",
    "random_road",
    "four_post_rig",
)

HANDLING_CHANNELS = (
    "steering_angle",
    "lateral_acceleration",
    "yaw_rate",
    "body_roll",
)

RIDE_CHANNELS = (
    "body_accel_z",
    "body_heave",
    "body_pitch",
    "body_roll",
)


def default_vehicle_acceptance_matrix() -> tuple[VehicleAcceptanceCase, ...]:
    """Return the frozen first-pass full-vehicle Adams acceptance matrix."""
    handling_tolerances = (
        EngineeringTolerance("steady_scalar", relative_percent=5.0),
        EngineeringTolerance("transient_peak_rms", relative_percent=10.0, phase_ms=20.0),
    )
    ride_tolerances = (
        EngineeringTolerance("displacement_or_travel", relative_percent=10.0, absolute=3.0),
        EngineeringTolerance("acceleration_rms", relative_percent=10.0),
        EngineeringTolerance("acceleration_peak", relative_percent=15.0),
        EngineeringTolerance("tire_load_variation", relative_percent=15.0),
        EngineeringTolerance("four_post_frequency_response", magnitude_db=2.0, phase_deg=10.0),
    )
    return tuple(
        VehicleAcceptanceCase(
            name=name,
            category="handling_stability",
            channels=HANDLING_CHANNELS,
            tolerances=handling_tolerances,
        )
        for name in HANDLING_CASES
    ) + tuple(
        VehicleAcceptanceCase(
            name=name,
            category="ride",
            channels=RIDE_CHANNELS,
            tolerances=ride_tolerances,
        )
        for name in RIDE_CASES
    )


def validate_vehicle_acceptance_matrix(
    cases: tuple[VehicleAcceptanceCase, ...],
) -> None:
    """Validate the required Adams acceptance split and reference sources."""
    by_category = {case.category for case in cases}
    if by_category != {"handling_stability", "ride"}:
        raise ValueError("vehicle Adams acceptance must include handling and ride")
    names = {case.name for case in cases}
    missing = (set(HANDLING_CASES) | set(RIDE_CASES)) - names
    if missing:
        raise ValueError(f"missing vehicle Adams acceptance cases: {sorted(missing)}")
    for case in cases:
        if case.adams_template_source != "adams_builtin":
            raise ValueError("full-vehicle acceptance must use Adams built-in templates")
        if case.pac2002_source != "adams_builtin":
            raise ValueError("full-vehicle acceptance must use Adams built-in PAC2002")
