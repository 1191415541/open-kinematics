"""
The vehicle parameter block the Adams input manifest records.

This is a *data shape*, not a model: it is the parameter set an Adams handling
or ride run was built from, and it is written into the immutable input manifest
so the run can be described later.  The reader on the other side
(`full_vehicle_model._parse_reference_mass`) takes the mass out of that manifest.

It used to live beside an independent 14/15-DOF Python vehicle model that the
Adams correlation gates were checked against.  That model -- and the gates that
consumed it -- are gone; the parameter block stayed because the manifest is
still written, and the field names are part of a frozen artifact.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Vehicle14DofParameters:
    """SI parameters for 6 body, 4 wheel-vertical, and 4 wheel-spin coordinates."""

    mass: float = 1_527.680888
    sprung_mass: float = 1_295.680888
    unsprung_mass: float = 58.0
    inertia_roll: float = 298.8515286
    inertia_pitch: float = 1_162.028448
    inertia_yaw: float = 1_340.386004
    wheelbase: float = 2.56
    front_axle_to_cg: float = 1.481397767
    rear_axle_to_cg: float = 1.078602233
    front_track: float = 1.52
    rear_track: float = 1.594
    center_of_mass_height: float = 0.55
    suspension_stiffness: float = 30_000.0
    suspension_damping: float = 3_000.0
    tire_vertical_stiffness: float = 180_000.0
    roll_stiffness: float = 55_000.0
    roll_damping: float = 5_500.0
    pitch_stiffness: float = 70_000.0
    pitch_damping: float = 6_500.0
    steering_ratio: float = 27.6
    steering_time_constant: float = 0.01
    wheel_radius: float = 0.31
    front_cornering_stiffness: float = 70_000.0
    rear_cornering_stiffness: float = 100_000.0
    tire_friction_coefficient: float = 0.9

    def __post_init__(self) -> None:
        if min(
            self.mass,
            self.sprung_mass,
            self.unsprung_mass,
            self.inertia_roll,
            self.inertia_pitch,
            self.inertia_yaw,
            self.wheelbase,
            self.front_axle_to_cg,
            self.rear_axle_to_cg,
            self.front_track,
            self.rear_track,
            self.suspension_stiffness,
            self.tire_vertical_stiffness,
            self.wheel_radius,
            self.front_cornering_stiffness,
            self.rear_cornering_stiffness,
            self.tire_friction_coefficient,
        ) <= 0.0:
            raise ValueError("vehicle correlation parameters must be positive")
        if self.sprung_mass + 4.0 * self.unsprung_mass > self.mass:
            raise ValueError("sprung and unsprung masses exceed vehicle mass")
        if not math.isclose(
            self.front_axle_to_cg + self.rear_axle_to_cg,
            self.wheelbase,
            abs_tol=1e-9,
        ):
            raise ValueError("front and rear axle distances must equal wheelbase")
