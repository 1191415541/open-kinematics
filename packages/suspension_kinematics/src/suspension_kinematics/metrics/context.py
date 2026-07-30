"""
Metric computation context.

Provides a single per-state object that resolves and caches shared geometry
needed by multiple metric functions (wheel axis, contact patch, ICs, etc.).
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cached_property
from typing import TYPE_CHECKING

import numpy as np

from suspension_kinematics.core.constants import EPS_GEOMETRIC
from suspension_kinematics.core.enums import Axis, PointID
from suspension_kinematics.core.types import Vec3
from suspension_kinematics.core.vector_utils.generic import normalize_vector
from suspension_kinematics.state import SuspensionState
from suspension_kinematics.suspensions.config.settings import SuspensionConfig

if TYPE_CHECKING:
    from suspension_kinematics.suspensions.base import Suspension


@dataclass
class MetricContext:
    """
    Shared context for computing metrics on a single solved state.

    Caches expensive geometry (ICs, wheel axis, etc.) so that multiple
    metric functions can share the same intermediate results.
    """

    state: SuspensionState
    suspension: "Suspension"
    config: SuspensionConfig

    @cached_property
    def side_view_ic(self) -> Vec3 | None:
        """
        Side-view instant center from the suspension.
        """
        return self.suspension.compute_side_view_instant_center(self.state)

    @cached_property
    def front_view_ic(self) -> Vec3 | None:
        """
        Front-view instant center from the suspension.
        """
        return self.suspension.compute_front_view_instant_center(self.state)

    @cached_property
    def front_view_roll_center(self) -> Vec3 | None:
        """Return the front-view roll center on the vehicle centerline.

        The roll center is the intersection of the line from this corner's
        contact patch to its front-view instant center with Y = 0.  The
        single-corner model assumes a mirrored opposite side, so this is the
        axle roll center for a symmetric layout.
        """
        front_view_ic = self.front_view_ic
        if front_view_ic is None:
            return None
        contact_patch = self.contact_patch_center
        delta_y = float(front_view_ic[Axis.Y] - contact_patch[Axis.Y])
        if abs(delta_y) <= EPS_GEOMETRIC:
            return None
        centerline_fraction = -float(contact_patch[Axis.Y]) / delta_y
        return contact_patch + centerline_fraction * (front_view_ic - contact_patch)

    @cached_property
    def wheel_center(self) -> Vec3:
        """
        Wheel center position.
        """
        return self.state.get(PointID.WHEEL_CENTER)

    @cached_property
    def design_wheel_center(self) -> Vec3:
        """Wheel-center position at the suspension design condition."""
        return self.suspension.initial_state().get(PointID.WHEEL_CENTER)

    @cached_property
    def contact_patch_center(self) -> Vec3:
        """
        Contact patch center position.
        """
        return self.state.get(PointID.CONTACT_PATCH_CENTER)

    @cached_property
    def wheel_axis(self) -> Vec3:
        """
        Unit vector along the axle from inboard to outboard.
        """
        axle_in = self.state.get(PointID.AXLE_INBOARD)
        axle_out = self.state.get(PointID.AXLE_OUTBOARD)
        return normalize_vector(axle_out - axle_in)

    @cached_property
    def steering_axis(self) -> Vec3:
        """
        Unit vector along the steering axis from lower to upper pivot.
        """
        lower, upper = self.suspension.steering_axis_points(self.state)
        return normalize_vector(upper - lower)

    @cached_property
    def ground_z(self) -> float:
        """
        Ground plane Z-height in the chassis-fixed frame.

        In a chassis-fixed reference frame the ground is not at Z=0; it
        follows the tire. We define ground level as the contact patch
        centre Z so that all ground-plane intersections (steering axis,
        instant centres, etc.) are evaluated at the actual tire-road
        interface.
        """
        return float(self.contact_patch_center[Axis.Z])

    @cached_property
    def steering_axis_ground_intersection(self) -> Vec3 | None:
        """
        Point where the steering axis intersects the ground plane.

        Parameterises the line from the lower ball joint through the upper
        ball joint and solves for the parameter t where Z = ground_z.
        Returns None if the steering axis is parallel to the ground plane.
        """
        lower, upper = self.suspension.steering_axis_points(self.state)
        direction = upper - lower
        dz = direction[Axis.Z]
        if abs(dz) < EPS_GEOMETRIC:
            return None
        # t such that lower + t * direction has Z = ground_z
        t = (self.ground_z - lower[Axis.Z]) / dz
        return lower + t * direction

    @cached_property
    def side_sign(self) -> float:
        """
        Vehicle side indicator: 1.0 for left (Y > 0), -1.0 for right.
        """
        y_pos = self.state.get(PointID.AXLE_OUTBOARD)[Axis.Y]
        return -1.0 if y_pos < 0 else 1.0

    @cached_property
    def tire_radius(self) -> float:
        """
        Nominal tire radius from configuration.
        """
        return self.config.wheel.tire.nominal_radius

    @cached_property
    def wheelbase(self) -> float:
        """
        Vehicle wheelbase from configuration.
        """
        return self.config.wheelbase

    @cached_property
    def brake_bias_front(self) -> float:
        """Front-axle brake force fraction from configuration."""
        return float(self.config.brake_bias_front)

    @cached_property
    def cg_position(self) -> Vec3:
        """
        Center of gravity position from configuration.
        """
        return np.asarray(self.config.cg_position, dtype=float)
