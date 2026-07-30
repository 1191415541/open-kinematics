"""Vehicle-level geometry metrics derived from one suspension corner.

The suspension solver models one corner.  Metrics that describe an axle use a
mirrored opposite-side assumption, matching the GUI hardpoint model.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from suspension_kinematics.core.constants import EPS_GEOMETRIC
from suspension_kinematics.core.enums import Axis

if TYPE_CHECKING:
    from suspension_kinematics.metrics.context import MetricContext


def calculate_roll_center_height(ctx: MetricContext) -> float | None:
    """Return roll-center height above the contact-patch ground plane in mm."""
    roll_center = ctx.front_view_roll_center
    if roll_center is None:
        return None
    return float(roll_center[Axis.Z] - ctx.ground_z)


def calculate_roll_center_lateral_offset(ctx: MetricContext) -> float | None:
    """Return lateral roll-center offset from the vehicle centerline in mm.

    A single-corner suspension is mirrored across Y = 0, so this result is
    zero for the supported symmetric layouts.  It remains an explicit output
    so a future full-axle solver can preserve the same result schema.
    """
    roll_center = ctx.front_view_roll_center
    if roll_center is None:
        return None
    return float(roll_center[Axis.Y])


def calculate_anti_pitch_pct(ctx: MetricContext) -> float | None:
    """Return the signed anti-pitch percentage for the modeled axle.

    The geometric term assumes the axle carries 100% of longitudinal tire force,
    then scales by the axle brake-force share:

        anti-pitch = -sign(CP_X - CG_X) * ((SVIC_Z - CP_Z) /
                      (SVIC_X - CP_X)) * wheelbase / CG_height
                      * brake_force_share * 100

    ``brake_force_share`` is ``brake_bias_front`` for the front axle and
    ``1 - brake_bias_front`` for the rear axle.  Positive values resist
    longitudinal load transfer.  The sign term derives front/rear axle
    orientation from the contact patch relative to the CG.
    """
    side_view_ic = ctx.side_view_ic
    if side_view_ic is None or ctx.wheelbase <= EPS_GEOMETRIC:
        return None

    contact_patch = ctx.contact_patch_center
    cg_position = ctx.cg_position
    longitudinal_span = float(side_view_ic[Axis.X] - contact_patch[Axis.X])
    cg_height = float(cg_position[Axis.Z] - contact_patch[Axis.Z])
    axle_relative_to_cg = float(contact_patch[Axis.X] - cg_position[Axis.X])
    if (
        abs(longitudinal_span) <= EPS_GEOMETRIC
        or abs(cg_height) <= EPS_GEOMETRIC
        or abs(axle_relative_to_cg) <= EPS_GEOMETRIC
    ):
        return None

    # Front axle sits ahead of the CG in vehicle coordinates used here:
    # CP_X - CG_X < 0 (see geometry fixtures with front corner near X=0).
    is_front_axle = axle_relative_to_cg < 0.0
    brake_force_share = (
        ctx.brake_bias_front if is_front_axle else (1.0 - ctx.brake_bias_front)
    )

    side_view_slope = float(
        (side_view_ic[Axis.Z] - contact_patch[Axis.Z]) / longitudinal_span
    )
    return float(
        -np.sign(axle_relative_to_cg)
        * side_view_slope
        * ctx.wheelbase
        / cg_height
        * brake_force_share
        * 100.0
    )


def calculate_track_change(ctx: MetricContext) -> float:
    """Return mirrored axle track-width change from the design state in mm."""
    current_track = 2.0 * abs(float(ctx.wheel_center[Axis.Y]))
    design_track = 2.0 * abs(float(ctx.design_wheel_center[Axis.Y]))
    return float(current_track - design_track)
