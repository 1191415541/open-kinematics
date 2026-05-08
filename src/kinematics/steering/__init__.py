"""
Pure 2D steering linkage solvers.

The steering package is intentionally separate from the 3D suspension templates.
It models top-view linkage kinematics for systems where only roadwheel angle
relationships are required.
"""

from kinematics.steering.csv_loader import load_two_segment_steering_hardpoints_csv
from kinematics.steering.geometry import (
    PitmanArmGeometry2D,
    PitmanArmHardpoints3D,
    SteeringCoordinateSystem,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
    WheelSteeringGeometry2D,
    WheelSteeringHardpoints3D,
    project_kingpin_axis_to_steering_top_view,
    project_point_to_steering_top_view,
)
from kinematics.steering.two_segment import (
    solve_two_segment_from_left_wheel_angle,
    solve_two_segment_from_right_wheel_angle,
    solve_two_segment_steering,
    sweep_two_segment_steering,
)

__all__ = [
    "PitmanArmGeometry2D",
    "PitmanArmHardpoints3D",
    "SteeringCoordinateSystem",
    "TwoSegmentSteeringHardpoints3D",
    "TwoSegmentSteeringGeometry",
    "TwoSegmentSteeringSolution",
    "WheelSteeringHardpoints3D",
    "WheelSteeringGeometry2D",
    "load_two_segment_steering_hardpoints_csv",
    "project_kingpin_axis_to_steering_top_view",
    "project_point_to_steering_top_view",
    "solve_two_segment_from_left_wheel_angle",
    "solve_two_segment_from_right_wheel_angle",
    "solve_two_segment_steering",
    "sweep_two_segment_steering",
]
