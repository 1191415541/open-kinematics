"""
Pure 2D steering linkage solvers.

The steering package is intentionally separate from the 3D suspension templates.
It models top-view linkage kinematics for systems where only roadwheel angle
relationships are required.
"""

from kinematics.steering.csv_loader import load_two_segment_steering_hardpoints_csv
from kinematics.steering.geometry import (
    BellcrankGeometry2D,
    PitmanArmGeometry2D,
    PitmanArmHardpoints3D,
    SteeringCoordinateSystem,
    ThreeSegmentSteeringGeometry,
    ThreeSegmentSteeringSolution,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
    WheelSteeringGeometry2D,
    WheelSteeringHardpoints3D,
    project_kingpin_axis_to_steering_top_view,
    project_point_to_steering_top_view,
)
from kinematics.steering.three_segment import (
    solve_three_segment_from_left_wheel_angle,
    solve_three_segment_from_right_bellcrank_angle,
    solve_three_segment_from_right_wheel_angle,
    solve_three_segment_steering,
    sweep_three_segment_steering,
)
from kinematics.steering.two_segment import (
    solve_two_segment_from_left_wheel_angle,
    solve_two_segment_from_right_wheel_angle,
    solve_two_segment_steering,
    sweep_two_segment_steering,
)

__all__ = [
    "BellcrankGeometry2D",
    "PitmanArmGeometry2D",
    "PitmanArmHardpoints3D",
    "SteeringCoordinateSystem",
    "ThreeSegmentSteeringGeometry",
    "ThreeSegmentSteeringSolution",
    "TwoSegmentSteeringHardpoints3D",
    "TwoSegmentSteeringGeometry",
    "TwoSegmentSteeringSolution",
    "WheelSteeringHardpoints3D",
    "WheelSteeringGeometry2D",
    "load_two_segment_steering_hardpoints_csv",
    "project_kingpin_axis_to_steering_top_view",
    "project_point_to_steering_top_view",
    "solve_three_segment_from_left_wheel_angle",
    "solve_three_segment_from_right_bellcrank_angle",
    "solve_three_segment_from_right_wheel_angle",
    "solve_three_segment_steering",
    "solve_two_segment_from_left_wheel_angle",
    "solve_two_segment_from_right_wheel_angle",
    "solve_two_segment_steering",
    "sweep_three_segment_steering",
    "sweep_two_segment_steering",
]
