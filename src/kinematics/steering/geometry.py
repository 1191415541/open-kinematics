"""
Shared 2D steering geometry types.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Final

import numpy as np
from numpy.typing import NDArray

from kinematics.core.constants import EPS_GEOMETRIC

Vec2 = NDArray[np.float64]
Vec3 = NDArray[np.float64]


class SteeringCoordinateSystem:
    """
    Vehicle coordinate convention used by pure 2D steering models.

    +X points rearward, +Y points to vehicle right, and +Z points upward.
    Top-view steering geometry uses the first two coordinates: [x_rear, y_right].
    """

    X_REAR: Final[Vec3] = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    Y_RIGHT: Final[Vec3] = np.array([0.0, 1.0, 0.0], dtype=np.float64)
    Z_UP: Final[Vec3] = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    TOP_VIEW_X_LABEL: Final[str] = "X [mm] - vehicle rear"
    TOP_VIEW_Y_LABEL: Final[str] = "Y [mm] - vehicle right"


def make_vec2(data: Any) -> Vec2:
    """Convert input data to a 2D float64 vector."""
    arr = np.asarray(data, dtype=np.float64)
    if arr.shape != (2,):
        raise ValueError(f"Vec2 must have shape (2,), got {arr.shape}")
    return arr.copy()


def make_vec3(data: Any) -> Vec3:
    """Convert input data to a 3D float64 vector."""
    arr = np.asarray(data, dtype=np.float64)
    if arr.shape != (3,):
        raise ValueError(f"Vec3 must have shape (3,), got {arr.shape}")
    return arr.copy()


def project_point_to_steering_top_view(point: Any) -> Vec2:
    """
    Map a 3D steering hardpoint to 2D top-view coordinates.
    """
    return make_vec3(point)[:2].copy()


def project_kingpin_axis_to_steering_top_view(
    lower_point: Any,
    upper_point: Any,
    reference_z: float,
) -> Vec2:
    """
    Map a 3D kingpin axis to one 2D top-view pivot point.
    """
    lower = make_vec3(lower_point)
    upper = make_vec3(upper_point)
    dz = upper[2] - lower[2]
    if abs(float(dz)) <= EPS_GEOMETRIC:
        raise ValueError("Kingpin axis points must not share the same Z coordinate")

    t = (float(reference_z) - lower[2]) / dz
    return (lower + t * (upper - lower))[:2].copy()


def distance_2d(a: Vec2, b: Vec2) -> float:
    """Return the Euclidean distance between two 2D points."""
    return float(np.linalg.norm(a - b))


def rotate_point_2d(point: Vec2, pivot: Vec2, angle_rad: float) -> Vec2:
    """Rotate a point around a pivot in the XY plane."""
    rel = point - pivot
    cos_a = float(np.cos(angle_rad))
    sin_a = float(np.sin(angle_rad))
    rotated = np.array(
        [
            rel[0] * cos_a - rel[1] * sin_a,
            rel[0] * sin_a + rel[1] * cos_a,
        ],
        dtype=np.float64,
    )
    return pivot + rotated


def _validate_distinct_points(name: str, a: Vec2, b: Vec2) -> None:
    if distance_2d(a, b) <= EPS_GEOMETRIC:
        raise ValueError(f"{name} points must be distinct")


@dataclass(frozen=True)
class WheelSteeringGeometry2D:
    """
    One roadwheel/knuckle in 2D top-view steering geometry.
    """

    kingpin: Vec2
    wheel_center: Vec2
    tie_rod_pickup: Vec2

    def __post_init__(self) -> None:
        kingpin = make_vec2(self.kingpin)
        wheel_center = make_vec2(self.wheel_center)
        tie_rod_pickup = make_vec2(self.tie_rod_pickup)
        _validate_distinct_points(
            "Wheel kingpin and tie-rod pickup",
            kingpin,
            tie_rod_pickup,
        )
        object.__setattr__(self, "kingpin", kingpin)
        object.__setattr__(self, "wheel_center", wheel_center)
        object.__setattr__(self, "tie_rod_pickup", tie_rod_pickup)


@dataclass(frozen=True)
class PitmanArmGeometry2D:
    """
    Center pitman arm with independent left and right tie-rod pickups.
    """

    pivot: Vec2
    left_output: Vec2
    right_output: Vec2

    def __post_init__(self) -> None:
        pivot = make_vec2(self.pivot)
        left_output = make_vec2(self.left_output)
        right_output = make_vec2(self.right_output)
        _validate_distinct_points("Pitman pivot and left output", pivot, left_output)
        _validate_distinct_points("Pitman pivot and right output", pivot, right_output)
        object.__setattr__(self, "pivot", pivot)
        object.__setattr__(self, "left_output", left_output)
        object.__setattr__(self, "right_output", right_output)

    def rotate(self, angle_rad: float) -> tuple[Vec2, Vec2]:
        """Return left and right pitman output positions after rotation."""
        return (
            rotate_point_2d(self.left_output, self.pivot, angle_rad),
            rotate_point_2d(self.right_output, self.pivot, angle_rad),
        )


@dataclass(frozen=True)
class TwoSegmentSteeringGeometry:
    """Complete two-segment steering geometry."""

    left_wheel: WheelSteeringGeometry2D
    right_wheel: WheelSteeringGeometry2D
    pitman: PitmanArmGeometry2D

    def __post_init__(self) -> None:
        _validate_distinct_points(
            "Left tie rod",
            self.left_wheel.tie_rod_pickup,
            self.pitman.left_output,
        )
        _validate_distinct_points(
            "Right tie rod",
            self.right_wheel.tie_rod_pickup,
            self.pitman.right_output,
        )

    @property
    def left_tie_rod_length(self) -> float:
        """Design length of the left tie rod."""
        return distance_2d(self.left_wheel.tie_rod_pickup, self.pitman.left_output)

    @property
    def right_tie_rod_length(self) -> float:
        """Design length of the right tie rod."""
        return distance_2d(self.right_wheel.tie_rod_pickup, self.pitman.right_output)


@dataclass(frozen=True)
class WheelSteeringHardpoints3D:
    """
    One roadwheel/knuckle hardpoint set in steering vehicle coordinates.
    """

    kingpin_lower: Vec3
    kingpin_upper: Vec3
    wheel_center: Vec3
    tie_rod_pickup: Vec3

    def __post_init__(self) -> None:
        object.__setattr__(self, "kingpin_lower", make_vec3(self.kingpin_lower))
        object.__setattr__(self, "kingpin_upper", make_vec3(self.kingpin_upper))
        object.__setattr__(self, "wheel_center", make_vec3(self.wheel_center))
        object.__setattr__(self, "tie_rod_pickup", make_vec3(self.tie_rod_pickup))

    def to_2d_geometry(self) -> WheelSteeringGeometry2D:
        """Project this 3D wheel hardpoint set to top-view steering geometry."""
        return WheelSteeringGeometry2D(
            kingpin=project_kingpin_axis_to_steering_top_view(
                self.kingpin_lower,
                self.kingpin_upper,
                reference_z=float(self.wheel_center[2]),
            ),
            wheel_center=project_point_to_steering_top_view(self.wheel_center),
            tie_rod_pickup=project_point_to_steering_top_view(self.tie_rod_pickup),
        )


@dataclass(frozen=True)
class PitmanArmHardpoints3D:
    """Center pitman arm hardpoints in steering vehicle coordinates."""

    pivot: Vec3
    left_output: Vec3
    right_output: Vec3

    def __post_init__(self) -> None:
        object.__setattr__(self, "pivot", make_vec3(self.pivot))
        object.__setattr__(self, "left_output", make_vec3(self.left_output))
        object.__setattr__(self, "right_output", make_vec3(self.right_output))

    def to_2d_geometry(self) -> PitmanArmGeometry2D:
        """Project this 3D pitman hardpoint set to top-view steering geometry."""
        return PitmanArmGeometry2D(
            pivot=project_point_to_steering_top_view(self.pivot),
            left_output=project_point_to_steering_top_view(self.left_output),
            right_output=project_point_to_steering_top_view(self.right_output),
        )


@dataclass(frozen=True)
class TwoSegmentSteeringHardpoints3D:
    """Complete 3D hardpoint set for a two-segment steering linkage."""

    left_wheel: WheelSteeringHardpoints3D
    right_wheel: WheelSteeringHardpoints3D
    pitman: PitmanArmHardpoints3D

    def to_2d_geometry(self) -> TwoSegmentSteeringGeometry:
        """Project 3D hardpoints to the 2D geometry required by the solver."""
        return TwoSegmentSteeringGeometry(
            left_wheel=self.left_wheel.to_2d_geometry(),
            right_wheel=self.right_wheel.to_2d_geometry(),
            pitman=self.pitman.to_2d_geometry(),
        )


@dataclass(frozen=True)
class TwoSegmentSteeringSolution:
    """Solved two-segment steering state for one pitman input angle."""

    pitman_angle_deg: float
    left_wheel_angle_deg: float
    right_wheel_angle_deg: float
    left_wheel_center: Vec2
    right_wheel_center: Vec2
    left_tie_rod_pickup: Vec2
    right_tie_rod_pickup: Vec2
    pitman_left_output: Vec2
    pitman_right_output: Vec2
    left_tie_rod_residual: float
    right_tie_rod_residual: float
    converged: bool
    nfev: int

    @property
    def max_abs_tie_rod_residual(self) -> float:
        """Maximum absolute tie-rod length residual in model units."""
        return max(abs(self.left_tie_rod_residual), abs(self.right_tie_rod_residual))
