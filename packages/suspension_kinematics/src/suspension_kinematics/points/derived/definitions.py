"""
Common derived point calculation functions.

These functions calculate positions of derived points based on the positions of other
points in the suspension system. They are shared across different suspension types to
avoid code duplication.
"""

import numpy as np

from suspension_kinematics.core.enums import PointID
from suspension_kinematics.core.types import Vec3, WorldAxisSystem
from suspension_kinematics.core.vector_utils.generic import normalize_vector


def get_wheel_plane_down_vector(positions: dict[PointID, Vec3]) -> Vec3:
    """
    Calculates the 'down' direction vector in the wheel's plane of rotation.

    This vector is always perpendicular to the axle's direction and is calculated
    by finding the component of the global down vector that is orthogonal to
    the axle vector (using Gram-Schmidt orthogonalization).

    Args:
        positions: Dictionary of point coordinates. Must contain AXLE_INBOARD
                   and AXLE_OUTBOARD.

    Returns:
        A normalized 3D vector representing the 'down' direction in the
        wheel's plane.

    Raises:
        ValueError: If the axle has zero length or the resulting projected
                    down vector is a zero vector (i.e., axle is vertical).
    """
    axle_inboard = positions[PointID.AXLE_INBOARD]
    axle_outboard = positions[PointID.AXLE_OUTBOARD]

    # Compute the normalized axle direction (wheel's spin axis).
    axle_vector = axle_outboard - axle_inboard
    axle_direction = normalize_vector(axle_vector)

    # Find the 'down' direction within the wheel plane (perpendicular to the axle).
    global_down = -1 * WorldAxisSystem.Z

    # Project global down onto the plane perpendicular to the axle. This removes
    # the component of 'down' that is parallel to the axle.
    down_parallel_to_axle = np.dot(global_down, axle_direction) * axle_direction
    wheel_down = global_down - down_parallel_to_axle

    # Normalize to get the final unit vector. This will raise a ValueError if
    # the axle is vertical, which is the correct fail-fast behavior.
    return normalize_vector(wheel_down)


def get_axle_midpoint(positions: dict[PointID, Vec3]) -> Vec3:
    """
    Computes the center point between the inboard and outboard axle positions.

    Args:
        positions: Dictionary mapping point IDs to their 3D coordinates.
                  Must contain AXLE_INBOARD and AXLE_OUTBOARD entries.

    Returns:
        A numpy array representing the 3D coordinates of the axle midpoint.
    """
    p1 = positions[PointID.AXLE_INBOARD]
    p2 = positions[PointID.AXLE_OUTBOARD]
    return (p1 + p2) / 2


def get_wheel_center(positions: dict[PointID, Vec3], wheel_offset: float) -> Vec3:
    """
    Determine wheel center from hub face using ISO/SAE wheel-offset convention.

    Starting at `AXLE_OUTBOARD` (hub mounting face), this moves along the axle
    axis by `wheel_offset` toward axle inboard for positive values.

    Args:
        positions: Dictionary mapping point IDs to their 3D coordinates.
                Must contain AXLE_INBOARD and AXLE_OUTBOARD entries.
        wheel_offset: Wheel offset (ET) from hub mounting face to wheel center
                  plane in mm. Positive values place the wheel centerline
                  inboard of the hub face; negative values place it outboard.

    Returns:
        A numpy array representing the 3D coordinates of the wheel center.
    """
    p1 = positions[PointID.AXLE_OUTBOARD]  # Hub face.
    p2 = positions[PointID.AXLE_INBOARD]  # Axle inboard point.
    v = p1 - p2  # Points outboard; from inboard to axle outboard (hub face).
    v = normalize_vector(v)

    # ISO/SAE wheel offset convention: positive offset places centerline inboard.
    return p1 - v * wheel_offset


def wheel_axis_from_static_alignment(
    *,
    camber_deg: float,
    toe_deg: float,
    side_sign: float,
) -> Vec3:
    """
    Build the axle unit vector (inboard -> outboard) from static camber/toe.

    Conventions match :mod:`suspension_kinematics.metrics.angles`:
    - Positive camber tilts the top of the wheel outward.
    - Positive toe is toe-in (front of the wheel points inward).
    - ``side_sign`` is +1 for left (Y > 0) and -1 for right.
    """
    camber_rad = np.deg2rad(float(camber_deg))
    toe_rad = np.deg2rad(float(toe_deg))
    side = 1.0 if float(side_sign) >= 0.0 else -1.0

    # Left side: outboard is +Y. Right side: outboard is -Y.
    # Camber: top-out positive => axle outboard end lower (negative Z for left).
    # Toe-in positive => axle outboard end moves rearward (-X for left when
    # measuring relative to +Y).
    lateral = np.cos(camber_rad) * np.cos(toe_rad)
    longitudinal = side * np.sin(toe_rad) * np.cos(camber_rad)
    vertical = -side * np.sin(camber_rad)

    axis = np.asarray(
        [longitudinal, side * lateral, vertical],
        dtype=np.float64,
    )
    return normalize_vector(axis)


def axle_points_from_wheel_center(
    wheel_center: Vec3,
    *,
    camber_deg: float,
    toe_deg: float,
    wheel_offset: float,
    axle_length_mm: float,
    side_sign: float | None = None,
) -> tuple[Vec3, Vec3]:
    """
    Generate axle inboard/outboard hardpoints from wheel-center + alignment.

    Returns:
        ``(axle_inboard, axle_outboard)`` in internal coordinates.
    """
    center = np.asarray(wheel_center, dtype=np.float64)
    if side_sign is None:
        side_sign = -1.0 if float(center[1]) < 0.0 else 1.0
    axis_outboard = wheel_axis_from_static_alignment(
        camber_deg=camber_deg,
        toe_deg=toe_deg,
        side_sign=side_sign,
    )
    # Positive offset: wheel center is inboard of hub face (AXLE_OUTBOARD).
    axle_outboard = center + axis_outboard * float(wheel_offset)
    axle_inboard = axle_outboard - axis_outboard * float(axle_length_mm)
    return axle_inboard, axle_outboard


def apply_static_alignment_to_hardpoints(
    hardpoints: dict[PointID, Vec3],
    *,
    camber_deg: float,
    toe_deg: float,
    wheel_offset: float,
    axle_length_mm: float,
    side_sign: float | None = None,
) -> dict[PointID, Vec3]:
    """
    Ensure axle hardpoints match wheel-center + static alignment parameters.

    If ``WHEEL_CENTER`` is present it is treated as the design input and axle
    ends are generated from it. Otherwise existing axle ends are left unchanged.
    """
    updated = {
        point_id: np.asarray(position, dtype=np.float64).copy()
        for point_id, position in hardpoints.items()
    }
    if PointID.WHEEL_CENTER not in updated:
        return updated
    axle_inboard, axle_outboard = axle_points_from_wheel_center(
        updated[PointID.WHEEL_CENTER],
        camber_deg=camber_deg,
        toe_deg=toe_deg,
        wheel_offset=wheel_offset,
        axle_length_mm=axle_length_mm,
        side_sign=side_sign,
    )
    updated[PointID.AXLE_INBOARD] = axle_inboard
    updated[PointID.AXLE_OUTBOARD] = axle_outboard
    return updated


def get_wheel_inboard(positions: dict[PointID, Vec3], wheel_width: float) -> Vec3:
    """
    Determines the inboard edge position of the wheel by moving inward from the wheel
    center by half the wheel width along the axle axis.

    Args:
        positions: Dictionary mapping point IDs to their 3D coordinates.
                Must contain AXLE_INBOARD and WHEEL_CENTER entries.
        wheel_width: Total width of the wheel across its axial dimension.

    Returns:
        A numpy array representing the 3D coordinates of the wheel's inboard lip/edge.
    """
    p1 = positions[PointID.AXLE_INBOARD]
    p2 = positions[PointID.WHEEL_CENTER]
    v = p2 - p1  # Points outboard; from inboard to wheel center.
    v = normalize_vector(v)
    return p2 - v * (wheel_width / 2)


def get_wheel_outboard(positions: dict[PointID, Vec3], wheel_width: float) -> Vec3:
    """
    Determines the outboard edge position of the wheel by moving outward from the wheel
    center by half the wheel width along the axle axis.

    Args:
        positions: Dictionary mapping point IDs to their 3D coordinates.
                Must contain WHEEL_CENTER and AXLE_INBOARD entries.
        wheel_width: Total width of the wheel across its axial dimension.

    Returns:
        A numpy array representing the 3D coordinates of the wheel's outboard lip/edge.
    """
    p1 = positions[PointID.WHEEL_CENTER]
    p2 = positions[PointID.AXLE_INBOARD]
    v = p1 - p2  # Points outboard; from axle inboard to wheel center.
    v = normalize_vector(v)
    return p1 + v * (wheel_width / 2)


def get_contact_patch_center(
    positions: dict[PointID, Vec3], tire_radius: float
) -> Vec3:
    """
    Computes the position of the geometric contact patch center.

    This is the lowest point on an ideal tire circle in the wheel's center
    plane. It is found by moving from the wheel center in the wheel-plane
    'down' direction by a distance equal to the tire radius. Its Z-coordinate
    is not fixed and will move with the suspension.

    Args:
        positions: Dictionary of point coordinates.
        tire_radius: The radius of the tire in mm.

    Returns:
        The 3D coordinates of the geometric contact point.
    """
    wheel_center = positions[PointID.WHEEL_CENTER]
    wheel_down_normalized = get_wheel_plane_down_vector(positions)

    # Calculate the contact point by moving from the wheel center by the radius.
    contact_point = wheel_center + wheel_down_normalized * tire_radius

    return contact_point
