"""
CSV loader for pure 2D steering hardpoints.
"""

from __future__ import annotations

import csv
from collections.abc import Iterable, Mapping
from pathlib import Path
from typing import Any

from kinematics.core.constants import EPS_GEOMETRIC
from kinematics.steering.geometry import (
    PitmanArmHardpoints3D,
    TwoSegmentSteeringHardpoints3D,
    Vec3,
    WheelSteeringHardpoints3D,
    make_vec3,
)

SYMMETRIC_NAMES = frozenset(
    {
        "wheel_kingpin_lower",
        "wheel_kingpin_upper",
        "wheel_center",
        "wheel_tie_rod_pickup",
        "pitman_output",
    }
)
CENTER_NAMES = frozenset({"pitman_pivot"})
REQUIRED_COLUMNS = ("category", "name", "x", "y", "z")


def _row_vec3(row: dict[str, Any]) -> Vec3:
    try:
        return make_vec3([row["x"], row["y"], row["z"]])
    except (TypeError, ValueError) as exc:
        name = row.get("name", "<unknown>")
        raise ValueError(f"Invalid coordinates for hardpoint '{name}'") from exc


def _read_rows(path: str | Path) -> list[dict[str, str]]:
    with Path(path).open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("Steering hardpoint CSV is empty")
        missing = set(REQUIRED_COLUMNS) - set(reader.fieldnames)
        if missing:
            missing_columns = ", ".join(sorted(missing))
            raise ValueError(f"Missing required CSV columns: {missing_columns}")
        return [row for row in reader]


def _normalize_rows(
    rows: Iterable[Mapping[str, Any]],
) -> list[dict[str, Any]]:
    normalized = []
    for row in rows:
        missing = set(REQUIRED_COLUMNS) - set(row)
        if missing:
            missing_columns = ", ".join(sorted(missing))
            raise ValueError(f"Missing required CSV columns: {missing_columns}")
        normalized.append({key: row[key] for key in REQUIRED_COLUMNS})
    return normalized


def _mirror_y(point: Vec3) -> Vec3:
    mirrored = point.copy()
    mirrored[1] *= -1.0
    return mirrored


def _put_point(points: dict[str, Vec3], name: str, point: Vec3) -> None:
    if name in points:
        raise ValueError(f"Duplicate steering hardpoint '{name}'")
    points[name] = point


def _parse_points(
    rows: list[dict[str, str]],
) -> tuple[dict[str, Vec3], dict[str, Vec3]]:
    symmetric: dict[str, Vec3] = {}
    center: dict[str, Vec3] = {}
    for row in rows:
        category = row["category"].strip().lower()
        name = row["name"].strip().lower()
        point = _row_vec3(row)
        if category == "symmetric":
            _parse_symmetric_point(symmetric, name, point)
        elif category == "center":
            _parse_center_point(center, name, point)
        else:
            raise ValueError(f"Unknown steering hardpoint category '{category}'")
    return symmetric, center


def _parse_symmetric_point(points: dict[str, Vec3], name: str, point: Vec3) -> None:
    if name not in SYMMETRIC_NAMES:
        raise ValueError(f"Unknown symmetric steering hardpoint '{name}'")
    if point[1] >= -EPS_GEOMETRIC:
        raise ValueError(f"Symmetric hardpoint '{name}' must use left-side Y < 0")
    _put_point(points, name, point)


def _parse_center_point(points: dict[str, Vec3], name: str, point: Vec3) -> None:
    if name not in CENTER_NAMES:
        raise ValueError(f"Unknown center steering hardpoint '{name}'")
    if abs(float(point[1])) > EPS_GEOMETRIC:
        raise ValueError(f"Center hardpoint '{name}' must lie on the centerline")
    _put_point(points, name, point)


def _validate_required(
    points: dict[str, Vec3],
    required: frozenset[str],
    label: str,
) -> None:
    missing = required - set(points)
    if missing:
        missing_points = ", ".join(sorted(missing))
        raise ValueError(f"Missing {label} steering hardpoints: {missing_points}")


def load_two_segment_steering_hardpoints_csv(
    path: str | Path,
) -> TwoSegmentSteeringHardpoints3D:
    """
    Load two-segment steering 3D hardpoints from CSV.
    """
    return load_two_segment_steering_hardpoints_rows(_read_rows(path))


def load_two_segment_steering_hardpoints_rows(
    rows: Iterable[Mapping[str, Any]],
) -> TwoSegmentSteeringHardpoints3D:
    """
    Build two-segment steering 3D hardpoints from CSV-like rows.
    """
    symmetric, center = _parse_points(_normalize_rows(rows))
    _validate_required(symmetric, SYMMETRIC_NAMES, "symmetric")
    _validate_required(center, CENTER_NAMES, "center")

    return TwoSegmentSteeringHardpoints3D(
        left_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=symmetric["wheel_kingpin_lower"],
            kingpin_upper=symmetric["wheel_kingpin_upper"],
            wheel_center=symmetric["wheel_center"],
            tie_rod_pickup=symmetric["wheel_tie_rod_pickup"],
        ),
        right_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=_mirror_y(symmetric["wheel_kingpin_lower"]),
            kingpin_upper=_mirror_y(symmetric["wheel_kingpin_upper"]),
            wheel_center=_mirror_y(symmetric["wheel_center"]),
            tie_rod_pickup=_mirror_y(symmetric["wheel_tie_rod_pickup"]),
        ),
        pitman=PitmanArmHardpoints3D(
            pivot=center["pitman_pivot"],
            left_output=symmetric["pitman_output"],
            right_output=_mirror_y(symmetric["pitman_output"]),
        ),
    )
