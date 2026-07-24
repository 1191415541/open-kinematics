"""Shared hardpoint naming and combined export helpers for the GUI."""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Literal, Mapping

import numpy as np

from kinematics.core.constants import EPS_GEOMETRIC
from kinematics.core.enums import PointID
from kinematics.steering.workbench import (
    SteeringHardpointRow,
    hardpoint_names_for_linkage,
)

MergeChoice = Literal["suspension", "steering", "average"]


SUSPENSION_POINT_DISPLAY_NAMES: dict[PointID, str] = {
    PointID.LOWER_WISHBONE_INBOARD_FRONT: "Lower Wishbone Inboard Front",
    PointID.LOWER_WISHBONE_INBOARD_REAR: "Lower Wishbone Inboard Rear",
    PointID.LOWER_WISHBONE_OUTBOARD: "Lower Ball Joint",
    PointID.UPPER_WISHBONE_INBOARD_FRONT: "Upper Wishbone Inboard Front",
    PointID.UPPER_WISHBONE_INBOARD_REAR: "Upper Wishbone Inboard Rear",
    PointID.UPPER_WISHBONE_OUTBOARD: "Upper Ball Joint",
    PointID.TRACKROD_INBOARD: "Tie Rod Inner",
    PointID.TRACKROD_OUTBOARD: "Tie Rod Outer",
    PointID.WHEEL_CENTER: "Wheel Center",
    PointID.AXLE_INBOARD: "Axle Inboard",
    PointID.AXLE_OUTBOARD: "Axle Outboard",
    PointID.CARRIER_STEERING_AXIS_LOWER: "Kingpin Lower",
    PointID.CARRIER_STEERING_AXIS_UPPER: "Kingpin Upper",
}

STEERING_POINT_DISPLAY_NAMES: dict[str, str] = {
    "wheel_kingpin_lower": "Kingpin Lower",
    "wheel_kingpin_upper": "Kingpin Upper",
    "wheel_center": "Wheel Center",
    "wheel_tie_rod_pickup": "Tie Rod Outer",
    "pitman_output": "Tie Rod Inner",
    "pitman_pivot": "Pitman Pivot",
    "bellcrank_pivot": "Bellcrank Pivot",
    "bellcrank_center_link_pickup": "Bellcrank Center Link Pickup",
    "bellcrank_tie_rod_pickup": "Bellcrank Tie Rod Pickup",
}

SUSPENSION_EXPORT_ALIASES: dict[PointID, str] = {
    PointID.CARRIER_STEERING_AXIS_LOWER: "kingpin_lower",
    PointID.CARRIER_STEERING_AXIS_UPPER: "kingpin_upper",
    PointID.TRACKROD_INBOARD: "tie_rod_inner",
    PointID.TRACKROD_OUTBOARD: "tie_rod_outer",
    PointID.UPPER_WISHBONE_OUTBOARD: "upper_ball_joint",
    PointID.LOWER_WISHBONE_OUTBOARD: "lower_ball_joint",
    PointID.AXLE_INBOARD: "axle_inboard",
    PointID.AXLE_OUTBOARD: "axle_outboard",
    PointID.WHEEL_CENTER: "wheel_center",
}

STEERING_EXPORT_ALIASES: dict[str, str] = {
    "wheel_kingpin_lower": "kingpin_lower",
    "wheel_kingpin_upper": "kingpin_upper",
    "wheel_center": "wheel_center",
    "wheel_tie_rod_pickup": "tie_rod_outer",
    "pitman_output": "tie_rod_inner",
    "pitman_pivot": "pitman_pivot",
    "bellcrank_pivot": "bellcrank_pivot",
    "bellcrank_center_link_pickup": "bellcrank_center_link_pickup",
    "bellcrank_tie_rod_pickup": "bellcrank_tie_rod_pickup",
}

SUSPENSION_TO_STEERING_HARDPOINT_SOURCES: dict[str, tuple[str, ...]] = {
    "wheel_kingpin_lower": ("kingpin_lower", "lower_ball_joint"),
    "wheel_kingpin_upper": ("kingpin_upper", "upper_ball_joint"),
    "wheel_tie_rod_pickup": ("tie_rod_outer",),
    "pitman_output": ("tie_rod_inner",),
}


@dataclass(frozen=True)
class ExportHardpoint:
    """One hardpoint in the merged GUI-facing export space."""

    source: Literal["suspension", "steering"]
    source_key: str
    export_name: str
    display_name: str
    position: np.ndarray


@dataclass(frozen=True)
class HardpointConflict:
    """One overlapping export point with different source coordinates."""

    export_name: str
    display_name: str
    suspension_position: np.ndarray
    steering_position: np.ndarray

    def positions_match(self) -> bool:
        return bool(
            np.linalg.norm(self.suspension_position - self.steering_position)
            <= EPS_GEOMETRIC
        )


def suspension_display_name(point_id: PointID) -> str:
    """Return the unified suspension hardpoint display name."""
    return SUSPENSION_POINT_DISPLAY_NAMES.get(
        point_id, point_id.name.replace("_", " ").title()
    )


def steering_display_name(name: str) -> str:
    """Return the unified steering hardpoint display name."""
    return STEERING_POINT_DISPLAY_NAMES.get(name, name.replace("_", " ").title())


def suspension_export_hardpoints(
    hardpoints: dict[PointID, np.ndarray],
) -> list[ExportHardpoint]:
    """Normalize suspension hardpoints into GUI-facing export rows."""
    items: list[ExportHardpoint] = []
    for point_id, position in sorted(hardpoints.items()):
        gui_position = np.asarray(
            [-float(position[0]), -float(position[1]), float(position[2])],
            dtype=np.float64,
        )
        items.append(
            ExportHardpoint(
                source="suspension",
                source_key=point_id.name,
                export_name=SUSPENSION_EXPORT_ALIASES.get(
                    point_id, point_id.name.lower()
                ),
                display_name=suspension_display_name(point_id),
                position=np.asarray(gui_position, dtype=np.float64),
            )
        )
    return items


def steering_rows_from_suspension_hardpoints(
    hardpoints: Mapping[PointID, np.ndarray],
    *,
    wheel_center: np.ndarray,
    existing_rows: list[SteeringHardpointRow],
    linkage_type: str = "two_segment",
) -> list[SteeringHardpointRow]:
    """Map a suspension corner into editable steering rows for one linkage type.

    Suspension export coordinates already use the steering GUI convention
    (rear/right/up).  The derived wheel center is supplied separately because
    it is not a suspension hardpoint.  Linkage-specific actuator hardpoints that
    are not present in the suspension corner keep their existing values.
    """
    allowed_names = set(hardpoint_names_for_linkage(linkage_type))
    suspension_items = suspension_export_hardpoints(dict(hardpoints))
    positions_by_export_name = {
        item.export_name: item.position for item in suspension_items
    }
    required_sources = {
        steering_name: source_names
        for steering_name, source_names in (
            SUSPENSION_TO_STEERING_HARDPOINT_SOURCES.items()
        )
        if steering_name in allowed_names
    }
    missing = sorted(
        steering_name
        for steering_name, source_names in required_sources.items()
        if not any(name in positions_by_export_name for name in source_names)
    )
    if missing:
        raise ValueError(
            "Suspension is missing steering hardpoints: " + ", ".join(missing)
        )

    imported_positions = {
        steering_name: positions_by_export_name[
            next(
                source_name
                for source_name in source_names
                if source_name in positions_by_export_name
            )
        ]
        for steering_name, source_names in required_sources.items()
    }
    if "wheel_center" in allowed_names:
        imported_positions["wheel_center"] = np.asarray(
            [-float(wheel_center[0]), -float(wheel_center[1]), float(wheel_center[2])],
            dtype=np.float64,
        )

    rows: list[SteeringHardpointRow] = []
    remaining_positions = dict(imported_positions)
    for row in existing_rows:
        if row.name not in allowed_names:
            continue
        position = remaining_positions.pop(row.name, None)
        if position is None:
            rows.append(
                SteeringHardpointRow(
                    category=row.category,
                    name=row.name,
                    x=float(row.x),
                    y=float(row.y),
                    z=float(row.z),
                )
            )
            continue
        rows.append(
            SteeringHardpointRow(
                category=row.category,
                name=row.name,
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2]),
            )
        )

    for name in hardpoint_names_for_linkage(linkage_type):
        position = remaining_positions.pop(name, None)
        if position is None:
            continue
        if any(row.name == name for row in rows):
            continue
        rows.append(
            SteeringHardpointRow(
                category="symmetric",
                name=name,
                x=float(position[0]),
                y=float(position[1]),
                z=float(position[2]),
            )
        )
    return rows


def steering_export_hardpoints(
    rows: list[SteeringHardpointRow],
) -> list[ExportHardpoint]:
    """Normalize steering hardpoints into GUI-facing export rows."""
    return [
        ExportHardpoint(
            source="steering",
            source_key=row.name,
            export_name=STEERING_EXPORT_ALIASES.get(row.name, row.name),
            display_name=steering_display_name(row.name),
            position=np.asarray([row.x, row.y, row.z], dtype=np.float64),
        )
        for row in rows
    ]


def detect_hardpoint_conflicts(
    suspension_items: list[ExportHardpoint],
    steering_items: list[ExportHardpoint],
) -> list[HardpointConflict]:
    """Return all overlapping export hardpoints that need merge choices."""
    suspension_by_name = {item.export_name: item for item in suspension_items}
    steering_by_name = {item.export_name: item for item in steering_items}
    conflicts: list[HardpointConflict] = []
    for export_name in sorted(set(suspension_by_name) & set(steering_by_name)):
        suspension_item = suspension_by_name[export_name]
        steering_item = steering_by_name[export_name]
        if (
            np.linalg.norm(suspension_item.position - steering_item.position)
            <= EPS_GEOMETRIC
        ):
            continue
        conflicts.append(
            HardpointConflict(
                export_name=export_name,
                display_name=suspension_item.display_name,
                suspension_position=suspension_item.position,
                steering_position=steering_item.position,
            )
        )
    return conflicts


def merge_export_hardpoints(
    suspension_items: list[ExportHardpoint],
    steering_items: list[ExportHardpoint],
    *,
    choices: dict[str, MergeChoice] | None = None,
) -> list[dict[str, str]]:
    """Merge steering and suspension export rows into one CSV-friendly set."""
    selected = dict(choices or {})
    merged: dict[str, ExportHardpoint] = {
        item.export_name: item for item in suspension_items
    }
    for item in steering_items:
        existing = merged.get(item.export_name)
        if existing is None:
            merged[item.export_name] = item
            continue
        if np.linalg.norm(existing.position - item.position) <= EPS_GEOMETRIC:
            continue
        choice = selected.get(item.export_name, "suspension")
        if choice == "steering":
            merged[item.export_name] = item
        elif choice == "average":
            merged[item.export_name] = ExportHardpoint(
                source="suspension",
                source_key=existing.source_key,
                export_name=existing.export_name,
                display_name=existing.display_name,
                position=(existing.position + item.position) / 2.0,
            )
    return [
        {
            "point": export_name,
            "label": item.display_name,
            "source": item.source,
            "x": f"{float(item.position[0]):.12g}",
            "y": f"{float(item.position[1]):.12g}",
            "z": f"{float(item.position[2]):.12g}",
        }
        for export_name, item in sorted(merged.items())
    ]


def save_merged_hardpoints_csv(
    rows: list[dict[str, str]],
    path: str | Path,
) -> None:
    """Write merged hardpoints to CSV."""
    with Path(path).open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=("point", "label", "source", "x", "y", "z"),
        )
        writer.writeheader()
        writer.writerows(rows)
