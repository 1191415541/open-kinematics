import numpy as np

from kinematics.core.enums import PointID
from kinematics.gui.hardpoint_merge import (
    detect_hardpoint_conflicts,
    merge_export_hardpoints,
    steering_display_name,
    steering_export_hardpoints,
    suspension_display_name,
    suspension_export_hardpoints,
)
from kinematics.steering.workbench import SteeringHardpointRow


def test_shared_display_names_unify_overlap_terms() -> None:
    assert suspension_display_name(PointID.CARRIER_STEERING_AXIS_LOWER) == "Kingpin Lower"
    assert suspension_display_name(PointID.TRACKROD_OUTBOARD) == "Tie Rod Outer"
    assert steering_display_name("wheel_kingpin_lower") == "Kingpin Lower"
    assert steering_display_name("wheel_tie_rod_pickup") == "Tie Rod Outer"


def test_detect_hardpoint_conflicts_finds_mismatched_overlap_points() -> None:
    suspension_items = suspension_export_hardpoints(
        {
            PointID.CARRIER_STEERING_AXIS_LOWER: np.asarray(
                [-10.0, -20.0, 30.0],
                dtype=np.float64,
            ),
            PointID.TRACKROD_OUTBOARD: np.asarray(
                [100.0, 200.0, 300.0],
                dtype=np.float64,
            ),
        }
    )
    steering_items = steering_export_hardpoints(
        [
            SteeringHardpointRow(
                "symmetric",
                "wheel_kingpin_lower",
                10.0,
                20.0,
                30.0,
            ),
            SteeringHardpointRow(
                "symmetric",
                "wheel_tie_rod_pickup",
                101.0,
                -199.0,
                300.0,
            ),
        ]
    )

    conflicts = detect_hardpoint_conflicts(suspension_items, steering_items)

    assert [conflict.export_name for conflict in conflicts] == ["tie_rod_outer"]


def test_merge_export_hardpoints_can_prefer_steering_or_average() -> None:
    suspension_items = suspension_export_hardpoints(
        {
            PointID.TRACKROD_INBOARD: np.asarray(
                [-100.0, -20.0, 30.0],
                dtype=np.float64,
            ),
        }
    )
    steering_items = steering_export_hardpoints(
        [
            SteeringHardpointRow(
                "symmetric",
                "pitman_output",
                90.0,
                -25.0,
                35.0,
            ),
        ]
    )

    steering_rows = merge_export_hardpoints(
        suspension_items,
        steering_items,
        choices={"tie_rod_inner": "steering"},
    )
    average_rows = merge_export_hardpoints(
        suspension_items,
        steering_items,
        choices={"tie_rod_inner": "average"},
    )

    assert steering_rows == [
        {
            "point": "tie_rod_inner",
            "label": "Tie Rod Inner",
            "source": "steering",
            "x": "90",
            "y": "-25",
            "z": "35",
        }
    ]
    assert average_rows == [
        {
            "point": "tie_rod_inner",
            "label": "Tie Rod Inner",
            "source": "suspension",
            "x": "95",
            "y": "-2.5",
            "z": "32.5",
        }
    ]
