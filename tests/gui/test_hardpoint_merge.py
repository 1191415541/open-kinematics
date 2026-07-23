import numpy as np

from kinematics.core.enums import PointID
from kinematics.gui.app import KinematicsWorkbenchApp
from kinematics.gui.hardpoint_merge import (
    detect_hardpoint_conflicts,
    merge_export_hardpoints,
    steering_display_name,
    steering_export_hardpoints,
    steering_rows_from_suspension_hardpoints,
    suspension_display_name,
    suspension_export_hardpoints,
)
from kinematics.gui.steering import SteeringWorkbenchApp
from kinematics.gui.suspension import SuspensionWorkbenchPage
from kinematics.gui.suspension.workbench import create_default_suspension_project
from kinematics.steering.workbench import (
    SteeringHardpointRow,
    default_hardpoint_rows,
    default_steering_project,
    input_angle_slider_limits,
    solve_steering_project,
)


def test_shared_display_names_unify_overlap_terms() -> None:
    assert (
        suspension_display_name(PointID.CARRIER_STEERING_AXIS_LOWER) == "Kingpin Lower"
    )
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


def test_suspension_hardpoints_map_to_steering_rows_in_gui_coordinates() -> None:
    rows = steering_rows_from_suspension_hardpoints(
        {
            PointID.CARRIER_STEERING_AXIS_LOWER: np.asarray([10.0, 20.0, 30.0]),
            PointID.CARRIER_STEERING_AXIS_UPPER: np.asarray([40.0, 50.0, 60.0]),
            PointID.TRACKROD_OUTBOARD: np.asarray([70.0, 80.0, 90.0]),
            PointID.TRACKROD_INBOARD: np.asarray([100.0, 110.0, 120.0]),
        },
        wheel_center=np.asarray([130.0, 140.0, 150.0]),
        existing_rows=default_hardpoint_rows("two_segment"),
    )
    rows_by_name = {row.name: row for row in rows}

    assert (
        rows_by_name["wheel_kingpin_lower"].x,
        rows_by_name["wheel_kingpin_lower"].y,
        rows_by_name["wheel_kingpin_lower"].z,
    ) == (-10.0, -20.0, 30.0)
    assert (
        rows_by_name["wheel_kingpin_upper"].x,
        rows_by_name["wheel_kingpin_upper"].y,
        rows_by_name["wheel_kingpin_upper"].z,
    ) == (-40.0, -50.0, 60.0)
    assert (
        rows_by_name["wheel_center"].x,
        rows_by_name["wheel_center"].y,
        rows_by_name["wheel_center"].z,
    ) == (-130.0, -140.0, 150.0)
    assert (
        rows_by_name["wheel_tie_rod_pickup"].x,
        rows_by_name["wheel_tie_rod_pickup"].y,
        rows_by_name["wheel_tie_rod_pickup"].z,
    ) == (-70.0, -80.0, 90.0)
    assert (
        rows_by_name["pitman_output"].x,
        rows_by_name["pitman_output"].y,
        rows_by_name["pitman_output"].z,
    ) == (-100.0, -110.0, 120.0)
    assert rows_by_name["pitman_pivot"].category == "center"


def test_imported_suspension_hardpoints_drive_rack_and_pinion_steering() -> None:
    suspension_project = create_default_suspension_project()
    suspension = suspension_project.build_suspension()
    wheel_center = suspension.initial_state().get(PointID.WHEEL_CENTER)
    steering_project = default_steering_project()
    steering_project.hardpoints = steering_rows_from_suspension_hardpoints(
        suspension_project.hardpoints,
        wheel_center=wheel_center,
        existing_rows=steering_project.hardpoints,
    )
    steering_project.input_mode = "pinion_angle"
    steering_project.input_value = 0.0

    solution, outputs = solve_steering_project(
        steering_project,
        include_limits=False,
    )

    assert solution.converged
    assert outputs["pinion_angle_deg"] == 0.0
    assert outputs["rack_displacement_mm"] == 0.0
    slider_limits = input_angle_slider_limits(
        steering_project.hardpoints,
        "pinion_angle",
        pinion_pitch_radius_mm=steering_project.pinion_pitch_radius_mm,
    )
    assert slider_limits.minimum < 0.0 < slider_limits.maximum


def test_main_gui_imports_suspension_hardpoints_into_steering_page() -> None:
    class FakeSteeringPage:
        def __init__(self) -> None:
            self.project = default_steering_project(linkage_type="three_segment")
            self.imported_default_hardpoints: list[SteeringHardpointRow] = []
            self.pending_optimized_hardpoints = object()
            self.cache_reset = False
            self.controls_loaded = False
            self.refreshed = False

        def _reset_refresh_caches(self) -> None:
            self.cache_reset = True

        def _load_project_to_controls(self) -> None:
            self.controls_loaded = True

        def refresh(self) -> None:
            self.refreshed = True

    class FakeSuspensionPage:
        def __init__(self) -> None:
            self.project = create_default_suspension_project()

        def _sync_controls_to_project(self) -> bool:
            return True

    class FakeNotebook:
        def __init__(self) -> None:
            self.selected_tab: str | None = None

        def select(self, tab_id: str) -> None:
            self.selected_tab = tab_id

    steering_page = FakeSteeringPage()
    suspension_page = FakeSuspensionPage()
    app = object.__new__(KinematicsWorkbenchApp)
    app.pages = {"steering-tab": steering_page}
    app.notebook = FakeNotebook()
    app._page_by_type = lambda page_type: (
        steering_page
        if page_type is SteeringWorkbenchApp
        else suspension_page
        if page_type is SuspensionWorkbenchPage
        else None
    )

    app.import_suspension_hardpoints_to_steering()

    assert steering_page.project.linkage_type == "two_segment"
    assert steering_page.project.input_mode == "pinion_angle"
    assert steering_page.project.input_value == 0.0
    assert steering_page.pending_optimized_hardpoints is None
    assert steering_page.cache_reset is True
    assert steering_page.controls_loaded is True
    assert steering_page.refreshed is True
    assert app.notebook.selected_tab == "steering-tab"
