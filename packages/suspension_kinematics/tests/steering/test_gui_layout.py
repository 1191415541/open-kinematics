import inspect
import re

from suspension_kinematics.gui.steering import SteeringWorkbenchApp
from suspension_kinematics.gui.steering.widgets import HardpointEditor
from suspension_kinematics.steering.gui import (
    SteeringWorkbenchApp as LegacySteeringWorkbenchApp,
)
from suspension_kinematics.steering.workbench import LINKAGE_TYPES, SteeringHardpointRow


def test_simulation_input_controls_use_wrapping_grid_layout():
    source = inspect.getsource(SteeringWorkbenchApp._build_controls)

    assert ".grid(" in source
    assert "side=tk.LEFT" not in source
    assert "Linkage" in source
    assert "linkage_type_var" in source
    assert "Sweep min" in source


def test_steering_suspension_parameters_use_shared_tire_fields():
    source = inspect.getsource(SteeringWorkbenchApp._build_suspension_parameters)

    assert "Tire width" in source
    assert "Static radius [mm]" in source
    assert "Wheel R" not in source
    assert "Wheel W" not in source
    assert "self.section_width_var" in source
    assert "self.static_radius_var" in source


def test_steering_linkage_selector_includes_rack_and_pinion():
    source = inspect.getsource(SteeringWorkbenchApp._sync_input_mode_values)
    visibility_source = inspect.getsource(
        SteeringWorkbenchApp._sync_linkage_control_visibility
    )

    assert "rack_and_pinion" in LINKAGE_TYPES
    assert "input_modes_for_linkage" in source
    assert "pinion_pitch_label" in visibility_source
    assert "RACK_AND_PINION_LINKAGE_TYPE" in visibility_source


def test_steering_controls_hide_pinion_radius_for_non_rack_linkages():
    source = inspect.getsource(SteeringWorkbenchApp._sync_linkage_control_visibility)
    geometry_source = inspect.getsource(SteeringWorkbenchApp._sync_geometry_controls)

    assert "grid_remove" in source
    assert 'linkage_type == "two_segment"' in geometry_source
    assert "RACK_AND_PINION_LINKAGE_TYPE" in geometry_source
    assert 'linkage_type == "three_segment"' in geometry_source
    assert "pack_forget" in geometry_source
    assert "rack_controls" in geometry_source
    assert "pitman_controls" in geometry_source
    assert "three_segment_controls" in geometry_source


def test_rack_geometry_controls_edit_steering_gear_x():
    from suspension_kinematics.gui.steering.widgets import RackGeometryControls

    build_source = inspect.getsource(RackGeometryControls._build)
    class_source = inspect.getsource(RackGeometryControls)

    assert "Steering gear X" in build_source
    assert "on_live_edit=lambda _event: None" in build_source
    assert "on_commit=self._on_entry_commit" in build_source
    assert "set_rack_x_position" in class_source
    assert "rack_x_position" in class_source


def test_three_segment_geometry_controls_edit_bellcrank_x_and_distance():
    from suspension_kinematics.gui.steering.widgets import ThreeSegmentGeometryControls

    build_source = inspect.getsource(ThreeSegmentGeometryControls._build)
    class_source = inspect.getsource(ThreeSegmentGeometryControls)

    assert "Bellcrank X" in build_source
    assert "L/R distance" in build_source
    assert "on_live_edit=lambda _event: None" in build_source
    assert "on_commit=self._on_entry_commit" in build_source
    assert "set_bellcrank_x_position" in class_source
    assert "set_bellcrank_lateral_distance" in class_source


def test_slider_drag_uses_throttled_preview_refresh():
    source = inspect.getsource(SteeringWorkbenchApp._on_input_slider_changed)

    assert "_schedule_preview_refresh" in source
    assert "self.refresh()" not in source


def test_hardpoint_edits_use_debounced_preview_then_full_refresh():
    source = inspect.getsource(SteeringWorkbenchApp._on_hardpoints_changed)

    assert "schedule_hardpoint_edit_refresh" in source
    assert "preview_callback=self._refresh_preview_only" in source
    assert "full_callback=self.refresh" in source
    assert "self.refresh()" not in source


def test_two_segment_preview_reuses_current_solved_state() -> None:
    source = inspect.getsource(SteeringWorkbenchApp._draw_preview_state)

    assert "solve_two_segment_steering(" in source
    assert "draw_steering_preview(" in source
    assert "state," in source


def test_side_panel_includes_optimization_tab():
    source = inspect.getsource(SteeringWorkbenchApp._build_side_panel)
    optimization_source = inspect.getsource(
        SteeringWorkbenchApp._build_optimization_tab
    )
    class_source = inspect.getsource(SteeringWorkbenchApp)

    assert "ttk.Notebook" in source
    assert "Optimization" in source
    assert "_build_optimization_tab" in source
    assert "Stop" in optimization_source
    assert "def stop_optimization" in class_source


def test_steering_layout_uses_shared_workspace_split_for_controls_preview_and_outputs():
    layout_source = inspect.getsource(SteeringWorkbenchApp._build_layout)
    preview_source = inspect.getsource(SteeringWorkbenchApp._build_preview)
    class_source = inspect.getsource(SteeringWorkbenchApp)

    assert "workspace = ttk.PanedWindow(right, orient=tk.HORIZONTAL)" in layout_source
    assert (
        "workspace.add(workspace_left, weight=self.WORKSPACE_PREVIEW_WEIGHT)"
        in layout_source
    )
    assert (
        "workspace.add(workspace_right, weight=self.WORKSPACE_SIDE_WEIGHT)"
        in layout_source
    )
    assert (
        'controls = ttk.LabelFrame(workspace_left, text="Simulation Input", padding=8)'
        in layout_source
    )
    assert "self._build_side_panel(workspace_right)" in layout_source
    assert "frame.pack(fill=tk.BOTH, expand=True)" in preview_source
    assert "WORKSPACE_PREVIEW_WEIGHT" in class_source
    assert "WORKSPACE_SIDE_WEIGHT" in class_source


def test_steering_numeric_entries_use_commit_refresh_and_not_trace_refresh():
    controls_source = inspect.getsource(SteeringWorkbenchApp._build_controls)
    trace_source = inspect.getsource(SteeringWorkbenchApp._bind_control_vars)

    assert "self.bind_entry_commit_refresh" in controls_source
    assert "self.input_value_var" not in trace_source
    assert "self.sweep_min_var" not in trace_source
    assert "self.sweep_max_var" not in trace_source
    assert "self.sweep_step_var" not in trace_source
    assert "self.static_radius_var" not in trace_source
    assert "self.section_width_var" not in trace_source
    assert "self.wheelbase_var" not in trace_source


def test_curve_manager_label_entry_commits_before_refresh():
    from suspension_kinematics.gui.steering.widgets import CurveManager

    binding_source = inspect.getsource(CurveManager._bind_selection_changes)
    build_source = inspect.getsource(CurveManager._build)

    assert "label_var" not in binding_source
    assert "bind_entry_commit_events" in build_source
    assert 'state="readonly"' in build_source


def test_pitman_geometry_entries_commit_before_applying_changes() -> None:
    from suspension_kinematics.gui.steering.widgets import PitmanTransformControls

    build_source = inspect.getsource(PitmanTransformControls._build)
    class_source = inspect.getsource(PitmanTransformControls)

    assert "on_live_edit=lambda _event: None" in build_source
    assert "on_commit=self._on_entry_commit" in build_source
    assert "def _on_entry_live_edit" not in class_source


def test_hardpoint_panel_includes_restore_default_button():
    source = inspect.getsource(SteeringWorkbenchApp._build_layout)

    assert "Restore Default Hardpoints" in source
    assert "restore_default_hardpoints" in source


def test_steering_hardpoint_editor_matches_suspension_table_format():
    source = inspect.getsource(HardpointEditor._build)

    assert "tksheet.Sheet(" in source
    assert "headers=list(self.COLUMNS)" in source
    assert "show_row_index=False" in source
    assert "enable_bindings(" in source


def test_steering_hardpoint_editor_enables_excel_like_bindings():
    source = inspect.getsource(HardpointEditor._build)

    assert '"copy"' in source
    assert '"paste"' in source
    assert '"undo"' in source
    assert '"edit_cell"' in source
    assert "bulk_table_edit_validation(" in source


def test_steering_hardpoint_editor_centers_text_and_auto_sizes_xyz_columns():
    source = inspect.getsource(HardpointEditor)

    assert 'table_align("center"' in source
    assert 'header_align("center"' in source
    assert re.search(
        r'align_columns\(\s*list\(range\(len\(self\.COLUMNS\)\)\),\s*align="center"',
        source,
    )
    assert 'width="text"' in source


def test_steering_hardpoint_editor_uses_descriptive_display_names():
    editor = object.__new__(HardpointEditor)

    assert (
        editor._display_name(
            SteeringHardpointRow("symmetric", "wheel_kingpin_lower", 0.0, 0.0, 0.0)
        )
        == "Kingpin Lower"
    )
    assert (
        editor._display_name(
            SteeringHardpointRow(
                "symmetric",
                "bellcrank_center_link_pickup",
                0.0,
                0.0,
                0.0,
            )
        )
        == "Bellcrank Center Link Pickup"
    )


def test_left_panel_moves_suspension_parameters_below_hardpoints_without_width_change():
    layout_source = inspect.getsource(SteeringWorkbenchApp._build_layout)
    parameter_source = inspect.getsource(
        SteeringWorkbenchApp._build_suspension_parameters
    )

    assert "main.add(left, weight=1)" in layout_source
    assert "main.add(right, weight=3)" in layout_source
    assert (
        layout_source.index("self.hardpoint_editor.pack(fill=tk.BOTH, expand=True)")
        < layout_source.index("self._build_suspension_parameters(left)")
        < layout_source.index("self.pitman_controls = PitmanTransformControls(")
    )
    assert "Suspension Parameters" in parameter_source
    assert "Wheelbase" in parameter_source
    assert "self.bind_entry_commit_refresh(entries)" in parameter_source


def test_steering_gui_supports_embedded_mode():
    source = inspect.getsource(SteeringWorkbenchApp.__init__)

    assert "standalone" in source
    assert "_build_menu" in source


def test_legacy_steering_gui_import_keeps_compatibility():
    assert LegacySteeringWorkbenchApp is SteeringWorkbenchApp
