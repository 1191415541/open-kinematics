import inspect

from kinematics.gui.steering import SteeringWorkbenchApp
from kinematics.gui.steering.widgets import HardpointEditor
from kinematics.steering.gui import SteeringWorkbenchApp as LegacySteeringWorkbenchApp
from kinematics.steering.workbench import SteeringHardpointRow


def test_simulation_input_controls_use_wrapping_grid_layout():
    source = inspect.getsource(SteeringWorkbenchApp._build_controls)

    assert ".grid(" in source
    assert "side=tk.LEFT" not in source
    assert "Linkage" in source
    assert "linkage_type_var" in source
    assert "Sweep min" in source


def test_slider_drag_uses_throttled_preview_refresh():
    source = inspect.getsource(SteeringWorkbenchApp._on_input_slider_changed)

    assert "_schedule_preview_refresh" in source
    assert "self.refresh()" not in source


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


def test_steering_numeric_entries_use_commit_refresh_and_not_trace_refresh():
    controls_source = inspect.getsource(SteeringWorkbenchApp._build_controls)
    trace_source = inspect.getsource(SteeringWorkbenchApp._bind_control_vars)

    assert "self.bind_entry_commit_refresh" in controls_source
    assert "self.input_value_var" not in trace_source
    assert "self.sweep_min_var" not in trace_source
    assert "self.sweep_max_var" not in trace_source
    assert "self.sweep_step_var" not in trace_source
    assert "self.wheel_radius_var" not in trace_source
    assert "self.wheel_width_var" not in trace_source
    assert "self.wheelbase_var" not in trace_source


def test_curve_manager_label_entry_commits_before_refresh():
    from kinematics.gui.steering.widgets import CurveManager

    binding_source = inspect.getsource(CurveManager._bind_selection_changes)
    build_source = inspect.getsource(CurveManager._build)

    assert "label_var" not in binding_source
    assert "bind_entry_commit_events" in build_source
    assert "state=\"readonly\"" in build_source


def test_hardpoint_panel_includes_restore_default_button():
    source = inspect.getsource(SteeringWorkbenchApp._build_layout)

    assert "Restore Default Hardpoints" in source
    assert "restore_default_hardpoints" in source


def test_steering_hardpoint_editor_matches_suspension_table_format():
    source = inspect.getsource(HardpointEditor._build)

    assert 'columns = ("point", "x", "y", "z")' in source
    assert 'show="headings"' in source
    assert 'height=10' in source
    assert 'self.tree.heading(column, text=column)' in source


def test_steering_hardpoint_editor_uses_descriptive_display_names():
    editor = object.__new__(HardpointEditor)

    assert (
        editor._display_name(
            SteeringHardpointRow("symmetric", "wheel_kingpin_lower", 0.0, 0.0, 0.0)
        )
        == "Wheel Kingpin Lower"
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
