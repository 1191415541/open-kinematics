import inspect

from kinematics import cli
from kinematics.core.enums import PointID
from kinematics.gui.app import KinematicsWorkbenchApp
from kinematics.gui.suspension import SuspensionWorkbenchPage
from kinematics.gui.suspension.widgets import HardpointTable


def test_main_gui_hosts_steering_and_suspension_tabs() -> None:
    source = inspect.getsource(KinematicsWorkbenchApp._build_layout)

    assert "ttk.Notebook" in source
    assert "Steering" in source
    assert "Suspension" in source
    assert "SuspensionWorkbenchPage" in source


def test_main_gui_shared_menu_manages_hardpoints_project_save_and_close() -> None:
    source = inspect.getsource(KinematicsWorkbenchApp._build_menu)

    assert "Open Project" in source
    assert "Import Hardpoints" in source
    assert "Export Hardpoints" in source
    assert "Save Project" in source
    assert "Save Project As" in source
    assert "Close" in source


def test_main_gui_menu_dispatches_to_active_page() -> None:
    source = inspect.getsource(KinematicsWorkbenchApp._active_page)
    class_source = inspect.getsource(KinematicsWorkbenchApp)

    assert "self.notebook.select()" in source
    assert "self.pages" in source
    assert 'self._call_active_page("open_project")' in class_source
    assert 'self._call_active_page("save_project_as")' in class_source


def test_suspension_page_exposes_load_solve_and_curve_controls() -> None:
    layout_source = inspect.getsource(SuspensionWorkbenchPage._build_layout)
    controls_source = inspect.getsource(SuspensionWorkbenchPage._build_controls)
    parameters_source = inspect.getsource(SuspensionWorkbenchPage._build_parameters)
    side_source = inspect.getsource(SuspensionWorkbenchPage._build_side_panel)

    assert "ttk.PanedWindow" in layout_source
    assert "3D Hardpoints" in layout_source
    assert "Restore Default Hardpoints" in layout_source
    assert "HardpointTable" in layout_source
    assert "Simulation Input" in layout_source
    assert "left_parameters" in layout_source
    assert "_build_parameters(left_parameters)" in layout_source
    assert "Suspension Type" in controls_source
    assert "suspension_type_var" in controls_source
    assert "geometry_path_var" in controls_source
    assert "Suspension Parameters" in layout_source
    assert "Wheelbase" in parameters_source
    assert "Tire width" in parameters_source
    assert "ttk.Notebook" in side_source
    assert "Outputs" in side_source
    assert "Curves" in side_source
    assert "Optimization" in side_source
    assert "CurveManager" in side_source


def test_suspension_page_supports_carrier_type_in_selector() -> None:
    controls_source = inspect.getsource(SuspensionWorkbenchPage._build_controls)
    workbench_source = inspect.getsource(SuspensionWorkbenchPage._on_type_changed)

    assert "supported_suspension_type_keys()" in controls_source
    assert "create_default_suspension_project" in workbench_source


def test_suspension_hardpoint_table_is_compact_and_auto_sized() -> None:
    source = inspect.getsource(HardpointTable)
    layout_source = inspect.getsource(SuspensionWorkbenchPage._build_layout)
    class_source = inspect.getsource(SuspensionWorkbenchPage)

    assert "tksheet.Sheet(" in source
    assert "show_row_index=False" in source
    assert "show_top_left=False" in source
    assert 'width=420' in source
    assert 'height=260' in source
    assert "enable_bindings(" in source
    assert "bulk_table_edit_validation(" in source
    assert "DISPLAY_NAMES" in source
    assert "_display_name" in source
    assert 'table_align("center"' in source
    assert 'header_align("center"' in source
    assert 'align_columns(list(range(len(self.COLUMNS))), align="center"' in source
    assert 'width="text"' in source
    assert "self.main_panedwindow.add(left, weight=0)" in layout_source
    assert "self.main_panedwindow.add(right, weight=5)" in layout_source
    assert "DEFAULT_LEFT_PANE_WIDTH" in class_source
    assert "_apply_default_layout" in class_source


def test_suspension_hardpoint_table_uses_descriptive_display_names() -> None:
    table = object.__new__(HardpointTable)

    assert (
        table._display_name(PointID.TRACKROD_INBOARD) == "Track Rod Inboard"
    )
    assert (
        table._display_name(PointID.CARRIER_STEERING_AXIS_LOWER)
        == "Carrier Steering Axis Lower"
    )


def test_suspension_page_has_wheel_travel_slider_with_throttled_preview() -> None:
    controls_source = inspect.getsource(SuspensionWorkbenchPage._build_controls)
    preview_source = inspect.getsource(SuspensionWorkbenchPage._refresh_preview_only)
    draw_result_source = inspect.getsource(SuspensionWorkbenchPage._draw_result_index)
    init_source = inspect.getsource(SuspensionWorkbenchPage.__init__)
    class_source = inspect.getsource(SuspensionWorkbenchPage)

    assert "travel_slider_var" in controls_source
    assert "_on_travel_slider_changed" in controls_source
    assert "_schedule_preview_refresh" in class_source
    assert "_refresh_preview_only" in class_source
    assert "PREVIEW_REFRESH_DELAY_MS" in class_source
    assert "update_outputs=False" in preview_source
    assert "self.preview_renderer = SuspensionPreviewRenderer()" in init_source
    assert "renderer=self.preview_renderer" in draw_result_source
    assert "preview_mode=not update_outputs" in draw_result_source


def test_suspension_page_uses_full_sweep_and_slider_preview() -> None:
    init_source = inspect.getsource(SuspensionWorkbenchPage.__init__)
    refresh_source = inspect.getsource(SuspensionWorkbenchPage.refresh)
    refresh_curves_source = inspect.getsource(SuspensionWorkbenchPage.refresh_curves)
    slider_release_source = inspect.getsource(
        SuspensionWorkbenchPage._on_travel_slider_released
    )

    assert "_bind_control_vars" in init_source
    assert "self.refresh()" in init_source
    assert "solve_suspension_project_at_travel" in refresh_source
    assert "solve_suspension_project(self.project)" in refresh_curves_source
    assert "self.refresh()" in slider_release_source


def test_suspension_numeric_entries_use_commit_refresh_and_not_trace_refresh() -> None:
    controls_source = inspect.getsource(SuspensionWorkbenchPage._build_controls)
    parameters_source = inspect.getsource(SuspensionWorkbenchPage._build_parameters)
    trace_source = inspect.getsource(SuspensionWorkbenchPage._bind_control_vars)

    assert "self.bind_entry_commit_refresh" in controls_source
    assert "self.bind_entry_commit_refresh" in parameters_source
    assert "self.start_var" not in trace_source
    assert "self.stop_var" not in trace_source
    assert "self.steps_var" not in trace_source
    assert "self.wheelbase_var" not in trace_source
    assert "self.rim_diameter_var" not in trace_source


def test_suspension_preview_preserves_3d_view_during_motion() -> None:
    class_source = inspect.getsource(SuspensionWorkbenchPage)

    assert "preview_has_drawn" in class_source
    assert "preserve_view=self.preview_has_drawn" in class_source
    assert "self.preview_has_drawn = True" in class_source


def test_gui_project_save_dialogs_use_shared_project_extension() -> None:
    from kinematics.gui.steering import file_actions as steering_files
    from kinematics.gui.suspension import app as suspension_app

    assert "*.okproj.json" in repr(steering_files.PROJECT_FILETYPES)
    assert "*.okproj.json" in repr(suspension_app.PROJECT_FILETYPES)


def test_suspension_page_has_open_save_and_save_as_project_actions() -> None:
    class_source = inspect.getsource(SuspensionWorkbenchPage)
    load_source = inspect.getsource(SuspensionWorkbenchPage.load_geometry)
    import_source = inspect.getsource(SuspensionWorkbenchPage.import_hardpoints)

    assert "self.project_path" in class_source
    assert "def open_project" in class_source
    assert "def save_project_as" in class_source
    assert "load_suspension_project" in class_source
    assert "imported_default_hardpoints" in class_source
    assert "imported_default_hardpoints" in load_source
    assert "imported_default_hardpoints" in import_source
    assert "restore_default_hardpoints" in class_source


def test_suspension_page_has_optimization_actions() -> None:
    optimization_source = inspect.getsource(
        SuspensionWorkbenchPage._build_optimization_tab
    )
    optimization_content_source = inspect.getsource(
        SuspensionWorkbenchPage._build_optimization_content
    )
    variables_source = inspect.getsource(
        SuspensionWorkbenchPage._sync_available_optimization_variables
    )
    run_source = inspect.getsource(SuspensionWorkbenchPage.run_optimization)
    load_source = inspect.getsource(
        SuspensionWorkbenchPage._load_optimization_to_controls
    )
    side_source = inspect.getsource(SuspensionWorkbenchPage._build_side_panel)
    class_source = inspect.getsource(SuspensionWorkbenchPage)

    assert "_build_optimization_tab" in side_source
    assert "tk.Canvas" in optimization_source
    assert "ttk.Scrollbar" in optimization_source
    assert "canvas.create_window" in optimization_source
    assert "scrollregion" in optimization_source
    assert "_build_optimization_content" in optimization_source
    assert "ttk.Checkbutton" in optimization_content_source
    assert "row=index" in variables_source
    assert "column=0" in variables_source
    assert "Pair constraints" in optimization_content_source
    assert "Optimization Method" in optimization_content_source
    assert "Mode" in optimization_content_source
    assert "Weight" in optimization_content_source
    assert "opt_solver_mode_var" in class_source
    assert "SUSPENSION_OPTIMIZATION_TARGET_MODES" in optimization_content_source
    assert "Analyze Variables" in optimization_content_source
    assert "Select Recommended" in optimization_content_source
    assert "Select All" in optimization_content_source
    assert "Select None" in optimization_content_source
    assert "Invert" in optimization_content_source
    assert "Stop" in optimization_content_source
    assert "ttk.Progressbar" in optimization_content_source
    assert "tk.Text" in optimization_content_source
    assert "output_actions = ttk.Frame(output_frame)" in optimization_content_source
    assert "state=tk.DISABLED" in optimization_content_source
    assert "opt_variable_vars" in class_source
    assert "opt_pair_constraint_vars" in class_source
    assert "opt_target_mode_vars" in class_source
    assert "opt_target_weight_vars" in class_source
    assert "threading.Thread" in run_source
    assert "def run_optimization_analysis" in class_source
    assert "def _optimization_analysis_worker" in class_source
    assert '"analysis_result"' in class_source
    assert "Morris mu*" in class_source
    assert "Sobol directions/base samples" in class_source
    assert "tag_configure" in class_source
    assert "clipboard_clear" in class_source
    assert "clipboard_append" in class_source
    assert "def _select_recommended_optimization_variables" in class_source
    assert "def _select_all_optimization_variables" in class_source
    assert "def _clear_optimization_variable_selection" in class_source
    assert "def _invert_optimization_variable_selection" in class_source
    assert "def _optimization_variable_style_name" in class_source
    assert "style=" in variables_source
    assert "_poll_optimization_progress" in class_source
    assert "def stop_optimization" in class_source
    assert "_sync_available_optimization_pair_constraints" in load_source
    assert "def run_optimization" in class_source
    assert "def apply_optimization" in class_source


def test_cli_exposes_unified_gui_entrypoint() -> None:
    source = inspect.getsource(cli.gui)

    assert "kinematics.gui.app" in source
    assert "uv run --extra viz kinematics gui" in source
