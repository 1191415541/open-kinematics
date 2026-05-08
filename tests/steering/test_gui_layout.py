import inspect

from kinematics.steering.gui import SteeringWorkbenchApp


def test_simulation_input_controls_use_wrapping_grid_layout():
    source = inspect.getsource(SteeringWorkbenchApp._build_controls)

    assert ".grid(" in source
    assert "side=tk.LEFT" not in source
    assert "Wheelbase" in source


def test_slider_drag_uses_throttled_preview_refresh():
    source = inspect.getsource(SteeringWorkbenchApp._on_input_slider_changed)

    assert "_schedule_preview_refresh" in source
    assert "self.refresh()" not in source


def test_side_panel_includes_optimization_tab():
    source = inspect.getsource(SteeringWorkbenchApp._build_side_panel)

    assert "ttk.Notebook" in source
    assert "Optimization" in source
    assert "_build_optimization_tab" in source


def test_hardpoint_panel_includes_restore_default_button():
    source = inspect.getsource(SteeringWorkbenchApp._build_layout)

    assert "Restore Default Hardpoints" in source
    assert "restore_default_hardpoints" in source
