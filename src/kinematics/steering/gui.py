"""
Tkinter steering workbench GUI.
"""

from __future__ import annotations

import tkinter as tk
from pathlib import Path
from tkinter import ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from kinematics.steering.gui_file_actions import SteeringFileActions
from kinematics.steering.gui_plotting import (
    draw_curve_plot,
    draw_steering_preview,
    fit_steering_preview,
)
from kinematics.steering.gui_widgets import (
    CurveManager,
    HardpointEditor,
    OutputTable,
    PitmanTransformControls,
)
from kinematics.steering.two_segment import solve_two_segment_steering
from kinematics.steering.workbench import (
    INPUT_MODES,
    available_steering_outputs,
    curve_specs_for_plot,
    default_steering_project,
    hardpoints_from_rows,
    parse_float_entry,
    solve_steering_project,
    sweep_steering_project,
)


class SteeringWorkbenchApp(SteeringFileActions):
    """Main steering workbench window."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.project = default_steering_project()
        self.project_path: Path | None = None
        self.updating_controls = False
        self.preview_has_drawn = False
        self.input_mode_var = tk.StringVar(value=self.project.input_mode)
        self.input_value_var = tk.StringVar(value=str(self.project.input_value))
        self.sweep_min_var = tk.StringVar(value=str(self.project.sweep_min))
        self.sweep_max_var = tk.StringVar(value=str(self.project.sweep_max))
        self.sweep_step_var = tk.StringVar(value=str(self.project.sweep_step))
        self._build_menu()
        self._build_layout()
        self._bind_control_vars()
        self._load_project_to_controls()
        self.refresh()

    def _build_menu(self) -> None:
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="New", command=self.new_project)
        file_menu.add_command(label="Open Project", command=self.open_project)
        file_menu.add_command(label="Save", command=self.save_project)
        file_menu.add_command(label="Save As", command=self.save_project_as)
        file_menu.add_separator()
        file_menu.add_command(label="Import 3D Hardpoints CSV", command=self.import_csv)
        file_menu.add_command(label="Export 3D Hardpoints CSV", command=self.export_csv)
        file_menu.add_separator()
        file_menu.add_command(label="Exit", command=self.root.destroy)
        menu.add_cascade(label="File", menu=file_menu)
        self.root.config(menu=menu)

    def _build_layout(self) -> None:
        self.root.title("Two-Segment Steering Workbench")
        self.root.geometry("1200x760")
        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main, padding=8)
        right = ttk.Frame(main, padding=8)
        main.add(left, weight=1)
        main.add(right, weight=3)

        ttk.Label(left, text="3D Hardpoints").pack(anchor="w")
        self.hardpoint_editor = HardpointEditor(left, self._on_hardpoints_changed)
        self.hardpoint_editor.pack(fill=tk.BOTH, expand=True)
        self.pitman_controls = PitmanTransformControls(
            left,
            self._on_pitman_transform_changed,
        )
        self.pitman_controls.pack(fill=tk.X, pady=(8, 0))

        controls = ttk.LabelFrame(right, text="Simulation Input", padding=8)
        controls.pack(fill=tk.X)
        self._build_controls(controls)

        body = ttk.PanedWindow(right, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True, pady=8)
        self._build_preview(body)
        self._build_side_panel(body)

    def _build_controls(self, parent: ttk.Frame) -> None:
        ttk.Label(parent, text="Control").pack(side=tk.LEFT)
        ttk.Combobox(
            parent,
            textvariable=self.input_mode_var,
            values=INPUT_MODES,
            state="readonly",
            width=22,
        ).pack(side=tk.LEFT, padx=6)
        ttk.Label(parent, text="Value [deg]").pack(side=tk.LEFT)
        value_entry = ttk.Entry(parent, textvariable=self.input_value_var, width=10)
        value_entry.pack(side=tk.LEFT, padx=6)
        for label, var in (
            ("Sweep min", self.sweep_min_var),
            ("Sweep max", self.sweep_max_var),
            ("Step", self.sweep_step_var),
        ):
            ttk.Label(parent, text=label).pack(side=tk.LEFT, padx=(12, 0))
            ttk.Entry(parent, textvariable=var, width=8).pack(side=tk.LEFT, padx=4)

    def _build_preview(self, parent: ttk.PanedWindow) -> None:
        frame = ttk.Frame(parent)
        parent.add(frame, weight=2)
        self.preview_fig = Figure(figsize=(7, 6), dpi=100)
        self.preview_ax = self.preview_fig.add_subplot(111)
        self.preview_canvas = FigureCanvasTkAgg(self.preview_fig, master=frame)
        toolbar_frame = ttk.Frame(frame)
        toolbar_frame.pack(fill=tk.X)
        toolbar = NavigationToolbar2Tk
        self.preview_toolbar = toolbar(
            self.preview_canvas, toolbar_frame, pack_toolbar=False
        )
        self.preview_toolbar.update()
        self.preview_toolbar.pack(side=tk.LEFT, fill=tk.X, expand=True)
        fit_button = ttk.Button(toolbar_frame, text="Fit", command=self.fit_preview)
        fit_button.pack(side=tk.RIGHT)
        self.preview_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_side_panel(self, parent: ttk.PanedWindow) -> None:
        panel = ttk.Frame(parent)
        parent.add(panel, weight=1)
        output_frame = ttk.LabelFrame(panel, text="Outputs", padding=6)
        output_frame.pack(fill=tk.BOTH, expand=True)
        self.output_table = OutputTable(output_frame)
        self.output_table.pack(fill=tk.BOTH, expand=True)

        curve_frame = ttk.LabelFrame(panel, text="Curves", padding=6)
        curve_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        outputs = available_steering_outputs()
        self.curve_manager = CurveManager(curve_frame, outputs, self.refresh_curves)
        self.curve_manager.pack(fill=tk.X)
        self.curve_fig = Figure(figsize=(5, 3), dpi=100)
        self.curve_ax = self.curve_fig.add_subplot(111)
        self.curve_canvas = FigureCanvasTkAgg(self.curve_fig, master=curve_frame)
        self.curve_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(6, 0))

    def _bind_control_vars(self) -> None:
        for var in (
            self.input_mode_var,
            self.input_value_var,
            self.sweep_min_var,
            self.sweep_max_var,
            self.sweep_step_var,
        ):
            var.trace_add("write", self._on_controls_changed)

    def _load_project_to_controls(self) -> None:
        self.updating_controls = True
        self.preview_has_drawn = False
        self.input_mode_var.set(self.project.input_mode)
        self.input_value_var.set(str(self.project.input_value))
        self.sweep_min_var.set(str(self.project.sweep_min))
        self.sweep_max_var.set(str(self.project.sweep_max))
        self.sweep_step_var.set(str(self.project.sweep_step))
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self.pitman_controls.set_rows(self.project.hardpoints)
        self.curve_manager.set_curves(self.project.curves)
        self.updating_controls = False

    def _sync_controls_to_project(self) -> bool:
        self.project.input_mode = self.input_mode_var.get()
        for attr, var in (
            ("input_value", self.input_value_var),
            ("sweep_min", self.sweep_min_var),
            ("sweep_max", self.sweep_max_var),
            ("sweep_step", self.sweep_step_var),
        ):
            parsed = parse_float_entry(var.get(), getattr(self.project, attr))
            if not parsed.is_valid:
                self.output_table.set_error(f"Invalid numeric input: {attr}")
                return False
            setattr(self.project, attr, parsed.value)
        return True

    def _on_controls_changed(self, *_args: object) -> None:
        if not self.updating_controls:
            self.refresh()

    def _on_hardpoints_changed(self) -> None:
        self.pitman_controls.set_rows(self.project.hardpoints)
        self.refresh()

    def _on_pitman_transform_changed(self) -> None:
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self.refresh()

    def fit_preview(self) -> None:
        fit_steering_preview(self.preview_ax)
        self.preview_toolbar.update()
        self.preview_canvas.draw_idle()

    def refresh(self) -> None:
        """Refresh preview, outputs, and curves."""
        if not self._sync_controls_to_project():
            return
        try:
            hardpoints = hardpoints_from_rows(self.project.hardpoints)
            design_state = solve_two_segment_steering(hardpoints, 0.0)
            state, outputs = solve_steering_project(self.project)
            draw_steering_preview(
                self.preview_ax,
                hardpoints,
                design_state,
                state,
                preserve_view=self.preview_has_drawn,
            )
            self.preview_has_drawn = True
            self.preview_toolbar.update()
            self.preview_canvas.draw_idle()
            self.output_table.set_outputs(outputs)
            self.refresh_curves()
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            self.output_table.set_error(str(exc))

    def refresh_curves(self) -> None:
        """Refresh managed curve plots."""
        if not self._sync_controls_to_project():
            return
        try:
            rows = sweep_steering_project(self.project, skip_unreachable=True)
            curves = curve_specs_for_plot(
                self.project.curves,
                self.curve_manager.x_var.get(),
                self.curve_manager.y_var.get(),
                self.curve_manager.label_var.get(),
            )
            draw_curve_plot(self.curve_ax, rows, curves)
            self.curve_canvas.draw_idle()
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            self.output_table.set_error(str(exc))


def main() -> None:
    """Run the steering workbench GUI."""
    root = tk.Tk()
    SteeringWorkbenchApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
