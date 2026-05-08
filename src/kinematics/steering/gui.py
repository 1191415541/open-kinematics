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
    copy_hardpoint_rows,
    curve_specs_for_plot,
    default_steering_project,
    hardpoints_from_rows,
    input_angle_slider_limits,
    optimize_steering_hardpoints,
    parse_float_entry,
    solve_steering_project,
    sweep_steering_project,
)

OPTIMIZATION_VARIABLE_OPTIONS = (
    ("pitman_x", "Pitman X"),
    ("pitman_arm_x_length", "Arm X length"),
    ("tie_rod_outer_x", "Outer X"),
    ("tie_rod_outer_y", "Outer Y"),
    ("tie_rod_inner_x", "Inner X"),
    ("tie_rod_inner_y", "Inner Y"),
)


class SteeringWorkbenchApp(SteeringFileActions):
    """Main steering workbench window."""

    PREVIEW_REFRESH_DELAY_MS = 16

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.project = default_steering_project()
        self.project_path: Path | None = None
        self.updating_controls = False
        self.preview_has_drawn = False
        self.pending_preview_refresh: str | None = None
        self.input_mode_var = tk.StringVar(value=self.project.input_mode)
        self.input_value_var = tk.StringVar(value=str(self.project.input_value))
        self.input_slider_var = tk.DoubleVar(value=self.project.input_value)
        self.sweep_min_var = tk.StringVar(value=str(self.project.sweep_min))
        self.sweep_max_var = tk.StringVar(value=str(self.project.sweep_max))
        self.sweep_step_var = tk.StringVar(value=str(self.project.sweep_step))
        self.wheel_radius_var = tk.StringVar(value=str(self.project.wheel_radius))
        self.wheel_width_var = tk.StringVar(value=str(self.project.wheel_width))
        self.wheelbase_var = tk.StringVar(value=str(self.project.wheelbase))
        self.opt_inner_wheel_var = tk.StringVar(value="right")
        self.opt_inner_angle_var = tk.StringVar(value="10.0")
        self.opt_target_delta_var = tk.StringVar(value="-4.0")
        self.opt_delta_limit_var = tk.StringVar(value="40.0")
        self.opt_variable_vars = {
            name: tk.BooleanVar(value=name in {"pitman_x", "pitman_arm_x_length"})
            for name, _label in OPTIMIZATION_VARIABLE_OPTIONS
        }
        self.pending_optimized_hardpoints = None
        self.imported_default_hardpoints = copy_hardpoint_rows(self.project.hardpoints)
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

        hardpoint_header = ttk.Frame(left)
        hardpoint_header.pack(fill=tk.X)
        ttk.Label(hardpoint_header, text="3D Hardpoints").pack(side=tk.LEFT)
        ttk.Button(
            hardpoint_header,
            text="Restore Default Hardpoints",
            command=self.restore_default_hardpoints,
        ).pack(side=tk.RIGHT)
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
        parent.columnconfigure(5, weight=1)
        ttk.Label(parent, text="Control").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            parent,
            textvariable=self.input_mode_var,
            values=INPUT_MODES,
            state="readonly",
            width=22,
        ).grid(row=0, column=1, sticky="w", padx=(6, 12))
        ttk.Label(parent, text="Value [deg]").grid(row=0, column=2, sticky="w")
        value_entry = ttk.Entry(parent, textvariable=self.input_value_var, width=10)
        value_entry.grid(row=0, column=3, sticky="w", padx=(6, 12))
        ttk.Label(parent, text="Input").grid(row=0, column=4, sticky="w")
        self.input_slider = ttk.Scale(
            parent,
            variable=self.input_slider_var,
            command=self._on_input_slider_changed,
            length=220,
        )
        self.input_slider.grid(row=0, column=5, sticky="ew", padx=(6, 0))
        self.input_slider.bind("<ButtonRelease-1>", self._on_input_slider_released)
        for column, (label, var) in enumerate(
            (
                ("Sweep min", self.sweep_min_var),
                ("Sweep max", self.sweep_max_var),
                ("Step", self.sweep_step_var),
                ("Wheel R", self.wheel_radius_var),
                ("Wheel W", self.wheel_width_var),
                ("Wheelbase", self.wheelbase_var),
            )
        ):
            label_column = column * 2
            entry_column = label_column + 1
            ttk.Label(parent, text=label).grid(
                row=1,
                column=label_column,
                sticky="w",
                pady=(6, 0),
            )
            ttk.Entry(parent, textvariable=var, width=8).grid(
                row=1,
                column=entry_column,
                sticky="w",
                padx=(6, 18),
                pady=(6, 0),
            )

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
        notebook = ttk.Notebook(panel)
        notebook.pack(fill=tk.BOTH, expand=True)

        output_tab = ttk.Frame(notebook)
        notebook.add(output_tab, text="Outputs")
        output_frame = ttk.LabelFrame(output_tab, text="Outputs", padding=6)
        output_frame.pack(fill=tk.BOTH, expand=True)
        self.output_table = OutputTable(output_frame)
        self.output_table.pack(fill=tk.BOTH, expand=True)

        curve_frame = ttk.LabelFrame(output_tab, text="Curves", padding=6)
        curve_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        outputs = available_steering_outputs()
        self.curve_manager = CurveManager(curve_frame, outputs, self.refresh_curves)
        self.curve_manager.pack(fill=tk.X)
        self.curve_fig = Figure(figsize=(5, 3), dpi=100)
        self.curve_ax = self.curve_fig.add_subplot(111)
        self.curve_canvas = FigureCanvasTkAgg(self.curve_fig, master=curve_frame)
        self.curve_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(6, 0))

        optimization_tab = ttk.Frame(notebook, padding=6)
        notebook.add(optimization_tab, text="Optimization")
        self._build_optimization_tab(optimization_tab)

    def _build_optimization_tab(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        ttk.Label(parent, text="Inner wheel").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            parent,
            textvariable=self.opt_inner_wheel_var,
            values=("left", "right"),
            state="readonly",
            width=10,
        ).grid(row=0, column=1, sticky="ew", padx=(6, 0), pady=2)
        fields = (
            ("Inner angle [deg]", self.opt_inner_angle_var),
            ("Target L-R [deg]", self.opt_target_delta_var),
            ("Variable limit [mm]", self.opt_delta_limit_var),
        )
        for row_index, (label, var) in enumerate(fields, start=1):
            ttk.Label(parent, text=label).grid(row=row_index, column=0, sticky="w")
            ttk.Entry(parent, textvariable=var, width=12).grid(
                row=row_index,
                column=1,
                sticky="ew",
                padx=(6, 0),
                pady=2,
            )

        variables = ttk.LabelFrame(parent, text="Variables", padding=6)
        variables.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        for index, (name, label) in enumerate(OPTIMIZATION_VARIABLE_OPTIONS):
            ttk.Checkbutton(
                variables,
                text=label,
                variable=self.opt_variable_vars[name],
            ).grid(row=index // 2, column=index % 2, sticky="w", padx=(0, 10))

        buttons = ttk.Frame(parent)
        buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(8, 0))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        ttk.Button(buttons, text="Optimize", command=self.run_optimization).grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 4),
        )
        ttk.Button(buttons, text="Apply", command=self.apply_optimization).grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(4, 0),
        )
        self.optimization_status_var = tk.StringVar(value="No optimization run")
        ttk.Label(
            parent,
            textvariable=self.optimization_status_var,
            justify=tk.LEFT,
        ).grid(row=6, column=0, columnspan=2, sticky="ew", pady=(8, 0))

    def _bind_control_vars(self) -> None:
        for var in (
            self.input_mode_var,
            self.input_value_var,
            self.sweep_min_var,
            self.sweep_max_var,
            self.sweep_step_var,
            self.wheel_radius_var,
            self.wheel_width_var,
            self.wheelbase_var,
        ):
            var.trace_add("write", self._on_controls_changed)

    def _load_project_to_controls(self) -> None:
        self.updating_controls = True
        self.preview_has_drawn = False
        self.input_mode_var.set(self.project.input_mode)
        self.input_value_var.set(str(self.project.input_value))
        self.input_slider_var.set(self.project.input_value)
        self.sweep_min_var.set(str(self.project.sweep_min))
        self.sweep_max_var.set(str(self.project.sweep_max))
        self.sweep_step_var.set(str(self.project.sweep_step))
        self.wheel_radius_var.set(str(self.project.wheel_radius))
        self.wheel_width_var.set(str(self.project.wheel_width))
        self.wheelbase_var.set(str(self.project.wheelbase))
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self.pitman_controls.set_rows(self.project.hardpoints)
        self.curve_manager.set_curves(self.project.curves)
        self.updating_controls = False

    def _sync_input_slider_limits(self, value: float) -> None:
        limits = input_angle_slider_limits(
            self.project.hardpoints,
            self.project.input_mode,
        )
        self.input_slider.configure(from_=limits.minimum, to=limits.maximum)
        slider_value = min(max(value, limits.minimum), limits.maximum)
        self.input_slider_var.set(slider_value)

    def _sync_controls_to_project(self) -> bool:
        self.project.input_mode = self.input_mode_var.get()
        for attr, var in (
            ("input_value", self.input_value_var),
            ("sweep_min", self.sweep_min_var),
            ("sweep_max", self.sweep_max_var),
            ("sweep_step", self.sweep_step_var),
            ("wheel_radius", self.wheel_radius_var),
            ("wheel_width", self.wheel_width_var),
            ("wheelbase", self.wheelbase_var),
        ):
            parsed = parse_float_entry(var.get(), getattr(self.project, attr))
            if not parsed.is_valid:
                self.output_table.set_error(f"Invalid numeric input: {attr}")
                return False
            setattr(self.project, attr, parsed.value)
        return True

    def _on_input_slider_changed(self, value: str) -> None:
        if self.updating_controls:
            return
        self.updating_controls = True
        self.input_value_var.set(f"{float(value):.6g}")
        self.updating_controls = False
        self._schedule_preview_refresh()

    def _on_input_slider_released(self, _event: tk.Event) -> None:
        self.refresh()

    def _schedule_preview_refresh(self) -> None:
        if self.pending_preview_refresh is not None:
            return
        self.pending_preview_refresh = self.root.after(
            self.PREVIEW_REFRESH_DELAY_MS,
            self._refresh_preview_only,
        )

    def _refresh_preview_only(self) -> None:
        self.pending_preview_refresh = None
        if not self._sync_controls_to_project():
            return
        try:
            hardpoints = hardpoints_from_rows(self.project.hardpoints)
            design_state = solve_two_segment_steering(hardpoints, 0.0)
            state, _outputs = solve_steering_project(self.project, include_limits=False)
            draw_steering_preview(
                self.preview_ax,
                hardpoints,
                design_state,
                state,
                preserve_view=self.preview_has_drawn,
                wheel_radius=self.project.wheel_radius,
                wheel_width=self.project.wheel_width,
            )
            self.preview_has_drawn = True
            self.preview_canvas.draw_idle()
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            self.output_table.set_error(str(exc))

    def _on_controls_changed(self, *_args: object) -> None:
        if not self.updating_controls:
            self.refresh()

    def _on_hardpoints_changed(self) -> None:
        self.pitman_controls.set_rows(self.project.hardpoints)
        self.refresh()

    def _on_pitman_transform_changed(self) -> None:
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self.refresh()

    def restore_default_hardpoints(self) -> None:
        """Restore hardpoints from the latest imported hardpoint snapshot."""
        self.project.hardpoints = copy_hardpoint_rows(self.imported_default_hardpoints)
        self.pending_optimized_hardpoints = None
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self.pitman_controls.set_rows(self.project.hardpoints)
        self.preview_has_drawn = False
        self.refresh()

    def _selected_optimization_variables(self) -> tuple[str, ...]:
        return tuple(
            name
            for name, _label in OPTIMIZATION_VARIABLE_OPTIONS
            if self.opt_variable_vars[name].get()
        )

    def run_optimization(self) -> None:
        """Run steering hardpoint optimization from the optimization tab."""
        if not self._sync_controls_to_project():
            return
        inner_angle = parse_float_entry(self.opt_inner_angle_var.get(), 0.0)
        target_delta = parse_float_entry(self.opt_target_delta_var.get(), 0.0)
        delta_limit = parse_float_entry(self.opt_delta_limit_var.get(), 40.0)
        if not inner_angle.is_valid or not target_delta.is_valid:
            self.optimization_status_var.set("Invalid target input")
            return
        if not delta_limit.is_valid or delta_limit.value <= 0.0:
            self.optimization_status_var.set("Invalid variable limit")
            return
        try:
            result = optimize_steering_hardpoints(
                self.project.hardpoints,
                inner_wheel=self.opt_inner_wheel_var.get(),
                inner_wheel_angle_deg=inner_angle.value,
                target_left_minus_right_deg=target_delta.value,
                variable_names=self._selected_optimization_variables(),
                variable_delta_limit=delta_limit.value,
            )
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            self.optimization_status_var.set(str(exc))
            return
        self.pending_optimized_hardpoints = result.hardpoints
        self.optimization_status_var.set(
            f"Initial error: {result.initial_error_deg:.6g} deg\n"
            f"Final error: {result.final_error_deg:.6g} deg\n"
            f"Actual L-R: {result.actual_left_minus_right_deg:.6g} deg"
        )

    def apply_optimization(self) -> None:
        """Apply the last optimized hardpoints to the current project."""
        if self.pending_optimized_hardpoints is None:
            self.optimization_status_var.set("No optimization result to apply")
            return
        self.project.hardpoints = self.pending_optimized_hardpoints
        self.pending_optimized_hardpoints = None
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self.pitman_controls.set_rows(self.project.hardpoints)
        self.preview_has_drawn = False
        self.refresh()
        self.optimization_status_var.set("Optimization applied")

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
            self.updating_controls = True
            self._sync_input_slider_limits(self.project.input_value)
            self.updating_controls = False
            draw_steering_preview(
                self.preview_ax,
                hardpoints,
                design_state,
                state,
                preserve_view=self.preview_has_drawn,
                wheel_radius=self.project.wheel_radius,
                wheel_width=self.project.wheel_width,
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
