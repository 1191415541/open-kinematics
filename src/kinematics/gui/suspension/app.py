"""Tkinter suspension simulation page."""

from __future__ import annotations

import copy
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from kinematics.gui.common import (
    OptimizationCancelledError,
    RefreshWorkflowMixin,
    parse_float_entry,
    parse_int_entry,
)
from kinematics.gui.suspension.optimization import (
    SUSPENSION_OPTIMIZATION_METRICS,
    SUSPENSION_OPTIMIZATION_SOLVER_MODES,
    SUSPENSION_OPTIMIZATION_TARGET_MODES,
    SUSPENSION_OPTIMIZATION_TRENDS,
    SuspensionOptimizationConfig,
    SuspensionOptimizationProgress,
    SuspensionOptimizationVariableAnalysisResult,
    SuspensionOptimizationPairDeltaConstraint,
    SuspensionOptimizationTarget,
    available_suspension_optimization_variables,
)
from kinematics.gui.steering.widgets import CurveManager
from kinematics.gui.suspension.plotting import (
    SuspensionPreviewRenderer,
    draw_suspension_curve_plot,
    draw_suspension_preview,
)
from kinematics.gui.suspension.widgets import HardpointTable
from kinematics.gui.suspension.workbench import (
    DEFAULT_CURVE_OPTIONS,
    SuspensionCurve,
    SuspensionProject,
    SuspensionSweepResult,
    SuspensionSweepSettings,
    suspension_gui_to_internal_vec3,
    suspension_internal_to_gui_vec3,
    create_default_suspension_project,
    curve_specs_for_plot,
    analyze_suspension_optimization_variables,
    load_suspension_hardpoints_csv,
    load_suspension_project,
    optimize_suspension_hardpoints,
    save_suspension_hardpoints_csv,
    save_suspension_project,
    solve_suspension_project,
    solve_suspension_project_at_travel,
    supported_suspension_type_keys,
)

PROJECT_FILETYPES = [("Kinematics project", "*.okproj.json")]
OPTIMIZATION_VARIABLE_COLOR_PALETTE = (
    "#0f766e",
    "#b45309",
    "#b91c1c",
    "#1d4ed8",
    "#047857",
    "#7c2d12",
    "#4338ca",
    "#be185d",
    "#15803d",
    "#92400e",
    "#0369a1",
    "#9a3412",
)


class SuspensionWorkbenchPage(RefreshWorkflowMixin, ttk.Frame):
    """Suspension sweep page for the unified GUI."""

    PREVIEW_REFRESH_DELAY_MS = 16
    DEFAULT_LEFT_PANE_WIDTH = 370
    WORKSPACE_PREVIEW_WEIGHT = 2
    WORKSPACE_SIDE_WEIGHT = 1

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master)
        self.project = create_default_suspension_project()
        self.imported_default_hardpoints = {
            point_id: position.copy()
            for point_id, position in self.project.hardpoints.items()
        }
        self.project_path: Path | None = None
        self.result: SuspensionSweepResult | None = None
        self.updating_controls = False
        self.preview_has_drawn = False
        self.pending_preview_refresh: str | None = None
        self.preview_renderer = SuspensionPreviewRenderer()
        self.geometry_path_var = tk.StringVar(value="No geometry loaded")
        self.suspension_type_var = tk.StringVar(value=self.project.suspension_type)
        self.travel_slider_var = tk.DoubleVar(value=0.0)
        self.travel_value_var = tk.StringVar(value="0")
        self.start_var = tk.StringVar(value=str(self.project.settings.start))
        self.stop_var = tk.StringVar(value=str(self.project.settings.stop))
        self.steps_var = tk.StringVar(value=str(self.project.settings.steps))
        self.steered_var = tk.BooleanVar(value=self.project.config.steered)
        self.wheelbase_var = tk.StringVar(value=str(self.project.config.wheelbase))
        self.cg_x_var = tk.StringVar(value=str(self.project.config.cg_position[0]))
        self.cg_y_var = tk.StringVar(value=str(self.project.config.cg_position[1]))
        self.cg_z_var = tk.StringVar(value=str(self.project.config.cg_position[2]))
        self.wheel_offset_var = tk.StringVar(
            value=str(self.project.config.wheel.offset)
        )
        self.tire_width_var = tk.StringVar(
            value=str(self.project.config.wheel.tire.section_width)
        )
        self.tire_aspect_var = tk.StringVar(
            value=str(self.project.config.wheel.tire.aspect_ratio)
        )
        self.static_radius_var = tk.StringVar(
            value=str(self.project.config.wheel.tire.static_radius_mm)
        )
        self.optimization_running = False
        self.optimization_queue: queue.Queue[
            tuple[str, object]
        ] | None = None
        self.optimization_thread: threading.Thread | None = None
        self.optimization_cancel_event: threading.Event | None = None
        self.pending_optimized_hardpoints = None
        self.last_optimization_analysis: SuspensionOptimizationVariableAnalysisResult | None = None
        self.opt_variable_limit_var = tk.StringVar(
            value=str(self.project.optimization.variable_delta_limit)
        )
        self.opt_solver_mode_var = tk.StringVar(
            value=self._optimization_solver_mode_label(
                self.project.optimization.solver_mode
            )
        )
        self.opt_variable_vars: dict[str, tk.BooleanVar] = {}
        self.opt_pair_constraint_vars: dict[str, tk.BooleanVar] = {}
        self.opt_target_enabled_vars = {
            metric_name: tk.BooleanVar(value=True)
            for metric_name, _label in SUSPENSION_OPTIMIZATION_METRICS
        }
        self.opt_target_trend_vars = {
            metric_name: tk.StringVar(value="ignore")
            for metric_name, _label in SUSPENSION_OPTIMIZATION_METRICS
        }
        self.opt_target_mode_vars = {
            metric_name: tk.StringVar(value="endpoint_delta")
            for metric_name, _label in SUSPENSION_OPTIMIZATION_METRICS
        }
        self.opt_target_delta_vars = {
            metric_name: tk.StringVar(value="0.0")
            for metric_name, _label in SUSPENSION_OPTIMIZATION_METRICS
        }
        self.opt_target_weight_vars = {
            metric_name: tk.StringVar(value="1.0")
            for metric_name, _label in SUSPENSION_OPTIMIZATION_METRICS
        }
        self.optimization_status_var = tk.StringVar(value="No optimization run")
        self._copyable_optimization_output = "No optimization run"
        self.status_var = tk.StringVar(value="Edit or import suspension geometry")
        self._build_layout()
        self._bind_control_vars()
        self._load_project_to_controls()
        self.refresh()

    def _build_layout(self) -> None:
        self.main_panedwindow = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        self.main_panedwindow.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(self.main_panedwindow, padding=8)
        right = ttk.Frame(self.main_panedwindow, padding=8)
        self.main_panedwindow.add(left, weight=0)
        self.main_panedwindow.add(right, weight=5)

        hardpoint_header = ttk.Frame(left)
        hardpoint_header.pack(fill=tk.X)
        ttk.Label(hardpoint_header, text="3D Hardpoints").pack(side=tk.LEFT)
        ttk.Button(
            hardpoint_header,
            text="Restore Default Hardpoints",
            command=self.restore_default_hardpoints,
        ).pack(side=tk.RIGHT)
        self.hardpoint_table = HardpointTable(left, self._on_hardpoints_changed)
        self.hardpoint_table.pack(fill=tk.BOTH, expand=True, pady=(0, 8))
        left_parameters = ttk.LabelFrame(left, text="Suspension Parameters", padding=8)
        left_parameters.pack(fill=tk.X)
        self._build_parameters(left_parameters)

        workspace = ttk.PanedWindow(right, orient=tk.HORIZONTAL)
        workspace.pack(fill=tk.BOTH, expand=True)
        workspace_left = ttk.Frame(workspace, padding=(0, 0, 8, 0))
        workspace_right = ttk.Frame(workspace, padding=(8, 0, 0, 0))
        workspace.add(workspace_left, weight=self.WORKSPACE_PREVIEW_WEIGHT)
        workspace.add(workspace_right, weight=self.WORKSPACE_SIDE_WEIGHT)

        controls = ttk.LabelFrame(workspace_left, text="Simulation Input", padding=8)
        controls.pack(fill=tk.X)
        self._build_controls(controls)

        preview_area = ttk.Frame(workspace_left)
        preview_area.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self._build_preview(preview_area)
        self._build_side_panel(workspace_right)
        self.after_idle(self._apply_default_layout)

    def _apply_default_layout(self) -> None:
        try:
            self.main_panedwindow.sashpos(0, self.DEFAULT_LEFT_PANE_WIDTH)
        except tk.TclError:
            return

    def _build_controls(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        refresh_commit_entries: list[ttk.Entry] = []
        ttk.Label(parent, text="Suspension Type").grid(row=0, column=0, sticky="w")
        self.suspension_type_combo = ttk.Combobox(
            parent,
            textvariable=self.suspension_type_var,
            values=supported_suspension_type_keys(),
            state="readonly",
            width=18,
        )
        self.suspension_type_combo.grid(row=0, column=1, sticky="w", padx=(6, 12))
        self.suspension_type_combo.bind("<<ComboboxSelected>>", self._on_type_changed)
        ttk.Label(parent, textvariable=self.geometry_path_var).grid(
            row=0,
            column=2,
            columnspan=5,
            sticky="ew",
            padx=(8, 0),
        )

        fields = (
            ("Start [mm]", self.start_var),
            ("Stop [mm]", self.stop_var),
            ("Steps", self.steps_var),
        )
        for index, (label, var) in enumerate(fields):
            label_col = index * 2
            ttk.Label(parent, text=label).grid(
                row=1,
                column=label_col,
                sticky="w",
                pady=(8, 0),
            )
            entry = ttk.Entry(parent, textvariable=var, width=10)
            entry.grid(
                row=1,
                column=label_col + 1,
                sticky="w",
                padx=(6, 16),
                pady=(8, 0),
            )
            refresh_commit_entries.append(entry)

        ttk.Label(parent, text="Travel [mm]").grid(row=2, column=0, sticky="w")
        self.travel_slider = ttk.Scale(
            parent,
            variable=self.travel_slider_var,
            command=self._on_travel_slider_changed,
            length=220,
        )
        self.travel_slider.grid(
            row=2,
            column=1,
            columnspan=3,
            sticky="ew",
            padx=(6, 16),
            pady=(8, 0),
        )
        self.travel_slider.bind("<ButtonRelease-1>", self._on_travel_slider_released)
        travel_entry = ttk.Entry(parent, textvariable=self.travel_value_var, width=10)
        travel_entry.grid(
            row=2,
            column=4,
            sticky="w",
            padx=(6, 12),
            pady=(8, 0),
        )
        refresh_commit_entries.append(travel_entry)
        self.bind_entry_commit_refresh(refresh_commit_entries)
        ttk.Label(parent, textvariable=self.status_var).grid(
            row=3,
            column=0,
            columnspan=8,
            sticky="ew",
            pady=(8, 0),
        )

    def _build_parameters(self, parent: ttk.Frame) -> None:
        fields = (
            ("Wheelbase", self.wheelbase_var),
            ("CG X rearward", self.cg_x_var),
            ("CG Y rightward", self.cg_y_var),
            ("CG Z upward", self.cg_z_var),
            ("Wheel offset", self.wheel_offset_var),
            ("Tire width", self.tire_width_var),
            ("Aspect ratio", self.tire_aspect_var),
            ("Static radius [mm]", self.static_radius_var),
        )
        ttk.Checkbutton(parent, text="Steered", variable=self.steered_var).grid(
            row=0,
            column=0,
            sticky="w",
        )
        refresh_commit_entries: list[ttk.Entry] = []
        for index, (label, var) in enumerate(fields, start=1):
            label_col = ((index - 1) % 2) * 2
            row = (index - 1) // 2 + 1
            ttk.Label(parent, text=label).grid(row=row, column=label_col, sticky="w")
            entry = ttk.Entry(parent, textvariable=var, width=8)
            entry.grid(
                row=row,
                column=label_col + 1,
                sticky="w",
                padx=(6, 12),
                pady=2,
            )
            refresh_commit_entries.append(entry)
        self.bind_entry_commit_refresh(refresh_commit_entries)

    def _build_preview(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True)
        self.preview_fig = Figure(figsize=(6, 5), dpi=100)
        self.preview_ax = self.preview_fig.add_subplot(111, projection="3d")
        self.preview_canvas = FigureCanvasTkAgg(self.preview_fig, master=frame)
        toolbar_frame = ttk.Frame(frame)
        toolbar_frame.pack(fill=tk.X)
        self.preview_toolbar = NavigationToolbar2Tk(
            self.preview_canvas,
            toolbar_frame,
            pack_toolbar=False,
        )
        self.preview_toolbar.update()
        self.preview_toolbar.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.preview_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_side_panel(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True)
        notebook = ttk.Notebook(frame)
        notebook.pack(fill=tk.BOTH, expand=True)

        output_tab = ttk.Frame(notebook)
        notebook.add(output_tab, text="Outputs")
        table_frame = ttk.LabelFrame(output_tab, text="Outputs", padding=6)
        table_frame.pack(fill=tk.BOTH, expand=True)
        self.output_table = ttk.Treeview(
            table_frame,
            columns=("name", "value"),
            show="headings",
            height=8,
        )
        self.output_table.heading("name", text="output")
        self.output_table.heading("value", text="value")
        self.output_table.column("name", width=180)
        self.output_table.column("value", width=120, anchor="e")
        self.output_table.pack(fill=tk.BOTH, expand=True)

        curve_controls = ttk.LabelFrame(output_tab, text="Curves", padding=6)
        curve_controls.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.curve_manager = CurveManager(
            curve_controls,
            DEFAULT_CURVE_OPTIONS,
            self.refresh_curves,
            curve_factory=SuspensionCurve,
        )
        self.curve_manager.pack(fill=tk.X)

        self.curve_fig = Figure(figsize=(5, 3), dpi=100)
        self.curve_ax = self.curve_fig.add_subplot(111)
        self.curve_canvas = FigureCanvasTkAgg(self.curve_fig, master=curve_controls)
        self.curve_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, pady=(6, 0))

        optimization_tab = ttk.Frame(notebook, padding=6)
        notebook.add(optimization_tab, text="Optimization")
        self._build_optimization_tab(optimization_tab)

    def _build_optimization_tab(self, parent: ttk.Frame) -> None:
        parent.rowconfigure(0, weight=1)
        parent.columnconfigure(0, weight=1)

        scroll_host = ttk.Frame(parent)
        scroll_host.grid(row=0, column=0, sticky="nsew")
        scroll_host.rowconfigure(0, weight=1)
        scroll_host.columnconfigure(0, weight=1)

        canvas = tk.Canvas(scroll_host, highlightthickness=0, borderwidth=0)
        scrollbar = ttk.Scrollbar(
            scroll_host,
            orient=tk.VERTICAL,
            command=canvas.yview,
        )
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        content = ttk.Frame(canvas)
        content_window = canvas.create_window((0, 0), window=content, anchor="nw")

        def _sync_scrollregion(_event: tk.Event) -> None:
            bbox = canvas.bbox("all")
            if bbox is not None:
                canvas.configure(scrollregion=bbox)

        def _sync_content_width(event: tk.Event) -> None:
            canvas.itemconfigure(content_window, width=event.width)

        content.bind("<Configure>", _sync_scrollregion)
        canvas.bind("<Configure>", _sync_content_width)
        self._build_optimization_content(content)

    def _build_optimization_content(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(1, weight=1)
        parent.rowconfigure(7, weight=1)
        optimization_commit_entries: list[ttk.Entry] = []
        ttk.Label(parent, text="Variable limit [mm]").grid(
            row=0,
            column=0,
            sticky="w",
        )
        variable_limit_entry = ttk.Entry(
            parent,
            textvariable=self.opt_variable_limit_var,
            width=12,
        )
        variable_limit_entry.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=(6, 0),
            pady=1,
        )
        optimization_commit_entries.append(variable_limit_entry)
        ttk.Label(parent, text="Optimization Method").grid(
            row=1,
            column=0,
            sticky="w",
            pady=(6, 0),
        )
        self.opt_solver_mode_combo = ttk.Combobox(
            parent,
            textvariable=self.opt_solver_mode_var,
            values=[label for _mode, label in SUSPENSION_OPTIMIZATION_SOLVER_MODES],
            state="readonly",
            width=24,
        )
        self.opt_solver_mode_combo.grid(
            row=1,
            column=1,
            sticky="ew",
            padx=(6, 0),
            pady=(6, 0),
        )
        variables = ttk.LabelFrame(parent, text="Variables", padding=4)
        variables.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        variables.columnconfigure(0, weight=1)
        variable_actions = ttk.Frame(variables)
        variable_actions.grid(row=0, column=0, sticky="ew", pady=(0, 4))
        for column in range(4):
            variable_actions.columnconfigure(column, weight=1)
        ttk.Button(
            variable_actions,
            text="Select Recommended",
            command=self._select_recommended_optimization_variables,
        ).grid(row=0, column=0, sticky="ew", padx=(0, 4))
        ttk.Button(
            variable_actions,
            text="Select All",
            command=self._select_all_optimization_variables,
        ).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Button(
            variable_actions,
            text="Select None",
            command=self._clear_optimization_variable_selection,
        ).grid(row=0, column=2, sticky="ew", padx=4)
        ttk.Button(
            variable_actions,
            text="Invert",
            command=self._invert_optimization_variable_selection,
        ).grid(row=0, column=3, sticky="ew", padx=(4, 0))
        variable_list = ttk.Frame(variables)
        variable_list.grid(row=1, column=0, sticky="ew")
        variable_list.columnconfigure(0, weight=1)
        self.opt_variables_frame = variables
        self.opt_variable_list_frame = variable_list

        pair_constraints = ttk.LabelFrame(parent, text="Pair constraints", padding=4)
        pair_constraints.grid(row=3, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        self.opt_pair_constraints_frame = pair_constraints

        targets = ttk.LabelFrame(parent, text="Targets", padding=4)
        targets.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        ttk.Label(targets, text="Metric").grid(row=0, column=0, sticky="w")
        ttk.Label(targets, text="Enable").grid(row=0, column=1, sticky="w")
        ttk.Label(targets, text="Trend").grid(row=0, column=2, sticky="w")
        ttk.Label(targets, text="Mode").grid(row=0, column=3, sticky="w")
        ttk.Label(targets, text="Target [deg]").grid(row=0, column=4, sticky="w")
        ttk.Label(targets, text="Weight").grid(row=0, column=5, sticky="w")
        for row_index, (metric_name, label) in enumerate(
            SUSPENSION_OPTIMIZATION_METRICS,
            start=1,
        ):
            ttk.Label(targets, text=label).grid(row=row_index, column=0, sticky="w")
            target_enabled = ttk.Checkbutton(
                targets,
                variable=self.opt_target_enabled_vars[metric_name],
            )
            target_enabled.grid(row=row_index, column=1, sticky="w")
            trend_combo = ttk.Combobox(
                targets,
                textvariable=self.opt_target_trend_vars[metric_name],
                values=SUSPENSION_OPTIMIZATION_TRENDS,
                state="readonly",
                width=10,
            )
            trend_combo.grid(
                row=row_index,
                column=2,
                sticky="w",
                padx=(6, 6),
                pady=2,
            )
            mode_combo = ttk.Combobox(
                targets,
                textvariable=self.opt_target_mode_vars[metric_name],
                values=[label for _mode, label in SUSPENSION_OPTIMIZATION_TARGET_MODES],
                state="readonly",
                width=18,
            )
            mode_combo.grid(
                row=row_index,
                column=3,
                sticky="w",
                padx=(0, 6),
                pady=2,
            )
            delta_entry = ttk.Entry(
                targets,
                textvariable=self.opt_target_delta_vars[metric_name],
                width=12,
            )
            delta_entry.grid(row=row_index, column=4, sticky="w", pady=2)
            weight_entry = ttk.Entry(
                targets,
                textvariable=self.opt_target_weight_vars[metric_name],
                width=8,
            )
            weight_entry.grid(
                row=row_index,
                column=5,
                sticky="w",
                padx=(6, 0),
                pady=2,
            )
            target_enabled.configure(command=self._on_optimization_controls_changed)
            trend_combo.bind(
                "<<ComboboxSelected>>",
                self._on_optimization_controls_changed,
            )
            mode_combo.bind(
                "<<ComboboxSelected>>",
                self._on_optimization_controls_changed,
            )
            optimization_commit_entries.extend((delta_entry, weight_entry))

        self.opt_solver_mode_combo.bind(
            "<<ComboboxSelected>>",
            self._on_optimization_controls_changed,
        )
        self.bind_entry_commit_callback(
            optimization_commit_entries,
            callback=self._on_optimization_controls_changed,
        )

        buttons = ttk.Frame(parent)
        buttons.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        buttons.columnconfigure(0, weight=1)
        buttons.columnconfigure(1, weight=1)
        buttons.columnconfigure(2, weight=1)
        buttons.columnconfigure(3, weight=1)
        self.analyze_variables_button = ttk.Button(
            buttons,
            text="Analyze Variables",
            command=self.run_optimization_analysis,
        )
        self.analyze_variables_button.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 4),
        )
        self.optimize_button = ttk.Button(
            buttons,
            text="Optimize",
            command=self.run_optimization,
        )
        self.optimize_button.grid(
            row=0,
            column=1,
            sticky="ew",
            padx=4,
        )
        self.apply_optimization_button = ttk.Button(
            buttons,
            text="Apply",
            command=self.apply_optimization,
        )
        self.apply_optimization_button.grid(
            row=0,
            column=2,
            sticky="ew",
            padx=(4, 0),
        )
        self.stop_optimization_button = ttk.Button(
            buttons,
            text="Stop",
            command=self.stop_optimization,
            state=tk.DISABLED,
        )
        self.stop_optimization_button.grid(
            row=0,
            column=3,
            sticky="ew",
            padx=(4, 0),
        )
        self.optimization_progressbar = ttk.Progressbar(
            parent,
            mode="indeterminate",
        )
        self.optimization_progressbar.grid(
            row=6,
            column=0,
            columnspan=2,
            sticky="ew",
            pady=(6, 0),
        )
        output_frame = ttk.LabelFrame(parent, text="Results", padding=4)
        output_frame.grid(
            row=7,
            column=0,
            columnspan=2,
            sticky="nsew",
            pady=(6, 0),
        )
        output_frame.rowconfigure(0, weight=1)
        output_frame.columnconfigure(0, weight=1)
        self.optimization_output = tk.Text(
            output_frame,
            height=14,
            wrap=tk.WORD,
            state=tk.DISABLED,
            relief=tk.FLAT,
            borderwidth=0,
            padx=6,
            pady=6,
        )
        output_scrollbar = ttk.Scrollbar(
            output_frame,
            orient=tk.VERTICAL,
            command=self.optimization_output.yview,
        )
        self.optimization_output.configure(yscrollcommand=output_scrollbar.set)
        self.optimization_output.grid(row=0, column=0, sticky="nsew")
        output_scrollbar.grid(row=0, column=1, sticky="ns")

        output_actions = ttk.Frame(output_frame)
        output_actions.grid(row=1, column=0, columnspan=2, sticky="e", pady=(6, 0))
        ttk.Button(
            output_actions,
            text="Copy Output",
            command=self._copy_optimization_output,
        ).pack(side=tk.RIGHT)

        self.optimization_output.bind("<Control-c>", self._copy_optimization_output)
        self.optimization_output.bind("<Command-c>", self._copy_optimization_output)
        self.optimization_output.bind("<<Copy>>", self._copy_optimization_output)
        self._configure_optimization_output_tags()
        self._render_optimization_output(
            [{"kind": "summary", "text": self.optimization_status_var.get()}]
        )

    def open_geometry(self) -> None:
        path = filedialog.askopenfilename(
            filetypes=[("YAML", "*.yaml *.yml"), ("All files", "*.*")]
        )
        if path:
            self.load_geometry(Path(path))

    def open_project(self) -> None:
        path = filedialog.askopenfilename(filetypes=PROJECT_FILETYPES)
        if not path:
            return
        self.load_geometry(Path(path))

    def import_hardpoints(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            self.project.hardpoints = load_suspension_hardpoints_csv(path)
            self.imported_default_hardpoints = {
                point_id: position.copy()
                for point_id, position in self.project.hardpoints.items()
            }
            self.hardpoint_table.set_hardpoints(self.project.hardpoints)
            self.result = None
            self.preview_has_drawn = False
            self.preview_renderer.reset()
            self._reset_optimization_analysis()
            self.status_var.set("Hardpoints imported")
            self.refresh()
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            messagebox.showerror("Import failed", str(exc))

    def export_hardpoints(self) -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
        )
        if not path:
            return
        try:
            save_suspension_hardpoints_csv(self.project.hardpoints, path)
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            messagebox.showerror("Export failed", str(exc))

    def save_project(self) -> None:
        if self.project_path is None:
            self.save_project_as()
            return
        try:
            self._sync_controls_to_project()
            save_suspension_project(self.project, self.project_path)
            self.status_var.set("Project saved")
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            messagebox.showerror("Save failed", str(exc))

    def save_project_as(self) -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".okproj.json",
            filetypes=PROJECT_FILETYPES,
        )
        if not path:
            return
        self.project_path = Path(path)
        self.save_project()

    def load_geometry(self, path: Path) -> None:
        try:
            self.project = load_suspension_project(path)
            self.imported_default_hardpoints = {
                point_id: position.copy()
                for point_id, position in self.project.hardpoints.items()
            }
            if path.suffix == ".json":
                self.project_path = path
            else:
                self.project_path = None
            self.geometry_path_var.set(str(path))
            self.result = None
            self.preview_has_drawn = False
            self.preview_renderer.reset()
            self._reset_optimization_analysis()
            self._load_project_to_controls()
            self.status_var.set("Geometry loaded")
            self.refresh()
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            self.status_var.set(str(exc))

    def run_sweep(self) -> None:
        self.refresh()

    def _draw_design_preview(self) -> None:
        suspension = self.project.build_suspension()
        draw_suspension_preview(
            self.preview_ax,
            suspension,
            suspension.initial_state(),
            preserve_view=False,
            renderer=self.preview_renderer,
            preview_mode=False,
        )
        self.preview_has_drawn = True
        self.preview_toolbar.update()
        self.preview_canvas.draw_idle()

    def _draw_result(self) -> None:
        if self.result is None:
            return
        self._draw_result_index(-1)
        self.refresh_curves()

    def _draw_result_index(self, index: int, *, update_outputs: bool = True) -> None:
        if self.result is None:
            return
        suspension = self.project.build_suspension()
        draw_suspension_preview(
            self.preview_ax,
            suspension,
            self.result.states[index],
            preserve_view=self.preview_has_drawn,
            renderer=self.preview_renderer,
            preview_mode=not update_outputs,
        )
        self.preview_has_drawn = True
        if update_outputs:
            self._set_output_rows(self.result.rows[index])
        travel = float(self.result.rows[index]["wheel_travel_mm"] or 0.0)
        self._sync_travel_controls(travel)
        self.preview_toolbar.update()
        self.preview_canvas.draw_idle()

    def _set_output_rows(self, row: dict[str, float | bool | None]) -> None:
        for item in self.output_table.get_children():
            self.output_table.delete(item)
        for name, value in row.items():
            if isinstance(value, float):
                display = f"{value:.6g}"
            else:
                display = str(value)
            self.output_table.insert("", "end", values=(name, display))

    def refresh_curves(self) -> None:
        """Refresh managed suspension curve plots."""
        def draw_curves() -> None:
            if not self._sync_controls_to_project():
                return
            sweep = solve_suspension_project(self.project)
            curves = curve_specs_for_plot(
                self.project.curves,
                self.curve_manager.x_var.get(),
                self.curve_manager.y_var.get(),
                self.curve_manager.label_var.get(),
            )
            draw_suspension_curve_plot(self.curve_ax, sweep.rows, curves)
            self.curve_canvas.draw_idle()
            self.status_var.set(f"Solved {len(sweep.rows)} steps")
        self.run_guarded(
            action=draw_curves,
            on_error=lambda exc: self.status_var.set(str(exc)),
        )

    def run_optimization(self) -> None:
        """Run suspension hardpoint optimization from the optimization tab."""
        if not self._sync_controls_to_project():
            return
        if self.optimization_running:
            self._show_optimization_message(
                "Optimization already running",
                heading="Optimization Busy",
                kind="secondary",
            )
            return
        snapshot = copy.deepcopy(self.project)
        self.pending_optimized_hardpoints = None
        self.optimization_queue = queue.Queue()
        self.optimization_cancel_event = threading.Event()
        self.optimization_thread = threading.Thread(
            target=self._optimization_worker,
            args=(snapshot,),
            daemon=True,
        )
        self._set_optimization_running(True)
        self._show_optimization_message(
            "Waiting for solver progress update",
            heading="Optimization Running",
            kind="progress",
        )
        self.optimization_thread.start()
        self.after(100, self._poll_optimization_progress)

    def run_optimization_analysis(self) -> None:
        """Analyze currently selected optimization variables before solving."""
        if not self._sync_controls_to_project():
            return
        if self.optimization_running:
            self._show_optimization_message(
                "Optimization already running",
                heading="Optimization Busy",
                kind="secondary",
            )
            return
        snapshot = copy.deepcopy(self.project)
        self.optimization_queue = queue.Queue()
        self.optimization_cancel_event = threading.Event()
        self.optimization_thread = threading.Thread(
            target=self._optimization_analysis_worker,
            args=(snapshot,),
            daemon=True,
        )
        self._set_optimization_running(True)
        self._show_optimization_message(
            "Preparing constrained global sensitivity analysis",
            heading="Variable Analysis Running",
            kind="progress",
        )
        self.optimization_thread.start()
        self.after(100, self._poll_optimization_progress)

    def apply_optimization(self) -> None:
        """Apply the last optimized hardpoints to the current suspension project."""
        if self.pending_optimized_hardpoints is None:
            self._show_optimization_message(
                "No optimization result to apply",
                heading="Optimization Output",
                kind="secondary",
            )
            return
        self.project.hardpoints = {
            point_id: position.copy()
            for point_id, position in self.pending_optimized_hardpoints.items()
        }
        self.pending_optimized_hardpoints = None
        self.result = None
        self.preview_has_drawn = False
        self.preview_renderer.reset()
        self._reset_optimization_analysis()
        self.hardpoint_table.set_hardpoints(self.project.hardpoints)
        self._sync_available_optimization_variables()
        self.refresh()
        self._show_optimization_message(
            "Optimized hardpoints applied to the current suspension project",
            heading="Optimization Applied",
            kind="recommended",
        )

    def stop_optimization(self) -> None:
        """Request cooperative cancellation for the current optimization task."""
        if (
            not self.optimization_running
            or self.optimization_cancel_event is None
            or self.optimization_cancel_event.is_set()
        ):
            return
        self.optimization_cancel_event.set()
        self._show_optimization_message(
            "Stopping current optimization task",
            heading="Optimization Stopping",
            kind="secondary",
        )

    def _optimization_analysis_worker(self, project: SuspensionProject) -> None:
        assert self.optimization_queue is not None
        try:
            result = analyze_suspension_optimization_variables(
                project,
                targets=project.optimization.targets,
                variable_names=tuple(project.optimization.variable_names),
                variable_delta_limit=project.optimization.variable_delta_limit,
                solver_mode=project.optimization.solver_mode,
                pair_delta_constraints=project.optimization.pair_delta_constraints,
                cancel_event=self.optimization_cancel_event,
            )
        except Exception as exc:  # noqa: BLE001 - surface in GUI polling loop.
            self.optimization_queue.put(("error", exc))
            return
        self.optimization_queue.put(("analysis_result", result))

    def _optimization_worker(self, project: SuspensionProject) -> None:
        assert self.optimization_queue is not None
        try:
            result = optimize_suspension_hardpoints(
                project,
                targets=project.optimization.targets,
                variable_names=tuple(project.optimization.variable_names),
                variable_delta_limit=project.optimization.variable_delta_limit,
                solver_mode=project.optimization.solver_mode,
                pair_delta_constraints=project.optimization.pair_delta_constraints,
                progress_callback=lambda progress: self.optimization_queue.put(
                    ("progress", progress)
                ),
                cancel_event=self.optimization_cancel_event,
            )
        except Exception as exc:  # noqa: BLE001 - surface in GUI polling loop.
            self.optimization_queue.put(("error", exc))
            return
        self.optimization_queue.put(("result", result))

    def _poll_optimization_progress(self) -> None:
        if self.optimization_queue is None:
            return
        finished = False
        while True:
            try:
                kind, payload = self.optimization_queue.get_nowait()
            except queue.Empty:
                break
            if kind == "progress":
                progress = payload
                if isinstance(progress, SuspensionOptimizationProgress):
                    self._render_optimization_output(
                        self._format_optimization_progress(progress)
                    )
            elif kind == "result":
                self.pending_optimized_hardpoints = payload.hardpoints
                self._render_optimization_output(
                    self._format_optimization_result(payload)
                )
                finished = True
            elif kind == "analysis_result":
                self.last_optimization_analysis = payload
                self._render_optimization_output(
                    self._format_optimization_analysis(payload)
                )
                finished = True
            elif kind == "error":
                self.pending_optimized_hardpoints = None
                if isinstance(payload, OptimizationCancelledError):
                    self._show_optimization_message(
                        "Stopped",
                        heading="Optimization Stopped",
                        kind="secondary",
                    )
                else:
                    self._show_optimization_message(
                        str(payload),
                        heading="Optimization Error",
                        kind="error",
                    )
                finished = True
        if finished:
            self._set_optimization_running(False)
            self.optimization_queue = None
            self.optimization_thread = None
            self.optimization_cancel_event = None
            return
        self.after(100, self._poll_optimization_progress)

    def _set_optimization_running(self, running: bool) -> None:
        self.optimization_running = running
        state = tk.DISABLED if running else tk.NORMAL
        self.analyze_variables_button.configure(state=state)
        self.optimize_button.configure(state=state)
        self.apply_optimization_button.configure(
            state=tk.DISABLED if running else tk.NORMAL
        )
        self.stop_optimization_button.configure(
            state=tk.NORMAL if running else tk.DISABLED
        )
        if running:
            self.optimization_progressbar.start(10)
        else:
            self.optimization_progressbar.stop()

    def _bind_control_vars(self) -> None:
        self.bind_control_var_traces(
            (
                self.steered_var,
            ),
            self._on_controls_changed,
        )

    def _load_project_to_controls(self) -> None:
        self.updating_controls = True
        self.suspension_type_var.set(self.project.suspension_type)
        self.hardpoint_table.set_hardpoints(self.project.hardpoints)
        cfg = self.project.config
        cg_position = suspension_internal_to_gui_vec3(cfg.cg_position)
        self.steered_var.set(cfg.steered)
        self.wheelbase_var.set(str(cfg.wheelbase))
        self.cg_x_var.set(str(cg_position[0]))
        self.cg_y_var.set(str(cg_position[1]))
        self.cg_z_var.set(str(cg_position[2]))
        self.wheel_offset_var.set(str(cfg.wheel.offset))
        self.tire_width_var.set(str(cfg.wheel.tire.section_width))
        self.tire_aspect_var.set(str(cfg.wheel.tire.aspect_ratio))
        self.static_radius_var.set(str(cfg.wheel.tire.static_radius_mm))
        self.start_var.set(str(self.project.settings.start))
        self.stop_var.set(str(self.project.settings.stop))
        self.steps_var.set(str(self.project.settings.steps))
        self._load_optimization_to_controls()
        self._sync_travel_slider_limits()
        self._sync_travel_controls(0.0)
        self.curve_manager.set_curves(self.project.curves)
        self.updating_controls = False

    def _sync_controls_to_project(self) -> bool:
        from kinematics.suspensions.config.settings import (
            SuspensionConfig,
            TireConfig,
            WheelConfig,
        )

        current_cfg = self.project.config
        current_settings = self.project.settings
        parsed_values = {
            "wheel_offset": parse_float_entry(
                self.wheel_offset_var.get(),
                float(current_cfg.wheel.offset),
            ),
            "tire_aspect_ratio": parse_float_entry(
                self.tire_aspect_var.get(),
                float(current_cfg.wheel.tire.aspect_ratio),
            ),
            "tire_section_width": parse_float_entry(
                self.tire_width_var.get(),
                float(current_cfg.wheel.tire.section_width),
            ),
            "static_radius_mm": parse_float_entry(
                self.static_radius_var.get(),
                float(current_cfg.wheel.tire.static_radius_mm),
            ),
            "cg_x": parse_float_entry(
                self.cg_x_var.get(),
                float(current_cfg.cg_position[0]),
            ),
            "cg_y": parse_float_entry(
                self.cg_y_var.get(),
                float(current_cfg.cg_position[1]),
            ),
            "cg_z": parse_float_entry(
                self.cg_z_var.get(),
                float(current_cfg.cg_position[2]),
            ),
            "wheelbase": parse_float_entry(
                self.wheelbase_var.get(),
                float(current_cfg.wheelbase),
            ),
            "start": parse_float_entry(
                self.start_var.get(),
                float(current_settings.start),
            ),
            "stop": parse_float_entry(
                self.stop_var.get(),
                float(current_settings.stop),
            ),
        }
        parsed_steps = parse_int_entry(self.steps_var.get(), int(current_settings.steps))

        for name, parsed in parsed_values.items():
            if not parsed.is_valid:
                self.status_var.set(f"Invalid numeric input: {name}")
                return False
            if not parsed.is_complete:
                return False
        if not parsed_steps.is_valid:
            self.status_var.set("Invalid numeric input: steps")
            return False
        if not parsed_steps.is_complete:
            return False
        if parsed_steps.value < 2:
            self.status_var.set("Invalid numeric input: steps")
            return False

        optimization = self._optimization_from_controls(self.project.optimization)
        if optimization is None:
            return False
        self.project.suspension_type = self.suspension_type_var.get()
        cg_position = suspension_gui_to_internal_vec3(
            (
                float(parsed_values["cg_x"].value),
                float(parsed_values["cg_y"].value),
                float(parsed_values["cg_z"].value),
            )
        )
        self.project.config = SuspensionConfig(
            steered=self.steered_var.get(),
            wheel=WheelConfig(
                offset=float(parsed_values["wheel_offset"].value),
                tire=TireConfig(
                    aspect_ratio=float(parsed_values["tire_aspect_ratio"].value),
                    section_width=float(parsed_values["tire_section_width"].value),
                    static_radius_mm=float(parsed_values["static_radius_mm"].value),
                ),
            ),
            cg_position=(
                float(cg_position[0]),
                float(cg_position[1]),
                float(cg_position[2]),
            ),
            wheelbase=float(parsed_values["wheelbase"].value),
        )
        self.project.settings = SuspensionSweepSettings(
            start=float(parsed_values["start"].value),
            stop=float(parsed_values["stop"].value),
            steps=int(parsed_steps.value),
        )
        self.project.curves = self.curve_manager.curves
        self.project.optimization = optimization
        return True

    def _load_optimization_to_controls(self) -> None:
        optimization = self.project.optimization
        self.opt_variable_limit_var.set(str(optimization.variable_delta_limit))
        self.opt_solver_mode_var.set(
            self._optimization_solver_mode_label(optimization.solver_mode)
        )
        targets_by_metric = {
            target.metric_name: target for target in optimization.targets
        }
        for metric_name, _label in SUSPENSION_OPTIMIZATION_METRICS:
            target = targets_by_metric.get(
                metric_name,
                SuspensionOptimizationTarget(metric_name=metric_name),
            )
            self.opt_target_enabled_vars[metric_name].set(target.enabled)
            self.opt_target_trend_vars[metric_name].set(target.trend)
            self.opt_target_mode_vars[metric_name].set(
                self._optimization_mode_label(target.target_mode)
            )
            self.opt_target_delta_vars[metric_name].set(f"{target.target_delta:.6g}")
            self.opt_target_weight_vars[metric_name].set(f"{target.weight:.6g}")
        self._sync_available_optimization_variables()
        self._sync_available_optimization_pair_constraints()

    def _optimization_from_controls(
        self,
        current: SuspensionOptimizationConfig,
    ) -> SuspensionOptimizationConfig | None:
        parsed_limit = parse_float_entry(
            self.opt_variable_limit_var.get(),
            float(current.variable_delta_limit),
        )
        if not parsed_limit.is_valid or parsed_limit.value <= 0.0:
            self._show_optimization_message(
                "Invalid optimization variable limit",
                heading="Optimization Input Error",
                kind="error",
            )
            return None
        if not parsed_limit.is_complete:
            return None
        variable_names = tuple(
            name for name, variable in self.opt_variable_vars.items() if variable.get()
        )
        if not variable_names:
            self._show_optimization_message(
                "At least one optimization variable is required",
                heading="Optimization Input Error",
                kind="error",
            )
            return None

        pair_delta_constraints: list[SuspensionOptimizationPairDeltaConstraint] = []
        for constraint in current.pair_delta_constraints:
            constraint_var = self.opt_pair_constraint_vars.get(constraint.key())
            pair_delta_constraints.append(
                SuspensionOptimizationPairDeltaConstraint(
                    point_a=constraint.point_a,
                    point_b=constraint.point_b,
                    label=constraint.label,
                    enabled=(
                        constraint_var.get()
                        if constraint_var is not None
                        else constraint.enabled
                    ),
                    axes=constraint.axes,
                )
            )

        targets: list[SuspensionOptimizationTarget] = []
        for metric_name, _label in SUSPENSION_OPTIMIZATION_METRICS:
            parsed_delta = parse_float_entry(
                self.opt_target_delta_vars[metric_name].get(),
                0.0,
            )
            if not parsed_delta.is_valid:
                self._show_optimization_message(
                    f"Invalid optimization target: {metric_name}",
                    heading="Optimization Input Error",
                    kind="error",
                )
                return None
            if not parsed_delta.is_complete:
                return None
            parsed_weight = parse_float_entry(
                self.opt_target_weight_vars[metric_name].get(),
                1.0,
            )
            if not parsed_weight.is_valid or parsed_weight.value <= 0.0:
                self._show_optimization_message(
                    f"Invalid optimization target weight: {metric_name}",
                    heading="Optimization Input Error",
                    kind="error",
                )
                return None
            if not parsed_weight.is_complete:
                return None
            targets.append(
                SuspensionOptimizationTarget(
                    metric_name=metric_name,
                    target_delta=float(parsed_delta.value),
                    trend=self.opt_target_trend_vars[metric_name].get(),
                    target_mode=self._optimization_mode_key(
                        self.opt_target_mode_vars[metric_name].get()
                    ),
                    enabled=self.opt_target_enabled_vars[metric_name].get(),
                    weight=float(parsed_weight.value),
                )
            )
        return SuspensionOptimizationConfig(
            variable_delta_limit=float(parsed_limit.value),
            solver_mode=self._optimization_solver_mode_key(
                self.opt_solver_mode_var.get()
            ),
            variable_names=list(variable_names),
            targets=targets,
            pair_delta_constraints=pair_delta_constraints,
        )

    def _sync_available_optimization_variables(self) -> None:
        variables = available_suspension_optimization_variables(self.project.hardpoints)
        selected = set(self.project.optimization.variable_names)
        self.opt_variable_vars = {
            name: tk.BooleanVar(value=name in selected)
            for name in variables
        }
        variable_list_frame = getattr(self, "opt_variable_list_frame", self.opt_variables_frame)
        for child in variable_list_frame.winfo_children():
            child.destroy()
        self._configure_optimization_variable_styles(variables)
        for index, name in enumerate(variables):
            ttk.Checkbutton(
                variable_list_frame,
                text=name,
                variable=self.opt_variable_vars[name],
                style=self._optimization_variable_style_name(name),
                command=self._store_selected_optimization_variables,
            ).grid(
                row=index,
                column=0,
                sticky="w",
                padx=(0, 8),
                pady=1,
            )

    def _reset_optimization_analysis(self) -> None:
        self.last_optimization_analysis = None

    def _store_selected_optimization_variables(self) -> None:
        project = getattr(self, "project", None)
        if project is None:
            return
        project.optimization.variable_names = [
            name for name, variable in self.opt_variable_vars.items() if variable.get()
        ]
        self._reset_optimization_analysis()

    def _set_optimization_variable_selection(self, selected_names: set[str]) -> None:
        for name, variable in self.opt_variable_vars.items():
            variable.set(name in selected_names)
        self._store_selected_optimization_variables()

    def _select_recommended_optimization_variables(self) -> None:
        analysis = self.last_optimization_analysis
        if analysis is None:
            self._show_optimization_message(
                "Run Analyze Variables before selecting recommended variables",
                heading="Variable Selection",
                kind="secondary",
            )
            return
        recommended = {
            item.variable_name
            for item in analysis.items
            if getattr(item, "recommendation", "") == "recommended"
        }
        if not recommended:
            self._show_optimization_message(
                "No recommended variables available for the current analysis",
                heading="Variable Selection",
                kind="secondary",
            )
            return
        available = set(self.opt_variable_vars)
        selected = recommended & available
        self._set_optimization_variable_selection(selected)
        self._show_optimization_message(
            "Selected recommended variables: " + ", ".join(sorted(selected)),
            heading="Variable Selection",
            kind="recommended",
        )

    def _select_all_optimization_variables(self) -> None:
        self._set_optimization_variable_selection(set(self.opt_variable_vars))

    def _clear_optimization_variable_selection(self) -> None:
        self._set_optimization_variable_selection(set())

    def _invert_optimization_variable_selection(self) -> None:
        updated: set[str] = set()
        for name, variable in self.opt_variable_vars.items():
            variable.set(not variable.get())
            if variable.get():
                updated.add(name)
        self._store_selected_optimization_variables()

    def _optimization_variable_style_name(self, variable_name: str) -> str:
        hardpoint_name = variable_name.rsplit("_", 1)[0]
        return f"SuspensionOptimizationVariable.{hardpoint_name}.TCheckbutton"

    def _configure_optimization_variable_styles(
        self,
        variable_names: tuple[str, ...],
    ) -> None:
        style = ttk.Style(self)
        hardpoint_names = sorted({name.rsplit("_", 1)[0] for name in variable_names})
        for index, hardpoint_name in enumerate(hardpoint_names):
            color = OPTIMIZATION_VARIABLE_COLOR_PALETTE[
                index % len(OPTIMIZATION_VARIABLE_COLOR_PALETTE)
            ]
            style_name = self._optimization_variable_style_name(
                f"{hardpoint_name}_x"
            )
            style.configure(style_name, foreground=color)
            style.map(
                style_name,
                foreground=[
                    ("disabled", color),
                    ("selected", color),
                    ("active", color),
                    ("!disabled", color),
                ],
            )

    def _optimization_mode_key(self, label: str) -> str:
        for mode, mode_label in SUSPENSION_OPTIMIZATION_TARGET_MODES:
            if mode_label == label:
                return mode
        return "endpoint_delta"

    def _optimization_mode_label(self, mode: str) -> str:
        for mode_key, label in SUSPENSION_OPTIMIZATION_TARGET_MODES:
            if mode_key == mode:
                return label
        return "End-to-end delta"

    def _optimization_solver_mode_key(self, label: str) -> str:
        for mode, mode_label in SUSPENSION_OPTIMIZATION_SOLVER_MODES:
            if mode_label == label:
                return mode
        return "dual_path"

    def _optimization_solver_mode_label(self, mode: str) -> str:
        for mode_key, label in SUSPENSION_OPTIMIZATION_SOLVER_MODES:
            if mode_key == mode:
                return label
        return "Dual Path"

    def _show_optimization_message(
        self,
        message: str,
        *,
        heading: str | None = None,
        kind: str = "summary",
    ) -> None:
        sections: list[dict[str, str]] = []
        if heading:
            sections.append({"kind": "heading", "text": heading})
        sections.append({"kind": kind, "text": message})
        self._render_optimization_output(sections)

    def _configure_optimization_output_tags(self) -> None:
        self.optimization_output.tag_configure(
            "heading",
            foreground="#0f172a",
            font=("TkDefaultFont", 10, "bold"),
            spacing3=4,
        )
        self.optimization_output.tag_configure(
            "summary",
            foreground="#334155",
            spacing1=1,
            spacing3=2,
        )
        self.optimization_output.tag_configure(
            "recommended",
            foreground="#047857",
            spacing1=1,
            spacing3=2,
        )
        self.optimization_output.tag_configure(
            "secondary",
            foreground="#b45309",
            spacing1=1,
            spacing3=2,
        )
        self.optimization_output.tag_configure(
            "suppress",
            foreground="#b91c1c",
            spacing1=1,
            spacing3=2,
        )
        self.optimization_output.tag_configure(
            "progress",
            foreground="#2563eb",
            spacing1=1,
            spacing3=2,
        )
        self.optimization_output.tag_configure(
            "error",
            foreground="#b91c1c",
            spacing1=1,
            spacing3=2,
        )

    def _render_optimization_output(
        self,
        sections: list[dict[str, str]],
    ) -> None:
        plain_sections: list[str] = []
        self.optimization_output.configure(state=tk.NORMAL)
        self.optimization_output.delete("1.0", tk.END)
        for index, section in enumerate(sections):
            text = str(section.get("text", "")).strip()
            if not text:
                continue
            tag = str(section.get("tone") or section.get("kind") or "summary")
            suffix = "\n\n" if index < len(sections) - 1 else ""
            self.optimization_output.insert(tk.END, text + suffix, (tag,))
            plain_sections.append(text)
        self.optimization_output.configure(state=tk.DISABLED)
        self._copyable_optimization_output = "\n\n".join(plain_sections)
        status_var = getattr(self, "optimization_status_var", None)
        if status_var is not None:
            status_var.set(self._copyable_optimization_output)

    def _copy_optimization_output(self, _event: object | None = None) -> str:
        selected_text = ""
        try:
            selected_text = str(
                self.optimization_output.get(tk.SEL_FIRST, tk.SEL_LAST)
            ).strip()
        except (AttributeError, tk.TclError):
            selected_text = ""
        text = selected_text or self._copyable_optimization_output
        self.clipboard_clear()
        self.clipboard_append(text)
        self.update()
        return "break"

    def _format_optimization_summary_line(self, summary: object) -> str:
        metric_name = getattr(summary, "metric_name")
        target_mode = getattr(summary, "target_mode")
        target_value = float(getattr(summary, "target_delta"))
        initial_value = float(getattr(summary, "initial_value"))
        final_value = float(getattr(summary, "final_value"))
        if target_mode == "absolute_value":
            return (
                f"{metric_name}: Curve value target {target_value:.6g}, "
                f"initial RMS error {initial_value:.6g}, "
                f"final RMS error {final_value:.6g}"
            )
        return (
            f"{metric_name}: {self._optimization_mode_label(target_mode)} "
            f"target {target_value:.6g}, "
            f"initial {initial_value:.6g}, "
            f"final {final_value:.6g}"
        )

    def _format_optimization_progress(
        self,
        progress: SuspensionOptimizationProgress,
    ) -> list[dict[str, str]]:
        return [
            {"kind": "heading", "text": "Optimization Running"},
            {
                "kind": "progress",
                "text": (
                    f"{progress.message}\n"
                    f"Phase: {progress.phase}\n"
                    f"Evaluations: {progress.evaluations}\n"
                    f"Elapsed: {progress.elapsed_seconds:.1f}s"
                ),
            },
        ]

    def _format_optimization_result(self, result: object) -> list[dict[str, str]]:
        sections = [
            {"kind": "heading", "text": "Optimization Result"},
            {
                "kind": "summary",
                "text": (
                    f"Method: {self._optimization_solver_mode_label(str(getattr(result, 'solver_mode', 'dual_path')))}\n"
                    f"Initial cost: {float(getattr(result, 'initial_cost')):.6g}\n"
                    f"Final cost: {float(getattr(result, 'final_cost')):.6g}\n"
                    f"Rounds: {int(getattr(result, 'rounds_completed'))}\n"
                    f"Evaluations: {int(getattr(result, 'total_evaluations'))}"
                ),
            },
            {
                "kind": "recommended"
                if bool(getattr(result, "success", False))
                else "secondary",
                "text": f"Solver message: {getattr(result, 'message', '')}",
            },
        ]
        for summary in getattr(result, "target_summaries", []):
            improved = float(getattr(summary, "final_value")) <= float(
                getattr(summary, "initial_value")
            )
            sections.append(
                {
                    "kind": "group",
                    "tone": "recommended" if improved else "secondary",
                    "text": self._format_optimization_summary_line(summary),
                }
            )
        return sections

    def _format_optimization_analysis(
        self,
        result: SuspensionOptimizationVariableAnalysisResult,
    ) -> list[dict[str, str]]:
        sections: list[dict[str, str]] = [
            {"kind": "heading", "text": "Global Sensitivity Analysis"},
            {
                "kind": "summary",
                "text": (
                    f"Method: {result.method}\n"
                    f"Effective rank: {result.effective_rank}/{result.variable_count}\n"
                    f"Constraint rank: {result.constraint_rank}\n"
                    f"Residual size: {result.residual_size}\n"
                    f"Morris trajectories: {result.morris_trajectories}\n"
                    "Sobol directions/base samples: "
                    f"{result.sobol_direction_count}/{result.sobol_base_samples}"
                ),
            },
        ]
        grouped_items = {
            "recommended": [
                item for item in result.items if item.recommendation == "recommended"
            ],
            "secondary": [
                item for item in result.items if item.recommendation == "secondary"
            ],
            "suppress": [
                item for item in result.items if item.recommendation == "suppress"
            ],
        }
        for recommendation, label in (
            ("recommended", "Recommended"),
            ("secondary", "Secondary"),
            ("suppress", "Suppress"),
        ):
            items = grouped_items[recommendation]
            details = "\n\n".join(
                self._format_optimization_analysis_item(item) for item in items
            ) or "None"
            sections.append(
                {
                    "kind": "group",
                    "tone": recommendation,
                    "text": f"{label}\n{details}",
                }
            )
        return sections

    def _format_optimization_analysis_item(self, item: object) -> str:
        sobol_text = "Sobol S1/ST: n/a"
        if (
            getattr(item, "sobol_first_order") is not None
            and getattr(item, "sobol_total") is not None
        ):
            sobol_text = (
                "Sobol S1/ST: "
                f"{float(getattr(item, 'sobol_first_order')):.3g}/"
                f"{float(getattr(item, 'sobol_total')):.3g}"
            )
        return (
            f"{getattr(item, 'variable_name')}\n"
            f"Morris mu*: {float(getattr(item, 'morris_mu_star')):.3g} | "
            f"sigma: {float(getattr(item, 'morris_sigma')):.3g}\n"
            f"{sobol_text}\n"
            f"{getattr(item, 'detail')}"
        )

    def _sync_available_optimization_pair_constraints(self) -> None:
        constraints = self.project.optimization.pair_delta_constraints
        self.opt_pair_constraint_vars = {
            constraint.key(): tk.BooleanVar(value=constraint.enabled)
            for constraint in constraints
        }
        for child in self.opt_pair_constraints_frame.winfo_children():
            child.destroy()
        for index, constraint in enumerate(constraints):
            key = constraint.key()
            ttk.Checkbutton(
                self.opt_pair_constraints_frame,
                text=f"{constraint.label} ({'/'.join(axis.upper() for axis in constraint.axes)})",
                variable=self.opt_pair_constraint_vars[key],
                command=self._on_optimization_controls_changed,
            ).grid(
                row=index,
                column=0,
                sticky="w",
                padx=(0, 10),
                pady=2,
            )

    def _sync_travel_slider_limits(self) -> None:
        self.travel_slider.configure(
            from_=self.project.settings.start,
            to=self.project.settings.stop,
        )

    def _sync_travel_controls(self, value: float) -> None:
        self.updating_controls = True
        try:
            low = min(self.project.settings.start, self.project.settings.stop)
            high = max(self.project.settings.start, self.project.settings.stop)
            slider_value = min(max(float(value), low), high)
            self.travel_slider_var.set(slider_value)
            self.travel_value_var.set(f"{slider_value:.6g}")
        finally:
            self.updating_controls = False

    def _on_travel_slider_changed(self, value: str) -> None:
        if self.updating_controls:
            return
        self.updating_controls = True
        self.travel_value_var.set(f"{float(value):.6g}")
        self.updating_controls = False
        self._schedule_preview_refresh()

    def _on_travel_slider_released(self, _event: tk.Event) -> None:
        self.refresh()

    def _schedule_preview_refresh(self) -> None:
        self.schedule_preview_refresh(
            scheduler=self.after,
            callback=self._refresh_preview_only,
        )

    def _refresh_preview_only(self) -> None:
        self.clear_pending_preview_refresh()

        def draw_preview() -> None:
            if not self._sync_controls_to_project():
                return
            self._sync_travel_slider_limits()
            parsed_travel = parse_float_entry(
                self.travel_value_var.get(),
                float(self.travel_slider_var.get()),
            )
            if not parsed_travel.is_valid:
                self.status_var.set("Invalid numeric input: travel")
                return
            if not parsed_travel.is_complete:
                return
            travel = float(parsed_travel.value)
            preview = solve_suspension_project_at_travel(self.project, travel)
            self.result = preview
            self._draw_result_index(0, update_outputs=False)
            self.status_var.set(f"Preview travel {travel:.6g} mm")
        self.run_guarded(
            action=draw_preview,
            on_error=lambda exc: self.status_var.set(str(exc)),
        )

    def _on_controls_changed(self, *_args: object) -> None:
        self._reset_optimization_analysis()
        self.trigger_refresh_if_ready()

    def _on_optimization_controls_changed(self, _event: tk.Event | None = None) -> None:
        self._reset_optimization_analysis()

    def refresh(self) -> None:
        """Refresh preview/output at current travel and regenerate sweep curves."""
        def redraw_all() -> None:
            if not self._sync_controls_to_project():
                return
            self._sync_travel_slider_limits()
            parsed_travel = parse_float_entry(
                self.travel_value_var.get(),
                float(self.travel_slider_var.get()),
            )
            if not parsed_travel.is_valid:
                self.status_var.set("Invalid numeric input: travel")
                return
            if not parsed_travel.is_complete:
                return
            travel = float(parsed_travel.value)
            preview = solve_suspension_project_at_travel(self.project, travel)
            self.result = preview
            self._draw_result_index(0)
            self.refresh_curves()
        self.run_guarded(
            action=redraw_all,
            on_error=lambda exc: self.status_var.set(str(exc)),
        )

    def _on_type_changed(self, _event: tk.Event) -> None:
        self.project = create_default_suspension_project(self.suspension_type_var.get())
        self.imported_default_hardpoints = {
            point_id: position.copy()
            for point_id, position in self.project.hardpoints.items()
        }
        self.geometry_path_var.set("New geometry")
        self.result = None
        self.preview_has_drawn = False
        self.preview_renderer.reset()
        self._reset_optimization_analysis()
        self._load_project_to_controls()
        self.status_var.set("Suspension type changed")
        self.refresh()

    def _on_hardpoints_changed(self) -> None:
        self.result = None
        self.preview_has_drawn = False
        self.preview_renderer.reset()
        self._reset_optimization_analysis()
        self.refresh()

    def restore_default_hardpoints(self) -> None:
        """Restore the hardpoints from the latest imported file snapshot."""
        self.project.hardpoints = {
            point_id: position.copy()
            for point_id, position in self.imported_default_hardpoints.items()
        }
        self.pending_optimized_hardpoints = None
        self.result = None
        self.preview_has_drawn = False
        self.preview_renderer.reset()
        self._reset_optimization_analysis()
        self.hardpoint_table.set_hardpoints(self.project.hardpoints)
        self._sync_available_optimization_variables()
        self.status_var.set("Restored default hardpoints")
        self.refresh()
