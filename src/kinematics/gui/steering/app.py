"""
Tkinter steering workbench GUI.
"""

from __future__ import annotations

import queue
import threading
import tkinter as tk
from dataclasses import replace
from pathlib import Path
from tkinter import Misc, ttk

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

from kinematics.gui.common import OptimizationCancelledError, RefreshWorkflowMixin
from kinematics.gui.steering.file_actions import SteeringFileActions
from kinematics.gui.steering.plotting import (
    draw_curve_plot,
    draw_rack_and_pinion_steering_preview,
    draw_steering_preview,
    draw_three_segment_steering_preview,
    fit_steering_preview,
)
from kinematics.gui.steering.widgets import (
    CurveManager,
    HardpointEditor,
    OutputTable,
    PitmanTransformControls,
)
from kinematics.steering.geometry import (
    ThreeSegmentSteeringSolution,
    TwoSegmentSteeringSolution,
)
from kinematics.steering.limits import (
    steering_limit_outputs,
    three_segment_steering_limit_outputs,
)
from kinematics.steering.two_segment import (
    solve_two_segment_steering,
)
from kinematics.steering.workbench import (
    INPUT_MODES,
    LINKAGE_TYPES,
    RACK_AND_PINION_INPUT_MODES,
    RACK_AND_PINION_LINKAGE_TYPE,
    THREE_SEGMENT_INPUT_MODES,
    TWO_SEGMENT_INPUT_MODES,
    available_steering_outputs,
    copy_hardpoint_rows,
    curve_specs_for_plot,
    default_steering_project,
    hardpoints_from_rows,
    input_angle_slider_limits,
    optimize_steering_hardpoints,
    parse_float_entry,
    solve_steering_project,
    steering_project_limit_outputs,
    sweep_steering_project,
    three_segment_geometry_from_rows,
    three_segment_hardpoints_from_rows,
)

OPTIMIZATION_VARIABLE_OPTIONS = (
    ("pitman_x", "Pitman X"),
    ("pitman_arm_x_length", "Arm X length"),
    ("tie_rod_outer_x", "Outer X"),
    ("tie_rod_outer_y", "Outer Y"),
    ("tie_rod_inner_x", "Inner X"),
    ("tie_rod_inner_y", "Inner Y"),
)


class SteeringWorkbenchApp(RefreshWorkflowMixin, SteeringFileActions):
    """Main steering workbench window."""

    PREVIEW_REFRESH_DELAY_MS = 16
    WORKSPACE_PREVIEW_WEIGHT = 2
    WORKSPACE_SIDE_WEIGHT = 1

    def __init__(self, root: Misc, *, standalone: bool = True) -> None:
        self.root = root
        self.standalone = standalone
        self.project = default_steering_project()
        self.project_path: Path | None = None
        self.updating_controls = False
        self.preview_has_drawn = False
        self.previous_three_segment_state: ThreeSegmentSteeringSolution | None = None
        self.pending_preview_refresh: str | None = None
        self.pending_hardpoint_full_refresh: str | None = None
        self._reset_refresh_caches()
        self.background_refresh_queue: queue.Queue[tuple[str, object]] | None = None
        self.background_refresh_generation = 0
        self.background_refresh_polling = False
        self.background_refresh_pending = 0
        self.linkage_type_var = tk.StringVar(value=self.project.linkage_type)
        self.input_mode_var = tk.StringVar(value=self.project.input_mode)
        self.input_value_var = tk.StringVar(value=str(self.project.input_value))
        self.input_slider_var = tk.DoubleVar(value=self.project.input_value)
        self.sweep_min_var = tk.StringVar(value=str(self.project.sweep_min))
        self.sweep_max_var = tk.StringVar(value=str(self.project.sweep_max))
        self.sweep_step_var = tk.StringVar(value=str(self.project.sweep_step))
        self.static_radius_var = tk.StringVar(value=str(self.project.static_radius_mm))
        self.section_width_var = tk.StringVar(value=str(self.project.section_width))
        self.wheelbase_var = tk.StringVar(value=str(self.project.wheelbase))
        self.pinion_pitch_radius_var = tk.StringVar(
            value=str(self.project.pinion_pitch_radius_mm)
        )
        self.opt_inner_wheel_var = tk.StringVar(value="right")
        self.opt_inner_angle_var = tk.StringVar(value="10.0")
        self.opt_target_delta_var = tk.StringVar(value="-4.0")
        self.opt_delta_limit_var = tk.StringVar(value="40.0")
        self.opt_variable_vars = {
            name: tk.BooleanVar(value=name in {"pitman_x", "pitman_arm_x_length"})
            for name, _label in OPTIMIZATION_VARIABLE_OPTIONS
        }
        self.optimization_running = False
        self.optimization_queue: queue.Queue[tuple[str, object]] | None = None
        self.optimization_thread: threading.Thread | None = None
        self.optimization_cancel_event: threading.Event | None = None
        self.pending_optimized_hardpoints = None
        self.imported_default_hardpoints = copy_hardpoint_rows(self.project.hardpoints)
        if self.standalone:
            self._build_menu()
        self._build_layout()
        self._bind_control_vars()
        self._load_project_to_controls()
        self.refresh()

    def _reset_refresh_caches(self) -> None:
        self.limit_outputs_cache_key: tuple[object, ...] | None = None
        self.limit_outputs_cache: dict[str, float] | None = None
        self.slider_limits_cache_key: tuple[object, ...] | None = None
        self.slider_limits_cache = None
        self.curve_rows_cache_key: tuple[object, ...] | None = None
        self.curve_rows_cache: list[dict[str, float]] | None = None

    def _has_valid_limit_outputs_cache(self) -> bool:
        return (
            self.limit_outputs_cache_key == self._limit_outputs_cache_signature()
            and self.limit_outputs_cache is not None
        )

    def _has_valid_slider_limits_cache(self) -> bool:
        return (
            self.slider_limits_cache_key == self._slider_limits_cache_signature()
            and self.slider_limits_cache is not None
        )

    def _has_valid_curve_rows_cache(self) -> bool:
        return (
            self.curve_rows_cache_key == self._curve_rows_cache_signature()
            and self.curve_rows_cache is not None
        )

    def _project_snapshot(self):
        return replace(
            self.project,
            hardpoints=copy_hardpoint_rows(self.project.hardpoints),
            curves=list(self.project.curves),
        )

    def _hardpoint_signature(self) -> tuple[tuple[object, ...], ...]:
        return tuple(
            (row.category, row.name, row.x, row.y, row.z)
            for row in self.project.hardpoints
        )

    def _limit_outputs_cache_signature(self) -> tuple[object, ...]:
        return (
            self.project.linkage_type,
            self.project.input_mode,
            self.project.pinion_pitch_radius_mm,
            self._hardpoint_signature(),
        )

    def _slider_limits_cache_signature(self) -> tuple[object, ...]:
        return (
            self.project.linkage_type,
            self.project.input_mode,
            self.project.pinion_pitch_radius_mm,
            self._hardpoint_signature(),
        )

    def _curve_rows_cache_signature(self) -> tuple[object, ...]:
        return (
            self.project.linkage_type,
            self.project.input_mode,
            self.project.sweep_min,
            self.project.sweep_max,
            self.project.sweep_step,
            self.project.wheelbase,
            self.project.pinion_pitch_radius_mm,
            self._hardpoint_signature(),
        )

    def _limit_outputs_for_current_project(self) -> dict[str, float]:
        key = self._limit_outputs_cache_signature()
        if key != self.limit_outputs_cache_key or self.limit_outputs_cache is None:
            if self.project.linkage_type == "three_segment":
                hardpoints = three_segment_hardpoints_from_rows(self.project.hardpoints)
                outputs = three_segment_steering_limit_outputs(hardpoints)
            elif (
                self.project.linkage_type == RACK_AND_PINION_LINKAGE_TYPE
                or self.project.input_mode in RACK_AND_PINION_INPUT_MODES
            ):
                outputs = steering_project_limit_outputs(self.project)
            else:
                hardpoints = hardpoints_from_rows(self.project.hardpoints)
                outputs = steering_limit_outputs(hardpoints)
            self.limit_outputs_cache_key = key
            self.limit_outputs_cache = outputs
        return dict(self.limit_outputs_cache)

    def _curve_rows_for_current_project(self) -> list[dict[str, float]]:
        key = self._curve_rows_cache_signature()
        if key != self.curve_rows_cache_key or self.curve_rows_cache is None:
            self.curve_rows_cache = sweep_steering_project(
                self.project,
                skip_unreachable=True,
            )
            self.curve_rows_cache_key = key
        return self.curve_rows_cache

    def _queue_background_refresh(
        self,
        *,
        project_snapshot,
        refresh_generation: int,
        need_limits: bool,
        need_curves: bool,
    ) -> None:
        if not need_limits and not need_curves:
            return
        if not hasattr(self, "background_refresh_pending"):
            self.background_refresh_pending = 0
        if self.background_refresh_queue is None:
            self.background_refresh_queue = queue.Queue()
        if need_limits:
            self.background_refresh_pending += 1
            threading.Thread(
                target=self._background_limits_worker,
                args=(project_snapshot, refresh_generation),
                daemon=True,
            ).start()
        if need_curves:
            self.background_refresh_pending += 1
            threading.Thread(
                target=self._background_curve_worker,
                args=(project_snapshot, refresh_generation),
                daemon=True,
            ).start()
        if not self.background_refresh_polling:
            self.background_refresh_polling = True
            self.root.after(50, self._poll_background_refresh)

    def _background_limits_worker(
        self,
        project_snapshot,
        refresh_generation: int,
    ) -> None:
        assert self.background_refresh_queue is not None
        try:
            if project_snapshot.linkage_type == "three_segment":
                hardpoints = three_segment_hardpoints_from_rows(
                    project_snapshot.hardpoints
                )
                limit_outputs = three_segment_steering_limit_outputs(hardpoints)
            elif (
                project_snapshot.linkage_type == RACK_AND_PINION_LINKAGE_TYPE
                or project_snapshot.input_mode in RACK_AND_PINION_INPUT_MODES
            ):
                limit_outputs = steering_project_limit_outputs(project_snapshot)
            else:
                hardpoints = hardpoints_from_rows(project_snapshot.hardpoints)
                limit_outputs = steering_limit_outputs(hardpoints)
            if (
                project_snapshot.linkage_type == RACK_AND_PINION_LINKAGE_TYPE
                or project_snapshot.input_mode in RACK_AND_PINION_INPUT_MODES
            ):
                slider_limits = input_angle_slider_limits(
                    project_snapshot.hardpoints,
                    project_snapshot.input_mode,
                    project_snapshot.linkage_type,
                    project_snapshot.pinion_pitch_radius_mm,
                )
            else:
                slider_limits = input_angle_slider_limits(
                    project_snapshot.hardpoints,
                    project_snapshot.input_mode,
                    project_snapshot.linkage_type,
                )
        except Exception as exc:  # noqa: BLE001 - surface in polling loop.
            self.background_refresh_queue.put(
                ("background_error", refresh_generation, exc)
            )
            return
        self.background_refresh_queue.put(
            (
                "limits",
                refresh_generation,
                project_snapshot,
                limit_outputs,
                slider_limits,
            )
        )

    def _background_curve_worker(
        self,
        project_snapshot,
        refresh_generation: int,
    ) -> None:
        assert self.background_refresh_queue is not None
        try:
            rows = sweep_steering_project(project_snapshot, skip_unreachable=True)
        except Exception as exc:  # noqa: BLE001 - surface in polling loop.
            self.background_refresh_queue.put(
                ("background_error", refresh_generation, exc)
            )
            return
        self.background_refresh_queue.put(
            ("curves", refresh_generation, project_snapshot, rows)
        )

    def _poll_background_refresh(self) -> None:
        if self.background_refresh_queue is None:
            self.background_refresh_polling = False
            return
        if not hasattr(self, "background_refresh_pending"):
            self.background_refresh_pending = 0

        while True:
            try:
                item = self.background_refresh_queue.get_nowait()
            except queue.Empty:
                break

            self.background_refresh_pending = max(
                0, self.background_refresh_pending - 1
            )
            kind = item[0]
            generation = item[1]
            if generation != self.background_refresh_generation:
                continue

            if kind == "background_error":
                self.output_table.set_error(str(item[2]))
                continue

            if kind == "preview":
                _kind, _generation, state = item
                if isinstance(state, ThreeSegmentSteeringSolution):
                    self.previous_three_segment_state = state
                self._draw_preview_state(state)
                self.preview_has_drawn = True
                self.preview_canvas.draw_idle()
                continue

            if kind == "limits":
                _kind, _generation, project_snapshot, limit_outputs, slider_limits = (
                    item
                )
                self.limit_outputs_cache_key = (
                    project_snapshot.linkage_type,
                    project_snapshot.input_mode,
                    project_snapshot.pinion_pitch_radius_mm,
                    tuple(
                        (row.category, row.name, row.x, row.y, row.z)
                        for row in project_snapshot.hardpoints
                    ),
                )
                self.limit_outputs_cache = limit_outputs
                self.slider_limits_cache_key = (
                    project_snapshot.linkage_type,
                    project_snapshot.input_mode,
                    project_snapshot.pinion_pitch_radius_mm,
                    tuple(
                        (row.category, row.name, row.x, row.y, row.z)
                        for row in project_snapshot.hardpoints
                    ),
                )
                self.slider_limits_cache = slider_limits
                if self.output_table.outputs:
                    outputs = dict(self.output_table.outputs[-1])
                    outputs.update(limit_outputs)
                    self.output_table.set_outputs(outputs)
                self.updating_controls = True
                self._sync_input_slider_limits(self.project.input_value)
                self.updating_controls = False
            elif kind == "curves":
                _kind, _generation, project_snapshot, rows = item
                self.curve_rows_cache_key = (
                    project_snapshot.linkage_type,
                    project_snapshot.input_mode,
                    project_snapshot.sweep_min,
                    project_snapshot.sweep_max,
                    project_snapshot.sweep_step,
                    project_snapshot.wheelbase,
                    project_snapshot.pinion_pitch_radius_mm,
                    tuple(
                        (row.category, row.name, row.x, row.y, row.z)
                        for row in project_snapshot.hardpoints
                    ),
                )
                self.curve_rows_cache = rows
                curves = curve_specs_for_plot(
                    self.project.curves,
                    self.curve_manager.x_var.get(),
                    self.curve_manager.y_var.get(),
                    self.curve_manager.label_var.get(),
                )
                draw_curve_plot(self.curve_ax, rows, curves)
                self.curve_canvas.draw_idle()

        if (
            self.background_refresh_queue.empty()
            and self.background_refresh_pending == 0
        ):
            self.background_refresh_polling = False
            return
        self.root.after(50, self._poll_background_refresh)

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

    def import_hardpoints(self) -> None:
        self.import_csv()

    def export_hardpoints(self) -> None:
        self.export_csv()

    def _build_layout(self) -> None:
        if self.standalone and isinstance(self.root, tk.Tk):
            self.root.title("Steering Workbench")
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
        self._build_suspension_parameters(left)
        self.pitman_controls = PitmanTransformControls(
            left,
            self._on_pitman_transform_changed,
        )
        self.pitman_controls.pack(fill=tk.X, pady=(8, 0))

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

    def _build_suspension_parameters(self, parent: ttk.Frame) -> None:
        frame = ttk.LabelFrame(parent, text="Suspension Parameters", padding=6)
        frame.pack(fill=tk.X, pady=(8, 0))
        frame.columnconfigure(1, weight=1)
        entries: list[ttk.Entry] = []
        for row_index, (label, var) in enumerate(
            (
                ("Tire width", self.section_width_var),
                ("Static radius [mm]", self.static_radius_var),
                ("Wheelbase", self.wheelbase_var),
            )
        ):
            ttk.Label(frame, text=label).grid(row=row_index, column=0, sticky="w")
            entry = ttk.Entry(frame, textvariable=var, width=12)
            entry.grid(
                row=row_index,
                column=1,
                sticky="ew",
                padx=(6, 0),
                pady=2,
            )
            entries.append(entry)
        self.bind_entry_commit_refresh(entries)

    def _build_controls(self, parent: ttk.Frame) -> None:
        parent.columnconfigure(7, weight=1)
        refresh_commit_entries: list[ttk.Entry] = []
        ttk.Label(parent, text="Linkage").grid(row=0, column=0, sticky="w")
        self.linkage_type_combo = ttk.Combobox(
            parent,
            textvariable=self.linkage_type_var,
            values=LINKAGE_TYPES,
            state="readonly",
            width=14,
        )
        self.linkage_type_combo.grid(row=0, column=1, sticky="w", padx=(6, 12))
        ttk.Label(parent, text="Control").grid(row=0, column=2, sticky="w")
        self.input_mode_combo = ttk.Combobox(
            parent,
            textvariable=self.input_mode_var,
            values=INPUT_MODES,
            state="readonly",
            width=22,
        )
        self.input_mode_combo.grid(row=0, column=3, sticky="w", padx=(6, 12))
        ttk.Label(parent, text="Value").grid(row=0, column=4, sticky="w")
        value_entry = ttk.Entry(parent, textvariable=self.input_value_var, width=10)
        value_entry.grid(row=0, column=5, sticky="w", padx=(6, 12))
        refresh_commit_entries.append(value_entry)
        ttk.Label(parent, text="Input").grid(row=0, column=6, sticky="w")
        self.input_slider = ttk.Scale(
            parent,
            variable=self.input_slider_var,
            command=self._on_input_slider_changed,
            length=220,
        )
        self.input_slider.grid(row=0, column=7, sticky="ew", padx=(6, 0))
        self.input_slider.bind("<ButtonRelease-1>", self._on_input_slider_released)
        for column, (label, var) in enumerate(
            (
                ("Sweep min", self.sweep_min_var),
                ("Sweep max", self.sweep_max_var),
                ("Step", self.sweep_step_var),
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
            entry = ttk.Entry(parent, textvariable=var, width=8)
            entry.grid(
                row=1,
                column=entry_column,
                sticky="w",
                padx=(6, 18),
                pady=(6, 0),
            )
            refresh_commit_entries.append(entry)
        ttk.Label(parent, text="Pinion pitch R [mm]").grid(
            row=1,
            column=6,
            sticky="w",
            pady=(6, 0),
        )
        pinion_entry = ttk.Entry(
            parent,
            textvariable=self.pinion_pitch_radius_var,
            width=8,
        )
        pinion_entry.grid(
            row=1,
            column=7,
            sticky="w",
            padx=(6, 0),
            pady=(6, 0),
        )
        refresh_commit_entries.append(pinion_entry)
        self.bind_entry_commit_refresh(refresh_commit_entries)

    def _build_preview(self, parent: ttk.Frame) -> None:
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.BOTH, expand=True)
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

    def _build_side_panel(self, parent: ttk.Frame) -> None:
        panel = ttk.Frame(parent)
        panel.pack(fill=tk.BOTH, expand=True)
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
        buttons.columnconfigure(2, weight=1)
        self.optimize_button = ttk.Button(
            buttons,
            text="Optimize",
            command=self.run_optimization,
        )
        self.optimize_button.grid(
            row=0,
            column=0,
            sticky="ew",
            padx=(0, 4),
        )
        self.apply_optimization_button = ttk.Button(
            buttons,
            text="Apply",
            command=self.apply_optimization,
        )
        self.apply_optimization_button.grid(
            row=0,
            column=1,
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
            column=2,
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
        self.bind_control_var_traces(
            (
                self.linkage_type_var,
                self.input_mode_var,
            ),
            self._on_controls_changed,
        )

    def _load_project_to_controls(self) -> None:
        self.updating_controls = True
        self.preview_has_drawn = False
        self.previous_three_segment_state = None
        self.linkage_type_var.set(self.project.linkage_type)
        self._sync_input_mode_values()
        self.input_mode_var.set(self.project.input_mode)
        self.input_value_var.set(str(self.project.input_value))
        self.input_slider_var.set(self.project.input_value)
        self.sweep_min_var.set(str(self.project.sweep_min))
        self.sweep_max_var.set(str(self.project.sweep_max))
        self.sweep_step_var.set(str(self.project.sweep_step))
        self.static_radius_var.set(str(self.project.static_radius_mm))
        self.section_width_var.set(str(self.project.section_width))
        self.wheelbase_var.set(str(self.project.wheelbase))
        self.pinion_pitch_radius_var.set(str(self.project.pinion_pitch_radius_mm))
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self._sync_pitman_controls()
        self._sync_input_slider_limits(self.project.input_value)
        self.curve_manager.set_curves(self.project.curves)
        self.updating_controls = False

    def _sync_input_slider_limits(self, value: float) -> None:
        key = self._slider_limits_cache_signature()
        if key != self.slider_limits_cache_key or self.slider_limits_cache is None:
            self.slider_limits_cache = input_angle_slider_limits(
                self.project.hardpoints,
                self.project.input_mode,
                self.project.linkage_type,
                *(
                    (self.project.pinion_pitch_radius_mm,)
                    if self.project.input_mode in RACK_AND_PINION_INPUT_MODES
                    else ()
                ),
            )
            self.slider_limits_cache_key = key
        limits = self.slider_limits_cache
        self.input_slider.configure(from_=limits.minimum, to=limits.maximum)
        slider_value = min(max(value, limits.minimum), limits.maximum)
        self.input_slider_var.set(slider_value)

    def _sync_controls_to_project(self) -> bool:
        linkage_type = self.linkage_type_var.get()
        if linkage_type != self.project.linkage_type:
            self._switch_linkage_type(linkage_type)
        previous_input_mode = self.project.input_mode
        self.project.input_mode = self.input_mode_var.get()
        if self.project.input_mode != previous_input_mode:
            self.previous_three_segment_state = None
            self._sync_pitman_controls()
        for attr, var in (
            ("input_value", self.input_value_var),
            ("sweep_min", self.sweep_min_var),
            ("sweep_max", self.sweep_max_var),
            ("sweep_step", self.sweep_step_var),
            ("static_radius_mm", self.static_radius_var),
            ("section_width", self.section_width_var),
            ("wheelbase", self.wheelbase_var),
            ("pinion_pitch_radius_mm", self.pinion_pitch_radius_var),
        ):
            parsed = parse_float_entry(var.get(), getattr(self.project, attr))
            if not parsed.is_valid:
                self.output_table.set_error(f"Invalid numeric input: {attr}")
                return False
            if not parsed.is_complete:
                return False
            if attr == "pinion_pitch_radius_mm" and parsed.value <= 0.0:
                self.output_table.set_error("Pinion pitch radius must be positive")
                return False
            if attr in {"static_radius_mm", "section_width"} and parsed.value <= 0.0:
                self.output_table.set_error(f"{attr} must be positive")
                return False
            setattr(self.project, attr, parsed.value)
        return True

    def _sync_input_mode_values(self) -> None:
        if self.project.linkage_type == "three_segment":
            modes = THREE_SEGMENT_INPUT_MODES
        elif self.project.linkage_type == RACK_AND_PINION_LINKAGE_TYPE:
            modes = RACK_AND_PINION_INPUT_MODES
        else:
            modes = TWO_SEGMENT_INPUT_MODES
        self.input_mode_combo.configure(values=modes)
        if self.project.input_mode not in modes:
            self.project.input_mode = modes[0]
            self.input_mode_var.set(self.project.input_mode)

    def _switch_linkage_type(self, linkage_type: str) -> None:
        self.project = default_steering_project(linkage_type=linkage_type)
        self._reset_refresh_caches()
        self.background_refresh_generation += 1
        self.background_refresh_pending = 0
        self.imported_default_hardpoints = copy_hardpoint_rows(self.project.hardpoints)
        self.pending_optimized_hardpoints = None
        self.preview_has_drawn = False
        self.previous_three_segment_state = None
        self._sync_input_mode_values()
        self.input_mode_var.set(self.project.input_mode)
        self.input_value_var.set(str(self.project.input_value))
        self.input_slider_var.set(self.project.input_value)
        self.pinion_pitch_radius_var.set(str(self.project.pinion_pitch_radius_mm))
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self._sync_pitman_controls()
        self._sync_input_slider_limits(self.project.input_value)

    def _sync_pitman_controls(self) -> None:
        if (
            self.project.linkage_type == "two_segment"
            and self.project.input_mode not in RACK_AND_PINION_INPUT_MODES
        ):
            self.pitman_controls.set_rows(self.project.hardpoints)
            self.pitman_controls.state(["!disabled"])
            return
        self.pitman_controls.state(["disabled"])

    def _on_input_slider_changed(self, value: str) -> None:
        if self.updating_controls:
            return
        self.updating_controls = True
        self.input_value_var.set(f"{float(value):.15g}")
        self.updating_controls = False
        self._schedule_preview_refresh()

    def _on_input_slider_released(self, _event: tk.Event) -> None:
        self.refresh()

    def _schedule_preview_refresh(self) -> None:
        self.schedule_preview_refresh(
            scheduler=self.root.after,
            callback=self._refresh_preview_only,
        )

    def _refresh_preview_only(self) -> None:
        self.clear_pending_preview_refresh()

        def queue_preview() -> None:
            if not self._sync_controls_to_project():
                return
            self.background_refresh_generation += 1
            refresh_generation = self.background_refresh_generation
            project_snapshot = self._project_snapshot()
            previous_state = (
                self.previous_three_segment_state
                if self.project.linkage_type == "three_segment"
                else None
            )
            if self.background_refresh_queue is None:
                self.background_refresh_queue = queue.Queue()
            self.background_refresh_pending += 1
            threading.Thread(
                target=self._background_preview_worker,
                args=(project_snapshot, refresh_generation, previous_state),
                daemon=True,
            ).start()
            if not self.background_refresh_polling:
                self.background_refresh_polling = True
                self.root.after(40, self._poll_background_refresh)

        self.run_guarded(
            action=queue_preview,
            on_error=lambda exc: self.output_table.set_error(str(exc)),
        )

    def _background_preview_worker(
        self,
        project_snapshot,
        refresh_generation: int,
        previous_state,
    ) -> None:
        assert self.background_refresh_queue is not None
        try:
            state, _outputs = solve_steering_project(
                project_snapshot,
                include_limits=False,
                previous_state=previous_state,
            )
        except Exception as exc:  # noqa: BLE001 - surface in polling loop.
            self.background_refresh_queue.put(
                ("background_error", refresh_generation, exc)
            )
            return
        self.background_refresh_queue.put(
            ("preview", refresh_generation, state)
        )

    def _on_controls_changed(self, *_args: object) -> None:
        self.trigger_refresh_if_ready()

    def _on_hardpoints_changed(self) -> None:
        self._reset_refresh_caches()
        self.background_refresh_generation += 1
        self.previous_three_segment_state = None
        self._sync_pitman_controls()
        self.schedule_hardpoint_edit_refresh(
            scheduler=self.root.after,
            cancel=self.root.after_cancel,
            preview_callback=self._refresh_preview_only,
            full_callback=self.refresh,
        )

    def _on_pitman_transform_changed(self) -> None:
        self._reset_refresh_caches()
        self.background_refresh_generation += 1
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self.refresh()

    def restore_default_hardpoints(self) -> None:
        """Restore hardpoints from the latest imported hardpoint snapshot."""
        self.project.hardpoints = copy_hardpoint_rows(self.imported_default_hardpoints)
        self._reset_refresh_caches()
        self.background_refresh_generation += 1
        self.pending_optimized_hardpoints = None
        self.previous_three_segment_state = None
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self._sync_pitman_controls()
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
        if self.optimization_running:
            self.optimization_status_var.set("Optimization already running")
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
        self.pending_optimized_hardpoints = None
        self.optimization_queue = queue.Queue()
        self.optimization_cancel_event = threading.Event()
        self.optimization_thread = threading.Thread(
            target=self._optimization_worker,
            args=(
                self.opt_inner_wheel_var.get(),
                inner_angle.value,
                target_delta.value,
                self._selected_optimization_variables(),
                delta_limit.value,
            ),
            daemon=True,
        )
        self._set_optimization_running(True)
        self.optimization_status_var.set("Optimization running")
        self.optimization_thread.start()
        self.root.after(100, self._poll_optimization_progress)

    def stop_optimization(self) -> None:
        """Request cooperative cancellation for steering optimization."""
        if (
            not self.optimization_running
            or self.optimization_cancel_event is None
            or self.optimization_cancel_event.is_set()
        ):
            return
        self.optimization_cancel_event.set()
        self.optimization_status_var.set("Stopping optimization")

    def _optimization_worker(
        self,
        inner_wheel: str,
        inner_angle_deg: float,
        target_left_minus_right_deg: float,
        variable_names: tuple[str, ...],
        variable_delta_limit: float,
    ) -> None:
        assert self.optimization_queue is not None
        try:
            result = optimize_steering_hardpoints(
                copy_hardpoint_rows(self.project.hardpoints),
                inner_wheel=inner_wheel,
                inner_wheel_angle_deg=inner_angle_deg,
                target_left_minus_right_deg=target_left_minus_right_deg,
                variable_names=variable_names,
                variable_delta_limit=variable_delta_limit,
                cancel_event=self.optimization_cancel_event,
            )
        except Exception as exc:  # noqa: BLE001 - surface in polling loop.
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
            if kind == "result":
                self.pending_optimized_hardpoints = payload.hardpoints
                self.optimization_status_var.set(
                    f"Initial error: {payload.initial_error_deg:.6g} deg\n"
                    f"Final error: {payload.final_error_deg:.6g} deg\n"
                    f"Actual L-R: {payload.actual_left_minus_right_deg:.6g} deg"
                )
                finished = True
            elif kind == "error":
                if isinstance(payload, OptimizationCancelledError):
                    self.optimization_status_var.set("Stopped")
                else:
                    self.optimization_status_var.set(str(payload))
                finished = True
        if finished:
            self._set_optimization_running(False)
            self.optimization_queue = None
            self.optimization_thread = None
            self.optimization_cancel_event = None
            return
        self.root.after(100, self._poll_optimization_progress)

    def _set_optimization_running(self, running: bool) -> None:
        self.optimization_running = running
        self.optimize_button.configure(state=tk.DISABLED if running else tk.NORMAL)
        self.apply_optimization_button.configure(
            state=tk.DISABLED if running else tk.NORMAL
        )
        self.stop_optimization_button.configure(
            state=tk.NORMAL if running else tk.DISABLED
        )

    def apply_optimization(self) -> None:
        """Apply the last optimized hardpoints to the current project."""
        if self.pending_optimized_hardpoints is None:
            self.optimization_status_var.set("No optimization result to apply")
            return
        self.project.hardpoints = self.pending_optimized_hardpoints
        self._reset_refresh_caches()
        self.background_refresh_generation += 1
        self.pending_optimized_hardpoints = None
        self.previous_three_segment_state = None
        self.hardpoint_editor.set_rows(self.project.hardpoints)
        self._sync_pitman_controls()
        self.preview_has_drawn = False
        self.refresh()
        self.optimization_status_var.set("Optimization applied")

    def fit_preview(self) -> None:
        fit_steering_preview(self.preview_ax)
        self.preview_toolbar.update()
        self.preview_canvas.draw_idle()

    def refresh(self) -> None:
        """Refresh preview, outputs, and curves."""

        def redraw_all() -> None:
            if not self._sync_controls_to_project():
                return
            self.background_refresh_generation += 1
            refresh_generation = self.background_refresh_generation
            previous_state = (
                self.previous_three_segment_state
                if self.project.linkage_type == "three_segment"
                else None
            )
            state, outputs = solve_steering_project(
                self.project,
                include_limits=False,
                previous_state=previous_state,
            )
            if isinstance(state, ThreeSegmentSteeringSolution):
                self.previous_three_segment_state = state
            if self._has_valid_limit_outputs_cache():
                outputs.update(self.limit_outputs_cache)
            if self._has_valid_slider_limits_cache():
                self.updating_controls = True
                self._sync_input_slider_limits(self.project.input_value)
                self.updating_controls = False
            self.updating_controls = True
            self.updating_controls = False
            self._draw_preview_state(state)
            self.preview_has_drawn = True
            self.preview_toolbar.update()
            self.preview_canvas.draw_idle()
            self.output_table.set_outputs(outputs)
            if self._has_valid_curve_rows_cache():
                curves = curve_specs_for_plot(
                    self.project.curves,
                    self.curve_manager.x_var.get(),
                    self.curve_manager.y_var.get(),
                    self.curve_manager.label_var.get(),
                )
                draw_curve_plot(self.curve_ax, self.curve_rows_cache, curves)
                self.curve_canvas.draw_idle()
            self._queue_background_refresh(
                project_snapshot=self._project_snapshot(),
                refresh_generation=refresh_generation,
                need_limits=not self._has_valid_limit_outputs_cache()
                or not self._has_valid_slider_limits_cache(),
                need_curves=not self._has_valid_curve_rows_cache(),
            )

        self.run_guarded(
            action=redraw_all,
            on_error=lambda exc: self.output_table.set_error(str(exc)),
        )

    def _draw_preview_state(self, state: object) -> None:
        if self.project.linkage_type == "three_segment":
            geometry = three_segment_geometry_from_rows(self.project.hardpoints)
            design_state = solve_steering_project(
                replace(
                    self.project,
                    input_mode="left_bellcrank_angle",
                    input_value=0.0,
                ),
                include_limits=False,
            )[0]
            draw_three_segment_steering_preview(
                self.preview_ax,
                geometry,
                design_state,
                state,
                preserve_view=self.preview_has_drawn,
                wheel_radius=self.project.static_radius_mm,
                wheel_width=self.project.section_width,
            )
            return
        hardpoints = hardpoints_from_rows(self.project.hardpoints)
        if (
            self.project.linkage_type == RACK_AND_PINION_LINKAGE_TYPE
            or self.project.input_mode in RACK_AND_PINION_INPUT_MODES
        ):
            assert isinstance(state, TwoSegmentSteeringSolution)
            design_state = solve_steering_project(
                replace(
                    self.project,
                    input_mode="rack_displacement",
                    input_value=0.0,
                ),
                include_limits=False,
            )[0]
            draw_rack_and_pinion_steering_preview(
                self.preview_ax,
                hardpoints,
                design_state,
                state,
                preserve_view=self.preview_has_drawn,
                wheel_radius=self.project.static_radius_mm,
                wheel_width=self.project.section_width,
            )
            return
        design_state = solve_two_segment_steering(hardpoints, 0.0)
        draw_steering_preview(
            self.preview_ax,
            hardpoints,
            design_state,
            state,
            preserve_view=self.preview_has_drawn,
            wheel_radius=self.project.static_radius_mm,
            wheel_width=self.project.section_width,
        )

    def refresh_curves(self) -> None:
        """Refresh managed curve plots."""

        def draw_curves() -> None:
            if not self._sync_controls_to_project():
                return
            if self._has_valid_curve_rows_cache():
                rows = self.curve_rows_cache
            else:
                self.background_refresh_generation += 1
                self._queue_background_refresh(
                    project_snapshot=self._project_snapshot(),
                    refresh_generation=self.background_refresh_generation,
                    need_limits=False,
                    need_curves=True,
                )
                return
            curves = curve_specs_for_plot(
                self.project.curves,
                self.curve_manager.x_var.get(),
                self.curve_manager.y_var.get(),
                self.curve_manager.label_var.get(),
            )
            draw_curve_plot(self.curve_ax, rows, curves)
            self.curve_canvas.draw_idle()

        self.run_guarded(
            action=draw_curves,
            on_error=lambda exc: self.output_table.set_error(str(exc)),
        )


def main() -> None:
    """Run the steering workbench GUI."""
    root = tk.Tk()
    SteeringWorkbenchApp(root, standalone=True)
    root.mainloop()


if __name__ == "__main__":
    main()
