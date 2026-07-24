"""Unified Tkinter GUI for kinematics workflows."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from typing import Protocol

from kinematics.core.enums import PointID
from kinematics.gui.hardpoint_merge import (
    detect_hardpoint_conflicts,
    merge_export_hardpoints,
    save_merged_hardpoints_csv,
    steering_export_hardpoints,
    steering_rows_from_suspension_hardpoints,
    suspension_export_hardpoints,
)
from kinematics.gui.reporting import (
    ReportCurveSelection,
    ReportExportOptions,
    export_gui_report_docx,
)
from kinematics.gui.steering import SteeringWorkbenchApp
from kinematics.gui.suspension import SuspensionWorkbenchPage
from kinematics.steering.workbench import (
    LINKAGE_TYPES,
    copy_hardpoint_rows,
    default_hardpoint_rows,
    default_steering_project,
    input_modes_for_linkage,
)


class KinematicsWorkbenchApp:
    """Main GUI shell hosting steering and suspension pages."""

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.pages: dict[str, object] = {}
        self._build_layout()
        self._build_menu()

    def _build_layout(self) -> None:
        self.root.title("Kinematics Workbench")
        self.root.geometry("1280x800")
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        steering_frame = ttk.Frame(self.notebook)
        self.notebook.add(steering_frame, text="Steering")
        self.pages[str(steering_frame)] = SteeringWorkbenchApp(
            steering_frame,
            standalone=False,
        )

        suspension_page = SuspensionWorkbenchPage(self.notebook)
        self.notebook.add(suspension_page, text="Suspension")
        self.pages[str(suspension_page)] = suspension_page

    def _build_menu(self) -> None:
        menu = tk.Menu(self.root)
        file_menu = tk.Menu(menu, tearoff=False)
        file_menu.add_command(label="Open Project", command=self.open_project)
        file_menu.add_separator()
        file_menu.add_command(
            label="Import Hardpoints",
            command=self.import_hardpoints,
        )
        file_menu.add_command(
            label="Export Hardpoints",
            command=self.export_hardpoints,
        )
        file_menu.add_command(
            label="Export Report",
            command=self.export_report,
        )
        file_menu.add_command(
            label="Export Combined Hardpoints",
            command=self.export_combined_hardpoints,
        )
        file_menu.add_command(
            label="Import Suspension Hardpoints to Steering",
            command=self.import_suspension_hardpoints_to_steering,
        )
        file_menu.add_separator()
        file_menu.add_command(label="Save Project", command=self.save_project)
        file_menu.add_command(
            label="Save Project As",
            command=self.save_project_as,
        )
        file_menu.add_separator()
        file_menu.add_command(label="Close", command=self.root.destroy)
        menu.add_cascade(label="File", menu=file_menu)
        self.root.config(menu=menu)

    def _active_page(self) -> object:
        selected = self.notebook.select()
        return self.pages[selected]

    def import_hardpoints(self) -> None:
        self._call_active_page("import_hardpoints")

    def open_project(self) -> None:
        self._call_active_page("open_project")

    def export_hardpoints(self) -> None:
        self._call_active_page("export_hardpoints")

    def export_report(self) -> None:
        suspension_page = self._page_by_type(SuspensionWorkbenchPage)
        steering_page = self._page_by_type(SteeringWorkbenchApp)
        if suspension_page is None and steering_page is None:
            messagebox.showerror(
                "Unavailable",
                "Report export requires at least one GUI page.",
            )
            return
        dialog = _ReportExportDialog(
            self.root,
            has_suspension=suspension_page is not None,
            has_steering=steering_page is not None,
            suspension_outputs=(
                tuple(suspension_page.curve_manager.outputs)
                if suspension_page is not None
                else ()
            ),
            steering_outputs=(
                tuple(steering_page.curve_manager.outputs)
                if steering_page is not None
                else ()
            ),
            suspension_initial_curves=(
                list(suspension_page.curve_manager.curves)
                if suspension_page is not None
                else []
            ),
            steering_initial_curves=(
                list(steering_page.curve_manager.curves)
                if steering_page is not None
                else []
            ),
        )
        self.root.wait_window(dialog)
        if dialog.result is None:
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".docx",
            filetypes=[("Word document", "*.docx")],
        )
        if not path:
            return
        try:
            if (
                dialog.result.scope in {"suspension", "combined"}
                and suspension_page is not None
                and not suspension_page._sync_controls_to_project()
            ):
                return
            if (
                dialog.result.scope in {"steering", "combined"}
                and steering_page is not None
                and not steering_page._sync_controls_to_project()
            ):
                return
            export_gui_report_docx(
                path,
                options=dialog.result,
                suspension_project=(
                    suspension_page.project if suspension_page is not None else None
                ),
                steering_project=(
                    steering_page.project if steering_page is not None else None
                ),
                suspension_source_path=(
                    suspension_page.project_path
                    if suspension_page is not None
                    else None
                ),
                steering_source_path=(
                    steering_page.project_path if steering_page is not None else None
                ),
            )
        except Exception as exc:
            messagebox.showerror("Report export failed", str(exc))
            return

    def export_combined_hardpoints(self) -> None:
        steering_page = self._page_by_type(SteeringWorkbenchApp)
        suspension_page = self._page_by_type(SuspensionWorkbenchPage)
        if steering_page is None or suspension_page is None:
            messagebox.showerror(
                "Unavailable",
                "Combined export requires both steering and suspension pages.",
            )
            return
        if not steering_page._sync_controls_to_project():
            return
        if not suspension_page._sync_controls_to_project():
            return
        suspension_items = suspension_export_hardpoints(
            suspension_page.project.hardpoints
        )
        steering_items = steering_export_hardpoints(steering_page.project.hardpoints)
        conflicts = detect_hardpoint_conflicts(suspension_items, steering_items)
        choices = self._prompt_merge_choices(conflicts)
        if choices is None:
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
        )
        if not path:
            return
        try:
            rows = merge_export_hardpoints(
                suspension_items,
                steering_items,
                choices=choices,
            )
            save_merged_hardpoints_csv(rows, path)
        except Exception as exc:
            messagebox.showerror("Export failed", str(exc))

    def import_suspension_hardpoints_to_steering(self) -> None:
        """Load the current suspension corner into the steering workbench."""
        steering_page = self._page_by_type(SteeringWorkbenchApp)
        suspension_page = self._page_by_type(SuspensionWorkbenchPage)
        if steering_page is None or suspension_page is None:
            messagebox.showerror(
                "Unavailable",
                "Suspension import requires both steering and suspension pages.",
            )
            return
        if not suspension_page._sync_controls_to_project():
            return

        dialog = _SteeringLinkageTypeDialog(
            self.root,
            initial_linkage_type=steering_page.project.linkage_type,
        )
        self.root.wait_window(dialog)
        if dialog.result is None:
            return
        linkage_type = dialog.result

        try:
            suspension = suspension_page.project.build_suspension()
            design_state = suspension.initial_state()
            wheel_center = design_state.get(PointID.WHEEL_CENTER)
            if steering_page.project.linkage_type == linkage_type:
                existing_rows = steering_page.project.hardpoints
            else:
                existing_rows = default_hardpoint_rows(linkage_type)
            steering_rows = steering_rows_from_suspension_hardpoints(
                design_state.positions,
                wheel_center=wheel_center,
                existing_rows=existing_rows,
                linkage_type=linkage_type,
            )
        except Exception as exc:
            messagebox.showerror("Suspension import failed", str(exc))
            return

        tire = suspension_page.project.config.wheel.tire
        defaults = default_steering_project(linkage_type=linkage_type)
        steering_page.project.linkage_type = linkage_type
        steering_page.project.hardpoints = steering_rows
        steering_page.project.input_mode = defaults.input_mode
        steering_page.project.input_value = 0.0
        steering_page.project.static_radius_mm = float(tire.static_radius_mm)
        steering_page.project.section_width = float(tire.section_width)
        steering_page.imported_default_hardpoints = copy_hardpoint_rows(steering_rows)
        steering_page.pending_optimized_hardpoints = None
        steering_page._reset_refresh_caches()
        steering_page._load_project_to_controls()
        steering_page.refresh()
        steering_tab = next(
            tab_id for tab_id, page in self.pages.items() if page is steering_page
        )
        self.notebook.select(steering_tab)

    def save_project(self) -> None:
        self._call_active_page("save_project")

    def save_project_as(self) -> None:
        self._call_active_page("save_project_as")

    def _call_active_page(self, method_name: str) -> None:
        page = self._active_page()
        method = getattr(page, method_name, None)
        if method is None:
            messagebox.showerror("Unavailable", f"Active page cannot {method_name}")
            return
        method()

    def _page_by_type(self, page_type: type[object]) -> object | None:
        for page in self.pages.values():
            if isinstance(page, page_type):
                return page
        return None

    def _prompt_merge_choices(self, conflicts):
        if not conflicts:
            return {}
        dialog = _CombinedExportConflictDialog(self.root, conflicts)
        self.root.wait_window(dialog)
        return dialog.result


class _CurveLike(Protocol):
    x_output: str
    y_output: str
    label: str


class _ReportCurveSelector(ttk.Frame):
    """Table-based selector for report curve combinations."""

    def __init__(
        self,
        master: tk.Misc,
        *,
        outputs: tuple[str, ...],
        initial_curves: list[_CurveLike],
    ) -> None:
        super().__init__(master)
        self.outputs = outputs
        self.rows = [
            ReportCurveSelection(
                x_output=curve.x_output,
                y_output=curve.y_output,
                label=curve.label,
            )
            for curve in initial_curves
        ]
        default_x = outputs[0] if outputs else ""
        default_y = outputs[1] if len(outputs) > 1 else default_x
        self.x_var = tk.StringVar(value=default_x)
        self.y_var = tk.StringVar(value=default_y)
        self.control_widgets: list[tk.Widget] = []
        self._build()
        self._refresh()

    def _build(self) -> None:
        controls = ttk.Frame(self)
        controls.pack(fill=tk.X)
        ttk.Label(controls, text="X Output").grid(row=0, column=0, sticky="w")
        x_combo = ttk.Combobox(
            controls,
            textvariable=self.x_var,
            values=self.outputs,
            state="readonly",
            width=24,
        )
        x_combo.grid(row=0, column=1, sticky="ew", padx=(6, 0))
        ttk.Label(controls, text="Y Output").grid(
            row=0,
            column=2,
            sticky="w",
            padx=(10, 0),
        )
        y_combo = ttk.Combobox(
            controls,
            textvariable=self.y_var,
            values=self.outputs,
            state="readonly",
            width=24,
        )
        y_combo.grid(row=0, column=3, sticky="ew", padx=(6, 0))
        add_button = ttk.Button(controls, text="+", width=4, command=self.add_curve)
        add_button.grid(row=0, column=4, padx=(10, 0))
        delete_button = ttk.Button(
            controls,
            text="-",
            width=4,
            command=self.delete_selected,
        )
        delete_button.grid(row=0, column=5, padx=(6, 0))
        up_button = ttk.Button(controls, text="Up", command=self.move_up)
        up_button.grid(row=0, column=6, padx=(6, 0))
        down_button = ttk.Button(controls, text="Down", command=self.move_down)
        down_button.grid(row=0, column=7, padx=(6, 0))
        controls.columnconfigure(1, weight=1)
        controls.columnconfigure(3, weight=1)
        self.control_widgets.extend(
            [x_combo, y_combo, add_button, delete_button, up_button, down_button]
        )

        table_frame = ttk.Frame(self)
        table_frame.pack(fill=tk.BOTH, expand=True, pady=(8, 0))
        self.tree = ttk.Treeview(
            table_frame,
            columns=("order", "x_output", "y_output", "label"),
            show="headings",
            height=6,
        )
        self.tree.heading("order", text="#")
        self.tree.heading("x_output", text="X Output")
        self.tree.heading("y_output", text="Y Output")
        self.tree.heading("label", text="Curve")
        self.tree.column("order", width=36, anchor="center", stretch=False)
        self.tree.column("x_output", width=160)
        self.tree.column("y_output", width=160)
        self.tree.column("label", width=220)
        scrollbar = ttk.Scrollbar(
            table_frame,
            orient=tk.VERTICAL,
            command=self.tree.yview,
        )
        self.tree.configure(yscrollcommand=scrollbar.set)
        self.tree.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        table_frame.rowconfigure(0, weight=1)
        table_frame.columnconfigure(0, weight=1)

    def add_curve(self) -> None:
        if not self.outputs:
            return
        self.rows.append(
            ReportCurveSelection(
                x_output=self.x_var.get(),
                y_output=self.y_var.get(),
            )
        )
        self._refresh()
        self._select_index(len(self.rows) - 1)

    def delete_selected(self) -> None:
        selected_index = self._selected_index()
        if selected_index is None:
            return
        del self.rows[selected_index]
        self._refresh()
        if self.rows:
            self._select_index(min(selected_index, len(self.rows) - 1))

    def move_up(self) -> None:
        selected_index = self._selected_index()
        if selected_index is None or selected_index == 0:
            return
        self.rows[selected_index - 1], self.rows[selected_index] = (
            self.rows[selected_index],
            self.rows[selected_index - 1],
        )
        self._refresh()
        self._select_index(selected_index - 1)

    def move_down(self) -> None:
        selected_index = self._selected_index()
        if selected_index is None or selected_index >= len(self.rows) - 1:
            return
        self.rows[selected_index + 1], self.rows[selected_index] = (
            self.rows[selected_index],
            self.rows[selected_index + 1],
        )
        self._refresh()
        self._select_index(selected_index + 1)

    def selections(self) -> tuple[ReportCurveSelection, ...]:
        return tuple(self.rows)

    def set_enabled(self, enabled: bool) -> None:
        state = "readonly" if enabled else tk.DISABLED
        button_state = tk.NORMAL if enabled else tk.DISABLED
        for widget in self.control_widgets[:2]:
            widget.configure(state=state)
        for widget in self.control_widgets[2:]:
            widget.configure(state=button_state)

    def _refresh(self) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)
        for index, curve in enumerate(self.rows, start=1):
            label = curve.label.strip() or f"{curve.y_output} vs {curve.x_output}"
            self.tree.insert(
                "",
                "end",
                iid=str(index - 1),
                values=(index, curve.x_output, curve.y_output, label),
            )

    def _selected_index(self) -> int | None:
        selection = self.tree.selection()
        if not selection:
            return None
        return int(selection[0])

    def _select_index(self, index: int) -> None:
        item_id = str(index)
        self.tree.selection_set(item_id)
        self.tree.focus(item_id)
        self.tree.see(item_id)


class _ReportExportDialog(tk.Toplevel):
    """Dialog for selecting report scope, images, and curve combinations."""

    def __init__(
        self,
        master: tk.Misc,
        *,
        has_suspension: bool,
        has_steering: bool,
        suspension_outputs: tuple[str, ...],
        steering_outputs: tuple[str, ...],
        suspension_initial_curves: list[_CurveLike],
        steering_initial_curves: list[_CurveLike],
    ) -> None:
        super().__init__(master)
        self.title("Export Report")
        self.transient(master)
        self.resizable(True, True)
        self.result: ReportExportOptions | None = None
        self.scope_var = tk.StringVar(
            value=_default_report_scope(
                has_suspension=has_suspension,
                has_steering=has_steering,
            )
        )
        self.image_vars = {
            "suspension_preview": tk.BooleanVar(value=has_suspension),
            "steering_preview": tk.BooleanVar(value=has_steering),
        }
        self.image_checks: dict[str, ttk.Checkbutton] = {}
        self.has_suspension = has_suspension
        self.has_steering = has_steering
        self.suspension_curve_selector: _ReportCurveSelector | None = None
        self.steering_curve_selector: _ReportCurveSelector | None = None
        self.suspension_outputs = suspension_outputs
        self.steering_outputs = steering_outputs
        self.suspension_initial_curves = suspension_initial_curves
        self.steering_initial_curves = steering_initial_curves
        self._build()
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _build(self) -> None:
        body = ttk.Frame(self, padding=12)
        body.pack(fill=tk.BOTH, expand=True)
        body.columnconfigure(0, weight=1)
        ttk.Label(
            body,
            text=(
                "Choose the report scope, preview images, and curve combinations to "
                "include in the Word report. Use the X and Y dropdowns with the "
                "table below to add, remove, and reorder exported curves. Selected "
                "curves always export with their own figures and descriptions."
            ),
            wraplength=760,
            justify=tk.LEFT,
        ).grid(row=0, column=0, sticky="ew")

        scope_frame = ttk.LabelFrame(body, text="Scope", padding=8)
        scope_frame.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        scope_options = (
            ("suspension", "Suspension", self.has_suspension),
            ("steering", "Steering", self.has_steering),
            (
                "combined",
                "Suspension + Steering",
                self.has_suspension and self.has_steering,
            ),
        )
        for row_index, (value, label, enabled) in enumerate(scope_options):
            ttk.Radiobutton(
                scope_frame,
                text=label,
                value=value,
                variable=self.scope_var,
                state=tk.NORMAL if enabled else tk.DISABLED,
                command=self._sync_image_states,
            ).grid(row=row_index, column=0, sticky="w", pady=2)

        image_frame = ttk.LabelFrame(body, text="Images", padding=8)
        image_frame.grid(row=2, column=0, sticky="ew", pady=(10, 0))
        image_specs = (
            ("suspension_preview", "Suspension Preview"),
            ("steering_preview", "Steering Preview"),
        )
        for row_index, (key, label) in enumerate(image_specs):
            check = ttk.Checkbutton(
                image_frame,
                text=label,
                variable=self.image_vars[key],
            )
            check.grid(row=row_index, column=0, sticky="w", pady=2)
            self.image_checks[key] = check

        curve_frame = ttk.LabelFrame(body, text="Curves", padding=8)
        curve_frame.grid(row=3, column=0, sticky="nsew", pady=(10, 0))
        curve_frame.columnconfigure(0, weight=1)
        body.rowconfigure(3, weight=1)
        self.suspension_curve_selector = self._build_curve_group(
            curve_frame,
            title="Suspension Curves",
            outputs=self.suspension_outputs,
            initial_curves=self.suspension_initial_curves,
            row=0,
        )
        self.steering_curve_selector = self._build_curve_group(
            curve_frame,
            title="Steering Curves",
            outputs=self.steering_outputs,
            initial_curves=self.steering_initial_curves,
            row=1,
        )

        buttons = ttk.Frame(body)
        buttons.grid(row=4, column=0, sticky="e", pady=(12, 0))
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(side=tk.RIGHT)
        ttk.Button(buttons, text="Export", command=self._confirm).pack(
            side=tk.RIGHT,
            padx=(0, 8),
        )
        self._sync_image_states()

    def _build_curve_group(
        self,
        parent: ttk.LabelFrame,
        *,
        title: str,
        outputs: tuple[str, ...],
        initial_curves: list[_CurveLike],
        row: int,
    ) -> _ReportCurveSelector | None:
        group = ttk.LabelFrame(parent, text=title, padding=6)
        group.grid(row=row, column=0, sticky="nsew", pady=(0, 8) if row == 0 else 0)
        group.columnconfigure(0, weight=1)
        group.rowconfigure(0, weight=1)
        if not outputs:
            ttk.Label(group, text="No curve outputs available.").grid(
                row=0,
                column=0,
                sticky="w",
            )
            return None
        selector = _ReportCurveSelector(
            group,
            outputs=outputs,
            initial_curves=initial_curves,
        )
        selector.grid(row=0, column=0, sticky="nsew")
        return selector

    def _sync_image_states(self) -> None:
        scope = self.scope_var.get()
        enable_suspension = scope in {"suspension", "combined"}
        enable_steering = scope in {"steering", "combined"}
        for key in ("suspension_preview",):
            if not enable_suspension:
                self.image_vars[key].set(False)
            self.image_checks[key].configure(
                state=tk.NORMAL if enable_suspension else tk.DISABLED
            )
        for key in ("steering_preview",):
            if not enable_steering:
                self.image_vars[key].set(False)
            self.image_checks[key].configure(
                state=tk.NORMAL if enable_steering else tk.DISABLED
            )
        if self.suspension_curve_selector is not None:
            self.suspension_curve_selector.set_enabled(enable_suspension)
        if self.steering_curve_selector is not None:
            self.steering_curve_selector.set_enabled(enable_steering)

    def _confirm(self) -> None:
        scope = self.scope_var.get()
        include_images = tuple(
            key for key, variable in self.image_vars.items() if variable.get()
        )
        self.result = ReportExportOptions(
            scope=scope,
            include_images=include_images,
            suspension_curves=(
                self.suspension_curve_selector.selections()
                if self.suspension_curve_selector is not None
                else None
            ),
            steering_curves=(
                self.steering_curve_selector.selections()
                if self.steering_curve_selector is not None
                else None
            ),
        )
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class _SteeringLinkageTypeDialog(tk.Toplevel):
    """Dialog that selects the target steering linkage type for import."""

    LINKAGE_LABELS = {
        "two_segment": "Two segment",
        "three_segment": "Three segment",
        "rack_and_pinion": "Rack and pinion",
    }

    def __init__(
        self,
        master: tk.Misc,
        *,
        initial_linkage_type: str = "two_segment",
    ) -> None:
        super().__init__(master)
        self.title("Import to Steering")
        self.transient(master)
        self.resizable(False, False)
        self.result: str | None = None
        default = (
            initial_linkage_type
            if initial_linkage_type in LINKAGE_TYPES
            else LINKAGE_TYPES[0]
        )
        self.linkage_type_var = tk.StringVar(value=default)
        self._build()
        self.protocol("WM_DELETE_WINDOW", self._cancel)
        self.grab_set()
        self.focus_set()

    def _build(self) -> None:
        body = ttk.Frame(self, padding=12)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            body,
            text="Select the steering linkage type that should receive the suspension hardpoints.",
            justify=tk.LEFT,
            wraplength=360,
        ).grid(row=0, column=0, columnspan=2, sticky="w")
        ttk.Label(body, text="Linkage type").grid(
            row=1,
            column=0,
            sticky="w",
            pady=(12, 0),
        )
        values = [
            f"{linkage} — {self.LINKAGE_LABELS.get(linkage, linkage)}"
            for linkage in LINKAGE_TYPES
        ]
        display_var = tk.StringVar(
            value=f"{self.linkage_type_var.get()} — "
            f"{self.LINKAGE_LABELS.get(self.linkage_type_var.get(), self.linkage_type_var.get())}"
        )
        combo = ttk.Combobox(
            body,
            textvariable=display_var,
            values=values,
            state="readonly",
            width=36,
        )
        combo.grid(row=1, column=1, sticky="ew", padx=(8, 0), pady=(12, 0))

        def _on_select(_event: object | None = None) -> None:
            selected = display_var.get().split(" — ", 1)[0]
            if selected in LINKAGE_TYPES:
                self.linkage_type_var.set(selected)

        combo.bind("<<ComboboxSelected>>", _on_select)
        buttons = ttk.Frame(body)
        buttons.grid(row=2, column=0, columnspan=2, sticky="e", pady=(16, 0))
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(
            side=tk.RIGHT,
            padx=(6, 0),
        )
        ttk.Button(buttons, text="Import", command=self._accept).pack(side=tk.RIGHT)

    def _accept(self) -> None:
        linkage_type = self.linkage_type_var.get()
        if linkage_type not in LINKAGE_TYPES:
            return
        self.result = linkage_type
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()


class _CombinedExportConflictDialog(tk.Toplevel):
    """Dialog that lets users pick how overlapping hardpoints are merged."""

    def __init__(self, master: tk.Misc, conflicts) -> None:
        super().__init__(master)
        self.title("Merge Hardpoints")
        self.transient(master)
        self.resizable(False, False)
        self.result = None
        self._choice_vars: dict[str, tk.StringVar] = {}
        self._build(conflicts)
        self.protocol("WM_DELETE_WINDOW", self._cancel)

    def _build(self, conflicts) -> None:
        body = ttk.Frame(self, padding=12)
        body.pack(fill=tk.BOTH, expand=True)
        ttk.Label(
            body,
            text=(
                "Select which coordinates to keep for overlapping steering and "
                "suspension hardpoints."
            ),
            justify=tk.LEFT,
            wraplength=520,
        ).grid(row=0, column=0, columnspan=4, sticky="w")
        headers = ("Point", "Suspension", "Steering", "Use")
        for column, label in enumerate(headers):
            ttk.Label(body, text=label).grid(
                row=1,
                column=column,
                sticky="w",
                padx=(0, 8),
                pady=(8, 4),
            )
        for row_index, conflict in enumerate(conflicts, start=2):
            ttk.Label(body, text=conflict.display_name).grid(
                row=row_index,
                column=0,
                sticky="w",
                padx=(0, 8),
                pady=2,
            )
            ttk.Label(
                body,
                text=self._format_vec3(conflict.suspension_position),
            ).grid(row=row_index, column=1, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(
                body,
                text=self._format_vec3(conflict.steering_position),
            ).grid(row=row_index, column=2, sticky="w", padx=(0, 8), pady=2)
            choice = tk.StringVar(value="suspension")
            self._choice_vars[conflict.export_name] = choice
            ttk.Combobox(
                body,
                textvariable=choice,
                values=("suspension", "steering", "average"),
                state="readonly",
                width=12,
            ).grid(row=row_index, column=3, sticky="w", pady=2)
        buttons = ttk.Frame(body)
        buttons.grid(
            row=len(conflicts) + 2,
            column=0,
            columnspan=4,
            sticky="e",
            pady=(12, 0),
        )
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(side=tk.RIGHT)
        ttk.Button(buttons, text="Export", command=self._confirm).pack(
            side=tk.RIGHT,
            padx=(0, 8),
        )

    def _confirm(self) -> None:
        self.result = {
            export_name: choice_var.get()
            for export_name, choice_var in self._choice_vars.items()
        }
        self.destroy()

    def _cancel(self) -> None:
        self.result = None
        self.destroy()

    @staticmethod
    def _format_vec3(value) -> str:
        return f"({float(value[0]):.6g}, {float(value[1]):.6g}, {float(value[2]):.6g})"


def _default_report_scope(
    *,
    has_suspension: bool,
    has_steering: bool,
) -> str:
    if has_suspension and has_steering:
        return "combined"
    if has_suspension:
        return "suspension"
    return "steering"


def main() -> None:
    """Run the unified kinematics GUI."""
    if "--smoke-test" in sys.argv:
        smoke_test()
        os._exit(0)
    root = tk.Tk()
    KinematicsWorkbenchApp(root)
    root.mainloop()


def smoke_test() -> None:
    """Create and destroy the workbench once for executable packaging checks."""
    _write_smoke_log("start")
    root = tk.Tk()
    _write_smoke_log("root-created")
    root.withdraw()
    _write_smoke_log("root-hidden")
    app = KinematicsWorkbenchApp(root)
    _write_smoke_log("app-created")
    notebook = app.notebook
    _write_smoke_log(f"tabs={len(notebook.tabs())}")
    for tab_id in notebook.tabs():
        notebook.select(tab_id)
        root.update_idletasks()
        _write_smoke_log(f"selected-tab={notebook.tab(tab_id, 'text')}")
    root.update_idletasks()
    _write_smoke_log("idle-updated")
    root.destroy()
    _write_smoke_log("destroyed")


def _write_smoke_log(message: str) -> None:
    log_path = os.environ.get("KINEMATICS_GUI_SMOKE_LOG")
    if log_path:
        with open(log_path, "a", encoding="utf-8") as log_file:
            log_file.write(f"{message}\n")


if __name__ == "__main__":
    main()
