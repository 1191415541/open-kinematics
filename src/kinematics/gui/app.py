"""Unified Tkinter GUI for kinematics workflows."""

from __future__ import annotations

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from kinematics.gui.hardpoint_merge import (
    detect_hardpoint_conflicts,
    merge_export_hardpoints,
    save_merged_hardpoints_csv,
    steering_export_hardpoints,
    suspension_export_hardpoints,
)
from kinematics.gui.steering import SteeringWorkbenchApp
from kinematics.gui.suspension import SuspensionWorkbenchPage


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
            label="Export Combined Hardpoints",
            command=self.export_combined_hardpoints,
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
        suspension_items = suspension_export_hardpoints(suspension_page.project.hardpoints)
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
        ttk.Button(buttons, text="Cancel", command=self._cancel).pack(
            side=tk.RIGHT
        )
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
        return (
            f"({float(value[0]):.6g}, {float(value[1]):.6g}, {float(value[2]):.6g})"
        )


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
