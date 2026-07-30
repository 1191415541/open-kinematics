"""
File actions for the Tkinter steering workbench GUI.
"""

from __future__ import annotations

from pathlib import Path
from tkinter import filedialog, messagebox

from suspension_kinematics.steering.workbench import (
    RACK_AND_PINION_LINKAGE_TYPE,
    copy_hardpoint_rows,
    default_steering_project,
    hardpoint_rows_from_csv,
    load_steering_project,
    save_hardpoint_rows_csv,
    save_steering_project,
)

PROJECT_FILETYPES = [("Kinematics project", "*.okproj.json")]


class SteeringFileActions:
    """Mixin with project and hardpoint file actions."""

    def new_project(self) -> None:
        self.project = default_steering_project()
        self.project_path = None
        self.imported_default_hardpoints = copy_hardpoint_rows(self.project.hardpoints)
        self._load_project_to_controls()
        self.refresh()

    def open_project(self) -> None:
        path = filedialog.askopenfilename(
            filetypes=PROJECT_FILETYPES + [("Legacy steering project", "*.json")]
        )
        if not path:
            return
        try:
            self.project = load_steering_project(path)
            self.project_path = Path(path)
            self.imported_default_hardpoints = copy_hardpoint_rows(
                self.project.hardpoints
            )
            self._load_project_to_controls()
            self.refresh()
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            messagebox.showerror("Open failed", str(exc))

    def save_project(self) -> None:
        if self.project_path is None:
            self.save_project_as()
            return
        if self._sync_controls_to_project():
            save_steering_project(self.project, self.project_path)

    def save_project_as(self) -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".okproj.json",
            filetypes=PROJECT_FILETYPES,
        )
        if not path:
            return
        self.project_path = Path(path)
        self.save_project()

    def import_csv(self) -> None:
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv")])
        if not path:
            return
        try:
            imported_rows = hardpoint_rows_from_csv(path)
            self.project.hardpoints = imported_rows
            if self.project.linkage_type == "three_segment":
                self.project.linkage_type = "two_segment"
            elif self.project.linkage_type == RACK_AND_PINION_LINKAGE_TYPE:
                self.project.input_mode = "pinion_angle"
            self.linkage_type_var.set(self.project.linkage_type)
            self.imported_default_hardpoints = copy_hardpoint_rows(imported_rows)
            self.hardpoint_editor.set_rows(self.project.hardpoints)
            self._sync_pitman_controls()
            self.preview_has_drawn = False
            self.refresh()
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            messagebox.showerror("Import failed", str(exc))

    def export_csv(self) -> None:
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv")],
        )
        if not path:
            return
        try:
            save_hardpoint_rows_csv(self.project.hardpoints, path)
        except Exception as exc:  # noqa: BLE001 - show GUI error.
            messagebox.showerror("Export failed", str(exc))
