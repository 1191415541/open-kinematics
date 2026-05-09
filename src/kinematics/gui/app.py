"""Unified Tkinter GUI for kinematics workflows."""

from __future__ import annotations

import tkinter as tk
from tkinter import messagebox, ttk

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


def main() -> None:
    """Run the unified kinematics GUI."""
    root = tk.Tk()
    KinematicsWorkbenchApp(root)
    root.mainloop()
