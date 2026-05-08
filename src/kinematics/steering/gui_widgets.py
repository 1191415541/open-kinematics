"""
Tk widgets for editing and inspecting steering workbench state.
"""

from __future__ import annotations

import tkinter as tk
from tkinter import ttk
from typing import Callable

from kinematics.steering.workbench import (
    SteeringCurve,
    SteeringHardpointRow,
    parse_float_entry,
    pitman_arm_x_length,
    pitman_x_position,
    set_pitman_arm_x_length,
    set_pitman_x_position,
)


class HardpointEditor(ttk.Frame):
    """Tree and coordinate editor for steering hardpoints."""

    def __init__(
        self,
        master: tk.Misc,
        on_change: Callable[[], None],
    ) -> None:
        super().__init__(master)
        self.on_change = on_change
        self.rows: list[SteeringHardpointRow] = []
        self.selected_index: int | None = None
        self.vars = {axis: tk.StringVar() for axis in ("x", "y", "z")}
        self._build()

    def _build(self) -> None:
        columns = ("category", "name", "x", "y", "z")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=12)
        for column in columns:
            self.tree.heading(column, text=column)
            self.tree.column(column, width=86, anchor="center")
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.bind("<<TreeviewSelect>>", self._on_select)

        form = ttk.Frame(self)
        form.pack(fill=tk.X, pady=(6, 0))
        for axis in ("x", "y", "z"):
            ttk.Label(form, text=axis.upper()).pack(side=tk.LEFT)
            entry = ttk.Entry(form, textvariable=self.vars[axis], width=10)
            entry.pack(side=tk.LEFT, padx=(2, 8))
            entry.bind("<KeyRelease>", self._on_entry)

    def set_rows(self, rows: list[SteeringHardpointRow]) -> None:
        """Load rows into the editor."""
        self.rows = rows
        self.selected_index = None
        for item in self.tree.get_children():
            self.tree.delete(item)
        for index, row in enumerate(rows):
            self.tree.insert(
                "",
                "end",
                iid=str(index),
                values=(row.category, row.name, row.x, row.y, row.z),
            )
        if rows:
            self.tree.selection_set("0")

    def _on_select(self, _event: tk.Event) -> None:
        selection = self.tree.selection()
        if not selection:
            return
        self.selected_index = int(selection[0])
        row = self.rows[self.selected_index]
        for axis in ("x", "y", "z"):
            self.vars[axis].set(str(getattr(row, axis)))

    def _on_entry(self, _event: tk.Event) -> None:
        if self.selected_index is None:
            return
        try:
            values = {axis: float(var.get()) for axis, var in self.vars.items()}
        except ValueError:
            return
        row = self.rows[self.selected_index]
        row.x = values["x"]
        row.y = values["y"]
        row.z = values["z"]
        self.tree.item(
            str(self.selected_index),
            values=(row.category, row.name, row.x, row.y, row.z),
        )
        self.on_change()


class OutputTable(ttk.Frame):
    """Table for scalar steering outputs."""

    def __init__(self, master: tk.Misc) -> None:
        super().__init__(master)
        self.tree = ttk.Treeview(self, columns=("name", "value"), show="headings")
        self.tree.heading("name", text="output")
        self.tree.heading("value", text="value")
        self.tree.column("name", width=190)
        self.tree.column("value", width=120, anchor="e")
        self.tree.pack(fill=tk.BOTH, expand=True)

    def set_outputs(self, outputs: dict[str, float]) -> None:
        """Refresh output rows."""
        for item in self.tree.get_children():
            self.tree.delete(item)
        for name, value in outputs.items():
            self.tree.insert("", "end", values=(name, f"{value:.6g}"))

    def set_error(self, message: str) -> None:
        """Show one error row."""
        for item in self.tree.get_children():
            self.tree.delete(item)
        self.tree.insert("", "end", values=("error", message))


class PitmanTransformControls(ttk.LabelFrame):
    """Controls for editing pitman helper geometry."""

    def __init__(
        self,
        master: tk.Misc,
        on_change: Callable[[], None],
    ) -> None:
        super().__init__(master, text="Pitman Geometry", padding=6)
        self.on_change = on_change
        self.rows: list[SteeringHardpointRow] = []
        self.updating = False
        self.x_var = tk.StringVar()
        self.length_var = tk.StringVar()
        self._build()

    def _build(self) -> None:
        self.columnconfigure(1, weight=1)
        for row_index, (label, var) in enumerate(
            (
                ("Pitman X", self.x_var),
                ("Arm X length", self.length_var),
            )
        ):
            ttk.Label(self, text=label).grid(row=row_index, column=0, sticky="w")
            entry = ttk.Entry(self, textvariable=var, width=12)
            entry.grid(row=row_index, column=1, sticky="ew", padx=(6, 0), pady=2)
            entry.bind("<KeyRelease>", self._on_entry)

    def set_rows(self, rows: list[SteeringHardpointRow]) -> None:
        """Load current hardpoint rows into the transform controls."""
        self.rows = rows
        self.updating = True
        try:
            self.x_var.set(str(pitman_x_position(rows)))
            self.length_var.set(str(pitman_arm_x_length(rows)))
        finally:
            self.updating = False

    def _on_entry(self, _event: tk.Event) -> None:
        if self.updating or not self.rows:
            return
        x_value = parse_float_entry(self.x_var.get(), pitman_x_position(self.rows))
        length = parse_float_entry(
            self.length_var.get(),
            pitman_arm_x_length(self.rows),
        )
        if not x_value.is_valid or not length.is_valid:
            return
        if not x_value.is_complete or not length.is_complete:
            return
        set_pitman_x_position(self.rows, x_value.value)
        set_pitman_arm_x_length(self.rows, length.value)
        self.on_change()


class CurveManager(ttk.Frame):
    """Curve creation and deletion controls."""

    def __init__(
        self,
        master: tk.Misc,
        outputs: tuple[str, ...],
        on_change: Callable[[], None],
    ) -> None:
        super().__init__(master)
        self.outputs = outputs
        self.on_change = on_change
        self.curves: list[SteeringCurve] = []
        self.x_var = tk.StringVar(value=outputs[0])
        self.y_var = tk.StringVar(value=outputs[2])
        self.label_var = tk.StringVar()
        self._build()
        self._bind_selection_changes()

    def _build(self) -> None:
        top = ttk.Frame(self)
        top.pack(fill=tk.X)
        top.columnconfigure(1, weight=1)
        ttk.Label(top, text="X").grid(row=0, column=0, sticky="w")
        ttk.Combobox(
            top,
            textvariable=self.x_var,
            values=self.outputs,
            width=24,
        ).grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Label(top, text="Y").grid(row=1, column=0, sticky="w", pady=(4, 0))
        ttk.Combobox(
            top,
            textvariable=self.y_var,
            values=self.outputs,
            width=24,
        ).grid(row=1, column=1, sticky="ew", padx=4, pady=(4, 0))
        ttk.Entry(top, textvariable=self.label_var, width=18).grid(
            row=2,
            column=1,
            sticky="ew",
            padx=4,
            pady=(4, 0),
        )
        ttk.Button(top, text="Add", command=self.add_curve).grid(
            row=2,
            column=2,
            padx=(4, 0),
            pady=(4, 0),
        )
        ttk.Button(top, text="Delete", command=self.delete_selected).grid(
            row=2,
            column=3,
            padx=(4, 0),
            pady=(4, 0),
        )

        self.listbox = tk.Listbox(self, height=5)
        self.listbox.pack(fill=tk.BOTH, expand=True, pady=(6, 0))

    def _bind_selection_changes(self) -> None:
        for var in (self.x_var, self.y_var, self.label_var):
            var.trace_add("write", self._on_selection_changed)

    def _on_selection_changed(self, *_args: object) -> None:
        self.on_change()

    def set_curves(self, curves: list[SteeringCurve]) -> None:
        """Load curves into the manager."""
        self.curves = curves
        self._refresh()

    def add_curve(self) -> None:
        """Add a curve definition."""
        self.curves.append(
            SteeringCurve(
                x_output=self.x_var.get(),
                y_output=self.y_var.get(),
                label=self.label_var.get(),
            )
        )
        self.label_var.set("")
        self._refresh()
        self.on_change()

    def delete_selected(self) -> None:
        """Delete the selected curve definition."""
        selection = self.listbox.curselection()
        if not selection:
            return
        del self.curves[selection[0]]
        self._refresh()
        self.on_change()

    def _refresh(self) -> None:
        self.listbox.delete(0, tk.END)
        for curve in self.curves:
            label = curve.label or f"{curve.y_output} vs {curve.x_output}"
            self.listbox.insert(tk.END, label)
