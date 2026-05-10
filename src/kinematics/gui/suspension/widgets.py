"""Tk widgets for the suspension workbench page."""

from __future__ import annotations

import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

import numpy as np

from kinematics.core.enums import PointID
from kinematics.gui.common import bind_entry_commit_events
from kinematics.suspensions.base import Suspension


class HardpointTable(ttk.Frame):
    """Editable suspension hardpoint table."""

    MIN_POINT_COLUMN_WIDTH = 94
    MIN_VALUE_COLUMN_WIDTH = 56
    DISPLAY_NAMES = {
        PointID.LOWER_WISHBONE_INBOARD_FRONT: "LWB_IN_F",
        PointID.LOWER_WISHBONE_INBOARD_REAR: "LWB_IN_R",
        PointID.LOWER_WISHBONE_OUTBOARD: "LWB_OUT",
        PointID.UPPER_WISHBONE_INBOARD_FRONT: "UWB_IN_F",
        PointID.UPPER_WISHBONE_INBOARD_REAR: "UWB_IN_R",
        PointID.UPPER_WISHBONE_OUTBOARD: "UWB_OUT",
        PointID.TRACKROD_INBOARD: "TR_IN",
        PointID.TRACKROD_OUTBOARD: "TR_OUT",
        PointID.AXLE_INBOARD: "AX_IN",
        PointID.AXLE_OUTBOARD: "AX_OUT",
        PointID.CARRIER_STEERING_AXIS_LOWER: "CAR_AX_LO",
        PointID.CARRIER_STEERING_AXIS_UPPER: "CAR_AX_UP",
    }

    def __init__(self, master: tk.Misc, on_change=None) -> None:
        super().__init__(master)
        self.on_change = on_change
        self.hardpoints: dict[PointID, np.ndarray] = {}
        self.selected_point: PointID | None = None
        self.vars = {axis: tk.StringVar() for axis in ("x", "y", "z")}
        columns = ("point", "x", "y", "z")
        self.tree = ttk.Treeview(self, columns=columns, show="headings", height=10)
        for column in columns:
            self.tree.heading(column, text=column)
            self.tree.column(column, anchor="center", minwidth=48)
        self._tree_font = tkfont.nametofont("TkDefaultFont")
        self._sync_column_widths()
        self.tree.pack(fill=tk.BOTH, expand=True)
        self.tree.bind("<<TreeviewSelect>>", self._on_select)

        form = ttk.Frame(self)
        form.pack(fill=tk.X, pady=(6, 0))
        for axis in ("x", "y", "z"):
            ttk.Label(form, text=axis.upper()).pack(side=tk.LEFT)
            entry = ttk.Entry(form, textvariable=self.vars[axis], width=10)
            entry.pack(side=tk.LEFT, padx=(2, 8))
            bind_entry_commit_events(
                entry,
                on_live_edit=self._on_entry_live_edit,
                on_commit=self._on_entry_commit,
            )

    def set_suspension(self, suspension: Suspension | None) -> None:
        """Refresh table rows from a loaded suspension."""
        self.set_hardpoints({} if suspension is None else suspension.hardpoints)

    def set_hardpoints(self, hardpoints: dict[PointID, np.ndarray]) -> None:
        """Refresh table rows from editable hardpoint data."""
        self.hardpoints = hardpoints
        self.selected_point = None
        for item in self.tree.get_children():
            self.tree.delete(item)
        for point_id, position in sorted(hardpoints.items()):
            name = (
                self._display_name(point_id)
                if isinstance(point_id, PointID)
                else str(point_id)
            )
            self.tree.insert(
                "",
                "end",
                iid=str(int(point_id)),
                values=(
                    name,
                    f"{position[0]:.6g}",
                    f"{position[1]:.6g}",
                    f"{position[2]:.6g}",
                ),
            )
        self._sync_column_widths()
        if hardpoints:
            self.tree.selection_set(str(int(sorted(hardpoints)[0])))

    def _on_select(self, _event: tk.Event) -> None:
        selection = self.tree.selection()
        if not selection:
            return
        self.selected_point = PointID(int(selection[0]))
        position = self.hardpoints[self.selected_point]
        for axis, value in zip(("x", "y", "z"), position):
            self.vars[axis].set(f"{value:.6g}")

    def _apply_current_entry_values(self) -> bool:
        if self.selected_point is None:
            return False
        try:
            values = np.asarray(
                [float(self.vars[axis].get()) for axis in ("x", "y", "z")],
                dtype=np.float64,
            )
        except ValueError:
            return False
        self.hardpoints[self.selected_point] = values
        self.tree.item(
            str(int(self.selected_point)),
            values=(
                self._display_name(self.selected_point),
                f"{values[0]:.6g}",
                f"{values[1]:.6g}",
                f"{values[2]:.6g}",
            ),
        )
        return True

    def _on_entry_live_edit(self, _event: tk.Event) -> None:
        self._apply_current_entry_values()

    def _on_entry_commit(self, _event: tk.Event) -> None:
        if self._apply_current_entry_values() and self.on_change is not None:
            self.on_change()

    def _sync_column_widths(self) -> None:
        """Keep the tree compact while fitting current content."""
        point_width = max(
            self.MIN_POINT_COLUMN_WIDTH,
            self._tree_font.measure("point") + 28,
            max(
                (
                    self._tree_font.measure(self._display_name(point_id)) + 18
                    for point_id in self.hardpoints
                ),
                default=0,
            ),
        )
        value_width = max(
            self.MIN_VALUE_COLUMN_WIDTH,
            self._tree_font.measure("0.00000") + 16,
            max(
                (
                    self._tree_font.measure(f"{float(position[axis_index]):.6g}") + 16
                    for position in self.hardpoints.values()
                    for axis_index in range(3)
                ),
                default=0,
            ),
        )
        self.tree.column("point", width=point_width)
        for axis in ("x", "y", "z"):
            self.tree.column(axis, width=value_width)

    def _display_name(self, point_id: PointID) -> str:
        return self.DISPLAY_NAMES.get(point_id, point_id.name)
