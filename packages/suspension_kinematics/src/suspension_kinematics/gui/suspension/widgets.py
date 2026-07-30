"""Tk widgets for the suspension workbench page."""

from __future__ import annotations

import tkinter as tk
from collections.abc import Callable, Sequence
from tkinter import messagebox, ttk
from typing import Any

import numpy as np
import tksheet

from suspension_kinematics.core.enums import PointID
from suspension_kinematics.gui.common import bind_entry_commit_events, parse_float_entry
from suspension_kinematics.gui.hardpoint_merge import suspension_display_name
from suspension_kinematics.gui.suspension.workbench import (
    apply_wishbone_inboard_delta,
    suspension_gui_to_internal_vec3,
    suspension_internal_to_gui_vec3,
)
from suspension_kinematics.suspensions.base import Suspension


def _format_coord(value: float) -> str:
    """Return a compact string for one coordinate cell."""
    return f"{value:.15g}"


class InboardMountControls(ttk.LabelFrame):
    """Bulk Y/Z adjustment controls for wishbone inboard hardpoints."""

    def __init__(
        self,
        master: tk.Misc,
        on_change: Callable[[], None],
    ) -> None:
        super().__init__(master, text="Wishbone Inboard Mounts", padding=6)
        self.on_change = on_change
        self.hardpoints: dict[PointID, np.ndarray] = {}
        self.baseline_hardpoints: dict[PointID, np.ndarray] = {}
        self.updating = False
        self.upper_dy_var = tk.StringVar(value="0")
        self.upper_dz_var = tk.StringVar(value="0")
        self.lower_dy_var = tk.StringVar(value="0")
        self.lower_dz_var = tk.StringVar(value="0")
        self._build()

    def _build(self) -> None:
        self.columnconfigure(1, weight=1)
        self.columnconfigure(3, weight=1)
        fields = (
            (0, 0, "Upper ΔY", self.upper_dy_var),
            (0, 2, "Upper ΔZ", self.upper_dz_var),
            (1, 0, "Lower ΔY", self.lower_dy_var),
            (1, 2, "Lower ΔZ", self.lower_dz_var),
        )
        commit_entries: list[ttk.Entry] = []
        for row, column, label, var in fields:
            ttk.Label(self, text=label).grid(row=row, column=column, sticky="w")
            entry = ttk.Entry(self, textvariable=var, width=8)
            entry.grid(
                row=row,
                column=column + 1,
                sticky="ew",
                padx=(6, 12),
                pady=2,
            )
            commit_entries.append(entry)
        for entry in commit_entries:
            bind_entry_commit_events(
                entry,
                on_live_edit=lambda _event: None,
                on_commit=self._on_entry_commit,
            )
        ttk.Button(self, text="Reset", command=self.reset_deltas).grid(
            row=2,
            column=0,
            columnspan=4,
            sticky="e",
            pady=(6, 0),
        )

    def set_hardpoints(self, hardpoints: dict[PointID, np.ndarray]) -> None:
        """Load the current hardpoints as the delta baseline."""
        self.hardpoints = hardpoints
        self.baseline_hardpoints = {
            point_id: np.asarray(position, dtype=np.float64).copy()
            for point_id, position in hardpoints.items()
        }
        self.updating = True
        try:
            self.upper_dy_var.set("0")
            self.upper_dz_var.set("0")
            self.lower_dy_var.set("0")
            self.lower_dz_var.set("0")
        finally:
            self.updating = False

    def reset_deltas(self) -> None:
        """Restore hardpoints to the baseline and clear delta fields."""
        if not self.baseline_hardpoints:
            return
        for point_id, position in self.baseline_hardpoints.items():
            self.hardpoints[point_id] = np.asarray(position, dtype=np.float64).copy()
        self.updating = True
        try:
            self.upper_dy_var.set("0")
            self.upper_dz_var.set("0")
            self.lower_dy_var.set("0")
            self.lower_dz_var.set("0")
        finally:
            self.updating = False
        self.on_change()

    def _apply_current_entry_values(self) -> bool:
        if self.updating or not self.baseline_hardpoints:
            return False
        parsed = {
            "upper_dy": parse_float_entry(self.upper_dy_var.get(), 0.0),
            "upper_dz": parse_float_entry(self.upper_dz_var.get(), 0.0),
            "lower_dy": parse_float_entry(self.lower_dy_var.get(), 0.0),
            "lower_dz": parse_float_entry(self.lower_dz_var.get(), 0.0),
        }
        for value in parsed.values():
            if not value.is_valid or not value.is_complete:
                return False
        updated = apply_wishbone_inboard_delta(
            self.baseline_hardpoints,
            upper_dy_mm=float(parsed["upper_dy"].value),
            upper_dz_mm=float(parsed["upper_dz"].value),
            lower_dy_mm=float(parsed["lower_dy"].value),
            lower_dz_mm=float(parsed["lower_dz"].value),
            gui_coordinates=True,
        )
        for point_id, position in updated.items():
            self.hardpoints[point_id] = position
        return True

    def _on_entry_commit(self, _event: tk.Event) -> None:
        if self._apply_current_entry_values():
            self.on_change()


class HardpointTable(ttk.Frame):
    """Editable suspension hardpoint sheet with spreadsheet-style interactions."""

    COLUMNS = ("Point", "X", "Y", "Z")

    def __init__(self, master: tk.Misc, on_change=None) -> None:
        super().__init__(master)
        self.on_change = on_change
        self.hardpoints: dict[PointID, np.ndarray] = {}
        self._row_order: list[PointID] = []
        self._suppress_sheet_events = False
        self._build()

    def _build(self) -> None:
        self.sheet = tksheet.Sheet(
            self,
            headers=list(self.COLUMNS),
            show_row_index=False,
            show_top_left=False,
            show_x_scrollbar=True,
            show_y_scrollbar=True,
            width=420,
            height=260,
            startup_focus=False,
            page_up_down_select_row=False,
            paste_can_expand_x=False,
            paste_can_expand_y=False,
            max_undos=100,
        )
        self.sheet.enable_bindings(
            "single_select",
            "drag_select",
            "column_width_resize",
            "arrowkeys",
            "right_click_popup_menu",
            "rc_select",
            "copy",
            "cut",
            "paste",
            "undo",
            "edit_cell",
            "edit_bindings",
            "select_all",
        )
        self.sheet.headers(list(self.COLUMNS))
        self.sheet.set_options(edit_cell_tab="right", edit_cell_return="down")
        self.sheet.edit_validation(self._on_single_cell_validate)
        self.sheet.bulk_table_edit_validation(self._on_sheet_modified)
        self.sheet.extra_bindings(
            [
                ("end_edit_cell", self._on_sheet_modified),
                ("end_paste", self._on_sheet_modified),
                ("end_undo", self._on_sheet_modified),
                ("end_redo", self._on_sheet_modified),
            ]
        )
        self._configure_sheet_appearance()
        self.sheet.pack(fill=tk.BOTH, expand=True)

    def set_suspension(self, suspension: Suspension | None) -> None:
        """Refresh table rows from a loaded suspension."""
        self.set_hardpoints({} if suspension is None else suspension.hardpoints)

    def set_hardpoints(self, hardpoints: dict[PointID, np.ndarray]) -> None:
        """Refresh table rows from editable hardpoint data."""
        self.hardpoints = hardpoints
        self._row_order = sorted(hardpoints)
        self._suppress_sheet_events = True
        try:
            self.sheet.set_sheet_data(
                self._sheet_rows(),
                reset_col_positions=False,
                reset_row_positions=True,
                redraw=True,
            )
            self._apply_column_sizing()
            if self._row_order:
                self.sheet.set_currently_selected(0, 1)
                self.sheet.selection_set((0, 1, 1, 4, "cells"))
        finally:
            self._suppress_sheet_events = False

    def _sheet_rows(self) -> list[list[str]]:
        return [
            [
                self._display_name(point_id),
                _format_coord(float(gui_position[0])),
                _format_coord(float(gui_position[1])),
                _format_coord(float(gui_position[2])),
            ]
            for point_id in self._row_order
            for gui_position in [
                suspension_internal_to_gui_vec3(self.hardpoints[point_id])
            ]
        ]

    def _display_name(self, point_id: PointID) -> str:
        return suspension_display_name(point_id)

    def _on_single_cell_validate(self, event_data: dict[str, Any]) -> str | None:
        if getattr(self, "_suppress_sheet_events", False):
            return event_data.get("value")
        row_index = int(event_data["row"])
        column_index = int(event_data["column"])
        if column_index == 0:
            return None
        value = str(event_data["value"]).strip()
        if not value:
            return value
        try:
            float(value)
        except ValueError:
            self._show_validation_error(row_index, self.COLUMNS[column_index], value)
            return None
        return value

    def _on_sheet_modified(
        self,
        event_data: dict[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        if getattr(self, "_suppress_sheet_events", False):
            return event_data
        try:
            changed = self._apply_sheet_values(self.sheet.get_sheet_data())
        except ValueError as exc:
            self._show_sheet_error(str(exc))
            self._restore_sheet_from_hardpoints()
            return None
        self._apply_column_sizing()
        if changed and self.on_change is not None:
            self.on_change()
        return event_data

    def _apply_sheet_values(self, data: Sequence[Sequence[Any]]) -> bool:
        if len(data) != len(self._row_order):
            raise ValueError(
                "Expected "
                f"{len(self._row_order)} hardpoint rows but received {len(data)} rows"
            )
        parsed_rows: list[np.ndarray] = []
        for row_index, values in enumerate(data, start=1):
            if len(values) < 4:
                raise ValueError(f"Expected 4 columns in row {row_index}")
            coords: list[float] = []
            for column_index, column_name in enumerate(self.COLUMNS[1:], start=1):
                raw = str(values[column_index]).strip()
                if not raw:
                    return False
                try:
                    coords.append(float(raw))
                except ValueError as exc:
                    raise ValueError(
                        "Invalid numeric value at row "
                        f"{row_index} column {column_name}: {raw}"
                    ) from exc
            parsed_rows.append(
                suspension_gui_to_internal_vec3(np.asarray(coords, dtype=np.float64))
            )

        changed = False
        for point_id, values in zip(self._row_order, parsed_rows):
            current = self.hardpoints[point_id]
            if not np.array_equal(current, values):
                self.hardpoints[point_id] = values
                changed = True
        return changed

    def _restore_sheet_from_hardpoints(self) -> None:
        if not hasattr(self.sheet, "set_sheet_data"):
            return
        self._suppress_sheet_events = True
        try:
            self.sheet.set_sheet_data(
                self._sheet_rows(),
                reset_col_positions=False,
                reset_row_positions=True,
                redraw=True,
            )
            self._apply_column_sizing()
        finally:
            self._suppress_sheet_events = False

    def _show_validation_error(
        self,
        row_index: int,
        column_name: str,
        value: str,
    ) -> None:
        self._show_sheet_error(
            "Invalid numeric value at row "
            f"{row_index + 1} column {column_name}: {value}"
        )

    def _show_sheet_error(self, message: str) -> None:
        if not hasattr(self, "tk"):
            return
        try:
            messagebox.showerror("Invalid hardpoint value", message, parent=self)
        except (tk.TclError, AttributeError):
            return

    def _configure_sheet_appearance(self) -> None:
        self.sheet.table_align("center", redraw=False)
        self.sheet.header_align("center", redraw=False)
        self.sheet.align_columns(
            list(range(len(self.COLUMNS))), align="center", redraw=False
        )

    def _apply_column_sizing(self) -> None:
        if not hasattr(self.sheet, "column_width") or not hasattr(
            self.sheet, "refresh"
        ):
            return
        for column_index in range(1, len(self.COLUMNS)):
            self.sheet.column_width(
                column_index,
                width="text",
                only_set_if_too_small=False,
                redraw=False,
            )
        self.sheet.refresh()
