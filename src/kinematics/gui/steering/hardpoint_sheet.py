"""Excel-like hardpoint sheet widget for the steering workbench."""

from __future__ import annotations

import tkinter as tk
from collections.abc import Sequence
from tkinter import messagebox, ttk
from typing import Any, Callable

import tksheet

from kinematics.steering.workbench import SteeringHardpointRow


def _format_coord(value: float) -> str:
    """Return a compact string for one coordinate cell."""
    return f"{value:.15g}"


class HardpointEditor(ttk.Frame):
    """Spreadsheet-style hardpoint editor with Excel-like interactions."""

    DISPLAY_NAMES = {
        "wheel_kingpin_lower": "Wheel Kingpin Lower",
        "wheel_kingpin_upper": "Wheel Kingpin Upper",
        "wheel_center": "Wheel Center",
        "wheel_tie_rod_pickup": "Wheel Tie Rod Pickup",
        "pitman_output": "Pitman Output",
        "pitman_pivot": "Pitman Pivot",
        "bellcrank_pivot": "Bellcrank Pivot",
        "bellcrank_center_link_pickup": "Bellcrank Center Link Pickup",
        "bellcrank_tie_rod_pickup": "Bellcrank Tie Rod Pickup",
    }
    COLUMNS = ("Point", "X", "Y", "Z")

    def __init__(
        self,
        master: tk.Misc,
        on_change: Callable[[], None],
    ) -> None:
        super().__init__(master)
        self.on_change = on_change
        self.rows: list[SteeringHardpointRow] = []
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

    def set_rows(self, rows: list[SteeringHardpointRow]) -> None:
        """Load rows into the sheet without emitting change callbacks."""
        self.rows = rows
        self._suppress_sheet_events = True
        try:
            self.sheet.set_sheet_data(
                self._sheet_rows(),
                reset_col_positions=False,
                reset_row_positions=True,
                redraw=True,
            )
            self._apply_column_sizing()
            if rows:
                self.sheet.set_currently_selected(0, 1)
                self.sheet.selection_set((0, 1, 1, 4, "cells"))
        finally:
            self._suppress_sheet_events = False

    def _sheet_rows(self) -> list[list[str]]:
        return [
            [
                self._display_name(row),
                _format_coord(row.x),
                _format_coord(row.y),
                _format_coord(row.z),
            ]
            for row in self.rows
        ]

    def _display_name(self, row: SteeringHardpointRow) -> str:
        return self.DISPLAY_NAMES.get(row.name, row.name.replace("_", " ").title())

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
            self._restore_sheet_from_rows()
            return None
        self._apply_column_sizing()
        if changed:
            self.on_change()
        return event_data

    def _apply_sheet_values(self, data: Sequence[Sequence[Any]]) -> bool:
        if len(data) != len(self.rows):
            raise ValueError(
                "Expected "
                f"{len(self.rows)} hardpoint rows but received {len(data)} rows"
            )
        parsed_rows: list[tuple[float, float, float]] = []
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
            parsed_rows.append((coords[0], coords[1], coords[2]))

        changed = False
        for row, (x_value, y_value, z_value) in zip(self.rows, parsed_rows):
            if (row.x, row.y, row.z) != (x_value, y_value, z_value):
                row.x = x_value
                row.y = y_value
                row.z = z_value
                changed = True
        return changed

    def _restore_sheet_from_rows(self) -> None:
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

    def _configure_sheet_appearance(self) -> None:
        self.sheet.table_align("center", redraw=False)
        self.sheet.header_align("center", redraw=False)
        self.sheet.align_columns(list(range(len(self.COLUMNS))), align="center", redraw=False)

    def _apply_column_sizing(self) -> None:
        if not hasattr(self.sheet, "column_width") or not hasattr(self.sheet, "refresh"):
            return
        for column_index in range(1, len(self.COLUMNS)):
            self.sheet.column_width(
                column_index,
                width="text",
                only_set_if_too_small=False,
                redraw=False,
            )
        self.sheet.refresh()

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
