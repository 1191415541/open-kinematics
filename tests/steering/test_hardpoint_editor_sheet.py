import inspect
from types import SimpleNamespace

import pytest

from kinematics.gui.steering.widgets import HardpointEditor
from kinematics.steering.workbench import SteeringHardpointRow


def _rows() -> list[SteeringHardpointRow]:
    return [
        SteeringHardpointRow("symmetric", "wheel_center", 60.0, -520.0, 320.0),
        SteeringHardpointRow("center", "pitman_pivot", -350.0, 0.0, 300.0),
    ]


def test_hardpoint_editor_uses_tksheet_for_excel_like_interaction() -> None:
    source = inspect.getsource(HardpointEditor._build)

    assert "tksheet.Sheet(" in source
    assert "enable_bindings(" in source
    assert "edit_validation(" in source
    assert "bulk_table_edit_validation(" in source


def test_sheet_rows_include_display_name_and_coordinates() -> None:
    editor = object.__new__(HardpointEditor)
    editor.rows = _rows()

    data = editor._sheet_rows()

    assert data == [
        ["Wheel Center", "60", "-520", "320"],
        ["Pitman Pivot", "-350", "0", "300"],
    ]


def test_apply_sheet_values_updates_hardpoint_rows() -> None:
    editor = object.__new__(HardpointEditor)
    editor.rows = _rows()

    changed = editor._apply_sheet_values(
        [
            ["Wheel Center", "61.5", "-521.25", "321.75"],
            ["Pitman Pivot", "-349", "0", "299.5"],
        ]
    )

    assert changed is True
    assert editor.rows[0].x == pytest.approx(61.5)
    assert editor.rows[0].y == pytest.approx(-521.25)
    assert editor.rows[0].z == pytest.approx(321.75)
    assert editor.rows[1].x == pytest.approx(-349.0)
    assert editor.rows[1].z == pytest.approx(299.5)


def test_apply_sheet_values_rejects_invalid_numeric_cell() -> None:
    editor = object.__new__(HardpointEditor)
    editor.rows = _rows()

    with pytest.raises(ValueError, match="row 1 column X"):
        editor._apply_sheet_values(
            [
                ["Wheel Center", "bad", "-521.25", "321.75"],
                ["Pitman Pivot", "-349", "0", "299.5"],
            ]
        )

    assert editor.rows[0].x == pytest.approx(60.0)
    assert editor.rows[1].x == pytest.approx(-350.0)


def test_apply_sheet_values_treats_blank_numeric_cell_as_incomplete() -> None:
    editor = object.__new__(HardpointEditor)
    editor.rows = _rows()

    changed = editor._apply_sheet_values(
        [
            ["Wheel Center", "", "-521.25", "321.75"],
            ["Pitman Pivot", "-349", "0", "299.5"],
        ]
    )

    assert changed is False
    assert editor.rows[0].x == pytest.approx(60.0)
    assert editor.rows[1].x == pytest.approx(-350.0)


def test_bulk_edit_handler_commits_valid_sheet_change() -> None:
    calls: list[str] = []
    editor = object.__new__(HardpointEditor)
    editor.rows = _rows()
    editor.on_change = lambda: calls.append("changed")
    editor.sheet = SimpleNamespace(
        get_sheet_data=lambda: [
            ["Wheel Center", "62", "-522", "322"],
            ["Pitman Pivot", "-348", "0", "298"],
        ]
    )

    event_data = {"cells": {"table": {(0, 1): "62"}}}

    result = editor._on_sheet_modified(event_data)

    assert result == event_data
    assert calls == ["changed"]
    assert editor.rows[0].x == pytest.approx(62.0)
    assert editor.rows[1].z == pytest.approx(298.0)


def test_bulk_edit_handler_rejects_invalid_sheet_change() -> None:
    editor = object.__new__(HardpointEditor)
    editor.rows = _rows()
    editor.on_change = lambda: None
    editor.sheet = SimpleNamespace(
        get_sheet_data=lambda: [
            ["Wheel Center", "62", "oops", "322"],
            ["Pitman Pivot", "-348", "0", "298"],
        ]
    )

    event_data = {"cells": {"table": {(0, 2): "oops"}}}

    result = editor._on_sheet_modified(event_data)

    assert result is None
    assert editor.rows[0].y == pytest.approx(-520.0)


def test_bulk_edit_handler_allows_blank_sheet_value_without_refresh() -> None:
    calls: list[str] = []
    editor = object.__new__(HardpointEditor)
    editor.rows = _rows()
    editor.on_change = lambda: calls.append("changed")
    editor.sheet = SimpleNamespace(
        get_sheet_data=lambda: [
            ["Wheel Center", "", "-522", "322"],
            ["Pitman Pivot", "-348", "0", "298"],
        ]
    )

    event_data = {"cells": {"table": {(0, 1): ""}}}

    result = editor._on_sheet_modified(event_data)

    assert result == event_data
    assert calls == []
    assert editor.rows[0].x == pytest.approx(60.0)
