from types import SimpleNamespace

import numpy as np
import pytest

from kinematics.core.enums import PointID
from kinematics.gui.suspension.widgets import HardpointTable


def _hardpoints() -> dict[PointID, np.ndarray]:
    return {
        PointID.TRACKROD_INBOARD: np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
        PointID.TRACKROD_OUTBOARD: np.asarray([240.0, -60.0, 210.0], dtype=np.float64),
    }


def test_sheet_rows_include_display_name_and_coordinates() -> None:
    table = object.__new__(HardpointTable)
    table.hardpoints = _hardpoints()
    table._row_order = sorted(table.hardpoints)

    data = table._sheet_rows()

    assert data == [
        ["Tie Rod Inner", "-120", "-40", "180"],
        ["Tie Rod Outer", "-240", "60", "210"],
    ]


def test_apply_sheet_values_updates_hardpoint_rows() -> None:
    table = object.__new__(HardpointTable)
    table.hardpoints = _hardpoints()
    table._row_order = sorted(table.hardpoints)

    changed = table._apply_sheet_values(
        [
            ["Tie Rod Inner", "121.5", "39.25", "181.75"],
            ["Tie Rod Outer", "241", "-59.5", "210"],
        ]
    )

    assert changed is True
    np.testing.assert_allclose(
        table.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([-121.5, -39.25, 181.75], dtype=np.float64),
    )
    np.testing.assert_allclose(
        table.hardpoints[PointID.TRACKROD_OUTBOARD],
        np.asarray([-241.0, 59.5, 210.0], dtype=np.float64),
    )


def test_apply_sheet_values_rejects_invalid_numeric_cell() -> None:
    table = object.__new__(HardpointTable)
    table.hardpoints = _hardpoints()
    table._row_order = sorted(table.hardpoints)

    with pytest.raises(ValueError, match="row 1 column X"):
        table._apply_sheet_values(
            [
                ["Tie Rod Inner", "bad", "39.25", "181.75"],
                ["Tie Rod Outer", "241", "-59.5", "210"],
            ]
        )

    np.testing.assert_allclose(
        table.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    )


def test_apply_sheet_values_treats_blank_numeric_cell_as_incomplete() -> None:
    table = object.__new__(HardpointTable)
    table.hardpoints = _hardpoints()
    table._row_order = sorted(table.hardpoints)

    changed = table._apply_sheet_values(
        [
            ["Tie Rod Inner", "", "39.25", "181.75"],
            ["Tie Rod Outer", "241", "-59.5", "210"],
        ]
    )

    assert changed is False
    np.testing.assert_allclose(
        table.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    )


def test_bulk_edit_handler_commits_valid_sheet_change() -> None:
    calls: list[str] = []
    table = object.__new__(HardpointTable)
    table.hardpoints = _hardpoints()
    table._row_order = sorted(table.hardpoints)
    table.on_change = lambda: calls.append("changed")
    table.sheet = SimpleNamespace(
        get_sheet_data=lambda: [
            ["Tie Rod Inner", "122", "38", "182"],
            ["Tie Rod Outer", "242", "-58", "211"],
        ]
    )

    event_data = {"cells": {"table": {(0, 1): "122"}}}

    result = table._on_sheet_modified(event_data)

    assert result == event_data
    assert calls == ["changed"]
    np.testing.assert_allclose(
        table.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([-122.0, -38.0, 182.0], dtype=np.float64),
    )


def test_bulk_edit_handler_rejects_invalid_sheet_change() -> None:
    table = object.__new__(HardpointTable)
    table.hardpoints = _hardpoints()
    table._row_order = sorted(table.hardpoints)
    table.on_change = lambda: None
    table.sheet = SimpleNamespace(
        get_sheet_data=lambda: [
            ["Tie Rod Inner", "122", "oops", "182"],
            ["Tie Rod Outer", "242", "-58", "211"],
        ]
    )

    event_data = {"cells": {"table": {(0, 2): "oops"}}}

    result = table._on_sheet_modified(event_data)

    assert result is None
    np.testing.assert_allclose(
        table.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    )


def test_bulk_edit_handler_allows_blank_sheet_value_without_refresh() -> None:
    calls: list[str] = []
    table = object.__new__(HardpointTable)
    table.hardpoints = _hardpoints()
    table._row_order = sorted(table.hardpoints)
    table.on_change = lambda: calls.append("changed")
    table.sheet = SimpleNamespace(
        get_sheet_data=lambda: [
            ["Tie Rod Inner", "", "38", "182"],
            ["Tie Rod Outer", "242", "-58", "211"],
        ]
    )

    event_data = {"cells": {"table": {(0, 1): ""}}}

    result = table._on_sheet_modified(event_data)

    assert result == event_data
    assert calls == []
    np.testing.assert_allclose(
        table.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    )
