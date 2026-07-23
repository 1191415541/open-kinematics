from types import MethodType, SimpleNamespace

import pytest

from kinematics.gui.steering import app as steering_app
from kinematics.gui.steering import widgets as steering_widgets
from kinematics.steering.workbench import default_steering_project
from kinematics.steering.workbench import copy_hardpoint_rows


class _FakeVar:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value) -> None:
        self.value = value


class _FakeSlider:
    def __init__(self) -> None:
        self.configured = []

    def configure(self, **kwargs) -> None:
        self.configured.append(kwargs)


class _FakeRoot:
    def __init__(self) -> None:
        self.after_calls = []
        self.cancelled = []
        self._after_seq = 0

    def after(self, delay_ms: int, callback):
        self._after_seq += 1
        handle = f"after-{self._after_seq}"
        self.after_calls.append((delay_ms, callback, handle))
        return handle

    def after_cancel(self, handle) -> None:
        self.cancelled.append(handle)
        self.after_calls = [
            item for item in self.after_calls if item[2] != handle
        ]


class _FakeOutputTable:
    def __init__(self) -> None:
        self.outputs = []
        self.errors = []

    def set_outputs(self, outputs: dict[str, float]) -> None:
        self.outputs.append(outputs)

    def set_error(self, message: str) -> None:
        self.errors.append(message)


class _FakeTreeview:
    def __init__(self) -> None:
        self.items: dict[str, tuple[str, str]] = {}
        self._next_id = 0

    def heading(self, _name: str, *, text: str) -> None:
        return None

    def column(self, _name: str, **_kwargs: object) -> None:
        return None

    def pack(self, **_kwargs: object) -> None:
        return None

    def get_children(self) -> list[str]:
        return list(self.items)

    def delete(self, item: str) -> None:
        self.items.pop(item, None)

    def insert(
        self,
        _parent: str,
        _index: str,
        *,
        values: tuple[str, str],
    ) -> str:
        item_id = f"item-{self._next_id}"
        self._next_id += 1
        self.items[item_id] = values
        return item_id


def _build_app_for_refresh_tests() -> steering_app.SteeringWorkbenchApp:
    app = object.__new__(steering_app.SteeringWorkbenchApp)
    app.project = default_steering_project()
    app.project.input_mode = "pitman_angle"
    app.project.input_value = 8.0
    app.project.sweep_min = -8.0
    app.project.sweep_max = 8.0
    app.project.sweep_step = 8.0
    app.updating_controls = False
    app.preview_has_drawn = False
    app.previous_three_segment_state = None
    app.pending_preview_refresh = None
    app.pending_hardpoint_full_refresh = None
    app.root = _FakeRoot()
    app.background_refresh_queue = None
    app.background_refresh_generation = 0
    app.background_refresh_polling = False
    app.background_refresh_pending = 0
    app._reset_refresh_caches()
    app._sync_controls_to_project = MethodType(lambda self: True, app)
    app._draw_preview_state = MethodType(lambda self, state: None, app)
    app.run_guarded = MethodType(
        lambda self, *, action, on_error: action(),
        app,
    )
    app.preview_toolbar = SimpleNamespace(update=lambda: None)
    app.preview_canvas = SimpleNamespace(draw_idle=lambda: None)
    app.input_slider = _FakeSlider()
    app.input_slider_var = _FakeVar(app.project.input_value)
    app.output_table = _FakeOutputTable()
    app.curve_manager = SimpleNamespace(
        x_var=_FakeVar("input_value"),
        y_var=_FakeVar("left_wheel_angle_deg"),
        label_var=_FakeVar(""),
    )
    app.curve_ax = object()
    app.curve_canvas = SimpleNamespace(draw_idle=lambda: None)
    return app


def test_output_table_tracks_outputs_and_errors_for_background_refresh(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake_tree = _FakeTreeview()

    monkeypatch.setattr(
        steering_widgets.ttk.Frame,
        "__init__",
        lambda self, master: None,
    )
    monkeypatch.setattr(
        steering_widgets.ttk,
        "Treeview",
        lambda *args, **kwargs: fake_tree,
    )

    table = steering_widgets.OutputTable(object())

    assert table.outputs == []
    assert table.errors == []

    table.set_outputs({"left_wheel_angle_deg": 8.0})
    table.set_error("background failed")
    table.set_outputs({"left_wheel_angle_deg": 8.0, "max_left_turn": 12.0})

    assert table.outputs == [
        {"left_wheel_angle_deg": 8.0},
        {"left_wheel_angle_deg": 8.0, "max_left_turn": 12.0},
    ]
    assert table.errors == ["background failed"]


def test_refresh_reuses_cached_limits_and_curve_rows_for_input_value_changes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _build_app_for_refresh_tests()
    call_counts = {
        "solve": 0,
        "limit_outputs": 0,
        "slider_limits": 0,
        "sweep": 0,
    }

    def fake_solve(project, *, include_limits=True, previous_state=None):
        call_counts["solve"] += 1
        assert include_limits is False
        return object(), {"left_wheel_angle_deg": project.input_value}

    def fake_limit_outputs(_geometry) -> dict[str, float]:
        call_counts["limit_outputs"] += 1
        return {"max_left_turn_left_wheel_angle_deg": 12.0}

    def fake_slider_limits(_rows, _input_mode, _linkage_type):
        call_counts["slider_limits"] += 1
        return SimpleNamespace(minimum=-10.0, maximum=10.0)

    def fake_sweep(project, *, skip_unreachable):
        call_counts["sweep"] += 1
        assert skip_unreachable is True
        return [{"input_value": project.sweep_min, "left_wheel_angle_deg": -1.0}]

    monkeypatch.setattr(steering_app, "solve_steering_project", fake_solve)
    monkeypatch.setattr(steering_app, "steering_limit_outputs", fake_limit_outputs)
    monkeypatch.setattr(steering_app, "input_angle_slider_limits", fake_slider_limits)
    monkeypatch.setattr(steering_app, "sweep_steering_project", fake_sweep)
    monkeypatch.setattr(steering_app, "draw_curve_plot", lambda ax, rows, curves: None)

    app.refresh()
    while app.root.after_calls:
        _delay, callback, _handle = app.root.after_calls.pop(0)
        callback()
    app.project.input_value = 12.0
    app.refresh()

    assert call_counts == {
        "solve": 2,
        "limit_outputs": 1,
        "slider_limits": 1,
        "sweep": 1,
    }
    assert not app.output_table.errors


def test_refresh_invalidates_cached_limits_and_curve_rows_when_geometry_changes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _build_app_for_refresh_tests()
    call_counts = {
        "limit_outputs": 0,
        "slider_limits": 0,
        "sweep": 0,
    }

    monkeypatch.setattr(
        steering_app,
        "solve_steering_project",
        lambda project, *, include_limits=True, previous_state=None: (
            object(),
            {"left_wheel_angle_deg": project.input_value},
        ),
    )
    monkeypatch.setattr(
        steering_app,
        "steering_limit_outputs",
        lambda _geometry: call_counts.__setitem__(
            "limit_outputs",
            call_counts["limit_outputs"] + 1,
        )
        or {"max_left_turn_left_wheel_angle_deg": 12.0},
    )
    monkeypatch.setattr(
        steering_app,
        "input_angle_slider_limits",
        lambda _rows, _input_mode, _linkage_type: call_counts.__setitem__(
            "slider_limits",
            call_counts["slider_limits"] + 1,
        )
        or SimpleNamespace(minimum=-10.0, maximum=10.0),
    )
    monkeypatch.setattr(
        steering_app,
        "sweep_steering_project",
        lambda project, *, skip_unreachable: call_counts.__setitem__(
            "sweep",
            call_counts["sweep"] + 1,
        )
        or [{"input_value": project.sweep_min, "left_wheel_angle_deg": -1.0}],
    )
    monkeypatch.setattr(steering_app, "draw_curve_plot", lambda ax, rows, curves: None)

    app.refresh()
    while app.root.after_calls:
        _delay, callback, _handle = app.root.after_calls.pop(0)
        callback()
    app.project.hardpoints[0].x += 1.0
    app.refresh()
    while app.root.after_calls:
        _delay, callback, _handle = app.root.after_calls.pop(0)
        callback()

    assert call_counts == {
        "limit_outputs": 2,
        "slider_limits": 2,
        "sweep": 2,
    }


def test_refresh_defers_limits_and_curve_sweep_to_background_workers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _build_app_for_refresh_tests()
    call_counts = {
        "solve": 0,
        "limit_outputs": 0,
        "slider_limits": 0,
        "sweep": 0,
        "draw_curve_plot": 0,
    }
    created_threads = []

    class FakeThread:
        def __init__(self, *, target, args=(), daemon=None) -> None:
            self.target = target
            self.args = args
            self.daemon = daemon
            self.started = False
            created_threads.append(self)

        def start(self) -> None:
            self.started = True

    monkeypatch.setattr(steering_app.threading, "Thread", FakeThread)

    def fake_solve(project, *, include_limits=True, previous_state=None):
        call_counts["solve"] += 1
        assert include_limits is False
        return object(), {"left_wheel_angle_deg": project.input_value}

    monkeypatch.setattr(steering_app, "solve_steering_project", fake_solve)
    monkeypatch.setattr(
        steering_app,
        "steering_limit_outputs",
        lambda _geometry: call_counts.__setitem__(
            "limit_outputs",
            call_counts["limit_outputs"] + 1,
        ),
    )
    monkeypatch.setattr(
        steering_app,
        "input_angle_slider_limits",
        lambda _rows, _input_mode, _linkage_type: call_counts.__setitem__(
            "slider_limits",
            call_counts["slider_limits"] + 1,
        ),
    )
    monkeypatch.setattr(
        steering_app,
        "sweep_steering_project",
        lambda project, *, skip_unreachable: call_counts.__setitem__(
            "sweep",
            call_counts["sweep"] + 1,
        ),
    )
    monkeypatch.setattr(
        steering_app,
        "draw_curve_plot",
        lambda ax, rows, curves: call_counts.__setitem__(
            "draw_curve_plot",
            call_counts["draw_curve_plot"] + 1,
        ),
    )

    app.refresh()

    assert call_counts == {
        "solve": 1,
        "limit_outputs": 0,
        "slider_limits": 0,
        "sweep": 0,
        "draw_curve_plot": 0,
    }
    assert app.output_table.outputs[-1] == {"left_wheel_angle_deg": 8.0}
    assert len(created_threads) == 2
    assert all(thread.started for thread in created_threads)
    assert app.root.after_calls


def test_background_refresh_results_update_outputs_slider_and_curve_plot(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _build_app_for_refresh_tests()
    created_threads = []
    drawn_rows = []

    class FakeThread:
        def __init__(self, *, target, args=(), daemon=None) -> None:
            self.target = target
            self.args = args
            self.daemon = daemon
            created_threads.append(self)

        def start(self) -> None:
            return

        def run(self) -> None:
            self.target(*self.args)

    monkeypatch.setattr(steering_app.threading, "Thread", FakeThread)
    monkeypatch.setattr(
        steering_app,
        "solve_steering_project",
        lambda project, *, include_limits=True, previous_state=None: (
            object(),
            {"left_wheel_angle_deg": project.input_value},
        ),
    )
    monkeypatch.setattr(
        steering_app,
        "steering_limit_outputs",
        lambda _geometry: {"max_left_turn_left_wheel_angle_deg": 12.0},
    )
    monkeypatch.setattr(
        steering_app,
        "input_angle_slider_limits",
        lambda _rows, _input_mode, _linkage_type: SimpleNamespace(
            minimum=-10.0,
            maximum=10.0,
        ),
    )
    monkeypatch.setattr(
        steering_app,
        "sweep_steering_project",
        lambda project, *, skip_unreachable: [
            {"input_value": project.sweep_min, "left_wheel_angle_deg": -1.0}
        ],
    )
    monkeypatch.setattr(
        steering_app,
        "draw_curve_plot",
        lambda ax, rows, curves: drawn_rows.append(rows),
    )

    app.refresh()
    for thread in created_threads:
        thread.run()
    app._poll_background_refresh()

    assert app.output_table.outputs[-1] == {
        "left_wheel_angle_deg": 8.0,
        "max_left_turn_left_wheel_angle_deg": 12.0,
    }
    assert app.input_slider.configured[-1] == {"from_": -10.0, "to": 10.0}
    assert drawn_rows == [[{"input_value": -8.0, "left_wheel_angle_deg": -1.0}]]


def test_pitman_transform_commit_applies_geometry_and_notifies_once() -> None:
    calls: list[str] = []
    controls = object.__new__(steering_widgets.PitmanTransformControls)
    controls.on_change = lambda: calls.append("changed")
    controls.updating = False
    controls.rows = copy_hardpoint_rows(default_steering_project().hardpoints)
    controls.x_var = _FakeVar("-420")
    controls.length_var = _FakeVar("80")

    controls._on_entry_commit(SimpleNamespace())

    rows_by_name = {
        (row.category, row.name): row
        for row in controls.rows
    }

    assert calls == ["changed"]
    assert rows_by_name[("center", "pitman_pivot")].x == pytest.approx(-420.0)
    assert rows_by_name[("symmetric", "pitman_output")].x == pytest.approx(-340.0)

def test_hardpoint_change_schedules_preview_then_full_refresh() -> None:
    app = _build_app_for_refresh_tests()
    calls: list[str] = []
    app._refresh_preview_only = MethodType(lambda self: calls.append("preview"), app)
    app.refresh = MethodType(lambda self: calls.append("full"), app)
    app._sync_pitman_controls = MethodType(lambda self: None, app)
    app.background_refresh_generation = 0

    app._on_hardpoints_changed()
    app._on_hardpoints_changed()

    assert len(app.root.after_calls) == 2
    delays = sorted(delay for delay, _callback, _handle in app.root.after_calls)
    assert delays == [
        app.HARDPOINT_PREVIEW_DELAY_MS,
        app.HARDPOINT_FULL_REFRESH_DELAY_MS,
    ]

    preview_callback = next(
        callback
        for delay, callback, _handle in app.root.after_calls
        if delay == app.HARDPOINT_PREVIEW_DELAY_MS
    )
    full_callback = next(
        callback
        for delay, callback, _handle in app.root.after_calls
        if delay == app.HARDPOINT_FULL_REFRESH_DELAY_MS
    )
    preview_callback()
    full_callback()

    assert calls == ["preview", "full"]
    assert app.pending_preview_refresh is None
    assert app.pending_hardpoint_full_refresh is None


def test_preview_only_refresh_runs_solve_in_background_thread(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    app = _build_app_for_refresh_tests()
    created_threads = []
    solved = []

    class FakeThread:
        def __init__(self, *, target, args=(), daemon=None) -> None:
            self.target = target
            self.args = args
            self.daemon = daemon
            self.started = False
            created_threads.append(self)

        def start(self) -> None:
            self.started = True

        def run(self) -> None:
            self.target(*self.args)

    monkeypatch.setattr(steering_app.threading, "Thread", FakeThread)
    monkeypatch.setattr(
        steering_app,
        "solve_steering_project",
        lambda project, *, include_limits=True, previous_state=None: (
            solved.append(project.input_value)
            or (object(), {"left_wheel_angle_deg": project.input_value})
        ),
    )
    drawn = []
    app._draw_preview_state = MethodType(lambda self, state: drawn.append(state), app)

    app._refresh_preview_only()

    assert len(created_threads) == 1
    assert created_threads[0].started
    assert solved == []
    created_threads[0].run()
    app._poll_background_refresh()

    assert solved == [8.0]
    assert len(drawn) == 1
    assert app.preview_has_drawn is True

