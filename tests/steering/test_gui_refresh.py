from types import MethodType, SimpleNamespace

import pytest

from kinematics.gui.steering import app as steering_app
from kinematics.steering.workbench import default_steering_project


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

    def after(self, delay_ms: int, callback):
        self.after_calls.append((delay_ms, callback))
        return f"after-{len(self.after_calls)}"


class _FakeOutputTable:
    def __init__(self) -> None:
        self.outputs = []
        self.errors = []

    def set_outputs(self, outputs: dict[str, float]) -> None:
        self.outputs.append(outputs)

    def set_error(self, message: str) -> None:
        self.errors.append(message)


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
    app.root = _FakeRoot()
    app.background_refresh_queue = None
    app.background_refresh_generation = 0
    app.background_refresh_polling = False
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
        _delay, callback = app.root.after_calls.pop(0)
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
        _delay, callback = app.root.after_calls.pop(0)
        callback()
    app.project.hardpoints[0].x += 1.0
    app.refresh()
    while app.root.after_calls:
        _delay, callback = app.root.after_calls.pop(0)
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
