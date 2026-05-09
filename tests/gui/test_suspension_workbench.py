from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from kinematics.core.enums import PointID
from kinematics.gui.suspension import workbench as suspension_workbench
from kinematics.gui.suspension.workbench import (
    DEFAULT_CURVE_X,
    DEFAULT_CURVE_Y,
    SuspensionCurve,
    SuspensionOptimizationTarget,
    SuspensionOptimizationPairDeltaConstraint,
    SuspensionSweepSettings,
    build_wheel_travel_sweep,
    create_default_suspension_project,
    curve_specs_for_plot,
    load_suspension_project,
    optimize_suspension_hardpoints,
    solve_suspension_project,
    suspension_pair_delta_constraint_residuals,
    suspension_optimization_residuals,
    supported_suspension_type_keys,
)


def test_build_wheel_travel_sweep_targets_wheel_center_z() -> None:
    settings = SuspensionSweepSettings(start=-25.0, stop=75.0, steps=5)

    sweep = build_wheel_travel_sweep(settings)

    assert sweep.n_steps == 5
    targets = sweep.target_sweeps[0]
    assert [target.value for target in targets] == [-25.0, 0.0, 25.0, 50.0, 75.0]
    assert {target.point_id for target in targets} == {PointID.WHEEL_CENTER}


def test_solve_suspension_project_returns_metrics_and_solver_rows(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-10.0, stop=20.0, steps=4)

    result = solve_suspension_project(project)

    assert len(result.rows) == 4
    assert result.curve_options[0] == DEFAULT_CURVE_X
    assert DEFAULT_CURVE_Y in result.curve_options
    assert all(row["step"] == index for index, row in enumerate(result.rows))
    assert all(row["solver_converged"] for row in result.rows)
    assert {row["wheel_travel_mm"] for row in result.rows} == {
        -10.0,
        0.0,
        10.0,
        20.0,
    }
    assert "camber_deg" in result.rows[0]
    assert "toe_deg" in result.rows[0]
    assert "roadwheel_angle_deg" in result.rows[0]
    assert result.states[0].positions[PointID.WHEEL_CENTER] is not None


def test_solve_suspension_project_aliases_toe_to_roadwheel_angle(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-10.0, stop=20.0, steps=4)

    result = solve_suspension_project(project)

    assert result.rows[0]["toe_deg"] == pytest.approx(
        result.rows[0]["roadwheel_angle_deg"]
    )


def test_suspension_optimization_residuals_penalize_wrong_trend() -> None:
    rows = [
        {"camber_deg": 0.0},
        {"camber_deg": -0.5},
        {"camber_deg": -1.0},
    ]
    targets = [
        SuspensionOptimizationTarget(
            metric_name="camber_deg",
            target_delta=-1.0,
            trend="positive",
        )
    ]

    residuals = suspension_optimization_residuals(rows, targets)

    assert residuals[0] == pytest.approx(0.0)
    assert residuals[1:] == pytest.approx([0.5, 0.5])


def test_suspension_optimization_residuals_support_value_range_target_mode() -> None:
    rows = [
        {"camber_deg": 0.1},
        {"camber_deg": -0.4},
        {"camber_deg": 0.0},
    ]
    targets = [
        SuspensionOptimizationTarget(
            metric_name="camber_deg",
            target_delta=0.5,
            trend="ignore",
            target_mode="value_range",
        )
    ]

    residuals = suspension_optimization_residuals(rows, targets)

    assert residuals == pytest.approx([0.0])


def test_suspension_optimization_residuals_support_absolute_value_target_mode() -> None:
    rows = [
        {"toe_deg": 0.3},
        {"toe_deg": -0.1},
        {"toe_deg": 0.2},
    ]
    targets = [
        SuspensionOptimizationTarget(
            metric_name="toe_deg",
            target_delta=0.0,
            trend="ignore",
            target_mode="absolute_value",
        )
    ]

    residuals = suspension_optimization_residuals(rows, targets)

    assert residuals == pytest.approx([0.3, -0.1, 0.2])


def test_suspension_pair_delta_constraint_residuals_keep_baseline_delta() -> None:
    baseline = {
        PointID.UPPER_WISHBONE_INBOARD_FRONT: np.asarray([10.0, 20.0, 30.0]),
        PointID.UPPER_WISHBONE_INBOARD_REAR: np.asarray([16.0, 26.0, 39.0]),
    }
    trial = {
        PointID.UPPER_WISHBONE_INBOARD_FRONT: np.asarray([11.0, 19.0, 32.0]),
        PointID.UPPER_WISHBONE_INBOARD_REAR: np.asarray([17.0, 25.0, 41.0]),
    }
    constraint = SuspensionOptimizationPairDeltaConstraint(
        point_a="UPPER_WISHBONE_INBOARD_FRONT",
        point_b="UPPER_WISHBONE_INBOARD_REAR",
        label="Upper wishbone inboard front/rear",
        enabled=True,
    )

    residuals = suspension_pair_delta_constraint_residuals(
        trial,
        baseline,
        [constraint],
    )

    assert residuals == pytest.approx([0.0, 0.0, 0.0])


def test_optimize_suspension_hardpoints_reduces_curve_delta_error(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-20.0, stop=40.0, steps=7)

    baseline = solve_suspension_project(project)
    camber_delta = baseline.rows[-1]["camber_deg"] - baseline.rows[0]["camber_deg"]
    toe_delta = baseline.rows[-1]["toe_deg"] - baseline.rows[0]["toe_deg"]
    targets = [
        SuspensionOptimizationTarget(
            metric_name="camber_deg",
            target_delta=float(camber_delta - 0.1),
            trend="negative",
        ),
        SuspensionOptimizationTarget(
            metric_name="toe_deg",
            target_delta=float(toe_delta + 0.1),
            trend="positive",
        ),
    ]

    result = optimize_suspension_hardpoints(
        project,
        targets=targets,
        variable_names=(
            "TRACKROD_INBOARD_z",
            "TRACKROD_OUTBOARD_z",
            "UPPER_WISHBONE_OUTBOARD_z",
            "LOWER_WISHBONE_OUTBOARD_z",
        ),
        variable_delta_limit=5.0,
    )

    assert result.success
    assert result.final_cost < result.initial_cost
    assert any(
        abs(result.applied_values[name] - project.hardpoints[PointID[name.rsplit("_", 1)[0]]][
            {"x": 0, "y": 1, "z": 2}[name.rsplit("_", 1)[1]]
        ])
        > 1e-6
        for name in result.applied_values
    )


def test_optimize_suspension_hardpoints_reports_progress(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-20.0, stop=40.0, steps=7)
    events = []

    optimize_suspension_hardpoints(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=-0.2,
                trend="negative",
            )
        ],
        variable_names=(
            "TRACKROD_INBOARD_z",
            "UPPER_WISHBONE_OUTBOARD_z",
        ),
        variable_delta_limit=5.0,
        progress_callback=events.append,
    )

    assert events
    assert any(event.phase == "solving" for event in events)
    assert events[-1].phase == "finished"


def test_optimize_suspension_hardpoints_runs_cma_es_and_full_refine(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = create_default_suspension_project()
    project.settings = SuspensionSweepSettings(start=-10.0, stop=10.0, steps=7)
    variable_names = ("TRACKROD_INBOARD_z",)
    call_steps: list[int] = []
    last_steps = project.settings.steps
    cma_called = False

    def fake_solve(current_project):
        value = float(current_project.hardpoints[PointID.TRACKROD_INBOARD][2])
        rows = [{"camber_deg": 0.0}, {"camber_deg": value}]
        return SimpleNamespace(rows=rows)

    def fake_least_squares(func, x0, bounds, method):
        nonlocal last_steps
        func(np.asarray(x0, dtype=np.float64))
        call_steps.append(int(last_steps))
        return SimpleNamespace(success=True, message="refine", x=np.asarray([2.0]))

    class FakeCMA:
        def __init__(self, x0, sigma0, options):
            nonlocal cma_called
            cma_called = True
            self.x0 = np.asarray(x0, dtype=np.float64)
            self.iteration = 0

        def stop(self):
            return self.iteration >= 1

        def ask(self):
            self.iteration += 1
            return [self.x0 + np.asarray([1.0]), self.x0 + np.asarray([2.0])]

        def tell(self, _candidates, _costs):
            return None

    def fake_solve_with_settings(current_project):
        nonlocal last_steps
        last_steps = int(current_project.settings.steps)
        return fake_solve(current_project)

    monkeypatch.setattr(
        suspension_workbench,
        "solve_suspension_project",
        fake_solve_with_settings,
    )
    monkeypatch.setattr(suspension_workbench, "least_squares", fake_least_squares)
    monkeypatch.setattr(
        suspension_workbench,
        "cma",
        SimpleNamespace(CMAEvolutionStrategy=FakeCMA),
    )

    result = optimize_suspension_hardpoints(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=2.0,
                trend="ignore",
            )
        ],
        variable_names=variable_names,
        variable_delta_limit=5.0,
    )

    assert cma_called
    assert all(step == 7 for step in call_steps)
    assert len(call_steps) >= 2
    assert result.applied_values["TRACKROD_INBOARD_z"] == pytest.approx(2.0)
    assert result.final_cost < result.initial_cost


def test_optimize_suspension_hardpoints_reports_value_range_summary(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-20.0, stop=40.0, steps=7)

    result = optimize_suspension_hardpoints(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=0.0,
                trend="flat",
                target_mode="value_range",
            )
        ],
        variable_names=(
            "TRACKROD_INBOARD_z",
            "UPPER_WISHBONE_OUTBOARD_z",
        ),
        variable_delta_limit=5.0,
    )

    assert result.target_summaries[0].target_mode == "value_range"
    assert result.target_summaries[0].target_delta == pytest.approx(0.0)
    assert result.target_summaries[0].final_value <= result.target_summaries[0].initial_value


def test_optimize_suspension_hardpoints_reports_absolute_value_summary(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-20.0, stop=40.0, steps=7)

    result = optimize_suspension_hardpoints(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="toe_deg",
                target_delta=0.0,
                trend="flat",
                target_mode="absolute_value",
            )
        ],
        variable_names=(
            "TRACKROD_INBOARD_z",
            "TRACKROD_OUTBOARD_z",
            "TRACKROD_INBOARD_y",
            "TRACKROD_OUTBOARD_y",
        ),
        variable_delta_limit=5.0,
    )

    assert result.target_summaries[0].target_mode == "absolute_value"
    assert result.target_summaries[0].target_delta == pytest.approx(0.0)
    assert result.target_summaries[0].final_value <= result.target_summaries[0].initial_value


def test_optimize_suspension_hardpoints_accepts_pair_delta_constraints(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-20.0, stop=40.0, steps=7)
    pair_constraint = SuspensionOptimizationPairDeltaConstraint(
        point_a="UPPER_WISHBONE_INBOARD_FRONT",
        point_b="UPPER_WISHBONE_INBOARD_REAR",
        label="Upper wishbone inboard front/rear",
        enabled=True,
    )

    result = optimize_suspension_hardpoints(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=-0.2,
                trend="negative",
            )
        ],
        variable_names=(
            "TRACKROD_INBOARD_z",
            "UPPER_WISHBONE_OUTBOARD_z",
        ),
        variable_delta_limit=5.0,
        pair_delta_constraints=[pair_constraint],
    )

    assert result.success
    assert result.final_cost <= result.initial_cost


def test_solve_suspension_project_locks_trackrod_inboard_in_pure_sweep(
    double_wishbone_geometry_file: Path,
) -> None:
    project = load_suspension_project(double_wishbone_geometry_file)
    project.settings = SuspensionSweepSettings(start=-10.0, stop=20.0, steps=4)

    result = solve_suspension_project(project)
    locked_position = project.hardpoints[PointID.TRACKROD_INBOARD]

    for state in result.states:
        np.testing.assert_allclose(
            state.positions[PointID.TRACKROD_INBOARD],
            locked_position,
            atol=1e-6,
        )


def test_default_project_is_type_driven_and_builds_suspension() -> None:
    project = create_default_suspension_project("double_wishbone")

    assert supported_suspension_type_keys() == ("double_wishbone",)
    assert project.suspension_type == "double_wishbone"
    assert set(project.hardpoints) == {
        PointID.LOWER_WISHBONE_INBOARD_FRONT,
        PointID.LOWER_WISHBONE_INBOARD_REAR,
        PointID.LOWER_WISHBONE_OUTBOARD,
        PointID.UPPER_WISHBONE_INBOARD_FRONT,
        PointID.UPPER_WISHBONE_INBOARD_REAR,
        PointID.UPPER_WISHBONE_OUTBOARD,
        PointID.TRACKROD_INBOARD,
        PointID.TRACKROD_OUTBOARD,
        PointID.AXLE_INBOARD,
        PointID.AXLE_OUTBOARD,
    }
    assert project.config.wheelbase == 2500.0
    assert project.build_suspension().TYPE_KEY == "double_wishbone"


def test_suspension_sweep_settings_rejects_invalid_steps() -> None:
    with pytest.raises(ValueError, match="steps"):
        SuspensionSweepSettings(start=0.0, stop=10.0, steps=1)


def test_suspension_curve_specs_use_saved_curves_or_live_preview() -> None:
    saved = [
        SuspensionCurve(
            x_output="wheel_travel_mm",
            y_output="toe_deg",
            label="toe",
        )
    ]

    assert curve_specs_for_plot(saved, DEFAULT_CURVE_X, DEFAULT_CURVE_Y, "") == [
        ("wheel_travel_mm", "toe_deg", "toe")
    ]
    assert curve_specs_for_plot([], DEFAULT_CURVE_X, DEFAULT_CURVE_Y, "") == [
        ("wheel_travel_mm", "camber_deg", "camber_deg preview")
    ]
