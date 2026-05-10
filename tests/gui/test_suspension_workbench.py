from pathlib import Path
from types import SimpleNamespace
from types import MethodType

import numpy as np
import pytest
import threading

from kinematics.core.enums import PointID
from kinematics.gui.suspension import workbench as suspension_workbench
from kinematics.gui.suspension import app as suspension_app
from kinematics.gui.suspension.workbench import (
    DEFAULT_CURVE_X,
    DEFAULT_CURVE_Y,
    SuspensionCurve,
    SuspensionOptimizationVariableAnalysisItem,
    SuspensionOptimizationTarget,
    SuspensionOptimizationPairDeltaConstraint,
    SuspensionSweepSettings,
    analyze_suspension_optimization_variables,
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
from kinematics.gui.suspension.workbench import OptimizationCancelledError
from kinematics.gui.suspension.global_sensitivity import MorrisVariableStat


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


def test_optimize_suspension_hardpoints_can_be_cancelled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = create_default_suspension_project()
    project.settings = SuspensionSweepSettings(start=-10.0, stop=10.0, steps=7)
    cancel_event = threading.Event()
    cancel_event.set()

    def fake_solve(current_project):
        value = float(current_project.hardpoints[PointID.TRACKROD_INBOARD][2])
        rows = [{"camber_deg": 0.0}, {"camber_deg": value}]
        return SimpleNamespace(rows=rows)

    monkeypatch.setattr(suspension_workbench, "solve_suspension_project", fake_solve)

    with pytest.raises(OptimizationCancelledError):
        optimize_suspension_hardpoints(
            project,
            targets=[
                SuspensionOptimizationTarget(
                    metric_name="camber_deg",
                    target_delta=-0.2,
                    trend="negative",
                )
            ],
            variable_names=("TRACKROD_INBOARD_z",),
            variable_delta_limit=5.0,
            cancel_event=cancel_event,
        )


def test_analyze_suspension_optimization_variables_can_be_cancelled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = create_default_suspension_project()
    cancel_event = threading.Event()
    cancel_event.set()

    monkeypatch.setattr(
        suspension_workbench,
        "build_linear_constraint_parameterization",
        lambda **kwargs: SimpleNamespace(
            direction_count=1,
            variable_count=1,
            constraint_rank=0,
            null_basis=np.eye(1, dtype=np.float64),
            map_to_variables=lambda values: np.asarray(values, dtype=np.float64),
        ),
    )
    monkeypatch.setattr(
        suspension_workbench,
        "run_morris_screening",
        lambda **kwargs: (_ for _ in ()).throw(AssertionError("should stop before morris")),
    )

    with pytest.raises(OptimizationCancelledError):
        analyze_suspension_optimization_variables(
            project,
            targets=[
                SuspensionOptimizationTarget(
                    metric_name="camber_deg",
                    target_delta=-0.1,
                    trend="negative",
                )
            ],
            variable_names=("TRACKROD_INBOARD_z",),
            variable_delta_limit=5.0,
            cancel_event=cancel_event,
        )


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


@pytest.mark.parametrize(
    ("solver_mode", "expected_least_squares_starts", "expected_cma_calls"),
    [
        ("dual_path", [250.0, 3.0, 254.0, 3.0], 1),
        ("baseline_local_only", [250.0, 3.0], 0),
        ("cma_es_then_local_refine", [254.0, 3.0], 1),
        ("cma_es_only", [], 1),
    ],
)
def test_optimize_suspension_hardpoints_honors_solver_mode(
    monkeypatch: pytest.MonkeyPatch,
    solver_mode: str,
    expected_least_squares_starts: list[float],
    expected_cma_calls: int,
) -> None:
    project = create_default_suspension_project()
    project.settings = SuspensionSweepSettings(start=-10.0, stop=10.0, steps=7)
    least_squares_starts: list[float] = []
    cma_calls = 0

    def fake_solve(current_project):
        value = float(current_project.hardpoints[PointID.TRACKROD_INBOARD][2])
        rows = [{"camber_deg": 0.0}, {"camber_deg": value}]
        return SimpleNamespace(rows=rows)

    def fake_least_squares(func, x0, bounds, method):
        least_squares_starts.append(float(np.asarray(x0, dtype=np.float64)[0]))
        func(np.asarray(x0, dtype=np.float64))
        return SimpleNamespace(success=True, message="refine", x=np.asarray([3.0]))

    class FakeCMA:
        def __init__(self, x0, sigma0, options):
            nonlocal cma_calls
            cma_calls += 1
            self.x0 = np.asarray(x0, dtype=np.float64)
            self.iteration = 0

        def stop(self):
            return self.iteration >= 1

        def ask(self):
            self.iteration += 1
            return [self.x0 + np.asarray([4.0])]

        def tell(self, _candidates, _costs):
            return None

    monkeypatch.setattr(suspension_workbench, "solve_suspension_project", fake_solve)
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
                target_delta=3.0,
                trend="ignore",
            )
        ],
        variable_names=("TRACKROD_INBOARD_z",),
        variable_delta_limit=5.0,
        solver_mode=solver_mode,
    )

    assert least_squares_starts == pytest.approx(expected_least_squares_starts)
    assert cma_calls == expected_cma_calls
    if solver_mode == "baseline_local_only":
        assert result.applied_values["TRACKROD_INBOARD_z"] == pytest.approx(3.0)
    elif solver_mode == "cma_es_then_local_refine":
        assert result.applied_values["TRACKROD_INBOARD_z"] == pytest.approx(3.0)
    elif solver_mode == "cma_es_only":
        assert result.applied_values["TRACKROD_INBOARD_z"] == pytest.approx(254.0)
    else:
        assert result.applied_values["TRACKROD_INBOARD_z"] == pytest.approx(3.0)


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


def test_analyze_suspension_optimization_variables_recommends_independent_axes(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = create_default_suspension_project()
    variable_names = (
        "TRACKROD_INBOARD_z",
        "TRACKROD_OUTBOARD_z",
        "UPPER_WISHBONE_OUTBOARD_z",
    )
    baseline = {
        name: float(
            project.hardpoints[PointID[name.rsplit("_", 1)[0]]][
                {"x": 0, "y": 1, "z": 2}[name.rsplit("_", 1)[1]]
            ]
        )
        for name in variable_names
    }

    def fake_solve(current_project):
        inboard = (
            float(current_project.hardpoints[PointID.TRACKROD_INBOARD][2])
            - baseline["TRACKROD_INBOARD_z"]
        )
        outboard = (
            float(current_project.hardpoints[PointID.TRACKROD_OUTBOARD][2])
            - baseline["TRACKROD_OUTBOARD_z"]
        )
        upper = (
            float(current_project.hardpoints[PointID.UPPER_WISHBONE_OUTBOARD][2])
            - baseline["UPPER_WISHBONE_OUTBOARD_z"]
        )
        value = inboard + outboard + 1e-5 * upper
        return SimpleNamespace(rows=[{"camber_deg": value}, {"camber_deg": value}])

    monkeypatch.setattr(suspension_workbench, "solve_suspension_project", fake_solve)

    result = analyze_suspension_optimization_variables(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=0.0,
                trend="ignore",
                target_mode="absolute_value",
            )
        ],
        variable_names=variable_names,
        variable_delta_limit=5.0,
    )

    items = {
        item.variable_name: item
        for item in result.items
        if isinstance(item, SuspensionOptimizationVariableAnalysisItem)
    }
    assert result.method == "constraint_parameterization+morris+sobol+validated_topk"
    assert result.effective_rank == 3
    assert result.sobol_direction_count >= 1
    assert len(result.recommended_variable_names) >= 1
    assert items["UPPER_WISHBONE_OUTBOARD_z"].recommendation == "suppress"
    kept = [
        item
        for item in items.values()
        if item.variable_name in result.recommended_variable_names
    ]
    assert kept
    assert all(item.recommendation == "recommended" for item in kept)


def test_analyze_suspension_optimization_variables_respects_pair_constraints(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = create_default_suspension_project()
    variable_names = (
        "UPPER_WISHBONE_INBOARD_FRONT_z",
        "UPPER_WISHBONE_INBOARD_REAR_z",
    )
    baseline = {
        name: float(
            project.hardpoints[PointID[name.rsplit("_", 1)[0]]][
                {"x": 0, "y": 1, "z": 2}[name.rsplit("_", 1)[1]]
            ]
        )
        for name in variable_names
    }

    def fake_solve(current_project):
        front = (
            float(current_project.hardpoints[PointID.UPPER_WISHBONE_INBOARD_FRONT][2])
            - baseline["UPPER_WISHBONE_INBOARD_FRONT_z"]
        )
        rear = (
            float(current_project.hardpoints[PointID.UPPER_WISHBONE_INBOARD_REAR][2])
            - baseline["UPPER_WISHBONE_INBOARD_REAR_z"]
        )
        value = rear - front
        return SimpleNamespace(rows=[{"camber_deg": value}, {"camber_deg": value}])

    monkeypatch.setattr(suspension_workbench, "solve_suspension_project", fake_solve)

    result = analyze_suspension_optimization_variables(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=0.0,
                trend="ignore",
                target_mode="absolute_value",
            )
        ],
        variable_names=variable_names,
        variable_delta_limit=5.0,
        pair_delta_constraints=[
            SuspensionOptimizationPairDeltaConstraint(
                point_a="UPPER_WISHBONE_INBOARD_FRONT",
                point_b="UPPER_WISHBONE_INBOARD_REAR",
                label="Upper wishbone inboard front/rear",
                enabled=True,
                axes=("z",),
            )
        ],
    )

    items = {
        item.variable_name: item
        for item in result.items
        if isinstance(item, SuspensionOptimizationVariableAnalysisItem)
    }
    assert result.constraint_rank == 1
    assert result.effective_rank == 1
    assert items["UPPER_WISHBONE_INBOARD_FRONT_z"].morris_mu_star >= 0.0
    assert items["UPPER_WISHBONE_INBOARD_REAR_z"].morris_mu_star >= 0.0
    assert all(
        item.recommendation in {"recommended", "secondary", "suppress"}
        for item in items.values()
    )


def test_analyze_suspension_optimization_variables_keeps_recommended_names_in_sync(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = create_default_suspension_project()
    variable_names = (
        "LOWER_WISHBONE_INBOARD_FRONT_x",
        "LOWER_WISHBONE_INBOARD_FRONT_y",
    )

    monkeypatch.setattr(
        suspension_workbench,
        "solve_suspension_project",
        lambda _project: SimpleNamespace(rows=[{"camber_deg": 0.0}, {"camber_deg": 0.0}]),
    )
    monkeypatch.setattr(
        suspension_workbench,
        "run_morris_screening",
        lambda **kwargs: (
            [
                MorrisVariableStat(
                    variable_index=0,
                    mu_star=1.0,
                    sigma=0.0,
                ),
                MorrisVariableStat(
                    variable_index=1,
                    mu_star=1e-6,
                    sigma=0.0,
                ),
            ],
            np.asarray([1.0, 1e-6], dtype=np.float64),
        ),
    )
    monkeypatch.setattr(
        suspension_workbench,
        "pick_reduced_directions_from_morris",
        lambda **kwargs: (),
    )
    monkeypatch.setattr(
        suspension_workbench,
        "run_pairwise_sobol_screening",
        lambda **kwargs: [],
    )
    monkeypatch.setattr(
        suspension_workbench,
        "optimize_suspension_hardpoints",
        lambda *args, **kwargs: SimpleNamespace(final_cost=0.0),
    )

    result = analyze_suspension_optimization_variables(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=0.8,
                trend="negative",
                target_mode="value_range",
            )
        ],
        variable_names=variable_names,
        variable_delta_limit=5.0,
    )

    recommendations = {item.variable_name: item.recommendation for item in result.items}
    recommended_names = set(result.recommended_variable_names)

    assert recommended_names == {
        name
        for name, recommendation in recommendations.items()
        if recommendation == "recommended"
    }
    assert "LOWER_WISHBONE_INBOARD_FRONT_y" not in recommended_names
    assert recommendations["LOWER_WISHBONE_INBOARD_FRONT_y"] == "suppress"


def test_analyze_suspension_optimization_variables_validates_recommended_subset(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    project = create_default_suspension_project()
    variable_names = (
        "LOWER_WISHBONE_INBOARD_FRONT_x",
        "LOWER_WISHBONE_INBOARD_FRONT_y",
        "LOWER_WISHBONE_INBOARD_FRONT_z",
        "LOWER_WISHBONE_INBOARD_REAR_x",
        "LOWER_WISHBONE_INBOARD_REAR_y",
        "LOWER_WISHBONE_INBOARD_REAR_z",
    )

    monkeypatch.setattr(
        suspension_workbench,
        "solve_suspension_project",
        lambda _project: SimpleNamespace(rows=[{"camber_deg": 0.0}, {"camber_deg": 0.0}]),
    )
    monkeypatch.setattr(
        suspension_workbench,
        "run_morris_screening",
        lambda **kwargs: (
            [
                MorrisVariableStat(variable_index=index, mu_star=1.0 - index * 0.1, sigma=0.0)
                for index in range(len(variable_names))
            ],
            np.asarray([1.0 - index * 0.1 for index in range(len(variable_names))], dtype=np.float64),
        ),
    )
    monkeypatch.setattr(
        suspension_workbench,
        "pick_reduced_directions_from_morris",
        lambda **kwargs: (),
    )
    monkeypatch.setattr(
        suspension_workbench,
        "run_pairwise_sobol_screening",
        lambda **kwargs: [],
    )

    calls: list[tuple[str, ...]] = []

    def fake_optimize(
        current_project,
        *,
        targets,
        variable_names,
        variable_delta_limit,
        solver_mode="dual_path",
        pair_delta_constraints=None,
        progress_callback=None,
        cancel_event=None,
        max_rounds=6,
        convergence_tolerance=1e-3,
    ):
        del current_project, targets, variable_delta_limit, solver_mode, pair_delta_constraints
        del progress_callback, cancel_event, max_rounds, convergence_tolerance
        calls.append(tuple(variable_names))
        cost_by_size = {
            4: 0.60,
            6: 0.20,
        }
        return SimpleNamespace(final_cost=cost_by_size[len(variable_names)])

    monkeypatch.setattr(
        suspension_workbench,
        "optimize_suspension_hardpoints",
        fake_optimize,
    )

    result = analyze_suspension_optimization_variables(
        project,
        targets=[
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                target_delta=0.8,
                trend="negative",
                target_mode="value_range",
            )
        ],
        variable_names=variable_names,
        variable_delta_limit=5.0,
    )

    assert calls == [
        (
            "LOWER_WISHBONE_INBOARD_FRONT_x",
            "LOWER_WISHBONE_INBOARD_FRONT_y",
            "LOWER_WISHBONE_INBOARD_FRONT_z",
            "LOWER_WISHBONE_INBOARD_REAR_x",
        ),
        variable_names,
    ]
    assert result.recommended_variable_names == variable_names
    assert result.method == "constraint_parameterization+morris+sobol+validated_topk"


def test_format_optimization_analysis_groups_sections_and_metrics() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)
    result = suspension_workbench.SuspensionOptimizationVariableAnalysisResult(
        items=(
            suspension_workbench.SuspensionOptimizationVariableAnalysisItem(
                variable_name="TRACKROD_OUTBOARD_z",
                morris_mu_star=0.21,
                morris_sigma=0.04,
                sobol_first_order=0.18,
                sobol_total=0.31,
                recommendation="recommended",
                detail="High constrained global influence for current targets",
            ),
            suspension_workbench.SuspensionOptimizationVariableAnalysisItem(
                variable_name="TRACKROD_INBOARD_z",
                morris_mu_star=0.05,
                morris_sigma=0.07,
                sobol_first_order=0.01,
                sobol_total=0.03,
                recommendation="secondary",
                detail="Useful but lower-priority variable under current targets",
            ),
            suspension_workbench.SuspensionOptimizationVariableAnalysisItem(
                variable_name="UPPER_WISHBONE_INBOARD_FRONT_z",
                morris_mu_star=0.002,
                morris_sigma=0.001,
                sobol_first_order=None,
                sobol_total=None,
                recommendation="suppress",
                detail="Low constrained global influence in Morris/Sobol screening",
            ),
        ),
        recommended_variable_names=("TRACKROD_OUTBOARD_z",),
        residual_size=162,
        variable_count=4,
        constraint_rank=1,
        effective_rank=3,
        morris_trajectories=6,
        sobol_base_samples=8,
        sobol_direction_count=2,
        method="constraint_parameterization+morris+sobol+validated_topk",
    )

    sections = page._format_optimization_analysis(result)

    assert [section["kind"] for section in sections] == [
        "heading",
        "summary",
        "group",
        "group",
        "group",
    ]
    assert "Global Sensitivity Analysis" in sections[0]["text"]
    assert "validated_topk" in sections[1]["text"]
    assert "Recommended" in sections[2]["text"]
    assert "Morris mu*" in sections[2]["text"]
    assert "Suppress" in sections[4]["text"]


def test_render_optimization_output_writes_copyable_text() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)
    recorded: list[tuple[str, tuple[object, ...]]] = []

    class FakeText:
        def configure(self, **kwargs):
            recorded.append(("configure", (kwargs,)))

        def delete(self, start, end):
            recorded.append(("delete", (start, end)))

        def insert(self, index, text, tags=()):
            recorded.append(("insert", (index, text, tags)))

        def tag_configure(self, name, **kwargs):
            recorded.append(("tag_configure", (name, kwargs)))

    page.optimization_output = FakeText()
    page._copyable_optimization_output = ""

    def fake_copy(self, _event=None):
        return "break"

    page._copy_optimization_output = MethodType(fake_copy, page)
    page._configure_optimization_output_tags()
    page._render_optimization_output(
        [
            {"kind": "heading", "text": "Heading"},
            {"kind": "summary", "text": "Summary line"},
            {"kind": "recommended", "text": "Recommended line"},
            {"kind": "group", "tone": "recommended", "text": "Recommended group line"},
        ]
    )

    inserted_text = "".join(
        args[1] for action, args in recorded if action == "insert"
    )
    assert "Heading" in inserted_text
    assert "Summary line" in inserted_text
    assert "Recommended line" in inserted_text
    assert "Recommended group line" in inserted_text
    assert "Heading" in page._copyable_optimization_output


def test_copy_optimization_output_flushes_clipboard_every_time() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)
    page._copyable_optimization_output = "Optimization result text"
    calls: list[tuple[str, str | None]] = []

    class FakeText:
        def get(self, _start, _end):
            raise suspension_app.tk.TclError("no selection")

    page.optimization_output = FakeText()
    page.clipboard_clear = MethodType(lambda self: calls.append(("clear", None)), page)
    page.clipboard_append = MethodType(
        lambda self, text: calls.append(("append", str(text))),
        page,
    )
    page.update = MethodType(lambda self: calls.append(("update", None)), page)

    assert page._copy_optimization_output() == "break"
    assert page._copy_optimization_output() == "break"
    assert calls == [
        ("clear", None),
        ("append", "Optimization result text"),
        ("update", None),
        ("clear", None),
        ("append", "Optimization result text"),
        ("update", None),
    ]


def test_select_recommended_optimization_variables_updates_checkboxes() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)
    page.last_optimization_analysis = suspension_workbench.SuspensionOptimizationVariableAnalysisResult(
        items=(
            suspension_workbench.SuspensionOptimizationVariableAnalysisItem(
                variable_name="TRACKROD_OUTBOARD_z",
                morris_mu_star=0.2,
                morris_sigma=0.01,
                sobol_first_order=0.1,
                sobol_total=0.2,
                recommendation="recommended",
                detail="High constrained global influence for current targets",
            ),
            suspension_workbench.SuspensionOptimizationVariableAnalysisItem(
                variable_name="TRACKROD_INBOARD_z",
                morris_mu_star=0.001,
                morris_sigma=0.0,
                sobol_first_order=None,
                sobol_total=None,
                recommendation="suppress",
                detail="Low constrained global influence in Morris/Sobol screening",
            ),
        ),
        recommended_variable_names=("TRACKROD_OUTBOARD_z",),
        residual_size=10,
        variable_count=3,
        constraint_rank=0,
        effective_rank=3,
        morris_trajectories=6,
        sobol_base_samples=8,
        sobol_direction_count=2,
        method="constraint_parameterization+morris+sobol",
    )

    class FakeVar:
        def __init__(self, value: bool) -> None:
            self.value = value

        def get(self) -> bool:
            return self.value

        def set(self, value: bool) -> None:
            self.value = value

    page.opt_variable_vars = {
        "TRACKROD_OUTBOARD_z": FakeVar(False),
        "TRACKROD_INBOARD_z": FakeVar(False),
        "UPPER_WISHBONE_OUTBOARD_z": FakeVar(True),
    }
    captured: list[tuple[str, str, str]] = []
    page._show_optimization_message = MethodType(
        lambda self, message, *, heading=None, kind="summary": captured.append(
            (str(heading), str(kind), str(message))
        ),
        page,
    )

    page._select_recommended_optimization_variables()

    assert page.opt_variable_vars["TRACKROD_OUTBOARD_z"].get() is True
    assert page.opt_variable_vars["TRACKROD_INBOARD_z"].get() is False
    assert page.opt_variable_vars["UPPER_WISHBONE_OUTBOARD_z"].get() is False
    assert captured[-1][2].startswith("Selected recommended variables")


def test_store_selected_optimization_variables_resets_analysis() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)
    page.project = create_default_suspension_project()
    page.last_optimization_analysis = object()

    class FakeVar:
        def __init__(self, value: bool) -> None:
            self.value = value

        def get(self) -> bool:
            return self.value

        def set(self, value: bool) -> None:
            self.value = value

    page.opt_variable_vars = {
        "TRACKROD_OUTBOARD_z": FakeVar(True),
        "TRACKROD_INBOARD_z": FakeVar(False),
    }

    page._store_selected_optimization_variables()

    assert page.project.optimization.variable_names == ["TRACKROD_OUTBOARD_z"]
    assert page.last_optimization_analysis is None


def test_on_controls_changed_resets_analysis_before_refresh() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)
    page.last_optimization_analysis = object()
    page.updating_controls = False
    refresh_calls: list[str] = []
    page.refresh = MethodType(lambda self: refresh_calls.append("refresh"), page)

    page._on_controls_changed()

    assert page.last_optimization_analysis is None
    assert refresh_calls == ["refresh"]


def test_bulk_optimization_variable_selection_helpers() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)

    class FakeVar:
        def __init__(self, value: bool) -> None:
            self.value = value

        def get(self) -> bool:
            return self.value

        def set(self, value: bool) -> None:
            self.value = value

    page.opt_variable_vars = {
        "A_x": FakeVar(False),
        "A_y": FakeVar(True),
        "B_z": FakeVar(False),
    }

    page._select_all_optimization_variables()
    assert all(variable.get() for variable in page.opt_variable_vars.values())

    page._clear_optimization_variable_selection()
    assert not any(variable.get() for variable in page.opt_variable_vars.values())

    page.opt_variable_vars["A_x"].set(True)
    page.opt_variable_vars["A_y"].set(False)
    page.opt_variable_vars["B_z"].set(True)
    page._invert_optimization_variable_selection()
    assert page.opt_variable_vars["A_x"].get() is False
    assert page.opt_variable_vars["A_y"].get() is True
    assert page.opt_variable_vars["B_z"].get() is False


def test_optimization_variable_style_name_groups_axes_by_hardpoint() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)

    assert (
        page._optimization_variable_style_name("TRACKROD_OUTBOARD_x")
        == page._optimization_variable_style_name("TRACKROD_OUTBOARD_z")
    )
    assert (
        page._optimization_variable_style_name("TRACKROD_OUTBOARD_x")
        != page._optimization_variable_style_name("TRACKROD_INBOARD_x")
    )


def test_format_optimization_summary_line_uses_curve_value_target_label() -> None:
    page = object.__new__(suspension_app.SuspensionWorkbenchPage)

    text = page._format_optimization_summary_line(
        SimpleNamespace(
            metric_name="toe_deg",
            target_mode="absolute_value",
            target_delta=0.0,
            initial_value=0.4,
            final_value=0.1,
        )
    )

    assert "Curve value target 0" in text
    assert "RMS error" in text


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

    assert supported_suspension_type_keys() == (
        "double_wishbone",
        "double_wishbone_carrier",
    )
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


def test_default_carrier_project_is_type_driven_and_builds_suspension() -> None:
    project = create_default_suspension_project("double_wishbone_carrier")

    assert project.suspension_type == "double_wishbone_carrier"
    assert PointID.CARRIER_STEERING_AXIS_LOWER in project.hardpoints
    assert PointID.CARRIER_STEERING_AXIS_UPPER in project.hardpoints
    assert project.build_suspension().TYPE_KEY == "double_wishbone_carrier"


def test_default_carrier_project_solves_preview_state_without_metric_errors() -> None:
    project = create_default_suspension_project("double_wishbone_carrier")

    result = suspension_workbench.solve_suspension_project_at_travel(project, -40.0)

    assert len(result.rows) == 1
    assert "camber_deg" in result.rows[0]
    assert "svic_x_mm" in result.rows[0]


def test_default_carrier_project_keeps_steering_axis_cross_distances_through_sweep() -> None:
    project = create_default_suspension_project("double_wishbone_carrier")
    project.settings = SuspensionSweepSettings(start=-40.0, stop=40.0, steps=9)

    result = solve_suspension_project(project)
    baseline = result.states[0].positions
    baseline_lo_to_up_mount = float(
        np.linalg.norm(
            baseline[PointID.CARRIER_STEERING_AXIS_LOWER]
            - baseline[PointID.UPPER_WISHBONE_OUTBOARD]
        )
    )
    baseline_up_to_lo_mount = float(
        np.linalg.norm(
            baseline[PointID.CARRIER_STEERING_AXIS_UPPER]
            - baseline[PointID.LOWER_WISHBONE_OUTBOARD]
        )
    )

    for state in result.states[1:]:
        lo_to_up_mount = float(
            np.linalg.norm(
                state.positions[PointID.CARRIER_STEERING_AXIS_LOWER]
                - state.positions[PointID.UPPER_WISHBONE_OUTBOARD]
            )
        )
        up_to_lo_mount = float(
            np.linalg.norm(
                state.positions[PointID.CARRIER_STEERING_AXIS_UPPER]
                - state.positions[PointID.LOWER_WISHBONE_OUTBOARD]
            )
        )
        assert lo_to_up_mount == pytest.approx(baseline_lo_to_up_mount, abs=1e-6)
        assert up_to_lo_mount == pytest.approx(baseline_up_to_lo_mount, abs=1e-6)


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
