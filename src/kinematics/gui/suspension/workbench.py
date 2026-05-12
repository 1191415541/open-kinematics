"""Workbench data model for suspension GUI simulation."""

from __future__ import annotations

import csv
import json
import threading
import time
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from typing import Any

import cma
import numpy as np
from scipy.optimize import least_squares

from kinematics.core.constants import MM_PER_INCH
from kinematics.core.enums import Axis, PointID, TargetPositionMode, Units
from kinematics.core.types import PointTarget, PointTargetAxis, SweepConfig
from kinematics.gui.common import OptimizationCancelledError, raise_if_cancelled
from kinematics.gui.project import build_project_document, write_project_document
from kinematics.gui.suspension.global_sensitivity import (
    build_linear_constraint_parameterization,
    pick_reduced_directions_from_morris,
    run_morris_screening,
    run_pairwise_sobol_screening,
)
from kinematics.gui.suspension.optimization import (
    SuspensionOptimizationConfig,
    SuspensionOptimizationMetricSummary,
    SuspensionOptimizationProgress,
    SuspensionOptimizationResult,
    SuspensionOptimizationVariableAnalysisItem,
    SuspensionOptimizationVariableAnalysisResult,
    SuspensionOptimizationPairDeltaConstraint,
    SuspensionOptimizationTarget,
    ProgressCallback,
    get_suspension_optimization_variable,
    metric_series_from_rows,
    optimization_config_from_dict,
    optimization_config_to_dict,
    suspension_metric_summary_value,
    suspension_pair_delta_constraint_residuals,
    set_suspension_optimization_variable,
    suspension_optimization_residuals,
)
from kinematics.io.geometry_loader import load_geometry
from kinematics.io.validation import coerce_enum
from kinematics.main import solve_sweep
from kinematics.metrics import compute_metrics_for_state_from_suspension
from kinematics.solver import SolverInfo
from kinematics.state import SuspensionState
from kinematics.suspensions.base import Suspension
from kinematics.suspensions.config.settings import (
    SuspensionConfig,
    TireConfig,
    WheelConfig,
)
from kinematics.suspensions.registry import get_suspension_class, list_supported_types

DEFAULT_CURVE_X = "wheel_travel_mm"
DEFAULT_CURVE_Y = "camber_deg"
DEFAULT_CURVE_OPTIONS = (
    "wheel_travel_mm",
    "camber_deg",
    "caster_deg",
    "roadwheel_angle_deg",
    "toe_deg",
    "kpi_deg",
    "scrub_radius_mm",
    "mechanical_trail_mm",
    "solver_max_residual",
)
OPTIMIZATION_COARSE_MAX_STEPS = 5
OPTIMIZATION_CMA_POPSIZE = 8
OPTIMIZATION_CMA_MAX_ITER = 12
OPTIMIZATION_REGULARIZATION_WEIGHT = 0.0
OPTIMIZATION_ANALYSIS_TOP_K_SIZES = (4, 6, 8, 10, 12)
OPTIMIZATION_ANALYSIS_VALIDATION_COST_TOLERANCE = 0.05
SUSPENSION_GUI_COORDINATE_SYSTEM = "rear_right_up"
SUSPENSION_INTERNAL_COORDINATE_SYSTEM = "forward_left_up"


@dataclass
class SuspensionSweepSettings:
    """Wheel-travel sweep settings for a suspension project."""

    start: float = -40.0
    stop: float = 120.0
    steps: int = 41

    def __post_init__(self) -> None:
        if self.steps < 2:
            raise ValueError("steps must be at least 2")


@dataclass
class SuspensionCurve:
    """Curve definition for plotting one suspension output against another."""

    x_output: str
    y_output: str
    label: str = ""


@dataclass
class SuspensionProject:
    """Editable suspension GUI project state."""

    geometry_path: Path | None = None
    suspension_type: str = "double_wishbone"
    name: str = "GUI suspension"
    version: str = "0.0.0"
    units: Units = Units.MILLIMETERS
    hardpoints: dict[PointID, np.ndarray] = field(default_factory=dict)
    config: SuspensionConfig = field(
        default_factory=lambda: default_suspension_config()
    )
    settings: SuspensionSweepSettings = field(default_factory=SuspensionSweepSettings)
    curves: list[SuspensionCurve] = field(default_factory=list)
    optimization: SuspensionOptimizationConfig = field(
        default_factory=SuspensionOptimizationConfig
    )

    def build_suspension(self) -> Suspension:
        """Build a solver suspension from the editable project data."""
        suspension_class = get_suspension_class(self.suspension_type)
        if suspension_class is None:
            raise ValueError(f"Unsupported suspension type: {self.suspension_type}")
        return suspension_class(
            name=self.name,
            version=self.version,
            units=self.units,
            hardpoints={
                point_id: position.copy()
                for point_id, position in self.hardpoints.items()
            },
            config=self.config,
        )


@dataclass
class SuspensionSweepResult:
    """Solved suspension sweep data for GUI tables and plots."""

    states: list[SuspensionState]
    solver_infos: list[SolverInfo]
    rows: list[dict[str, float | bool | None]]
    curve_options: tuple[str, ...]


def build_wheel_travel_sweep(
    settings: SuspensionSweepSettings,
    *,
    locked_trackrod_inboard: np.ndarray | None = None,
) -> SweepConfig:
    """Build a wheel-center Z relative-displacement sweep."""
    values = np.linspace(settings.start, settings.stop, settings.steps)
    return build_wheel_travel_targets(
        [float(value) for value in values],
        locked_trackrod_inboard=locked_trackrod_inboard,
    )


def build_wheel_travel_targets(
    values: list[float],
    *,
    locked_trackrod_inboard: np.ndarray | None = None,
) -> SweepConfig:
    """Build wheel-center Z relative-displacement targets from values."""
    wheel_targets = [
        PointTarget(
            point_id=PointID.WHEEL_CENTER,
            direction=PointTargetAxis(Axis.Z),
            value=float(value),
            mode=TargetPositionMode.RELATIVE,
        )
        for value in values
    ]
    target_sweeps = [wheel_targets]

    if locked_trackrod_inboard is not None:
        locked = np.asarray(locked_trackrod_inboard, dtype=np.float64)
        for axis in (Axis.X, Axis.Y, Axis.Z):
            target_sweeps.append(
                [
                    PointTarget(
                        point_id=PointID.TRACKROD_INBOARD,
                        direction=PointTargetAxis(axis),
                        value=float(locked[axis]),
                        mode=TargetPositionMode.ABSOLUTE,
                    )
                    for _ in values
                ]
            )

    return SweepConfig(target_sweeps)


def supported_suspension_type_keys() -> tuple[str, ...]:
    """Return concrete suspension type keys supported by the core solver."""
    concrete: list[str] = []
    seen_classes: set[type[Suspension]] = set()
    for type_key in list_supported_types():
        suspension_class = get_suspension_class(type_key)
        if suspension_class is None or suspension_class in seen_classes:
            continue
        concrete.append(suspension_class.TYPE_KEY)
        seen_classes.add(suspension_class)
    return tuple(sorted(concrete))


def default_suspension_config() -> SuspensionConfig:
    """Create default editable suspension parameters."""
    return SuspensionConfig(
        steered=True,
        wheel=WheelConfig(
            offset=0.0,
            tire=TireConfig(
                aspect_ratio=0.55,
                section_width=270.0,
                static_radius_mm=283.1,
            ),
        ),
        cg_position=(1250.0, 0.0, 450.0),
        wheelbase=2500.0,
    )


def create_default_suspension_project(
    suspension_type: str = "double_wishbone",
) -> SuspensionProject:
    """Create a default editable suspension project for a supported type."""
    suspension_class = get_suspension_class(suspension_type)
    if suspension_class is None:
        raise ValueError(f"Unsupported suspension type: {suspension_type}")
    hardpoints = {
        point_id: _default_hardpoint(point_id)
        for point_id in sorted(suspension_class.REQUIRED_POINTS)
    }
    return SuspensionProject(
        suspension_type=suspension_class.TYPE_KEY,
        hardpoints=hardpoints,
        config=default_suspension_config(),
    )


def load_suspension_project(path: str | Path) -> SuspensionProject:
    """Load a GUI suspension project or legacy geometry YAML."""
    geometry_path = Path(path)
    if geometry_path.suffix == ".json":
        data = json.loads(geometry_path.read_text(encoding="utf-8"))
        if isinstance(data, dict) and data.get("module") == "suspension":
            return suspension_project_from_dict(data, geometry_path)
    suspension = load_geometry(geometry_path)
    return SuspensionProject(
        geometry_path=geometry_path,
        suspension_type=suspension.TYPE_KEY,
        name=suspension.name,
        version=suspension.version,
        units=suspension.units,
        hardpoints={
            point_id: np.asarray(position, dtype=np.float64).copy()
            for point_id, position in suspension.hardpoints.items()
        },
        config=suspension.config or default_suspension_config(),
    )


def suspension_internal_to_gui_vec3(value: object) -> np.ndarray:
    """Convert internal suspension coordinates to GUI-facing coordinates."""
    vec = np.asarray(value, dtype=np.float64)
    return np.asarray([-float(vec[0]), -float(vec[1]), float(vec[2])], dtype=np.float64)


def suspension_gui_to_internal_vec3(value: object) -> np.ndarray:
    """Convert GUI-facing suspension coordinates to internal coordinates."""
    return suspension_internal_to_gui_vec3(value)


def suspension_metric_internal_to_gui(
    metric_name: str,
    value: float | None,
) -> float | None:
    """Convert internal-coordinate scalar metrics to GUI-facing values."""
    if value is None:
        return None
    if metric_name in {"svic_x_mm", "svsa_length_mm"}:
        return -float(value)
    if metric_name in {"fvic_y_mm", "fvsa_length_mm"}:
        return -float(value)
    return float(value)


def suspension_project_to_dict(project: SuspensionProject) -> dict[str, Any]:
    """Convert a suspension GUI project to the shared JSON project format."""
    return build_project_document(
        module="suspension",
        system_type=project.suspension_type,
        name=project.name,
        version=project.version,
        units=project.units.name,
        hardpoints={
            point_id.name: _vec3_to_dict(position, gui_coordinates=True)
            for point_id, position in sorted(project.hardpoints.items())
        },
        parameters={
            "coordinate_system": SUSPENSION_GUI_COORDINATE_SYSTEM,
            "config": _suspension_config_to_dict(project.config, gui_coordinates=True),
        },
        simulation={
            "start": float(project.settings.start),
            "stop": float(project.settings.stop),
            "steps": int(project.settings.steps),
            "optimization": optimization_config_to_dict(project.optimization),
        },
        curves=[asdict(curve) for curve in project.curves],
    )


def suspension_project_from_dict(
    data: dict[str, Any],
    geometry_path: Path | None = None,
) -> SuspensionProject:
    """Create a suspension GUI project from the shared JSON project format."""
    module = data.get("module")
    if module != "suspension":
        raise ValueError(f"Expected suspension project, got {module!r}")

    settings_data = data.get("simulation", {})
    parameters = data.get("parameters", {})
    coordinate_system = str(
        parameters.get("coordinate_system", SUSPENSION_INTERNAL_COORDINATE_SYSTEM)
    )
    gui_coordinates = coordinate_system == SUSPENSION_GUI_COORDINATE_SYSTEM
    return SuspensionProject(
        geometry_path=geometry_path,
        suspension_type=str(data.get("system_type", "double_wishbone")),
        name=str(data.get("name", "GUI suspension")),
        version=str(data.get("version", "0.0.0")),
        units=coerce_enum(Units, data.get("units", Units.MILLIMETERS.name)),
        hardpoints=_suspension_hardpoints_from_dict(
            data.get("hardpoints", {}),
            gui_coordinates=gui_coordinates,
        ),
        config=SuspensionConfig.model_validate(
            _suspension_config_from_dict(
                parameters.get("config", default_suspension_config().model_dump()),
                gui_coordinates=gui_coordinates,
            )
        ),
        settings=SuspensionSweepSettings(
            start=float(settings_data.get("start", -40.0)),
            stop=float(settings_data.get("stop", 120.0)),
            steps=int(settings_data.get("steps", 41)),
        ),
        curves=[SuspensionCurve(**curve) for curve in data.get("curves", [])],
        optimization=optimization_config_from_dict(settings_data.get("optimization")),
    )


def solve_suspension_project(project: SuspensionProject) -> SuspensionSweepResult:
    """Solve the current suspension project sweep."""
    suspension = project.build_suspension()

    sweep = build_wheel_travel_sweep(
        project.settings,
        locked_trackrod_inboard=project.hardpoints.get(PointID.TRACKROD_INBOARD),
    )
    states, solver_infos = solve_sweep(suspension, sweep)
    rows = [
        _row_from_state(index, travel, state, solver_info, suspension)
        for index, (travel, state, solver_info) in enumerate(
            zip(_wheel_travel_values(project.settings), states, solver_infos)
        )
    ]
    return SuspensionSweepResult(
        states=states,
        solver_infos=solver_infos,
        rows=rows,
        curve_options=_curve_options(rows),
    )


def _solve_suspension_project_with_settings(
    project: SuspensionProject,
    settings: SuspensionSweepSettings,
) -> SuspensionSweepResult:
    sweep_project = replace(project, settings=settings)
    return solve_suspension_project(sweep_project)


def _coarse_optimization_settings(
    settings: SuspensionSweepSettings,
) -> SuspensionSweepSettings:
    coarse_steps = min(settings.steps, OPTIMIZATION_COARSE_MAX_STEPS)
    if settings.steps >= 3:
        coarse_steps = max(3, coarse_steps)
    return SuspensionSweepSettings(
        start=float(settings.start),
        stop=float(settings.stop),
        steps=int(coarse_steps),
    )


def _hardpoints_from_optimization_values(
    base_hardpoints: dict[PointID, np.ndarray],
    variable_names: tuple[str, ...],
    values: np.ndarray,
) -> dict[PointID, np.ndarray]:
    hardpoints = {
        point_id: np.asarray(position, dtype=np.float64).copy()
        for point_id, position in base_hardpoints.items()
    }
    for variable_name, variable_value in zip(variable_names, values, strict=True):
        set_suspension_optimization_variable(hardpoints, variable_name, float(variable_value))
    return hardpoints


def analyze_suspension_optimization_variables(
    project: SuspensionProject,
    *,
    targets: list[SuspensionOptimizationTarget],
    variable_names: tuple[str, ...],
    variable_delta_limit: float,
    solver_mode: str = "dual_path",
    pair_delta_constraints: list[SuspensionOptimizationPairDeltaConstraint] | None = None,
    cancel_event: threading.Event | None = None,
) -> SuspensionOptimizationVariableAnalysisResult:
    """Screen optimization variables with constrained global sensitivity."""
    if not variable_names:
        raise ValueError("At least one optimization variable is required")
    if variable_delta_limit <= 0.0:
        raise ValueError("variable_delta_limit must be positive")
    active_targets = [target for target in targets if target.enabled]
    if not active_targets:
        raise ValueError("At least one optimization target must be enabled")
    active_pair_delta_constraints = [
        constraint for constraint in (pair_delta_constraints or []) if constraint.enabled
    ]

    initial_hardpoints = {
        point_id: np.asarray(position, dtype=np.float64).copy()
        for point_id, position in project.hardpoints.items()
    }
    raise_if_cancelled(cancel_event)
    x0 = np.asarray(
        [
            get_suspension_optimization_variable(initial_hardpoints, name)
            for name in variable_names
        ],
        dtype=np.float64,
    )
    lower = x0 - float(variable_delta_limit)
    upper = x0 + float(variable_delta_limit)

    axis_index_map = {"x": 0, "y": 1, "z": 2}
    constraint_rows: list[np.ndarray] = []
    for constraint in active_pair_delta_constraints:
        point_a = PointID[constraint.point_a]
        point_b = PointID[constraint.point_b]
        for axis_name in constraint.axes:
            row = np.zeros_like(x0)
            for index, variable_name in enumerate(variable_names):
                point_name, variable_axis = variable_name.rsplit("_", 1)
                if variable_axis.lower() != axis_name.lower():
                    continue
                if point_name == point_a.name:
                    row[index] = -1.0
                elif point_name == point_b.name:
                    row[index] = 1.0
            if np.any(np.abs(row) > 0.0):
                constraint_rows.append(row)
    constraint_matrix = (
        np.vstack(constraint_rows).astype(np.float64)
        if constraint_rows
        else np.zeros((0, x0.size), dtype=np.float64)
    )
    parameterization = build_linear_constraint_parameterization(
        anchor=x0,
        lower=lower,
        upper=upper,
        constraint_matrix=constraint_matrix,
    )
    raise_if_cancelled(cancel_event)

    def evaluate(values: np.ndarray) -> np.ndarray:
        raise_if_cancelled(cancel_event)
        hardpoints = _hardpoints_from_optimization_values(
            initial_hardpoints,
            variable_names,
            values,
        )
        rows = solve_suspension_project(replace(project, hardpoints=hardpoints)).rows
        metric_residuals = suspension_optimization_residuals(
            rows,
            active_targets,
            values=values,
            baseline_values=x0,
            regularization_weight=0.0,
        )
        pair_residuals = suspension_pair_delta_constraint_residuals(
            hardpoints,
            initial_hardpoints,
            active_pair_delta_constraints,
        )
        return np.concatenate((metric_residuals, pair_residuals))

    base_residual = evaluate(x0)
    residual_size = int(base_residual.size)
    variable_count = int(x0.size)

    def evaluate_objective(values: np.ndarray) -> float:
        raise_if_cancelled(cancel_event)
        residuals = evaluate(values)
        return float(np.linalg.norm(residuals))

    morris_trajectories = 6
    sobol_base_samples = 8
    morris_stats, morris_mu_stars = run_morris_screening(
        parameterization=parameterization,
        evaluate_objective=evaluate_objective,
        trajectories=morris_trajectories,
        cancel_event=cancel_event,
    )
    raise_if_cancelled(cancel_event)
    selected_direction_indices = pick_reduced_directions_from_morris(
        parameterization=parameterization,
        morris_mu_stars=morris_mu_stars,
        max_directions=min(3, parameterization.direction_count),
    )
    sobol_stats = run_pairwise_sobol_screening(
        parameterization=parameterization,
        evaluate_objective=evaluate_objective,
        direction_indices=selected_direction_indices,
        base_samples=sobol_base_samples,
        cancel_event=cancel_event,
    )
    raise_if_cancelled(cancel_event)
    sobol_by_variable: dict[int, tuple[float, float]] = {}
    for stat in sobol_stats:
        projected = np.abs(parameterization.null_basis[:, stat.variable_index])
        if float(np.sum(projected)) <= 1e-12:
            continue
        dominant_index = int(np.argmax(projected))
        sobol_by_variable[dominant_index] = (
            float(stat.first_order),
            float(stat.total_order),
        )

    items: list[SuspensionOptimizationVariableAnalysisItem] = []
    max_morris = max((stat.mu_star for stat in morris_stats), default=0.0)
    low_threshold = max(max_morris * 0.1, 1e-8)
    for stat in morris_stats:
        variable_name = variable_names[stat.variable_index]
        sobol_first, sobol_total = sobol_by_variable.get(stat.variable_index, (None, None))
        if stat.mu_star <= low_threshold and (sobol_total is None or sobol_total <= 0.02):
            recommendation = "suppress"
            detail = "Low constrained global influence in Morris/Sobol screening"
        elif stat.mu_star > 1e-8 and (sobol_total is None or sobol_total > 0.02):
            recommendation = "recommended"
            detail = "High constrained global influence for current targets"
        else:
            recommendation = "secondary"
            detail = "Useful but lower-priority variable under current targets"
        items.append(
            SuspensionOptimizationVariableAnalysisItem(
                variable_name=variable_name,
                morris_mu_star=float(stat.mu_star),
                morris_sigma=float(stat.sigma),
                sobol_first_order=sobol_first,
                sobol_total=sobol_total,
                recommendation=recommendation,
                detail=detail,
            )
        )

    items.sort(
        key=lambda item: (
            0
            if item.recommendation == "recommended"
            else 1 if item.recommendation == "secondary" else 2,
            -item.morris_mu_star,
            item.variable_name,
        )
    )
    recommended_items = [item for item in items if item.recommendation == "recommended"]
    items, recommended_items = _validate_recommended_variable_subset(
        project=project,
        targets=active_targets,
        variable_delta_limit=variable_delta_limit,
        solver_mode=solver_mode,
        pair_delta_constraints=active_pair_delta_constraints,
        items=items,
        recommended_items=recommended_items,
        cancel_event=cancel_event,
    )
    return SuspensionOptimizationVariableAnalysisResult(
        items=tuple(items),
        recommended_variable_names=tuple(
            item.variable_name for item in recommended_items
        ),
        residual_size=residual_size,
        variable_count=variable_count,
        constraint_rank=parameterization.constraint_rank,
        effective_rank=parameterization.direction_count,
        morris_trajectories=morris_trajectories,
        sobol_base_samples=sobol_base_samples,
        sobol_direction_count=len(selected_direction_indices),
        method="constraint_parameterization+morris+sobol+validated_topk",
    )


def _validate_recommended_variable_subset(
    *,
    project: SuspensionProject,
    targets: list[SuspensionOptimizationTarget],
    variable_delta_limit: float,
    solver_mode: str,
    pair_delta_constraints: list[SuspensionOptimizationPairDeltaConstraint],
    items: list[SuspensionOptimizationVariableAnalysisItem],
    recommended_items: list[SuspensionOptimizationVariableAnalysisItem],
    cancel_event: threading.Event | None,
) -> tuple[list[SuspensionOptimizationVariableAnalysisItem], list[SuspensionOptimizationVariableAnalysisItem]]:
    raise_if_cancelled(cancel_event)
    ranked_candidates = [
        item.variable_name
        for item in items
        if item.recommendation != "suppress"
    ]
    if not ranked_candidates:
        ranked_candidates = [item.variable_name for item in items]
    if not ranked_candidates:
        return items, recommended_items

    validation_sizes: list[int] = []
    for size in OPTIMIZATION_ANALYSIS_TOP_K_SIZES:
        if size <= len(ranked_candidates):
            validation_sizes.append(size)
    if len(ranked_candidates) not in validation_sizes:
        validation_sizes.append(len(ranked_candidates))
    validation_sizes = sorted(set(validation_sizes))

    evaluated_subsets: list[tuple[tuple[str, ...], float]] = []

    for size in validation_sizes:
        raise_if_cancelled(cancel_event)
        subset = tuple(ranked_candidates[:size])
        result = optimize_suspension_hardpoints(
            project,
            targets=targets,
            variable_names=subset,
            variable_delta_limit=variable_delta_limit,
            solver_mode=solver_mode,
            pair_delta_constraints=pair_delta_constraints,
            cancel_event=cancel_event,
            max_rounds=2,
            convergence_tolerance=5e-2,
        )
        evaluated_subsets.append((subset, float(result.final_cost)))

    best_cost = min((cost for _subset, cost in evaluated_subsets), default=float("inf"))
    tolerance = 1.0 + OPTIMIZATION_ANALYSIS_VALIDATION_COST_TOLERANCE
    eligible_subsets = [
        (subset, cost)
        for subset, cost in evaluated_subsets
        if cost <= best_cost * tolerance
    ]
    if eligible_subsets:
        best_subset, _selected_cost = min(
            eligible_subsets,
            key=lambda item: (len(item[0]), item[1], item[0]),
        )
    else:
        best_subset = tuple()

    if not best_subset:
        return items, recommended_items

    best_subset_names = set(best_subset)
    updated_items: list[SuspensionOptimizationVariableAnalysisItem] = []
    updated_recommended_items: list[SuspensionOptimizationVariableAnalysisItem] = []
    for item in items:
        if item.variable_name in best_subset_names:
            updated_item = SuspensionOptimizationVariableAnalysisItem(
                variable_name=item.variable_name,
                morris_mu_star=item.morris_mu_star,
                morris_sigma=item.morris_sigma,
                sobol_first_order=item.sobol_first_order,
                sobol_total=item.sobol_total,
                recommendation="recommended",
                detail="Validated by low-budget subset optimization for current targets",
            )
        elif item.recommendation == "recommended":
            updated_item = SuspensionOptimizationVariableAnalysisItem(
                variable_name=item.variable_name,
                morris_mu_star=item.morris_mu_star,
                morris_sigma=item.morris_sigma,
                sobol_first_order=item.sobol_first_order,
                sobol_total=item.sobol_total,
                recommendation="secondary",
                detail="Sensitive variable, but excluded by subset validation for current targets",
            )
        else:
            updated_item = item
        updated_items.append(updated_item)
        if updated_item.recommendation == "recommended":
            updated_recommended_items.append(updated_item)

    updated_items.sort(
        key=lambda item: (
            0
            if item.recommendation == "recommended"
            else 1 if item.recommendation == "secondary" else 2,
            -item.morris_mu_star,
            item.variable_name,
        )
    )
    updated_recommended_items.sort(key=lambda item: updated_items.index(item))
    return updated_items, updated_recommended_items


def optimize_suspension_hardpoints(
    project: SuspensionProject,
    *,
    targets: list[SuspensionOptimizationTarget],
    variable_names: tuple[str, ...],
    variable_delta_limit: float,
    solver_mode: str = "dual_path",
    pair_delta_constraints: list[SuspensionOptimizationPairDeltaConstraint] | None = None,
    progress_callback: ProgressCallback | None = None,
    cancel_event: threading.Event | None = None,
    max_rounds: int = 6,
    convergence_tolerance: float = 1e-3,
) -> SuspensionOptimizationResult:
    """Optimize suspension hardpoints with CMA-ES global search + local polish."""
    if not variable_names:
        raise ValueError("At least one optimization variable is required")
    if variable_delta_limit <= 0.0:
        raise ValueError("variable_delta_limit must be positive")
    active_targets = [target for target in targets if target.enabled]
    if not active_targets:
        raise ValueError("At least one optimization target must be enabled")
    active_pair_delta_constraints = [
        constraint for constraint in (pair_delta_constraints or []) if constraint.enabled
    ]
    supported_solver_modes = {
        "dual_path",
        "baseline_local_only",
        "cma_es_then_local_refine",
        "cma_es_only",
    }
    if solver_mode not in supported_solver_modes:
        raise ValueError(f"Unsupported suspension optimization solver mode: {solver_mode!r}")

    initial_hardpoints = {
        point_id: np.asarray(position, dtype=np.float64).copy()
        for point_id, position in project.hardpoints.items()
    }
    start_time = time.perf_counter()
    evaluation_count = 0

    def emit_progress(phase: str, message: str) -> None:
        raise_if_cancelled(cancel_event)
        if progress_callback is None:
            return
        progress_callback(
            SuspensionOptimizationProgress(
                phase=phase,
                evaluations=evaluation_count,
                elapsed_seconds=time.perf_counter() - start_time,
                message=message,
            )
        )

    emit_progress("starting", "Preparing optimization")
    baseline_result = solve_suspension_project(project)
    baseline_rows = baseline_result.rows
    baseline_x = np.asarray(
        [
            get_suspension_optimization_variable(initial_hardpoints, name)
            for name in variable_names
        ],
        dtype=np.float64,
    )
    baseline_metric_residuals = suspension_optimization_residuals(
        baseline_rows,
        active_targets,
        values=baseline_x,
        baseline_values=baseline_x,
        regularization_weight=0.0,
    )
    baseline_pair_residuals = suspension_pair_delta_constraint_residuals(
        initial_hardpoints,
        initial_hardpoints,
        active_pair_delta_constraints,
    )
    baseline_residuals = np.concatenate(
        (baseline_metric_residuals, baseline_pair_residuals)
    )
    baseline_cost = float(np.linalg.norm(baseline_residuals))
    lower = baseline_x - float(variable_delta_limit)
    upper = baseline_x + float(variable_delta_limit)
    coarse_settings = _coarse_optimization_settings(project.settings)

    def evaluate_values(
        values: np.ndarray,
        *,
        settings: SuspensionSweepSettings,
        regularization_weight: float,
    ) -> tuple[np.ndarray, dict[PointID, np.ndarray], list[dict[str, float | bool | None]]]:
        raise_if_cancelled(cancel_event)
        hardpoints = _hardpoints_from_optimization_values(
            initial_hardpoints,
            variable_names,
            values,
        )
        trial_project = replace(project, hardpoints=hardpoints, settings=settings)
        rows = solve_suspension_project(trial_project).rows
        metric_residuals = suspension_optimization_residuals(
            rows,
            active_targets,
            values=values,
            baseline_values=baseline_x,
            regularization_weight=regularization_weight,
        )
        pair_residuals = suspension_pair_delta_constraint_residuals(
            hardpoints,
            initial_hardpoints,
            active_pair_delta_constraints,
        )
        if pair_residuals.size == 0:
            residuals = metric_residuals
        else:
            residuals = np.concatenate((metric_residuals, pair_residuals))
        return residuals, hardpoints, rows

    def run_least_squares_stage(
        *,
        start_values: np.ndarray,
        settings: SuspensionSweepSettings,
        stage_label: str,
    ) -> tuple[object, float, dict[PointID, np.ndarray], list[dict[str, float | bool | None]]]:
        def residual(values: np.ndarray) -> np.ndarray:
            nonlocal evaluation_count
            evaluation_count += 1
            try:
                raise_if_cancelled(cancel_event)
                residuals, _hardpoints, _rows = evaluate_values(
                    values,
                    settings=settings,
                    regularization_weight=OPTIMIZATION_REGULARIZATION_WEIGHT,
                )
            except OptimizationCancelledError:
                raise
            except Exception:
                return np.full_like(baseline_residuals, 1e3, dtype=np.float64)
            if evaluation_count == 1 or evaluation_count % 3 == 0:
                emit_progress("solving", f"{stage_label}, evaluations: {evaluation_count}")
            return residuals

        raise_if_cancelled(cancel_event)
        emit_progress("solving", f"Running {stage_label}")
        result = least_squares(residual, start_values, bounds=(lower, upper), method="trf")
        objective_residuals, hardpoints, rows = evaluate_values(
            np.asarray(result.x, dtype=np.float64),
            settings=settings,
            regularization_weight=0.0,
        )
        return result, float(np.linalg.norm(objective_residuals)), hardpoints, rows

    def run_cma_es_stage() -> np.ndarray:
        if baseline_x.size <= 1:
            emit_progress(
                "solving",
                "Skipping CMA-ES for a single optimization variable; using local refine",
            )
            return baseline_x.copy()
        sigma0 = max(float(variable_delta_limit) / 3.0, 1e-3)
        strategy = cma.CMAEvolutionStrategy(
            baseline_x,
            sigma0,
            {
                "bounds": [lower.tolist(), upper.tolist()],
                "popsize": OPTIMIZATION_CMA_POPSIZE,
                "maxiter": OPTIMIZATION_CMA_MAX_ITER,
                "seed": 0,
                "verbose": -9,
            },
        )
        best_values = baseline_x.copy()
        best_cost = float("inf")
        iteration = 0

        while not strategy.stop():
            raise_if_cancelled(cancel_event)
            iteration += 1
            candidates = strategy.ask()
            costs: list[float] = []
            for candidate in candidates:
                nonlocal evaluation_count
                evaluation_count += 1
                raise_if_cancelled(cancel_event)
                clipped = np.clip(np.asarray(candidate, dtype=np.float64), lower, upper)
                try:
                    residuals, _hardpoints, _rows = evaluate_values(
                        clipped,
                        settings=coarse_settings,
                        regularization_weight=0.0,
                    )
                    cost = float(np.linalg.norm(residuals))
                except OptimizationCancelledError:
                    raise
                except Exception:
                    cost = 1e6
                costs.append(cost)
                if cost < best_cost:
                    best_cost = cost
                    best_values = clipped.copy()
            strategy.tell(candidates, costs)
            emit_progress(
                "solving",
                f"CMA-ES iter {iteration}/{OPTIMIZATION_CMA_MAX_ITER}, best coarse cost {best_cost:.6g}",
            )

        return best_values

    def build_result_from_values(
        *,
        values: np.ndarray,
        rows: list[dict[str, float | bool | None]],
        hardpoints: dict[PointID, np.ndarray],
        rounds_completed: int,
        success: bool,
        message: str,
    ) -> SuspensionOptimizationResult:
        final_metric_residuals = suspension_optimization_residuals(
            rows,
            active_targets,
            values=np.asarray(values, dtype=np.float64),
            baseline_values=baseline_x,
            regularization_weight=0.0,
        )
        final_pair_residuals = suspension_pair_delta_constraint_residuals(
            hardpoints,
            initial_hardpoints,
            active_pair_delta_constraints,
        )
        final_residuals = np.concatenate((final_metric_residuals, final_pair_residuals))
        summaries = [
            SuspensionOptimizationMetricSummary(
                metric_name=target.metric_name,
                trend=target.trend,
                target_mode=target.target_mode,
                target_delta=float(target.target_delta),
                initial_value=float(
                    suspension_metric_summary_value(
                        metric_series_from_rows(baseline_rows, target.metric_name),
                        target,
                    )
                ),
                final_value=float(
                    suspension_metric_summary_value(
                        metric_series_from_rows(rows, target.metric_name),
                        target,
                    )
                ),
            )
            for target in active_targets
        ]
        return SuspensionOptimizationResult(
            hardpoints=hardpoints,
            initial_cost=baseline_cost,
            final_cost=float(np.linalg.norm(final_residuals)),
            solver_mode=solver_mode,
            rounds_completed=rounds_completed,
            total_evaluations=evaluation_count,
            success=success,
            message=message,
            applied_values={
                name: get_suspension_optimization_variable(hardpoints, name)
                for name in variable_names
            },
            target_summaries=summaries,
        )

    def run_iterative_full_refine(
        *,
        start_values: np.ndarray,
        candidate_label: str,
    ) -> tuple[
        object,
        float,
        dict[PointID, np.ndarray],
        list[dict[str, float | bool | None]],
        int,
    ]:
        current_values = np.asarray(start_values, dtype=np.float64).copy()
        current_cost = float("inf")
        best_local_result = None
        best_local_cost = float("inf")
        best_local_hardpoints = {
            point_id: position.copy() for point_id, position in initial_hardpoints.items()
        }
        best_local_rows = baseline_rows
        rounds = 0

        for round_index in range(1, max_rounds + 1):
            raise_if_cancelled(cancel_event)
            result, cost, hardpoints, rows = run_least_squares_stage(
                start_values=current_values,
                settings=project.settings,
                stage_label=f"{candidate_label} round {round_index}/{max_rounds}",
            )
            rounds = round_index
            if cost < best_local_cost:
                best_local_cost = cost
                best_local_result = result
                best_local_hardpoints = hardpoints
                best_local_rows = rows
            improvement = current_cost - cost
            improvement_ratio = improvement / max(current_cost, 1e-12)
            emit_progress(
                "solving",
                f"{candidate_label} round {round_index}/{max_rounds} cost {cost:.6g}",
            )
            current_values = np.asarray(result.x, dtype=np.float64)
            if current_cost < float("inf") and (
                improvement <= 0.0 or improvement_ratio <= convergence_tolerance
            ):
                break
            current_cost = cost

        if best_local_result is None:
            raise RuntimeError("Suspension optimization full refinement did not converge")
        return (
            best_local_result,
            best_local_cost,
            best_local_hardpoints,
            best_local_rows,
            rounds,
        )

    if solver_mode == "baseline_local_only":
        raise_if_cancelled(cancel_event)
        best_result, _best_cost, best_hardpoints, best_rows, rounds_completed = (
            run_iterative_full_refine(
                start_values=baseline_x,
                candidate_label="Baseline full refine",
            )
        )
        emit_progress("solving", "Baseline full refine selected as requested")
        optimization_result = build_result_from_values(
            values=np.asarray(best_result.x, dtype=np.float64),
            rows=best_rows,
            hardpoints=best_hardpoints,
            rounds_completed=rounds_completed,
            success=bool(best_result.success),
            message=str(best_result.message),
        )
        emit_progress(
            "finished",
            f"Optimization finished with Baseline Local Only; rounds {rounds_completed}",
        )
        return optimization_result

    if solver_mode == "cma_es_then_local_refine":
        raise_if_cancelled(cancel_event)
        cma_start_values = run_cma_es_stage()
        best_result, _best_cost, best_hardpoints, best_rows, rounds_completed = (
            run_iterative_full_refine(
                start_values=cma_start_values,
                candidate_label="CMA-ES full refine",
            )
        )
        emit_progress("solving", "CMA-ES + local refine selected as requested")
        optimization_result = build_result_from_values(
            values=np.asarray(best_result.x, dtype=np.float64),
            rows=best_rows,
            hardpoints=best_hardpoints,
            rounds_completed=rounds_completed,
            success=bool(best_result.success),
            message=str(best_result.message),
        )
        emit_progress(
            "finished",
            f"Optimization finished with CMA-ES + Local Refine; rounds {rounds_completed}",
        )
        return optimization_result

    if solver_mode == "cma_es_only":
        raise_if_cancelled(cancel_event)
        best_values = run_cma_es_stage()
        final_residuals, best_hardpoints, best_rows = evaluate_values(
            best_values,
            settings=project.settings,
            regularization_weight=0.0,
        )
        optimization_result = build_result_from_values(
            values=best_values,
            rows=best_rows,
            hardpoints=best_hardpoints,
            rounds_completed=0,
            success=True,
            message=f"CMA-ES best final cost {float(np.linalg.norm(final_residuals)):.6g}",
        )
        emit_progress(
            "finished",
            "Optimization finished with CMA-ES Only; no local refine rounds",
        )
        return optimization_result

    raise_if_cancelled(cancel_event)
    baseline_result_local, baseline_cost_local, baseline_hardpoints_local, baseline_rows_local, baseline_rounds = (
        run_iterative_full_refine(
            start_values=baseline_x,
            candidate_label="Baseline full refine",
        )
    )
    emit_progress("solving", f"Baseline full refine best cost {baseline_cost_local:.6g}")

    raise_if_cancelled(cancel_event)
    cma_start_values = run_cma_es_stage()
    cma_result_local, cma_cost_local, cma_hardpoints_local, cma_rows_local, cma_rounds = (
        run_iterative_full_refine(
            start_values=cma_start_values,
            candidate_label="CMA-ES full refine",
        )
    )
    emit_progress("solving", f"CMA-ES full refine best cost {cma_cost_local:.6g}")

    raise_if_cancelled(cancel_event)
    if baseline_cost_local <= cma_cost_local:
        best_result = baseline_result_local
        best_hardpoints = baseline_hardpoints_local
        best_rows = baseline_rows_local
        rounds_completed = baseline_rounds
    else:
        best_result = cma_result_local
        best_hardpoints = cma_hardpoints_local
        best_rows = cma_rows_local
        rounds_completed = cma_rounds

    optimization_result = build_result_from_values(
        values=np.asarray(best_result.x, dtype=np.float64),
        rows=best_rows,
        hardpoints=best_hardpoints,
        rounds_completed=rounds_completed,
        success=bool(best_result.success),
        message=str(best_result.message),
    )
    emit_progress(
        "finished",
        f"Optimization finished with Dual Path; winning polish used {rounds_completed} round(s)",
    )
    return optimization_result


def solve_suspension_project_at_travel(
    project: SuspensionProject,
    wheel_travel: float,
) -> SuspensionSweepResult:
    """Solve one suspension state at a wheel-travel input."""
    suspension = project.build_suspension()
    states, solver_infos = solve_sweep(
        suspension,
        build_wheel_travel_targets(
            [float(wheel_travel)],
            locked_trackrod_inboard=project.hardpoints.get(PointID.TRACKROD_INBOARD),
        ),
    )
    rows = [
        _row_from_state(
            0,
            float(wheel_travel),
            states[0],
            solver_infos[0],
            suspension,
        )
    ]
    return SuspensionSweepResult(
        states=states,
        solver_infos=solver_infos,
        rows=rows,
        curve_options=_curve_options(rows),
    )


def curve_specs_for_plot(
    curves: list[SuspensionCurve],
    selected_x_output: str,
    selected_y_output: str,
    selected_label: str,
) -> list[tuple[str, str, str]]:
    """Return saved suspension curve specs, or a live preview spec."""
    if curves:
        return [(curve.x_output, curve.y_output, curve.label) for curve in curves]
    label = selected_label.strip() or f"{selected_y_output} preview"
    return [(selected_x_output, selected_y_output, label)]


def load_suspension_hardpoints_csv(path: str | Path) -> dict[PointID, np.ndarray]:
    """Load suspension hardpoints from a point,x,y,z CSV file."""
    hardpoints: dict[PointID, np.ndarray] = {}
    with Path(path).open(newline="", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        if reader.fieldnames is None:
            raise ValueError("Suspension hardpoint CSV is empty")
        missing = {"point", "x", "y", "z"} - set(reader.fieldnames)
        if missing:
            columns = ", ".join(sorted(missing))
            raise ValueError(f"Missing required CSV columns: {columns}")
        for row in reader:
            point_id = PointID[row["point"].strip().upper()]
            hardpoints[point_id] = suspension_gui_to_internal_vec3(
                [float(row["x"]), float(row["y"]), float(row["z"])]
            )
    return hardpoints


def save_suspension_hardpoints_csv(
    hardpoints: dict[PointID, np.ndarray],
    path: str | Path,
) -> None:
    """Save suspension hardpoints to a point,x,y,z CSV file."""
    with Path(path).open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=("point", "x", "y", "z"))
        writer.writeheader()
        for point_id, position in sorted(hardpoints.items()):
            gui_position = suspension_internal_to_gui_vec3(position)
            writer.writerow(
                {
                    "point": point_id.name,
                    "x": f"{gui_position[0]:.12g}",
                    "y": f"{gui_position[1]:.12g}",
                    "z": f"{gui_position[2]:.12g}",
                }
            )


def save_suspension_project(project: SuspensionProject, path: str | Path) -> None:
    """Save an editable suspension project as shared GUI JSON."""
    write_project_document(suspension_project_to_dict(project), path)


def _suspension_hardpoints_from_dict(
    hardpoints: object,
    *,
    gui_coordinates: bool = False,
) -> dict[PointID, np.ndarray]:
    if not isinstance(hardpoints, dict):
        raise ValueError("Suspension project hardpoints must be an object")
    return {
        coerce_enum(PointID, name): (
            suspension_gui_to_internal_vec3(
                [float(value["x"]), float(value["y"]), float(value["z"])]
            )
            if gui_coordinates
            else np.asarray(
                [float(value["x"]), float(value["y"]), float(value["z"])],
                dtype=np.float64,
            )
        )
        for name, value in hardpoints.items()
    }


def _suspension_config_to_dict(
    config: SuspensionConfig,
    *,
    gui_coordinates: bool = False,
) -> dict[str, Any]:
    data = {
        "steered": config.steered,
        "wheel": {
            "offset": float(config.wheel.offset),
            "tire": {
                "aspect_ratio": float(config.wheel.tire.aspect_ratio),
                "section_width": float(config.wheel.tire.section_width),
                "static_radius_mm": float(config.wheel.tire.static_radius_mm),
            },
        },
        "cg_position": _vec3_to_dict(
            config.cg_position,
            gui_coordinates=gui_coordinates,
        ),
        "wheelbase": float(config.wheelbase),
        "upright_mounted_points": list(config.upright_mounted_points),
    }
    if config.camber_shim is not None:
        shim = config.camber_shim
        data["camber_shim"] = {
            "shim_face_point_a": _vec3_to_dict(
                shim.shim_face_point_a,
                gui_coordinates=gui_coordinates,
            ),
            "shim_face_point_b": _vec3_to_dict(
                shim.shim_face_point_b,
                gui_coordinates=gui_coordinates,
            ),
            "shim_face_normal": _vec3_to_dict(
                shim.shim_face_normal,
                gui_coordinates=gui_coordinates,
            ),
            "design_thickness": float(shim.design_thickness),
            "setup_thickness": float(shim.setup_thickness),
        }
    return data


def _suspension_config_from_dict(
    config: object,
    *,
    gui_coordinates: bool = False,
) -> dict[str, Any]:
    if not isinstance(config, dict):
        return default_suspension_config().model_dump()

    data = dict(config)
    wheel = data.get("wheel")
    if isinstance(wheel, dict):
        wheel_data = dict(wheel)
        tire = wheel_data.get("tire")
        if isinstance(tire, dict):
            tire_data = dict(tire)
            if "static_radius_mm" not in tire_data and "rim_diameter" in tire_data:
                section_width = float(tire_data.get("section_width", 0.0))
                aspect_ratio = float(tire_data.get("aspect_ratio", 0.0))
                rim_diameter_inches = float(tire_data["rim_diameter"])
                tire_data["static_radius_mm"] = (
                    rim_diameter_inches * MM_PER_INCH
                    + 2.0 * (aspect_ratio * section_width)
                ) / 2.0
            tire_data.pop("rim_diameter", None)
            wheel_data["tire"] = tire_data
        data["wheel"] = wheel_data
    if gui_coordinates and isinstance(data.get("cg_position"), dict):
        data["cg_position"] = _vec3_dict_from_gui_dict(data["cg_position"])

    camber_shim = data.get("camber_shim")
    if gui_coordinates and isinstance(camber_shim, dict):
        shim = dict(camber_shim)
        for key in ("shim_face_point_a", "shim_face_point_b", "shim_face_normal"):
            if isinstance(shim.get(key), dict):
                shim[key] = _vec3_dict_from_gui_dict(shim[key])
        data["camber_shim"] = shim
    return data


def _vec3_to_dict(
    value: object,
    *,
    gui_coordinates: bool = False,
) -> dict[str, float]:
    vec = (
        suspension_internal_to_gui_vec3(value)
        if gui_coordinates
        else np.asarray(value, dtype=np.float64)
    )
    return {"x": float(vec[0]), "y": float(vec[1]), "z": float(vec[2])}


def _vec3_dict_from_gui_dict(value: dict[str, Any]) -> dict[str, float]:
    vec = suspension_gui_to_internal_vec3(
        [float(value["x"]), float(value["y"]), float(value["z"])]
    )
    return {"x": float(vec[0]), "y": float(vec[1]), "z": float(vec[2])}


def _default_hardpoint(point_id: PointID) -> np.ndarray:
    defaults = {
        PointID.LOWER_WISHBONE_INBOARD_FRONT: (250.0, 400.0, 200.0),
        PointID.LOWER_WISHBONE_INBOARD_REAR: (-250.0, 450.0, 200.0),
        PointID.LOWER_WISHBONE_OUTBOARD: (0.0, 900.0, 200.0),
        PointID.UPPER_WISHBONE_INBOARD_FRONT: (225.0, 350.0, 500.0),
        PointID.UPPER_WISHBONE_INBOARD_REAR: (-275.0, 350.0, 500.0),
        PointID.UPPER_WISHBONE_OUTBOARD: (-25.0, 750.0, 500.0),
        PointID.TRACKROD_INBOARD: (50.0, 200.0, 250.0),
        PointID.TRACKROD_OUTBOARD: (150.0, 800.0, 275.0),
        PointID.AXLE_INBOARD: (-20.0, 800.0, 308.426),
        PointID.AXLE_OUTBOARD: (-20.0, 950.0, 313.426),
        PointID.CARRIER_STEERING_AXIS_LOWER: (15.0, 820.0, 230.0),
        PointID.CARRIER_STEERING_AXIS_UPPER: (15.0, 820.0, 470.0),
    }
    return np.asarray(defaults.get(point_id, (0.0, 0.0, 0.0)), dtype=np.float64)


def _wheel_travel_values(settings: SuspensionSweepSettings) -> list[float]:
    return [
        float(value)
        for value in np.linspace(settings.start, settings.stop, settings.steps)
    ]


def _row_from_state(
    index: int,
    wheel_travel: float,
    state: SuspensionState,
    solver_info: SolverInfo,
    suspension: Suspension,
) -> dict[str, float | bool | None]:
    metrics = {
        name: suspension_metric_internal_to_gui(name, value)
        for name, value in compute_metrics_for_state_from_suspension(
            state, suspension
        ).items()
    }
    if "roadwheel_angle_deg" in metrics and "toe_deg" not in metrics:
        metrics["toe_deg"] = metrics["roadwheel_angle_deg"]
    row: dict[str, float | bool | None] = {
        "step": index,
        "wheel_travel_mm": wheel_travel,
        "solver_converged": solver_info.converged,
        "solver_nfev": solver_info.nfev,
        "solver_max_residual": solver_info.max_residual,
    }
    row.update(metrics)
    return row


def _curve_options(rows: list[dict[str, float | bool | None]]) -> tuple[str, ...]:
    if not rows:
        return DEFAULT_CURVE_OPTIONS
    keys = tuple(
        key for key, value in rows[0].items() if isinstance(value, int | float)
    )
    defaults = tuple(option for option in DEFAULT_CURVE_OPTIONS if option in keys)
    extras = tuple(key for key in keys if key not in defaults)
    return defaults + extras
