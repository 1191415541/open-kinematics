"""Suspension curve-optimization helpers for the GUI workbench."""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field
from typing import Callable, Mapping, Sequence

import numpy as np

from kinematics.core.enums import Axis, PointID

SUSPENSION_OPTIMIZATION_TRENDS = ("ignore", "positive", "negative", "flat")
SUSPENSION_OPTIMIZATION_TARGET_MODES = (
    ("endpoint_delta", "End-to-end delta"),
    ("value_range", "Full-range variation"),
    ("absolute_value", "Absolute target"),
)
SUSPENSION_OPTIMIZATION_METRICS = (
    ("camber_deg", "Camber"),
    ("toe_deg", "Toe"),
)
DEFAULT_SUSPENSION_OPTIMIZATION_VARIABLES = (
    "TRACKROD_INBOARD_z",
    "TRACKROD_OUTBOARD_z",
    "UPPER_WISHBONE_OUTBOARD_z",
    "LOWER_WISHBONE_OUTBOARD_z",
)


def _default_suspension_optimization_pair_delta_constraints() -> list[
    "SuspensionOptimizationPairDeltaConstraint"
]:
    return [
        SuspensionOptimizationPairDeltaConstraint(
            point_a="UPPER_WISHBONE_INBOARD_FRONT",
            point_b="UPPER_WISHBONE_INBOARD_REAR",
            label="Upper wishbone inboard front/rear",
        ),
        SuspensionOptimizationPairDeltaConstraint(
            point_a="LOWER_WISHBONE_INBOARD_FRONT",
            point_b="LOWER_WISHBONE_INBOARD_REAR",
            label="Lower wishbone inboard front/rear",
        ),
    ]


@dataclass
class SuspensionOptimizationTarget:
    """One metric target for suspension sweep optimization."""

    metric_name: str
    target_delta: float = 0.0
    trend: str = "ignore"
    target_mode: str = "endpoint_delta"
    enabled: bool = True

    def __post_init__(self) -> None:
        if self.trend not in SUSPENSION_OPTIMIZATION_TRENDS:
            raise ValueError(f"Unsupported optimization trend: {self.trend!r}")
        supported_modes = {mode for mode, _label in SUSPENSION_OPTIMIZATION_TARGET_MODES}
        if self.target_mode not in supported_modes:
            raise ValueError(
                f"Unsupported optimization target mode: {self.target_mode!r}"
            )


@dataclass
class SuspensionOptimizationPairDeltaConstraint:
    """Optional pair-wise hardpoint delta constraint."""

    point_a: str
    point_b: str
    label: str = ""
    enabled: bool = False
    axes: tuple[str, ...] = ("x", "y", "z")

    def __post_init__(self) -> None:
        if not self.label:
            self.label = f"{self.point_a} / {self.point_b}"
        self.axes = tuple(axis.lower() for axis in self.axes)
        if not self.axes:
            raise ValueError("axes must not be empty")
        for axis_name in self.axes:
            if axis_name not in ("x", "y", "z"):
                raise ValueError(f"Unsupported optimization axis: {axis_name!r}")
        _point_id_from_name(self.point_a)
        _point_id_from_name(self.point_b)

    def key(self) -> str:
        return f"{self.point_a}->{self.point_b}:{','.join(self.axes)}"


@dataclass
class SuspensionOptimizationConfig:
    """Editable optimization settings stored with a suspension project."""

    variable_delta_limit: float = 5.0
    variable_names: list[str] = field(
        default_factory=lambda: list(DEFAULT_SUSPENSION_OPTIMIZATION_VARIABLES)
    )
    targets: list[SuspensionOptimizationTarget] = field(
        default_factory=lambda: [
            SuspensionOptimizationTarget(
                metric_name="camber_deg",
                trend="negative",
            ),
            SuspensionOptimizationTarget(
                metric_name="toe_deg",
                trend="positive",
            ),
        ]
    )
    pair_delta_constraints: list[SuspensionOptimizationPairDeltaConstraint] = field(
        default_factory=_default_suspension_optimization_pair_delta_constraints
    )

    def __post_init__(self) -> None:
        if self.variable_delta_limit <= 0.0:
            raise ValueError("variable_delta_limit must be positive")


@dataclass
class SuspensionOptimizationMetricSummary:
    """Before/after summary for one optimized suspension metric."""

    metric_name: str
    trend: str
    target_mode: str
    target_delta: float
    initial_value: float
    final_value: float


@dataclass
class SuspensionOptimizationResult:
    """Result of one suspension hardpoint optimization run."""

    hardpoints: dict[PointID, np.ndarray]
    initial_cost: float
    final_cost: float
    rounds_completed: int
    total_evaluations: int
    success: bool
    message: str
    applied_values: dict[str, float]
    target_summaries: list[SuspensionOptimizationMetricSummary]


@dataclass(frozen=True)
class SuspensionOptimizationVariableAnalysisItem:
    """One variable's constrained global-sensitivity summary."""

    variable_name: str
    morris_mu_star: float
    morris_sigma: float
    sobol_first_order: float | None
    sobol_total: float | None
    recommendation: str
    detail: str


@dataclass(frozen=True)
class SuspensionOptimizationVariableAnalysisResult:
    """Pre-optimization constrained global-sensitivity result."""

    items: tuple[SuspensionOptimizationVariableAnalysisItem, ...]
    recommended_variable_names: tuple[str, ...]
    residual_size: int
    variable_count: int
    constraint_rank: int
    effective_rank: int
    morris_trajectories: int
    sobol_base_samples: int
    sobol_direction_count: int
    method: str


@dataclass(frozen=True)
class SuspensionOptimizationProgress:
    """Progress event emitted while optimizing suspension hardpoints."""

    phase: str
    evaluations: int
    elapsed_seconds: float
    message: str


def available_suspension_optimization_variables(
    hardpoints: Mapping[PointID, np.ndarray],
) -> tuple[str, ...]:
    """Return all editable hardpoint-axis variables for optimization."""
    names: list[str] = []
    for point_id in sorted(hardpoints):
        for axis_name in ("x", "y", "z"):
            names.append(f"{point_id.name}_{axis_name}")
    return tuple(names)


def parse_suspension_optimization_variable_names(value: str) -> tuple[str, ...]:
    """Parse comma-separated optimization variable names from the GUI."""
    names = tuple(
        item.strip()
        for item in value.replace("\n", ",").split(",")
        if item.strip()
    )
    if not names:
        raise ValueError("At least one optimization variable is required")
    return names


def get_suspension_optimization_variable(
    hardpoints: Mapping[PointID, np.ndarray],
    variable_name: str,
) -> float:
    """Read one hardpoint-axis variable by its serialized name."""
    point_id, axis = _variable_parts(variable_name)
    return float(np.asarray(hardpoints[point_id], dtype=np.float64)[axis])


def set_suspension_optimization_variable(
    hardpoints: dict[PointID, np.ndarray],
    variable_name: str,
    value: float,
) -> None:
    """Set one hardpoint-axis variable by its serialized name."""
    point_id, axis = _variable_parts(variable_name)
    updated = np.asarray(hardpoints[point_id], dtype=np.float64).copy()
    updated[axis] = float(value)
    hardpoints[point_id] = updated


def optimization_config_to_dict(
    config: SuspensionOptimizationConfig,
) -> dict[str, object]:
    """Convert optimization settings to project-file JSON data."""
    return {
        "variable_delta_limit": float(config.variable_delta_limit),
        "variable_names": list(config.variable_names),
        "targets": [asdict(target) for target in config.targets],
        "pair_delta_constraints": [
            asdict(constraint) for constraint in config.pair_delta_constraints
        ],
    }


def optimization_config_from_dict(data: object) -> SuspensionOptimizationConfig:
    """Build optimization settings from project-file JSON data."""
    if not isinstance(data, dict):
        return SuspensionOptimizationConfig()
    targets_data = data.get("targets", [])
    targets = [
        SuspensionOptimizationTarget(
            metric_name=str(item.get("metric_name", "camber_deg")),
            target_delta=float(item.get("target_delta", 0.0)),
            trend=str(item.get("trend", "ignore")),
            target_mode=str(item.get("target_mode", "endpoint_delta")),
            enabled=bool(item.get("enabled", True)),
        )
        for item in targets_data
        if isinstance(item, dict)
    ]
    if not targets:
        targets = SuspensionOptimizationConfig().targets
    pair_delta_constraints_data = data.get("pair_delta_constraints", [])
    pair_delta_constraints = [
        SuspensionOptimizationPairDeltaConstraint(
            point_a=str(item.get("point_a", "")),
            point_b=str(item.get("point_b", "")),
            label=str(item.get("label", "")),
            enabled=bool(item.get("enabled", False)),
            axes=tuple(str(axis) for axis in item.get("axes", ("x", "y", "z"))),
        )
        for item in pair_delta_constraints_data
        if isinstance(item, dict)
    ]
    if not pair_delta_constraints:
        pair_delta_constraints = SuspensionOptimizationConfig().pair_delta_constraints
    return SuspensionOptimizationConfig(
        variable_delta_limit=float(data.get("variable_delta_limit", 5.0)),
        variable_names=[str(name) for name in data.get("variable_names", [])]
        or list(DEFAULT_SUSPENSION_OPTIMIZATION_VARIABLES),
        targets=targets,
        pair_delta_constraints=pair_delta_constraints,
    )


def metric_series_from_rows(
    rows: Sequence[Mapping[str, float | bool | None]],
    metric_name: str,
) -> np.ndarray:
    """Extract one numeric metric series from solved sweep rows."""
    if not rows:
        raise ValueError("Suspension optimization requires at least one sweep row")
    key = metric_name
    first_row = rows[0]
    if key not in first_row:
        if metric_name == "toe_deg" and "roadwheel_angle_deg" in first_row:
            key = "roadwheel_angle_deg"
        else:
            raise ValueError(f"Unknown suspension optimization metric {metric_name!r}")
    values: list[float] = []
    for row in rows:
        value = row.get(key)
        if not isinstance(value, int | float):
            raise ValueError(f"Metric {metric_name!r} is not numeric in sweep rows")
        values.append(float(value))
    return np.asarray(values, dtype=np.float64)


def suspension_metric_target_value(series: np.ndarray, target_mode: str) -> float:
    """Compute the optimization target value for one solved metric series."""
    if target_mode == "endpoint_delta":
        return float(series[-1] - series[0])
    if target_mode == "value_range":
        return float(np.max(series) - np.min(series))
    if target_mode == "absolute_value":
        raise ValueError("absolute_value requires a target-aware summary helper")
    raise ValueError(f"Unsupported optimization target mode: {target_mode!r}")


def suspension_metric_primary_residuals(
    series: np.ndarray,
    target: SuspensionOptimizationTarget,
) -> np.ndarray:
    """Compute the main residual contribution for one optimization target."""
    if target.target_mode == "absolute_value":
        return (np.asarray(series, dtype=np.float64) - target.target_delta).astype(
            np.float64
        )
    return np.asarray(
        [
            suspension_metric_target_value(series, target.target_mode)
            - target.target_delta
        ],
        dtype=np.float64,
    )


def suspension_metric_summary_value(
    series: np.ndarray,
    target: SuspensionOptimizationTarget,
) -> float:
    """Compute a scalar before/after summary value for one target."""
    if target.target_mode == "absolute_value":
        return float(
            np.sqrt(
                np.mean(
                    np.square(
                        np.asarray(series, dtype=np.float64) - float(target.target_delta)
                    )
                )
            )
        )
    return suspension_metric_target_value(series, target.target_mode)


def suspension_optimization_residuals(
    rows: Sequence[Mapping[str, float | bool | None]],
    targets: Sequence[SuspensionOptimizationTarget],
    *,
    values: np.ndarray | None = None,
    baseline_values: np.ndarray | None = None,
    regularization_weight: float = 0.0,
) -> np.ndarray:
    """Build one residual vector for metric-value and trend matching."""
    residuals: list[float] = []
    active_targets = [target for target in targets if target.enabled]
    if not active_targets:
        raise ValueError("At least one optimization target must be enabled")

    for target in active_targets:
        series = metric_series_from_rows(rows, target.metric_name)
        residuals.extend(suspension_metric_primary_residuals(series, target).tolist())
        residuals.extend(_trend_residuals(series, target.trend))

    if (
        regularization_weight > 0.0
        and values is not None
        and baseline_values is not None
        and values.size > 0
    ):
        residuals.extend(
            ((np.asarray(values) - np.asarray(baseline_values)) * regularization_weight)
            .astype(np.float64)
            .tolist()
        )

    return np.asarray(residuals, dtype=np.float64)


def suspension_pair_delta_constraint_residuals(
    hardpoints: Mapping[PointID, np.ndarray],
    baseline_hardpoints: Mapping[PointID, np.ndarray],
    constraints: Sequence[SuspensionOptimizationPairDeltaConstraint],
) -> np.ndarray:
    """Build residuals that keep paired hardpoint deltas unchanged."""
    residuals: list[float] = []
    active_constraints = [constraint for constraint in constraints if constraint.enabled]
    if not active_constraints:
        return np.asarray(residuals, dtype=np.float64)

    axis_map = {"x": Axis.X, "y": Axis.Y, "z": Axis.Z}
    for constraint in active_constraints:
        point_a = _point_id_from_name(constraint.point_a)
        point_b = _point_id_from_name(constraint.point_b)
        current_a = np.asarray(hardpoints[point_a], dtype=np.float64)
        current_b = np.asarray(hardpoints[point_b], dtype=np.float64)
        baseline_a = np.asarray(baseline_hardpoints[point_a], dtype=np.float64)
        baseline_b = np.asarray(baseline_hardpoints[point_b], dtype=np.float64)
        for axis_name in constraint.axes:
            axis = axis_map[axis_name]
            residuals.append(
                float(
                    (current_b[axis] - current_a[axis])
                    - (baseline_b[axis] - baseline_a[axis])
                )
            )
    return np.asarray(residuals, dtype=np.float64)


ProgressCallback = Callable[[SuspensionOptimizationProgress], None]


def _trend_residuals(series: np.ndarray, trend: str) -> list[float]:
    diffs = np.diff(series)
    if trend == "ignore":
        return []
    if trend == "positive":
        return np.maximum(-diffs, 0.0).astype(np.float64).tolist()
    if trend == "negative":
        return np.maximum(diffs, 0.0).astype(np.float64).tolist()
    if trend == "flat":
        return diffs.astype(np.float64).tolist()
    raise ValueError(f"Unsupported optimization trend: {trend!r}")


def _variable_parts(variable_name: str) -> tuple[PointID, Axis]:
    try:
        point_name, axis_name = variable_name.rsplit("_", 1)
    except ValueError as exc:
        raise ValueError(f"Invalid optimization variable {variable_name!r}") from exc
    try:
        point_id = PointID[point_name]
    except KeyError as exc:
        raise ValueError(f"Unknown hardpoint in optimization variable {variable_name!r}") from exc
    axis_map = {"x": Axis.X, "y": Axis.Y, "z": Axis.Z}
    try:
        axis = axis_map[axis_name.lower()]
    except KeyError as exc:
        raise ValueError(f"Unknown axis in optimization variable {variable_name!r}") from exc
    return point_id, axis


def _point_id_from_name(point_name: str) -> PointID:
    try:
        return PointID[point_name]
    except KeyError as exc:
        raise ValueError(f"Unknown hardpoint in optimization constraint {point_name!r}") from exc
