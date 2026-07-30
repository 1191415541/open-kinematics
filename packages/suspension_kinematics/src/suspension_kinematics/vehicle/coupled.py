"""Weakly coupled suspension and two-segment steering sweeps."""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from typing import Literal, Sequence

import numpy as np

from suspension_kinematics.constraints import Constraint, PointOnLineConstraint
from suspension_kinematics.core.enums import Axis, PointID, TargetPositionMode
from suspension_kinematics.core.types import PointTarget, PointTargetAxis, SweepConfig
from suspension_kinematics.main import SolverInfo
from suspension_kinematics.metrics import compute_metrics_for_state
from suspension_kinematics.points.derived.manager import DerivedPointsManager
from suspension_kinematics.solver import solve_suspension_sweep
from suspension_kinematics.state import SuspensionState
from suspension_kinematics.steering import (
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
    solve_two_segment_steering,
)
from suspension_kinematics.suspensions.base import Suspension
from suspension_kinematics.suspensions.config.settings import SuspensionConfig

CornerSide = Literal["left", "right"]


@dataclass(frozen=True)
class SymmetricCornerPair:
    """Left and right suspension corners derived from one side."""

    left: Suspension
    right: Suspension
    source_side: CornerSide


@dataclass(frozen=True)
class CoupledSweepResult:
    """One weakly coupled vehicle sweep result row."""

    step_index: int
    wheel_travel: float
    pitman_angle_deg: float
    steering: TwoSegmentSteeringSolution
    left_state: SuspensionState
    right_state: SuspensionState
    left_solver_info: SolverInfo
    right_solver_info: SolverInfo
    metrics: dict[str, float | None]

    @property
    def solver_info(self) -> SolverInfo:
        """Combined convergence diagnostics for both suspension corners."""
        return SolverInfo(
            converged=self.left_solver_info.converged
            and self.right_solver_info.converged
            and self.steering.converged,
            nfev=self.left_solver_info.nfev + self.right_solver_info.nfev,
            max_residual=max(
                self.left_solver_info.max_residual,
                self.right_solver_info.max_residual,
                self.steering.max_abs_tie_rod_residual,
            ),
        )


def mirror_suspension_y(suspension: Suspension) -> Suspension:
    """Create the opposite-side suspension by mirroring all Y coordinates."""
    hardpoints = {
        pid: np.array([pos[0], -pos[1], pos[2]], dtype=np.float64)
        for pid, pos in suspension.hardpoints.items()
    }
    config = _mirror_config_y(suspension.config)
    return type(suspension)(
        name=f"{suspension.name}_mirrored_y",
        version=suspension.version,
        units=suspension.units,
        hardpoints=hardpoints,
        config=config,
    )


def _mirror_vec_y(value) -> np.ndarray:
    arr = np.asarray(value, dtype=np.float64)
    return np.array([arr[0], -arr[1], arr[2]], dtype=np.float64)


def _mirror_config_y(config: SuspensionConfig | None) -> SuspensionConfig | None:
    if config is None:
        return None
    if config.camber_shim is None:
        return config.model_copy(deep=True)

    shim = config.camber_shim
    mirrored_shim = shim.model_copy(
        update={
            "shim_face_point_a": _mirror_vec_y(shim.shim_face_point_a),
            "shim_face_point_b": _mirror_vec_y(shim.shim_face_point_b),
            "shim_face_normal": _mirror_vec_y(shim.shim_face_normal),
        },
        deep=True,
    )
    return config.model_copy(update={"camber_shim": mirrored_shim}, deep=True)


def _infer_source_side(suspension: Suspension) -> CornerSide:
    y = float(suspension.hardpoints[PointID.LOWER_WISHBONE_OUTBOARD][Axis.Y])
    return "right" if y >= 0.0 else "left"


def build_symmetric_corner_pair(source: Suspension) -> SymmetricCornerPair:
    """Build left/right corners from one symmetric suspension input."""
    source_side = _infer_source_side(source)
    mirrored = mirror_suspension_y(source)
    if source_side == "right":
        return SymmetricCornerPair(left=mirrored, right=source, source_side=source_side)
    return SymmetricCornerPair(left=source, right=mirrored, source_side=source_side)


def _corner_sweep_config(
    *,
    wheel_travel: float,
    trackrod_inboard: np.ndarray,
) -> SweepConfig:
    return SweepConfig(
        [
            [
                PointTarget(
                    point_id=PointID.WHEEL_CENTER,
                    direction=PointTargetAxis(Axis.Z),
                    value=float(wheel_travel),
                    mode=TargetPositionMode.RELATIVE,
                )
            ],
            [
                PointTarget(
                    point_id=PointID.TRACKROD_INBOARD,
                    direction=PointTargetAxis(Axis.X),
                    value=float(trackrod_inboard[0]),
                    mode=TargetPositionMode.ABSOLUTE,
                )
            ],
            [
                PointTarget(
                    point_id=PointID.TRACKROD_INBOARD,
                    direction=PointTargetAxis(Axis.Y),
                    value=float(trackrod_inboard[1]),
                    mode=TargetPositionMode.ABSOLUTE,
                )
            ],
            [
                PointTarget(
                    point_id=PointID.TRACKROD_INBOARD,
                    direction=PointTargetAxis(Axis.Z),
                    value=float(trackrod_inboard[2]),
                    mode=TargetPositionMode.ABSOLUTE,
                )
            ],
        ]
    )


def _solve_corner(
    suspension: Suspension,
    *,
    wheel_travel: float,
    trackrod_inboard: np.ndarray,
) -> tuple[SuspensionState, SolverInfo]:
    derived_manager = DerivedPointsManager(suspension.derived_spec())
    states, solver_infos = solve_suspension_sweep(
        initial_state=suspension.initial_state(),
        constraints=_constraints_without_rack_line(suspension.constraints()),
        sweep_config=_corner_sweep_config(
            wheel_travel=wheel_travel,
            trackrod_inboard=trackrod_inboard,
        ),
        derived_manager=derived_manager,
    )
    return states[0], solver_infos[0]


def _constraints_without_rack_line(constraints: list[Constraint]) -> list[Constraint]:
    return [
        constraint
        for constraint in constraints
        if not (
            isinstance(constraint, PointOnLineConstraint)
            and constraint.point_id == PointID.TRACKROD_INBOARD
        )
    ]


def _prefixed_metrics(
    prefix: str,
    state: SuspensionState,
    suspension: Suspension,
) -> dict[str, float | None]:
    if suspension.config is None:
        return {}
    return {
        f"{prefix}_{name}": value
        for name, value in compute_metrics_for_state(
            state,
            suspension,
            suspension.config,
        ).items()
    }


def _steering_metrics(
    solution: TwoSegmentSteeringSolution,
) -> dict[str, float | None]:
    return {
        "steering_pitman_angle_deg": solution.pitman_angle_deg,
        "steering_left_wheel_angle_deg": solution.left_wheel_angle_deg,
        "steering_right_wheel_angle_deg": solution.right_wheel_angle_deg,
        "steering_left_minus_right_deg": (
            solution.left_wheel_angle_deg - solution.right_wheel_angle_deg
        ),
        "steering_left_trackrod_inboard_y": float(solution.pitman_left_output[1]),
        "steering_right_trackrod_inboard_y": float(solution.pitman_right_output[1]),
        "steering_max_tie_rod_residual": solution.max_abs_tie_rod_residual,
    }


def _pitman_output_target(
    output_2d: np.ndarray,
    source_suspension: Suspension,
) -> np.ndarray:
    rack_z = float(
        source_suspension.initial_state().positions[PointID.TRACKROD_INBOARD][2]
    )
    return np.array([output_2d[0], output_2d[1], rack_z], dtype=np.float64)


def solve_coupled_sweep(
    *,
    source_suspension: Suspension,
    steering_geometry: TwoSegmentSteeringHardpoints3D,
    wheel_travel_values: Sequence[float],
    pitman_angle_values: Sequence[float],
) -> list[CoupledSweepResult]:
    """
    Run weakly coupled left/right suspension and two-segment steering sweeps.

    The steering solve maps pitman arm angle to left/right pitman output
    positions. Those positions drive each corner's TRACKROD_INBOARD target while
    WHEEL_CENTER Z drives suspension travel.
    """
    corners = build_symmetric_corner_pair(source_suspension)
    results: list[CoupledSweepResult] = []

    for step_index, (wheel_travel, pitman_angle) in enumerate(
        product(wheel_travel_values, pitman_angle_values)
    ):
        steering = solve_two_segment_steering(
            steering_geometry,
            pitman_angle_deg=float(pitman_angle),
        )
        left_state, left_solver = _solve_corner(
            corners.left,
            wheel_travel=float(wheel_travel),
            trackrod_inboard=_pitman_output_target(
                steering.pitman_left_output,
                source_suspension,
            ),
        )
        right_state, right_solver = _solve_corner(
            corners.right,
            wheel_travel=float(wheel_travel),
            trackrod_inboard=_pitman_output_target(
                steering.pitman_right_output,
                source_suspension,
            ),
        )

        metrics = {
            "wheel_travel_mm": float(wheel_travel),
            **_steering_metrics(steering),
            **_prefixed_metrics("left", left_state, corners.left),
            **_prefixed_metrics("right", right_state, corners.right),
        }
        results.append(
            CoupledSweepResult(
                step_index=step_index,
                wheel_travel=float(wheel_travel),
                pitman_angle_deg=float(pitman_angle),
                steering=steering,
                left_state=left_state,
                right_state=right_state,
                left_solver_info=left_solver,
                right_solver_info=right_solver,
                metrics=metrics,
            )
        )

    return results
