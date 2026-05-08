"""
Matplotlib drawing helpers for the steering workbench GUI.
"""

from __future__ import annotations

import numpy as np
from matplotlib.axes import Axes
from matplotlib.patches import Polygon

from kinematics.steering.geometry import (
    TwoSegmentSteeringHardpoints3D,
    TwoSegmentSteeringSolution,
    Vec2,
)

WHEEL_LENGTH = 360.0
WHEEL_WIDTH = 120.0
FIT_MARGIN_RATIO = 0.08
PREVIEW_GEOMETRY_COLORS = {
    "wheel": "#1f77b4",
    "pitman": "#9467bd",
    "tie_rod": "#ff7f0e",
    "knuckle_arm": "#2ca02c",
    "wheel_radius": "#7f7f7f",
    "kingpin": "#111111",
}


def _rotated_rect(center: Vec2, angle_deg: float) -> np.ndarray:
    half_l = WHEEL_LENGTH / 2.0
    half_w = WHEEL_WIDTH / 2.0
    corners = np.array(
        [
            [-half_l, -half_w],
            [half_l, -half_w],
            [half_l, half_w],
            [-half_l, half_w],
        ],
        dtype=np.float64,
    )
    angle = np.deg2rad(angle_deg)
    rot = np.array(
        [
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)],
        ],
        dtype=np.float64,
    )
    return center + corners @ rot.T


def _draw_wheel(ax: Axes, center: Vec2, angle_deg: float, alpha: float) -> None:
    color = PREVIEW_GEOMETRY_COLORS["wheel"]
    ax.add_patch(
        Polygon(
            _rotated_rect(center, angle_deg),
            closed=True,
            facecolor=color,
            edgecolor=color,
            alpha=alpha * 0.22,
            linewidth=2.0,
        )
    )
    ax.scatter(center[0], center[1], color=color, alpha=alpha, s=20, zorder=5)


def _draw_pitman(
    ax: Axes,
    pivot: Vec2,
    left_output: Vec2,
    right_output: Vec2,
    alpha: float,
) -> None:
    color = PREVIEW_GEOMETRY_COLORS["pitman"]
    points = np.vstack([pivot, left_output, right_output])
    ax.add_patch(
        Polygon(
            points,
            closed=True,
            facecolor=color,
            edgecolor=color,
            alpha=alpha * 0.22,
            linewidth=2.0,
        )
    )
    ax.plot(
        points[[0, 1, 2, 0], 0],
        points[[0, 1, 2, 0], 1],
        color=color,
        alpha=alpha,
    )
    ax.scatter(pivot[0], pivot[1], color=color, alpha=alpha, s=28, zorder=5)


def _draw_segment(ax: Axes, start: Vec2, end: Vec2, color: str, alpha: float) -> None:
    ax.plot(
        [start[0], end[0]],
        [start[1], end[1]],
        color=color,
        alpha=alpha,
        linewidth=2.0,
    )


def _draw_state(
    ax: Axes,
    hardpoints: TwoSegmentSteeringHardpoints3D,
    state: TwoSegmentSteeringSolution,
    alpha: float,
) -> None:
    geometry = hardpoints.to_2d_geometry()
    _draw_wheel(ax, state.left_wheel_center, state.left_wheel_angle_deg, alpha)
    _draw_wheel(ax, state.right_wheel_center, state.right_wheel_angle_deg, alpha)
    _draw_pitman(
        ax,
        geometry.pitman.pivot,
        state.pitman_left_output,
        state.pitman_right_output,
        alpha,
    )
    _draw_segment(
        ax,
        state.pitman_left_output,
        state.left_tie_rod_pickup,
        PREVIEW_GEOMETRY_COLORS["tie_rod"],
        alpha,
    )
    _draw_segment(
        ax,
        state.pitman_right_output,
        state.right_tie_rod_pickup,
        PREVIEW_GEOMETRY_COLORS["tie_rod"],
        alpha,
    )
    _draw_segment(
        ax,
        geometry.left_wheel.kingpin,
        state.left_tie_rod_pickup,
        PREVIEW_GEOMETRY_COLORS["knuckle_arm"],
        alpha,
    )
    _draw_segment(
        ax,
        geometry.right_wheel.kingpin,
        state.right_tie_rod_pickup,
        PREVIEW_GEOMETRY_COLORS["knuckle_arm"],
        alpha,
    )
    _draw_segment(
        ax,
        geometry.left_wheel.kingpin,
        state.left_wheel_center,
        PREVIEW_GEOMETRY_COLORS["wheel_radius"],
        alpha,
    )
    _draw_segment(
        ax,
        geometry.right_wheel.kingpin,
        state.right_wheel_center,
        PREVIEW_GEOMETRY_COLORS["wheel_radius"],
        alpha,
    )
    for wheel in (geometry.left_wheel, geometry.right_wheel):
        ax.scatter(
            wheel.kingpin[0],
            wheel.kingpin[1],
            marker="x",
            color=PREVIEW_GEOMETRY_COLORS["kingpin"],
            alpha=alpha,
            s=42,
            zorder=6,
        )


def draw_steering_preview(
    ax: Axes,
    hardpoints: TwoSegmentSteeringHardpoints3D,
    design_state: TwoSegmentSteeringSolution,
    current_state: TwoSegmentSteeringSolution,
    *,
    preserve_view: bool = False,
) -> None:
    """Draw the design and current top-view steering geometry."""
    previous_xlim = ax.get_xlim()
    previous_ylim = ax.get_ylim()
    had_data = ax.has_data()
    ax.clear()
    _draw_state(ax, hardpoints, design_state, alpha=0.28)
    _draw_state(ax, hardpoints, current_state, alpha=1.0)
    ax.figure.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
    ax.set_position([0.0, 0.0, 1.0, 1.0])
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_axis_off()
    ax.format_coord = lambda _x, _y: ""
    fit_steering_preview(ax)
    if preserve_view and had_data:
        ax.set_xlim(previous_xlim)
        ax.set_ylim(previous_ylim)


def fit_steering_preview(ax: Axes, margin_ratio: float = FIT_MARGIN_RATIO) -> None:
    """Fit the preview axes to currently drawn steering geometry."""
    data_x0, data_y0, data_width, data_height = ax.dataLim.bounds
    if not np.all(np.isfinite([data_x0, data_y0, data_width, data_height])):
        return
    if data_width <= 0.0 or data_height <= 0.0:
        return
    x_margin = max(data_width * margin_ratio, 1.0)
    y_margin = max(data_height * margin_ratio, 1.0)
    ax.set_xlim(data_x0 - x_margin, data_x0 + data_width + x_margin)
    ax.set_ylim(data_y0 - y_margin, data_y0 + data_height + y_margin)


def draw_curve_plot(
    ax: Axes,
    rows: list[dict[str, float]],
    curves: list[tuple[str, str, str]],
) -> None:
    """Draw managed output curves."""
    ax.clear()
    for x_output, y_output, label in curves:
        x_values = [row[x_output] for row in rows]
        y_values = [row[y_output] for row in rows]
        curve_label = label or f"{y_output} vs {x_output}"
        ax.plot(x_values, y_values, marker="o", label=curve_label)
    ax.set_title("Output Curves")
    ax.set_xlabel(curves[0][0] if curves else "x")
    ax.set_ylabel("selected output")
    ax.grid(True, alpha=0.25)
    if curves:
        ax.legend(loc="best")
