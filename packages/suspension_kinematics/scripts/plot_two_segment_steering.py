"""Run and plot a practical two-segment steering case."""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes
from matplotlib.patches import Polygon

from suspension_kinematics.steering import (
    PitmanArmGeometry2D,
    SteeringCoordinateSystem,
    TwoSegmentSteeringGeometry,
    TwoSegmentSteeringSolution,
    WheelSteeringGeometry2D,
    solve_two_segment_steering,
    sweep_two_segment_steering,
)

OUTPUT_DIR = Path("scripts/plots")
OUTPUT_IMAGE = OUTPUT_DIR / "two_segment_steering_case.png"
OUTPUT_CSV = OUTPUT_DIR / "two_segment_steering_case.csv"
OUTPUT_ANIMATION = OUTPUT_DIR / "two_segment_steering_case.gif"
PITMAN_SWEEP_DEG = [*range(-20, 0, 2), 0, *range(2, 22, 2)]
DISPLAY_PITMAN_DEG = 20.0


def _build_geometry() -> TwoSegmentSteeringGeometry:
    return TwoSegmentSteeringGeometry(
        left_wheel=WheelSteeringGeometry2D(
            kingpin=np.array([0.0, -900.0]),
            wheel_center=np.array([60.0, -930.0]),
            tie_rod_pickup=np.array([-350.0, -850.0]),
        ),
        right_wheel=WheelSteeringGeometry2D(
            kingpin=np.array([0.0, 900.0]),
            wheel_center=np.array([60.0, 930.0]),
            tie_rod_pickup=np.array([-350.0, 850.0]),
        ),
        pitman=PitmanArmGeometry2D(
            pivot=np.array([-1000.0, 0.0]),
            left_output=np.array([-350.0, -180.0]),
            right_output=np.array([-350.0, 180.0]),
        ),
    )


def _write_csv(solutions: list[TwoSegmentSteeringSolution]) -> None:
    with open(OUTPUT_CSV, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "pitman_angle_deg",
                "left_wheel_angle_deg",
                "right_wheel_angle_deg",
                "left_minus_right_deg",
            ]
        )
        for state in solutions:
            writer.writerow(
                [
                    f"{state.pitman_angle_deg:.3f}",
                    f"{state.left_wheel_angle_deg:.6f}",
                    f"{state.right_wheel_angle_deg:.6f}",
                    f"{state.left_wheel_angle_deg - state.right_wheel_angle_deg:.6f}",
                ]
            )


def _wheel_polygon(center: np.ndarray, angle_deg: float) -> np.ndarray:
    angle = np.deg2rad(angle_deg)
    forward = np.array([np.cos(angle), np.sin(angle)])
    lateral = np.array([-np.sin(angle), np.cos(angle)])
    half_length = 330.0
    half_width = 105.0
    return np.array(
        [
            center + half_length * forward + half_width * lateral,
            center + half_length * forward - half_width * lateral,
            center - half_length * forward - half_width * lateral,
            center - half_length * forward + half_width * lateral,
        ]
    )


def _draw_wheel(ax: Axes, center: np.ndarray, angle_deg: float, color: str) -> None:
    wheel = Polygon(
        _wheel_polygon(center, angle_deg),
        closed=True,
        facecolor=color,
        edgecolor=color,
        alpha=0.18,
        linewidth=1.8,
    )
    ax.add_patch(wheel)
    ax.scatter(center[0], center[1], color=color, s=28, zorder=4)


def _outer_wheel_angle_abs(state: TwoSegmentSteeringSolution) -> float:
    if state.left_wheel_angle_deg * state.right_wheel_angle_deg <= 0.0:
        return 0.0
    return min(abs(state.left_wheel_angle_deg), abs(state.right_wheel_angle_deg))


def _draw_pitman_triangle(
    ax: Axes,
    pivot: np.ndarray,
    left_output: np.ndarray,
    right_output: np.ndarray,
    color: str,
    label: str,
) -> None:
    triangle = Polygon(
        np.array([pivot, left_output, right_output]),
        closed=True,
        facecolor=color,
        edgecolor=color,
        alpha=0.16,
        linewidth=2.0,
        label=label,
    )
    ax.add_patch(triangle)
    ax.plot(
        [pivot[0], left_output[0], right_output[0], pivot[0]],
        [pivot[1], left_output[1], right_output[1], pivot[1]],
        color=color,
        linewidth=2.0,
    )


def _plot_geometry(
    ax: Axes,
    geometry: TwoSegmentSteeringGeometry,
    state: TwoSegmentSteeringSolution,
) -> None:
    design = solve_two_segment_steering(geometry, 0.0)
    _plot_state(ax, geometry, design, "#7f7f7f", "design")
    _plot_state(
        ax,
        geometry,
        state,
        "#1f77b4",
        f"pitman {state.pitman_angle_deg:.0f} deg",
    )
    ax.set_title("Two-segment steering geometry [top view]")
    ax.set_xlabel(SteeringCoordinateSystem.TOP_VIEW_X_LABEL)
    ax.set_ylabel(SteeringCoordinateSystem.TOP_VIEW_Y_LABEL)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.legend(loc="upper left")


def _configure_steering_axis(ax: Axes) -> None:
    ax.set_xlim(-1220, 420)
    ax.set_ylim(-1160, 1160)
    ax.set_xlabel(SteeringCoordinateSystem.TOP_VIEW_X_LABEL)
    ax.set_ylabel(SteeringCoordinateSystem.TOP_VIEW_Y_LABEL)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)


def _plot_state(
    ax: Axes,
    geometry: TwoSegmentSteeringGeometry,
    state: TwoSegmentSteeringSolution,
    color: str,
    label: str,
) -> None:
    pitman = geometry.pitman
    _draw_wheel(ax, state.left_wheel_center, state.left_wheel_angle_deg, color)
    _draw_wheel(ax, state.right_wheel_center, state.right_wheel_angle_deg, color)
    radii = [
        (geometry.left_wheel.kingpin, state.left_wheel_center, f"{label} wheel radius"),
        (geometry.right_wheel.kingpin, state.right_wheel_center, "_nolegend_"),
    ]
    for kingpin, center, legend in radii:
        ax.plot(
            [kingpin[0], center[0]],
            [kingpin[1], center[1]],
            ":",
            color=color,
            linewidth=1.8,
            label=legend,
        )
        ax.scatter(kingpin[0], kingpin[1], color=color, marker="x", s=54, zorder=6)
    _draw_pitman_triangle(
        ax,
        pitman.pivot,
        state.pitman_left_output,
        state.pitman_right_output,
        color,
        f"{label} triangular pitman",
    )
    tie_rods = [
        (state.pitman_left_output, state.left_tie_rod_pickup, f"{label} tie rods"),
        (state.pitman_right_output, state.right_tie_rod_pickup, "_nolegend_"),
    ]
    for start, end, legend in tie_rods:
        ax.plot(
            [start[0], end[0]],
            [start[1], end[1]],
            "--",
            color=color,
            linewidth=2.2,
            label=legend,
        )
    points = np.array(
        [
            pitman.pivot,
            state.pitman_left_output,
            state.pitman_right_output,
            state.left_wheel_center,
            state.right_wheel_center,
            state.left_tie_rod_pickup,
            state.right_tie_rod_pickup,
        ]
    )
    ax.scatter(points[:, 0], points[:, 1], color=color, s=24, zorder=5)


def _plot_angle_relationship(
    ax: Axes,
    solutions: list[TwoSegmentSteeringSolution],
) -> None:
    pitman = [s.pitman_angle_deg for s in solutions]
    left = [s.left_wheel_angle_deg for s in solutions]
    right = [s.right_wheel_angle_deg for s in solutions]
    ax.plot(pitman, left, "o-", label="left wheel")
    ax.plot(pitman, right, "s-", label="right wheel")
    ax.axhline(0, color="black", linewidth=0.8)
    ax.axvline(0, color="black", linewidth=0.8)
    ax.set_title("Roadwheel angle relationship")
    ax.set_xlabel("Pitman arm angle [deg]")
    ax.set_ylabel("Roadwheel angle [deg]")
    ax.grid(True, alpha=0.3)
    ax.legend()


def _save_animation(
    geometry: TwoSegmentSteeringGeometry,
    solutions: list[TwoSegmentSteeringSolution],
) -> None:
    frames = solutions + solutions[-2:0:-1]
    fig, ax = plt.subplots(figsize=(8, 7))
    design = solve_two_segment_steering(geometry, 0.0)

    def update(frame_index: int) -> list:
        state = frames[frame_index]
        ax.clear()
        _plot_state(ax, geometry, design, "#9a9a9a", "design")
        _plot_state(ax, geometry, state, "#1f77b4", "current")
        _configure_steering_axis(ax)
        ax.set_title(
            "Two-segment steering animation\n"
            f"pitman {state.pitman_angle_deg:.1f} deg | "
            f"L {state.left_wheel_angle_deg:.1f} deg | "
            f"R {state.right_wheel_angle_deg:.1f} deg | "
            f"outer {_outer_wheel_angle_abs(state):.1f} deg"
        )
        ax.legend(loc="upper left")
        return []

    anim = animation.FuncAnimation(fig, update, frames=len(frames), interval=80)
    anim.save(OUTPUT_ANIMATION, writer=animation.PillowWriter(fps=12), dpi=140)
    plt.close(fig)


def _main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    geometry = _build_geometry()
    solutions = sweep_two_segment_steering(geometry, PITMAN_SWEEP_DEG)
    display_state = solve_two_segment_steering(geometry, DISPLAY_PITMAN_DEG)

    _write_csv(solutions)
    _save_animation(geometry, solutions)
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    _plot_geometry(axes[0], geometry, display_state)
    _plot_angle_relationship(axes[1], solutions)
    fig.tight_layout()
    fig.savefig(OUTPUT_IMAGE, dpi=180)
    plt.close(fig)

    print(f"Saved {OUTPUT_IMAGE}")
    print(f"Saved {OUTPUT_CSV}")
    print(f"Saved {OUTPUT_ANIMATION}")
    for state in solutions:
        print(
            f"{state.pitman_angle_deg:>6.1f} deg pitman -> "
            f"L {state.left_wheel_angle_deg:>8.3f} deg, "
            f"R {state.right_wheel_angle_deg:>8.3f} deg, "
            f"outer {_outer_wheel_angle_abs(state):>8.3f} deg"
        )


if __name__ == "__main__":
    _main()
