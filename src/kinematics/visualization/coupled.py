"""Vehicle-level animation for weakly coupled suspension and steering results."""

from __future__ import annotations

from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np

from kinematics.core.enums import PointID
from kinematics.core.types import Vec3
from kinematics.steering import TwoSegmentSteeringHardpoints3D
from kinematics.suspensions.base import Suspension
from kinematics.vehicle import CoupledSweepResult, build_symmetric_corner_pair
from kinematics.visualization.main import SuspensionVisualizer, WheelVisualization
from kinematics.visualization.plots import compute_bounds_from_states, configure_3d_axis


def _steering_point_z(
    source_suspension: Suspension,
    point_id: PointID = PointID.TRACKROD_INBOARD,
) -> float:
    return float(source_suspension.initial_state().positions[point_id][2])


def _steering_points_3d(
    steering_geometry: TwoSegmentSteeringHardpoints3D,
    result: CoupledSweepResult,
    source_suspension: Suspension,
) -> dict[str, Vec3]:
    z = _steering_point_z(source_suspension)
    return {
        "pitman_pivot": np.array(
            [
                steering_geometry.pitman.pivot[0],
                steering_geometry.pitman.pivot[1],
                z,
            ],
            dtype=np.float64,
        ),
        "left_pitman_output": np.array(
            [
                result.steering.pitman_left_output[0],
                result.steering.pitman_left_output[1],
                z,
            ],
            dtype=np.float64,
        ),
        "right_pitman_output": np.array(
            [
                result.steering.pitman_right_output[0],
                result.steering.pitman_right_output[1],
                z,
            ],
            dtype=np.float64,
        ),
        "left_tie_rod_pickup": result.left_state.positions[PointID.TRACKROD_OUTBOARD],
        "right_tie_rod_pickup": result.right_state.positions[PointID.TRACKROD_OUTBOARD],
    }


def _combined_position_states(
    results: list[CoupledSweepResult],
    steering_geometry: TwoSegmentSteeringHardpoints3D,
    source_suspension: Suspension,
) -> list[dict[PointID | str, Vec3]]:
    states: list[dict[PointID | str, Vec3]] = []
    for result in results:
        states.append(
            {
                **{f"left_{pid.name}": pos for pid, pos in result.left_state.items()},
                **{f"right_{pid.name}": pos for pid, pos in result.right_state.items()},
                **_steering_points_3d(steering_geometry, result, source_suspension),
            }
        )
    return states


def _draw_steering(ax, points: dict[str, Vec3]) -> dict[str, object]:
    artists: dict[str, object] = {}
    (pitman,) = ax.plot(
        [points["pitman_pivot"][0], points["left_pitman_output"][0]],
        [points["pitman_pivot"][1], points["left_pitman_output"][1]],
        [points["pitman_pivot"][2], points["left_pitman_output"][2]],
        color="#b45309",
        linewidth=3,
        marker="o",
        label="pitman",
    )
    (pitman_right,) = ax.plot(
        [points["pitman_pivot"][0], points["right_pitman_output"][0]],
        [points["pitman_pivot"][1], points["right_pitman_output"][1]],
        [points["pitman_pivot"][2], points["right_pitman_output"][2]],
        color="#b45309",
        linewidth=3,
        marker="o",
    )
    (left_tie,) = ax.plot(
        [points["left_pitman_output"][0], points["left_tie_rod_pickup"][0]],
        [points["left_pitman_output"][1], points["left_tie_rod_pickup"][1]],
        [points["left_pitman_output"][2], points["left_tie_rod_pickup"][2]],
        color="#0f766e",
        linewidth=3,
        marker="o",
        label="left tie rod",
    )
    (right_tie,) = ax.plot(
        [points["right_pitman_output"][0], points["right_tie_rod_pickup"][0]],
        [points["right_pitman_output"][1], points["right_tie_rod_pickup"][1]],
        [points["right_pitman_output"][2], points["right_tie_rod_pickup"][2]],
        color="#2563eb",
        linewidth=3,
        marker="o",
        label="right tie rod",
    )
    artists["pitman_left"] = pitman
    artists["pitman_right"] = pitman_right
    artists["left_tie"] = left_tie
    artists["right_tie"] = right_tie
    return artists


def _update_line(line, a: Vec3, b: Vec3) -> None:
    line.set_data([a[0], b[0]], [a[1], b[1]])
    line.set_3d_properties([a[2], b[2]])


def _update_steering(artists: dict[str, object], points: dict[str, Vec3]) -> None:
    _update_line(
        artists["pitman_left"],
        points["pitman_pivot"],
        points["left_pitman_output"],
    )
    _update_line(
        artists["pitman_right"],
        points["pitman_pivot"],
        points["right_pitman_output"],
    )
    _update_line(
        artists["left_tie"],
        points["left_pitman_output"],
        points["left_tie_rod_pickup"],
    )
    _update_line(
        artists["right_tie"],
        points["right_pitman_output"],
        points["right_tie_rod_pickup"],
    )


def _without_track_rod_links(suspension: Suspension):
    return [
        link
        for link in suspension.get_visualization_links()
        if link.label != "Track Rod"
    ]


def create_coupled_animation(
    *,
    source_suspension: Suspension,
    steering_geometry: TwoSegmentSteeringHardpoints3D,
    results: list[CoupledSweepResult],
    output_path: Path,
    fps: int = 12,
    dpi: int = 120,
) -> None:
    """Create a GIF for weakly coupled left/right suspension and steering motion."""
    if not results:
        raise ValueError("No coupled results to animate")
    if source_suspension.config is None:
        raise ValueError("Suspension has no config")

    corners = build_symmetric_corner_pair(source_suspension)
    wheel_cfg = source_suspension.config.wheel
    wheel_visual = WheelVisualization(
        diameter=wheel_cfg.tire.nominal_radius * 2,
        width=wheel_cfg.tire.section_width,
        num_points=32,
    )
    left_links = _without_track_rod_links(corners.left)
    right_links = _without_track_rod_links(corners.right)
    left_visualizer = SuspensionVisualizer(
        left_links,
        wheel_visual,
    )
    right_visualizer = SuspensionVisualizer(
        right_links,
        wheel_visual,
    )

    combined_states = _combined_position_states(
        results,
        steering_geometry,
        source_suspension,
    )
    _, _, (x_mid, y_mid, z_mid, max_range) = compute_bounds_from_states(
        combined_states
    )

    fig = plt.figure(figsize=(9, 7))
    ax = fig.add_subplot(111, projection="3d")
    configure_3d_axis(ax, "iso", x_mid, y_mid, z_mid, max_range)

    first = results[0]
    left_link_artists = left_visualizer.draw_links(ax, first.left_state.positions)
    right_link_artists = right_visualizer.draw_links(ax, first.right_state.positions)
    left_wheel = left_visualizer.draw_wheel(
        ax,
        first.left_state.positions,
        num_bands=16,
    )
    right_wheel = right_visualizer.draw_wheel(
        ax,
        first.right_state.positions,
        num_bands=16,
    )
    steering_artists = _draw_steering(
        ax,
        _steering_points_3d(steering_geometry, first, source_suspension),
    )
    ax.legend(loc="upper left")

    title_artist = fig.suptitle("", fontsize=12)
    plt.tight_layout()

    pingpong = results + results[-2:0:-1]
    writer = animation.PillowWriter(fps=fps)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with writer.saving(fig, str(output_path), dpi):
        for result in pingpong:
            left_visualizer.update_links(
                left_link_artists,
                result.left_state.positions,
            )
            right_visualizer.update_links(
                right_link_artists,
                result.right_state.positions,
            )
            left_visualizer.update_wheel(
                left_wheel,
                result.left_state.positions,
                num_bands=16,
            )
            right_visualizer.update_wheel(
                right_wheel,
                result.right_state.positions,
                num_bands=16,
            )
            _update_steering(
                steering_artists,
                _steering_points_3d(steering_geometry, result, source_suspension),
            )
            title_artist.set_text(
                "\n".join(
                    [
                        f"Wheel travel: {result.wheel_travel:.1f} mm",
                        f"Pitman angle: {result.pitman_angle_deg:.1f} deg",
                        "Left / Right wheel angle: "
                        f"{result.steering.left_wheel_angle_deg:.2f} / "
                        f"{result.steering.right_wheel_angle_deg:.2f} deg",
                    ]
                )
            )
            writer.grab_frame()
    plt.close(fig)
