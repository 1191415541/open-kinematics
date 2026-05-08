import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")
import numpy as np  # noqa: E402
from matplotlib import pyplot as plt  # noqa: E402
from matplotlib.colors import to_hex  # noqa: E402

from kinematics.steering.gui_plotting import (  # noqa: E402
    PREVIEW_GEOMETRY_COLORS,
    draw_steering_preview,
    fit_steering_preview,
)
from kinematics.steering.two_segment import solve_two_segment_steering  # noqa: E402
from kinematics.steering.workbench import (  # noqa: E402
    default_steering_project,
    hardpoints_from_rows,
)


def _preview_inputs():
    hardpoints = hardpoints_from_rows(default_steering_project().hardpoints)
    design_state = solve_two_segment_steering(hardpoints, 0.0)
    current_state = solve_two_segment_steering(hardpoints, 8.0)
    return hardpoints, design_state, current_state


def _has_line_segment(ax, start, end):
    return _line_segment_color(ax, start, end) is not None


def _line_segment_color(ax, start, end):
    for line in ax.lines:
        points = np.column_stack([line.get_xdata(), line.get_ydata()])
        if points.shape != (2, 2):
            continue
        forward = np.allclose(points[0], start) and np.allclose(points[1], end)
        reverse = np.allclose(points[0], end) and np.allclose(points[1], start)
        if forward or reverse:
            return to_hex(line.get_color())
    return None


def test_steering_preview_can_preserve_existing_view_limits():
    fig, ax = plt.subplots()
    hardpoints, design_state, current_state = _preview_inputs()
    draw_steering_preview(ax, hardpoints, design_state, current_state)
    ax.set_xlim(-250.0, 125.0)
    ax.set_ylim(-300.0, 80.0)

    draw_steering_preview(
        ax,
        hardpoints,
        design_state,
        current_state,
        preserve_view=True,
    )

    assert ax.get_xlim() == (-250.0, 125.0)
    assert ax.get_ylim() == (-300.0, 80.0)
    plt.close(fig)


def test_fit_steering_preview_expands_view_to_drawn_geometry():
    fig, ax = plt.subplots()
    hardpoints, design_state, current_state = _preview_inputs()
    draw_steering_preview(ax, hardpoints, design_state, current_state)
    data_x0, data_y0, data_width, data_height = ax.dataLim.bounds
    ax.set_xlim(0.0, 1.0)
    ax.set_ylim(0.0, 1.0)

    fit_steering_preview(ax)

    xlim = ax.get_xlim()
    ylim = ax.get_ylim()
    assert xlim[0] < data_x0
    assert xlim[1] > data_x0 + data_width
    assert ylim[0] < data_y0
    assert ylim[1] > data_y0 + data_height
    plt.close(fig)


def test_steering_preview_hides_coordinate_system_and_fills_figure():
    fig, ax = plt.subplots()
    hardpoints, design_state, current_state = _preview_inputs()

    draw_steering_preview(ax, hardpoints, design_state, current_state)

    assert not ax.axison
    assert ax.get_title() == ""
    assert ax.get_xlabel() == ""
    assert ax.get_ylabel() == ""
    assert ax.get_legend() is None
    assert ax.format_coord(12.0, 34.0) == ""
    assert ax.get_position().bounds == (0.0, 0.0, 1.0, 1.0)
    plt.close(fig)


def test_steering_preview_draws_knuckle_arms_to_tie_rod_pickups():
    fig, ax = plt.subplots()
    hardpoints, design_state, current_state = _preview_inputs()
    geometry = hardpoints.to_2d_geometry()

    draw_steering_preview(ax, hardpoints, design_state, current_state)

    assert _has_line_segment(
        ax,
        geometry.left_wheel.kingpin,
        current_state.left_tie_rod_pickup,
    )
    assert _has_line_segment(
        ax,
        geometry.right_wheel.kingpin,
        current_state.right_tie_rod_pickup,
    )
    plt.close(fig)


def test_steering_preview_uses_distinct_colors_for_geometry_types():
    fig, ax = plt.subplots()
    hardpoints, design_state, current_state = _preview_inputs()
    geometry = hardpoints.to_2d_geometry()

    draw_steering_preview(ax, hardpoints, design_state, current_state)

    assert _line_segment_color(
        ax,
        current_state.pitman_left_output,
        current_state.left_tie_rod_pickup,
    ) == PREVIEW_GEOMETRY_COLORS["tie_rod"]
    assert _line_segment_color(
        ax,
        geometry.left_wheel.kingpin,
        current_state.left_tie_rod_pickup,
    ) == PREVIEW_GEOMETRY_COLORS["knuckle_arm"]
    assert _line_segment_color(
        ax,
        geometry.left_wheel.kingpin,
        current_state.left_wheel_center,
    ) == PREVIEW_GEOMETRY_COLORS["wheel_radius"]
    line_colors = {to_hex(line.get_color()) for line in ax.lines}
    assert PREVIEW_GEOMETRY_COLORS["pitman"] in line_colors
    patch_edge_colors = {to_hex(patch.get_edgecolor()) for patch in ax.patches}
    assert PREVIEW_GEOMETRY_COLORS["wheel"] in patch_edge_colors
    plt.close(fig)
