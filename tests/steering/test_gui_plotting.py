import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")
import numpy as np  # noqa: E402
from matplotlib import pyplot as plt  # noqa: E402
from matplotlib.colors import to_hex  # noqa: E402

from kinematics.gui.steering.plotting import (  # noqa: E402
    PREVIEW_GEOMETRY_COLORS,
    draw_curve_plot,
    draw_steering_preview,
    draw_three_segment_steering_preview,
    fit_steering_preview,
)
from kinematics.steering.three_segment import solve_three_segment_steering  # noqa: E402
from kinematics.steering.two_segment import solve_two_segment_steering  # noqa: E402
from kinematics.steering.workbench import (  # noqa: E402
    default_steering_project,
    hardpoints_from_rows,
    three_segment_geometry_from_rows,
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
        if len(points.shape) != 2 or points.shape[1] != 2:
            continue
        for index in range(points.shape[0] - 1):
            segment_start = points[index]
            segment_end = points[index + 1]
            forward = np.allclose(segment_start, start) and np.allclose(
                segment_end, end
            )
            reverse = np.allclose(segment_start, end) and np.allclose(
                segment_end, start
            )
            if forward or reverse:
                return to_hex(line.get_color())
    return None


def _has_polygon(ax, expected_points, color):
    expected = np.asarray(expected_points, dtype=np.float64)
    for patch in ax.patches:
        if to_hex(patch.get_edgecolor()) != color:
            continue
        points = np.asarray(patch.get_xy(), dtype=np.float64)
        if points.shape != (4, 2):
            continue
        if not np.allclose(points[0], points[-1]):
            continue
        actual = points[:-1]
        has_expected_points = all(
            any(np.allclose(point, candidate) for candidate in actual)
            for point in expected
        )
        if has_expected_points:
            return True
    return False


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


def test_steering_preview_uses_visual_wheel_dimensions():
    fig, ax = plt.subplots()
    hardpoints, design_state, current_state = _preview_inputs()

    draw_steering_preview(
        ax,
        hardpoints,
        design_state,
        current_state,
        wheel_radius=260.0,
        wheel_width=180.0,
    )

    wheel_patch = ax.patches[0]
    vertices = wheel_patch.get_xy()[:4]
    side_lengths = [
        np.linalg.norm(vertices[(index + 1) % 4] - vertices[index])
        for index in range(4)
    ]
    np.testing.assert_allclose(sorted(side_lengths), [180.0, 180.0, 520.0, 520.0])
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


def test_three_segment_preview_draws_center_link_and_bellcrank_tie_rods():
    fig, ax = plt.subplots()
    project = default_steering_project(linkage_type="three_segment")
    geometry = three_segment_geometry_from_rows(project.hardpoints)
    design_state = solve_three_segment_steering(geometry, 0.0)
    current_state = solve_three_segment_steering(geometry, 8.0)

    draw_three_segment_steering_preview(ax, geometry, design_state, current_state)

    assert _line_segment_color(
        ax,
        current_state.left_bellcrank_center_link_pickup,
        current_state.right_bellcrank_center_link_pickup,
    ) == PREVIEW_GEOMETRY_COLORS["center_link"]
    assert _line_segment_color(
        ax,
        current_state.left_bellcrank_tie_rod_pickup,
        current_state.left_tie_rod_pickup,
    ) == PREVIEW_GEOMETRY_COLORS["tie_rod"]
    assert _line_segment_color(
        ax,
        geometry.left_bellcrank.pivot,
        current_state.left_bellcrank_tie_rod_pickup,
    ) == PREVIEW_GEOMETRY_COLORS["bellcrank"]
    plt.close(fig)


def test_three_segment_preview_draws_bellcranks_as_triangles():
    fig, ax = plt.subplots()
    project = default_steering_project(linkage_type="three_segment")
    geometry = three_segment_geometry_from_rows(project.hardpoints)
    design_state = solve_three_segment_steering(geometry, 0.0)
    current_state = solve_three_segment_steering(geometry, 8.0)

    draw_three_segment_steering_preview(ax, geometry, design_state, current_state)

    assert _has_polygon(
        ax,
        [
            geometry.left_bellcrank.pivot,
            current_state.left_bellcrank_center_link_pickup,
            current_state.left_bellcrank_tie_rod_pickup,
        ],
        PREVIEW_GEOMETRY_COLORS["bellcrank"],
    )
    assert _has_polygon(
        ax,
        [
            geometry.right_bellcrank.pivot,
            current_state.right_bellcrank_center_link_pickup,
            current_state.right_bellcrank_tie_rod_pickup,
        ],
        PREVIEW_GEOMETRY_COLORS["bellcrank"],
    )
    plt.close(fig)


def test_curve_plot_keeps_x_label_inside_figure():
    fig, ax = plt.subplots(figsize=(5.6, 2.9), dpi=100)
    rows = [
        {"input_value": float(value), "left_bellcrank_angle_deg": float(value * value)}
        for value in range(-5, 6)
    ]

    draw_curve_plot(
        ax,
        rows,
        [("input_value", "left_bellcrank_angle_deg", "preview")],
    )
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()

    figure_bottom = fig.bbox.y0
    label_bottom = ax.xaxis.label.get_window_extent(renderer).y0
    assert label_bottom >= figure_bottom
    plt.close(fig)


def test_curve_plot_keeps_y_label_inside_narrow_figure():
    fig, ax = plt.subplots(figsize=(3.6, 3.0), dpi=100)
    rows = [
        {"input_value": float(value), "left_bellcrank_angle_deg": float(value * value)}
        for value in range(-5, 6)
    ]

    draw_curve_plot(
        ax,
        rows,
        [("input_value", "left_bellcrank_angle_deg", "preview")],
    )
    fig.canvas.draw()
    renderer = fig.canvas.get_renderer()

    figure_left = fig.bbox.x0
    label_left = ax.yaxis.label.get_window_extent(renderer).x0
    assert label_left >= figure_left
    plt.close(fig)
