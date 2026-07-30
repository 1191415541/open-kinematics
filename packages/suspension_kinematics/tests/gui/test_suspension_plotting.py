import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")
import numpy as np  # noqa: E402
from matplotlib import pyplot as plt  # noqa: E402

from suspension_kinematics.gui.suspension.plotting import (  # noqa: E402
    SuspensionPreviewRenderer,
    apply_preview_view_plane,
    draw_suspension_preview,
)
from suspension_kinematics.gui.suspension.workbench import (  # noqa: E402
    load_suspension_project,
    solve_suspension_project_at_travel,
)


def test_suspension_preview_can_preserve_existing_3d_view_limits(
    double_wishbone_geometry_file,
):
    project = load_suspension_project(double_wishbone_geometry_file)
    suspension = project.build_suspension()
    first = suspension.initial_state()
    second = solve_suspension_project_at_travel(project, 50.0).states[0]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    draw_suspension_preview(ax, suspension, first)
    ax.set_xlim3d(-100.0, 200.0)
    ax.set_ylim3d(300.0, 1000.0)
    ax.set_zlim3d(100.0, 800.0)
    ax.view_init(elev=15.0, azim=35.0)

    draw_suspension_preview(ax, suspension, second, preserve_view=True)

    assert ax.get_xlim3d() == (-100.0, 200.0)
    assert ax.get_ylim3d() == (300.0, 1000.0)
    assert ax.get_zlim3d() == (100.0, 800.0)
    assert ax.elev == 15.0
    assert ax.azim == 35.0
    plt.close(fig)


def test_suspension_preview_renderer_preserves_view_across_preview_and_full_modes(
    double_wishbone_geometry_file,
):
    project = load_suspension_project(double_wishbone_geometry_file)
    suspension = project.build_suspension()
    first = suspension.initial_state()
    second = solve_suspension_project_at_travel(project, 40.0).states[0]
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    renderer = SuspensionPreviewRenderer()

    draw_suspension_preview(
        ax,
        suspension,
        first,
        preserve_view=False,
        renderer=renderer,
        preview_mode=True,
    )
    ax.set_xlim3d(-120.0, 180.0)
    ax.set_ylim3d(250.0, 900.0)
    ax.set_zlim3d(80.0, 760.0)
    ax.view_init(elev=12.0, azim=40.0)

    draw_suspension_preview(
        ax,
        suspension,
        second,
        preserve_view=True,
        renderer=renderer,
        preview_mode=False,
    )

    assert ax.get_xlim3d() == (-120.0, 180.0)
    assert ax.get_ylim3d() == (250.0, 900.0)
    assert ax.get_zlim3d() == (80.0, 760.0)
    assert ax.elev == 12.0
    assert ax.azim == 40.0
    plt.close(fig)


def test_apply_preview_view_plane_sets_named_camera_angles(
    double_wishbone_geometry_file,
):
    project = load_suspension_project(double_wishbone_geometry_file)
    suspension = project.build_suspension()
    state = suspension.initial_state()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    draw_suspension_preview(ax, suspension, state)

    apply_preview_view_plane(ax, "xy", positions=state.positions, fit_bounds=False)
    assert ax.elev == pytest.approx(90.0)
    assert ax.azim == pytest.approx(-90.0)

    apply_preview_view_plane(ax, "xz", positions=state.positions, fit_bounds=False)
    assert ax.elev == pytest.approx(0.0)
    assert ax.azim == pytest.approx(-90.0)

    apply_preview_view_plane(ax, "yz", positions=state.positions, fit_bounds=False)
    assert ax.elev == pytest.approx(0.0)
    assert ax.azim == pytest.approx(0.0)

    apply_preview_view_plane(ax, "zy", positions=state.positions, fit_bounds=False)
    assert ax.elev == pytest.approx(0.0)
    assert ax.azim == pytest.approx(180.0)
    plt.close(fig)


def test_suspension_preview_uses_rear_right_up_axis_labels_and_coordinates(
    double_wishbone_geometry_file,
):
    project = load_suspension_project(double_wishbone_geometry_file)
    suspension = project.build_suspension()
    state = suspension.initial_state()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    draw_suspension_preview(ax, suspension, state)

    assert ax.get_xlabel() == "X rearward [mm]"
    assert ax.get_ylabel() == "Y rightward [mm]"
    assert ax.get_zlabel() == "Z upward [mm]"

    first_line = ax.lines[0]
    first_point_id = suspension.get_visualization_links()[0].points[0]
    expected = state.positions[first_point_id]
    xdata, ydata, zdata = first_line.get_data_3d()
    assert np.isclose(xdata[0], -expected[0])
    assert np.isclose(ydata[0], -expected[1])
    assert np.isclose(zdata[0], expected[2])
    plt.close(fig)


def test_suspension_preview_places_legend_in_figure_corner_outside_axis(
    double_wishbone_geometry_file,
):
    project = load_suspension_project(double_wishbone_geometry_file)
    suspension = project.build_suspension()
    state = suspension.initial_state()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    draw_suspension_preview(ax, suspension, state)

    legend = ax.get_legend()
    assert legend is not None
    assert ax.get_position().x0 > 0.2
    anchor = legend.get_bbox_to_anchor().transformed(fig.transFigure.inverted())
    assert anchor.x0 == pytest.approx(0.02)
    assert anchor.y0 == pytest.approx(0.96)
    assert anchor.x0 < ax.get_position().x0

    plt.close(fig)
