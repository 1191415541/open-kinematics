import pytest

matplotlib = pytest.importorskip("matplotlib")
matplotlib.use("Agg")
from matplotlib import pyplot as plt  # noqa: E402

from kinematics.gui.suspension.plotting import draw_suspension_preview  # noqa: E402
from kinematics.gui.suspension.workbench import (  # noqa: E402
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
