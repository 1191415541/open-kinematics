from pathlib import Path

import numpy as np
import pytest

from kinematics.steering import (
    load_two_segment_steering_hardpoints_csv,
    solve_two_segment_steering,
)


def write_csv(path: Path, rows: list[str]) -> None:
    path.write_text(
        "category,name,x,y,z\n" + "\n".join(rows) + "\n",
        encoding="utf-8",
    )


def test_csv_loader_mirrors_symmetric_left_hardpoints(tmp_path):
    csv_path = tmp_path / "steering_hardpoints.csv"
    write_csv(
        csv_path,
        [
            "symmetric,wheel_kingpin_lower,0,-500,280",
            "symmetric,wheel_kingpin_upper,0,-500,340",
            "symmetric,wheel_center,60,-520,320",
            "symmetric,wheel_tie_rod_pickup,-180,-420,280",
            "symmetric,pitman_output,-350,-120,285",
            "center,pitman_pivot,-350,0,300",
        ],
    )

    hardpoints = load_two_segment_steering_hardpoints_csv(csv_path)

    np.testing.assert_allclose(
        hardpoints.left_wheel.kingpin_lower,
        np.array([0.0, -500.0, 280.0]),
    )
    np.testing.assert_allclose(
        hardpoints.right_wheel.kingpin_lower,
        np.array([0.0, 500.0, 280.0]),
    )
    np.testing.assert_allclose(
        hardpoints.pitman.left_output,
        np.array([-350.0, -120.0, 285.0]),
    )
    np.testing.assert_allclose(
        hardpoints.pitman.right_output,
        np.array([-350.0, 120.0, 285.0]),
    )
    np.testing.assert_allclose(
        hardpoints.pitman.pivot,
        np.array([-350.0, 0.0, 300.0]),
    )

    solution = solve_two_segment_steering(hardpoints, pitman_angle_deg=8.0)
    assert solution.converged


def test_csv_loader_rejects_symmetric_hardpoints_on_right_side(tmp_path):
    csv_path = tmp_path / "right_side_symmetric.csv"
    write_csv(
        csv_path,
        [
            "symmetric,wheel_kingpin_lower,0,500,280",
            "symmetric,wheel_kingpin_upper,0,-500,340",
            "symmetric,wheel_center,60,-520,320",
            "symmetric,wheel_tie_rod_pickup,-180,-420,280",
            "symmetric,pitman_output,-350,-120,285",
            "center,pitman_pivot,-350,0,300",
        ],
    )

    with pytest.raises(ValueError, match="left-side"):
        load_two_segment_steering_hardpoints_csv(csv_path)


def test_csv_loader_rejects_center_hardpoints_off_centerline(tmp_path):
    csv_path = tmp_path / "off_center.csv"
    write_csv(
        csv_path,
        [
            "symmetric,wheel_kingpin_lower,0,-500,280",
            "symmetric,wheel_kingpin_upper,0,-500,340",
            "symmetric,wheel_center,60,-520,320",
            "symmetric,wheel_tie_rod_pickup,-180,-420,280",
            "symmetric,pitman_output,-350,-120,285",
            "center,pitman_pivot,-350,5,300",
        ],
    )

    with pytest.raises(ValueError, match="centerline"):
        load_two_segment_steering_hardpoints_csv(csv_path)
