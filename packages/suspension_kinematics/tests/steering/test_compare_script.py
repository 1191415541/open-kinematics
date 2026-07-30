import csv
from pathlib import Path

import numpy as np

from suspension_kinematics.steering import load_two_segment_steering_hardpoints_csv
from suspension_kinematics.steering.comparison import build_comparison_rows


def _write_sample_csv(path: Path) -> None:
    with path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(("category", "name", "x", "y", "z"))
        writer.writerow(
            ("symmetric", "wheel_kingpin_lower", -10.311, -939.693, -166.505)
        )
        writer.writerow(("symmetric", "wheel_kingpin_upper", 5.246, -899.264, 87.832))
        writer.writerow(("symmetric", "wheel_center", 6.608, -1083.36, -0.909))
        writer.writerow(
            (
                "symmetric",
                "wheel_tie_rod_pickup",
                350.936,
                -826.302,
                -88.396,
            )
        )
        writer.writerow(("symmetric", "pitman_output", 361.524, -33.332, -132.0))
        writer.writerow(("center", "pitman_pivot", 0.0, 0.0, 0.0))


def test_build_comparison_rows_emits_expected_columns(tmp_path: Path) -> None:
    csv_path = tmp_path / "steering.csv"
    _write_sample_csv(csv_path)
    hardpoints = load_two_segment_steering_hardpoints_csv(csv_path)

    rows = build_comparison_rows(
        hardpoints,
        pitman_angles_deg=[-10.0, 0.0, 10.0],
    )

    assert len(rows) == 3
    assert rows[0]["pitman_angle_deg"] == -10.0
    assert rows[1]["pitman_angle_deg"] == 0.0
    assert rows[2]["pitman_angle_deg"] == 10.0
    assert "left_wheel_angle_2d_deg" in rows[0]
    assert "left_wheel_angle_3d_deg" in rows[0]
    assert "left_wheel_angle_delta_deg" in rows[0]
    assert "max_abs_wheel_angle_delta_deg" in rows[0]
    assert "left_tie_rod_residual_3d" in rows[0]
    assert "right_tie_rod_residual_3d" in rows[0]


def test_build_comparison_rows_detects_projection_delta_for_inclined_kingpin(
    tmp_path: Path,
) -> None:
    csv_path = tmp_path / "steering.csv"
    _write_sample_csv(csv_path)
    hardpoints = load_two_segment_steering_hardpoints_csv(csv_path)

    rows = build_comparison_rows(
        hardpoints,
        pitman_angles_deg=[10.0],
    )

    assert len(rows) == 1
    assert abs(rows[0]["left_wheel_angle_delta_deg"]) > 1e-4
    assert abs(rows[0]["right_wheel_angle_delta_deg"]) > 1e-4
    np.testing.assert_allclose(rows[0]["left_tie_rod_residual_3d"], 0.0, atol=1e-6)
    np.testing.assert_allclose(rows[0]["right_tie_rod_residual_3d"], 0.0, atol=1e-6)
