import numpy as np

from suspension_kinematics.cli import coupled_sweep as cli_coupled_sweep
from suspension_kinematics.core.enums import PointID
from suspension_kinematics.io.coupled_loader import parse_coupled_sweep_file
from suspension_kinematics.io.geometry_loader import load_geometry
from suspension_kinematics.steering import (
    PitmanArmHardpoints3D,
    TwoSegmentSteeringHardpoints3D,
    WheelSteeringHardpoints3D,
)
from suspension_kinematics.vehicle.coupled import (
    build_symmetric_corner_pair,
    solve_coupled_sweep,
)


def suspension_matched_steering() -> TwoSegmentSteeringHardpoints3D:
    return TwoSegmentSteeringHardpoints3D(
        left_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([0.0, -900.0, 200.0]),
            kingpin_upper=np.array([-25.0, -750.0, 500.0]),
            wheel_center=np.array([-20.0, -950.0, 313.426]),
            tie_rod_pickup=np.array([150.0, -800.0, 275.0]),
        ),
        right_wheel=WheelSteeringHardpoints3D(
            kingpin_lower=np.array([0.0, 900.0, 200.0]),
            kingpin_upper=np.array([-25.0, 750.0, 500.0]),
            wheel_center=np.array([-20.0, 950.0, 313.426]),
            tie_rod_pickup=np.array([150.0, 800.0, 275.0]),
        ),
        pitman=PitmanArmHardpoints3D(
            pivot=np.array([50.0, 0.0, 250.0]),
            left_output=np.array([50.0, -200.0, 250.0]),
            right_output=np.array([50.0, 200.0, 250.0]),
        ),
    )


def test_build_symmetric_corner_pair_mirrors_positive_y_geometry_to_left(
    double_wishbone_geometry_file,
):
    source = load_geometry(double_wishbone_geometry_file)

    pair = build_symmetric_corner_pair(source)

    assert pair.source_side == "right"
    np.testing.assert_allclose(
        pair.left.hardpoints[PointID.LOWER_WISHBONE_OUTBOARD],
        np.array([0.0, -900.0, 200.0]),
    )
    np.testing.assert_allclose(
        pair.right.hardpoints[PointID.LOWER_WISHBONE_OUTBOARD],
        np.array([0.0, 900.0, 200.0]),
    )
    assert pair.left.config is not None
    assert pair.left.config.camber_shim is not None
    np.testing.assert_allclose(
        pair.left.config.camber_shim.shim_face_point_a,
        np.array([-25.0, -750.0, 510.0]),
    )
    np.testing.assert_allclose(
        pair.left.config.camber_shim.shim_face_normal,
        np.array([0.0, -1.0, 0.0]),
    )


def test_coupled_sweep_maps_steering_outputs_to_trackrod_inboard_3d_targets(
    double_wishbone_geometry_file,
):
    source = load_geometry(double_wishbone_geometry_file)

    results = solve_coupled_sweep(
        source_suspension=source,
        steering_geometry=suspension_matched_steering(),
        wheel_travel_values=[0.0],
        pitman_angle_values=[6.0],
    )

    assert len(results) == 1
    result = results[0]
    assert result.solver_info.converged
    assert result.steering.converged
    rack_z = source.initial_state().positions[PointID.TRACKROD_INBOARD][2]
    np.testing.assert_allclose(
        result.left_state.positions[PointID.TRACKROD_INBOARD],
        np.array(
            [
                result.steering.pitman_left_output[0],
                result.steering.pitman_left_output[1],
                rack_z,
            ]
        ),
        atol=1e-6,
    )
    np.testing.assert_allclose(
        result.right_state.positions[PointID.TRACKROD_INBOARD],
        np.array(
            [
                result.steering.pitman_right_output[0],
                result.steering.pitman_right_output[1],
                rack_z,
            ]
        ),
        atol=1e-6,
    )
    assert "left_camber_deg" in result.metrics
    assert "right_camber_deg" in result.metrics
    assert result.metrics["steering_pitman_angle_deg"] == 6.0
    assert (
        result.metrics["steering_left_wheel_angle_deg"]
        == result.steering.left_wheel_angle_deg
    )
    assert abs(result.metrics["steering_left_wheel_angle_deg"]) > 0.0


def test_parse_coupled_sweep_file_expands_values(tmp_path):
    sweep_path = tmp_path / "coupled_sweep.yaml"
    sweep_path.write_text(
        """
version: 1
wheel_travel:
  start: -10
  stop: 10
  steps: 3
pitman_angle:
  values: [-5, 0, 5]
""",
        encoding="utf-8",
    )

    config = parse_coupled_sweep_file(sweep_path)

    assert config.wheel_travel_values == [-10.0, 0.0, 10.0]
    assert config.pitman_angle_values == [-5.0, 0.0, 5.0]


def test_coupled_sweep_cli_writes_prefixed_vehicle_csv(
    tmp_path,
    double_wishbone_geometry_file,
):
    steering_path = tmp_path / "steering.csv"
    steering_path.write_text(
        "\n".join(
            [
                "category,name,x,y,z",
                "symmetric,wheel_kingpin_lower,0,-900,200",
                "symmetric,wheel_kingpin_upper,-25,-750,500",
                "symmetric,wheel_center,-20,-950,313.426",
                "symmetric,wheel_tie_rod_pickup,150,-800,275",
                "symmetric,pitman_output,50,-200,250",
                "center,pitman_pivot,50,0,250",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    coupled_path = tmp_path / "coupled.yaml"
    coupled_path.write_text(
        """
version: 1
wheel_travel:
  values: [0]
pitman_angle:
  values: [6]
""",
        encoding="utf-8",
    )
    out_path = tmp_path / "coupled.csv"

    cli_coupled_sweep(
        geometry=double_wishbone_geometry_file,
        steering=steering_path,
        coupled_sweep=coupled_path,
        out=out_path,
    )

    content = out_path.read_text(encoding="utf-8")
    assert "steering_left_wheel_angle_deg" in content
    assert "left_camber_deg" in content
    assert "right_camber_deg" in content
    assert "left_WHEEL_CENTER_x" in content
    assert "right_WHEEL_CENTER_x" in content


def test_coupled_sweep_cli_writes_vehicle_gif(
    tmp_path,
    double_wishbone_geometry_file,
):
    import pytest

    pytest.importorskip("matplotlib")

    steering_path = tmp_path / "steering.csv"
    steering_path.write_text(
        "\n".join(
            [
                "category,name,x,y,z",
                "symmetric,wheel_kingpin_lower,0,-900,200",
                "symmetric,wheel_kingpin_upper,-25,-750,500",
                "symmetric,wheel_center,-20,-950,313.426",
                "symmetric,wheel_tie_rod_pickup,150,-800,275",
                "symmetric,pitman_output,50,-200,250",
                "center,pitman_pivot,50,0,250",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    coupled_path = tmp_path / "coupled.yaml"
    coupled_path.write_text(
        """
version: 1
wheel_travel:
  values: [-10, 0]
pitman_angle:
  values: [-3, 3]
""",
        encoding="utf-8",
    )
    out_path = tmp_path / "coupled.csv"
    animation_path = tmp_path / "coupled.gif"

    cli_coupled_sweep(
        geometry=double_wishbone_geometry_file,
        steering=steering_path,
        coupled_sweep=coupled_path,
        out=out_path,
        animation_out=animation_path,
    )

    assert animation_path.exists()
    assert animation_path.stat().st_size > 0
