import json
from pathlib import Path

import numpy as np
import pytest

from kinematics.core.enums import PointID
from kinematics.gui.suspension.workbench import (
    SUSPENSION_GUI_COORDINATE_SYSTEM,
    create_default_suspension_project,
    load_suspension_hardpoints_csv,
    load_suspension_project,
    save_suspension_hardpoints_csv,
    save_suspension_project,
    suspension_gui_to_internal_vec3,
    suspension_internal_to_gui_vec3,
)


def test_suspension_coordinate_transform_is_self_inverse() -> None:
    internal = np.asarray([120.0, 40.0, 180.0], dtype=np.float64)

    gui = suspension_internal_to_gui_vec3(internal)

    np.testing.assert_allclose(gui, np.asarray([-120.0, -40.0, 180.0]))
    np.testing.assert_allclose(suspension_gui_to_internal_vec3(gui), internal)


def test_suspension_hardpoint_csv_uses_gui_coordinate_convention(tmp_path: Path) -> None:
    path = tmp_path / "hardpoints.csv"
    hardpoints = {
        PointID.TRACKROD_INBOARD: np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    }

    save_suspension_hardpoints_csv(hardpoints, path)

    text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
    assert "TRACKROD_INBOARD,-120,-40,180\n" in text

    path.write_text("point,x,y,z\nTRACKROD_INBOARD,-120,-40,180\n", encoding="utf-8")
    loaded = load_suspension_hardpoints_csv(path)

    np.testing.assert_allclose(
        loaded[PointID.TRACKROD_INBOARD],
        np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    )


def test_suspension_project_json_uses_gui_coordinate_convention(tmp_path: Path) -> None:
    path = tmp_path / "project.okproj.json"
    project = create_default_suspension_project()
    project.hardpoints = {
        PointID.TRACKROD_INBOARD: np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    }
    project.config = project.config.model_copy(
        update={"cg_position": (1250.0, 50.0, 450.0)}
    )

    save_suspension_project(project, path)
    data = json.loads(path.read_text(encoding="utf-8"))

    assert data["parameters"]["coordinate_system"] == SUSPENSION_GUI_COORDINATE_SYSTEM
    assert data["hardpoints"]["TRACKROD_INBOARD"] == {
        "x": -120.0,
        "y": -40.0,
        "z": 180.0,
    }
    assert data["parameters"]["config"]["cg_position"] == {
        "x": -1250.0,
        "y": -50.0,
        "z": 450.0,
    }

    loaded = load_suspension_project(path)
    np.testing.assert_allclose(
        loaded.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    )
    np.testing.assert_allclose(
        loaded.config.cg_position,
        np.asarray([1250.0, 50.0, 450.0], dtype=np.float64),
    )


def test_legacy_suspension_project_without_coordinate_system_remains_internal(
    tmp_path: Path,
) -> None:
    path = tmp_path / "legacy.okproj.json"
    project = create_default_suspension_project()
    legacy_data = {
        "schema_version": 1,
        "module": "suspension",
        "system_type": project.suspension_type,
        "name": project.name,
        "version": project.version,
        "units": project.units.name,
        "hardpoints": {
            "TRACKROD_INBOARD": {"x": 120.0, "y": 40.0, "z": 180.0},
        },
        "parameters": {
            "config": {
                "steered": project.config.steered,
                "wheel": {
                    "offset": float(project.config.wheel.offset),
                    "tire": {
                        "aspect_ratio": float(project.config.wheel.tire.aspect_ratio),
                        "section_width": float(project.config.wheel.tire.section_width),
                        "static_radius_mm": float(
                            project.config.wheel.tire.static_radius_mm
                        ),
                    },
                },
                "cg_position": {"x": 1250.0, "y": 50.0, "z": 450.0},
                "wheelbase": float(project.config.wheelbase),
                "upright_mounted_points": list(project.config.upright_mounted_points),
            }
        },
        "simulation": {"start": -40.0, "stop": 120.0, "steps": 41},
        "curves": [],
    }
    path.write_text(json.dumps(legacy_data), encoding="utf-8")

    loaded = load_suspension_project(path)

    np.testing.assert_allclose(
        loaded.hardpoints[PointID.TRACKROD_INBOARD],
        np.asarray([120.0, 40.0, 180.0], dtype=np.float64),
    )
    np.testing.assert_allclose(
        loaded.config.cg_position,
        np.asarray([1250.0, 50.0, 450.0], dtype=np.float64),
    )
    assert loaded.config.wheel.tire.static_radius_mm == pytest.approx(283.1)
