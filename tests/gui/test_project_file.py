import json

from kinematics.core.enums import PointID
from kinematics.gui.suspension.workbench import (
    SuspensionCurve,
    SuspensionOptimizationTarget,
    SuspensionOptimizationPairDeltaConstraint,
    SuspensionSweepSettings,
    create_default_suspension_project,
    load_suspension_project,
    save_suspension_project,
)
from kinematics.steering.workbench import (
    SteeringCurve,
    default_steering_project,
    load_steering_project,
    save_steering_project,
)


def test_suspension_project_saves_and_loads_unified_json(tmp_path):
    path = tmp_path / "suspension.okproj.json"
    project = create_default_suspension_project()
    project.name = "front suspension"
    project.settings = SuspensionSweepSettings(start=-30.0, stop=90.0, steps=7)
    project.config = project.config.model_copy(update={"wheelbase": 2650.0})
    project.optimization.variable_delta_limit = 8.0
    project.optimization.variable_names = [
        "TRACKROD_INBOARD_z",
        "UPPER_WISHBONE_OUTBOARD_z",
    ]
    project.optimization.solver_mode = "cma_es_only"
    project.optimization.targets = [
        SuspensionOptimizationTarget(
            metric_name="camber_deg",
            target_delta=-0.2,
            trend="negative",
            target_mode="endpoint_delta",
            weight=2.5,
        ),
        SuspensionOptimizationTarget(
            metric_name="toe_deg",
            target_delta=0.0,
            trend="flat",
            target_mode="absolute_value",
            weight=0.75,
        ),
    ]
    project.optimization.pair_delta_constraints = [
        SuspensionOptimizationPairDeltaConstraint(
            point_a="UPPER_WISHBONE_INBOARD_FRONT",
            point_b="UPPER_WISHBONE_INBOARD_REAR",
            label="Upper wishbone inboard front/rear",
            enabled=True,
        ),
        SuspensionOptimizationPairDeltaConstraint(
            point_a="LOWER_WISHBONE_INBOARD_FRONT",
            point_b="LOWER_WISHBONE_INBOARD_REAR",
            label="Lower wishbone inboard front/rear",
            enabled=False,
        ),
    ]
    project.curves.append(
        SuspensionCurve(
            x_output="wheel_travel_mm",
            y_output="camber_deg",
            label="camber",
        )
    )
    project.hardpoints[PointID.WHEEL_CENTER] = project.hardpoints[
        PointID.TRACKROD_OUTBOARD
    ].copy()

    save_suspension_project(project, path)

    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["schema_version"] == 1
    assert data["module"] == "suspension"
    assert data["system_type"] == "double_wishbone"
    assert data["name"] == "front suspension"
    assert "TRACKROD_OUTBOARD" in data["hardpoints"]
    assert data["parameters"]["config"]["wheelbase"] == 2650.0
    assert data["simulation"] == {
        "start": -30.0,
        "stop": 90.0,
        "steps": 7,
        "optimization": {
            "variable_delta_limit": 8.0,
            "variable_names": [
                "TRACKROD_INBOARD_z",
                "UPPER_WISHBONE_OUTBOARD_z",
            ],
            "solver_mode": "cma_es_only",
            "targets": [
                {
                    "metric_name": "camber_deg",
                    "target_delta": -0.2,
                    "trend": "negative",
                    "target_mode": "endpoint_delta",
                    "enabled": True,
                    "weight": 2.5,
                },
                {
                    "metric_name": "toe_deg",
                    "target_delta": 0.0,
                    "trend": "flat",
                    "target_mode": "absolute_value",
                    "enabled": True,
                    "weight": 0.75,
                },
            ],
            "pair_delta_constraints": [
                {
                    "point_a": "UPPER_WISHBONE_INBOARD_FRONT",
                    "point_b": "UPPER_WISHBONE_INBOARD_REAR",
                    "label": "Upper wishbone inboard front/rear",
                    "enabled": True,
                    "axes": ["x", "y", "z"],
                },
                {
                    "point_a": "LOWER_WISHBONE_INBOARD_FRONT",
                    "point_b": "LOWER_WISHBONE_INBOARD_REAR",
                    "label": "Lower wishbone inboard front/rear",
                    "enabled": False,
                    "axes": ["x", "y", "z"],
                },
            ],
        },
    }
    assert data["curves"] == [
        {
            "x_output": "wheel_travel_mm",
            "y_output": "camber_deg",
            "label": "camber",
        }
    ]

    loaded = load_suspension_project(path)

    assert loaded.name == "front suspension"
    assert loaded.suspension_type == "double_wishbone"
    assert loaded.config.wheelbase == 2650.0
    assert loaded.settings.start == -30.0
    assert loaded.settings.stop == 90.0
    assert loaded.settings.steps == 7
    assert loaded.optimization.variable_delta_limit == 8.0
    assert loaded.optimization.variable_names == [
        "TRACKROD_INBOARD_z",
        "UPPER_WISHBONE_OUTBOARD_z",
    ]
    assert loaded.optimization.solver_mode == "cma_es_only"
    assert [constraint.enabled for constraint in loaded.optimization.pair_delta_constraints] == [
        True,
        False,
    ]
    assert [target.metric_name for target in loaded.optimization.targets] == [
        "camber_deg",
        "toe_deg",
    ]
    assert [target.target_mode for target in loaded.optimization.targets] == [
        "endpoint_delta",
        "absolute_value",
    ]
    assert [target.weight for target in loaded.optimization.targets] == [2.5, 0.75]
    assert loaded.curves[0].label == "camber"
    assert PointID.TRACKROD_OUTBOARD in loaded.hardpoints


def test_steering_project_saves_and_loads_unified_json(tmp_path):
    path = tmp_path / "steering.okproj.json"
    project = default_steering_project(linkage_type="three_segment")
    project.name = "rackless steering"
    project.input_mode = "right_bellcrank_angle"
    project.input_value = 6.5
    project.wheel_radius = 310.0
    project.wheel_width = 220.0
    project.wheelbase = 2780.0
    project.curves.append(
        SteeringCurve(
            x_output="left_bellcrank_angle_deg",
            y_output="left_wheel_angle_deg",
            label="left wheel",
        )
    )

    save_steering_project(project, path)

    data = json.loads(path.read_text(encoding="utf-8"))
    assert data["schema_version"] == 1
    assert data["module"] == "steering"
    assert data["system_type"] == "three_segment"
    assert data["parameters"] == {
        "wheel_radius": 310.0,
        "wheel_width": 220.0,
        "wheelbase": 2780.0,
    }
    assert data["simulation"]["input_mode"] == "right_bellcrank_angle"
    assert data["hardpoints"][0]["name"] == "wheel_kingpin_lower"

    loaded = load_steering_project(path)

    assert loaded.name == "rackless steering"
    assert loaded.linkage_type == "three_segment"
    assert loaded.input_mode == "right_bellcrank_angle"
    assert loaded.input_value == 6.5
    assert loaded.wheel_radius == 310.0
    assert loaded.wheel_width == 220.0
    assert loaded.wheelbase == 2780.0
    assert loaded.curves[0].label == "left wheel"
