"""Relative coordinate and spin boundary reject ambiguous physical declarations."""

import copy

import pytest

from suspension_contracts import (
    ContractError,
    validate_resolved_model,
    validate_solve_plan,
)


def coordinate_model():
    document = {
        "schema_version": 1, "name": "bearing", "units": "SI",
        "bodies": [{"name": "arm", "mass": 2, "inertia": [[1, 0, 0], [0, 1, 0], [0, 0, 1]]}],
        "frames": [{"name": "pin", "body": "arm", "point": [0, 0, 0], "quaternion": [1, 0, 0, 0]}],
        "joints": [{"name": "hinge", "type": "revolute", "body_a": "arm", "body_b": "ground", "axis_a": [0, 1, 0], "axis_b": [0, 1, 0]}],
        "elements": [], "ports": [],
    }
    document["frames"].append({"name": "reaction", "body": "ground", "point": [0, 0, 0], "quaternion": [1, 0, 0, 0]})
    document["coordinates"] = [{"name": "spin", "source_joint_id": "hinge", "kind": "rotation", "frame_a": "pin", "frame_b": "reaction", "axis": [0, 1, 0], "units": "rad", "reference": .2}]
    document["ports"].append({"name": "spin_port", "kind": "spin", "owner": "arm", "coordinate": "spin", "units": "rad"})
    return document


def plan():
    return {"schema_version": 1, "name": "run", "study": "dynamic", "samples": [0, 1], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}


def test_spin_port_points_to_one_bearing_and_two_local_frames():
    validate_resolved_model(coordinate_model())


@pytest.mark.parametrize("mutation", ["joint", "kind", "axis", "axis_direction", "frame", "units", "owner", "coordinate"])
def test_invalid_coordinate_fails_before_submission(mutation):
    document = copy.deepcopy(coordinate_model())
    coordinate = document["coordinates"][0]
    if mutation == "joint":
        coordinate["source_joint_id"] = "missing"
    elif mutation == "kind":
        document["joints"][0]["type"] = "fixed"
    elif mutation == "axis":
        coordinate["axis"] = [0, 2, 0]
    elif mutation == "axis_direction":
        coordinate["axis"] = [1, 0, 0]
    elif mutation == "frame":
        coordinate["frame_b"] = "pin"
    elif mutation == "units":
        document["ports"][-1]["units"] = "deg"
    elif mutation == "owner":
        document["ports"][-1]["owner"] = "missing"
    else:
        document["ports"][-1]["coordinate"] = "missing"
    with pytest.raises(ContractError):
        validate_resolved_model(document)


@pytest.mark.parametrize("mode", ["free", "locked", "prescribed_angle"])
def test_motion_boundary_is_separate_from_torque(mode):
    document = plan()
    boundary = {"name": "motion", "coordinate": "spin", "mode": mode, "units": "rad"}
    if mode == "locked":
        boundary["value"] = .2
    elif mode == "prescribed_angle":
        boundary["program"] = "angle"
    document["boundaries"] = [boundary]
    document["inputs"] = [{"name": "drive", "kind": "torque", "target": "spin", "value": 7}]
    validate_solve_plan(document)


def test_speed_control_is_rejected_explicitly():
    document = plan()
    document["boundaries"] = [{"name": "motion", "coordinate": "spin", "mode": "prescribed_speed", "program": "speed"}]
    with pytest.raises(ContractError, match="not supported"):
        validate_solve_plan(document)
