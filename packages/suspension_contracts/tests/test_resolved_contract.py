"""The SI graph and run plan have distinct, strict contracts."""

import copy

import pytest

from suspension_contracts import (
    ContractError,
    validate_resolved_model,
    validate_solve_plan,
)


def model():
    return {
        "schema_version": 1, "name": "mechanism", "units": "SI",
        "bodies": [{"name": "arm", "mass": 2., "inertia": [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]], "center_of_mass": [0.1, 0., 0.]}],
        "frames": [{"name": "pin", "body": "arm", "point": [0., 0., 0.], "quaternion": [1., 0., 0., 0.]}],
        "joints": [{"name": "hinge", "type": "revolute", "body_a": "arm", "body_b": "ground", "axis_a": [0., 1., 0.], "axis_b": [0., 1., 0.]}],
        "elements": [], "ports": [{"name": "mount", "kind": "geometry", "owner": "arm", "frame": "pin"}],
    }


def plan():
    return {"schema_version": 1, "name": "run", "study": "dynamic", "samples": [0., 1.], "solver": {}, "boundaries": [], "inputs": [], "outputs": []}


def test_graph_is_si_and_accepts_local_com():
    validate_resolved_model(model())


@pytest.mark.parametrize("mutation", ["units", "body", "duplicate", "frame", "inertia", "negative", "triangle"])
def test_invalid_graph_is_rejected(mutation):
    document = model()
    if mutation == "units":
        document["units"] = "mm"
    elif mutation == "body":
        document["joints"][0]["body_b"] = "missing"
    elif mutation == "duplicate":
        document["bodies"].append(copy.deepcopy(document["bodies"][0]))
    elif mutation == "frame":
        document["ports"][0]["frame"] = "missing"
    elif mutation == "inertia":
        document["bodies"][0]["inertia"][0][1] = 0.2
    elif mutation == "negative":
        document["bodies"][0]["inertia"][0][0] = -1.
    elif mutation == "triangle":
        document["bodies"][0]["inertia"][0][0] = 3.
    with pytest.raises(ContractError):
        validate_resolved_model(document)


def test_negative_products_of_inertia_are_legal():
    document = model()
    document["bodies"][0]["inertia"] = [[1., -0.1, 0.], [-0.1, 1., 0.], [0., 0., 1.]]
    validate_resolved_model(document)


def test_plan_cannot_carry_a_second_model():
    document = plan()
    validate_solve_plan(document)
    document["dynamic_model"] = model()
    with pytest.raises(ContractError, match="unexpected"):
        validate_solve_plan(document)


def test_torque_is_an_input_and_can_coexist_with_lock():
    document = plan()
    document["boundaries"] = [{"name": "lock", "coordinate": "spin", "mode": "locked", "value": 0., "units": "rad"}]
    document["inputs"] = [{"name": "drive", "kind": "torque", "target": "spin", "value": 10.}]
    validate_solve_plan(document)
    document["boundaries"][0]["mode"] = "torque"
    with pytest.raises(ContractError):
        validate_solve_plan(document)


def test_duplicate_motion_is_rejected():
    document = plan()
    document["boundaries"] = [
        {"name": "lock", "coordinate": "spin", "mode": "locked", "value": 0.},
        {"name": "release", "coordinate": "spin", "mode": "free"},
    ]
    with pytest.raises(ContractError, match="duplicate motion"):
        validate_solve_plan(document)
