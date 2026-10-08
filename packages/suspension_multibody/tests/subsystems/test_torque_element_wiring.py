"""Peer brake/drive subsystems bind ports and preserve native demand and wrench units."""

import ast
import inspect

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import (
    assemble_generic,
    generic,
    migrate_v1_vehicle,
    migrate_v1_vehicle_case,
)
from suspension_multibody.schema import TimeSignal
from tests.vehicle import vehicle_fixtures as fixture


def _with_demand(demand, **values):
    model = fixture._positioned_vehicle(fixture._vehicle())
    return model.model_copy(update={"driveline": model.driveline.model_copy(update={"torque_demand": demand, **values})})


def _torques(model):
    return tuple(row for row in assemble_generic(migrate_v1_vehicle(model)).elements if row["type"] == "rotational_torque")


def _documents(model, brake=.4):
    return migrate_v1_vehicle_case(fixture._case(model, brake=brake))


def test_the_default_model_gets_no_torque_element():
    model = _with_demand("none")
    assert model.driveline.torque_demand == "none"
    assert _torques(model) == ()


def test_a_declared_brake_demand_gives_one_element_per_braked_wheel():
    model = _with_demand("brake")
    elements = _torques(model)
    braked = tuple(wheel.name for wheel in model.wheels if wheel.braked)
    assert len(elements) == len(braked) == 4
    for row in elements:
        wheel = row["name"].removeprefix("brake_").removesuffix(".brake")
        body = next(item.body for item in model.wheels if item.name == wheel)
        assert row["body_b"] == "wheel_"+wheel+"."+body
        assert row["body_a"] != row["body_b"]


def test_a_declared_drive_demand_gives_one_element_per_driven_wheel():
    model = _with_demand("drive", driven_wheels=("rear_left", "rear_right"), drive_split=(0, 0, .5, .5))
    assert {row["name"] for row in _torques(model)} == {"drive_rear_left.drive", "drive_rear_right.drive"}


def test_both_declarations_together_are_two_channels():
    model = _with_demand("both", driven_wheels=("rear_left", "rear_right"), drive_split=(0, 0, .5, .5))
    elements = _torques(model)
    assert len(elements) == 6
    assert {row["name"] for row in elements if row["name"].startswith("brake_")} == {
        "brake_"+wheel+".brake" for wheel in ("front_left", "front_right", "rear_left", "rear_right")}


def test_the_two_ends_are_resolved_by_a_port_match_not_by_a_name():
    literals = {node.value for node in ast.walk(ast.parse(inspect.getsource(generic)))
                if isinstance(node, ast.Constant) and isinstance(node.value, str)}
    assert not {"upright_L", "upright_R", "chassis", "wheel_front_left"}.intersection(literals)


def test_the_gain_is_converted_once_to_the_kernel_units():
    full_amplitude = 2*2500*.1*.4*145
    assert full_amplitude == pytest.approx(29000)
    expected = full_amplitude*(.6/2)*.001
    assert expected == pytest.approx(8.7)
    compiled = validate(*_documents(_with_demand("brake")))
    front = next(row for row in compiled.model_document["elements"] if row["name"] == "brake_front_left.brake")
    assert front["parameters"]["stiffness"] == pytest.approx(expected)
    assert front["parameters"]["max_torque"] == pytest.approx(expected)
    assert front["parameters"]["demand_source"] == 2
    assert front["parameters"]["demand_tire"] == 0


def test_the_document_carries_the_couple_and_the_normalized_demand():
    compiled = validate(*_documents(_with_demand("brake")))
    assert sum(row["type"] == "rotational_torque" for row in compiled.model_document["elements"]) == 4
    couple = next(row for row in compiled.model_document["elements"] if row["type"] == "rotational_torque")
    assert set(couple["parameters"]) >= {"axis_a", "stiffness", "max_torque"}
    roles = {row["role"] for row in compiled.case_document["blobs"]}
    assert "brake_pressure" in roles
    assert "brake_torque" not in roles
    assert "steering_target" in roles


def test_a_document_that_declares_nothing_preserves_explicit_torque_tables():
    compiled = validate(*_documents(_with_demand("none")))
    assert not any(row["type"] == "rotational_torque" for row in compiled.model_document["elements"])
    roles = {row["role"] for row in compiled.case_document["blobs"]}
    assert "brake_torque" in roles
    assert not {"brake_pressure", "throttle_demand"}.intersection(roles)


def test_the_couple_reaches_the_solver_as_an_element():
    guarded = simulate(*_documents(_with_demand("brake"), brake=1)).raw
    control = simulate(*_documents(_with_demand("none"), brake=1)).raw
    active = guarded.blocks["element_wrench"]
    rows = active[active[:, :, 6] == 10]
    baseline = control.blocks["element_wrench"]
    assert baseline[baseline[:, :, 6] == 10].shape[0] == 0
    assert rows.shape[0] == 4*2*active.shape[0]
    for index in range(0, len(rows), 2):
        np.testing.assert_allclose(rows[index, 3:6], -rows[index+1, 3:6], atol=0)
    np.testing.assert_allclose(rows[:, 3:6], 0)


@pytest.mark.parametrize("demand", ["none", "brake", "drive", "both"])
def test_every_declaration_uses_one_loader_and_compiler(demand, monkeypatch):
    import builtins

    original = builtins.__import__
    def reject(name, *args, **kwargs):
        if name.startswith("suspension_multibody.preparation") or name.startswith("suspension_multibody.subsystems"):
            raise AssertionError("retired execution import: "+name)
        return original(name, *args, **kwargs)
    documents = _documents(_with_demand(demand))
    monkeypatch.setattr(builtins, "__import__", reject)
    assert validate(*documents).metadata["compiler"] == "ResolvedModelCompiler"


def test_peer_brakes_do_not_read_the_old_front_bias():
    low = _with_demand("brake", front_brake_bias=.01)
    high = _with_demand("brake", front_brake_bias=.99)
    assert _torques(low) == _torques(high)
    assert _torques(low)[0]["parameters"]["max_torque"] == pytest.approx(8.7)


@pytest.mark.parametrize("demand,field", [("brake", "wheel_drive_torque"), ("drive", "wheel_brake_torque")])
def test_a_peer_demand_refuses_explicit_wheel_torques_it_would_discard(demand, field):
    model = _with_demand(demand)
    signal = TimeSignal(times=np.array([0, .001]), values=np.array([0, 100]))
    case = fixture._case(model).model_copy(update={field: (("front_left", signal),)})
    with pytest.raises(ValueError, match=field):
        migrate_v1_vehicle_case(case)
