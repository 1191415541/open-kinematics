"""Brake declarations preserve the Adams magnitude, units and port-owned ends."""

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring import TemplateDocument
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.presets import generic_template

from ._torque import locked_case, torque_assembly, torque_graph


def test_zero_body_brake_contributes_only_the_declared_force():
    built = torque_graph("brake")
    assert not any(name.startswith("unit.") for name in built.bodies)
    assert len(built.elements) == 1
    assert built.elements[0]["type"] == "rotational_torque"
    assert built.elements[0]["parameters"]["axis_a"] == [0, 1, 0]


def test_all_recipe_slots_and_defaults_round_trip(tmp_path):
    template = generic_template("brake")
    assert TemplateDocument.load(template.save(tmp_path / "brake.json")).payload == template.payload
    defaults = {row["name"]: row["default"] for row in template.payload["property_slots"]}
    assert defaults == {"piston_area": 2500, "effective_radius": 145, "friction_coeff": .4,
        "share": 1, "input": 1, "demand_scale": .1, "amplitude": 0}


@pytest.mark.parametrize("share,expected", [(.6, 17.4), (.4, 11.6), (0, 0), (1, 29)])
@pytest.mark.parametrize("demand", [0, .5, 1])
def test_adams_sforce_magnitude_is_scaled_once_to_si(share, expected, demand):
    parameters = torque_graph("brake", values={"share": share}, demand=demand).elements[0]["parameters"]
    assert parameters["stiffness"] == pytest.approx(expected * demand, abs=1e-12)
    assert parameters["max_torque"] == parameters["stiffness"]
    assert parameters["stiffness"] >= 0


@pytest.mark.parametrize("owner", ["upright", "caliper_carrier", "unrelated_fixture"])
def test_reaction_end_is_owned_by_the_bound_port(owner):
    row = torque_graph("brake", reaction=owner).elements[0]
    assert row["body_a"] == owner + ".carrier"
    assert row["body_b"] == "wheel.wheel"


def test_undeclared_reaction_endpoint_is_rejected():
    payload = generic_template("brake").to_payload()
    payload["elements"][0]["body_a"] = "@absent"
    with pytest.raises(AuthoringError, match="absent"):
        TemplateDocument.from_payload(payload)


def test_brake_declared_row_reaches_native_with_balanced_couple():
    run = simulate(torque_assembly("brake"), locked_case())
    a = run.result.element_wrench("unit.brake", body_id="support.carrier")
    b = run.result.element_wrench("unit.brake", body_id="wheel.wheel")
    np.testing.assert_array_equal(a.moment, -b.moment)
    np.testing.assert_array_equal(a.force, 0)
    assert run.status == "success"
