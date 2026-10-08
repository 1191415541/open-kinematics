"""A drive subsystem compiles its signed transfer formula into native code."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import TemplateDocument
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.presets import generic_template

from ._torque import locked_case, torque_assembly, torque_graph


def test_drive_has_no_implicit_bodies():
    graph = torque_graph("drive")
    assert list(graph.bodies) == ["support.carrier", "wheel.wheel"]
    assert len(graph.elements) == 1
    assert graph.elements[0]["type"] == "torque"
    assert len(graph.function_programs) == 1


def test_transfer_properties_and_formula_round_trip(tmp_path):
    template = generic_template("drive")
    assert TemplateDocument.load(template.save(tmp_path / "drive.json")).payload == template.payload
    assert {row["name"]: row["default"] for row in template.payload["property_slots"]} == {
        "max_torque": 0, "gear_ratio": 1, "efficiency": 1, "share": 1}
    assert template.payload["elements"][0]["function"] == "max_torque * gear_ratio * efficiency * share * input"


@pytest.mark.parametrize("demand", [-1, -.5, 0, .5, 1])
@pytest.mark.parametrize("gain,ratio,efficiency,share", [(2000, 1, 1, .5), (1234.5, 1, 1, 1), (500, 4, .9, 1), (500, 1, 1, 0)])
def test_signed_drive_transfer_is_the_actual_native_couple(demand, gain, ratio, efficiency, share):
    values = {"max_torque": gain, "gear_ratio": ratio, "efficiency": efficiency, "share": share}
    run = simulate(torque_assembly("drive", values=values, demand=demand), locked_case())
    a = run.result.element_wrench("unit.drive", body_id="support.carrier")
    b = run.result.element_wrench("unit.drive", body_id="wheel.wheel")
    np.testing.assert_array_equal(a.moment, -b.moment)
    np.testing.assert_array_equal(b.force, 0)
    np.testing.assert_allclose(b.moment[:, 1], gain * ratio * efficiency * share * demand, atol=1e-12, rtol=1e-12)
    np.testing.assert_array_equal(b.moment[:, [0, 2]], 0)


@pytest.mark.parametrize("owner", ["subframe", "vehicle_body", "motor_stator"])
def test_drive_reacts_against_the_exact_port_owner(owner):
    row = torque_graph("drive", reaction=owner).elements[0]["parameters"]
    assert row["reaction"]["body"] == owner + ".carrier"
    assert row["action"]["body"] == "wheel.wheel"


def test_unknown_action_endpoint_is_rejected():
    payload = generic_template("drive").to_payload()
    payload["elements"][0]["action"] = "@spare"
    with pytest.raises(AuthoringError, match="spare"):
        TemplateDocument.from_payload(payload)


def test_drive_compiler_records_normalized_signal_data():
    compiled = validate(torque_assembly("drive", demand=-1), locked_case())
    assert compiled.metadata["compiler"] == "ResolvedModelCompiler"
    assert any(row["source"] == "signal" for row in compiled.model_document["function_programs"][0]["bindings"])
