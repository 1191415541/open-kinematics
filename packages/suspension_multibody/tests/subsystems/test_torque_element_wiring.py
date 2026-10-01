"""
The brake and drive torque elements, attached to a vehicle's wheel ends.

Subtask p2-09.  The two roles were registered and never called: a vehicle
composition produced an `elements` tuple with no torque element in it, so a
model's braking and driving reached the solver only through the preparation
layer's pre-sampled per-wheel torque tables.  What this file pins, in the order
a vehicle is built:

1. **the elements exist, and only when the model asks for them.**  The default
   (`DrivelineSpec.torque_demand == "none"`) produces exactly the element list
   the assembly always produced -- that is what makes the feature additive rather
   than a change to every recorded result.  A model that declares a demand gets
   one element per declared wheel end.
2. **no body is named.**  The two ends come from a port match, and the assertion
   is structural: the module's own source contains no body-name literal.  A test
   that only compared two strings would pass for a module that happened to guess
   right about this fixture.
3. **the route carries it.**  The elements leave through the *production*
   document route -- `model_document` for the couple, `case_document` for the
   normalized demand -- and a wheel is described in one unit system, not both:
   the Newton-metre table for a wheel the elements brake is gone, and the
   `brake_pressure` role that replaces it is there.
4. **the run reads it back.**  A whole vehicle runs through
   `simulation.run_request` and the couple's response is measured on *both*
   ends, because a couple applied to one body and not reacted on another is the
   failure mode that still converges.

The unit conversion is asserted by number rather than by shape: the template's
standardized slots are the recorded Adams simple-brake figures in millimetres
and square millimetres, while the kernel's law is newton-metres, so the gain the
document carries is the amplitude converted once.  Converting twice (or not at
all) leaves every sign and every body pair correct and changes the magnitude by
a factor of a thousand, which no structural assertion would see.
"""

from __future__ import annotations

import ast
import inspect
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.preparation.vehicle_dynamic import prepare_vehicle_run
from suspension_multibody.simulation import SimulationRequest, run_request
from suspension_multibody.subsystems import torque_elements
from suspension_multibody.subsystems.vehicle_assembly import compose_vehicle_runtime

_FIXTURE = Path(__file__).resolve().parents[1] / "vehicle" / "test_native_vehicle.py"


def _fixture():
    import importlib.util

    spec = importlib.util.spec_from_file_location("native_vehicle_fixture", _FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def fixture():
    return _fixture()


def _with_demand(model, demand: str, **driveline):
    """Return ``model`` with a driver-demand declaration stated."""
    return model.model_copy(
        update={
            "driveline": model.driveline.model_copy(
                update={"torque_demand": demand, **driveline}
            )
        }
    )


def _torques(runtime):
    """Return the torque elements a runtime carries, in assembly order."""
    return tuple(
        element
        for element in runtime.elements
        if type(element).__name__ == "RotationalTorqueElement"
    )


def test_the_default_model_gets_no_torque_element(fixture) -> None:
    """
    A model that declares nothing is the vehicle every result was recorded from.

    This is the additive property the whole subtask hangs on: the elements are
    opt-in, so the default path's element list has to be *exactly* what the
    assembly produced before this module existed -- not "an empty element of the
    new kind", which would still be a different model.
    """
    model = fixture._positioned_vehicle(fixture._vehicle())
    assert model.driveline.torque_demand == "none"
    assert _torques(compose_vehicle_runtime(model, mode="K")) == ()


def test_a_declared_brake_demand_gives_one_element_per_braked_wheel(
    fixture,
) -> None:
    """
    One element per declared wheel end, and the count comes from the declaration.

    The count is read against the model's own braked wheels rather than a
    literal, so a template that states one braked corner is a different vehicle
    from one that states four -- and the test says so instead of asserting a
    number that happens to match this fixture.
    """
    model = _with_demand(fixture._positioned_vehicle(fixture._vehicle()), "brake")
    torques = _torques(compose_vehicle_runtime(model, mode="K"))
    braked = tuple(wheel.name for wheel in model.wheels if wheel.braked)
    assert len(braked) == 4
    assert len(torques) == len(braked)
    # Each element's driven end is the wheel end the assembly built for that
    # wheel, and the two ends are different bodies: a couple against itself is
    # no couple at all.
    runtime = compose_vehicle_runtime(model, mode="K")
    for torque in torques:
        assert torque.body_a != torque.body_b
        assert torque.body_b == runtime.wheel_body_names[torque.name.removeprefix("brake_")]


def test_a_declared_drive_demand_gives_one_element_per_driven_wheel(
    fixture,
) -> None:
    """The drive side is the model's `driven_wheels`, not "all four"."""
    base = fixture._positioned_vehicle(fixture._vehicle())
    model = _with_demand(
        base,
        "drive",
        driven_wheels=("rear_left", "rear_right"),
        drive_split=(0.0, 0.0, 0.5, 0.5),
    )
    torques = _torques(compose_vehicle_runtime(model, mode="K"))
    assert {torque.name for torque in torques} == {"drive_rear_left", "drive_rear_right"}


def test_both_declarations_together_are_two_channels(fixture) -> None:
    """
    A driven *and* braked wheel is two elements, not a conflict.

    The kernel refuses a case that states one wheel's channel in both unit
    systems; it does not refuse a wheel that is both braked and driven, because
    those are different channels.  Getting this wrong would make every
    four-wheel-drive car unbrakable.
    """
    base = fixture._positioned_vehicle(fixture._vehicle())
    model = _with_demand(
        base,
        "both",
        driven_wheels=("rear_left", "rear_right"),
        drive_split=(0.0, 0.0, 0.5, 0.5),
    )
    torques = _torques(compose_vehicle_runtime(model, mode="K"))
    assert len(torques) == 6
    assert {torque.name for torque in torques if torque.name.startswith("brake_")} == {
        "brake_front_left",
        "brake_front_right",
        "brake_rear_left",
        "brake_rear_right",
    }


def test_the_two_ends_are_resolved_by_a_port_match_not_by_a_name() -> None:
    """
    Which member reacts the couple is the pairing's answer, not this module's.

    Asserted structurally: the module that decides the two bodies must not
    contain a body-name literal, because a literal that happens to be right for
    one topology is a guess for every other one.  The AST walk is used rather
    than a substring search so the module's prose may still *explain* the rule.
    """
    tree = ast.parse(inspect.getsource(torque_elements))
    literals = {
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    # `wheel_centers`/`wheel_body_names`, the two tables the assembly filled,
    # are the only sources of a body; the names this module does spell are roles
    # and demand keys.
    assert "upright_L" not in literals
    assert "upright_R" not in literals
    assert "chassis" not in literals
    assert "wheel_front_left" not in literals


def test_the_gain_is_converted_once_to_the_kernel_units(fixture) -> None:
    """
    The template's slots are engineering units; the kernel's couple is SI.

    The standardized brake slots give an amplitude of 29 000 N*mm at full demand
    with the whole demand on one wheel
    (2 * piston_area 2500 * demand 1 * the ``0.1`` demand scale * mu 0.4 *
    effective_radius 145 mm), and the front wheel's recorded 0.6 bias split over
    the two braked front wheels makes that 0.3 of it: 8 700 N*mm, i.e. 8.7 N*m.
    The number is derived here rather than read back from the code, so a double
    conversion (1000x too large) and a missing one (1000x too small) both fail.
    """
    from suspension_multibody.subsystems.brake import brake_amplitude
    from suspension_multibody.templates.builtin import BRAKE

    slots = {slot.name: float(slot.default) for slot in BRAKE.property_slots}
    amplitude_n_mm = brake_amplitude(slots, demand=1.0, share=1.0)
    assert amplitude_n_mm == pytest.approx(29_000.0)
    share = 0.6 / 2
    expected_n_m = amplitude_n_mm * share * torque_elements.MM_TO_M
    assert expected_n_m == pytest.approx(8.7)

    model = _with_demand(fixture._positioned_vehicle(fixture._vehicle()), "brake")
    prepared = prepare_vehicle_run(model, fixture._case(model))
    front = next(
        torque
        for torque in prepared.native_model.rotational_torques
        if torque.name == "brake_front_left"
    )
    assert front.stiffness_n_m_per_rad == pytest.approx(expected_n_m)
    assert front.max_torque_n_m == pytest.approx(expected_n_m)
    # The demand channel rides on the element: the brake column, and this wheel's
    # own tire index.  A source without a tire is refused by the kernel reader, so
    # the pair has to be there.
    assert front.demand_source == torque_elements.BRAKE_SOURCE
    assert front.demand_tire == 0


def test_the_document_carries_the_couple_and_the_normalized_demand(fixture) -> None:
    """
    The production route: the couple leaves as an element, the demand as a table.

    A wheel is described in one unit system, and the assertion says which: the
    braked wheels have a `brake_pressure` table and no `brake_torque` table,
    while the unbraked ones are the other way round.  Both halves are needed --
    finding the new role would also pass for a document that sent *both*.
    """
    from suspension_multibody.cases.vehicle_dynamic import case_document, model_document

    model = _with_demand(fixture._positioned_vehicle(fixture._vehicle()), "brake")
    prepared = prepare_vehicle_run(model, fixture._case(model, brake=0.4))
    document, _ = model_document(model, prepared)
    types = [element["type"] for element in document["elements"]]
    assert types.count("rotational_torque") == 4
    couple = next(
        element
        for element in document["elements"]
        if element["type"] == "rotational_torque"
    )
    assert set(couple["parameters"]) >= {"axis_a", "stiffness", "max_torque"}
    assert couple["demand_source"] == torque_elements.BRAKE_SOURCE

    _case, _blob = case_document(model, fixture._case(model, brake=0.4), prepared)
    roles = sorted(entry["role"] for entry in _case["blobs"])
    assert "brake_pressure" in roles
    assert "brake_torque" not in roles
    # The steering and road tables are untouched by this subtask.
    assert "steering_target" in roles


def test_a_document_that_declares_nothing_is_unchanged(fixture) -> None:
    """
    The control: the default model's documents carry the tables they always did.

    Without this, "the new role appears" could be satisfied by a document that
    always sends it -- which would make every recorded case a different case.
    """
    from suspension_multibody.cases.vehicle_dynamic import case_document, model_document

    model = fixture._positioned_vehicle(fixture._vehicle())
    prepared = prepare_vehicle_run(model, fixture._case(model, brake=0.4))
    document, _ = model_document(model, prepared)
    assert "rotational_torque" not in {e["type"] for e in document["elements"]}
    _case, _blob = case_document(model, fixture._case(model, brake=0.4), prepared)
    roles = {entry["role"] for entry in _case["blobs"]}
    assert "brake_torque" in roles
    assert "brake_pressure" not in roles
    assert "throttle_demand" not in roles


def test_the_couple_reaches_the_solver_as_an_element(fixture, monkeypatch) -> None:
    """
    One whole vehicle, through the production entry point, with the couple read.

    The observable is the kernel's own element-wrench channel, because the two
    product result blocks do **not** separate this element from the rest: the
    couple acts between a wheel and the carrier it turns in, and the two are
    joined by a revolute joint whose axis is the couple's own -- so the joint
    reacts the couple exactly and the *state* trajectory is unchanged.  Reading
    `body_state` here would therefore be a test that passes for an element the
    kernel never evaluated.

    What is asserted instead is the record the law itself writes: the channel
    carries type code 10 (the rotational actuator), one pair of rows per element
    per sample, and the pair is equal and opposite in the moment columns.  The
    control -- the same model without the declaration -- has no code-10 row at
    all, so "the element was evaluated" is what the difference means.

    The magnitude is asserted too, and it is *zero* on purpose.  This fixture's
    pair starts with no relative angular rate, which is the state the law's own
    comment describes: a stopped pair gets no couple rather than a full-strength
    one of arbitrary sign.  A run whose couple were non-zero here would be the
    bug the p2-08 slip branch was written to avoid.
    """
    from suspension_multibody.results.element_wrench import (
        ELEMENT_WRENCH_SWITCH,
        element_wrench_block,
    )

    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "1")
    base = fixture._positioned_vehicle(fixture._vehicle())
    guarded = _with_demand(base, "brake")

    def wrench(model):
        case = fixture._case(model, brake=1.0)
        prepared = prepare_vehicle_run(model, case)
        raw = run_request(
            SimulationRequest(
                assembly="vehicle",
                family="vehicle_dynamic",
                model=model,
                case=case,
                context={"prepared": prepared},
            )
        ).raw
        return raw, element_wrench_block(raw)

    _raw, guarded_block = wrench(guarded)
    _control_raw, control_block = wrench(base)
    assert guarded_block is not None, "the element-wrench channel produced no block"

    def code10(block):
        # Column 6 is the channel's frozen type code; the force/moment columns
        # are 0:3 and 3:6, and a row an element applied nothing to stays NaN.
        mask = block[:, :, 6] == 10
        return block[mask]

    assert code10(control_block).shape[0] == 0, (
        "the control run carries rotational-actuator rows, so the declaration "
        "was not what produced them"
    )
    rows = code10(guarded_block)
    # Four elements, two ends each: the count is the element's own two-row
    # shape times the model's own declaration, not a number this test invented.
    samples = guarded_block.shape[0]
    expected = 4 * 2 * samples
    assert rows.shape[0] == expected, (rows.shape, samples)
    # The two ends of one couple are equal and opposite.  The rows are grouped
    # in element order, so each consecutive pair is one element's.
    for index in range(0, expected, 2):
        first = rows[index, 3:6]
        second = rows[index + 1, 3:6]
        np.testing.assert_allclose(first, -np.asarray(second), atol=0.0)
    # And with no relative rate at the start, the demanded magnitude is not
    # applied: the pair is not accelerated by the sign of a rounding error.
    np.testing.assert_allclose(np.asarray(rows[:, 3:6], dtype=float), 0.0)
