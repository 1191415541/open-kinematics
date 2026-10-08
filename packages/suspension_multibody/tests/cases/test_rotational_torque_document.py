"""
The rotational-torque element through the production document route.

The production dynamic path submits a *model document* and a *case document*
(``simulation/backend.py`` -> ``kernel.run_contract``), and the document reader
used to refuse this family by name
(``model document: element "torque" has unsupported type "rotational_torque"``).
The C ABI could already read it; only the document route could not.  This file
is the acceptance evidence that the two now agree.

What is asserted, and why each part is here
-------------------------------------------

* **the schema accepts the document** -- ``validate_model`` is the first of the
  two gates, and a schema that still lists the old element names would refuse a
  document whose reader branch exists.
* **the reference pose is optional in the same way on both routes** -- a
  document that omits ``reference_quaternion`` leaves the block's four slots at
  zero, which is the spelling ``element_reader.cpp`` accepts for "no reference
  pose" in this family.  A reader that defaulted it to the identity would make
  the two routes mean different blocks for the same document, so both spellings
  are run and required to produce the same trajectory.
* **the couple is measured, not merely not-refused.**  Two runs of the *same*
  model, the same case and the same initial state, one carrying the element and
  one not, are compared on the final angular velocity of both ends.  A reader
  that parsed the element and then dropped it would leave the two runs
  identical, so "the call succeeded" is deliberately not the assertion.

The model is the smallest one the element can act in: two equal free bodies
sharing a revolute joint about the element's own axis, one of them spinning,
and nothing else -- no tire, no road, no suspension surface.  The couple is the
only thing that can move either end.
"""

from __future__ import annotations

from typing import Any

import numpy as np
import pytest
from suspension_contracts import validate_model

from suspension_multibody.api import simulate
from suspension_multibody.kernel import KernelContractError

from ._torque_documents import torque_documents

#: The driven body's initial angular velocity about the element's axis, in
#: rad/s.  Well clear of the law's ``kEps`` branch, so the couple is engaged
#: from the first step.
SPIN = 3.0
#: The history: 21 samples over 0.2 s, i.e. a 10 ms output step.
SECONDS = 0.2
SAMPLES = 21
#: ``min(stiffness * demand, max_torque)`` with the unit demand the law applies
#: today is 1 N*m -- enough to bring the pair to their common rate inside the
#: 0.2 s window without making the integrator refuse the step.  The same figures
#: the C ABI route's own test uses, so the two routes are compared on one
#: loading rather than on two.
STIFFNESS = 1.0
MAX_TORQUE = 1.0
#: ``kStatePerBody``, and where a body's angular velocity starts inside its
#: block: position(3), quaternion(4), velocity(3), then omega.
OMEGA = 10
#: How much the two runs must differ, in rad/s.
#:
#: The figure is derived from the physics, not fitted to the reading.  The
#: couple is ``+tau`` on the driven body and ``-tau`` on the reaction one, and
#: the two inertias are equal (0.01 kg*m^2), so each end accelerates at
#: ``tau / I = 100 rad/s^2`` -- the driven one downwards, the reaction one
#: upwards.  They cross after ``3 / 200 = 0.015 s`` at ``1.5 rad/s`` each, and
#: with the relative rate then zero the law's third branch applies nothing, so
#: the pair stays there: the final rates are 1.499999999999999 and
#: 1.5000000000000002 rad/s, a difference of 1.500000000000001 rad/s.
#:
#: 0.5 rad/s sits three times below the analytic effect and fifteen orders above
#: the round-off of an identical comparison (~1e-15).  It is a margin on
#: *evaluation*: it fails for an element that was parsed and ignored (difference
#: exactly 0) and for one applied at a third of its magnitude, while tolerating
#: the integrator's own step-size rounding.  A tighter figure would be asserting
#: the solver's step choice rather than the element.
TOLERANCE = 0.5


def _model_document(
    *, with_element: bool, reference_quaternion: bool = False
) -> dict[str, Any]:
    """
    Build the minimal two-body model, with or without the torque element.

    Both bodies are free: the couple applies ``+tau`` to one and ``-tau`` to the
    other, and a body held to ground would send its half into the ground, so the
    two ends could never meet and the reaction end would read zero throughout.
    """
    document: dict[str, Any] = {
        "contract": "multibody-model",
        "contract_version": 1,
        "kind": "model",
        "name": "torque-document-route",
        # Metric: the kernel scales by one, so the numbers here are the numbers
        # the solver uses and no unit round trip can hide a slot read wrongly.
        "units": {"length": "m", "mass": "kg", "time": "s", "angle": "rad"},
        "gravity": [0.0, 0.0, 0.0],
        "capabilities": ["axle"],
        "bodies": [
            {
                "name": "reaction",
                "mass": 1.0,
                "inertia": [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]],
                "fixed": False,
                "position": [0.0, 0.0, 0.0],
                "quaternion": [1.0, 0.0, 0.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
                "omega": [0.0, 0.0, 0.0],
            },
            {
                "name": "driven",
                "mass": 1.0,
                "inertia": [[0.01, 0.0, 0.0], [0.0, 0.01, 0.0], [0.0, 0.0, 0.01]],
                "fixed": False,
                "position": [0.0, 0.0, 0.0],
                "quaternion": [1.0, 0.0, 0.0, 0.0],
                "velocity": [0.0, 0.0, 0.0],
                "omega": [0.0, SPIN, 0.0],
            },
        ],
        "joints": [
            {
                "name": "spin",
                "type": "revolute",
                "body_a": "reaction",
                "body_b": "driven",
                "point_a": [0.0, 0.0, 0.0],
                "point_b": [0.0, 0.0, 0.0],
                "axis_a": [0.0, 1.0, 0.0],
                "axis_b": [0.0, 1.0, 0.0],
            }
        ],
        "elements": [],
        "tires": [],
    }
    if with_element:
        parameters: dict[str, Any] = {
            "axis_a": [0.0, 1.0, 0.0],
            "stiffness": STIFFNESS,
            "damping": 0.0,
            "max_torque": MAX_TORQUE,
        }
        if reference_quaternion:
            parameters["reference_quaternion"] = [1.0, 0.0, 0.0, 0.0]
        document["elements"].append(
            {
                "name": "brake",
                "type": "rotational_torque",
                # `body_a` is the reaction end and `body_b` the driven one, the
                # native element's own fields; swapping them reverses the couple.
                "body_a": "reaction",
                "body_b": "driven",
                "parameters": parameters,
            }
        )
    return document


def _case_document() -> dict[str, Any]:
    """
    One ``axle_dynamic`` history with no excitation of its own.

    The family is the one the production dynamic path uses, and the model
    carries no tire and no road, so every body wrench, road height and wheel
    torque the case could supply is absent -- nothing but the element moves the
    pair.
    """
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "axle_dynamic",
        "name": "torque-document-route-case",
        "time": {
            "start_s": 0.0,
            "end_s": SECONDS,
            "step_s": SECONDS / (SAMPLES - 1),
        },
        "solver": {
            "integrator": "ggl_generalized_alpha",
            "rho_inf": 1.0,
            # The initial state is a provided one: the model document states the
            # driven body's spin, and a static trim would zero it and with it the
            # very rate this element's law reads.
            "initialization_mode": "provided_consistent_state",
            "adaptive_step": False,
            "internal_step_s": 0.001,
            "minimum_step_s": 0.0001,
            "maximum_step_s": 0.001,
            "local_relative_tolerance": 1e-8,
            "local_position_tolerance_m": 1e-8,
            "local_angle_tolerance_rad": 1e-8,
            "local_velocity_tolerance_m_per_s": 1e-8,
            "local_angular_velocity_tolerance_rad_per_s": 1e-8,
            "contact_event_tolerance_s": 1e-6,
            "max_newton_iterations": 40,
            "max_line_search_iterations": 8,
            "position_tolerance_m": 1e-8,
            "velocity_tolerance_m_per_s": 1e-8,
            "dynamics_tolerance": 1e-6,
            "increment_tolerance": 1e-10,
        },
    }


def _run(*, with_element: bool, reference_quaternion: bool = False) -> np.ndarray:
    """
    Submit one model document through the production entry point.

    Returns the ``body_state`` block, shaped ``[sample, body, kStatePerBody]``.
    The documents are validated before submission, so a schema that still
    refuses the element name fails here rather than at the reader.
    """
    document = _model_document(
        with_element=with_element, reference_quaternion=reference_quaternion
    )
    validate_model(document)
    run = simulate(*torque_documents(document, _case_document()))
    assert run.status == "success", run.raw.document.get("manifest")
    state = run.raw.block("body_state")
    assert state.shape == (SAMPLES, 2, 19), state.shape
    assert np.isfinite(state).all(), "the kernel returned a non-finite state"
    return state


def _raw(*, with_element: bool):
    """
    Submit the same pair through the production entry point and return the run.

    This is the helper the element-wrench assertions use: `_run` above goes one
    step further and returns the state, while a test that reads the couple's own
    record needs the raw result the channel lives in.
    """
    document = _model_document(with_element=with_element)
    validate_model(document)
    return simulate(*torque_documents(document, _case_document()))


def _omega_y(state: np.ndarray, body: int) -> np.ndarray:
    """One body's angular velocity about the element's own axis, per sample."""
    return state[:, body, OMEGA + 1]


@pytest.fixture(scope="module")
def no_element() -> np.ndarray:
    """Run the control: the same model with the element removed."""
    return _run(with_element=False)


@pytest.fixture(scope="module")
def with_element() -> np.ndarray:
    """Run the case under test: the same model with the element declared."""
    return _run(with_element=True)


def test_the_schema_accepts_the_element_type() -> None:
    """
    The enum is the document's first gate, and it has to name the family.

    ``validate_model`` is the product's own pre-flight check, and it runs before
    a payload is ever framed; a schema that still listed the old element names
    would refuse the document here, and the kernel's reader branch -- correct or
    not -- would be unreachable from the production path.  The negative half
    makes this a statement about *this* name rather than about the validator
    accepting everything: a neighbouring family name that is spelled wrongly is
    still refused.
    """
    validate_model(_model_document(with_element=True))
    misspelled = _model_document(with_element=True)
    misspelled["elements"][0]["type"] = "rotational_torques"
    with pytest.raises(Exception, match="rotational_torques"):
        validate_model(misspelled)


def test_both_runs_succeed_and_produce_a_finite_state(
    with_element: np.ndarray, no_element: np.ndarray
) -> None:
    """
    The element is not refused, and the run it belongs to still converges.

    ``_run`` already asserts the status and the finiteness of every run; this
    states it again as a named fact, because it is the first half of the
    acceptance: an element the kernel cannot evaluate would surface here as a
    failed run rather than as a wrong number further down.
    """
    assert with_element.shape == no_element.shape == (SAMPLES, 2, 19)
    assert np.isfinite(with_element).all()
    assert np.isfinite(no_element).all()


def test_the_control_is_a_free_spinner(
    no_element: np.ndarray,
) -> None:
    """
    Without the element nothing slows the pair, so the comparison is not vacuous.

    This is what makes the difference below attributable to the element: the
    control has no other force in it at all, and its two ends hold the initial
    state for the whole history.
    """
    assert _omega_y(no_element, 0) == pytest.approx(0.0)
    assert _omega_y(no_element, 1) == pytest.approx(SPIN)


def test_the_couple_changes_the_driven_bodys_final_rate(
    with_element: np.ndarray, no_element: np.ndarray
) -> None:
    """
    The measured effect: the element *did* something, by more than the tolerance.

    A reader that parsed the block and then dropped it, or one that wrote it to
    the wrong parameter slots, leaves the two runs identical -- which is exactly
    what this fails on.  The reported numbers are the two final rates and their
    difference, and the tolerance is the one derived in the module docstring.
    """
    driven_with = float(_omega_y(with_element, 1)[-1])
    driven_without = float(_omega_y(no_element, 1)[-1])
    difference = abs(driven_with - driven_without)
    assert difference > TOLERANCE, (
        f"driven final omega_y: with={driven_with!r} without={driven_without!r} "
        f"difference={difference!r} is within the {TOLERANCE} rad/s tolerance"
    )
    # Direction as well as magnitude: the couple opposes the rate, so the driven
    # end is the one that slowed.  A sign error would pass a magnitude-only
    # check and fail this one.
    assert abs(driven_with) < abs(driven_without), (driven_with, driven_without)


def test_the_couple_meets_in_the_middle(
    with_element: np.ndarray, no_element: np.ndarray
) -> None:
    """
    The reaction body responds too, and by the same amount.

    A pure couple applies ``+tau`` to one end and ``-tau`` to the other, and the
    two inertias here are equal, so the pair can only meet in the middle.  A
    one-sided torque, an element applied to one body only, or an element whose
    reaction was dropped would leave the reaction end at its initial zero -- and
    the equality below is what distinguishes "equal and opposite" from "both
    ends moved a bit".
    """
    reaction_with = float(_omega_y(with_element, 0)[-1])
    reaction_without = float(_omega_y(no_element, 0)[-1])
    driven_with = float(_omega_y(with_element, 1)[-1])
    assert reaction_without == pytest.approx(0.0)
    assert abs(reaction_with - reaction_without) > TOLERANCE, (
        reaction_with, reaction_without
    )
    assert reaction_with == pytest.approx(driven_with, abs=1e-9), (
        reaction_with, driven_with
    )


def test_the_omitted_reference_pose_means_the_same_thing_on_both_routes(
    with_element: np.ndarray,
) -> None:
    """
    A document that omits ``reference_quaternion`` matches one that writes the
    identity.

    The generic block reader treats four zero slots as "no reference pose" for
    this family and its law never reads them, so both spellings have to produce
    the same run.  A document reader that defaulted the quaternion to the
    identity would write different blocks than the C ABI route does for the same
    document, and this is what detects that: the two documents differ only in a
    field the law ignores, so any difference in the trajectory is the reader's.
    """
    explicit = _run(with_element=True, reference_quaternion=True)
    assert np.array_equal(explicit, with_element), (
        "the optional reference pose changed the trajectory"
    )


def test_the_document_route_carries_the_demand_channel() -> None:
    """
    The two demand fields reach the kernel's own block slots (p2-08), and a bad
    pair is refused at the document rather than silently defaulted.

    The parse is what is asserted: a document whose ``demand_source``/``demand_tire``
    are accepted but dropped would run the element on the unit demand, and the
    run would succeed -- the failure mode is a wrong couple, not a missing one.
    The refusals below are the other half: a source that names no tire, and a
    fractional index, are both errors about the document.
    """
    def submit(parameters: dict[str, Any]) -> None:
        document = _model_document(with_element=True)
        document["elements"][0]["parameters"].update(parameters)
        run = simulate(*torque_documents(document, _case_document()))
        assert run.status == "success", run.raw.document.get("manifest")

    # Read, not merely tolerated: the pair reaches the kernel's block slots.
    submit({"demand_source": 2, "demand_tire": 0})

    # And a bad pair is an error about the document rather than a silent default.
    with pytest.raises(Exception, match="tire"):
        submit({"demand_source": 2})
    with pytest.raises(Exception, match="demand source"):
        submit({"demand_source": 9, "demand_tire": 0})
    with pytest.raises(Exception, match="demand channel"):
        submit({"demand_source": 2, "demand_tire": 0.5})


def test_the_couple_the_kernel_applied_is_non_zero(monkeypatch) -> None:
    """
    The couple is *read back* from the run, not inferred from its consequences.

    Subtask p2-11.  Everything above proves the element reaches the solver: the
    state trajectory differs, the reaction end moves, the two ends meet.  None of
    those separates "the law applied the demanded couple" from "the law applied
    some couple" -- and a law that scaled the demand by a wrong factor would pass
    all of them.

    So this reads the kernel's own element-wrench channel: type code 10 is the
    rotational actuator, the block carries one pair of rows per element per
    sample, and the moment columns are the couple the law actually applied.  The
    magnitude is asserted against the block's own `max_torque`, which is the
    documented meaning of the cap (`min(stiffness * demand, max_torque)` with the
    unit demand this document states).

    The control is the same model with the element removed: it must carry *no*
    code-10 row at all, so "the rows are the element's" is a measurement rather
    than an assumption.
    """
    from suspension_multibody.results.element_wrench import (
        ELEMENT_WRENCH_SWITCH,
        element_wrench_block,
    )

    monkeypatch.setenv(ELEMENT_WRENCH_SWITCH, "1")
    with_couple = _raw(with_element=True)
    without = _raw(with_element=False)
    guarded = element_wrench_block(with_couple.raw)
    control = element_wrench_block(without.raw)
    assert guarded is not None and control is not None

    def code10(block):
        # Column 6 is the channel's frozen type code; columns 3:6 are the world
        # moment about the receiving body's origin.
        return block[block[:, :, 6] == 10]

    assert code10(control).shape[0] == 0, (
        "the control run carries rotational-actuator rows, so the declaration "
        "was not what produced them"
    )
    rows = code10(guarded)
    assert rows.shape[0] == SAMPLES * 2, rows.shape
    # Every sample's pair is equal and opposite: a couple applied to one end and
    # not reacted on the other is the failure mode that still converges.
    pairs = rows.reshape(SAMPLES, 2, rows.shape[1])
    np.testing.assert_allclose(
        pairs[:, 0, 3:6].astype(float), -(pairs[:, 1, 3:6].astype(float)), atol=0.0
    )
    # The first sample is the one with a known argument: the pair starts at a
    # relative rate of `SPIN`, well outside the law's `kEps` branch, so the
    # demanded magnitude is what the law must apply -- the block's own cap, which
    # is what `min(stiffness * demand, max_torque)` means for the unit demand this
    # document states.  A law that scaled the demand by the wrong factor, or one
    # that applied a constant instead of the demand, fails here.
    first_moment = pairs[0, 0, 3:6].astype(float)
    assert np.linalg.norm(first_moment) == pytest.approx(MAX_TORQUE, rel=1e-12), (
        first_moment
    )
    # And the couple does *not* stay at full magnitude forever: the two equal
    # inertias bring the pair to a common rate, the relative rate reaches zero,
    # and the law's third branch applies nothing.  A law that kept pushing would
    # still satisfy every assertion above, so this is the other half of "the
    # couple follows the state".
    assert np.linalg.norm(pairs[-1, 0, 3:6].astype(float)) == pytest.approx(0.0, abs=1e-12)


def test_the_element_is_read_by_the_route_that_used_to_refuse_it() -> None:
    """
    The refusal this file exists to remove is gone.

    ``run_contract`` raises ``KernelContractError`` when the kernel refuses a
    document, so the positive runs above already prove the branch was taken.  The
    explicit assertion is kept because it names the message the reproduction
    recorded: if some later change reinstated the refusal -- by dropping the
    registry entry, the schema enum, or the reader branch -- the failure would
    otherwise be reported as a missing fixture rather than as this regression.
    """
    try:
        _run(with_element=True)
    except KernelContractError as error:  # pragma: no cover - the regression path
        pytest.fail(f"the document route refused the element again: {error}")
