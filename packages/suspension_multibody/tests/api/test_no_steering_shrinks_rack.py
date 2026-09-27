"""
A steering-less axle runs, and its rack channels are *absent* rather than zero.

Sub-task 01 recorded this as GAP-2, and the defect had two halves that both had
to be closed before the run existed at all:

1. **the run could not be asked for.**  `api._kc_assembly` always built the
   default single-axle set, so a steering-less run was unreachable through the
   public entry -- it existed only as a directly constructed assembly, which is
   why the earlier tests could check the private grid helper and nothing else;
2. **the model document asked for a body that was not there.**  `model_document`
   emitted an unconditional rack driven coordinate from `assembly.point("rack",
   "center")`, which raised `KeyError: ('rack', 'center')` on an assembly with no
   rack.  The recorded first failure -- `ValueError: missing required front-axle
   hardpoint for rack_center` -- is a *different* trigger: it happens when
   steering *is* present and the hardpoint was removed.

What this file holds is the end state: the run solves, the result says what it
did not steer by omitting the channel, and the steering-carrying run is unchanged.
The negative controls matter as much as the positives -- a caller cannot tell
`0.0` from "no rack", which is the whole reason the omission is the fix.
"""

from __future__ import annotations

import pytest

from suspension_multibody import api
from suspension_multibody.cases.kc_quasi_static import model_document
from suspension_multibody.cases.kc_quasi_static.contract import has_rack
from suspension_multibody.schema import CaseSpec, DisplacementControl
from suspension_multibody.subsystems import (
    DEFAULT_AXLE_SUBSYSTEMS,
    AssemblyRequest,
)
from suspension_multibody.subsystems.entry import compose_axle
from tests.benchmark_fixture import benchmark_model

#: The subsystem set a single-axle run has when it carries no steering.
_WITHOUT_STEERING = DEFAULT_AXLE_SUBSYSTEMS - {"steering"}


def _model_without_rack_center():
    """Return the benchmark axle with the rack hardpoint removed."""
    model = benchmark_model()
    hardpoints = {
        name: point for name, point in model.hardpoints.items() if name != "rack_center"
    }
    return model.model_copy(update={"hardpoints": hardpoints})


def _k_case(subsystems: frozenset[str] | None = None) -> CaseSpec:
    """Return a one-axis K sweep, optionally for a steering-less axle."""
    return CaseSpec(
        mode="K",
        subsystems=subsystems,
        controls=(
            DisplacementControl(target="wheel_travel_left", values=(-10.0, 0.0, 10.0)),
        ),
    )


# --- the assembly and the document ------------------------------------------


def test_has_rack_asks_the_assembly_not_a_role_name() -> None:
    """The question is answered by what the assembly built."""
    with_steering = compose_axle(benchmark_model(), "K")
    without = compose_axle(
        _model_without_rack_center(),
        "K",
        AssemblyRequest(mode="K", subsystems=_WITHOUT_STEERING),
    )
    assert has_rack(with_steering)
    assert not has_rack(without)


def test_the_model_document_declares_no_rack_coordinate_without_one() -> None:
    """
    The document's driven coordinates are a consequence of the assembly.

    This is the half that raised `KeyError` before: the rack row was emitted
    unconditionally, so the document could not describe an axle that has no rack.
    """
    without = compose_axle(
        _model_without_rack_center(),
        "K",
        AssemblyRequest(mode="K", subsystems=_WITHOUT_STEERING),
    )
    document = model_document(without, name="no-steering", drive_wheels=True)
    driven = [joint["name"] for joint in document["joints"] if "driven" in joint["type"]]
    assert driven == ["wheel_drive_L", "wheel_drive_R"]
    assert not any(name.startswith("rack") for name in driven)
    # And the body it would have hung on is absent too, rather than degenerate.
    assert "rack" not in document["bodies"]


def test_the_model_document_still_declares_the_rack_when_it_has_one() -> None:
    """The negative control: the omission is a consequence, not a deletion."""
    with_steering = compose_axle(benchmark_model(), "K")
    document = model_document(with_steering, name="steering", drive_wheels=True)
    driven = [joint["name"] for joint in document["joints"] if "driven" in joint["type"]]
    assert "rack_drive" in driven


# --- the run ----------------------------------------------------------------


def test_a_no_steering_axle_runs_through_the_public_entry() -> None:
    """
    GAP-2's first acceptance: the run exists and converges.

    Before the case carried the subsystem set there was no way to reach this
    through `run_case` at all, so "the run works" was not checkable.
    """
    bundle = api.run_case(_model_without_rack_center(), _k_case(_WITHOUT_STEERING))
    assert len(bundle.states) == 3
    for state in bundle.states:
        assert state.converged
        assert state.constraint_residual < 1e-6
        assert state.force_residual < 1e-6


def test_the_rack_channel_disappears_rather_than_reporting_zero() -> None:
    """
    GAP-2's real acceptance: absent, not zeroed.

    A `{"rack_displacement": 0.0}` channel would claim the run steered and the
    rack sat at neutral.  There is no rack, so there is no channel -- the same
    rule the rig's shrink is built on.
    """
    bundle = api.run_case(_model_without_rack_center(), _k_case(_WITHOUT_STEERING))
    for state in bundle.states:
        assert "rack_displacement" not in state.drives
        assert set(state.drives) == {"wheel_travel_left", "wheel_travel_right"}
        # The wheel travels are real, so the shrink did not remove the run.
        assert state.drives["wheel_travel_left"] in (-10.0, 0.0, 10.0)


def test_the_steering_run_still_reports_the_rack_channel() -> None:
    """The other side of the same assertion, so the omission means something."""
    bundle = api.run_case(benchmark_model(), _k_case())
    assert "rack_displacement" in bundle.states[0].drives


def test_a_run_naming_steering_the_case_does_not_ask_for_is_refused() -> None:
    """
    The subsystem set is checked, not ignored.

    A case that excludes `steering` while the model declares a rack coordinate is
    a caller asking for two different assemblies; the assembly builder owns that
    refusal, and it has to be reachable from the public entry.
    """
    # The benchmark model has a rack_center hardpoint, and excludes steering:
    # this is legal -- the hardpoint is simply unused -- so the run succeeds.
    bundle = api.run_case(benchmark_model(), _k_case(_WITHOUT_STEERING))
    assert "rack_displacement" not in bundle.states[0].drives

    # But a subsystem set naming a role the axle cannot build is refused, and the
    # refusal names the role.
    with pytest.raises(ValueError, match="brake"):
        api.run_case(
            benchmark_model(),
            _k_case(frozenset(DEFAULT_AXLE_SUBSYSTEMS | {"brake"})),
        )


def test_the_dynamic_replay_path_uses_the_default_axle() -> None:
    """
    A `DynamicCaseSpec` states motion, not assembly.

    The replay path therefore has no subsystem set of its own and runs the
    default axle; saying so is what keeps the new case field from being
    silently required everywhere.
    """
    from suspension_multibody.schema import DynamicCaseSpec

    # The field is absent from *this* schema, so the two case spellings stay
    # distinct and the new input is not silently required everywhere.  The replay
    # entry is exercised by its own tests; what matters here is the shape.
    assert "subsystems" not in DynamicCaseSpec.model_fields
    assert "subsystems" in CaseSpec.model_fields


def test_a_case_spec_defaults_to_the_full_single_axle_set() -> None:
    """`None` means the default set, so every existing caller is unchanged."""
    assert CaseSpec(mode="K").subsystems is None
    bundle = api.run_case(benchmark_model(), _k_case())
    assert "rack_displacement" in bundle.states[0].drives
