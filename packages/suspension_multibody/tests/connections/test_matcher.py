"""
Port matching: explicit first, then filtered, ambiguity refused rather than guessed.
"""

from __future__ import annotations

import pytest

from suspension_multibody.connections import (
    AmbiguousBindingError,
    BindingError,
    UnmetRequirementError,
    match_requirements,
)
from suspension_multibody.modeling import EntityId, GeometryPort, PortRequirement


def _port(
    name: str,
    role: str,
    *,
    capabilities: frozenset[str] = frozenset(),
    labels: frozenset[str] = frozenset(),
) -> GeometryPort:
    return GeometryPort(
        id=EntityId(("rig",), name),
        owner=EntityId(("rig",), "carrier"),
        role=role,
        capabilities=capabilities,
        labels=labels,
    )


def _candidates(*ports: GeometryPort) -> dict[str, GeometryPort]:
    return {str(port.id): port for port in ports}


def test_a_single_candidate_is_matched_by_role() -> None:
    wheel = _port("wheel_centre", "wheel_centre")
    report = match_requirements(
        [PortRequirement(role="wheel_centre")], _candidates(wheel)
    )
    assert report.binding_for("wheel_centre").port_ids == (str(wheel.id),)


def test_an_explicit_mapping_wins_over_inference() -> None:
    """A user who knows the model should not have to argue with a heuristic."""
    left = _port("wheel_centre_L", "wheel_centre", labels=frozenset({"L"}))
    right = _port("wheel_centre_R", "wheel_centre", labels=frozenset({"R"}))
    candidates = _candidates(left, right)
    report = match_requirements(
        [PortRequirement(role="wheel_centre")],
        candidates,
        explicit={"wheel_centre": str(right.id)},
    )
    binding = report.binding_for("wheel_centre")
    assert binding.port_ids == (str(right.id),)
    assert binding.explicit is True


def test_an_explicit_mapping_to_a_missing_port_is_an_error() -> None:
    """Silently falling back to inference would hide the wrong mapping."""
    wheel = _port("wheel_centre", "wheel_centre")
    with pytest.raises(BindingError, match="does not exist"):
        match_requirements(
            [PortRequirement(role="wheel_centre")],
            _candidates(wheel),
            explicit={"wheel_centre": "rig/nowhere"},
        )


def test_an_explicit_mapping_for_an_undeclared_requirement_is_an_error() -> None:
    wheel = _port("wheel_centre", "wheel_centre")
    with pytest.raises(BindingError, match="does not declare"):
        match_requirements(
            [PortRequirement(role="wheel_centre")],
            _candidates(wheel),
            explicit={"rack_input": str(wheel.id)},
        )


def test_two_candidates_are_an_ambiguity_not_a_choice() -> None:
    """
    The central refusal: picking by name similarity or distance is what produced
    rigs that bind to whatever happens to be there.
    """
    first = _port("mount_a", "mount")
    second = _port("mount_b", "mount")
    with pytest.raises(AmbiguousBindingError) as error:
        match_requirements([PortRequirement(role="mount")], _candidates(first, second))
    assert str(first.id) in error.value.candidates
    assert str(second.id) in error.value.candidates


def test_labels_break_the_tie_only_when_the_requirement_names_them() -> None:
    """With labels named, L meets L and nothing else; without them it is ambiguous."""
    left = _port("c_L", "c", labels=frozenset({"L"}))
    right = _port("c_R", "c", labels=frozenset({"R"}))
    report = match_requirements(
        [PortRequirement(role="c", match_labels=frozenset({"L"}))],
        _candidates(left, right),
    )
    assert report.binding_for("c").port_ids == (str(left.id),)


def test_capabilities_filter_candidates() -> None:
    plain = _port("plain", "mount")
    braced = _port("braced", "mount", capabilities=frozenset({"load"}))
    report = match_requirements(
        [PortRequirement(role="mount", requires_capabilities=frozenset({"load"}))],
        _candidates(plain, braced),
    )
    assert report.binding_for("mount").port_ids == (str(braced.id),)


def test_a_required_requirement_with_no_candidate_is_reported_by_name() -> None:
    wheel = _port("wheel_centre", "wheel_centre")
    with pytest.raises(UnmetRequirementError, match="required port 'rack_input'"):
        match_requirements([PortRequirement(role="rack_input")], _candidates(wheel))


def test_an_optional_requirement_disappears_and_takes_its_outputs() -> None:
    """A shrink must not leave an output pointing at a branch that is gone."""
    wheel = _port("wheel_centre", "wheel_centre")
    report = match_requirements(
        [
            PortRequirement(
                role="rack_input",
                required=False,
                bound_outputs=("rack_travel", "rack_force"),
            )
        ],
        _candidates(wheel),
    )
    assert report.disappeared == ("rack_input",)
    assert set(report.dropped_outputs) == {"rack_travel", "rack_force"}
    assert report.binding_for("rack_input") is None


def test_an_optional_requirement_that_finds_a_port_keeps_its_outputs() -> None:
    rack = _port("rack_input", "rack_input")
    report = match_requirements(
        [PortRequirement(role="rack_input", required=False, bound_outputs=("rack_travel",))],
        _candidates(rack),
    )
    assert report.disappeared == ()
    assert report.dropped_outputs == ()
    assert report.binding_for("rack_input") is not None


def test_the_trace_records_how_each_decision_was_made() -> None:
    """A binding has to be auditable afterwards, not re-derived."""
    wheel = _port("wheel_centre", "wheel_centre")
    report = match_requirements(
        [PortRequirement(role="wheel_centre"), PortRequirement(role="rack_input", required=False, bound_outputs=("r",))],
        _candidates(wheel),
    )
    joined = " | ".join(report.trace)
    assert "explicit" in joined or "inferred" in joined
    assert "optional" in joined


def test_a_count_of_two_selects_two_distinct_ports() -> None:
    first = _port("mount_a", "mount")
    second = _port("mount_b", "mount")
    report = match_requirements(
        [PortRequirement(role="mount", count=2)], _candidates(first, second)
    )
    assert len(report.binding_for("mount").port_ids) == 2


def test_no_requirements_produces_an_empty_report() -> None:
    report = match_requirements([], _candidates(_port("p", "r")))
    assert report.bindings == ()
    assert report.trace == ()
