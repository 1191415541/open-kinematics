"""
Ports are semantic, not name-matched; units have one factor in one place.
"""

from __future__ import annotations

import pytest

from suspension_multibody.modeling import (
    METRES_PER_MILLIMETRE,
    ChannelPort,
    EntityId,
    GeometryPort,
    PortError,
    PortRequirement,
    link_counts_agree,
    to_metres,
)


def _port(role: str = "wheel_centre", **kwargs) -> GeometryPort:
    return GeometryPort(
        id=EntityId(("rig",), role),
        owner=EntityId(("rig",), "carrier"),
        role=role,
        **kwargs,
    )


def test_requirement_matches_on_role() -> None:
    requirement = PortRequirement(role="wheel_centre")
    assert requirement.accepts(_port())
    assert not requirement.accepts(_port("rack_input"))


def test_requirement_matches_on_capabilities_it_needs() -> None:
    requirement = PortRequirement(role="p", requires_capabilities=frozenset({"load"}))
    assert requirement.accepts(_port("p", capabilities=frozenset({"load", "extra"})))
    assert not requirement.accepts(_port("p", capabilities=frozenset({"extra"})))


def test_labels_must_match_exactly_so_left_never_binds_to_right() -> None:
    """A left port meeting a right requirement is the classic silent mismatch."""
    requirement = PortRequirement(role="p", match_labels=frozenset({"L"}))
    assert requirement.accepts(_port("p", labels=frozenset({"L"})))
    assert not requirement.accepts(_port("p", labels=frozenset({"R"})))
    assert not requirement.accepts(_port("p", labels=frozenset({"L", "front"})))


def test_a_port_declared_unconnectable_is_never_accepted() -> None:
    requirement = PortRequirement(role="p")
    assert not requirement.accepts(_port("p", cardinality="none"))


def test_label_check_is_skipped_when_the_requirement_names_none() -> None:
    requirement = PortRequirement(role="p")
    assert requirement.accepts(_port("p", labels=frozenset({"L"})))


def test_an_optional_requirement_is_marked_not_inferred() -> None:
    """Only an explicitly optional branch may disappear wholesale."""
    optional = PortRequirement(role="rack_input", required=False, note="axle has no steering")
    assert optional.required is False
    assert optional.note


def test_optional_branch_records_the_outputs_that_go_with_it() -> None:
    """Otherwise a shrink leaves an output pointing at a branch that is gone."""
    requirement = PortRequirement(
        role="rack_input", required=False, bound_outputs=("rack_travel", "rack_force")
    )
    assert "rack_travel" in requirement.bound_outputs


def test_requirement_needs_a_role() -> None:
    with pytest.raises(PortError, match="non-empty role"):
        PortRequirement(role="")


def test_requirement_count_must_be_positive() -> None:
    with pytest.raises(PortError, match="count must be"):
        PortRequirement(role="p", count=0)


def test_cardinality_gates_the_link_count() -> None:
    single = _port("p", cardinality="one")
    many = _port("p", cardinality="many")
    assert link_counts_agree(single, PortRequirement(role="p", count=1))
    assert not link_counts_agree(single, PortRequirement(role="p", count=2))
    assert link_counts_agree(many, PortRequirement(role="p", count=5))


def test_channel_port_has_no_frame_but_may_carry_a_direction() -> None:
    """A drive input has units and a direction; inventing a pose would be a lie."""
    channel = ChannelPort(
        id=EntityId((), "drive"),
        owner=EntityId((), "drive"),
        role="drive_input",
        units="N*m",
        direction=(0.0, 1.0, 0.0),
    )
    assert channel.kind == "channel"
    assert channel.units == "N*m"


def test_channel_direction_must_be_a_real_vector() -> None:
    with pytest.raises(PortError, match="zero vector"):
        ChannelPort(
            id=EntityId((), "d"),
            owner=EntityId((), "d"),
            role="r",
            direction=(0.0, 0.0, 0.0),
        )


def test_geometry_port_and_channel_port_are_different_kinds() -> None:
    assert _port().kind == "geometry"
    assert _port().kind != ChannelPort(
        id=EntityId((), "d"), owner=EntityId((), "d"), role="r"
    ).kind


def test_unknown_cardinality_is_rejected() -> None:
    with pytest.raises(PortError, match="unknown cardinality"):
        _port("p", cardinality="two")  # type: ignore[arg-type]


def test_millimetre_boundary_has_exactly_one_factor() -> None:
    assert METRES_PER_MILLIMETRE == 1e-3
    assert to_metres(1000.0) == 1.0
    assert to_metres(-500.0) == -0.5
    assert to_metres(0.0) == 0.0
