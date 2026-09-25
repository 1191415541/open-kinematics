"""
Entity identity survives a rebuild; it is not an index and not a display name.
"""

from __future__ import annotations

import pytest

from suspension_multibody.modeling import EntityId, qualified


def test_str_round_trips_through_qualified() -> None:
    identifier = EntityId(("front_axle", "L"), "upper_arm")
    assert str(identifier) == "front_axle/L/upper_arm"
    assert qualified("front_axle", "L", "upper_arm") == identifier


def test_root_entity_has_empty_path() -> None:
    identifier = EntityId.root("chassis")
    assert identifier.path == ()
    assert str(identifier) == "chassis"


def test_local_id_must_not_contain_the_separator() -> None:
    """Otherwise a rendered id would parse back to a different identity."""
    with pytest.raises(ValueError, match="must not contain"):
        EntityId(("axle",), "upper/arm")


def test_empty_path_step_is_rejected() -> None:
    with pytest.raises(ValueError, match="empty step"):
        EntityId(("axle", ""), "arm")


def test_empty_local_id_is_rejected() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        EntityId((), "")


def test_path_must_be_a_tuple_so_it_stays_hashable() -> None:
    with pytest.raises(TypeError, match="tuple"):
        EntityId(["axle"], "arm")  # type: ignore[arg-type]


def test_identity_is_orderable_and_hashable() -> None:
    """A set/dict of entities is how ownership is checked; it needs both."""
    first = EntityId(("a",), "one")
    second = EntityId(("a",), "two")
    assert sorted([second, first]) == [first, second]
    assert len({first, first, second}) == 2


def test_child_extends_the_path_and_keeps_the_local_id() -> None:
    parent = EntityId(("vehicle",), "body")
    child = parent.child("front", "upright_L")
    assert child.path == ("vehicle", "front")
    assert child.local == "upright_L"


def test_under_prefixes_without_renaming() -> None:
    identifier = EntityId(("axle",), "wheel")
    assert identifier.under("vehicle", "front") == EntityId(
        ("vehicle", "front", "axle"), "wheel"
    )
    assert identifier.under() is identifier


def test_belongs_to_is_a_path_prefix_test() -> None:
    identifier = EntityId(("vehicle", "front", "axle"), "wheel")
    assert identifier.belongs_to(("vehicle",))
    assert identifier.belongs_to(("vehicle", "front"))
    assert not identifier.belongs_to(("vehicle", "rear"))
    assert identifier.belongs_to(())
