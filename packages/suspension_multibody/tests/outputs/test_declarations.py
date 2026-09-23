"""
Output declarations: what a run says it produces, before anything computes it.

The declarations are the contract between two sides that never see each other --
an assembly author and a rig author.  These tests pin the two properties that
make that contract usable: a declaration survives a JSON round trip unchanged,
and a malformed one is refused where it is written rather than when a run fails.
"""

from __future__ import annotations

import pytest

from suspension_multibody.outputs import (
    DOMAINS,
    DeclarationError,
    DeclarationSet,
    OutputDeclaration,
)


def _declaration(**overrides: object) -> OutputDeclaration:
    fields: dict[str, object] = {
        "name": "tire_normal_force",
        "unit": "N",
        "dimension": "force",
        "domain": "assembly",
        "shape": (None, None),
        "description": "per-sample normal force",
    }
    fields.update(overrides)
    return OutputDeclaration(**fields)  # ty: ignore[invalid-argument-type]


def test_a_declaration_round_trips_through_json_unchanged() -> None:
    declaration = _declaration()
    assert OutputDeclaration.from_json(declaration.to_json()) == declaration


def test_a_shape_may_leave_an_axis_length_to_the_run() -> None:
    declaration = _declaration(shape=(3, None))
    restored = OutputDeclaration.from_json(declaration.to_json())
    assert restored.shape == (3, None)


def test_the_three_domains_are_the_declared_ones() -> None:
    assert set(DOMAINS) == {"assembly", "rig", "derived", "merged"}


def test_a_declaration_set_round_trips_through_json_unchanged() -> None:
    declaration_set = DeclarationSet(
        domain="rig",
        outputs=(_declaration(domain="rig"), _declaration(name="rack_drive", domain="rig")),
    )
    restored = DeclarationSet.loads(declaration_set.dumps())
    assert restored == declaration_set
    assert restored.names() == ("tire_normal_force", "rack_drive")


def test_a_declaration_must_carry_a_name_unit_and_dimension() -> None:
    with pytest.raises(DeclarationError, match="must carry a name"):
        _declaration(name="")
    with pytest.raises(DeclarationError, match="must declare a unit"):
        _declaration(unit="")
    with pytest.raises(DeclarationError, match="physical dimension"):
        _declaration(dimension="")


def test_an_unknown_domain_is_refused_and_names_the_known_ones() -> None:
    with pytest.raises(DeclarationError, match="unknown domain"):
        _declaration(domain="wishful")


def test_a_negative_axis_length_is_refused() -> None:
    with pytest.raises(DeclarationError, match="negative axis length"):
        _declaration(shape=(-1,))


def test_a_set_refuses_an_output_from_another_domain() -> None:
    with pytest.raises(DeclarationError, match="declared under another domain"):
        DeclarationSet(domain="rig", outputs=(_declaration(domain="assembly"),))


def test_a_set_refuses_a_duplicate_name() -> None:
    with pytest.raises(DeclarationError, match="more than once"):
        DeclarationSet(
            domain="assembly", outputs=(_declaration(), _declaration())
        )


def test_getting_an_undeclared_name_names_what_is_declared() -> None:
    declaration_set = DeclarationSet(domain="assembly", outputs=(_declaration(),))
    with pytest.raises(DeclarationError, match="tire_normal_force"):
        declaration_set.get("no_such_output")
