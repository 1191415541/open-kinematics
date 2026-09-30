"""The pairing section: accepted where it is stated, refused where it is not."""

from __future__ import annotations

import pytest

from suspension_contracts import ContractError, validate_assembly


def _document(pairings: object = None) -> dict[str, object]:
    entry: dict[str, object] = {
        "ref": "front.json",
        "functional_role": "suspension",
        "placement_role": "front",
    }
    if pairings is not None:
        entry["pairings"] = pairings
    return {
        "document": "assembly",
        "schema_version": 1,
        "name": "front_axle",
        "assembly_kind": "suspension_axle",
        "subsystems": [entry],
    }


def test_an_assignment_without_pairings_still_validates() -> None:
    # Optional, and absent means "match by role and capability": every assembly
    # file written before the pairing section has to keep loading.
    validate_assembly(_document())


def test_a_stated_pairing_is_accepted() -> None:
    validate_assembly(
        _document([{"requirement_role": "mount", "port": "mount"}])
    )


def test_a_pairing_without_a_port_is_refused() -> None:
    with pytest.raises(ContractError) as caught:
        validate_assembly(_document([{"requirement_role": "mount"}]))
    assert "port" in str(caught.value)


def test_a_pairing_carrying_an_unknown_field_is_refused() -> None:
    with pytest.raises(ContractError) as caught:
        validate_assembly(
            _document(
                [{"requirement_role": "mount", "port": "mount", "guess": "yes"}]
            )
        )
    assert "guess" in str(caught.value)


def test_an_unknown_field_on_the_assignment_is_still_refused() -> None:
    """The pairing section is an addition, not a hole in the entry level."""
    document = _document([{"requirement_role": "mount", "port": "mount"}])
    entries = document["subsystems"]
    assert isinstance(entries, list)
    entry = entries[0]
    assert isinstance(entry, dict)
    entry["mystery"] = "value"
    with pytest.raises(ContractError) as caught:
        validate_assembly(document)
    assert "mystery" in str(caught.value)
