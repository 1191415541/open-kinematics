"""
Merging an assembly's declarations with a rig's.

A run is one assembly plus one rig, and the two declare their outputs without
seeing each other.  The merge is where that is made safe: two sides describing the
same thing identically is one output, and two sides describing it differently is
an error naming the field that disagrees -- not a list containing both a
millimetre and an inch.
"""

from __future__ import annotations

import pytest

from suspension_multibody.outputs import (
    DeclarationError,
    DeclarationSet,
    OutputDeclaration,
    merge_declarations,
)


def _output(
    name: str,
    *,
    domain: str,
    unit: str = "N",
    dimension: str = "force",
    shape: tuple[int | None, ...] = (),
) -> OutputDeclaration:
    return OutputDeclaration(
        name=name, unit=unit, dimension=dimension, domain=domain, shape=shape
    )


def _assembly(*outputs: OutputDeclaration) -> DeclarationSet:
    return DeclarationSet(domain="assembly", outputs=outputs)


def _rig(*outputs: OutputDeclaration) -> DeclarationSet:
    return DeclarationSet(domain="rig", outputs=outputs)


def test_the_same_output_from_both_sides_becomes_one_entry() -> None:
    merged = merge_declarations(
        _assembly(_output("wheel_load_front_left", domain="assembly")),
        _rig(_output("wheel_load_front_left", domain="rig")),
    )
    assert merged.names() == ("wheel_load_front_left",)
    # The member keeps the domain that declared it -- that is the evidence of
    # who owns what -- while the set says it is a merge.
    assert merged.get("wheel_load_front_left").domain == "assembly"
    assert merged.domain == "merged"


def test_merge_preserves_the_order_of_first_appearance() -> None:
    merged = merge_declarations(
        _assembly(_output("a_first", domain="assembly"), _output("b_second", domain="assembly")),
        _rig(_output("c_third", domain="rig"), _output("b_second", domain="rig")),
    )
    assert merged.names() == ("a_first", "b_second", "c_third")


def test_a_unit_conflict_names_the_field_and_both_values() -> None:
    with pytest.raises(DeclarationError) as error:
        merge_declarations(
            _assembly(_output("wheel_load_front_left", domain="assembly", unit="N")),
            _rig(_output("wheel_load_front_left", domain="rig", unit="lbf")),
        )
    message = str(error.value)
    assert "unit" in message
    assert "'N'" in message and "'lbf'" in message
    assert "assembly" in message and "rig" in message


def test_a_dimension_conflict_names_the_field_and_both_values() -> None:
    with pytest.raises(DeclarationError) as error:
        merge_declarations(
            _assembly(_output("shared", domain="assembly", dimension="force")),
            _rig(_output("shared", domain="rig", dimension="mass")),
        )
    message = str(error.value)
    assert "dimension" in message
    assert "'force'" in message and "'mass'" in message


def test_a_shape_conflict_names_the_field_and_both_values() -> None:
    with pytest.raises(DeclarationError) as error:
        merge_declarations(
            _assembly(_output("shared", domain="assembly", shape=(None,))),
            _rig(_output("shared", domain="rig", shape=(None, None))),
        )
    message = str(error.value)
    assert "shape" in message
    assert "(None,)" in message and "(None, None)" in message


def test_a_derived_output_cannot_share_a_name_with_a_produced_one() -> None:
    with pytest.raises(DeclarationError, match="one is computed"):
        merge_declarations(
            _assembly(_output("shared", domain="assembly")),
            DeclarationSet(
                domain="derived",
                outputs=(
                    OutputDeclaration(
                        name="shared",
                        unit="N",
                        dimension="force",
                        domain="derived",
                    ),
                ),
            ),
        )


def test_merging_nothing_is_refused_rather_than_returned_empty() -> None:
    with pytest.raises(DeclarationError, match="at least one set"):
        merge_declarations()


def test_a_merged_set_survives_a_json_round_trip() -> None:
    merged = merge_declarations(
        _assembly(_output("a", domain="assembly")),
        _rig(_output("b", domain="rig")),
    )
    assert DeclarationSet.from_json(merged.to_json()) == merged
