"""
The joint table is the single home of the joint facts.

Two of them used to be restated in places that could drift apart silently: the
kernel's row counts lived in C++ and nowhere in Python, and the
`constant_velocity -> convel` translation was written out twice.  These tests
pin the Python mirror to the C++ source it mirrors.
"""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from suspension_multibody.joints import (
    COMMON_FIELDS,
    DRIVEN_COORDINATES,
    DRIVEN_KINDS,
    JOINT_KINDS,
    JOINT_ROWS,
    JOINT_TYPES,
    SCHEMA_KINDS,
    JointTableError,
    definition_for,
    document_type,
    document_type_for_schema_kind,
    kernel_row_count,
    required_axis_fields,
    rows_for,
)

ROOT = Path(__file__).parents[4]
REGISTRY = ROOT / "packages/suspension_kernel/cpp/src/contract/contract_registry.cpp"
ENUMS = ROOT / "packages/suspension_kernel/cpp/include/mb_model/enums.hpp"


def _cjpoints_from_registry() -> dict[str, int]:
    """Read the kernel's `kJoints` table out of its source."""
    text = REGISTRY.read_text(encoding="utf-8")
    block = re.search(r"kJoints\[\]\s*=\s*\{(.*?)\n\};", text, re.S)
    assert block is not None, f"kJoints not found in {REGISTRY}"
    return {
        name: int(rows)
        for name, rows in re.findall(r'\{\s*"([a-z_]+)"\s*,\s*(\d+)\s*\}', block.group(1))
    }


def _enum_order() -> list[str]:
    """Read the `AxleConstraintType` order out of the kernel's enum header."""
    text = ENUMS.read_text(encoding="utf-8")
    names = re.findall(r"AXLE_([A-Z_]+)\s*=\s*\d+", text)
    return [name.lower() for name in names]


def test_the_table_holds_eight_joints_and_two_driven_coordinates() -> None:
    assert len(JOINT_TYPES) == 8
    assert len(DRIVEN_COORDINATES) == 2
    assert [d.kernel_name for d in JOINT_TYPES] == [
        "spherical",
        "revolute",
        "fixed",
        "prismatic",
        "universal",
        "cylindrical",
        "inplane",
        "convel",
    ]
    assert [d.kernel_name for d in DRIVEN_COORDINATES] == [
        "driven_translation",
        "driven_rotation",
    ]


def test_row_counts_match_the_kernel_registry() -> None:
    """The Python mirror and `contract_registry.cpp` must agree row for row."""
    kernel = _cjpoints_from_registry()
    mirrored = {name: rows for name, rows in kernel.items() if name in JOINT_ROWS}
    assert mirrored == JOINT_ROWS, (
        "the joint table and the kernel registry disagree; "
        f"kernel={mirrored} python={JOINT_ROWS}"
    )


def test_the_registry_names_exactly_our_twenty_rows_plus_driven() -> None:
    """Ten entries: our eight joints plus the two driven coordinates."""
    kernel = _cjpoints_from_registry()
    assert set(kernel) == set(JOINT_ROWS) | {
        d.kernel_name for d in DRIVEN_COORDINATES
    }
    assert kernel_row_count() == sum(kernel[name] for name in JOINT_ROWS)


def test_the_table_follows_the_kernel_enumeration_order() -> None:
    """
    Row counts are positional in the kernel, so the order is load-bearing.

    `kJointTypeInfo` is indexed by `AxleConstraintType`, so a table listed in a
    different order would still agree on the *set* of counts while attaching them
    to the wrong types.
    """
    order = _enum_order()
    joints = [d.kernel_name for d in JOINT_TYPES]
    assert joints == order[:8], f"table order {joints} != kernel enum order {order[:8]}"
    driven = [d.kernel_name for d in DRIVEN_COORDINATES]
    assert driven == order[8:], f"driven order {driven} != kernel enum order {order[8:]}"


def test_a_tampered_row_count_is_caught() -> None:
    """
    The consistency assertion must fail when the two sides drift.

    This is the negative case for the row-count check: without it, the test above
    could pass by comparing a value against itself.
    """
    kernel = _cjpoints_from_registry()
    tampered = dict(kernel)
    tampered["spherical"] = 999
    mirrored = {name: rows for name, rows in tampered.items() if name in JOINT_ROWS}
    assert mirrored != JOINT_ROWS


def test_driven_coordinates_are_not_in_the_joint_table() -> None:
    """The two namespaces share the document array, not the meaning."""
    joint_names = set(JOINT_ROWS)
    driven_names = {d.kernel_name for d in DRIVEN_COORDINATES}
    assert not (joint_names & driven_names)
    for name in driven_names:
        with pytest.raises(JointTableError):
            rows_for(name)


def test_constant_velocity_documents_as_convel_in_one_place() -> None:
    """Both authoring spellings resolve to the document's `convel`."""
    assert document_type("ConstantVelocityJoint") == "convel"
    assert document_type_for_schema_kind("constant_velocity") == "convel"
    assert definition_for("ConstantVelocityJoint").schema_kind == "constant_velocity"


def test_every_joint_has_a_schema_kind_and_axis_requirement() -> None:
    """Each joint is reachable from both authoring paths."""
    assert len(SCHEMA_KINDS) == 8
    assert set(JOINT_KINDS) == {d.authoring_name for d in JOINT_TYPES}
    # Only the constant-velocity joint carries secondary axes.
    assert required_axis_fields("convel") == (
        "axis_a",
        "axis_b",
        "axis_a_secondary",
        "axis_b_secondary",
    )
    assert required_axis_fields("spherical") == ()
    assert required_axis_fields("fixed") == ()
    # In-plane uses the plane normal only; it has no second axis.
    assert required_axis_fields("inplane") == ("axis_a",)


def test_common_fields_are_the_ones_every_entry_carries() -> None:
    assert COMMON_FIELDS == ("name", "type", "body_a", "body_b", "point_a", "point_b")
    for definition in JOINT_TYPES:
        assert not (set(definition.axis_fields) & set(COMMON_FIELDS))


def test_driven_kinds_cover_both_schema_kinds() -> None:
    assert set(DRIVEN_KINDS) == {"translation", "rotation"}
    assert DRIVEN_KINDS["translation"].kernel_name == "driven_translation"
    assert DRIVEN_KINDS["rotation"].kernel_name == "driven_rotation"


def test_unknown_names_fail_with_a_helpful_message() -> None:
    with pytest.raises(JointTableError, match="unsupported joint"):
        definition_for("NotAJoint")
    with pytest.raises(JointTableError, match="unknown joint type"):
        rows_for("not_a_type")
    with pytest.raises(JointTableError, match="unsupported joint kind"):
        document_type_for_schema_kind("not_a_kind")
