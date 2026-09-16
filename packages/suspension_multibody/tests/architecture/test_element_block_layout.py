"""
The generic element-block layout table must describe one layout per family.

`AxleInput` and `VehicleInput` share this table so a `kind` cannot mean two
different things on the two entry points.  Two invariants make that true, and both
are checkable from the header text without building anything:

* the per-family parameter runs must not overlap, or two families would silently
  share a slot and a reader of one would pick up the other's value;
* every referenced slot must be inside the uniform block, or a family's own
  declared layout would read past the end of the element.

The parser is deliberately literal.  It reads the `enum ElementParameter` values
and the `kElementLayouts` rows as written, because the point is to check the
numbers the compiler is given, not to re-derive them from the names.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).parents[4]
HEADER = (
    ROOT / "packages" / "suspension_kernel" / "cpp" / "axle_dynamics" / "axle_kernel.hpp"
).read_text(encoding="utf-8")

#: Family prefix -> the `ElementKind` enumerator that selects it.
FAMILIES = {
    "SPRING": "ELEMENT_SPRING",
    "BUSHING": "ELEMENT_BUSHING",
    "ANTI_ROLL": "ELEMENT_ANTI_ROLL",
    "TIRE": "ELEMENT_TIRE",
    "AERODYNAMIC_DRAG": "ELEMENT_AERODYNAMIC_DRAG",
}

#: The slot run each family declares, as `(first, last_exclusive)`.
#:
#: Written out rather than inferred from the enumerators because the point of the
#: assertion below is that the *declared* runs do not overlap: a derivation from
#: the same enumerators would agree with itself no matter how the runs were placed.
#: These are the runs the C++ `static_assert`s pin, and this repeats the check on
#: the numbers a reader sees.
FAMILY_RUNS = {
    "SPRING": (0, 16),
    "BUSHING": (16, 144),
    "ANTI_ROLL": (144, 154),
    "TIRE": (154, 168),
    "AERODYNAMIC_DRAG": (168, 176),}


def _integer_constants() -> dict[str, int]:
    """Return every `inline constexpr std::size_t kFoo = N;` in the header."""
    return {
        name: int(value)
        for name, value in re.findall(
            r"inline constexpr std::size_t (\w+) = (\d+);", HEADER
        )
    }


def _parameter_indices() -> dict[str, int]:
    """Return every `ELEMENT_<FAMILY>_<ROLE> = N` enumerator value."""
    body = re.search(
        r"enum ElementParameter \{(.*?)\n\};", HEADER, re.S
    )
    assert body is not None, "ElementParameter enum not found"
    return {
        name: int(value)
        for name, value in re.findall(r"(ELEMENT_\w+) = (\d+)", body.group(1))
    }


def _layout_rows() -> dict[str, tuple[int, int]]:
    """
    Return, per family, (parameter start index, curve slot count).

    Each row is `{ELEMENT_X, first_param, ... , curve_slots, int_count}`, so the
    first field is the family and the second is the parameter run's start.
    """
    body = re.search(r"inline constexpr ElementLayout kElementLayouts\[\] = \{(.*?)\n\};", HEADER, re.S)
    assert body is not None, "kElementLayouts not found"
    rows: dict[str, tuple[int, int]] = {}
    for row in re.findall(r"\{(ELEMENT_\w+),(.*?)\}", body.group(1), re.S):
        family, rest = row
        match = re.search(r"curve_slots=\*/(\d+)", rest)
        assert match is not None, f"no curve_slots in the row for {family}"
        curve_slots = int(match.group(1))
        # The parameter run starts at the family's lowest declared index.
        starts = [
            value
            for name, value in _parameter_indices().items()
            if name.startswith(f"{family}_")
        ]
        assert starts, f"no parameter indices declared for {family}"
        rows[family] = (min(starts), curve_slots)
    return rows


def test_every_family_has_a_layout_row() -> None:
    rows = _layout_rows()
    for family in FAMILIES.values():
        assert family in rows, f"{family} has no row in kElementLayouts"


def test_family_parameter_runs_do_not_overlap() -> None:
    """
    Two families sharing a slot is the failure this table exists to prevent.

    The check uses the declared runs rather than the layout rows, so it fails on an
    overlapping *declaration* even when the table happens to be self-consistent.
    It also checks the declaration against the enumerators: every declared index in
    a family must fall inside that family's run, which is what catches a new
    enumerator placed on top of a neighbour.
    """
    occupied: dict[int, str] = {}
    for prefix, (first, last) in FAMILY_RUNS.items():
        assert first < last, f"{prefix} declares an empty run"
        for slot in range(first, last):
            previous = occupied.setdefault(slot, prefix)
            assert previous == prefix, (
                f"parameter slot {slot} is used by both {previous} and {prefix}"
            )

        indices = [
            value
            for name, value in _parameter_indices().items()
            if name.startswith(f"ELEMENT_{prefix}_")
        ]
        assert indices, f"no parameter indices declared for {prefix}"
        outside = sorted(value for value in indices if not first <= value < last)
        assert not outside, (
            f"{prefix} declares indices outside its run [{first}, {last}): {outside}"
        )


def test_every_parameter_index_is_inside_the_uniform_block() -> None:
    constants = _integer_constants()
    size = constants["kElementBlockSize"]
    for name, value in _parameter_indices().items():
        assert 0 <= value < size, f"{name} = {value} is outside [0, {size})"


def test_the_largest_family_fits_the_block() -> None:
    """A bushing is the widest family, so it fixes the block size."""
    constants = _integer_constants()
    indices = _parameter_indices()
    # The bushing's last slot is its reference quaternion, four doubles wide.
    last = indices["ELEMENT_BUSHING_REFERENCE_QUATERNION"] + 4
    assert last <= constants["kElementBlockSize"]
    # And nothing may exceed it either: the highest declared index belongs to the
    # family that ends last.
    assert max(indices.values()) < constants["kElementBlockSize"]


def test_the_ctypes_element_block_matches_the_kernel_width() -> None:
    """
    The Python mirror must be exactly as wide as the C structure.

    A narrower mirror cannot address the later families' slots at all: the failure
    is an `IndexError` from the mirror rather than a kernel complaint, which reads
    as a test bug rather than an ABI drift.  That happened while adding the
    aerodynamic family, so the width is pinned here.
    """
    import ctypes

    from suspension_multibody.axle_dynamics import native

    constants = _integer_constants()
    # The field descriptor is not sized; an instance is what has a length, and
    # ctypes' `.size` on the descriptor is in bytes rather than elements.
    probe = native._ElementBlock()
    assert len(probe.parameters) == constants["kElementBlockSize"]
    assert len(probe.ints) == constants["kElementIntBlockSize"]
    # A spot check that the mirror can actually reach the last declared slot.
    indices = _parameter_indices()
    highest = max(
        value
        for name, value in indices.items()
        if name == "ELEMENT_AERODYNAMIC_DRAG_COEFFICIENT"
    )
    block = native._ElementBlock()
    block.parameters[highest] = 1.0
    assert block.parameters[highest] == 1.0
    assert ctypes.sizeof(native._ElementBlock) > highest * 8


def test_the_integer_block_is_large_enough() -> None:
    constants = _integer_constants()
    body = re.search(r"enum ElementInteger \{(.*?)\n\};", HEADER, re.S)
    assert body is not None, "ElementInteger enum not found"
    highest = max(
        int(value) for value in re.findall(r"ELEMENT_INT_\w+ = (\d+)", body.group(1))
    )
    assert highest < constants["kElementIntBlockSize"]


def test_curve_slots_fit_the_shared_slot_count() -> None:
    constants = _integer_constants()
    for family, (_, curve_slots) in _layout_rows().items():
        assert curve_slots <= constants["kElementCurveSlots"], (
            f"{family} declares {curve_slots} curve slots but the shared slot "
            f"count is {constants['kElementCurveSlots']}"
        )
