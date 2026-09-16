"""
The `ctypes` mirror of the C ABI structures must list every field, in order.

A mirror that stops short of the real structure is not a harmless omission.  The
kernel checks `struct_size` against its own `sizeof`, so a short mirror is either
rejected outright or -- worse, before that check existed -- read past its end.
Both symptoms are confusing at the call site, so the field lists are compared
here instead.

The comparison is on names, because layout follows from the names and types being
right: a missing field shifts every field after it, which this catches directly.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).parents[4]
HEADER = (
    ROOT / "packages" / "suspension_kernel" / "cpp" / "axle_dynamics" / "axle_kernel.hpp"
).read_text(encoding="utf-8")

#: `ctypes` class name -> the C struct it mirrors.
MIRRORS = {
    "_AxleInput": "AxleInput",
    "_AxleOutput": "AxleOutput",
    "_VehicleInput": "VehicleInput",
    "_VehicleOutput": "VehicleOutput",
}

#: A field declaration inside a struct body: leading whitespace, an optional
#: `const`, a type spelling, and the name.  The type part is deliberately open --
#: an enumerated list of spellings silently skips any type I forget to add, which
#: is how the first version of this test missed the `const ElementBlock*` fields.
_CPP_FIELD = re.compile(
    r"^\s+(?:const\s+)?"
    r"(?:std::)?[A-Za-z_]\w*(?:\s+[A-Za-z_]\w*)*?"
    r"\s*(\*?)\s+(\w+)(\[[^\]]*\])?;",
    re.M,
)


def _cpp_fields(struct: str) -> list[str]:
    body = re.search(rf"struct {struct} \{{(.*?)\n\}};", HEADER, re.S)
    assert body is not None, f"struct {struct} not found in the ABI header"
    return [match.group(2) for match in _CPP_FIELD.finditer(body.group(1))]


def _mirror_fields(class_name: str) -> list[str]:
    from suspension_multibody.axle_dynamics import native

    structure = getattr(native, class_name)
    return [name for name, _ in structure._fields_]


def test_mirrors_are_present_in_the_native_module() -> None:
    from suspension_multibody.axle_dynamics import native

    for class_name in MIRRORS:
        assert hasattr(native, class_name), f"{class_name} is missing"


def test_every_mirror_lists_the_same_fields_in_the_same_order() -> None:
    offenders: list[str] = []
    for class_name, struct in MIRRORS.items():
        expected = _cpp_fields(struct)
        actual = _mirror_fields(class_name)
        if expected == actual:
            continue
        missing = [name for name in expected if name not in actual]
        extra = [name for name in actual if name not in expected]
        if missing or extra:
            offenders.append(
                f"{class_name} vs {struct}: missing={missing} extra={extra}"
            )
        else:
            offenders.append(
                f"{class_name} vs {struct}: same names, different order"
            )
    assert not offenders, "\n".join(offenders)


def test_the_mirror_can_declare_a_size_the_kernel_accepts() -> None:
    """
    A caller has to be able to satisfy the kernel's `struct_size` requirement.

    The kernel rejects an input whose `struct_size` is below its own
    `sizeof(AxleInput)`, so the largest honest claim the mirror can make -- its own
    size -- must be a value a caller can actually set.  The field-list test above
    is what makes that the *right* size; this one only pins that the field exists
    and round-trips.
    """
    import ctypes

    from suspension_multibody.axle_dynamics import native

    axle_input = native._AxleInput()
    axle_input.struct_size = ctypes.sizeof(native._AxleInput)
    axle_input.abi_version = native._NATIVE_KERNEL_ABI_VERSION
    assert axle_input.struct_size == ctypes.sizeof(native._AxleInput)
    assert axle_input.abi_version == native._NATIVE_KERNEL_ABI_VERSION


def test_element_blocks_are_applied_to_both_native_input_structures() -> None:
    """
    Both entry points carry the same element fields, and both must be fillable.

    `VehicleInput` embeds `AxleInput` by value, so a field set on the axle
    structure does not travel into the vehicle structure with the copy.  This
    pins that the shared helper fills either one, which is what makes "one `kind`,
    one layout, both entry points" true at run time and not only on paper.
    """
    import ctypes

    from suspension_multibody.axle_dynamics import native

    block = native._spring_element_block(
        body_a=0,
        body_b=1,
        stiffness=10_000.0,
        compression_damping=100.0,
        rebound_damping=100.0,
        free_length=0.25,
    )
    for structure in (native._AxleInput(), native._VehicleInput()):
        # Start from a state where the per-family arrays are set, so the helper's
        # clearing is observable rather than trivially already true.
        structure.spring_count = 3
        structure.bushing_count = 2
        structure.anti_roll_bar_count = 1
        structure.tire_count = 4
        structure.topology_extension_count = 5
        if hasattr(structure, "aerodynamic_drag_count"):
            structure.aerodynamic_drag_count = 6
        blocks, curves = native._apply_element_blocks(structure, (block,))
        assert structure.spring_count == 0
        assert structure.bushing_count == 0
        assert structure.anti_roll_bar_count == 0
        # Every element family is cleared, not just the three that had a block
        # builder first: the kernel refuses a caller that supplies both spellings,
        # so a family left set would turn a valid model into a rejected run.  The
        # aerodynamic drag is the vehicle structure's own family and the topology
        # extensions are not read at all, so both counters must go to zero too.
        assert structure.tire_count == 0
        assert structure.topology_extension_count == 0
        if hasattr(structure, "aerodynamic_drag_count"):
            assert structure.aerodynamic_drag_count == 0
        assert structure.element_count == 1
        assert structure.elements is not None
        assert structure.element_curves is not None
        # The arrays must stay alive: the structure holds their addresses.
        assert ctypes.sizeof(blocks) > 0
        assert ctypes.sizeof(curves) > 0
