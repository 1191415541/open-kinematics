"""
Executable checks for the single-source-of-truth ABI version contract.

Two different things can drift apart here, and both are checked:

* the C++ side: a version literal spelled somewhere other than the one header
  that is allowed to define it, which is how a bump ends up half-applied;
* the artefact side: the version the library exports, the version its metadata
  records, and the version the Python boundary asserts, which must all agree.
"""

from __future__ import annotations

import ctypes
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).parents[4]
#: The kernel's C++ tree.  K6 is still moving translation units into
#: `cpp/src/<module>/`, so these tests locate a unit by name rather than by path:
#: a test that hard-codes the directory breaks on every move and teaches nothing.
KERNEL_CPP = ROOT / "packages" / "suspension_kernel" / "cpp"
VERSION_HEADER = KERNEL_CPP / "include" / "mb_base" / "version.hpp"


def _kernel_source(name: str) -> Path:
    """Return the one kernel source called `name`, wherever the module tree put it."""
    matches = sorted(path for path in KERNEL_CPP.rglob(name) if path.is_file())
    assert len(matches) == 1, (name, matches)
    return matches[0]

#: The only places a version may be spelled in C++, as (pattern, description).
#: Each pattern matches a *definition* -- the constant's own name followed by an
#: integer -- so unrelated numeric literals (a PAC2002 parameter fallback, a
#: channel index) and references to the constant are not counted.
VERSION_DEFINITION_PATTERNS = (
    (re.compile(r"\bkAxleKernelAbiVersion\s*=\s*\d+"), "axle"),
    (re.compile(r"\bkVehicleKernelAbiVersion\s*=\s*\d+"), "vehicle"),
    (re.compile(r"\bkCoreKernelAbiVersion\s*=\s*\d+"), "core"),
)


def _kernel_sources() -> list[Path]:
    return sorted(
        path for path in KERNEL_CPP.rglob("*") if path.suffix in (".hpp", ".cpp")
    )


def test_hoisted_symbols_are_defined_in_their_target_layer() -> None:
    """
    Each symbol the epic moved must be defined in the layer it was moved to.

    The check is textual against the translation unit that owns the layer, which
    is what makes it a statement about where the definition *lives* rather than
    about whether the program links.
    """
    #: symbol -> (translation unit, the layer that unit implements)
    hoisted = {
        "finite_vec": ("kernel_base.cpp", "mb_base"),
        "max_abs": ("kernel_base.cpp", "mb_base"),
        # D9/K5: the tire state block has its own layer, so the width, the storage
        # and the two accessors live in the tire-state unit rather than with the
        # model or with the PAC2002 law.  The total width used to be defined in
        # `kernel_pac2002_law.cpp`, which made the state layer depend on a tire
        # model; it is also checked below so it cannot drift back.
        "tire_state_width": ("kernel_tire_state.cpp", "mb_tire_state"),
        "tire_block_width": ("kernel_tire_state.cpp", "mb_tire_state"),
        "resize_tire_states": ("kernel_tire_state.cpp", "mb_tire_state"),
        "tire_state_value": ("kernel_tire_state.cpp", "mb_tire_state"),
        "read_tire_states": ("kernel_tire_state.cpp", "mb_tire_state"),
        # D8: road geometry is model state, so the contact-kinematics layer no
        # longer reaches into the tire translation unit for it.
        "road_profile_height": ("kernel_model.cpp", "mb_model"),
        "road_profile_slope": ("kernel_model.cpp", "mb_model"),
    }
    offenders: list[str] = []
    for symbol, (unit, layer) in hoisted.items():
        text = _kernel_source(unit).read_text(encoding="utf-8")
        if not re.search(rf"^\s*\S[^\n]*\b{symbol}\s*\(", text, re.MULTILINE):
            offenders.append(f"{symbol} is not defined in {unit} ({layer})")
    assert not offenders, "\n".join(offenders)


def test_the_integrator_does_not_dispatch_on_the_tire_model_kind() -> None:
    """
    K5's contract: a new tire model must not require an integrator change.

    The integrator used to compare `model_kind` in twenty-five places to decide
    whether a slot needed the exact-exponential relaxation row, the brush return
    mapping or the PAC2002 reuse window.  It now reads `Tire`'s state-semantics
    flags, and this test is what keeps a later edit from reaching for the kind
    again: `model_kind` is the tire model's own business, not the integrator's.
    """
    integrator_units = (
        "kernel_integrator.cpp",
        "kernel_integrator_residual.cpp",
        "kernel_integrator_newton.cpp",
        "kernel_integrator_step.cpp",
        "kernel_integrator_input.cpp",
    )
    offenders: list[str] = []
    for unit in integrator_units:
        text = _kernel_source(unit).read_text(encoding="utf-8")
        for match in re.finditer(r"\bmodel_kind\b", text):
            line = text.count("\n", 0, match.start()) + 1
            offenders.append(f"{unit}:{line} still dispatches on model_kind")
    assert not offenders, "\n".join(offenders)


def test_the_tire_slot_table_is_the_written_down_slot_alignment_list() -> None:
    """
    The slot layout must be stated once, and the allocation ladder must match it.

    `kTireSlotDescriptors` is the epic's "`ResidualWorkspace` slot alignment list":
    the residual and the analytic Jacobian stride a tire's state block by slot, so
    the slot-to-`State`-member mapping has to live in one table rather than in a
    switch per consumer.  The widths checked here are the ones
    `resize_tire_states` allocates by, so a slot added to the table without a
    matching allocation (or the reverse) fails.
    """
    slot_header = (
        ROOT
        / "packages"
        / "suspension_kernel"
        / "cpp"
        / "include"
        / "mb_tire_state"
        / "tire_state.hpp"
    )
    header = slot_header.read_text(encoding="utf-8")
    table = re.search(
        r"kTireSlotDescriptors\[\]\s*=\s*\{(.*?)\n\};", header, re.DOTALL
    )
    assert table, f"the slot descriptor table is missing from {slot_header.name}"
    slots = sorted(
        int(match)
        for match in re.findall(r"^\s*\{(\d+),", table.group(1), re.MULTILINE)
    )
    assert slots == list(range(12)), slots
    # The table belongs to the tire-state layer, and it is the only copy: K6
    # deleted the transitional aggregate header, so the check is now that no
    # kernel source at all carries a second one.
    copies = [
        path
        for path in _kernel_sources()
        if "kTireSlotDescriptors[]" in path.read_text(encoding="utf-8")
    ]
    assert copies == [slot_header], copies

    state = _kernel_source("kernel_tire_state.cpp").read_text(encoding="utf-8")
    # The allocation ladder in `resize_tire_states` is the other half of the
    # contract: these are the widths it must switch on.
    for width in ("4", "6", "8", "12"):
        assert f"slot_width < {width}" in state, (
            f"resize_tire_states no longer allocates at width {width}"
        )
    # And the mapping itself must not be duplicated outside the table.
    residual = _kernel_source("kernel_integrator_residual.cpp").read_text(
        encoding="utf-8"
    )
    assert "switch (slot)" not in residual, (
        "the residual still maps slots by hand instead of reading the table"
    )


def test_cpp_version_literals_are_defined_in_exactly_one_header() -> None:
    """No C++ source outside `version.hpp` may define a version constant."""
    offenders: list[str] = []
    for source in _kernel_sources():
        text = source.read_text(encoding="utf-8")
        for pattern, label in VERSION_DEFINITION_PATTERNS:
            for match in pattern.finditer(text):
                if source == VERSION_HEADER:
                    continue
                line = text.count("\n", 0, match.start()) + 1
                offenders.append(f"{source.name}:{line} defines the {label} version")
    assert not offenders, (
        "ABI versions must be defined only in version.hpp; found:\n"
        + "\n".join(offenders)
    )


def test_version_header_defines_all_three_versions() -> None:
    """The header the contract names must actually carry all three constants."""
    text = VERSION_HEADER.read_text(encoding="utf-8")
    for pattern, label in VERSION_DEFINITION_PATTERNS:
        assert pattern.search(text), f"version.hpp does not define the {label} version"


def test_exported_metadata_and_boundary_versions_agree() -> None:
    """
    The library, its metadata and the Python boundary must report one version.

    Reading the exports here is deliberately independent of
    `suspension_kernel.binding`: the point is to compare two separately produced
    answers, not to re-read the same cached value.
    """
    if sys.platform == "win32":
        library_name = "suspension_kernel.dll"
    elif sys.platform == "darwin":
        library_name = "libsuspension_kernel.dylib"
    else:
        library_name = "libsuspension_kernel.so"
    native_dir = (
        ROOT
        / "packages"
        / "suspension_multibody"
        / "src"
        / "suspension_multibody"
        / "native"
    )
    library = native_dir / library_name
    exports: dict[str, int] = {}
    handle = ctypes.CDLL(str(library))
    for symbol in (
        "axle_kernel_abi_version",
        "vehicle_kernel_abi_version",
        "mb_core_abi_version",
    ):
        probe = getattr(handle, symbol)
        probe.argtypes = []
        probe.restype = ctypes.c_int
        exports[symbol] = int(probe())

    metadata = json.loads((native_dir / "native_build.json").read_text("utf-8"))
    assert metadata["abi_version"] == exports["axle_kernel_abi_version"]
    assert metadata["vehicle_abi_version"] == exports["vehicle_kernel_abi_version"]
    assert metadata["core_abi_version"] == exports["mb_core_abi_version"]

    from suspension_multibody.axle_dynamics import native

    assert metadata["abi_version"] == native._NATIVE_KERNEL_ABI_VERSION
    assert metadata["vehicle_abi_version"] == native._NATIVE_VEHICLE_KERNEL_ABI_VERSION
    assert metadata["core_abi_version"] == native._NATIVE_CORE_ABI_VERSION
