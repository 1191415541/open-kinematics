"""
The tire's own mass and inertia: contract fields, ``Tire`` fields, ABI guard.

Subtask 08 moved the tire's mass ownership from the wheel-end body to the tire
element.  This file covers the half of that work which stops at ``Tire``: the
document fields are parsed, the parsed pair reaches the built model, and a
document that declares neither behaves exactly as it did before the fields
existed.

Why a compiled probe instead of pure Python
-------------------------------------------

``ContractModel::tire_masses()`` and ``Tire::mass`` are internal: the product C
ABI deliberately does not expose a tire array, and adding one to a frozen
structure is an ABI freeze release this step must not open.  So the observable
under test is only reachable from C++.  ``fixtures/tire_mass_probe.cpp`` is that
observable: it compiles against the kernel's public headers, links the static
libraries the kernel build already produced, and prints one line per fact
asserted below.  Nothing here re-implements the reader -- the probe calls it.

The ABI guard
-------------

The two new fields are appended last, so every pre-existing ``Tire`` field keeps
its offset.  The probe prints those offsets and the test compares them against
the table recorded below, which was measured from the tree *before* the fields
were added.  That is a real regression test: reordering any field, or inserting
one in the middle, moves a number here.  The version constants are read from
their single-source header, and the two frozen ``-Input`` structures are pinned
by field count so an accidental per-tire array cannot ship quietly.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
KERNEL = ROOT / "packages" / "suspension_kernel"
FIXTURE = KERNEL / "tests" / "fixtures" / "tire_mass_probe.cpp"
VERSION_HEADER = KERNEL / "cpp" / "include" / "mb_config" / "version.hpp"
INPUT_HEADER = KERNEL / "cpp" / "include" / "mb_input" / "types.hpp"

#: ``Tire`` field offsets in bytes, measured from the tree before the two
#: appended fields existed.  A moved entry means the append-only rule was broken,
#: whatever the reason looked like at the time.
FROZEN_TIRE_OFFSETS = {
    "body": 0,
    "frame_body": 4,
    "drive_torque_body": 8,
    "drive_torque_reaction_body": 12,
    "center": 16,
    "frame_center": 40,
    "drive_torque_axis": 64,
    "spin_axis": 88,
    "forward_axis": 112,
    "radius": 136,
    "maximum_compression": 144,
    "k": 152,
    "c": 160,
    "mu_longitudinal": 168,
    "mu_lateral": 176,
    "brush_k_longitudinal": 184,
    "brush_k_lateral": 192,
    "relaxation_length_longitudinal": 200,
    "relaxation_length_lateral": 208,
    "detached_relaxation": 216,
    "model_kind": 224,
    "state_slot_width": 228,
    "contact_mass": 232,
    "maxwell_enabled": 233,
    "uses_exact_relaxation": 234,
    "has_state_return_mapping": 235,
    "uses_pac2002_law": 236,
    "evaluates_at_wheel_center": 237,
    "projects_compression_on_spin": 238,
    "pac2002_parameters": 240,
    "deflection_curve": 2048,
    "bottoming_curve": 2072,
}

#: The two appended fields, and the size the appended layout produces.
APPENDED_TIRE_OFFSETS = {"mass": 2096, "inertia": 2104}
FROZEN_TIRE_SIZE = 2176

#: The frozen ABI version constants.  Subtask 08 changes neither: it appends to
#: an internal structure and adds no field to ``AxleInput``/``VehicleInput``.
FROZEN_ABI_VERSIONS = {
    "kAxleKernelAbiVersion": 15,
    "kVehicleKernelAbiVersion": 30,
    "kCoreKernelAbiVersion": 1,
}

#: ``AxleInput`` / ``VehicleInput`` field counts recorded after subtask 08.  A new
#: per-tire array on either structure would move one of these numbers, so this is
#: where such an addition has to be noticed rather than shipped.
AXLE_INPUT_FIELDS = 106
VEHICLE_INPUT_FIELDS = 97


def _relative(path: Path) -> str:
    """
    Return ``path`` relative to the repository root, as the linker wants it.

    Every *input* goes through this: the LTO plugin opens each archive it is
    handed, and it fails on an absolute path under this repository, whose
    directory name is non-ASCII.  The linker's own output does not go through the
    plugin, so the ``-o`` path may stay absolute.
    """
    return path.relative_to(ROOT).as_posix()


def _field_count(text: str, name: str) -> int:
    """Return the number of field declarations in ``struct name``."""
    body = re.search(rf"struct {name} \{{(.*?)\n\}};", text, re.S)
    assert body is not None, name
    return len(
        [
            line
            for line in body.group(1).splitlines()
            if line.strip().endswith(";") and not line.strip().startswith("//")
        ]
    )


@pytest.fixture(scope="module")
def probe(tmp_path_factory: pytest.TempPathFactory) -> dict[str, list[str]]:
    """
    Compile the probe once and return its output grouped by line prefix.

    Every line is ``<group> <rest>``, where the group is one of ``declared``,
    ``absent``, ``mismatch``, ``offset`` and ``sizeof``.  The executable goes to a
    temporary directory, so the source tree keeps only the probe source.
    """
    if not FIXTURE.is_file():
        pytest.skip(f"probe source is missing at {FIXTURE}")
    build = KERNEL / "build" / "Release"
    archives = sorted(build.glob("libmb_*.a"))
    if not archives:
        pytest.skip(f"kernel static libraries are not built under {build}")

    sys.path.insert(0, str(KERNEL / "src"))
    from suspension_kernel.binding.build import discover_compiler

    executable = tmp_path_factory.mktemp("tire_mass") / "tire_mass_probe.exe"
    compiled = subprocess.run(
        [
            str(discover_compiler()),
            "-std=c++17",
            "-O2",
            "-flto=auto",
            "-fno-fat-lto-objects",
            "-fopenmp",
            "-I",
            _relative(KERNEL / "cpp" / "include"),
            _relative(FIXTURE),
            "-o",
            str(executable),
            "-Wl,--start-group",
            *[_relative(archive) for archive in archives],
            "-Wl,--end-group",
        ],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    if compiled.returncode != 0:
        pytest.fail("the tire-mass probe did not compile:\n" + compiled.stderr)

    completed = subprocess.run(
        [str(executable)], capture_output=True, text=True, check=False
    )
    assert completed.returncode == 0, completed.stderr
    grouped: dict[str, list[str]] = {}
    for line in completed.stdout.splitlines():
        group, _, rest = line.partition(" ")
        grouped.setdefault(group, []).append(rest)
    return grouped


def test_a_declared_mass_and_inertia_reach_the_built_tire(
    probe: dict[str, list[str]],
) -> None:
    """The document pair is parsed and installed on ``Model::tires``."""
    declared = probe["declared"]
    assert "tires 1" in declared
    assert "mass 12.5" in declared
    # The 3x3 tensor the document wrote, in row-major order.
    assert declared.count("inertia 0.40000000000000002") == 1
    assert declared.count("inertia 0.69999999999999996") == 2
    assert declared.count("inertia 0") == 6
    # And the reader's own table agrees, so the install did not invent values.
    assert "parsed_mass 12.5" in declared
    assert declared.count("parsed_inertia 0.40000000000000002") == 1


def test_a_document_without_the_fields_gets_zero_mass(
    probe: dict[str, list[str]],
) -> None:
    """
    The historical document keeps the historical behaviour.

    Zero is the definition of "the wheel-end body still owns this tire's
    inertia", so every document the Python layer currently emits -- none of which
    declares the pair -- produces a ``Tire`` that behaves as before.
    """
    absent = probe["absent"]
    assert "tires 1" in absent
    assert "mass 0" in absent
    assert absent.count("inertia 0") == 9
    assert "parsed_mass 0" in absent
    assert absent.count("parsed_inertia 0") == 9


def test_a_length_mismatch_is_named_rather_than_truncated(
    probe: dict[str, list[str]],
) -> None:
    """A declaration that cannot be installed must fail with its counts."""
    assert len(probe["mismatch"]) == 1, probe["mismatch"]
    (message,) = probe["mismatch"]
    assert message.startswith("error tire mass declaration does not match")
    assert "1 masses" in message
    assert "1 inertias" in message
    assert "0 tires" in message


def test_the_appended_fields_left_every_earlier_tire_offset_alone(
    probe: dict[str, list[str]],
) -> None:
    """The append-only rule, measured rather than argued."""
    observed = {
        name: int(value) for name, value in (line.split() for line in probe["offset"])
    }
    for field, offset in {**FROZEN_TIRE_OFFSETS, **APPENDED_TIRE_OFFSETS}.items():
        assert observed[field] == offset, field
    assert int(probe["sizeof"][0]) == FROZEN_TIRE_SIZE


def test_the_abi_version_constants_are_unchanged() -> None:
    """The frozen constants, read from their single source."""
    text = VERSION_HEADER.read_text(encoding="utf-8")
    for name, expected in FROZEN_ABI_VERSIONS.items():
        match = re.search(rf"constexpr int {name} = (\d+);", text)
        assert match is not None, name
        assert int(match.group(1)) == expected, name


def test_the_frozen_input_structures_are_untouched() -> None:
    """
    Subtask 08 adds no field to ``AxleInput``/``VehicleInput``.

    A per-tire mass array on either structure would be an ABI freeze release, so
    it does not exist: the parsed pair travels through ``ContractModel`` and is
    installed on the built model instead.  The counts are the recorded shape of
    the two structures, so an accidental addition fails here.
    """
    text = INPUT_HEADER.read_text(encoding="utf-8")
    assert _field_count(text, "AxleInput") == AXLE_INPUT_FIELDS
    assert _field_count(text, "VehicleInput") == VEHICLE_INPUT_FIELDS
