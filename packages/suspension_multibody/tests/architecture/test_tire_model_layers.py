"""
K6: the tire models are separate libraries, and no file escapes the build lists.

The epic's acceptance for the domain split has three parts this file can check:

* the tire models live in `tire/<model>/` directories, one CMake target each;
* the three models do not call each other, which is what makes them independent
  and is why the link graph can enforce it rather than a comment;
* every translation unit under `cpp/` is named in both build lists, so a file
  moved into place cannot be silently left out of the library.

The last one is the one that catches real mistakes: the build already asserts the
reverse direction (everything listed exists), but nothing asserted that everything
that exists is listed.
"""

from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / "packages" / "suspension_kernel"
CPP = PACKAGE / "cpp"
#: The tire models live under the module tree, next to the layer libraries they
#: belong to.  K6 moved them there from `axle_dynamics/tire/`; this is the one
#: place the path is written down.
TIRE = CPP / "src" / "tire"
CMAKE = PACKAGE / "CMakeLists.txt"
BUILD = PACKAGE / "src" / "suspension_kernel" / "binding" / "build.py"

#: The model directories and the CMake target each one belongs to.
MODELS = {
    "brush": "mb_tire_brush",
    "fiala": "mb_tire_fiala",
    "pac2002": "mb_tire_pac2002",
}


def _sources() -> list[Path]:
    return sorted(CPP.rglob("*.cpp"))


def _headers() -> list[Path]:
    return sorted(CPP.rglob("*.hpp"))


def test_each_tire_model_is_a_directory_with_its_own_translation_units() -> None:
    for model in MODELS:
        units = sorted((TIRE / model).glob("*.cpp"))
        assert units, f"the {model} tire model has no translation unit"


def test_each_tire_model_is_its_own_cmake_target() -> None:
    cmake = CMAKE.read_text(encoding="utf-8")
    for model, target in MODELS.items():
        assert re.search(rf"add_library\(\s*{target}\s+STATIC\b", cmake), (
            f"{target} is not a static library target"
        )
        # The sources of a target are listed under a variable named after it, and
        # every file of the model directory must appear there.
        for unit in sorted((TIRE / model).glob("*.cpp")):
            relative = unit.relative_to(PACKAGE).as_posix()
            assert relative in cmake, f"{relative} is not in the CMake source lists"


def test_the_tire_models_do_not_call_each_other() -> None:
    """
    A model directory may only call its own model's functions.

    This is the property the separate libraries exist to enforce.  Checking it in
    the sources as well as in the link graph is deliberate: a cross-model call that
    happened to be resolved through the aggregate shared library would link
    perfectly happily, and the link graph alone would never notice.
    """
    offenders: list[str] = []
    for model in MODELS:
        for unit in sorted((TIRE / model).glob("*.cpp")):
            text = unit.read_text(encoding="utf-8")
            text = re.sub(r"//.*", "", text)
            for match in re.finditer(r"\b((?:pac2002|fiala)_\w+)\s*\(", text):
                name = match.group(1)
                if not name.startswith(model):
                    line = text.count("\n", 0, match.start()) + 1
                    offenders.append(
                        f"{unit.relative_to(PACKAGE)}:{line} calls {name}"
                    )
    assert not offenders, "\n".join(offenders)


def test_the_aggregate_links_the_tire_libraries_without_linking_them_to_each_other() -> None:
    cmake = CMAKE.read_text(encoding="utf-8")
    link = re.search(
        r"target_link_libraries\(\s*suspension_kernel PRIVATE(.*?)\)", cmake, re.DOTALL
    )
    assert link, "the aggregate's link line is missing"
    linked = set(link.group(1).split())
    for target in MODELS.values():
        assert target in linked, f"{target} is not linked into the shared library"
    for model, target in MODELS.items():
        for other, other_target in MODELS.items():
            if other == model:
                continue
            assert not re.search(
                rf"target_link_libraries\(\s*{target}\b[^)]*\b{other_target}\b", cmake
            ), f"{target} links {other_target}"


def test_every_tire_model_is_a_row_in_the_registry() -> None:
    """
    The integrator's questions come from the model registry, not from a kind switch.

    `MODULES.md` §5.5 makes `TireModelDescriptor` the single place a tire model is
    described, so that adding one is adding a row rather than editing the
    integrator.  Two things have to hold for that to be true: every
    `VehicleTireModelKind` has a row, and the flags the integrator reads are taken
    from that row instead of being derived from `model_kind` again.
    """
    registry = (
        ROOT / "packages" / "suspension_kernel" / "cpp" / "include" / "mb_tire"
        / "model.hpp"
    ).read_text(encoding="utf-8")
    # One row per registered kind, keyed by the kind itself.
    table = re.search(
        r"kTireModelDescriptors\[\]\s*=\s*\{(.*?)\n\};", registry, re.DOTALL
    )
    assert table, "the registry table is missing"
    for kind in (
        "VEHICLE_TIRE_NATIVE_BRUSH",
        "VEHICLE_TIRE_PAC2002_PURE_SLIP",
        "VEHICLE_TIRE_PAC2002_ADAMS_SOURCE",
        "VEHICLE_TIRE_FIALA",
    ):
        assert table.group(1).count(kind) == 1, kind

    # The semantics helper reads the row; it must not go back to comparing kinds.
    assert "tire_model_descriptor(tire.model_kind)" in registry
    assert "tire.model_kind ==" not in registry
    # K6 deleted the transitional aggregate header that used to be the place a
    # second derivation could hide, so the same statement is now made about every
    # kernel header: the flags have one derivation, and it is the table's.
    for header in _headers():
        assert "tire.model_kind ==" not in header.read_text(encoding="utf-8"), (
            f"{header.name} derives the tire flags from the model kind again"
        )


def test_every_translation_unit_is_in_both_build_lists() -> None:
    """
    A `.cpp` that is on disk but in no list compiles nowhere.

    CMake does not glob and the build metadata is explicit, so a file moved into
    place without a list edit would simply not be built -- and would show up only
    as an undefined symbol, or not at all if nothing referenced it yet.
    """
    cmake = CMAKE.read_text(encoding="utf-8")
    build = BUILD.read_text(encoding="utf-8")
    offenders: list[str] = []
    for unit in _sources():
        relative = unit.relative_to(PACKAGE).as_posix()
        if relative not in cmake:
            offenders.append(f"{relative} is missing from CMakeLists.txt")
        if f'"packages/suspension_kernel/{relative}"' not in build:
            offenders.append(f"{relative} is missing from SOURCE_RELATIVES")
    assert not offenders, "\n".join(offenders)
