"""
The case document's driven axes come from the rig, not from a name search.

"Which axes does this run have" is a consequence of two declarations: the bench
says which coordinates it drives, the assembly says which it can offer, and the
intersection is what the run actually moves.  The case document used to decide its
`axis_map` by searching the emitted coordinate names for a prefix
(``value.startswith("wheel_drive_")``), which makes the grouping a property of the
*spelling*: a coordinate renamed in the rig, or a group the kernel grows, would fall
out of the map silently and the grid would lose an axis without reporting it.

These tests assert the two properties that a prefix search cannot provide:

1. changing one coordinate in ``RigSpec.drives`` changes the case document's drive
   section -- so the document follows the declaration rather than the model;
2. a coordinate the assembly cannot offer is **absent**, not zero -- because a rack
   axis of zeros makes a run look steered when it is not.
"""

from __future__ import annotations

import json

import pytest

from suspension_multibody.cases.kc_quasi_static import case_document
from suspension_multibody.rigs import RIGS, compose, get_rig
from suspension_multibody.rigs.rig import DriveSpec, RigSpec
from suspension_multibody.schema import FrontAxleModel
from suspension_multibody.subsystems.capabilities import (
    KERNEL_AXIS_GROUPS,
    kernel_axis,
)
from suspension_multibody.subsystems.entry import compose_axle
from suspension_multibody.subsystems.types import (
    DEFAULT_AXLE_SUBSYSTEMS,
    AssemblyRequest,
)

FIXTURE = "packages/suspension_multibody/tests/data/benchmark_axle.json"


def _model() -> FrontAxleModel:
    payload = json.loads(open(FIXTURE, encoding="utf-8").read())
    return FrontAxleModel.model_validate(payload["model"])


def _drives_for(rig: str, assembly) -> tuple[str, ...]:
    """Return the coordinates the resolved run moves, in the rig's order."""
    return tuple(
        drive.coordinate
        for drive in compose(get_rig(rig), assembly.capabilities).drives
    )


def _document(assembly, rig: str, **overrides) -> dict:
    return case_document(
        assembly,
        family="kc_quasi_static",
        name="probe",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        drives=_drives_for(rig, assembly),
        **overrides,
    )


# -- 1. the document follows the rig's declaration -------------------------


def test_the_drive_section_follows_the_rigs_declaration() -> None:
    """
    Removing a coordinate from ``RigSpec.drives`` removes it from the document.

    The document is authored with an explicit drive set here, so the rig's
    declaration is the only input that decides the section -- which is what makes
    the assertion about the source of truth rather than about the model.
    """
    assembly = compose_axle(_model(), "K")
    full = _drives_for("kc_quasi_static", assembly)
    assert set(full) == {"wheel_drive_L", "wheel_drive_R", "rack_drive"}

    document = _document(assembly, "kc_quasi_static")
    assert document["k"]["axis_map"]["rack"] == "rack_drive"
    assert document["k"]["axis_map"]["wheel"] == ["wheel_drive_L", "wheel_drive_R"]

    # The same assembly, with the rig declaring no rack drive at all.
    without_rack = tuple(name for name in full if name != "rack_drive")
    reduced = case_document(
        assembly,
        family="kc_quasi_static",
        name="probe",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        drives=without_rack,
    )
    assert "rack" not in reduced["k"]["axis_map"], (
        "the rack axis must follow the rig's declaration, not the model"
    )
    assert reduced["k"]["axis_map"]["wheel"] == ["wheel_drive_L", "wheel_drive_R"]


def test_a_rig_that_drives_one_wheel_gives_the_document_one_wheel_axis() -> None:
    """The wheel group is a consequence of the set, not of the coordinate name."""
    assembly = compose_axle(_model(), "K")
    document = case_document(
        assembly,
        family="kc_quasi_static",
        name="probe",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        drives=("wheel_drive_L", "rack_drive"),
    )
    assert document["k"]["axis_map"]["wheel"] == ["wheel_drive_L"]


def test_a_coordinate_the_kernel_has_no_group_for_is_not_invented() -> None:
    """
    An unknown coordinate is dropped, not filed under a guessed group.

    Filing it by prefix is exactly the behaviour this replaced: a name that merely
    *looks* like a wheel coordinate would join the wheel list and the grid would
    sweep something the kernel does not know about.
    """
    assembly = compose_axle(_model(), "K")
    document = case_document(
        assembly,
        family="kc_quasi_static",
        name="probe",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        drives=("wheel_drive_L", "not_a_coordinate"),
    )
    named = document["k"]["axis_map"].get("wheel", [])
    named += [document["k"]["axis_map"]["rack"]] if "rack" in document["k"]["axis_map"] else []
    assert "not_a_coordinate" not in named


def test_the_group_table_covers_every_coordinate_the_axle_uses() -> None:
    """
    Every coordinate the rigs declare has a kernel group.

    A coordinate with no group can never appear in an axis map, so a rig adding one
    would produce a run that silently drives nothing.  The check is on the rigs'
    own declarations, so a new bench is covered without editing this test.
    """
    from suspension_multibody.subsystems.capabilities import DRIVE_COORDINATE_NAMES

    declared = {
        drive.coordinate
        for spec in RIGS.values()
        for drive in spec.drives
        if drive.from_assembly
    }
    assert declared, "no rig declares an assembly-provided drive"
    assert declared <= DRIVE_COORDINATE_NAMES, (
        f"rigs drive coordinates the capability table does not know: "
        f"{sorted(declared - DRIVE_COORDINATE_NAMES)}"
    )
    for coordinate in declared:
        assert kernel_axis(coordinate) is not None, (
            f"{coordinate} has no kernel axis group, so it could never be swept"
        )


# -- 2. a coordinate the assembly cannot offer is absent -------------------


def _without_steering() -> FrontAxleModel:
    model = _model()
    hardpoints = {
        name: point for name, point in model.hardpoints.items() if name != "rack_center"
    }
    return model.model_copy(update={"hardpoints": hardpoints})


def test_a_steeringless_assembly_loses_the_rack_coordinate_entirely() -> None:
    """
    The shrink is the rig's own: the rack drive is dropped, not zeroed.

    ``rack_neutral`` is the bench's *own* input rather than an assembly drive, so
    it never reaches the drive set either way -- which is why the assertion is on
    the drive set and on the document together.
    """
    assembly = compose_axle(
        _without_steering(),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    drives = _drives_for("kc_quasi_static", assembly)
    assert drives == ("wheel_drive_L", "wheel_drive_R")
    assert "rack_drive" not in drives

    document = _document(assembly, "kc_quasi_static")
    assert "rack" not in document["k"]["axis_map"]
    # And the shorthand's rack list is empty rather than a zero: "no rack axis" and
    # "a rack axis at zero" are different statements, and the document says the
    # first.
    assert document["k"]["rack_values_mm"] == []


def test_the_concrete_dropped_coordinate_is_named() -> None:
    """
    The shrink names what it removed, so the omission is reportable.

    ``Composition.dropped`` is the value a caller reads to say *which* coordinate
    the assembly could not offer; without it the only evidence would be an axis
    quietly missing from a document.
    """
    assembly = compose_axle(
        _without_steering(),
        "K",
        AssemblyRequest(mode="K", subsystems=DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    composition = compose(get_rig("kc_quasi_static"), assembly.capabilities)
    assert composition.dropped == ("rack_drive",)
    assert composition.shrunk


# -- 3. the capability derivation itself ----------------------------------


def test_capabilities_are_derived_from_the_subsystem_set() -> None:
    """
    ``capabilities.py`` states the rule; this is the value it produces.

    Asserted on both directions so the derivation cannot be satisfied by always
    reporting the full set: steering adds the rack coordinate and its absence
    removes it.
    """
    with_steering = compose_axle(_model(), "K")
    assert "rack_drive" in with_steering.capabilities.drive_coordinates
    assert {"wheel_drive_L", "wheel_drive_R"} <= with_steering.capabilities.drive_coordinates

    from suspension_multibody.subsystems.capabilities import capabilities_for

    without = capabilities_for(
        subsystems=frozenset({"suspension", "wheel", "chassis"}),
        body_names=frozenset({"chassis", "upright_L"}),
    )
    assert without.drive_coordinates == frozenset({"wheel_drive_L", "wheel_drive_R"})
    assert not any(name.startswith("rack") for name in without.drive_coordinates)


def test_an_unknown_role_is_refused_rather_than_ignored() -> None:
    """A typo in a role name is an error, not a silently smaller capability set."""
    from suspension_multibody.subsystems.capabilities import (
        CapabilityError,
        capabilities_for,
    )

    with pytest.raises(CapabilityError, match="unknown subsystem role"):
        capabilities_for(subsystems=frozenset({"suspension", "spoiler"}), body_names=frozenset())


def test_a_rig_spec_change_alone_changes_the_document() -> None:
    """
    The end-to-end property: one edit to the declaration, one change in the output.

    A synthetic bench drives the left wheel and holds the rack; the same assembly
    then produces a document with one wheel axis and no rack, without any code
    change.  This is what "generated from the rig" has to mean to be worth
    asserting.

    ``coupled_with`` is quoted here because it is easy to read as "this drive also
    moves that coordinate".  It is not: the K/C bench lists ``wheel_drive_R`` as its
    own drive entry, and ``coupled_with`` records that a *caller* may use one travel
    for both.  So a rig that declares only the left coordinate moves only the left
    coordinate, and the document says exactly that -- which is the behaviour a
    prefix search could not have produced, because it would have found both names in
    the model.
    """
    assembly = compose_axle(_model(), "K")
    bench = RigSpec(
        name="probe_bench",
        drives=(
            DriveSpec("wheel_drive_L", coupled_with=("wheel_drive_R",)),
        ),
        outputs=(),
    )
    drives = tuple(
        drive.coordinate for drive in compose(bench, assembly.capabilities).drives
    )
    assert drives == ("wheel_drive_L",)
    document = case_document(
        assembly,
        family="kc_quasi_static",
        name="probe",
        wheel_values_mm=(-10.0, 0.0, 10.0),
        drives=drives,
    )
    assert "rack" not in document["k"]["axis_map"]
    assert document["k"]["axis_map"]["wheel"] == ["wheel_drive_L"]


def test_the_kc_bench_declares_both_wheels_and_the_rack() -> None:
    """
    The built-in bench's own declaration, which is what makes the map complete.

    Stated because the test above shows the opposite outcome from a different
    declaration: the map follows the set, so the set has to be right.  The bench
    lists each wheel and the rack as separate drives rather than coupling them.
    """
    assembly = compose_axle(_model(), "K")
    drives = _drives_for("kc_quasi_static", assembly)
    assert drives == ("wheel_drive_L", "wheel_drive_R", "rack_drive")
    document = _document(assembly, "kc_quasi_static")
    assert document["k"]["axis_map"]["wheel"] == ["wheel_drive_L", "wheel_drive_R"]
    assert document["k"]["axis_map"]["rack"] == "rack_drive"


def test_the_group_table_is_the_one_the_kernel_reads() -> None:
    """The table's groups are the kernel's own keys, stated once."""
    assert set(KERNEL_AXIS_GROUPS.values()) == {"wheel", "rack"}
    assert KERNEL_AXIS_GROUPS["rack_drive"] == "rack"
    assert KERNEL_AXIS_GROUPS["rack_neutral"] == "rack"
    assert KERNEL_AXIS_GROUPS["wheel_drive_L"] == "wheel"
