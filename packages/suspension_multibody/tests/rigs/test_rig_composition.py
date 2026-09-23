"""
An assembly and a rig are two axes, and the rig shrinks to fit.

The requirement is that a run be "one assembly plus one rig" rather than one of
seven enumerated pairs, and that a rig adapt to the assembly it is given instead
of demanding everything it can imagine.  These tests hold both halves: the matrix
is queryable and refuses impossible pairs, and an assembly missing a subsystem
loses the corresponding axis entirely rather than having it filled with zeros.
"""

from __future__ import annotations

import pytest

from suspension_multibody.rigs import (
    ASSEMBLIES,
    CompositionError,
    RigError,
    combinations,
    compose,
    get_rig,
    resolve_combination,
    rig_names,
)
from suspension_multibody.subsystems import (
    DEFAULT_AXLE_SUBSYSTEMS,
    DEFAULT_VEHICLE_SUBSYSTEMS,
    capabilities_for,
)


def _caps(subsystems, bodies=("upright_L", "upright_R")):
    return capabilities_for(subsystems=frozenset(subsystems), body_names=frozenset(bodies))


def test_the_seven_existing_combinations_are_registered() -> None:
    """The pairs callers already use must keep resolving to the same rigs."""
    expected = {
        ("axle", "kc_quasi_static"),
        ("axle", "axle_dynamic"),
        ("vehicle", "vehicle_dynamic"),
        ("vehicle", "vehicle_kc"),
        ("vehicle", "handling"),
        ("vehicle", "ride_four_post"),
        ("vehicle", "ride_random_road"),
    }
    assert set(combinations()) == expected
    assert len(combinations()) == 7


def test_the_matrix_is_queryable_by_either_axis() -> None:
    assert set(ASSEMBLIES) == {"axle", "vehicle"}
    assert len(rig_names()) == 7
    for _, rig in combinations():
        assert get_rig(rig).name == rig


def test_an_unknown_rig_is_refused_and_names_the_registered_ones() -> None:
    with pytest.raises(RigError, match="unknown rig"):
        get_rig("no_such_bench")
    with pytest.raises(RigError, match="kc_quasi_static"):
        get_rig("nope")


def test_a_rig_belonging_to_another_assembly_is_refused() -> None:
    """
    A four-post bench drives a vehicle, not a bare axle.

    This is a registration error rather than a capability shortfall: saying "your
    axle is missing a part" would send the reader to the wrong place.
    """
    with pytest.raises(CompositionError, match="registered for the 'vehicle' assembly"):
        resolve_combination("axle", "ride_four_post", _caps(DEFAULT_AXLE_SUBSYSTEMS))


def test_an_unknown_assembly_is_refused() -> None:
    with pytest.raises(CompositionError, match="unknown assembly"):
        resolve_combination("trailer", "kc_quasi_static", _caps(DEFAULT_AXLE_SUBSYSTEMS))


def test_an_assembly_with_steering_drives_the_rack() -> None:
    composition = resolve_combination(
        "axle", "kc_quasi_static", _caps(DEFAULT_AXLE_SUBSYSTEMS)
    )
    assert composition.drives_coordinate("rack_drive")
    assert composition.dropped == ()
    assert not composition.shrunk


def test_an_assembly_without_steering_loses_the_rack_axis_entirely() -> None:
    """
    The shrinkage this whole subtask exists for.

    The rack drive must be *absent*, not zero-filled: a run that carried a rack
    axis of zeros would look steered and not be, and downstream code would happily
    report a rack motion of zero.  Disappearing is the honest answer.
    """
    composition = resolve_combination(
        "axle",
        "kc_quasi_static",
        _caps(DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
    )
    assert not composition.drives_coordinate("rack_drive")
    assert composition.dropped == ("rack_drive",)
    assert composition.shrunk
    # The wheel axes are the assembly's own and must survive untouched.
    assert composition.drives_coordinate("wheel_drive_L")
    assert composition.drives_coordinate("wheel_drive_R")


def test_the_wheel_axis_survives_without_steering() -> None:
    """Only what depends on the missing subsystem goes."""
    with_steering = resolve_combination(
        "axle", "kc_quasi_static", _caps(DEFAULT_AXLE_SUBSYSTEMS)
    )
    without = resolve_combination(
        "axle", "kc_quasi_static", _caps(DEFAULT_AXLE_SUBSYSTEMS - {"steering"})
    )
    kept = [d.coordinate for d in without.drives]
    assert kept == [
        name for name in with_steering.rig.coordinate_names() if name != "rack_drive"
    ]


def test_shrinking_is_a_strict_subset_not_a_rewrite() -> None:
    """Every surviving drive is one the rig asked for, with its kind intact."""
    rig = get_rig("kc_quasi_static")
    composition = compose(rig, _caps(DEFAULT_AXLE_SUBSYSTEMS - {"steering"}))
    asked = {drive.coordinate: drive for drive in rig.drives}
    for drive in composition.drives:
        assert drive.coordinate in asked
        assert drive.kind == asked[drive.coordinate].kind


def test_a_rig_whose_only_drives_are_absent_is_refused() -> None:
    """
    Adapting must not degrade into doing nothing.

    With no drive coordinate left, the bench cannot run at all, and saying so is
    what keeps "the rig adapted" from meaning "the rig silently did nothing".
    """
    empty = capabilities_for(subsystems=frozenset({"chassis"}), body_names=frozenset())
    with pytest.raises(CompositionError, match="offers none of them"):
        compose(get_rig("kc_quasi_static"), empty)


def test_the_judgement_is_capability_driven_not_name_probing() -> None:
    """
    The decision reads `drive_coordinates`, and nothing else.

    An assembly with the right coordinate set but no body named `rack` must still
    drive the rack, and one with a `rack` body but no steering capability must not.
    Probing names would get both backwards, which is how a missing subsystem
    becomes a deep failure instead of a boundary decision.
    """
    offered = capabilities_for(
        subsystems=frozenset(DEFAULT_AXLE_SUBSYSTEMS), body_names=frozenset({"nothing"})
    )
    composition = compose(get_rig("kc_quasi_static"), offered)
    assert composition.drives_coordinate("rack_drive")

    withheld = capabilities_for(
        subsystems=frozenset(DEFAULT_AXLE_SUBSYSTEMS - {"steering"}),
        body_names=frozenset({"rack", "upright_L", "upright_R"}),
    )
    without = compose(get_rig("kc_quasi_static"), withheld)
    assert not without.drives_coordinate("rack_drive")


def test_a_vehicle_assembly_keeps_its_rack_with_steering() -> None:
    composition = resolve_combination(
        "vehicle", "vehicle_kc", _caps(DEFAULT_VEHICLE_SUBSYSTEMS)
    )
    assert composition.drives_coordinate("rack_drive")
    assert not composition.shrunk
