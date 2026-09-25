"""
A bench really contributes entities, and adapts by capability rather than by name.

``G2``/``G3`` ask for a rig that is a subsystem like any other: it declares
bodies, joints, motions and force elements, and the same template serves both root
categories by branching on what the assembly can do.  The weak version -- a
registration whose only content is a name and a drive list -- is what these tests
exist to exclude, so they assert on the *entity counts* and on *who owns the
wheel*, not on the registry listing.
"""

from __future__ import annotations

import pytest

from suspension_multibody.modeling import ModelFragment
from suspension_multibody.rigs import RIGS, RigError, rig_names
from suspension_multibody.rigs.bench import (
    BENCH_TEMPLATE_NAME,
    bench_capability,
    build_rig_assembly,
    build_rig_fragment,
    rig_ports,
)
from suspension_multibody.subsystems import capabilities_for


def _caps(subsystems, bodies=("upright_L", "upright_R")):
    return capabilities_for(subsystems=frozenset(subsystems), body_names=frozenset(bodies))


def test_every_shipped_bench_builds_a_fragment() -> None:
    """No bench may be a name with nothing behind it."""
    for name in rig_names():
        fragment = build_rig_fragment(name)
        assert isinstance(fragment, ModelFragment)
        assert fragment.bodies, f"{name} contributes no body at all"


def test_a_wheel_supplying_bench_really_builds_wheels_joints_and_tires() -> None:
    """
    The entities the bench owns, asserted individually.

    A single-axle assembly builds no wheel body (D9), so if the bench does not
    create one there is no wheel at all: the tire, its force element and the
    travel motion that moves it all have to come from here.
    """
    fragment = build_rig_fragment("kc_quasi_static")
    assert {"wheel_carrier_L", "wheel_carrier_R"} <= set(fragment.bodies)
    assert len(fragment.tires) == 2
    assert len(fragment.forces) == 2
    assert len(fragment.joints) == 2
    assert any("wheel_drive_L" == entry["coordinate"] for entry in fragment.drives.values())


def test_a_vehicle_loading_bench_does_not_create_a_second_wheel() -> None:
    """
    The vehicle owns its wheels; a bench that made another would double a mass.

    This is the asymmetric half of the rule and the reason the branch exists at
    all -- the two root categories differ in *who owns the wheel*, not in the
    bench's name.
    """
    for name in ("vehicle_dynamic", "vehicle_kc", "handling", "ride_four_post", "ride_random_road"):
        fragment = build_rig_fragment(name)
        assert fragment.tires == {}, name
        assert not any("wheel_carrier" in body for body in fragment.bodies), name


def test_the_capability_comes_from_the_declared_drive_list() -> None:
    """One source, so a bench cannot claim wheels in one place and not another."""
    for name, spec in RIGS.items():
        expected = "wheel_supplying" if spec.supplies_wheels else "vehicle_loading"
        assert bench_capability(name) == expected
        assert bench_capability(spec) == expected


def test_a_wheel_supplying_bench_offers_a_wheel_centre_per_side() -> None:
    ports = rig_ports("kc_quasi_static")
    assert set(ports) == {"wheel_centre_L", "wheel_centre_R"}
    assert ports["wheel_centre_L"].labels == frozenset({"L"})
    assert "wheel" in ports["wheel_centre_L"].capabilities


def test_a_vehicle_loading_bench_offers_a_mount_instead() -> None:
    """It pushes on the body; it does not supply a wheel the vehicle already has."""
    ports = rig_ports("vehicle_dynamic")
    assert set(ports) == {"mount"}
    assert ports["mount"].role == "body_mount"


def test_the_bench_assembly_carries_no_root_kind_of_its_own() -> None:
    """
    Global rules are checked against the device under test, not against the bench.

    Giving the bench a kind would have it judged twice -- once as itself -- and the
    single-axle rule would then complain about a bench that legitimately owns
    wheels.
    """
    bench = build_rig_assembly("kc_quasi_static")
    assert bench.root_kind == ""


def test_the_bench_records_its_template_and_capability() -> None:
    """A5 traces entities to their source; a bench is a source like any other."""
    fragment = build_rig_fragment("kc_quasi_static", instance=("kc_quasi_static",))
    assert fragment.provenance is not None
    assert fragment.provenance.template == BENCH_TEMPLATE_NAME
    assert fragment.provenance.revision == "wheel_supplying"
    assert fragment.instance == ("kc_quasi_static",)


def test_the_bench_inputs_become_motions_it_owns() -> None:
    """
    A bench's own input (road height, steering-wheel angle) is a motion it owns.

    A coordinate the *assembly* was supposed to provide is deliberately not
    emitted here: it is a request, and the interface resolver decides whether it
    survives.  Emitting it would let a bench drive something never offered.
    """
    handling = build_rig_fragment("handling")
    coordinates = {entry["coordinate"] for entry in handling.drives.values()}
    assert "steering_wheel_angle" in coordinates
    assert "road_height" in coordinates

    quasi_static = build_rig_fragment("kc_quasi_static")
    # `wheel_drive_L` is the assembly's to offer; it must not appear as a bench
    # motion owned by the bench.
    assert not any(
        entry.get("coordinate") == "wheel_drive_L"
        and entry.get("base") is not None
        for entry in quasi_static.drives.values()
    )


def test_an_unknown_bench_names_the_registered_ones() -> None:
    with pytest.raises(RigError, match="unknown bench"):
        bench_capability("no_such_bench")
    with pytest.raises(RigError, match="unknown rig"):
        build_rig_fragment("no_such_bench")


def test_an_unknown_mode_is_refused() -> None:
    """Passed rather than ignored, so a caller cannot silently get another bench."""
    with pytest.raises(RigError, match="unknown mode"):
        build_rig_fragment("kc_quasi_static", mode="X")


def test_the_two_branches_differ_in_what_they_contribute() -> None:
    """The capability is load-bearing, not decorative."""
    wheel_bench = build_rig_fragment("kc_quasi_static")
    load_bench = build_rig_fragment("vehicle_dynamic")
    assert len(wheel_bench.bodies) > len(load_bench.bodies)
    assert wheel_bench.tires and not load_bench.tires
