"""Wheel owns its inertia and tire; a study only changes the spin boundary."""

import numpy as np

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.compilation.motion import resolve_motion_boundaries
from tests.physics.test_wheel_spin_boundary import plan, wheel_assembly


def test_one_declared_wheel_owns_the_only_wheel_mass_and_tire():
    built = assemble_generic(wheel_assembly())
    assert set(built.bodies) == {"support.carrier", "wheel.wheel"}
    assert built.bodies["wheel.wheel"].mass == 20
    np.testing.assert_array_equal(built.bodies["wheel.wheel"].inertia, np.eye(3))
    assert len(built.tires) == 1
    assert built.tires[0]["body"] == "wheel.wheel" and built.tires[0]["mass"] == 0
    assert set(built.markers) == {"wheel.center", "wheel.spin"}


def test_bench_and_vehicle_readings_keep_the_identical_wheel_document():
    source = wheel_assembly()
    original = source.entries[1].subsystem.to_payload()
    graph = assemble_generic(source).resolved_model()
    locked, free = [resolve_motion_boundaries(graph, plan(mode)).model.to_document() for mode in ("locked", "free")]
    for section in ("bodies", "tires", "frames", "coordinates", "ports"):
        assert locked[section] == free[section]
    assert source.entries[1].subsystem.to_payload() == original
    assert len(locked["joints"]) == len(free["joints"]) + 1
    assert locked["joints"][:-1] == free["joints"]


def test_wheel_removal_removes_its_declared_body_tire_and_bearing():
    from tests.authoring.test_unified_subsystem_templates import (
        assembly,
        carrier_subsystem,
    )

    original = assemble_generic(wheel_assembly())
    reduced = assemble_generic(assembly({"support": carrier_subsystem()}))
    assert set(original.bodies) - set(reduced.bodies) == {"wheel.wheel"}
    assert reduced.tires == () and reduced.joints == ()


def test_every_wheel_reference_names_an_existing_entity():
    graph = assemble_generic(wheel_assembly()).resolved_model().to_document()
    bodies = {row["name"] for row in graph["bodies"]}
    for row in graph["joints"]:
        assert row["body_a"] in bodies and row["body_b"] in bodies
    for row in graph["frames"]:
        assert row["body"] in bodies
    assert graph["tires"][0]["parameters"]["frame_body"] == "support.carrier"
