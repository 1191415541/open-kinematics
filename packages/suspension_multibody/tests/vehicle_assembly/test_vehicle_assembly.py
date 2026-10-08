"""Wheel, brake and drive use ordinary subsystem ownership in any assembly."""

import pytest

from suspension_multibody.authoring import assemble_generic, migrate_v1_vehicle
from suspension_multibody.presets import generic_template
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)
from tests.vehicle.vehicle_fixtures import _vehicle


def test_vehicle_wheel_entities_have_one_subsystem_owner():
    source = migrate_v1_vehicle(_vehicle())
    graph = assemble_generic(source)
    wheel_entries = [row for row in source.entries if row.functional_role == "wheel"]
    assert len(wheel_entries) >= 4
    assert len(graph.tires) == 4
    assert all(tire["body"].startswith("wheel_") for tire in graph.tires)
    assert all(tire["parameters"]["frame_body"] != tire["body"] for tire in graph.tires)


@pytest.mark.parametrize("role", ["brake", "drive"])
def test_force_subsystems_are_allowed_in_a_single_wheel_bench(role):
    graph = assemble_generic(assembly({"support": carrier_subsystem(),
        "wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}),
        "force": subsystem(generic_template(role), signals={"drive_input": [[0, 1], [1, 1]]})}))
    assert len(graph.tires) == 1
    assert any(row["name"].startswith("force.") for row in graph.elements)
    assert not any(key.startswith("force.") for key in graph.bodies)


def test_tire_mass_has_one_declared_owner_and_changes_fingerprint():
    model = _vehicle()
    plain = assemble_generic(migrate_v1_vehicle(model))
    shared = assemble_generic(migrate_v1_vehicle(model.model_copy(update={"wheels": tuple(wheel.model_copy(update={"tire_mass": 5}) for wheel in model.wheels)})))
    assert sum(row.mass for row in shared.bodies.values()) == sum(row.mass for row in plain.bodies.values())
    assert sum(row["mass"] for row in shared.tires) == 20
    assert all(row["mass"] == 5 for row in shared.tires)
    assert shared.resolved_model().fingerprint != plain.resolved_model().fingerprint
