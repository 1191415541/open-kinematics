"""The subsystem set follows declarations rather than automotive categories."""

import pytest

from suspension_multibody.authoring import assemble_generic

from ._torque import torque_assembly


@pytest.mark.parametrize("role", ["brake", "drive"])
def test_torque_subsystem_is_available_on_a_single_wheel_bench(role):
    graph = assemble_generic(torque_assembly(role))
    assert len(graph.tires) == 1
    assert len(graph.elements) == 1
    assert graph.elements[0]["name"] == "unit." + role
    assert not any(name.startswith("unit.") for name in graph.bodies)


def test_omitting_a_force_subsystem_removes_exactly_its_declared_rows():
    from suspension_multibody.authoring import AssemblyDocument

    source = torque_assembly("brake")
    payload = source.to_payload()
    payload["subsystems"] = [row for row in payload["subsystems"] if row["ref"] != "unit"]
    subset = AssemblyDocument.from_payload(payload, subsystems={row.ref: row.subsystem for row in source.entries if row.ref != "unit"})
    full, reduced = [assemble_generic(row) for row in (source, subset)]
    assert full.bodies.keys() == reduced.bodies.keys()
    assert full.joints == reduced.joints and full.tires == reduced.tires
    assert reduced.elements == ()
