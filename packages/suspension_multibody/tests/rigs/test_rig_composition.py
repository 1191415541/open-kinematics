"""Rig input selection follows declared ports, independently of assembly labels."""

import pytest

from suspension_multibody.authoring import AssemblyDocument, assemble_generic

from ._generic import rig_graph, rig_source


def test_without_steering_only_the_rack_input_disappears():
    full = rig_graph()
    reduced = rig_graph(offered=("wheel_drive_L", "wheel_drive_R"), body="rack")
    assert {row["name"] for row in full.inputs} == {"rig.wheel_drive_L", "rig.wheel_drive_R", "rig.rack_drive"}
    assert {row["name"] for row in reduced.inputs} == {"rig.wheel_drive_L", "rig.wheel_drive_R"}
    original = {row["name"]: row for row in full.inputs}
    assert all(row["kind"] == original[row["name"]]["kind"] for row in reduced.inputs)


def test_absent_optional_inputs_and_their_outputs_are_removed():
    graph = rig_graph(offered=())
    assert not graph.inputs
    assert not graph.fragments[-1].outputs


def test_absent_required_inputs_are_named():
    with pytest.raises(ValueError, match="required port.*wheel_drive_L"):
        rig_graph(offered=(), required=True)


@pytest.mark.parametrize("name", ["kc_quasi_static", "vehicle_kc", "ride_four_post"])
@pytest.mark.parametrize("label", ["single_axle", "vehicle", "trailer"])
def test_business_labels_do_not_select_a_construction_path(name, label):
    source = rig_source(name)
    data = source.to_payload()
    data["name"] = label
    graph = assemble_generic(AssemblyDocument.from_payload(data, subsystems={row.ref: row.subsystem for row in source.entries}))
    assert len(graph.inputs) == len(rig_graph(name).inputs)
    assert list(graph.bodies) == list(rig_graph(name).bodies)
