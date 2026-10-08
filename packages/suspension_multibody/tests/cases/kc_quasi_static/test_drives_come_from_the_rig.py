"""Rig input bindings follow typed declarations rather than coordinate names."""

import pytest

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.modeling.coordinates import (
    DRIVE_COORDINATE_NAMES,
    KERNEL_AXIS_GROUPS,
    kernel_axis,
)
from suspension_multibody.rigs import RIGS, generic_rig_subsystem
from tests.authoring.test_unified_subsystem_templates import assembly
from tests.rigs._generic import rig_graph, rig_source


def _names(graph):
    return tuple(row["name"].removeprefix("rig.") for row in graph.inputs)


def test_the_drive_section_follows_the_rigs_declaration():
    full = rig_graph()
    assert _names(full) == ("wheel_drive_L", "wheel_drive_R", "rack_drive")
    reduced = rig_graph(offered=("wheel_drive_L", "wheel_drive_R"))
    assert _names(reduced) == ("wheel_drive_L", "wheel_drive_R")


def test_a_rig_that_drives_one_wheel_gives_the_document_one_wheel_axis():
    graph = rig_graph(offered=("wheel_drive_L", "rack_drive"))
    assert _names(graph) == ("wheel_drive_L", "rack_drive")


def test_a_coordinate_the_rig_does_not_declare_is_not_invented():
    interfaces = {drive.coordinate: {"role": drive.coordinate, "kind": "geometry", "required": False}
        for drive in RIGS["kc_quasi_static"].drives}
    interfaces["not_a_coordinate"] = {"role": "wheel_hub"}
    with pytest.raises(ValueError, match="unknown input interfaces"):
        generic_rig_subsystem("kc_quasi_static", interfaces=interfaces)


def test_the_group_table_covers_every_coordinate_the_axle_uses():
    declared = {drive.coordinate for spec in RIGS.values() for drive in spec.drives if drive.from_assembly}
    assert declared and declared <= DRIVE_COORDINATE_NAMES
    assert all(kernel_axis(name) is not None for name in declared)


def test_a_steeringless_assembly_loses_the_rack_coordinate_entirely():
    graph = rig_graph(offered=("wheel_drive_L", "wheel_drive_R"))
    assert "rack_drive" not in _names(graph)
    assert not any(row["owner"].endswith("rack") for row in graph.inputs)


def test_the_concrete_missing_interface_is_reported():
    graph = rig_graph(offered=("wheel_drive_L", "wheel_drive_R"))
    report = graph.bindings["rig"]
    assert report.disappeared == ("rack_drive",)


def test_port_bindings_are_derived_from_declared_interfaces():
    first = rig_graph(body="arbitrary_part")
    assert all(row["owner"] == "specimen.arbitrary_part" for row in first.inputs)
    reduced = rig_graph(offered=("wheel_drive_L",))
    assert _names(reduced) == ("wheel_drive_L",)


def test_an_unknown_port_is_refused_rather_than_ignored():
    source = rig_source()
    payload = source.to_payload()
    payload["subsystems"][-1]["pairings"][0]["port"] = "specimen.spoiler"
    from suspension_multibody.authoring import AssemblyDocument

    with pytest.raises(ValueError, match="spoiler"):
        assemble_generic(AssemblyDocument.from_payload(payload, subsystems={e.ref: e.subsystem for e in source.entries}))


def test_a_rig_template_change_alone_changes_its_interface():
    source = rig_source()
    spec = source.entries[0].subsystem
    from suspension_multibody.rigs.rig import DriveSpec, RigSpec

    rig = generic_rig_subsystem(RigSpec("probe", drives=(DriveSpec("left", coupled_with=("right",)),)),
        interfaces={"left": {"role": "wheel_drive_L", "kind": "geometry"}})
    graph = assemble_generic(assembly({"specimen": spec, "rig": rig}, pairings={"rig": {"left": "specimen.wheel_drive_L"}}))
    assert _names(graph) == ("left",)


def test_the_kc_bench_declares_both_wheels_and_the_rack():
    assert RIGS["kc_quasi_static"].coordinate_names() == ("wheel_drive_L", "wheel_drive_R", "rack_drive")


def test_the_group_table_is_the_one_the_kernel_reads():
    assert set(KERNEL_AXIS_GROUPS.values()) == {"wheel", "rack"}
    assert KERNEL_AXIS_GROUPS["rack_drive"] == KERNEL_AXIS_GROUPS["rack_neutral"] == "rack"
    assert KERNEL_AXIS_GROUPS["wheel_drive_L"] == "wheel"
