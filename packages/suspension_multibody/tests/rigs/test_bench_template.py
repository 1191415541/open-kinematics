"""Every bench contributes declared entities and keeps wheel ownership intact."""

import pytest

from suspension_multibody.rigs import RIGS, RigError, generic_rig_template, rig_names

from ._generic import rig_graph


@pytest.mark.parametrize("name", rig_names())
def test_every_bench_contributes_only_its_declared_support(name):
    graph = rig_graph(name)
    assert "rig.fixture" in graph.bodies
    assert graph.bodies["rig.fixture"].fixed
    assert not graph.tires and not graph.elements and not graph.joints
    assert not any("wheel_carrier" in key for key in graph.bodies)
    assert {row["name"].removeprefix("rig.") for row in graph.inputs} == set(RIGS[name].coordinate_names())
    assert graph.fragments[-1].provenance.template == name + "_rig"


def test_handling_and_four_post_inputs_belong_to_the_rig():
    handling = rig_graph("handling")
    assert {row["name"] for row in handling.inputs} == {"rig.road_height", "rig.steering_wheel_angle"}
    posts = rig_graph("ride_four_post")
    assert len(posts.inputs) == 4
    assert all(row["name"].startswith("rig.road_height_") for row in posts.inputs)
    assert all(not row.get("owner") for row in posts.inputs)


def test_unknown_bench_or_interface_is_rejected():
    with pytest.raises(RigError, match="unknown rig"):
        generic_rig_template("absent", interfaces={})
    with pytest.raises(ValueError, match="unknown input interfaces"):
        generic_rig_template("handling", interfaces={"absent": {}})
    with pytest.raises(ValueError, match="missing interface.*wheel_drive_L"):
        generic_rig_template("kc_quasi_static", interfaces={})
