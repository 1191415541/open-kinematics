"""The resolved graph owns all bodies, constraints, force elements and coordinates."""

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring import assemble_generic
from suspension_multibody.presets import generic_template
from tests.authoring.test_generic_multibody import (
    _assembly,
    _case,
    _subsystem,
    _template,
)
from tests.authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)

from ._generic import axle_source, resolved


@pytest.mark.parametrize("mode", ["K", "C"])
def test_the_graph_carries_the_declared_constraint_column(mode):
    template = generic_template("suspension")
    points = {name: [0, 0, .334] for name in template.hardpoint_names}
    points.update(upper_rear=[1, 0, .334], lower_rear=[1, 0, .334])
    source = assembly({"support": carrier_subsystem(), "unit": subsystem(template, points)}, mode=mode)
    graph = resolved(source).to_document()
    assert graph["joints"]
    rows = template.payload["joints"]
    assert any("K" in row.get("modes", ()) for row in rows)
    assert all(mode in row.get("modes", ("K", "C")) for row in rows
               if any(item["name"].startswith("unit."+row["name"]) for item in graph["joints"]))


def test_the_result_state_uses_exactly_the_compiled_bodies():
    run = simulate(_assembly(), _case())
    assert run.result.body_ids == tuple(row["name"] for row in run.compiled.model_document["bodies"])
    assert run.raw.states.shape[1] == len(run.result.body_ids)


def test_the_resolved_graph_exposes_complete_entities():
    payload = _template()
    payload["coordinates"] = [{"name": "fixture_input", "joint": "guide", "kind": "translation"}]
    graph = resolved(_assembly({"unit": _subsystem(payload)})).to_document()
    for key in ("bodies", "frames", "joints", "elements", "ports", "coordinates"):
        assert key in graph
        assert graph[key]
        assert len({row["name"] for row in graph[key]}) == len(graph[key])


def test_coordinates_come_from_joint_declarations():
    payload = _template()
    payload["coordinates"] = [{"name": "fixture_input", "joint": "guide", "kind": "translation"}]
    graph = resolved(_assembly({"unit": _subsystem(payload)})).to_document()
    assert [row["name"] for row in graph["coordinates"]] == ["unit.fixture_input"]
    assert graph["coordinates"][0]["source_joint_id"] == "unit.guide"


def test_a_graph_read_is_a_detached_copy():
    model = resolved(axle_source())
    graph = model.to_document()
    original = model.to_document()["frames"][0]["point"]
    graph["frames"][0]["point"][:] = [999]*3
    np.testing.assert_array_equal(model.to_document()["frames"][0]["point"], original)


def test_c_mode_keeps_bushings_as_ordinary_elements():
    k, c = [resolved(axle_source(mode)).to_document() for mode in ("K", "C")]
    assert not [row for row in k["elements"] if row["type"] == "bushing"]
    assert len([row for row in c["elements"] if row["type"] == "bushing"]) == 8
    assert {row["name"] for row in k["bodies"]} == {row["name"] for row in c["bodies"]}


def test_resolved_ids_follow_assembled_entities():
    source = axle_source()
    built = assemble_generic(source)
    graph = built.resolved_model().to_document()
    assert tuple(built.bodies) == tuple(row["name"] for row in graph["bodies"])
    assert tuple(built.elements) == tuple(row["name"] for row in graph["elements"])
