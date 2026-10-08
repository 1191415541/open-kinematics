"""Existing native model declarations survive the common graph compiler."""

import numpy as np
import pytest
from suspension_contracts import ContractError

from suspension_multibody.authoring import TemplateDocument, assemble_generic
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedModel, ResolvedSolvePlan
from suspension_multibody.presets import generic_template
from suspension_multibody.simulation import run_compiled

from ..authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)
from ..physics.test_wheel_spin_boundary import plan


def paired_graph():
    support = carrier_subsystem()
    template = support.template.to_payload()
    template["ports"][-1]["cardinality"] = "many"
    source = assembly({
        "support": subsystem(TemplateDocument.from_payload(template), support.payload["hardpoints"]),
        "left": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}),
        "right": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}),
    })
    model = assemble_generic(source).resolved_model()
    graph = model.to_document()
    graph["couplers"] = [{"name": "ratio", "joint_a": "left.bearing", "joint_b": "right.bearing",
        "coordinate_a": "rotation", "coordinate_b": "rotation", "scale_a": 1, "scale_b": -1}]
    graph["gauges"] = [{"body": "left.wheel", "axis_local": [0, 1, 0]}]
    graph["capabilities"] = ["vehicle"]
    graph["elements"] = [{"name": "drag", "type": "aerodynamic_drag", "body_a": "left.wheel",
        "parameters": {"application_point": [0, 0, 0], "forward_axis": [1, 0, 0], "coefficient": .4}}]
    return graph, model.resource_payload


def test_coupler_gauge_and_aero_reach_the_same_native_submission():
    graph, payload = paired_graph()
    run_plan = plan("free").to_document()
    run_plan["boundaries"] = []
    run_plan["solver"] = {"initialization_mode": "provided_consistent_state"}
    compiled = compile_resolved(ResolvedModel(graph, payload), ResolvedSolvePlan(run_plan))
    for key in ("couplers", "gauges", "capabilities", "elements"):
        assert compiled.model_document[key] == graph[key]
    run = run_compiled(compiled)
    assert run.status == "success"
    for name in ("left.wheel", "right.wheel"):
        assert np.isfinite(run.result.body_state(name)).all()


def test_coupler_publishes_stable_multiplier_and_all_four_end_reactions():
    graph, payload = paired_graph()
    run_plan = plan("free").to_document()
    run_plan["boundaries"] = [{"name": "lock", "coordinate": "left.spin", "mode": "locked", "value": 0, "units": "rad"}]
    run_plan["inputs"] = [{"name": "load", "role": "body_wrench", "body": "right.wheel",
        "values": [[0, 0, 0, 0, 17, 0]]*len(run_plan["samples"])}]
    run_plan["solver"] = {"initialization_mode": "provided_consistent_state"}
    result = run_compiled(compile_resolved(ResolvedModel(graph, payload), ResolvedSolvePlan(run_plan))).result
    assert "ratio" in result.constraint_ids
    assert result.constraint_multiplier("ratio").values.shape == (len(run_plan["samples"]), 1)
    ends = [result.constraint_wrench("ratio", end=end) for end in ("a_a", "a_b", "b_a", "b_b")]
    assert [channel.body_id for channel in ends] == ["support.carrier", "left.wheel", "support.carrier", "right.wheel"]
    np.testing.assert_allclose(sum(channel.moment for channel in ends), 0, atol=1e-9)
    assert np.max(np.abs(ends[-1].moment[:, 1])) > 16.9
    with pytest.raises(ValueError, match="constraint end"):
        result.constraint_wrench("ratio", end="a")


@pytest.mark.parametrize("field,change,message", [
    ("couplers", {"joint_b": "missing"}, "unknown joint"),
    ("couplers", {"coordinate_a": "translation"}, "incompatible coordinate"),
    ("gauges", {"body": "missing"}, "unknown body"),
    ("gauges", {"axis_local": [0, 0, 0]}, "nonzero"),
])
def test_invalid_native_graph_references_fail_before_submission(field, change, message):
    graph, payload = paired_graph()
    graph[field][0].update(change)
    with pytest.raises(ContractError, match=message):
        ResolvedModel(graph, payload)
