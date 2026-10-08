"""Explicit fixture links preserve wheel identity and joint geometry."""

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring import assemble_generic
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from tests.benchmark_fixture import benchmark_model
from tests.rigs._generic import rig_graph, wheel_specimen


def test_input_owners_are_the_declared_wheels_not_fixture_bodies():
    graph = assemble_generic(wheel_specimen("kc_quasi_static"))
    assert {row["owner"] for row in graph.inputs} == {"left.wheel", "right.wheel"}
    assert {row["name"] for row in graph.inputs} == {"rig.wheel_drive_L", "rig.wheel_drive_R"}
    assert {row["parameters"]["frame_body"] for row in graph.tires} == {"suspension.upright_L", "suspension.upright_R"}


def test_bearing_endpoints_remain_coincident_and_rig_adds_no_mechanical_rows():
    plain = assemble_generic(wheel_specimen())
    linked = assemble_generic(wheel_specimen("kc_quasi_static"))
    assert plain.joints == linked.joints
    bearings = [row for row in linked.joints if row["name"].endswith(".bearing")]
    assert len(bearings) == 2
    for row in bearings:
        a = linked.bodies[row["body_a"]].pose.transform_point(np.asarray(row["point_a"]))
        b = linked.bodies[row["body_b"]].pose.transform_point(np.asarray(row["point_b"]))
        np.testing.assert_allclose(a, b, atol=1e-9)


def test_missing_required_fixture_port_is_rejected():
    with pytest.raises(ValueError, match="required port.*wheel_drive_L"):
        rig_graph(offered=(), required=True)


def test_prescribed_travel_moves_the_suspension_in_a_real_native_run():
    source, case = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(-10, 0, 10))
    run = simulate(source, case)
    assert run.status == "success"
    wheel = next(name for name in run.result.body_ids if name.endswith("wheel_hub_L"))
    index = run.result.body_ids.index(wheel)
    z = [run.result.case_body_state(key)[index, 2] for key in range(len(run.result.cases))]
    assert max(z) - min(z) == pytest.approx(.02, abs=1e-6)
