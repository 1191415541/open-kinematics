"""Arbitrary subsystem placements replace the old two-axle assembler."""

import numpy as np
import pytest

from suspension_multibody.authoring import AssemblyDocument, assemble_generic
from suspension_multibody.rigs import generic_rig_subsystem
from tests.rigs._generic import wheel_specimen


def test_three_arbitrary_placements_share_one_declared_support():
    base = wheel_specimen()
    documents = {"support": base.entries[0].subsystem}
    payload = base.to_payload()
    payload["subsystems"] = [payload["subsystems"][0]]
    for label, x in (("front", 1.4), ("middle", 0), ("rear", -1.4)):
        for original in base.entries[1:]:
            ref = label + "_" + original.ref
            documents[ref] = original.subsystem
            pairs = {"support": "support.support", "rack": "support.rack"} if original.ref == "suspension" else {
                "carrier": label + "_suspension.carrier_" + ("L" if original.ref == "left" else "R"), "road": "support.road"}
            payload["subsystems"].append({"ref": ref, "functional_role": original.functional_role, "placement_role": "any",
                "placement": {"translation": [x, 0, 0]}, "pairings": [{"requirement_role": k, "port": v} for k, v in pairs.items()]})
    graph = assemble_generic(AssemblyDocument.from_payload(payload, subsystems=documents))
    assert sum(key == "support.carrier" for key in graph.bodies) == 1
    assert len(graph.tires) == 6
    for label, x in (("front", 1.4), ("middle", 0), ("rear", -1.4)):
        assert all(label + "_" + name + ".wheel" in graph.bodies for name in ("left", "right"))
        assert graph.bodies[label + "_left.wheel"].pose.translation[0] == pytest.approx(x)
    assert not any("front_middle" in key for key in graph.bodies)


def test_one_support_receives_fixture_binding_without_rebuilding_specimen():
    source = wheel_specimen()
    baseline = assemble_generic(source).resolved_model()
    documents = {entry.ref: entry.subsystem for entry in source.entries}
    documents["rig"] = generic_rig_subsystem("ride_four_post", interfaces={})
    payload = source.to_payload()
    payload["subsystems"].append({"ref": "rig", "functional_role": "generic", "placement_role": "any"})
    bound = assemble_generic(AssemblyDocument.from_payload(payload, subsystems=documents)).resolved_model()
    ids = [row["name"] for row in baseline.to_document()["bodies"]]
    assert baseline.subgraph_fingerprint(ids) == bound.subgraph_fingerprint(ids)
    np.testing.assert_array_equal(baseline.to_document()["bodies"], bound.to_document()["bodies"][:-1])
