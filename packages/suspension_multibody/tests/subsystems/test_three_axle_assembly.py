"""Repeated ordinary subsystem entries retain six wheels and run one native study."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.vehicle.static_loads import compute_static_wheel_loads


def _assembly(placements=("front", "middle", "rear")):
    body = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "body", "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"},
        "bodies": [{"name": "chassis", "fixed": True, "mass": 1200,
                    "position": [0, 0, .6], "inertia": np.eye(3).tolist()}],
        "hardpoints": [{"name": "origin", "owner": "chassis"}], "joints": [], "elements": [],
        "property_slots": [], "ports": [{"name": "support", "role": "support", "owner": "chassis",
                                         "point": "origin", "cardinality": "many"}]})
    axle = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": "trailing-arm", "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "mirrored_xz", "units": {"length": "m"},
        "bodies": [{"name": "upright", "mass": 10, "inertia": np.eye(3).tolist()}],
        "hardpoints": [{"name": "pivot", "owner": "upright"}, {"name": "center", "owner": "upright"}],
        "joints": [{"name": "pivot", "type": "revolute", "body_a": "@support", "body_b": "upright",
                    "point_a": "pivot", "point_b": "pivot", "axis": [0, 1, 0]}],
        "elements": [], "property_slots": [],
        "needs": [{"name": "support", "role": "support", "count": 1, "required": True}],
        "markers": [{"name": "center", "owner": "upright", "point": "center"}],
        "ports": [{"name": "wheel", "role": "wheel_hub", "marker": "center"}]})

    def subsystem(template, points):
        return SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1,
            "name": template.name, "template": template.name, "functional_role": "generic",
            "placement_role": "any", "hardpoints": points, "property_bindings": {}}, template=template)

    subsystems = {"body": subsystem(body, {"origin": [0, 0, .6]})}
    subsystems.update({placement: subsystem(axle, {"pivot": [-.35, -.5, .25], "center": [0, -.75, .3]})
                       for placement in placements})
    return AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1,
        "name": "multi-axle", "assembly_kind": "generic_multibody", "gravity": [0, 0, 0],
        "subsystems": [{"ref": "body", "functional_role": "generic", "placement_role": "any"}]
            + [{"ref": placement, "functional_role": "generic", "placement_role": "any",
                "placement": {"translation": [x, 0, 0]}}
               for placement, x in zip(placements, (1.4, 0, -1.4))]}, subsystems=subsystems)


def _case():
    return {"schema_version": 1, "name": "multi-axle", "study": "dynamic", "protocol": "axle_dynamic",
        "samples": [0, .0005, .001], "solver": {"initialization_mode": "provided_consistent_state",
        "internal_step_s": .00025}, "boundaries": [], "inputs": [], "outputs": []}


def _contacts(source):
    return {entry.ref+"_"+side: entry.ref+".center_"+suffix for entry in source.entries if entry.ref != "body"
            for side, suffix in (("left", "L"), ("right", "R"))}


def test_three_axles_assemble_and_each_keeps_its_own_prefix():
    source = _assembly()
    graph = assemble_generic(source).resolved_model().to_document()
    for placement in ("front", "middle", "rear"):
        assert {row["name"] for row in graph["bodies"] if row["name"].startswith(placement+".")} == {
            placement+".upright_L", placement+".upright_R"}
    assert len(_contacts(source)) == 6
    assert {row["name"] for row in graph["ports"] if row["role"] == "wheel_hub"} == {
        p+".wheel_"+s for p in ("front", "middle", "rear") for s in ("L", "R")}


def test_the_three_axle_shape_is_accepted_and_the_two_axle_one_still_is():
    for placements in (("front", "middle", "rear"), ("front", "rear")):
        compiled = validate(_assembly(placements), _case())
        assert len(compiled.model_document["bodies"]) == 1+2*len(placements)
        assert len(compiled.model_document["joints"]) == 2*len(placements)


def test_unknown_wheel_selection_is_refused():
    model = assemble_generic(_assembly()).resolved_model()
    with pytest.raises(ValueError, match="unknown contact"):
        compute_static_wheel_loads(model, contact_frames={"middle_left": "missing"})


def test_two_entries_with_one_identity_are_refused():
    with pytest.raises(ValueError, match="repeats a subsystem reference"):
        validate(_assembly(("front", "front")), _case())


def test_a_model_with_no_contact_is_refused():
    model = assemble_generic(_assembly(())).resolved_model()
    with pytest.raises(ValueError, match="at least one"):
        compute_static_wheel_loads(model, contact_frames={})


def test_three_axles_are_readable_by_explicit_frame_ids():
    source = _assembly()
    graph = assemble_generic(source).resolved_model()
    result = compute_static_wheel_loads(graph, contact_frames=_contacts(source), gravity=9.81)
    assert set(result.wheel_loads) == set(_contacts(source))
    assert sum(result.wheel_loads.values()) == pytest.approx(result.total_mass*9.81, abs=result.residual_tolerance)


def test_an_axle_entry_runs_its_own_quasi_static_study():
    source = _assembly(("middle",))
    payload = source.to_payload()
    payload["connections"] = [{"name": "travel_"+side, "port_a": "middle.wheel_"+side, "port_b": "body.support",
        "type": "driven_translation", "axis": [0, 0, 1], "target": "travel_"+side} for side in ("L", "R")]
    source = AssemblyDocument.from_payload(payload, subsystems={entry.ref: entry.subsystem for entry in source.entries})
    case = _case()
    case.update(study="quasi_static", protocol="kc_quasi_static", excitation={"k": {
        "wheel_values_mm": [0], "rack_values_mm": [], "axis_map": {"wheel": ["travel_L", "travel_R"]}}})
    result = simulate(source, case).result
    assert result.status == "success"
    assert result.body_state("middle.upright_L").shape[0] == len(case["samples"])


def test_the_three_axle_vehicle_assembles_and_runs_one_study():
    result = simulate(_assembly(), _case()).result
    assert result.status == "success"
    for placement in ("front", "middle", "rear"):
        for side in ("L", "R"):
            state = result.body_state(placement+".upright_"+side)
            assert state.shape == (3, 19)
            assert np.isfinite(state).all()
    np.testing.assert_array_equal(result.times_s, [0, .0005, .001])
