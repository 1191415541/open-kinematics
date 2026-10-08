"""Synthetic topology and physical bench extend only ordinary declaration data."""

from __future__ import annotations

import hashlib
import math
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
)
from suspension_multibody.authoring.migration import migrate_v1_axle, migrate_v1_kc_case
from suspension_multibody.authoring.properties import ElementPropertyDocument
from suspension_multibody.report.kc_evidence import frame_fields
from tests.benchmark_fixture import benchmark_model
from tests.composable.fixtures import (
    load_bench_payload,
    trailing_arm_expected,
    trailing_arm_model,
    trailing_arm_payload,
)


def _run(model=None, travel=(-10., 0., 10.)):
    assembly, case = migrate_v1_kc_case(trailing_arm_model() if model is None else model,
        mode="K", wheel_values_mm=travel, rack_values_mm=(0.,))
    return simulate(assembly, case)


def _poses(run, side="L"):
    marker = "wheel.sub.json.wheel_center_"+side
    values = run.result.frame_pose(marker)
    return np.array([values[int(case["sample_offset"])+int(case["sample_count"])-1] for case in run.result.cases])


def test_the_fixture_is_marked_synthetic_and_is_not_a_wishbone_rename():
    source = trailing_arm_payload()
    assert source["_synthetic"] is True and "SYNTHETIC" in source["description"]
    trailing = assemble_generic(migrate_v1_axle(trailing_arm_model())).resolved_model().to_document()
    wishbone = assemble_generic(migrate_v1_axle(benchmark_model())).resolved_model().to_document()
    assert len(trailing["bodies"]) == 3
    assert len(trailing["bodies"]) < len(wishbone["bodies"])
    assert len(trailing["joints"]) == 2
    assert {row["type"] for row in trailing["joints"]} == {"revolute"}
    assert not any("rack" in row["name"] or "tie_rod" in row["name"] for row in trailing["bodies"])


def test_the_double_wishbone_is_not_the_trailing_arm():
    graph = assemble_generic(migrate_v1_axle(benchmark_model())).resolved_model().to_document()
    assert any("upper_arm" in row["name"] for row in graph["bodies"])
    assert any("tie_rod" in row["name"] for row in graph["bodies"])
    assert len({row["type"] for row in graph["joints"]}) > 1


def test_the_synthetic_trailing_arm_solves_through_the_public_entry():
    run = _run()
    assert run.status == "success" and len(run.result.cases) == 3
    for index in range(3):
        constraint, force, _ = run.raw.case_residuals(index)
        assert abs(constraint) < 1e-5
        assert abs(force) < 1e-5


def test_the_solved_wheel_centre_matches_the_derived_rotation():
    expected = trailing_arm_expected()
    dx, _, dz = expected["wheel_centre_offset_from_pivot_mm"]
    for travel, pose in zip(expected["wheel_travel_mm"], _poses(_run())):
        theta = travel / expected["wheel_centre_dz_per_radian_mm"]
        predicted = 250-dx*math.sin(theta)+dz*math.cos(theta)
        assert pose[2, 3]*1000 == pytest.approx(predicted, abs=.05)
        assert pose[2, 3]*1000 == pytest.approx(300+travel, abs=1e-4)


def test_the_trailing_arm_drives_both_explicit_side_ports():
    run = _run()
    np.testing.assert_allclose(_poses(run, "L")[:, 2, 3], _poses(run, "R")[:, 2, 3], atol=1e-9, rtol=0)


def _bench():
    source = load_bench_payload()
    template = TemplateDocument.from_payload({"document": "template", "schema_version": 1,
        "name": source["name"], "functional_role": "generic", "allowed_placement_roles": ["any"],
        "symmetry": "asymmetric", "units": {"length": "m"},
        "bodies": [{"name": row["name"], "fixed": row["fixed"], "mass": row["mass"],
            "inertia": row["inertia"], "position": row["position_m"]} for row in source["bodies"]],
        "hardpoints": [{"name": "anchor", "owner": "bench_frame"},
            {"name": "guide", "owner": "bench_frame"}, {"name": "mount", "owner": "load_carriage"}],
        "joints": [{"name": "carriage_guide", "type": "prismatic", "body_a": "bench_frame",
            "body_b": "load_carriage", "point_a": "guide", "point_b": "mount", "axis": [0, 0, 1]}],
        "coordinates": [{"name": "load_travel", "joint": "carriage_guide", "kind": "translation"}],
        "elements": [{"name": row["name"], "type": row["kind"], "body_a": "bench_frame",
            "body_b": "load_carriage", "point_a": "anchor", "point_b": "mount", "property_slot": row["kind"]}
            for row in source["forces"]],
        "property_slots": [{"name": kind, "element_type": kind, "required": True} for kind in ("spring", "damper")]})
    laws = {kind: ElementPropertyDocument.from_payload({"document": "element_properties", "schema_version": 1,
        "name": kind, "element_type": kind, "model": "linear", "units": {"force": "N", "length": "mm", "time": "s"},
        "parameters": {"stiffness": 25, "free_length": 400} if kind == "spring" else {"viscous_damping": 1.2}})
        for kind in ("spring", "damper")}
    return SubsystemDocument.from_payload({"document": "subsystem", "schema_version": 1, "name": source["name"],
        "template": template.name, "functional_role": "generic", "placement_role": "any",
        "hardpoints": {"anchor": [0, 0, 0], "guide": [0, 0, .4], "mount": [0, 0, .4]},
        "property_bindings": {kind: kind for kind in laws}}, template=template, properties=laws)


def _bench_assembly(topology=None):
    base = {} if topology is None else {entry.ref: entry.subsystem for entry in topology.entries}
    subsystems = {**base, "bench": _bench()}
    return AssemblyDocument.from_payload({"document": "assembly", "schema_version": 1, "name": "physical-bench",
        "assembly_kind": "generic_multibody", "gravity": [0, 0, 0],
        "subsystems": [{"ref": ref, "functional_role": sub.functional_role, "placement_role": sub.placement_role}
            for ref, sub in subsystems.items()]}, subsystems=subsystems)


def _bench_case(name="bench"):
    return {"schema_version": 1, "name": name, "study": "dynamic", "protocol": "axle_dynamic", "samples": [0, .001],
        "solver": {"initialization_mode": "provided_consistent_state"},
        "boundaries": [{"name": "motion_load_travel", "coordinate": "bench.load_travel", "mode": "locked", "value": 0, "units": "m"}],
        "inputs": [], "outputs": []}


def test_the_synthetic_bench_declares_real_entities():
    graph = assemble_generic(_bench_assembly()).resolved_model().to_document()
    expected = load_bench_payload()["expected"]
    assert [row["name"].split(".")[-1] for row in graph["bodies"]] == expected["contributes_bodies"]
    assert [row["name"].split(".")[-1] for row in graph["joints"]] == expected["contributes_joints"]
    assert [row["name"].split(".")[-1] for row in graph["elements"]] == expected["contributes_forces"]
    assert not graph["tires"]


def test_the_bench_entities_reach_native_with_the_derived_spring_force():
    assembly = _bench_assembly()
    payload = assembly.to_payload()
    payload["subsystems"][0]["overrides"] = {"hardpoints": {"mount": [0, 0, .39]}}
    source = AssemblyDocument.from_payload(payload, subsystems={"bench": _bench()})
    run = simulate(source, _bench_case())
    assert run.status == "success"
    force = np.max(np.abs(run.raw.block("element_wrench")[:, 0, :3]))
    assert force == pytest.approx(load_bench_payload()["expected"]["spring_force_at_10mm_n"], rel=1e-6)
    assert "bench.load_carriage" in run.result.body_ids


@pytest.mark.parametrize("model", [benchmark_model, trailing_arm_model])
def test_one_kc_bench_drives_both_topologies(model):
    run = _run(model(), (0.,))
    assert run.status == "success"
    assert any(row["type"] == "driven_translation" for row in run.compiled.model_document["joints"])
    assert len(run.result.cases) == 1


@pytest.mark.parametrize("model", [benchmark_model, trailing_arm_model])
def test_physical_bench_contributes_the_same_entities_to_two_topologies(model):
    source = _bench_assembly(migrate_v1_axle(model()))
    graph = assemble_generic(source).resolved_model().to_document()
    assert {"bench.bench_frame", "bench.load_carriage"} <= {row["name"] for row in graph["bodies"]}
    assert sum(row["name"].startswith("bench.") for row in graph["elements"]) == 2


def test_a_hardpoint_perturbation_rebuilds_the_bench_attachment():
    from suspension_multibody.schema import Vec3
    base = trailing_arm_model()
    points = dict(base.hardpoints)
    point = points["WHEEL_CENTER"]
    points["WHEEL_CENTER"] = Vec3(x=point.x, y=point.y, z=point.z+20)
    moved = base.model_copy(update={"hardpoints": points})
    assert (_poses(_run(moved, (0.,)))[0, 2, 3]-_poses(_run(base, (0.,)))[0, 2, 3])*1000 == pytest.approx(20, abs=1e-4)


def test_an_attitude_perturbation_changes_initial_alignment():
    from suspension_multibody.schema import Vec3
    base = trailing_arm_model()
    moved = base.model_copy(update={"joints": tuple(j.model_copy(update={"axis_a": Vec3(x=.05, y=1, z=0),
        "axis_b": Vec3(x=.05, y=1, z=0)}) for j in base.joints)})
    runs = [_run(item, (10., 0., -10.)) for item in (base, moved)]
    angles = [frame_fields(_poses(run)[0], side="L")["left_camber_deg"] for run in runs]
    assert angles[0] != pytest.approx(angles[1], abs=1e-9)
    assert all(run.status == "success" for run in runs)


def test_the_proof_needs_no_core_edit():
    root = Path(__file__).parents[2] / "src/suspension_multibody"
    before = {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in
        ("api.py", "authoring/generic.py", "compilation/resolved.py", "simulation/runner.py", "results/envelope.py")}
    _run(travel=(0.,))
    assert before == {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in before}


def test_bench_name_does_not_change_model_identity():
    source = _bench_assembly()
    first, second = (validate(source, _bench_case(name)) for name in ("first", "another-bench"))
    assert first.model_payload == second.model_payload
    assert first.case_payload != second.case_payload


def test_the_synthetic_topology_reaches_a_dynamic_study():
    source = migrate_v1_axle(trailing_arm_model())
    case = {"schema_version": 1, "name": "dynamic", "study": "dynamic", "samples": [0, .0005, .001],
        "solver": {"initialization_mode": "provided_consistent_state"}, "boundaries": [], "inputs": [], "outputs": []}
    run = simulate(source, case)
    assert run.status == "success"
    assert len(run.result.times_s) == 3
    assert np.isfinite(run.raw.states).all()


def test_the_trailing_arm_has_no_implicit_steering():
    run = _run(travel=(0.,))
    assert not any("rack" in row["name"] for row in run.compiled.model_document["joints"])
    assert not any("brake" in row["name"] or "drive" in row["name"] for row in run.compiled.model_document["elements"])
