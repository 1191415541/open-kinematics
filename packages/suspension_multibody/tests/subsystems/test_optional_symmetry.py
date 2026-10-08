"""Mirroring and explicitly authored corners use the same ordinary data assembler."""

import numpy as np
import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
    assemble_generic,
    migrate_v1_axle,
    migrate_v1_kc_case,
)
from tests.benchmark_fixture import benchmark_model
from tests.subsystems.test_three_axle_assembly import _assembly, _case


def _corner(side):
    source, case = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(-20,), rack_values_mm=(0,))
    removed = "R" if side == "L" else "L"
    subsystems = {}
    for entry in source.entries:
        payload = entry.subsystem.template.to_payload()
        for table in ("bodies", "joints", "elements", "ports", "markers", "needs", "coordinates"):
            if table in payload:
                payload[table] = [row for row in payload[table] if not row["name"].endswith("_"+removed)
                    and not any(str(row.get(key, "")).endswith("_"+removed) for key in ("owner", "body_a", "body_b"))]
        payload["hardpoints"] = [row for row in payload["hardpoints"]
                                 if not str(row.get("owner", "")).endswith("_"+removed)
                                 and not row["name"].endswith("_"+removed)]
        template = TemplateDocument.from_payload(payload)
        values = entry.subsystem.to_payload()
        values["hardpoints"] = {key: value for key, value in values["hardpoints"].items() if key in template.hardpoint_names}
        subsystems[entry.ref] = SubsystemDocument.from_payload(values, template=template, properties=entry.subsystem.properties)
    assembly_payload = source.to_payload()
    for row in assembly_payload["subsystems"]:
        if "pairings" in row:
            row["pairings"] = [pair for pair in row["pairings"]
                if not str(pair.get("requirement_role", "")).endswith("_"+removed)
                and not str(pair.get("port", "")).endswith("_"+removed)]
    source = AssemblyDocument.from_payload(assembly_payload, subsystems=subsystems)
    run = case.to_payload()
    run["boundaries"] = [row for row in run["boundaries"] if not row["coordinate"].endswith("_"+removed)]
    run["excitation"]["k"]["axis_map"]["wheel"] = ["kc_rig.sub.json.wheel_drive_"+side]
    run["excitation"]["k"]["left_right_mode"] = "single"
    return source, run


def test_the_default_still_pairs_both_sides():
    source = migrate_v1_axle(benchmark_model())
    names = {row["name"] for row in assemble_generic(source).resolved_model().to_document()["bodies"]}
    for stem in ("upper_arm", "lower_arm", "upright", "tie_rod"):
        assert "model.sub.json."+stem+"_L" in names
        assert "model.sub.json."+stem+"_R" in names
    assert {"wheel.sub.json.wheel_hub_L", "wheel.sub.json.wheel_hub_R"} <= names
    mirrored = _assembly(("front",)).entries[1].subsystem.template
    assert mirrored.mirrors


def test_one_declared_side_is_a_topology_not_a_hole():
    source, case = _corner("L")
    graph = validate(source, case).model_document
    assert "model.sub.json.upright_L" in {row["name"] for row in graph["bodies"]}
    assert not any(row["name"].endswith("_R") for table in ("bodies", "joints", "markers") for row in graph[table])
    assert not any(row["name"].endswith("_R") for row in graph["elements"])


def test_a_one_sided_assembly_runs_a_quasi_static_study():
    source, case = _corner("L")
    result = simulate(source, case).result
    assert result.status == "success"
    assert np.isfinite(result.body_state("model.sub.json.upright_L")).all()
    assert not any(name.endswith("_R") for name in result.raw.body_names)
    center = result.frame_pose("wheel.sub.json.wheel_center_L")
    assert center[-1, 2, 3] == pytest.approx(.28, abs=1e-8)


def test_a_declared_right_side_overrides_the_mirror():
    source = benchmark_model()
    raw = source.model_dump(mode="json")
    raw["hardpoints"]["lca_outer__R"] = [-40, 740, 155]
    graph = assemble_generic(migrate_v1_axle(raw)).resolved_model().to_document()
    joints = {row["name"]: row for row in graph["joints"]}
    np.testing.assert_allclose(joints["model.sub.json.lower_arm_R_outer_joint"]["point_a"], [-.04, .74, .155])
    np.testing.assert_allclose(joints["model.sub.json.lower_arm_L_outer_joint"]["point_a"], [0, -.7, .15])
    np.testing.assert_allclose(joints["model.sub.json.uca_mount_R_inner_front"]["point_b"], [-.1, .5, .4])


def test_a_three_wheeled_vehicle_is_two_suspensions_and_one_corner():
    source = _assembly(("front", "rear"))
    subsystems = {entry.ref: entry.subsystem for entry in source.entries}
    rear = subsystems["rear"]
    template = rear.template.to_payload()
    template["symmetry"] = "asymmetric"
    template["bodies"][0]["name"] = "upright_R"
    for table in ("hardpoints", "markers", "ports"):
        for row in template[table]:
            if row.get("owner") == "upright":
                row["owner"] = "upright_R"
    template["joints"][0]["body_b"] = "upright_R"
    values = rear.to_payload()
    values["hardpoints"] = {key: [value[0], -value[1], value[2]] for key, value in values["hardpoints"].items()}
    subsystems["rear"] = SubsystemDocument.from_payload(values, template=TemplateDocument.from_payload(template))
    source = AssemblyDocument.from_payload(source.to_payload(), subsystems=subsystems)
    result = simulate(source, _case()).result
    assert {name for name in result.raw.body_names if "upright" in name} == {
        "front.upright_L", "front.upright_R", "rear.upright_R"}
    assert np.isfinite(result.body_state("rear.upright_R")).all()
    corner, case = _corner("R")
    right = simulate(corner, case).result
    assert right.status == "success"
    assert not any(name.endswith("_L") for name in right.raw.body_names)
    assert right.frame_pose("wheel.sub.json.wheel_center_R")[-1, 2, 3] == pytest.approx(.28, abs=1e-8)
