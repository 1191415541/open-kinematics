"""The common compiler preserves physical declarations and actual rig drives."""

import pytest

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring import assemble_generic, migrate_v1_axle
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from suspension_multibody.schema import BumpStop, LinearSpring, StaticDamper, Vec3
from tests.authoring.test_generic_multibody import _case
from tests.benchmark_fixture import benchmark_model


def elements(**overrides):
    model = benchmark_model().model_copy(update=overrides)
    return assemble_generic(migrate_v1_axle(model)).model_document()["elements"]


def test_spring_and_damper_survive_the_only_compiler_with_si_properties():
    point, other = Vec3(y=-600, z=100), Vec3(y=-600, z=400)
    rows = elements(springs=(LinearSpring(name="corner_spring", body_a="chassis", body_b="lower_arm_L",
        point_a=point, point_b=other, stiffness=200, free_length=250),),
        dampers=(StaticDamper(name="corner_damper", body_a="chassis", body_b="lower_arm_L",
        point_a=point, point_b=other, viscous_damping=12),))
    springs = [row for row in rows if row["type"] == "spring"]
    dampers = [row for row in rows if row["type"] == "damper"]
    assert {row["name"].rsplit(".", 1)[1] for row in springs} == {"corner_spring_L", "corner_spring_R"}
    assert {row["name"].rsplit(".", 1)[1] for row in dampers} == {"corner_damper_L", "corner_damper_R"}
    assert all(row["parameters"]["stiffness"] == 200000 for row in springs)
    assert all(row["parameters"]["compression_damping"] == row["parameters"]["rebound_damping"] == 12000 for row in dampers)


def test_preload_preserves_the_original_zero_force_length():
    rows = elements(springs=(LinearSpring(name="preloaded", body_a="chassis", body_b="lower_arm_L",
        point_a=Vec3(y=-600, z=100), point_b=Vec3(y=-600, z=400), stiffness=200,
        reference_length=300, preload=1000),))
    assert len(rows) == 2
    assert all(row["parameters"]["free_length"] == pytest.approx(.295) for row in rows)
    assert all(row["parameters"]["stiffness"] == 200000 for row in rows)


def test_bump_stop_is_a_distinct_unilateral_record():
    rows = elements(stops=(BumpStop(name="stop", body_a="chassis", body_b="lower_arm_L",
        point_a=Vec3(y=-600, z=120), point_b=Vec3(y=-600, z=380), clearance=20, stiffness=500),))
    stops = [row for row in rows if row["type"] == "bump_stop"]
    assert {row["name"].rsplit(".", 1)[1] for row in stops} == {"stop_L", "stop_R"}
    assert all(row["parameters"]["clearance"] == .02 and row["parameters"]["stiffness"] == 500000 and row["parameters"]["direction"] == 1 for row in stops)


def test_public_run_uses_the_declared_rig_and_resolved_model(monkeypatch):
    import suspension_multibody.api as api
    original = api.compile_resolved
    seen = []
    def record(model, plan):
        seen.append(model.fingerprint)
        return original(model, plan)
    monkeypatch.setattr(api, "compile_resolved", record)
    source, case = migrate_v1_kc_case(benchmark_model(), mode="K", wheel_values_mm=(-10, 0, 10))
    run = simulate(source, case)
    assert run.status == "success" and len(run.result.cases) == 3
    assert seen == [run.result.model_fingerprint]
    assert any(row["name"].startswith("kc_rig.sub.json.") for row in run.compiled.model_document["joints"])


def test_legacy_model_is_not_a_second_execution_entry():
    with pytest.raises(AuthoringError, match="AssemblyDocument"):
        validate(benchmark_model(), _case())
