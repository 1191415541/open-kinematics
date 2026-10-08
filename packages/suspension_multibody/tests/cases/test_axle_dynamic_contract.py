"""Unified axle submission preserves every published result ledger."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
from suspension_contracts import unpack_container

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
from suspension_multibody.simulation.runner import run_compiled
from tests.axle_dynamics._frozen_result_projection import DIAGNOSTIC_FIELDS
from tests.axle_dynamics._unified_entry import solve_axle

_SCRIPT = Path(__file__).resolve().parents[2]/"scripts/run_axle_dynamics_acceptance.py"


def _module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def acceptance():
    return _module(_SCRIPT, "axle_acceptance_fixture")


def test_a_driven_coordinate_survives_the_contract_path():
    fixture = _module(Path(__file__).parents[1]/"axle_dynamics/test_driven_coordinate.py", "driven_fixture")
    model = fixture._translation_model(driven=(fixture._slide("slide"),), initial_velocity_m_per_s=(2., 0., 0.))
    case = fixture._case(samples=101, target=tuple(2.*.01*i for i in range(101)), rate=(2.,)*101)
    run = solve_axle(model, case)
    assert run.raw.block("constraint_wrench").shape == run.constraint_wrench.shape
    np.testing.assert_array_equal(run.raw.block("constraint_wrench"), run.constraint_wrench)
    np.testing.assert_array_equal(run.raw.block("body_state"), run.states)
    np.testing.assert_array_equal(run.raw.block("energy"), run.energy)
    assert any(row["type"] == "driven_translation" for row in run.compiled.model_document["joints"])


@pytest.mark.parametrize("case_name", ["static_equilibrium", "road_pulse", "opposite_phase_road", "braking", "combined_load", "tire_liftoff_and_recontact"])
def test_contract_ledgers_match_the_frozen_result_projection(acceptance, case_name):
    run = solve_axle(acceptance.build_axle_model(), acceptance.build_case(case_name))
    raw = run.raw
    assert raw.status == "success"
    assert [entry["name"] for entry in raw.cases] == [case_name]
    np.testing.assert_array_equal(raw.block("body_state"), run.states)
    for name, values in (("constraint_wrench", run.constraint_wrench), ("tire_output", run.tire_output),
        ("spring_output", run.spring_output), ("bushing_output", run.bushing_output), ("energy", run.energy)):
        if values.shape[1] == 0:
            assert name not in raw.blocks
        else:
            np.testing.assert_array_equal(raw.block(name), values)
    diagnostics = np.column_stack([getattr(run.diagnostics, field) for field in DIAGNOSTIC_FIELDS])
    np.testing.assert_array_equal(raw.block("diagnostics")[:len(run.times_s)], diagnostics)


def test_the_model_document_declares_metres(acceptance):
    assembly, case = migrate_v1_dynamic_axle(acceptance.build_axle_model(), acceptance.build_case("static_equilibrium"))
    assert validate(assembly, case).model_document["units"]["length"] == "m"


def test_the_case_document_describes_its_sample_tables(acceptance):
    assembly, case = migrate_v1_dynamic_axle(acceptance.build_axle_model(), acceptance.build_case("opposite_phase_road"))
    compiled = validate(assembly, case)
    document, blob = unpack_container(compiled.case_payload)
    assert document["family"] == "axle_dynamic"
    assert document["time"]["step_s"] == pytest.approx(.001)
    roles = [(row["role"], row.get("tire", "").rsplit(".", 1)[-1]) for row in document["blobs"]]
    assert sorted(roles) == sorted([("road_height", "tire_l"), ("road_velocity", "tire_l"),
        ("road_height", "tire_r"), ("road_velocity", "tire_r")])
    for row in document["blobs"]:
        assert row["offset"]+row["length"] <= len(blob)
        assert row["dtype"] == "float64"


def test_an_unknown_blob_role_is_rejected(acceptance):
    assembly, case = migrate_v1_dynamic_axle(acceptance.build_axle_model(), acceptance.build_case("road_pulse"))
    wrong = case.to_payload()
    wrong["inputs"][0]["role"] = "road_curvature"
    with pytest.raises(Exception, match="road_curvature"):
        simulate(assembly, wrong)


def test_compiled_containers_carry_the_declared_documents(acceptance):
    assembly, case = migrate_v1_dynamic_axle(acceptance.build_axle_model(), acceptance.build_case("static_equilibrium"))
    compiled = validate(assembly, case)
    assert unpack_container(compiled.model_payload)[0] == compiled.model_document
    assert unpack_container(compiled.case_payload)[0] == compiled.case_document
    assert compiled.metadata["compiler"] == "ResolvedModelCompiler"
    assert compiled.metadata["solve_plan"]["name"] == case.to_payload()["name"]


def test_file_and_memory_documents_have_identical_submissions(acceptance, tmp_path):
    from suspension_multibody.authoring.migration import save_migrated_assembly

    assembly, case = migrate_v1_dynamic_axle(acceptance.build_axle_model(), acceptance.build_case("road_pulse"))
    model_path = save_migrated_assembly(assembly, tmp_path)
    case_path = case.save(tmp_path/"case.json")
    memory, file = validate(assembly, case), validate(model_path, case_path)
    assert memory.model_document == file.model_document
    assert memory.case_document == file.case_document
    assert memory.model_payload == file.model_payload
    assert memory.case_payload == file.case_payload


@pytest.mark.parametrize("target,field,value", [("model", "document", "wrong"),
    ("model", "schema_version", -1), ("case", "schema_version", -1),
    ("case", "study", "invalid"), ("case", "protocol", "handling")])
def test_documents_reject_invalid_identity(acceptance, target, field, value):
    from suspension_multibody.authoring import AssemblyDocument

    assembly, case = migrate_v1_dynamic_axle(acceptance.build_axle_model(), acceptance.build_case("static_equilibrium"))
    model, plan = assembly.to_payload(), case.to_payload()
    (model if target == "model" else plan)[field] = value
    with pytest.raises(Exception):
        validate(AssemblyDocument.from_payload(model, subsystems={e.ref: e.subsystem for e in assembly.entries}), plan)


@pytest.mark.parametrize("staged", [False, True])
def test_document_request_submits_once_and_returns_the_same_envelope(acceptance, monkeypatch, staged):
    from suspension_multibody.simulation import runner

    assembly, case = migrate_v1_dynamic_axle(acceptance.build_axle_model(), acceptance.build_case("road_pulse"))
    original, submissions = runner.NativeContractBackend.run, []

    def submit(self, compiled):
        submissions.append(compiled)
        return original(self, compiled)

    monkeypatch.setattr(runner.NativeContractBackend, "run", submit)
    run = run_compiled(validate(assembly, case)) if staged else simulate(assembly, case)
    assert len(submissions) == 1
    assert run.status == "success"
    assert run.result.raw is run.raw
    assert run.raw.block("body_state").shape[0] == len(run.raw.times_s)
    assert len(run.raw.times_s) > 0
    assert np.isfinite(run.raw.block("body_state")).all()
