"""Stable native constraint and element channels share one result surface."""

from dataclasses import replace

import numpy as np
import pytest
from suspension_contracts import pack_container, unpack_container

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.presets import generic_template
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.results.raw import RawContractResult
from suspension_multibody.simulation.runner import run_compiled

from ..authoring.test_unified_document_loader import EXAMPLES
from ..authoring.test_unified_subsystem_templates import (
    assembly,
    carrier_subsystem,
    subsystem,
)
from ..physics.test_wheel_spin_boundary import compiled_motion


def test_lock_reaction_and_multiplier_are_identified_and_balance_drive():
    run = run_compiled(compiled_motion(torque=7))
    result = run.result
    assert isinstance(result, ResultEnvelope)
    assert result.constraint_ids == ("wheel.bearing", "lock")
    reaction = result.constraint_wrench("lock", end="a")
    opposite = result.constraint_wrench("lock", end="b")
    assert reaction.body_id == "wheel.wheel"
    assert opposite.body_id == "support.carrier"
    np.testing.assert_allclose(reaction.moment[:, 1], -7, atol=1e-8)
    np.testing.assert_allclose(reaction.force+opposite.force, 0, atol=1e-12)
    np.testing.assert_allclose(reaction.moment+opposite.moment, 0, atol=1e-12)
    np.testing.assert_allclose(reaction.power, 0, atol=1e-12)
    multiplier = result.constraint_multiplier("lock")
    assert multiplier.units == "Nm"
    np.testing.assert_allclose(multiplier.values[:, 0], reaction.moment[:, 1], atol=1e-8)
    drive = result.element_wrench("drive", body_id="wheel.wheel")
    np.testing.assert_allclose(drive.moment+reaction.moment, 0, atol=1e-8)
    with pytest.raises(ValueError):
        reaction.moment[0, 0] = 9


def test_multibody_artifact_retains_stable_channels_and_all_native_blocks(tmp_path):
    import json

    from suspension_multibody.io.artifacts import read_artifact, write_artifact

    run = run_compiled(compiled_motion(torque=7))
    result = run.result
    path = write_artifact(result, tmp_path, model=result.model, case=run.compiled.request.case)
    artifact = read_artifact(path)
    manifest = artifact["manifest"]
    assert manifest["artifact_type"] == "multibody_result"
    assert manifest["model_fingerprint"] == result.model_fingerprint
    assert {row["id"] for row in manifest["channels"]["constraints"]} == set(result.constraint_ids)
    assert set(artifact["arrays"]) == {"times_s", *result.named_blocks}
    for name, expected in result.named_blocks.items():
        np.testing.assert_array_equal(artifact["arrays"][name], expected)
    native = json.loads((tmp_path/manifest["result_file"]).read_text(encoding="utf-8"))
    assert native["status"] == result.status


def test_prescribed_motion_reports_constraint_work_with_world_reference():
    speed = 4
    run = run_compiled(compiled_motion("prescribed_angle", torque=7, omega=speed,
        values=[0, .004, .008], rates=[speed]*3))
    result = run.result
    assert isinstance(result, ResultEnvelope)
    reaction = result.constraint_wrench("lock", end="a")
    assert reaction.frame_id == "world"
    assert reaction.moment_reference == "body_com"
    assert reaction.units == ("N", "Nm", "m", "W")
    np.testing.assert_allclose(reaction.power, -7*speed, atol=1e-7)
    drive = result.element_wrench("drive", body_id="wheel.wheel")
    np.testing.assert_allclose(drive.power+reaction.power, 0, atol=1e-7)
    np.testing.assert_allclose(result.constraint_wrench("lock", end="b").power, 0, atol=1e-12)


def test_channel_sampling_is_read_only_and_does_not_change_law_state():
    compiled = compiled_motion(torque=7)
    observed = run_compiled(compiled)
    payload_document, payload_blob = unpack_container(compiled.case_payload)
    document = dict(payload_document)
    document["outputs"] = {"constraint_reaction": False, "element_wrench": False}
    silent = run_compiled(replace(compiled, case_document=document, case_payload=pack_container(document, payload_blob)))
    for name, values in silent.raw.blocks.items():
        np.testing.assert_array_equal(values, observed.raw.blocks[name])
    result = observed.result
    assert isinstance(result, ResultEnvelope)
    before = result.law_state("wheel.tire")
    result.constraint_multiplier("lock")
    result.element_wrench("drive", body_id="wheel.wheel")
    after = result.law_state("wheel.tire")
    for name in before:
        np.testing.assert_array_equal(before[name], after[name])


def test_unknown_entity_or_channel_version_is_rejected():
    run = run_compiled(compiled_motion())
    result = run.result
    assert isinstance(result, ResultEnvelope)
    with pytest.raises(KeyError, match="unknown constraint"):
        result.constraint_wrench("other.lock", end="a")
    with pytest.raises(KeyError, match="unknown body"):
        result.body_state("other.wheel")
    document = dict(run.raw.document)
    document["manifest"] = {**run.raw.metadata, "constraint_channel_version": 42}
    with pytest.raises(ValueError, match="channel version"):
        ResultEnvelope(RawContractResult(document, run.raw.blocks), result.model)


def test_file_and_python_sources_have_identical_entity_channels():
    loader = DocumentLoader(resource_root=EXAMPLES)
    file = loader.load("tire.assembly.json", "dynamic.case.json")
    memory = loader.load(file.assembly, file.case)
    runs = [run_compiled(compile_resolved(source.resolve(), plan_from_case(source.case.to_payload())))
        for source in (file, memory)]
    assert all(isinstance(run.result, ResultEnvelope) for run in runs)
    assert runs[0].result.body_ids == runs[1].result.body_ids
    assert runs[0].result.constraint_ids == runs[1].result.constraint_ids
    assert runs[0].result.model_fingerprint == runs[1].result.model_fingerprint
    assert runs[0].raw.blocks.keys() == runs[1].raw.blocks.keys()
    for name, values in runs[0].raw.blocks.items():
        np.testing.assert_array_equal(values, runs[1].raw.blocks[name])
    assert runs[0].result.case_samples(0) == slice(0, len(runs[0].raw.states))
    np.testing.assert_array_equal(runs[0].result.energy, runs[0].raw.block("energy"))
    assert runs[0].result.contact_events == ()


def test_repeated_local_names_keep_full_instance_ids():
    source = assembly({"first.support": carrier_subsystem(), "second.support": carrier_subsystem(),
        "first.wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]}),
        "second.wheel": subsystem(generic_template("wheel"), {"center": [0, 0, .334]})},
        pairings={f"{name}.wheel": {"carrier": f"{name}.support.carrier", "road": f"{name}.support.road"}
            for name in ("first", "second")})
    model = assemble_generic(source).resolved_model()
    plan = ResolvedSolvePlan({"schema_version": 1, "name": "pair", "study": "dynamic", "samples": [0, .001],
        "solver": {"initialization_mode": "provided_consistent_state"}, "inputs": [], "outputs": [],
        "boundaries": [{"name": f"{name}.lock", "coordinate": f"{name}.wheel.spin", "mode": "locked", "units": "rad", "value": 0}
            for name in ("first", "second")]})
    result = run_compiled(compile_resolved(model, plan)).result
    assert isinstance(result, ResultEnvelope)
    assert set(result.constraint_ids) == {"first.wheel.bearing", "second.wheel.bearing", "first.lock", "second.lock"}
    for name in ("first", "second"):
        assert result.constraint_wrench(f"{name}.lock", end="a").body_id == f"{name}.wheel.wheel"
