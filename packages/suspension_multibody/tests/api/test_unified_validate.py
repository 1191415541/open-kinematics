"""Validation produces the exact submission consumed by native execution."""

import json

from suspension_multibody import api
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from suspension_multibody.simulation import CompiledSimulation, run_compiled

from ..authoring.test_unified_document_loader import EXAMPLES


def test_validate_path_and_object_produce_identical_compiled_submission(tmp_path):
    bundle = DocumentLoader(resource_root=EXAMPLES).load("tire.assembly.json", "dynamic.case.json")
    case_path = tmp_path / "case.json"
    case_path.write_text(json.dumps(bundle.case.to_payload()), encoding="utf-8")
    file = api.validate(EXAMPLES / "tire.assembly.json", case_path)
    memory = api.validate(bundle.assembly, bundle.case.to_payload())
    assert isinstance(file, CompiledSimulation)
    assert file.model_payload == memory.model_payload
    assert file.case_payload == memory.case_payload
    assert run_compiled(file).status == "success"
    simulated = api.simulate(bundle.assembly, case_path)
    assert simulated.compiled.model_payload == memory.model_payload
    assert simulated.compiled.case_payload == memory.case_payload


def test_validate_and_generic_submit_share_compilation():
    bundle = DocumentLoader(resource_root=EXAMPLES).load("assembly.json", "dynamic.case.json")
    validated = api.validate(bundle.assembly, bundle.case.to_payload())
    compiled = compile_resolved(bundle.resolve(), plan_from_case(bundle.case.to_payload()))
    assert validated.model_payload == compiled.model_payload
    assert validated.case_payload == compiled.case_payload
    assert validated.metadata["model_fingerprint"] == compiled.metadata["model_fingerprint"]


def test_validate_does_not_submit_or_prepare_family_entities(monkeypatch):
    from suspension_multibody.kernel import capabilities
    from suspension_multibody.simulation import runner

    capabilities.kernel_capability_document()
    monkeypatch.setattr(runner, "run_compiled", lambda *_: (_ for _ in ()).throw(AssertionError("validation submitted")))
    bundle = DocumentLoader(resource_root=EXAMPLES).load("assembly.json", "dynamic.case.json")
    assert api.validate(bundle.assembly, bundle.case.to_payload()).metadata["compiler"] == "ResolvedModelCompiler"
