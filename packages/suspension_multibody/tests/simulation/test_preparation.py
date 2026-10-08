"""Preparation is the same loader/resolver for every declaration and protocol."""
import builtins
from copy import deepcopy

import pytest

from suspension_multibody import api
from suspension_multibody.authoring.errors import AuthoringError
from suspension_multibody.authoring.loader import DocumentLoader, LoadedDocuments
from suspension_multibody.compilation.resolved import compile_resolved, plan_from_case
from suspension_multibody.modeling.resolved import ResolvedModel
from suspension_multibody.simulation import run_compiled

from ._documents import PROTOCOLS, documents, synthetic_run


class CountingBackend:
    def __init__(self):
        self.submissions = 0
        self.compiled = None

    def run(self, compiled):
        self.submissions += 1
        self.compiled = compiled
        return synthetic_run(compiled)


@pytest.mark.parametrize("protocol", PROTOCOLS)
def test_every_protocol_loads_and_resolves_without_importing_business_preparation(monkeypatch, protocol):
    source, case = documents(protocol)
    original = builtins.__import__
    imports = []

    def guarded(name, *args, **kwargs):
        imports.append(name)
        assert not name.startswith("suspension_multibody.preparation")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", guarded)
    loaded = DocumentLoader().load(source, case)
    graph = loaded.resolve()
    assert isinstance(graph, ResolvedModel)
    assert loaded.assembly.to_payload() == source.to_payload()
    assert graph.name == source.name
    assert compile_resolved(graph, plan_from_case(case)).request.model is graph


def test_repeated_resolution_is_deterministic_without_self_reference():
    source, case = documents()
    loader = DocumentLoader()
    one, two = loader.load(source, case).resolve(), loader.load(source, case).resolve()
    assert one.to_document() == two.to_document()
    assert one.fingerprint == two.fingerprint
    assert one.resource_payload == two.resource_payload
    assert one.to_document()["schema_version"] == 1


def test_preparation_preserves_inputs_and_never_merges_hidden_context():
    source, case = documents()
    saved = deepcopy(case)
    one = api.validate(source, case)
    assert case == saved
    assert one.request.context == {}
    assert one.request.model.name == source.name
    assert one.request.case.to_document()["protocol"] == case["family"]


def test_facade_loads_resolves_compiles_and_submits_exactly_once(monkeypatch):
    source, case = documents()
    counts = {"load": 0, "resolve": 0, "compile": 0}
    original_load, original_resolve, original_compile = DocumentLoader.load, LoadedDocuments.resolve, api.compile_resolved

    def load(self, *args, **kwargs):
        counts["load"] += 1
        bundle = original_load(self, *args, **kwargs)
        return bundle

    def compile_once(model, plan):
        counts["compile"] += 1
        return original_compile(model, plan)

    def resolve_once(self):
        counts["resolve"] += 1
        return original_resolve(self)

    backend = CountingBackend()
    monkeypatch.setattr(DocumentLoader, "load", load)
    monkeypatch.setattr(LoadedDocuments, "resolve", resolve_once)
    monkeypatch.setattr(api, "compile_resolved", compile_once)
    monkeypatch.setattr(api, "run_compiled", lambda submitted: run_compiled(submitted, backend=backend))
    result = api.simulate(source, case)
    assert counts == {"load": 1, "resolve": 1, "compile": 1}
    assert backend.submissions == 1
    assert result.compiled is backend.compiled
    assert result.status == "success"


def test_staged_path_uses_the_resolved_model_once_and_submits_once():
    source, case = documents()
    bundle = DocumentLoader().load(source, case)
    model = bundle.resolve()
    compiled = compile_resolved(model, plan_from_case(bundle.case.to_payload()))
    backend = CountingBackend()
    result = run_compiled(compiled, backend=backend)
    assert result.compiled is compiled
    assert result.request.model is model
    assert backend.submissions == 1
    assert result.raw.model_document["name"] == model.name


def test_submitting_compiled_value_skips_loading_and_compilation(monkeypatch):
    compiled = api.validate(*documents())
    backend = CountingBackend()

    def refuse(*args, **kwargs):
        pytest.fail("a compiled submission must not be prepared or compiled again")

    monkeypatch.setattr(DocumentLoader, "load", refuse)
    monkeypatch.setattr(api, "compile_resolved", refuse)
    result = run_compiled(compiled, backend=backend)
    assert result.compiled is compiled
    assert backend.submissions == 1


def test_loader_failure_propagates_the_original_exception(monkeypatch):
    source, case = documents()
    error = RuntimeError("resource preparation exploded")

    def explode(*args, **kwargs):
        raise error

    monkeypatch.setattr(DocumentLoader, "load", explode)
    with pytest.raises(RuntimeError) as caught:
        api.validate(source, case)
    assert caught.value is error


@pytest.mark.parametrize("case", [
    {"kind": "case"},
    {"schema_version": 1, "name": "missing-study", "samples": [0, .001]},
])
def test_malformed_case_is_refused_before_submission(case):
    with pytest.raises(AuthoringError):
        api.validate(documents()[0], case)
