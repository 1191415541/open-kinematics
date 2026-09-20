"""
Protocol, registry and lifecycle tests for unified preparation.

The six non-vehicle families and ``preparation/vehicle_dynamic.py`` do not
exist yet, so every test here injects a ``PreparationRegistry``/``Preparation``
double or monkeypatches the lazy import seam.  Nothing here catches or rewrites
``ImportError``/``ModuleNotFoundError``, and no production placeholder family is
introduced.
"""

from __future__ import annotations

from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.kernel import ContractRun
from suspension_multibody.simulation import (
    CompilerRegistry,
    DocumentPairCompiler,
    Preparation,
    PreparationRegistry,
    PreparedSimulation,
    SimulationRequest,
    compile_request,
    default_preparation_registry,
    prepare_request,
    run_compiled,
    run_request,
)
from suspension_multibody.simulation import preparation as preparation_module

AXLE_DYNAMIC_MODULE = "suspension_multibody.preparation.axle_dynamic"
KC_QUASI_STATIC_MODULE = "suspension_multibody.preparation.kc_quasi_static"
VEHICLE_KC_MODULE = "suspension_multibody.preparation.vehicle_kc"
HANDLING_MODULE = "suspension_multibody.preparation.handling"
RIDE_FOUR_POST_MODULE = "suspension_multibody.preparation.ride_four_post"
RIDE_RANDOM_ROAD_MODULE = "suspension_multibody.preparation.ride_random_road"
VEHICLE_DYNAMIC_MODULE = "suspension_multibody.preparation.vehicle_dynamic"

REQUIRED_KEYS = {
    ("axle", "axle_dynamic"),
    ("axle", "kc_quasi_static"),
    ("vehicle", "vehicle_kc"),
    ("vehicle", "handling"),
    ("vehicle", "ride_four_post"),
    ("vehicle", "ride_random_road"),
    ("vehicle", "vehicle_dynamic"),
}

KEY_MODULES = {
    ("axle", "axle_dynamic"): AXLE_DYNAMIC_MODULE,
    ("axle", "kc_quasi_static"): KC_QUASI_STATIC_MODULE,
    ("vehicle", "vehicle_kc"): VEHICLE_KC_MODULE,
    ("vehicle", "handling"): HANDLING_MODULE,
    ("vehicle", "ride_four_post"): RIDE_FOUR_POST_MODULE,
    ("vehicle", "ride_random_road"): RIDE_RANDOM_ROAD_MODULE,
    ("vehicle", "vehicle_dynamic"): VEHICLE_DYNAMIC_MODULE,
}


class FakeImporter:
    """Stand-in for the lazy import seam that records requested module names."""

    def __init__(self) -> None:
        self.requested: list[str] = []
        self.modules: dict[str, object] = {}

    def add(self, module_name: str, **attributes: object) -> None:
        self.modules[module_name] = SimpleNamespace(**attributes)

    def __call__(self, module_name: str):
        self.requested.append(module_name)
        try:
            return self.modules[module_name]
        except KeyError:
            raise ModuleNotFoundError(
                f"No module named {module_name!r}", name=module_name
            ) from None

    def install(self, monkeypatch) -> FakeImporter:
        monkeypatch.setattr(preparation_module, "import_module", self)
        return self


class FakePreparation:
    """Injected preparation double that records the requests it prepared."""

    def __init__(
        self,
        assembly: str,
        family: str,
        *,
        value: object = None,
        context: dict[str, object] | None = None,
        metadata: dict[str, object] | None = None,
    ) -> None:
        self.assembly = assembly
        self.family = family
        self.value = value
        self.context = {} if context is None else dict(context)
        self.metadata = {} if metadata is None else dict(metadata)
        self.calls: list[SimulationRequest] = []

    def prepare(self, request: SimulationRequest) -> PreparedSimulation:
        self.calls.append(request)
        return PreparedSimulation(
            request=request,
            value=self.value,
            context=dict(self.context),
            metadata=dict(self.metadata),
        )


class ExplodingRegistry:
    """Registry double that fails if preparation consults it."""

    def resolve(self, assembly: str, family: str):
        raise AssertionError(
            f"preparation registry must not be consulted for {assembly}/{family}"
        )


class CountingBackend:
    """Backend double that counts native submission attempts."""

    def __init__(self) -> None:
        self.submissions = 0
        self.compiled = None

    def run(self, compiled):
        self.submissions += 1
        self.compiled = compiled
        return ContractRun(
            document={"status": "success"},
            blocks={"body_state": np.zeros((1, 1, 19))},
            model_document=compiled.model_document,
            case_document=compiled.case_document,
            times_s=np.array([0.0]),
        )


def _registry_with(*preparations: FakePreparation) -> PreparationRegistry:
    registry = PreparationRegistry()
    for preparation in preparations:
        registry.register(preparation)
    return registry


def _probe_fixture() -> tuple[
    FakePreparation, PreparationRegistry, CompilerRegistry, SimulationRequest
]:
    preparation = FakePreparation(
        "axle",
        "probe",
        value="prepared-value",
        context={
            "model_document": {"kind": "model"},
            "case_document": {"kind": "case"},
        },
    )
    compiler_registry = CompilerRegistry()
    compiler_registry.register(DocumentPairCompiler("axle", "probe"))
    request = SimulationRequest(
        assembly="axle", family="probe", model=object(), case=object()
    )
    return preparation, _registry_with(preparation), compiler_registry, request


def test_registry_normalizes_keys_and_rejects_empty_or_unknown_entries() -> None:
    registry = PreparationRegistry()
    preparation = FakePreparation(" Axle ", "AXLE_DYNAMIC")

    registry.register(preparation)

    assert registry.keys() == (("axle", "axle_dynamic"),)
    assert registry.resolve("AXLE", "  Axle_Dynamic ") is preparation
    assert ("axle", "axle_dynamic") in registry

    with pytest.raises(KeyError, match="already registered"):
        registry.register(FakePreparation("axle", "axle_dynamic"))
    registry.register(FakePreparation("axle", "axle_dynamic"), replace=True)
    assert registry.keys() == (("axle", "axle_dynamic"),)

    for assembly, family in (("", "axle_dynamic"), ("axle", "   "), (None, "x")):
        with pytest.raises(ValueError, match="must not be empty"):
            registry.register(FakePreparation(assembly, family))

    with pytest.raises(ValueError, match="must not be empty"):
        registry.resolve("vehicle", "   ")

    with pytest.raises(KeyError) as raised:
        registry.resolve(" Vehicle ", "Handling")
    message = str(raised.value)
    assert "no preparation registered" in message
    assert "vehicle" in message and "handling" in message


def test_default_registry_registers_seven_keys_without_importing_families(
    monkeypatch,
) -> None:
    importer = FakeImporter().install(monkeypatch)

    registry = default_preparation_registry()

    assert set(registry.keys()) == REQUIRED_KEYS
    assert len(registry.keys()) == 7
    assert registry.keys() == tuple(sorted(REQUIRED_KEYS))
    assert importer.requested == []

    for key in sorted(REQUIRED_KEYS):
        entry = registry.resolve(*key)
        assert (entry.assembly, entry.family) == key
    assert importer.requested == []


@pytest.mark.parametrize(("key", "module"), sorted(KEY_MODULES.items()))
def test_default_registry_lazily_imports_the_matrix_module_per_key(
    monkeypatch, key: tuple[str, str], module: str
) -> None:
    importer = FakeImporter().install(monkeypatch)
    seen: list[SimulationRequest] = []

    def prepare(request: SimulationRequest) -> PreparedSimulation:
        seen.append(request)
        return PreparedSimulation(request=request, value="prepared")

    importer.add(module, prepare_request=prepare)
    registry = default_preparation_registry()
    request = SimulationRequest(key[0], key[1], model=object(), case=object())

    assert importer.requested == []
    preparation = registry.resolve(*key)
    assert importer.requested == []

    prepared = preparation.prepare(request)

    assert importer.requested == [module]
    assert seen == [request]
    assert prepared.value == "prepared"


def test_preparation_protocol_accepts_family_shaped_objects() -> None:
    assert isinstance(FakePreparation("axle", "axle_dynamic"), Preparation)
    assert isinstance(
        default_preparation_registry().resolve("vehicle", "handling"), Preparation
    )


def test_prepare_request_merges_preparation_context_over_request_context() -> None:
    preparation = FakePreparation(
        "vehicle",
        "handling",
        value="prepared-value",
        context={"case_document": {"kind": "case"}, "shared": "preparation"},
        metadata={"source": "fake"},
    )
    request = SimulationRequest(
        assembly="vehicle",
        family="handling",
        model=object(),
        case=object(),
        context={"shared": "request", "extra": 1},
    )

    prepared = prepare_request(request, registry=_registry_with(preparation))

    assert preparation.calls == [request]
    assert prepared.value == "prepared-value"
    assert prepared.metadata == {"source": "fake"}
    assert prepared.request is not request
    assert prepared.request.assembly == "vehicle"
    assert prepared.request.family == "handling"
    assert prepared.request.model is request.model
    assert prepared.request.case is request.case
    assert prepared.request.context["extra"] == 1
    assert prepared.request.context["shared"] == "preparation"
    assert prepared.request.context["case_document"] == {"kind": "case"}

    injected = prepared.request.context["prepared_simulation"]
    assert isinstance(injected, PreparedSimulation)
    assert injected is prepared.context["prepared_simulation"]
    assert injected is not prepared
    assert injected.request is request
    assert "prepared_simulation" not in injected.request.context


def test_repeated_preparation_stays_equal_without_self_reference() -> None:
    preparation = FakePreparation(
        "vehicle",
        "handling",
        value="prepared-value",
        context={"case_document": {"kind": "case"}},
    )
    registry = _registry_with(preparation)
    request = SimulationRequest(
        assembly="vehicle", family="handling", model=object(), case=object()
    )

    first = prepare_request(request, registry=registry)
    second = prepare_request(request, registry=registry)

    # A self-referential prepared_simulation would recurse here instead.
    assert first == second
    assert first.request.context["prepared_simulation"] is not first


@pytest.mark.parametrize(
    "context",
    [
        {"model_document": {"kind": "model"}},
        {"case_document": {"kind": "case"}},
        {"model_document_pair": ({"kind": "model"}, b"model-payload")},
        {"model_payload": b"model-payload"},
        {"case_payload": b"case-payload"},
    ],
)
def test_document_context_keys_bypass_preparation(monkeypatch, context) -> None:
    importer = FakeImporter().install(monkeypatch)
    request = SimulationRequest(
        assembly="axle",
        family="axle_dynamic",
        model=object(),
        case=object(),
        context=context,
    )

    prepared = prepare_request(request, registry=ExplodingRegistry())

    assert prepared.request is request
    assert prepared.value is None
    assert prepared.context == context
    assert importer.requested == []


def test_model_or_case_contract_documents_bypass_preparation(monkeypatch) -> None:
    importer = FakeImporter().install(monkeypatch)
    model_document = {"contract": "multibody-model", "kind": "model"}
    case_document = {"contract": "multibody-case", "kind": "case"}
    document_request = SimulationRequest(
        assembly="vehicle", family="handling", model=model_document, case=case_document
    )
    pair_request = SimulationRequest(
        assembly="vehicle",
        family="handling",
        model=(model_document, b"model-payload"),
        case=case_document,
    )

    for request in (document_request, pair_request):
        prepared = prepare_request(request, registry=ExplodingRegistry())
        assert prepared.request is request
        assert prepared.value is None

    assert importer.requested == []


def test_document_request_bypasses_the_legacy_prepared_adapter(monkeypatch) -> None:
    importer = FakeImporter().install(monkeypatch)
    adapter_calls: list[SimulationRequest] = []
    importer.add(
        VEHICLE_DYNAMIC_MODULE,
        adapt_legacy_prepared_request=lambda request: adapter_calls.append(request),
    )
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=object(),
        case=object(),
        context={"prepared": object(), "case_document": {"kind": "case"}},
    )

    prepared = prepare_request(request, registry=ExplodingRegistry())

    assert prepared.request is request
    assert adapter_calls == []
    assert importer.requested == []


def test_legacy_prepared_is_wrapped_then_reused_without_family_preparation(
    monkeypatch,
) -> None:
    importer = FakeImporter().install(monkeypatch)
    model, case = object(), object()
    legacy_prepared = object()
    adapter_calls: list[SimulationRequest] = []
    wrapped = PreparedSimulation(
        request=SimulationRequest(
            assembly="vehicle",
            family="vehicle_dynamic",
            model=model,
            case=case,
            context={"prepared": legacy_prepared},
        ),
        value=legacy_prepared,
        context={"vehicle_dynamic_prepared": legacy_prepared},
    )

    def adapt(request: SimulationRequest) -> SimulationRequest:
        adapter_calls.append(request)
        return replace(
            request,
            context={
                **request.context,
                "prepared_simulation": wrapped,
                "vehicle_dynamic_prepared": legacy_prepared,
            },
        )

    def refuse(request: SimulationRequest) -> PreparedSimulation:
        raise AssertionError("wrapped legacy prepared must be reused, not re-prepared")

    importer.add(
        VEHICLE_DYNAMIC_MODULE,
        adapt_legacy_prepared_request=adapt,
        prepare_request=refuse,
    )
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=model,
        case=case,
        context={"prepared": legacy_prepared},
    )

    prepared = prepare_request(request, registry=ExplodingRegistry())

    assert importer.requested == [VEHICLE_DYNAMIC_MODULE]
    assert adapter_calls == [request]
    assert prepared.value is legacy_prepared
    assert prepared.request.context["vehicle_dynamic_prepared"] is legacy_prepared
    assert prepared.request.context["prepared"] is legacy_prepared
    assert prepared.request.context["prepared_simulation"] is wrapped
    assert prepared.request.model is model
    assert prepared.request.case is case


def test_wrapped_legacy_prepared_is_reused_from_a_later_request(monkeypatch) -> None:
    importer = FakeImporter().install(monkeypatch)
    model, case = object(), object()
    legacy_prepared = object()
    stored = PreparedSimulation(
        request=SimulationRequest(
            assembly="vehicle", family="vehicle_dynamic", model=model, case=case
        ),
        value=legacy_prepared,
        context={"vehicle_dynamic_prepared": legacy_prepared},
    )
    importer.add(
        VEHICLE_DYNAMIC_MODULE,
        adapt_legacy_prepared_request=lambda request: pytest.fail(
            "a request without the legacy key must not reach the adapter"
        ),
        prepare_request=lambda request: pytest.fail(
            "matching prepared_simulation must be reused"
        ),
    )
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=model,
        case=case,
        context={"prepared_simulation": stored},
    )

    prepared = prepare_request(request, registry=ExplodingRegistry())

    assert prepared.value is legacy_prepared
    assert prepared.request.context["prepared_simulation"] is stored
    assert importer.requested == []


def test_legacy_prepared_key_is_ignored_by_other_families(monkeypatch) -> None:
    importer = FakeImporter().install(monkeypatch)
    preparation = FakePreparation("vehicle", "handling", value="prepared-value")
    request = SimulationRequest(
        assembly="vehicle",
        family="handling",
        model=object(),
        case=object(),
        context={"prepared": object()},
    )

    prepared = prepare_request(request, registry=_registry_with(preparation))

    assert preparation.calls == [request]
    assert prepared.value == "prepared-value"
    assert importer.requested == []


def test_matching_prepared_simulation_is_reused_without_preparation() -> None:
    model, case = object(), object()
    stored = PreparedSimulation(
        request=SimulationRequest(
            assembly="vehicle", family="handling", model=model, case=case
        ),
        value="stored-value",
        context={"case_document": {"kind": "case"}},
        metadata={"source": "stored"},
    )
    request = SimulationRequest(
        assembly="vehicle",
        family="handling",
        model=model,
        case=case,
        context={"prepared_simulation": stored, "extra": 1},
    )

    prepared = prepare_request(request, registry=ExplodingRegistry())

    assert prepared.value == "stored-value"
    assert prepared.metadata == {"source": "stored"}
    assert prepared.request.context["extra"] == 1
    assert prepared.request.context["case_document"] == {"kind": "case"}
    assert prepared.request.context["prepared_simulation"] is stored
    assert prepared.request.model is model
    assert prepared.request.case is case


def test_stale_or_mismatched_prepared_simulation_is_reprepared() -> None:
    fresh = FakePreparation("vehicle", "handling", value="fresh-value")
    registry = _registry_with(fresh)
    model, case = object(), object()
    other_model, other_case = object(), object()
    stored = PreparedSimulation(
        request=SimulationRequest(
            assembly="vehicle", family="handling", model=model, case=case
        ),
        value="stale-value",
    )
    stale_contexts = [
        {"prepared_simulation": "not-a-prepared-simulation"},
        {"prepared_simulation": replace(stored, request=replace(stored.request, model=other_model))},
        {"prepared_simulation": replace(stored, request=replace(stored.request, case=other_case))},
        {
            "prepared_simulation": PreparedSimulation(
                request=SimulationRequest(
                    assembly="axle", family="handling", model=model, case=case
                ),
                value="other-family",
            )
        },
        {
            "prepared_simulation": PreparedSimulation(
                request=SimulationRequest(
                    assembly="vehicle", family="vehicle_kc", model=model, case=case
                ),
                value="other-family",
            )
        },
    ]

    for context in stale_contexts:
        fresh.calls.clear()
        request = SimulationRequest(
            assembly="vehicle",
            family="handling",
            model=model,
            case=case,
            context=context,
        )

        prepared = prepare_request(request, registry=registry)

        assert fresh.calls == [request], context
        assert prepared.value == "fresh-value"


def test_facade_prepares_compiles_and_submits_exactly_once() -> None:
    preparation, preparation_registry, compiler_registry, request = _probe_fixture()
    backend = CountingBackend()

    run = run_request(
        request,
        preparation_registry=preparation_registry,
        registry=compiler_registry,
        backend=backend,
    )

    assert preparation.calls == [request]
    assert backend.submissions == 1
    assert run.status == "success"
    assert run.request.assembly == "axle"
    assert run.request.family == "probe"


def test_staged_path_prepares_compiles_and_submits_exactly_once() -> None:
    preparation, preparation_registry, compiler_registry, request = _probe_fixture()
    backend = CountingBackend()

    prepared = prepare_request(request, registry=preparation_registry)
    compiled = compile_request(prepared.request, registry=compiler_registry)
    run = run_compiled(compiled, backend=backend)

    assert preparation.calls == [request]
    assert backend.submissions == 1
    assert run.compiled is compiled
    assert prepared.request.context["model_document"] == {"kind": "model"}


def test_compiled_request_submission_skips_preparation_and_compilation() -> None:
    preparation, preparation_registry, compiler_registry, request = _probe_fixture()
    prepared = prepare_request(request, registry=preparation_registry)
    compiled = compile_request(prepared.request, registry=compiler_registry)
    backend = CountingBackend()

    run = run_request(
        compiled,
        preparation_registry=ExplodingRegistry(),
        registry=CompilerRegistry(),
        backend=backend,
    )

    assert preparation.calls == [request]
    assert backend.submissions == 1
    assert run.compiled is compiled
    assert run.status == "success"


def test_facade_document_request_never_consults_preparation() -> None:
    compiler_registry = CompilerRegistry()
    compiler_registry.register(DocumentPairCompiler("axle", "probe"))
    backend = CountingBackend()
    request = SimulationRequest(
        assembly="axle",
        family="probe",
        context={
            "model_document": {"kind": "model"},
            "case_document": {"kind": "case"},
        },
    )

    run = run_request(
        request,
        preparation_registry=ExplodingRegistry(),
        registry=compiler_registry,
        backend=backend,
    )

    assert run.request is request
    assert backend.submissions == 1


def test_missing_preparation_registration_raises_key_error_with_the_key() -> None:
    request = SimulationRequest(
        assembly="vehicle", family="handling", model=object(), case=object()
    )

    with pytest.raises(KeyError) as raised:
        prepare_request(request, registry=PreparationRegistry())

    message = str(raised.value)
    assert "no preparation registered" in message
    assert "vehicle" in message and "handling" in message


def test_missing_family_module_error_is_propagated_unchanged(monkeypatch) -> None:
    importer = FakeImporter().install(monkeypatch)
    request = SimulationRequest(
        assembly="vehicle", family="handling", model=object(), case=object()
    )

    with pytest.raises(ModuleNotFoundError) as raised:
        prepare_request(request)

    assert type(raised.value) is ModuleNotFoundError
    assert raised.value.name == HANDLING_MODULE
    assert importer.requested == [HANDLING_MODULE]


def test_missing_legacy_adapter_module_error_is_propagated_unchanged(
    monkeypatch,
) -> None:
    importer = FakeImporter().install(monkeypatch)
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=object(),
        case=object(),
        context={"prepared": object()},
    )

    with pytest.raises(ModuleNotFoundError) as raised:
        prepare_request(request, registry=ExplodingRegistry())

    assert type(raised.value) is ModuleNotFoundError
    assert raised.value.name == VEHICLE_DYNAMIC_MODULE
    assert importer.requested == [VEHICLE_DYNAMIC_MODULE]


def test_preparation_failure_propagates_the_original_exception(monkeypatch) -> None:
    importer = FakeImporter().install(monkeypatch)

    def explode(request: SimulationRequest) -> PreparedSimulation:
        raise RuntimeError("preparation exploded")

    importer.add(HANDLING_MODULE, prepare_request=explode)
    request = SimulationRequest(
        assembly="vehicle", family="handling", model=object(), case=object()
    )

    with pytest.raises(RuntimeError, match="preparation exploded"):
        prepare_request(request)


def test_preparation_must_return_prepared_simulation(monkeypatch) -> None:
    importer = FakeImporter().install(monkeypatch)
    importer.add(HANDLING_MODULE, prepare_request=lambda request: object())
    request = SimulationRequest(
        assembly="vehicle", family="handling", model=object(), case=object()
    )

    with pytest.raises(TypeError, match="must return PreparedSimulation"):
        prepare_request(request)


def test_run_request_propagates_compiler_validation_and_lookup_errors() -> None:
    preparation, preparation_registry, _, request = _probe_fixture()
    validating = CompilerRegistry()
    validating.register(
        DocumentPairCompiler("axle", "probe", validate_contract_family=True)
    )

    with pytest.raises(ValueError, match="multibody-model document"):
        run_request(
            request,
            preparation_registry=preparation_registry,
            registry=validating,
            backend=CountingBackend(),
        )

    with pytest.raises(KeyError, match="no compiler registered"):
        run_request(
            request,
            preparation_registry=preparation_registry,
            registry=CompilerRegistry(),
            backend=CountingBackend(),
        )

    assert preparation.calls == [request, request]
