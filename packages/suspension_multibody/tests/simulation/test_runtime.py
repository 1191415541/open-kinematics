from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.kernel import ContractRun, KernelContractError
from suspension_multibody.results.raw import RawContractResult
from suspension_multibody.simulation import (
    CompilerRegistry,
    DocumentPairCompiler,
    HandlingCompiler,
    KcQuasiStaticCompiler,
    RideFourPostCompiler,
    RideRandomRoadCompiler,
    SimulationRequest,
    VehicleKcCompiler,
    compile_document_pair,
    default_registry,
    run_compiled,
    run_request,
)


class FakeBackend:
    def __init__(self) -> None:
        self.compiled = None

    def run(self, compiled):
        self.compiled = compiled
        return ContractRun(
            document={"status": "success"},
            blocks={"body_state": np.zeros((1, 1, 19))},
            model_document=compiled.model_document,
            case_document=compiled.case_document,
            times_s=np.array([0.0]),
        )


def test_request_normalizes_dimensions_context_and_kind() -> None:
    request = SimulationRequest(
        assembly=" Vehicle ",
        family=" HANDLING ",
        request_kind=" HandlingScenario ",
        context={"case_document": {"family": "handling"}},
    )

    assert request.assembly == "vehicle"
    assert request.family == "handling"
    assert request.kind == "handlingscenario"
    assert request.context == {"case_document": {"family": "handling"}}

    with pytest.raises(ValueError, match="assembly"):
        SimulationRequest(assembly="", family="handling")


def test_compile_document_pair_preserves_layout_metadata_and_payloads() -> None:
    request = SimulationRequest(assembly="axle", family="kc_quasi_static")
    compiled = compile_document_pair(
        request,
        model_document={"kind": "model"},
        case_document={"kind": "case"},
        compiler_name="KcQuasiStaticCompiler",
        payload_schema="kc_quasi_static_contract_documents",
        layout={"sample_order": "contract"},
        metadata={"source": "test"},
    )

    assert compiled.layout["document_order"] == ["model", "case"]
    assert compiled.layout["sample_order"] == "contract"
    assert compiled.metadata == {
        "compiler": "KcQuasiStaticCompiler",
        "assembly": "axle",
        "family": "kc_quasi_static",
        "request_kind": "kc_quasi_static",
        "payload_schema": "kc_quasi_static_contract_documents",
        "source": "test",
    }
    assert all(isinstance(payload, bytes) for payload in compiled.payloads())


def test_compiler_rejects_missing_documents() -> None:
    compiler = DocumentPairCompiler("axle", "probe")
    with pytest.raises(TypeError, match="model_document"):
        compiler.compile(SimulationRequest("axle", "probe"))


def test_compile_document_pair_and_run_compiled_use_backend_contract() -> None:
    request = SimulationRequest(assembly="axle", family="kc_quasi_static")
    compiled = compile_document_pair(
        request,
        model_document={"kind": "model"},
        case_document={"kind": "case"},
    )
    backend = FakeBackend()

    result = run_compiled(compiled, backend=backend)

    assert result.status == "success"
    assert result.request is request
    assert backend.compiled is compiled
    assert compiled.documents() == ({"kind": "model"}, {"kind": "case"})
    assert all(isinstance(payload, bytes) for payload in compiled.payloads())


def test_run_request_uses_explicit_registry_and_backend() -> None:
    registry = CompilerRegistry()
    registry.register(DocumentPairCompiler("axle", "probe"))
    request = SimulationRequest(
        assembly="AXLE",
        family="PROBE",
        context={
            "model_document": {"kind": "model"},
            "case_document": {"kind": "case"},
        },
    )
    backend = FakeBackend()

    result = run_request(request, registry=registry, backend=backend)

    assert result.request == request
    assert result.raw.model_document == {"kind": "model"}
    assert result.raw.case_document == {"kind": "case"}
    with pytest.raises(KeyError, match="no compiler registered"):
        run_request(
            SimulationRequest("axle", "missing", context=request.context),
            registry=registry,
            backend=backend,
        )


def test_default_registry_covers_current_contract_families() -> None:
    expected = {
        ("axle", "axle_dynamic"): "AxleDynamicCompiler",
        ("axle", "kc_quasi_static"): "KcQuasiStaticCompiler",
        ("vehicle", "vehicle_dynamic"): "VehicleDynamicCompiler",
        ("vehicle", "vehicle_kc"): "VehicleKcCompiler",
        ("vehicle", "handling"): "HandlingCompiler",
        ("vehicle", "ride_four_post"): "RideFourPostCompiler",
        ("vehicle", "ride_random_road"): "RideRandomRoadCompiler",
    }

    registry = default_registry()
    assert set(registry.keys()) == set(expected)
    assert {type(registry.resolve(*key)).__name__ for key in expected} == set(
        expected.values()
    )
    assert isinstance(registry.resolve("vehicle", "handling"), HandlingCompiler)
    assert isinstance(registry.resolve("axle", "kc_quasi_static"), KcQuasiStaticCompiler)
    assert isinstance(registry.resolve("vehicle", "vehicle_kc"), VehicleKcCompiler)
    assert isinstance(registry.resolve("vehicle", "ride_four_post"), RideFourPostCompiler)
    assert isinstance(registry.resolve("vehicle", "ride_random_road"), RideRandomRoadCompiler)
def test_run_compiled_attaches_partial_raw_result_before_reraising() -> None:
    class PartialBackend:
        def run(self, compiled):
            raise KernelContractError(
                "solver stopped after partial progress",
                partial_run=ContractRun(
                    document={
                        "status": "partial",
                        "manifest": {"failed_sample_index": 1},
                    },
                    blocks={"body_state": np.zeros((1, 1, 19))},
                    model_document=compiled.model_document,
                    case_document=compiled.case_document,
                    times_s=np.array([0.0]),
                ),
            )

    compiled = compile_document_pair(
        SimulationRequest(assembly="axle", family="kc_quasi_static"),
        model_document={"kind": "model"},
        case_document={"kind": "case"},
    )

    with pytest.raises(KernelContractError) as raised:
        run_compiled(compiled, backend=PartialBackend())

    assert isinstance(raised.value.partial_raw_result, RawContractResult)
    assert raised.value.partial_raw_result.status == "partial"
    assert raised.value.partial_raw_result.failure_evidence["failed_sample_index"] == 1
