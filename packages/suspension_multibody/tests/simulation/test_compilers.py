from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest
from suspension_contracts import unpack_container

from suspension_multibody.kernel import ContractRun, KernelContractError
from suspension_multibody.simulation import (
    AxleDynamicCompiler,
    HandlingCompiler,
    KcQuasiStaticCompiler,
    RideFourPostCompiler,
    RideRandomRoadCompiler,
    SimulationRequest,
    VehicleDynamicCompiler,
    VehicleKcCompiler,
    compile_document_pair,
    run_compiled,
)


def _documents(family: str) -> tuple[dict[str, object], dict[str, object]]:
    return (
        {
            "contract": "multibody-model",
            "contract_version": 1,
            "kind": "model",
            "name": f"{family}-model",
        },
        {
            "contract": "multibody-case",
            "contract_version": 1,
            "kind": "case",
            "family": family,
            "name": f"{family}-case",
        },
    )


def _request(
    assembly: str,
    family: str,
    *,
    model: object | None = None,
    case: object | None = None,
    context: dict[str, object] | None = None,
) -> SimulationRequest:
    return SimulationRequest(
        assembly=assembly,
        family=family,
        model=model,
        case=case,
        context=context or {},
    )


@pytest.mark.parametrize(
    ("compiler", "assembly", "family"),
    [
        (KcQuasiStaticCompiler(), "axle", "kc_quasi_static"),
        (HandlingCompiler(), "vehicle", "handling"),
        (RideFourPostCompiler(), "vehicle", "ride_four_post"),
        (RideRandomRoadCompiler(), "vehicle", "ride_random_road"),
    ],
)
def test_document_pair_compilers_materialize_contracts_and_metadata(
    compiler, assembly: str, family: str
) -> None:
    model, case = _documents(family)
    compiled = compiler.compile(
        _request(
            assembly,
            family,
            context={"model_document": model, "case_document": case},
        )
    )

    assert compiled.documents() == (model, case)
    assert compiled.layout == {
        "document_order": ["model", "case"],
        "payload_order": ["model", "case"],
    }
    assert compiled.metadata["compiler"] == type(compiler).__name__
    assert compiled.metadata["assembly"] == assembly
    assert compiled.metadata["family"] == family
    assert compiled.metadata["request_kind"] == family
    assert all(isinstance(payload, bytes) for payload in compiled.payloads())


def test_document_pair_compiler_preserves_embedded_payloads() -> None:
    model, case = _documents("handling")
    model_payload = b"model-payload"
    case_payload = b"case-payload"
    compiled = HandlingCompiler().compile(
        _request(
            "vehicle",
            "handling",
            context={
                "model_document": (model, model_payload),
                "case_document": (case, case_payload),
            },
        )
    )

    assert compiled.model_payload == model_payload
    assert compiled.case_payload == case_payload


def test_document_pair_compiler_checks_contract_version_and_request_identity() -> None:
    model, case = _documents("kc_quasi_static")
    compiler = KcQuasiStaticCompiler()
    context = {"model_document": model, "case_document": case}

    with pytest.raises(ValueError, match="assembly"):
        compiler.compile(_request("vehicle", "kc_quasi_static", context=context))
    with pytest.raises(ValueError, match="request kind"):
        compiler.compile(
            SimulationRequest(
                "axle",
                "kc_quasi_static",
                request_kind="wrong_kind",
                context=context,
            )
        )

    bad_case = dict(case)
    bad_case["contract_version"] = 2
    with pytest.raises(ValueError, match="case contract_version"):
        compiler.compile(
            _request(
                "axle",
                "kc_quasi_static",
                context={"model_document": model, "case_document": bad_case},
            )
        )


def test_axle_dynamic_compiler_owns_document_and_blob_authoring(monkeypatch) -> None:
    from suspension_multibody.cases import axle_dynamic

    model_document = _documents("axle_dynamic")[0]
    case_document = _documents("axle_dynamic")[1]
    monkeypatch.setattr(
        axle_dynamic,
        "model_document",
        lambda model, name=None: (model_document, b"model-blob"),
    )
    monkeypatch.setattr(
        axle_dynamic,
        "case_document",
        lambda model, case, name=None: (case_document, b"case-blob"),
    )

    compiled = AxleDynamicCompiler().compile(
        _request("axle", "axle_dynamic", model=object(), case=object())
    )
    model, model_blob = unpack_container(compiled.model_payload)
    case, case_blob = unpack_container(compiled.case_payload)

    assert model == model_document
    assert case == case_document
    assert model_blob == b"model-blob"
    assert case_blob == b"case-blob"


def test_vehicle_dynamic_compiler_uses_prepared_context(monkeypatch) -> None:
    from suspension_multibody.cases import vehicle_dynamic

    model_document = _documents("vehicle_dynamic")[0]
    case_document = _documents("vehicle_dynamic")[1]
    prepared = SimpleNamespace(name="prepared")
    seen: dict[str, object] = {}

    def emit_model(model, actual_prepared, *, name=None):
        seen["model_prepared"] = actual_prepared
        return model_document, b"model-blob"

    def emit_case(model, case, actual_prepared, *, name=None):
        seen["case_prepared"] = actual_prepared
        return case_document, b"case-blob"

    monkeypatch.setattr(vehicle_dynamic, "model_document", emit_model)
    monkeypatch.setattr(vehicle_dynamic, "case_document", emit_case)

    compiled = VehicleDynamicCompiler().compile(
        _request(
            "vehicle",
            "vehicle_dynamic",
            model=object(),
            case=object(),
            context={"prepared": prepared},
        )
    )

    assert seen == {"model_prepared": prepared, "case_prepared": prepared}
    assert compiled.metadata["prepared"] is True
    assert compiled.model_payload
    assert compiled.case_payload


def test_vehicle_kc_compiler_derives_driven_model_metadata(monkeypatch) -> None:
    from suspension_multibody.cases import vehicle_kc

    source_model, source_payload = _documents("vehicle_kc")[0], b"source"
    case = _documents("vehicle_kc")[1]
    derived_model = dict(source_model)
    derived_model["joints"] = [{"type": "driven_translation"}, {"type": "fixed"}]
    monkeypatch.setattr(
        vehicle_kc,
        "model_document",
        lambda pair, *, wheels, assembly: (derived_model, b"derived-blob"),
    )

    compiled = VehicleKcCompiler().compile(
        _request(
            "vehicle",
            "vehicle_kc",
            model=(source_model, source_payload),
            case=case,
            context={
                "model_document_pair": (source_model, source_payload),
                "wheels": (),
                "vehicle_assembly": object(),
            },
        )
    )

    assert compiled.model_document == derived_model
    assert compiled.metadata["derived_model"] is True
    assert compiled.metadata["driven_joint_count"] == 1


def test_run_compiled_preserves_partial_status_diagnostics_and_performance() -> None:
    model, case = _documents("handling")
    compiled = compile_document_pair(
        _request("vehicle", "handling"),
        model_document=model,
        case_document=case,
    )
    diagnostics = np.zeros((3, 16), dtype=float)
    diagnostics[1, 0] = 1.0

    class PartialBackend:
        def run(self, submitted):
            return ContractRun(
                document={
                    "status": "partial",
                    "manifest": {
                        "bodies": ["body"],
                        "cases": [{"sample_offset": 0, "sample_count": 1}],
                    },
                },
                blocks={
                    "body_state": np.zeros((1, 1, 19)),
                    "diagnostics": diagnostics,
                },
                model_document=submitted.model_document,
                case_document=submitted.case_document,
                times_s=np.array([0.0]),
            )

    result = run_compiled(compiled, backend=PartialBackend())

    assert result.status == "partial"
    assert result.raw.diagnostics is not None
    assert result.raw.diagnostics.shape == (1, 16)
    assert result.raw.performance["available"] is True
def test_run_compiled_preserves_failed_raw_result() -> None:
    model, case = _documents("handling")
    compiled = compile_document_pair(
        _request("vehicle", "handling"),
        model_document=model,
        case_document=case,
    )

    class FailedBackend:
        def run(self, submitted):
            return ContractRun(
                document={"status": "failed", "manifest": {"failure_message": "trim failed"}},
                blocks={},
                model_document=submitted.model_document,
                case_document=submitted.case_document,
                times_s=np.array([]),
            )

    result = run_compiled(compiled, backend=FailedBackend())

    assert result.status == "failed"
    assert result.raw.document["manifest"]["failure_message"] == "trim failed"
def test_native_backend_attaches_partial_raw_result(monkeypatch) -> None:
    from suspension_multibody.simulation import backend as backend_module

    model, case = _documents("handling")
    compiled = compile_document_pair(
        _request("vehicle", "handling"),
        model_document=model,
        case_document=case,
    )
    diagnostics = np.zeros((3, 16), dtype=float)
    diagnostics[1, 0] = 1.0
    partial = ContractRun(
        document={
            "status": "partial",
            "manifest": {
                "bodies": ["body"],
                "cases": [{"sample_offset": 0, "sample_count": 1}],
            },
        },
        blocks={"body_state": np.zeros((1, 1, 19)), "diagnostics": diagnostics},
        model_document=model,
        case_document=case,
        times_s=np.array([0.0]),
    )

    def raise_partial(*args, **kwargs):
        raise KernelContractError("trim failed", partial_run=partial)

    monkeypatch.setattr(backend_module, "run_contract", raise_partial)
    with pytest.raises(KernelContractError) as caught:
        run_compiled(compiled)

    error = caught.value
    assert error.partial_raw_result is not None
    assert error.partial_raw_result.status == "partial"
    assert error.partial_raw_result.performance["available"] is True
    assert error.partial_raw_result.diagnostics.shape == (1, 16)
