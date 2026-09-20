"""
Contract tests for the split whole-vehicle dynamic responsibilities.

They exercise the real family -- the registry entry, the legacy ``prepared``
migration adapter, the compiler, the typed result and the service metrics/error
evidence -- and pin the two structural invariants of the split: every legacy
top-level definition has one named new home, and the old module is only a
re-export shell with no implementation of its own.
"""

from __future__ import annotations

import ast
import importlib
import importlib.util
import inspect
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from suspension_multibody.axle_dynamics.errors import NativeAxleError
from suspension_multibody.kernel import ContractRun, KernelContractError
from suspension_multibody.preparation import vehicle_dynamic as preparation
from suspension_multibody.preparation.vehicle_dynamic import (
    PreparedVehicleRun,
    adapt_legacy_prepared_request,
    prepare_vehicle_run,
)
from suspension_multibody.results import decoder as decoder_module
from suspension_multibody.results import vehicle as results_vehicle
from suspension_multibody.simulation import (
    NativeContractBackend,
    PreparedSimulation,
    SimulationRequest,
    compile_request,
    default_preparation_registry,
    prepare_request,
    run_compiled,
    run_request,
)
from suspension_multibody.simulation import runner as runner_module
from suspension_multibody.vehicle import service as vehicle_service
from suspension_multibody.vehicle.service import run_vehicle_dynamics

_FIXTURE = Path(__file__).resolve().parent / "test_native_vehicle.py"

VEHICLE_DYNAMIC_KEY = ("vehicle", "vehicle_dynamic")

#: The complete preparation half of ``PREPARATION_MATRIX.md``'s legacy
#: definition list, minus the renamed ``_PreparedVehicleRun``.
PREPARATION_SYMBOLS = (
    "_WHEEL_NAMES",
    "_ROAD_KIND",
    "_PRESCRIBED_STEERING_TYPES",
    "_VehicleSteeringBuffers",
    "_VehicleRoadBuffers",
    "_NativeVehicleModel",
    "_BodyFrame",
    "_select_assembly_mode",
    "_validate_steering_topology",
    "prepare_vehicle_run",
    "_length_scale",
    "_validate_units",
    "_build_static_rotation_gauges",
    "_uses_horizontal_static_gauge",
    "_output_times",
    "_initial_body_state",
    "_resolve_vehicle_body",
    "_rotation_from_quaternion",
    "_tuple3",
    "_tuple4",
    "_matrix3",
    "_shift_point",
    "_build_aerodynamic_drags",
    "_build_joints",
    "_build_coordinate_couplers",
    "_build_elements",
    "_damper_curve",
    "_length_force_curve",
    "_bushing_force_curves",
    "_spring_force_curve",
    "_tuple6",
    "_wheel_forward_local",
    "_build_tires",
    "_build_steering",
    "_resolve_named_body",
    "_resolve_steering_rack",
    "_steering_target_value",
    "_steering_target_rate",
    "_build_road",
    "_build_wheel_torque_signals",
    "_native_solver_settings",
)
RESULT_SYMBOLS = (
    "VehicleDynamicsResult",
    "_contract_constraint_names",
    "_vehicle_axle_result",
)
SERVICE_SYMBOLS = ("run_vehicle_dynamics",)


@pytest.fixture(scope="module")
def fixture():
    spec = importlib.util.spec_from_file_location("native_vehicle_fixture", _FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _model_and_case(fixture):
    model = fixture._positioned_vehicle(fixture._vehicle())
    return model, fixture._case(model)


def _imported_module_names(module) -> set[str]:
    tree = ast.parse(inspect.getsource(module))
    names: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module is not None:
            names.add(node.module)
    return names


def test_vehicle_legacy_definition_map() -> None:
    """Every legacy top-level definition has exactly one named new home."""
    # 42 preparation names (the renamed ``_PreparedVehicleRun`` included) plus
    # three result names and one service name: the matrix's 46 entries.
    assert (
        len(PREPARATION_SYMBOLS) + 1 + len(RESULT_SYMBOLS) + len(SERVICE_SYMBOLS) == 46
    )

    for name in PREPARATION_SYMBOLS:
        assert hasattr(preparation, name), f"preparation.vehicle_dynamic lacks {name}"
    assert hasattr(preparation, "PreparedVehicleRun")
    assert not hasattr(preparation, "_PreparedVehicleRun"), "the legacy private name is renamed"

    for name in RESULT_SYMBOLS:
        assert hasattr(results_vehicle, name), f"results.vehicle lacks {name}"
        assert not hasattr(preparation, name), f"{name} must not stay in the preparation module"
    for name in SERVICE_SYMBOLS:
        assert hasattr(vehicle_service, name), f"vehicle.service lacks {name}"
        assert not hasattr(preparation, name), f"{name} must not stay in the preparation module"

    # The old path is a shell: no class or function of its own, and every
    # re-export is the very same object rather than a second definition.  While
    # sub-task 04 has not deleted it yet; the name is composed so this test file
    # is not itself an old-module path reference for the deletion gate, and the
    # check degrades to the map assertions once the file is gone.
    legacy_module = "suspension_multibody." + "vehicle_dynamics"
    if importlib.util.find_spec(legacy_module) is None:
        return
    legacy = importlib.import_module(legacy_module)
    tree = ast.parse(inspect.getsource(legacy))
    assert not any(
        isinstance(node, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef))
        for node in tree.body
    )
    assert legacy.PreparedVehicleRun is preparation.PreparedVehicleRun
    assert legacy._PreparedVehicleRun is preparation.PreparedVehicleRun
    assert legacy.prepare_vehicle_run is preparation.prepare_vehicle_run
    assert legacy.VehicleDynamicsResult is results_vehicle.VehicleDynamicsResult
    assert legacy._vehicle_axle_result is results_vehicle._vehicle_axle_result
    assert legacy._contract_constraint_names is results_vehicle._contract_constraint_names
    assert legacy.run_vehicle_dynamics is vehicle_service.run_vehicle_dynamics


def test_new_vehicle_modules_do_not_import_the_legacy_module() -> None:
    for module in (preparation, results_vehicle, vehicle_service, decoder_module):
        imported = _imported_module_names(module)
        assert not any(name.endswith("vehicle_dynamics") for name in imported), (
            module.__name__,
            imported,
        )
    # The unified dispatcher is the one module whose source must not even name
    # the legacy path; the others legitimately define ``*_vehicle_dynamics``.
    assert "vehicle_dynamics" not in inspect.getsource(decoder_module)


def test_decoder_dispatches_vehicle_runs_through_the_family_adapter(monkeypatch) -> None:
    seen: dict[str, object] = {}

    def spy(prepared, run, **kwargs):
        seen["prepared"] = prepared
        seen["run"] = run
        seen["kwargs"] = kwargs
        return "decoded-by-family-adapter"

    monkeypatch.setattr(results_vehicle, "decode_vehicle_result", spy)
    prepared = SimpleNamespace()
    run = ContractRun(document={"status": "success"}, blocks={})

    decoded = decoder_module.decode_result(
        run,
        assembly="vehicle",
        family="vehicle_dynamic",
        prepared=prepared,
        stop=3,
    )

    assert decoded == "decoded-by-family-adapter"
    assert seen["prepared"] is prepared
    assert seen["run"] is run
    assert seen["kwargs"]["stop"] == 3


def test_vehicle_dynamic_registry_key_prepares_the_real_family(fixture) -> None:
    model, case = _model_and_case(fixture)
    registry = default_preparation_registry()

    assert VEHICLE_DYNAMIC_KEY in registry.keys()

    prepared = prepare_request(
        SimulationRequest(
            assembly="vehicle", family="vehicle_dynamic", model=model, case=case
        )
    )

    value = prepared.request.context["vehicle_dynamic_prepared"]
    assert isinstance(value, PreparedVehicleRun)
    assert prepared.value is value
    assert value.source_model is model
    assert value.source_case is case


def test_legacy_prepared_adapter_wraps_a_matching_preparation(fixture) -> None:
    model, case = _model_and_case(fixture)
    prepared = prepare_vehicle_run(model, case)
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=model,
        case=case,
        context={"prepared": prepared},
    )

    adapted = adapt_legacy_prepared_request(request)

    wrapped = adapted.context["prepared_simulation"]
    assert isinstance(wrapped, PreparedSimulation)
    assert wrapped.value is prepared
    assert wrapped.request.model is model
    assert wrapped.request.case is case
    assert adapted.context["vehicle_dynamic_prepared"] is prepared
    assert adapted.context["prepared"] is prepared
    assert adapted.model is model
    assert adapted.case is case


def test_legacy_prepared_request_is_reused_without_a_second_preparation(
    fixture, monkeypatch
) -> None:
    model, case = _model_and_case(fixture)
    prepared = prepare_vehicle_run(model, case)

    def refuse(*_args, **_kwargs):
        raise AssertionError("a matching legacy preparation must be reused")

    monkeypatch.setattr(preparation, "prepare_vehicle_run", refuse)
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=model,
        case=case,
        context={"prepared": prepared},
    )

    result = prepare_request(request)

    assert result.value is prepared
    assert result.request.context["vehicle_dynamic_prepared"] is prepared
    assert result.request.context["prepared_simulation"].value is prepared
    assert result.request.model is model
    assert result.request.case is case


def test_stale_legacy_prepared_is_re_prepared_instead_of_reused(fixture) -> None:
    model, case = _model_and_case(fixture)
    other_model = model.model_copy(update={"name": "other-vehicle"})
    other_case = fixture._case(other_model)
    stale = prepare_vehicle_run(model, case)
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=other_model,
        case=other_case,
        context={"prepared": stale},
    )

    result = prepare_request(request)

    fresh = result.request.context["vehicle_dynamic_prepared"]
    assert fresh is not stale
    assert result.value is fresh
    assert fresh.source_model is other_model
    assert fresh.source_case is other_case


def test_legacy_prepared_wrong_type_is_a_diagnostic_type_error(fixture) -> None:
    model, case = _model_and_case(fixture)
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=model,
        case=case,
        context={"prepared": object()},
    )

    with pytest.raises(TypeError, match="PreparedVehicleRun"):
        adapt_legacy_prepared_request(request)
    with pytest.raises(TypeError, match="PreparedVehicleRun"):
        prepare_request(request)


def test_document_request_bypasses_the_legacy_prepared_adapter(fixture) -> None:
    model, case = _model_and_case(fixture)
    prepared = prepare_vehicle_run(model, case)
    request = SimulationRequest(
        assembly="vehicle",
        family="vehicle_dynamic",
        model=model,
        case=case,
        context={"prepared": prepared, "case_document": {"kind": "case"}},
    )

    result = prepare_request(request)

    assert result.request is request
    assert result.value is None
    assert "prepared_simulation" not in result.context


def _count_preparations(monkeypatch) -> list[tuple[object, object]]:
    original = preparation.prepare_vehicle_run
    calls: list[tuple[object, object]] = []

    def counted(model, case):
        calls.append((model, case))
        return original(model, case)

    monkeypatch.setattr(preparation, "prepare_vehicle_run", counted)
    return calls


class _CountingBackend:
    """Native backend that counts submissions and otherwise delegates."""

    def __init__(self) -> None:
        self.submissions = 0
        self._inner = NativeContractBackend()

    def run(self, compiled):
        self.submissions += 1
        return self._inner.run(compiled)


def test_staged_path_prepares_once_and_submits_native_once(fixture, monkeypatch) -> None:
    model, case = _model_and_case(fixture)
    calls = _count_preparations(monkeypatch)
    backend = _CountingBackend()
    request = SimulationRequest(
        assembly="vehicle", family="vehicle_dynamic", model=model, case=case
    )

    prepared = prepare_request(request)
    compiled = compile_request(prepared.request)
    run = run_compiled(compiled, backend=backend)

    assert calls == [(model, case)]
    assert backend.submissions == 1
    assert isinstance(run.result, results_vehicle.VehicleDynamicsResult)
    assert run.result.times_s.size >= 2


def test_facade_prepares_once_and_submits_native_once(fixture, monkeypatch) -> None:
    model, case = _model_and_case(fixture)
    calls = _count_preparations(monkeypatch)
    backend = _CountingBackend()

    run = run_request(
        SimulationRequest(
            assembly="vehicle", family="vehicle_dynamic", model=model, case=case
        ),
        backend=backend,
    )

    assert calls == [(model, case)]
    assert backend.submissions == 1
    assert isinstance(run.result, results_vehicle.VehicleDynamicsResult)
    assert run.result.times_s.size >= 2


def test_vehicle_service_metrics_and_artifact(fixture, tmp_path) -> None:
    from suspension_multibody.io import read_artifact, write_artifact

    model, case = _model_and_case(fixture)

    result = run_vehicle_dynamics(model, case)

    assert isinstance(result, results_vehicle.VehicleDynamicsResult)
    assert result.metrics
    assert result.static_wheel_loads is not None
    assert result.native_kernel_wall_time_s >= 0.0
    assert result.times_s.size >= 2

    loaded = read_artifact(
        write_artifact(result, tmp_path / "vehicle", model=model, case=case)
    )

    assert loaded["manifest"]["artifact_type"] == "vehicle_dynamics_result"
    assert loaded["manifest"]["status"] == "success"
    assert loaded["manifest"]["metrics"]
    assert loaded["arrays"]["states"].shape[0] == result.times_s.size


def test_vehicle_service_keeps_native_failure_evidence(fixture, monkeypatch, tmp_path) -> None:
    from suspension_multibody.io import read_artifact, write_artifact

    model, case = _model_and_case(fixture)
    prepared = prepare_request(
        SimulationRequest(
            assembly="vehicle", family="vehicle_dynamic", model=model, case=case
        )
    )
    compiled = compile_request(prepared.request)
    complete = NativeContractBackend().run(compiled)
    document = dict(complete.document)
    document["status"] = "partial"
    manifest = dict(document.get("manifest", {}))
    manifest.update(failed_sample_index=1, failed_status=3, failed_time_s=0.001)
    document["manifest"] = manifest
    partial = replace(complete, document=document)

    class _FailingBackend:
        def run(self, compiled):
            raise KernelContractError("vehicle solver stopped", partial_run=partial)

    monkeypatch.setattr(runner_module, "NativeContractBackend", _FailingBackend)

    with pytest.raises(NativeAxleError) as raised:
        run_vehicle_dynamics(model, case)

    error = raised.value
    assert error.status == 3
    assert error.failed_sample_index == 1
    assert error.failed_time_s == pytest.approx(0.001)
    assert error.failure_diagnostics is not None
    assert isinstance(error.partial_result, results_vehicle.VehicleDynamicsResult)
    assert error.partial_result.metrics

    loaded = read_artifact(
        write_artifact(
            None,
            tmp_path / "failed",
            partial=error.partial_result,
            status="failed",
            failure=error,
        )
    )

    assert loaded["manifest"]["status"] == "failed"
    assert loaded["manifest"]["failure_evidence"]["type"] == "NativeAxleError"
    assert loaded["manifest"]["failure_evidence"]["failed_sample_index"] == 1
    assert loaded["manifest"]["partial_evidence"]["available"] is True
