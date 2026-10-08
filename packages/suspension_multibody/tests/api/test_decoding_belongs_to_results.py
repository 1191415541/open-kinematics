"""Public execution delegates decoding to the uniform result envelope."""

import ast
import inspect

import numpy as np
import pytest

from suspension_multibody import api
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from suspension_multibody.results import ResultEnvelope
from tests.authoring.test_generic_multibody import _assembly, _case
from tests.authoring.test_generic_tire import _stationary_case, _tire_subsystem
from tests.benchmark_fixture import benchmark_model


def test_api_declares_no_native_column_index_of_its_own():
    tree = ast.parse(inspect.getsource(api))
    assert not any(isinstance(node, ast.Subscript) for function in tree.body if isinstance(function, ast.FunctionDef)
                   for statement in function.body for node in ast.walk(statement))
    names = {node.id for node in ast.walk(tree) if isinstance(node, ast.Name)}
    assert not names.intersection({"_DIAGNOSTIC_POSITION_RESIDUAL", "_DIAGNOSTIC_DYNAMICS_RESIDUAL", "_DIAGNOSTIC_ROWS_PER_CASE"})


def test_api_leaves_native_decoding_to_the_runner():
    assert "run_compiled" in inspect.getsource(api.simulate)
    assert ResultEnvelope.__module__ == "suspension_multibody.results.envelope"
    assert not hasattr(api, "_rigid_state")
    assert not hasattr(api, "_tire_compression")
    assert not hasattr(api, "_collect_element_results")


def test_api_does_not_import_the_element_force_law_for_decoding():
    tree = ast.parse(inspect.getsource(api))
    imports = [node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)]
    assert not any("primitives" in name or "kc_state" in name for name in imports if name)


def test_the_reported_tire_compression_is_the_kernel_s_own_number():
    run = api.simulate(_assembly({"unit": _tire_subsystem()}), _stationary_case())
    assert run.status == "success"
    assert run.result.tire_ids == run.raw.tire_names
    assert len(run.result.tire_ids) == 1
    for index, name in enumerate(run.result.tire_ids):
        np.testing.assert_array_equal(run.result.tire_state(name)[:, 2], run.raw.block("tire_output")[:, index, 2])
    assert run.result.tire_state(run.result.tire_ids[0])[0, 2] > 0


def test_a_run_with_no_tire_reports_no_compression_key():
    result = api.simulate(_assembly(), _case()).result
    assert result.tire_ids == ()
    with pytest.raises(KeyError, match="unknown tire"):
        result.tire_state("missing")


@pytest.mark.parametrize("mode", ["K", "C"])
def test_both_modes_decode_their_state_from_the_contract(mode):
    model = benchmark_model()
    if mode == "C":
        from tests.cases.kc_quasi_static.kc_fixtures import _compliant_model
        model = _compliant_model()
    arguments = {"wheel_values_mm": (-10., 0., 10.)} if mode == "K" else {"paths": ("fz",), "levels": 3, "maximum": 1}
    run = api.simulate(*migrate_v1_kc_case(model, mode=mode, **arguments))
    assert isinstance(run.result, ResultEnvelope)
    assert run.status == "success"
    assert len(run.result.cases) == 3
    for index in range(3):
        np.testing.assert_array_equal(run.result.case_body_state(index), run.raw.case_body_state(index))
        assert max(run.result.case_residuals(index)) < 1e-6
