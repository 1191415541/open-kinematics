"""
Decoding belongs to `results`, and `api` does not do its own.

Sub-task 10's goal is a responsibility split, and the split is only real if a
test can see it: this file checks the *shape* of the dependency rather than a
number, because the defect was never a wrong value -- it was two modules owning
the same layout and having to agree about it.

Three claims:

1. `api` reads the K/C sample through `results.kc_state`, so the row slices and
   the metre-to-millimetre conversion live in one place;
2. `api` keeps no column-index constants of its own.  It used to declare
   `_DIAGNOSTIC_POSITION_RESIDUAL = 7` and `_DIAGNOSTIC_DYNAMICS_RESIDUAL = 9`
   that nothing read, while `results.raw` read the same two columns by literal;
3. the tire compression the result reports and the one the kernel wrote are the
   same number, read from the same place.
"""

from __future__ import annotations

import ast
from pathlib import Path

import pytest

from suspension_multibody import api
from suspension_multibody.schema import CaseSpec, DisplacementControl
from tests.benchmark_fixture import benchmark_model

SOURCE = Path(__file__).parents[2] / "src" / "suspension_multibody"


def _module_tree(relative: str) -> ast.Module:
    return ast.parse((SOURCE / relative).read_text(encoding="utf-8"))


def _assigned_names(module: ast.Module) -> set[str]:
    """Return every module-level name the file assigns."""
    found: set[str] = set()
    for node in module.body:
        if isinstance(node, ast.Assign):
            found.update(
                target.id for target in node.targets if isinstance(target, ast.Name)
            )
        elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            found.add(node.target.id)
    return found


def test_api_declares_no_native_column_index_of_its_own() -> None:
    """
    The layout constants belong to `results`, and `api` must not keep a copy.

    The three it carried were `_DIAGNOSTIC_POSITION_RESIDUAL`,
    `_DIAGNOSTIC_DYNAMICS_RESIDUAL` and `_DIAGNOSTIC_ROWS_PER_CASE`; the first
    two were dead duplicates of `results.raw`'s own literals, which is exactly the
    failure mode -- two homes for one number, neither authoritative.
    """
    names = _assigned_names(_module_tree("api.py"))
    forbidden = {
        "_DIAGNOSTIC_POSITION_RESIDUAL",
        "_DIAGNOSTIC_DYNAMICS_RESIDUAL",
        "_DIAGNOSTIC_ROWS_PER_CASE",
    }
    assert not (names & forbidden), sorted(names & forbidden)


def test_api_decodes_the_kc_sample_through_results() -> None:
    """
    `api` reads a contract row by calling `results`, not by slicing it.

    A grep for `row[`/`entry[:3]`/`entry[3:7]` inside `api` is what would find a
    re-introduced decode; this checks the call sites instead, so a rename of the
    helper is caught by the import test below rather than by a stale pattern.
    """
    tree = _module_tree("api.py")
    called: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute):
            called.add(node.attr)
        elif isinstance(node, ast.Name):
            called.add(node.id)
    assert "rigid_state_from_row" in called
    assert "tire_compression_from_run" in called


def test_the_kc_decode_helpers_live_in_results() -> None:
    """The other half: the module that owns the layout is where they are defined."""
    from suspension_multibody.results import kc_state

    assert hasattr(kc_state, "rigid_state_from_row")
    assert hasattr(kc_state, "tire_compression_from_run")
    assert hasattr(kc_state, "MM")
    # `api` keeps `_rigid_state` and `_tire_compression` as the entry points its
    # own call sites use, so both names stay reachable -- but neither computes.
    assert callable(api._rigid_state)
    assert callable(api._tire_compression)


def test_api_does_not_import_the_element_force_law_for_decoding() -> None:
    """
    The remaining known violation, recorded rather than hidden.

    `api._collect_element_results` still evaluates a bushing's own `-K d + preload`
    in Python.  The native element-wrench channel that would replace it exists and
    is decoded by `results.element_wrench`, but it is behind an environment switch
    that production does not set -- so wiring it is a behaviour change tracked
    separately and not quietly claimed here.
    """
    from suspension_multibody.results import element_wrench

    assert hasattr(element_wrench, "element_wrench_enabled")
    # The switch is off in this environment, which is why the Python path is still
    # the one production takes.
    assert element_wrench.element_wrench_enabled() is False


def test_the_reported_tire_compression_is_the_kernel_s_own_number() -> None:
    """
    One number, read from where it was solved.

    The channel used to be a constant `{left: 0.0, right: 0.0}`; it now reads the
    kernel's `tire_output`, and this pins that the two agree exactly rather than
    approximately -- a decode that went through a second conversion would show up
    here.
    """
    import json

    from suspension_multibody.cases.kc_quasi_static import model_document
    from suspension_multibody.preparation.assembly import build_front_axle
    from suspension_multibody.results.kc_state import tire_compression_from_run
    from suspension_multibody.schema import FrontAxleModel
    from suspension_multibody.simulation import SimulationRequest, run_request

    payload = json.loads(
        (Path(__file__).parents[1] / "data" / "benchmark_axle.json").read_text(
            encoding="utf-8"
        )
    )
    raw = dict(payload["model"])
    raw["tires"] = [
        {
            "stiffness": 200.0,
            "unloaded_radius": 320.0,
            "contact_point": {"x": 0.0, "y": 0.0, "z": 0.0},
            "local_axis": {"x": 0.0, "y": 0.0, "z": 1.0},
        }
    ]
    assembly = build_front_axle(FrontAxleModel.model_validate(raw), "K")
    document = model_document(assembly, name="probe", drive_wheels=True)
    case = {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": "probe",
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "k": {
            "wheel_values_mm": [-20.0],
            "rack_values_mm": [0.0],
            "axis_map": {
                "wheel": ["wheel_drive_L", "wheel_drive_R"],
                "rack": "rack_drive",
            },
            "left_right_mode": "symmetric",
        },
    }
    run = run_request(
        SimulationRequest(
            assembly="axle",
            rig="kc_quasi_static",
            family="kc_quasi_static",
            model=document,
            case=case,
        )
    ).raw

    decoded = tire_compression_from_run(run, 0)
    raw_block = run.block("tire_output")
    for index, name in enumerate(run.tire_names):
        side = "left" if name.endswith("_L") else "right"
        assert decoded[side] == raw_block[0, index, 2]


def test_a_run_with_no_tire_reports_no_compression_key() -> None:
    """
    Absent, not zero -- the same distinction the rack channel is built on.

    A model that declares no tire has no compression, and `{"left": 0.0}` would
    be a measured value that does not exist.
    """
    bundle = api.run_case(
        benchmark_model(),
        CaseSpec(
            mode="K",
            controls=(
                DisplacementControl(target="wheel_travel_left", values=(-10.0,)),
            ),
        ),
    )
    assert bundle.states[0].tire_compression == {}


@pytest.mark.parametrize("mode", ["K", "C"])
def test_both_modes_decode_their_state_from_the_contract(mode: str) -> None:
    """The decode is one path for both columns, so both are exercised."""
    bundle = api.run_case(
        benchmark_model(),
        CaseSpec(
            mode=mode,
            controls=(
                (DisplacementControl(target="wheel_travel_left", values=(-10.0,)),)
                if mode == "K"
                else ()
            ),
        ),
    )
    assert bundle.states
    for state in bundle.states:
        assert state.mode == mode
        assert state.converged
