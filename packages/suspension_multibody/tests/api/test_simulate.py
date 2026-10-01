"""
`simulate(assembly_document, case_document)`: the document-driven public entry.

The entry is a thin wrapper, and these tests are written against that claim: an
assembly *file* plus a case *document* go in, and what comes back is the same
neutral run the object entry produces -- `run.status`, `run.raw` states, and the
compiled submission beside them.  Nothing here asserts a new physics; what it
asserts is that a document pair reaches the existing assembler and the existing
runner, that the reading follows from the documents, and that a request the
documents do not answer is refused by name instead of guessed at.
"""

from __future__ import annotations

import inspect
import json
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from suspension_multibody import api
from suspension_multibody.authoring.documents import (
    AssemblyDocument,
    SimulationAssembly,
)
from suspension_multibody.axle_dynamics.schema import AxleSolverSettings
from tests.authoring.fixtures import (
    write_builtin_axle_project,
    write_c_ready_axle_project,
)


def _case_document(name: str, sections: dict[str, Any]) -> dict[str, Any]:
    """Return one case contract document carrying `sections` as its reading."""
    return {
        "contract": "multibody-case",
        "contract_version": 1,
        "kind": "case",
        "family": "kc_quasi_static",
        "name": name,
        "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
        "solver": AxleSolverSettings().model_dump(mode="json"),
        **sections,
    }


#: The K reading: two wheel travels driven together, no rack sweep.
_K_SECTION: dict[str, Any] = {
    "k": {
        "wheel_values_mm": [0.0, 20.0],
        "rack_values_mm": [],
        "drive": "wheel_center",
        "left_right_mode": "symmetric",
        "axis_map": {"wheel": ["wheel_drive_L", "wheel_drive_R"]},
    }
}

#: The C reading: a vertical load path at the left wheel centre.
_C_SECTION: dict[str, Any] = {
    "c": {
        "paths": ["fz"],
        "levels": 3,
        "maximum": 1000.0,
        "side_mode": "single",
        "load_marker": "wheel_center_L",
    }
}


def _project(
    root: Path, write: Any, bench: str | None = "kc_quasi_static"
) -> dict[str, Path]:
    """
    Write one file project, and bind its rig to a bench when one is named.

    The fixture's rig declares ports and measurements but no bench, because the
    bench is what *drives* the model and the fixtures predate that field.  A
    simulatable project's rig names it, which is what `simulate` reads.
    """
    directory = root / "project"
    directory.mkdir(parents=True, exist_ok=True)
    paths = write(directory)
    if bench is None:
        return paths
    payload = json.loads(paths["rig"].read_text(encoding="utf-8"))
    payload["bench"] = bench
    paths["rig"].write_text(json.dumps(payload), encoding="utf-8")
    return paths


# --- the entry is public ----------------------------------------------------


def test_simulate_is_a_public_name() -> None:
    """`from suspension_multibody import simulate` works, and `__all__` says so."""
    import suspension_multibody as package

    assert callable(package.simulate)
    assert package.simulate is api.simulate
    assert "simulate" in package.__all__
    assert package._PUBLIC_NAMES["simulate"] == (".api", "simulate")


def test_simulate_has_a_signature_and_a_documented_contract() -> None:
    """The signature names the two documents, and the docstring says what they are."""
    signature = inspect.signature(api.simulate)
    assert list(signature.parameters) == ["assembly_document", "case_document"]
    # No hidden mode/family parameter: the reading follows from the documents.
    assert all(
        parameter.default is inspect.Parameter.empty
        for parameter in signature.parameters.values()
    )
    docstring = inspect.getdoc(api.simulate) or ""
    assert docstring.startswith(
        "Run one assembly document against one case document."
    )
    assert "``assembly_document``" in docstring
    assert "``case_document``" in docstring


def test_simulate_only_compiles_and_submits() -> None:
    """
    The entry is a wrapper, and the source says which existing layers it uses.

    A second assembly path is the failure mode this entry exists to avoid, so the
    check is that it names the authoring conversion, the family preparation and
    the shared runner rather than re-implementing any of them.
    """
    source = inspect.getsource(api.simulate)
    assert "assembly_request_for" in source
    assert "front_axle_model_for" in source
    assert "preparation.kc_quasi_static" in source
    assert "run_request" in source


# --- the end-to-end run -----------------------------------------------------


def test_a_k_document_pair_runs_end_to_end(tmp_path: Path) -> None:
    """One K case document over one assembly file reaches the kernel and solves."""
    paths = _project(tmp_path, write_builtin_axle_project)
    run = api.simulate(paths["assembly"], _case_document("k-run", _K_SECTION))

    assert run.status == "success", run.raw.failure_evidence
    assert len(run.raw.cases) == len(_K_SECTION["k"]["wheel_values_mm"])
    states = run.raw.states
    assert states.shape[0] == sum(int(entry["sample_count"]) for entry in run.raw.cases)
    assert states.size
    assert bool(np.isfinite(states).all()), "a solved K run reports finite body states"


def test_the_document_pair_reaches_the_same_submission(tmp_path: Path) -> None:
    """
    The compiled submission carries the bench, family and reading the documents state.

    This is what "the documents say what the run is" means concretely: the request
    the runner was handed names the rig the assembly is bound to and the family the
    case declares, and the model document the compiled pair carries is authored
    from the assembly the family preparation built.
    """
    paths = _project(tmp_path, write_builtin_axle_project)
    run = api.simulate(paths["assembly"], _case_document("k-run", _K_SECTION))

    request = run.request
    assert request.assembly == "axle"
    assert request.rig == "kc_quasi_static"
    assert request.family == "kc_quasi_static"
    assert request.study == "quasi_static"
    assert run.compiled.case_document["name"] == "k-run"
    assert run.compiled.model_document["contract"] == "multibody-model"
    # The wheel centres the case drives are the markers the model declares.
    markers = {marker["name"] for marker in run.compiled.model_document["markers"]}
    assert markers == {"wheel_center_L", "wheel_center_R"}


def test_a_c_document_pair_runs_end_to_end(tmp_path: Path) -> None:
    """The same entry, and the reading is the one the `c` section names."""
    paths = _project(tmp_path, write_c_ready_axle_project)
    run = api.simulate(paths["assembly"], _case_document("c-run", _C_SECTION))

    assert run.status == "success", run.raw.failure_evidence
    assert len(run.raw.cases) == _C_SECTION["c"]["levels"]
    assert run.request.family == "kc_quasi_static"


def test_the_assembly_document_s_override_reaches_the_model(tmp_path: Path) -> None:
    """
    An assembly file's own override is what the run reads.

    The override is copy-on-write over the subsystem file, so a run that ignored
    it would still solve -- against the un-overridden coordinate.  Moving the wheel
    centre through the assembly file is the cheapest way to show that the
    difference is the document's.
    """
    paths = _project(tmp_path, write_builtin_axle_project)
    before = api.simulate(paths["assembly"], _case_document("k-run", _K_SECTION))
    restated = {
        marker["name"]: marker["point"]
        for marker in before.compiled.model_document["markers"]
    }
    assert restated["wheel_center_L"] == [0.0, -700.0, 300.0]

    payload = json.loads(paths["assembly"].read_text(encoding="utf-8"))
    payload["subsystems"][0]["overrides"] = {
        "hardpoints": {"wheel_center": [0.0, -700.0, 340.0]}
    }
    paths["assembly"].write_text(json.dumps(payload), encoding="utf-8")

    after = api.simulate(paths["assembly"], _case_document("k-run", _K_SECTION))
    moved = {
        marker["name"]: marker["point"]
        for marker in after.compiled.model_document["markers"]
    }
    assert moved["wheel_center_L"] == [0.0, -700.0, 340.0]
    assert after.status == "success", after.raw.failure_evidence


@pytest.mark.parametrize("spelling", ["path", "document", "simulation"])
def test_the_three_assembly_spellings_are_one_run(tmp_path: Path, spelling: str) -> None:
    """A path, a loaded document and a rig-bound one all describe the same run."""
    paths = _project(tmp_path, write_builtin_axle_project)
    target: Any
    if spelling == "path":
        target = paths["assembly"]
    elif spelling == "document":
        target = AssemblyDocument.load(paths["assembly"])
    else:
        target = SimulationAssembly.load(paths["assembly"])

    run = api.simulate(target, _case_document("k-run", _K_SECTION))
    assert run.status == "success", run.raw.failure_evidence
    assert run.request.rig == "kc_quasi_static"


# --- refusals ---------------------------------------------------------------


def test_a_document_carrying_both_readings_is_refused(tmp_path: Path) -> None:
    """`k` and `c` together describe two runs, so neither is chosen silently."""
    paths = _project(tmp_path, write_builtin_axle_project)
    both = _case_document("both", {**_K_SECTION, **_C_SECTION})
    with pytest.raises(ValueError, match="exactly one of"):
        api.simulate(paths["assembly"], both)


def test_a_document_carrying_no_reading_is_refused(tmp_path: Path) -> None:
    """A case with nothing to solve says so rather than running an empty grid."""
    paths = _project(tmp_path, write_builtin_axle_project)
    with pytest.raises(ValueError, match="neither"):
        api.simulate(paths["assembly"], _case_document("none", {}))


def test_a_case_for_another_family_is_refused(tmp_path: Path) -> None:
    paths = _project(tmp_path, write_builtin_axle_project)
    document = _case_document("handling", _K_SECTION)
    document["family"] = "handling"
    with pytest.raises(ValueError, match="kc_quasi_static"):
        api.simulate(paths["assembly"], document)


def test_a_rig_without_a_bench_is_refused_by_name(tmp_path: Path) -> None:
    """
    A rig that names no bench has nothing to drive with.

    The fixtures' rigs predate the field, which makes them the natural negative
    control: the refusal names the registered benches rather than defaulting to
    one, because a default would decide the bench for the caller.
    """
    paths = _project(tmp_path, write_builtin_axle_project, bench=None)
    with pytest.raises(ValueError, match="declares no 'bench'"):
        api.simulate(paths["assembly"], _case_document("k-run", _K_SECTION))


def test_a_rig_whose_bench_routes_elsewhere_is_refused(tmp_path: Path) -> None:
    """The bench and the case have to agree about which family reads the model."""
    paths = _project(tmp_path, write_builtin_axle_project, bench="vehicle_kc")
    with pytest.raises(ValueError, match="routes to family"):
        api.simulate(paths["assembly"], _case_document("k-run", _K_SECTION))


def test_a_contract_model_document_is_not_an_assembly_document() -> None:
    """
    The entry takes an *assembly* document, and a model document is not one.

    A contract model document does not say which assembly it belongs to, so
    accepting it here would mean guessing the device under test; the pair of
    already-authored contract documents goes to `run_request` instead.
    """
    with pytest.raises(TypeError, match="assembly document"):
        api.simulate(
            {"contract": "multibody-model", "contract_version": 1, "kind": "model"},
            _case_document("k-run", _K_SECTION),
        )


def test_a_non_document_case_is_refused() -> None:
    with pytest.raises(TypeError, match="case contract document"):
        api.simulate("assembly.json", "not-a-document")


def test_the_legacy_entries_are_still_reachable() -> None:
    """
    The strangler guarantee: adding `simulate` removed nothing.

    `FrontAxleModel` and the two object entries are what every historical caller
    uses, and they stay public names of their own.
    """
    import suspension_multibody as package

    assert "FrontAxleModel" in package.__all__
    assert "run_case" in package.__all__
    assert "run_dynamic_case" in package.__all__
    assert package.FrontAxleModel is not None
    assert callable(package.run_case)
    assert callable(package.run_dynamic_case)
