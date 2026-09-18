"""
The dynamic axle family through the contract boundary.

This family is the one with a frozen byte-for-byte hash, so the bar is not
"agrees to a tolerance": the contract path has to reproduce what the ctypes path
produced, exactly, for every case in the acceptance matrix.  Anything less would
mean the boundary changed the answer, and a boundary that changes the answer is
not a boundary.

The comparison covers every array the result publishes for every case in
`_CASE_DURATIONS`: body states, constraint wrenches, per-sample diagnostics, and
the element, tire and energy ledgers.  The ledgers were scratch while the flat
ABI was the only route to them, so their equality used to be *argued* from the
states; now that the contract carries them it is asserted, which is also what
lets the flat route be retired without losing a number.  The artifact-level hash
remains the end-to-end gate.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import run_axle_dynamics
from suspension_multibody.cases import run_axle_dynamic_contract

_SCRIPT = (
    Path(__file__).resolve().parents[2] / "scripts" / "run_axle_dynamics_acceptance.py"
)

#: The diagnostic row the kernel writes, in the order it writes it.  The names
#: are the ones `AxleRunDiagnostics` exposes for the same columns.
_DIAGNOSTIC_FIELDS = (
    "accepted",
    "internal_steps",
    "rejected_attempts",
    "newton_iterations",
    "minimum_accepted_step_s",
    "maximum_accepted_step_s",
    "last_accepted_step_s",
    "position_residual",
    "velocity_residual",
    "dynamics_residual",
    "active_contacts",
    "contact_events",
    "local_error_ratio",
    "energy_residual",
    "failure_code",
    "pinned_null_directions",
)


def _acceptance():
    """Load the acceptance script, which owns the frozen model and cases."""
    spec = importlib.util.spec_from_file_location("axle_acceptance_fixture", _SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_DRIVEN_FIXTURE = (
    Path(__file__).resolve().parents[1] / "axle_dynamics" / "test_driven_coordinate.py"
)


def _driven_fixture():
    """Load the driven-coordinate fixture, which owns that model and its case."""
    spec = importlib.util.spec_from_file_location("driven_fixture", _DRIVEN_FIXTURE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_a_driven_coordinate_survives_the_contract_path() -> None:
    """
    A model with a driven coordinate must not lose its constraint row.

    The acceptance matrix does not cover this: its model declares no driven
    coordinates, so the case layer could drop them -- and did -- while every case
    in that matrix stayed bit-identical.  The *count* is what matters, because the
    wrench block is indexed by row: one missing row mislabels every row after it.
    The states agreed even when it was broken, because the axis was unloaded;
    a case that drives the coordinate is what makes the row observable.
    """
    fixture = _driven_fixture()
    model = fixture._translation_model(
        driven=(fixture._slide("slide"),), initial_velocity_m_per_s=(2.0, 0.0, 0.0)
    )
    case = fixture._case(
        samples=101,
        target=tuple(2.0 * 0.01 * index for index in range(101)),
        rate=tuple(2.0 for _ in range(101)),
    )
    reference = run_axle_dynamics(model, case)
    produced = run_axle_dynamic_contract(model, case)

    assert produced.block("constraint_wrench").shape == reference.constraint_wrench.shape, (
        "the contract path dropped a constraint row"
    )
    assert np.array_equal(produced.block("constraint_wrench"), reference.constraint_wrench)
    assert np.array_equal(produced.block("body_state"), reference.states)
    assert np.array_equal(produced.block("energy"), reference.energy)


def _diagnostics_matrix(result) -> np.ndarray:
    return np.column_stack(
        [
            np.asarray(getattr(result.diagnostics, field), dtype=float)
            for field in _DIAGNOSTIC_FIELDS
        ]
    )


@pytest.fixture(scope="module")
def acceptance():
    return _acceptance()


@pytest.mark.parametrize(
    "case_name",
    [
        "static_equilibrium",
        "road_pulse",
        "opposite_phase_road",
        "braking",
        "combined_load",
        "tire_liftoff_and_recontact",
    ],
)
def test_contract_path_reproduces_the_ctypes_path(acceptance, case_name: str) -> None:
    model = acceptance.build_axle_model()
    case = acceptance.build_case(case_name)
    reference = run_axle_dynamics(model, case)
    produced = run_axle_dynamic_contract(model, case)

    assert produced.status == "success"
    assert [entry["name"] for entry in produced.cases()] == [case_name]
    assert np.array_equal(produced.block("body_state"), reference.states)
    for block, ledger in (
        ("constraint_wrench", reference.constraint_wrench),
        ("tire_output", reference.tire_output),
        ("spring_output", reference.spring_output),
        ("bushing_output", reference.bushing_output),
        ("energy", reference.energy),
    ):
        if ledger.shape[1] == 0:
            # A descriptor cannot express a zero extent, so an empty ledger is
            # an absent block rather than a block of nothing.
            assert block not in produced.blocks
            continue
        assert np.array_equal(produced.block(block), ledger), block
    assert np.array_equal(
        produced.block("constraint_wrench"), reference.constraint_wrench
    )
    sample_count = reference.times_s.size
    assert np.array_equal(
        produced.block("diagnostics")[:sample_count], _diagnostics_matrix(reference)
    )


def test_the_model_document_declares_metres(acceptance) -> None:
    from suspension_multibody.cases import axle_dynamic_model_document

    document, _ = axle_dynamic_model_document(acceptance.build_axle_model())
    # The dynamic family's own objects are SI, so the document says so and the
    # kernel scales by one; a millimetre round trip would cost an ulp per value.
    assert document["units"]["length"] == "m"


def test_the_case_document_describes_its_sample_tables(acceptance) -> None:
    from suspension_multibody.cases.axle_dynamic import case_document

    model = acceptance.build_axle_model()
    case = acceptance.build_case("opposite_phase_road")
    document, blob = case_document(model, case)
    assert document["family"] == "axle_dynamic"
    assert document["time"]["step_s"] == pytest.approx(1e-3)
    roles = [(entry["role"], entry.get("tire")) for entry in document["blobs"]]
    assert roles == [
        ("road_height", "tire_l"),
        ("road_velocity", "tire_l"),
        ("road_height", "tire_r"),
        ("road_velocity", "tire_r"),
    ]
    # Every descriptor has to point at a real range of the blob it travels in.
    for entry in document["blobs"]:
        assert entry["offset"] + entry["length"] <= len(blob)
        assert entry["dtype"] == "float64"


def test_an_unknown_blob_role_is_rejected(acceptance) -> None:
    from suspension_multibody.cases.axle_dynamic import case_document
    from suspension_multibody.kernel import KernelContractError, run_contract

    model = acceptance.build_axle_model()
    # `static_equilibrium` carries no sample tables at all, so this needs a
    # case that does before there is a role to corrupt.
    document, blob = case_document(model, acceptance.build_case("road_pulse"))
    document["blobs"][0]["role"] = "road_curvature"
    from suspension_contracts import pack_container

    from suspension_multibody.cases import axle_dynamic_model_document

    with pytest.raises(KernelContractError, match="unknown blob role"):
        run_contract(
            axle_dynamic_model_document(model)[0],
            document,
            case_payload=pack_container(document, blob),
        )
