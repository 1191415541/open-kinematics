"""
P1 step 2, end to end: the audit consumes the *native* constraint derivative.

The synthetic-Jacobian tests in ``test_kinematic_response.py`` pin the audit's
logic, but they cannot show that the audit is wired to the channel the kernel
actually writes.  This module drives a real compile/run and feeds the resulting
``constraint_jacobian`` block into the audit, so the shape, the row ordering and
the units are the kernel's rather than a fixture's.

It also asserts the property the design turns on: the derivative must come from
the native channel and must *not* be a production finite difference.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import (
    CharacteristicsError,
    audit_boundary,
    rank_and_null_space,
    solve_motion,
)
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.simulation.runner import run_compiled

from ..results.test_measurement_queries import roll_model


def _run(kind: str) -> ResultEnvelope:
    run = {
        "schema_version": 1,
        "name": "linkage",
        "study": "dynamic",
        "samples": [0, 0.001],
        "boundaries": [],
        "inputs": [],
        "outputs": [],
        "solver": {"initialization_mode": "provided_consistent_state"},
    }
    result = run_compiled(
        compile_resolved(roll_model(kind), ResolvedSolvePlan(run))
    ).result
    assert isinstance(result, ResultEnvelope)
    return result


def test_the_native_channel_supplies_the_jacobian() -> None:
    """The audit must read the kernel's block, not recompute the constraint law."""
    result = _run("prismatic")
    assert "constraint_jacobian" in result.raw.blocks
    jacobian = np.asarray(result.raw.block("constraint_jacobian"))
    # (samples, rows, dof) as the ABI push_block call defines it.
    assert jacobian.ndim == 3
    samples, rows, dof = jacobian.shape
    assert samples == len(result.raw.states)
    # Derive the expected row count from the kernel's own manifest rather than
    # hard-coding a number the model is free to change.
    expected_rows = sum(row["row_count"] for row in result.raw.metadata["constraints"])
    assert rows == expected_rows
    assert rows > 0, "the fixture must actually constrain something"
    assert dof == 6 * len(result.raw.metadata["free_bodies"])
    assert np.isfinite(jacobian).all()


def test_the_audit_ranks_the_real_jacobian_per_sample() -> None:
    """
    The audit runs on the kernel's own numbers, sample by sample.

    The observed values for this fixture: 12 coordinates, 10 constraint rows,
    full row rank, so two directions stay free and the audit reports
    ``unidentifiable`` rather than inventing a motion.  Asserting the numbers
    (not just ``rank <= rows``) is what makes this an acceptance test: if the
    kernel changed what it emits, the audit would have to be re-judged.
    """
    result = _run("prismatic")
    jacobian = np.asarray(result.raw.block("constraint_jacobian"))
    dof = 6 * len(result.raw.metadata["free_bodies"])
    assert (jacobian.shape[1], jacobian.shape[2]) == (10, dof) == (10, 12)
    for sample in range(jacobian.shape[0]):
        audit = audit_boundary(jacobian[sample])
        assert audit["rank"] == 10
        assert audit["nullity"] == 2
        assert audit["unique"] is False
        assert audit["status"] == "unidentifiable"
        assert audit["rank"] + audit["nullity"] == audit["dof"] == dof


def test_rank_plus_nullity_always_accounts_for_every_coordinate() -> None:
    """A rank/nullity pair that does not sum to the dof is a silent miscount."""
    for kind in ("prismatic", "revolute"):
        result = _run(kind)
        jacobian = np.asarray(result.raw.block("constraint_jacobian"))
        for sample in range(jacobian.shape[0]):
            rank, null = rank_and_null_space(jacobian[sample])
            assert rank + null.shape[1] == jacobian.shape[2]
            if null.size:
                # Every reported null direction must genuinely be unconstrained.
                residual = jacobian[sample] @ null
                assert np.allclose(residual, 0.0, atol=1e-9)


def test_held_coordinates_close_the_null_space() -> None:
    """
    Adding the held rows must be able to remove the remaining freedom.

    This is what makes "hold this coordinate" a boundary rather than a hint: the
    audit shows the freedom shrinking from two directions to none as the held
    rows are supplied, and the rank rising to the full coordinate count.
    """
    result = _run("prismatic")
    jacobian = np.asarray(result.raw.block("constraint_jacobian"))[0]
    free = audit_boundary(jacobian)
    assert (free["rank"], free["nullity"]) == (10, 2)
    held = audit_boundary(jacobian, driven_rows=np.eye(jacobian.shape[1]))
    assert held["nullity"] == 0
    assert held["unique"] is True
    assert held["rank"] == jacobian.shape[1] == 12
    # 22 rows for 12 coordinates: the held rows make ten of them dependent, and
    # redundancy alone must not be reported as a defect.
    assert held["row_count"] == 22
    assert held["redundant_rows"] == 10
    assert held["empty_rows"] == 0
    assert held["status"] == "valid"


def test_a_difference_quotient_is_not_used_as_a_production_derivative() -> None:
    """
    The native block must be the derivative, not a per-sample difference of it.

    The mount geometry is identical at both samples, so the analytic rows are
    bit-identical across them.  A central difference over an unchanged state
    would return zeros and could not reproduce a rank-10 matrix; the matrix is
    also nonzero, which is what rules the difference route out.
    """
    result = _run("revolute")
    jacobian = np.asarray(result.raw.block("constraint_jacobian"))
    assert jacobian.shape[0] >= 2
    assert np.array_equal(jacobian[0], jacobian[1]), (
        "the constraint rows must be state-independent for this fixture"
    )
    assert np.abs(jacobian[0]).max() > 0.0
    assert np.linalg.matrix_rank(jacobian[0]) == 10


def test_an_all_zero_jacobian_is_reported_as_singular() -> None:
    """
    No row constrains anything, so the audit names the degeneracy directly.

    ``unidentifiable`` would also be true here (four free directions), but it
    would under-describe the sample: the boundary is broken, not merely loose.
    """
    with pytest.raises(CharacteristicsError, match="constrain nothing"):
        solve_motion(np.zeros((1, 4)), np.array([1.0]))
    audit = audit_boundary(np.zeros((1, 4)))
    assert audit["empty_rows"] == 1
    assert audit["status"] == "singular"
    assert audit["nullity"] == 4
