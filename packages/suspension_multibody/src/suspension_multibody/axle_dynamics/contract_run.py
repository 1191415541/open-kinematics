"""
The public axle-dynamics adapter for the versioned kernel contract.

``suspension_kernel_run`` takes one model document and one case document and
returns one result document.  This module authors neither document: the case
emitter does that.  It owns the product-facing half of the boundary -- decode the
result blocks, preserve the reporting types, and turn a failed run into the same
``NativeAxleError`` evidence a caller saw before the flat boundary was retired.
"""

from __future__ import annotations

import numpy as np

from .errors import NativeAxleError
from .result import (
    AxleContactEventRecord,
    AxleDynamicsResult,
    AxleRunDiagnostics,
    AxleRunPerformance,
)
from .schema import AxleDynamicsCase, AxleDynamicsModel

#: The diagnostics columns, in the order the kernel writes them.  The first
#: column is the per-sample accepted flag rather than a count.
DIAGNOSTIC_FIELDS = (
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

#: Rows one case reserves beyond its samples: the static trim solution and the
#: state it started from.  The kernel packs the run's performance counters into
#: them -- the first row whole, the second row's first eight columns.
_DIAGNOSTIC_TAIL_ROWS = 2

#: ``(field, is_integer)`` in the order the performance record is laid out.
PERFORMANCE_FIELDS: tuple[tuple[str, bool], ...] = (
    ("residual_calls", True),
    ("residual_time_s", False),
    ("constraint_jacobian_calls", True),
    ("constraint_jacobian_time_s", False),
    ("force_evaluations", True),
    ("force_time_s", False),
    ("mass_inverse_calls", True),
    ("mass_inverse_time_s", False),
    ("reaction_time_s", False),
    ("linear_factorizations", True),
    ("linear_factorization_time_s", False),
    ("linear_solves", True),
    ("linear_solve_time_s", False),
    ("line_search_trials", True),
    ("newton_iterations", True),
    ("accepted_steps", True),
    ("rejected_attempts", True),
    ("analytic_jacobian_columns", True),
    ("finite_difference_jacobian_columns", True),
    ("nonsmooth_fallback_columns", True),
    ("analytic_jacobian_time_s", False),
    ("finite_difference_jacobian_time_s", False),
    ("dynamic_integration_time_s", False),
)


def constraint_names(model: AxleDynamicsModel) -> tuple[str, ...]:
    """
    Return the constraint-row names, in the kernel's row order.

    A constraint row is a joint, a prescribed steering actuator, or a driven
    coordinate, and the model document emits them in that order.  Getting this
    wrong would not change a number -- it would mislabel one, which is worse.
    """
    return (
        *(joint.name for joint in model.joints),
        *(driven.name for driven in model.driven_coordinates),
    )


def run_axle_dynamics(
    model: AxleDynamicsModel, case: AxleDynamicsCase
) -> AxleDynamicsResult:
    """Run one validated SI axle case through the contract boundary."""
    from ..cases.axle_dynamic import run_axle_dynamic_contract
    from ..kernel import KernelContractError

    try:
        run = run_axle_dynamic_contract(model, case)
    except KernelContractError as error:
        partial = error.partial_run
        if partial is None:
            # The kernel rejected the payload rather than failing a run, so there
            # is no evidence to hand back.
            raise NativeAxleError(str(error), status=3) from error
        manifest = partial.document.get("manifest", {})
        index = int(manifest.get("failed_sample_index", 0))
        failure_diagnostics = failure_row(partial, index)
        raise NativeAxleError(
            str(error),
            status=int(manifest.get("failed_status", 3)),
            partial_result=build_result(model, case, partial, stop=index),
            failure_diagnostics=failure_diagnostics,
            failed_sample_index=index,
            failed_time_s=float(manifest.get("failed_time_s", 0.0)),
        ) from error
    return build_result(model, case, run)


def failure_row(run, index: int) -> np.ndarray:
    """Return the diagnostics row of the sample a run failed on."""
    entry = run.cases()[0]
    first = int(entry["sample_offset"])
    return run.block("diagnostics")[first + index].copy()


def build_result(
    model: AxleDynamicsModel,
    case: AxleDynamicsCase,
    run,
    *,
    stop: int | None = None,
) -> AxleDynamicsResult:
    """
    Map one contract result into the axle reporting type.

    ``stop`` truncates the answer to the samples that were actually solved, which
    is what a failed run has: the kernel writes the samples that converged and
    the one it failed on, so the partial result is the prefix before that row.
    """
    entry = run.cases()[0]
    declared = int(entry["sample_count"])
    sample_count = declared if stop is None else min(stop, declared)
    first = int(entry["sample_offset"])
    diagnostics = run.block("diagnostics")[first : first + sample_count]

    tail = run.block("diagnostics")[first + declared : first + declared + 2]
    performance_row = np.concatenate((tail[0], tail[1][:8]))

    def metric_int(index: int) -> int:
        value = performance_row[index]
        return int(value) if np.isfinite(value) else 0

    def metric_float(index: int) -> float:
        value = performance_row[index]
        return float(value) if np.isfinite(value) else 0.0

    performance = AxleRunPerformance(
        available=bool(np.isfinite(performance_row[0]) and performance_row[0] > 0.5),
        **{
            name: metric_int(index + 1) if is_integer else metric_float(index + 1)
            for index, (name, is_integer) in enumerate(PERFORMANCE_FIELDS)
        },
    )

    tire_names = tuple(tire.name for tire in model.tires)
    events = run.blocks.get("contact_events")
    contact_events = (
        ()
        if events is None
        else tuple(
            AxleContactEventRecord(
                time_s=float(row[0]),
                tire=tire_names[int(row[1])],
                transition="enter" if int(row[2]) > 0 else "exit",
            )
            for row in events
        )
    )

    def ledger(name: str, count: int, width: int) -> np.ndarray:
        """Return one ledger, empty when the descriptor omitted the block."""
        if count == 0:
            return np.zeros((sample_count, 0, width), dtype=np.float64)
        return run.block(name)[first : first + sample_count]

    return AxleDynamicsResult(
        times_s=np.asarray(case.times_s[:sample_count], dtype=np.float64),
        body_names=tuple(str(name) for name in run.document["manifest"]["bodies"]),
        constraint_names=constraint_names(model),
        spring_names=tuple(spring.name for spring in model.springs),
        bushing_names=tuple(bushing.name for bushing in model.bushings),
        anti_roll_bar_names=tuple(bar.name for bar in model.anti_roll_bars),
        tire_names=tire_names,
        states=run.block("body_state")[first : first + sample_count],
        constraint_wrench=run.block("constraint_wrench")[first : first + sample_count],
        spring_output=ledger("spring_output", len(model.springs), 7),
        bushing_output=ledger("bushing_output", len(model.bushings), 12),
        anti_roll_output=ledger("anti_roll_output", len(model.anti_roll_bars), 3),
        diagnostics=AxleRunDiagnostics(  # ty: ignore[missing-argument]
            **{
                field: diagnostic_column(diagnostics, index, field)
                for index, field in enumerate(DIAGNOSTIC_FIELDS)
            }
        ),
        tire_output=ledger("tire_output", len(model.tires), 41),
        energy=run.block("energy")[first : first + sample_count],
        contact_events=contact_events,
        performance=performance,
    )


def diagnostic_column(rows: np.ndarray, index: int, field: str) -> np.ndarray:
    """Read one diagnostics column, as the type the reporting field declares."""
    column = rows[:, index]
    if field == "accepted":
        return column.astype(bool)
    if field in {
        "internal_steps",
        "rejected_attempts",
        "newton_iterations",
        "active_contacts",
        "contact_events",
        "failure_code",
        "pinned_null_directions",
    }:
        return column.astype(int)
    return column
