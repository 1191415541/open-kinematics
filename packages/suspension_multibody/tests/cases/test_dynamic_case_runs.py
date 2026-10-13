"""
One dynamic document, several runs: the `runs` case form end to end.

A time-history family used to name exactly one excitation per document, so
"compare two excitations" could only be done by submitting two documents.  The
`runs` list states them in one document, on one shared grid.  The tests below are
about the parts only the whole list can get wrong:

* every run becomes exactly one case, named as the run is;
* a run's own table reaches that run and no other;
* the shared grid is the document's, so the per-case states stay comparable;
* a form the kernel cannot honour is *refused* -- at the case layer, and one
  layer earlier where the caller can still read the reason -- never silently
  dropped.
"""
from __future__ import annotations

import copy
import re

import numpy as np
import pytest
from suspension_contracts import ContractError, pack_container

from suspension_multibody.api import validate
from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
from suspension_multibody.axle_dynamics import AxleDynamicsCase, AxleSolverSettings
from suspension_multibody.characteristics.values import CharacteristicsError
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.kernel import KernelContractError, run_contract
from suspension_multibody.kernel.capabilities import (
    kernel_capability_document,
    require_case_runs_version,
)
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.results.sampling import select_samples
from suspension_multibody.simulation.runner import run_compiled
from tests.axle_dynamics.test_integrator import _oscillator
from tests.benchmark_fixture import benchmark_model
from tests.cases.kc_quasi_static._documents import documents, native_case

TIMES = (0.0, 0.01, 0.02)
#: The body the oscillator's moving mass is exported under.
BODY = "body.body"


def _settings() -> AxleSolverSettings:
    return AxleSolverSettings(
        initialization_mode="provided_consistent_state",
        adaptive_step=False,
        internal_step_s=0.0005,
        minimum_step_s=1e-9,
        maximum_step_s=0.004,
        max_newton_iterations=30,
    )


def _wrench(name: str, force_y: float, *, run: str | None = None, times=TIMES) -> dict:
    """One constant lateral body wrench, in the table shape the plan carries."""
    row = {
        "name": name,
        "role": "body_wrench",
        "body": BODY,
        "values": [[0.0, force_y, 0.0, 0.0, 0.0, 0.0]] * len(times),
    }
    if run is not None:
        row["run"] = run
    return row


def _plan(runs, inputs=(), *, outputs=("accepted_state",), times=TIMES, name="runs"):
    """Return the resolved model and a dynamic plan whose excitation is `runs`."""
    assembly, case = migrate_v1_dynamic_axle(
        _oscillator(), AxleDynamicsCase(name=name, times_s=times, solver=_settings())
    )
    original = validate(assembly, case)
    document = original.request.case.to_document()
    document["outputs"] = list(outputs)
    document["excitation"] = {"runs": runs}
    document["inputs"] = list(inputs)
    return original.request.model, ResolvedSolvePlan(
        document, original.request.case.input_payload
    )


def _compile(runs, inputs=(), **kwargs):
    model, plan = _plan(runs, inputs, **kwargs)
    return compile_resolved(model, plan)


@pytest.fixture(scope="module")
def two_runs():
    """Return the two-run run: the object almost every assertion below is about."""
    compiled = _compile(
        [{"name": "left"}, {"name": "right"}],
        [
            _wrench("left-wrench", 100.0, run="left"),
            _wrench("right-wrench", -100.0, run="right"),
        ],
    )
    return compiled, run_compiled(compiled)


@pytest.fixture(scope="module")
def one_run():
    """Return the same document with only its first run, as a per-run reference."""
    compiled = _compile([{"name": "left"}], [_wrench("left-wrench", 100.0, run="left")])
    return run_compiled(compiled)


def _accepted_points(raw, case_index: int):
    """Return the stored accepted identities of one case, in sampling order."""
    return [
        point
        for point in select_samples(raw, "accepted").identities
        if point.case_index == case_index
    ]


# --------------------------------------------------------------------------- #
# one run per case, on the document's own grid
# --------------------------------------------------------------------------- #


def test_every_run_becomes_one_case_and_the_names_are_the_runs(two_runs):
    """
    Two runs are two cases, and the manifest names them as the runs are named.

    The plan's own name stays the document's: a reader looking for the *study*
    must not be handed the last run's name, which is what a manifest that
    reported the merged document would do.
    """
    compiled, run = two_runs
    assert int(run.raw.document["manifest"]["case_count"]) == 2
    assert [entry["name"] for entry in run.raw.cases] == ["left", "right"]
    assert run.raw.document["manifest"]["case_name"] == "runs"
    assert run.status == "success"
    assert compiled.case_document["runs"][0]["name"] == "left"
    assert compiled.case_document["runs"][1]["name"] == "right"


def test_both_cases_are_solved_on_the_document_time_grid(two_runs):
    """
    One grid, shared, and it is the document's own.

    The kernel sizes its per-case tables from the *first* case, so a run on a
    shorter grid would be a heap overrun inside the kernel rather than a wrong
    number here.  The grid is asserted as measured, from the decoded times of
    both cases, rather than as the intent recorded in the document.
    """
    _, run = two_runs
    assert [int(entry["sample_count"]) for entry in run.raw.cases] == [3, 3]
    times = np.asarray(run.raw.times_s, dtype=float)
    assert times.size == 6
    np.testing.assert_array_equal(times[:3], np.asarray(TIMES))
    np.testing.assert_array_equal(times[:3], times[3:])
    firsts = [_accepted_points(run.raw, case)[0].time_s for case in (0, 1)]
    assert firsts == [0.0, 0.0]
    last = _accepted_points(run.raw, 1)[-1]
    assert last.time_s == pytest.approx(TIMES[-1])


def test_each_run_applies_its_own_table_and_only_its_own(two_runs, one_run):
    """
    The per-run wrench reaches its own case, byte for byte.

    Case 0 of the two-run document carries exactly what the one-run document
    carries, and case 1 does not: without both halves, a merge that dropped
    every run's table or applied one run's table twice would still produce a
    plausible pair of trajectories.
    """
    _, run = two_runs
    states = run.raw.block("body_state")
    reference = one_run.raw.block("body_state")
    assert states.shape[0] == 2 * reference.shape[0]
    np.testing.assert_array_equal(states[: reference.shape[0]], reference)
    assert not np.array_equal(states[reference.shape[0] :], reference)
    # The two runs really differ, so "case 1 differs" is not a tautology of the
    # fixture: their wrenches are opposite.
    assert not np.array_equal(states[: reference.shape[0]], states[reference.shape[0] :])


def test_a_shared_document_table_stays_visible_to_every_run():
    """
    A run's table supplements the document's; it does not replace the list.

    The document-level ``solve-plan-samples`` descriptor has to survive the
    merge: it is the grid every run is solved on, and a merge that let one run's
    blob array overwrite the document's would have taken the grid away from the
    others.
    """
    compiled = _compile(
        [{"name": "left"}, {"name": "right"}],
        [
            _wrench("shared-wrench", 5.0),
            _wrench("left-wrench", 100.0, run="left"),
            _wrench("right-wrench", -100.0, run="right"),
        ],
        times=(0.0, 0.005, 0.02),
    )
    document = compiled.case_document
    assert [row["name"] for row in document["blobs"]] == [
        "solve-plan-samples",
        "shared-wrench",
    ]
    assert [row["name"] for row in document["runs"][0]["blobs"]] == ["left-wrench"]
    assert [row["name"] for row in document["runs"][1]["blobs"]] == ["right-wrench"]
    run = run_compiled(compiled)
    assert [entry["name"] for entry in run.raw.cases] == ["left", "right"]
    full = run.raw.block("body_state")
    assert full.shape[0] == 2 * len(TIMES)
    assert not np.array_equal(full[:3], full[3:])


# --------------------------------------------------------------------------- #
# the accepted grid of a real multi-case dynamic run
# --------------------------------------------------------------------------- #


def test_the_accepted_grid_keeps_one_counter_per_case(two_runs):
    """
    Accepted ids restart in every case, and the case resolves the ambiguity.

    This is the gap the multi-case reader was written for: the same integer is a
    real accepted step in every case, so a bare id must be refused rather than
    answered from whichever case the reader happened to walk first.
    """
    _, run = two_runs
    firsts = [
        select_samples(run.result, "accepted", ids=[0], case_index=case).identities[0]
        for case in (0, 1)
    ]
    assert [point.case_index for point in firsts] == [0, 1]
    assert [point.accepted_step_id for point in firsts] == [0, 0]
    counts = [len(_accepted_points(run.raw, case)) for case in (0, 1)]
    # More accepted steps than output samples: the accepted grid is its own, so
    # this is not the output grid under another name.
    assert counts[0] == counts[1] > len(TIMES), counts
    assert [point.index for point in firsts] == [0, counts[0]]
    with pytest.raises(CharacteristicsError, match="more than one case"):
        select_samples(run.result, "accepted", ids=[0])


def test_the_accepted_states_of_the_two_cases_are_different(two_runs, one_run):
    """The persisted checkpoints are per case, not one case's states relabelled."""
    _, run = two_runs
    states = run.raw.block("accepted_body_state")
    reference = one_run.raw.block("accepted_body_state")
    count = len(_accepted_points(run.raw, 0))
    assert states.shape[0] == 2 * count == 2 * reference.shape[0]
    np.testing.assert_array_equal(states[:count], reference)
    assert not np.array_equal(states[:count], states[count : 2 * count])


# --------------------------------------------------------------------------- #
# the case layer refuses a form it cannot honour
# --------------------------------------------------------------------------- #


@pytest.fixture(scope="module")
def submitted():
    """Return the one-run submission, ready to be mutated one field at a time."""
    return _compile([{"name": "left"}], [_wrench("left-wrench", 100.0, run="left")])


def _submit(submitted, mutate):
    """Submit one mutated case document; exactly one of (error, run) is None."""
    case = copy.deepcopy(submitted.case_document)
    payload = bytearray(submitted.case_payload)
    mutate(case, payload)
    try:
        run = run_contract(
            submitted.model_document,
            case,
            model_payload=submitted.model_payload,
            case_payload=pack_container(case, bytes(payload)),
        )
    except KernelContractError as error:
        return str(error), None
    return None, run


@pytest.mark.parametrize(
    "mutate,message",
    [
        (
            lambda case, payload: case.update({"family": "kc_quasi_static"}),
            'case family "kc_quasi_static" expands its own case grid and does not '
            "read a runs list",
        ),
        (
            lambda case, payload: case["runs"][0].update({"time": {}}),
            'runs entry "left" may not override "time" for family axle_dynamic',
        ),
        (
            lambda case, payload: case["runs"][0].update({"solver": {}}),
            'runs entry "left" may not override "solver" for family axle_dynamic',
        ),
        (
            lambda case, payload: case["runs"][0].update({"outputs": {}}),
            'runs entry "left" may not override "outputs" for family axle_dynamic',
        ),
        (
            lambda case, payload: case["runs"][0].update(
                {"characteristics": {"schema_version": 1}},
            ),
            'runs entry "left" may not override "characteristics" for family '
            "axle_dynamic",
        ),
        (
            lambda case, payload: case["runs"][0].update({"inputs": {"motion": []}}),
            'runs entry "left" may not declare motion: the document\'s motion '
            "applies to every run",
        ),
        (lambda case, payload: case.update({"runs": []}), "runs must not be empty"),
        (
            lambda case, payload: case["runs"][0].pop("name"),
            "a runs entry needs a name",
        ),
        (
            lambda case, payload: case["runs"][0].update({"name": ""}),
            "a runs entry needs a name",
        ),
        (
            lambda case, payload: case["runs"].append({"name": "left"}),
            'two runs share the name "left"',
        ),
        (lambda case, payload: case.pop("name"), "case document has no name"),
        (
            lambda case, payload: case.update({"runs": {"left": {}}}),
            "runs must be an array",
        ),
        (
            lambda case, payload: case["runs"].__setitem__(0, "left"),
            "a runs entry must be an object",
        ),
        (
            lambda case, payload: case["runs"][0].update(
                {"blobs": [{"role": "body_wrench"}]},
            ),
            "a runs entry blob descriptor needs a name",
        ),
        (
            lambda case, payload: case["runs"][0].update({"blobs": {"name": "x"}}),
            'runs entry "left" blobs must be an array',
        ),
        (
            lambda case, payload: case["runs"][0].update({"inputs": []}),
            'runs entry "left" inputs must be an object',
        ),
    ],
)
def test_the_case_layer_refuses_a_run_list_it_cannot_honour(
    submitted, mutate, message
):
    """
    Every refusal names the reason, and the reason is the run's own.

    The refusals are the point of the whitelist: a key the family never reads
    per run would otherwise be accepted and silently dropped, and the caller
    would read a successful run that ignored half of what it asked for.  The
    nameless-descriptor case is the same failure at one remove -- the document
    here carries no table of its own, so that descriptor would become the run's
    whole list, unusable and unnamed.
    """
    error, run = _submit(submitted, mutate)
    assert run is None, "the kernel accepted a document it must refuse"
    assert message in error, error


def test_a_run_on_another_grid_is_refused_before_the_kernel_sizes_its_buffers():
    """
    A run with its own shorter grid is refused, not copied behind a short buffer.

    The kernel allocates its per-case tables from the *first* case's sample
    count and copies each run's table into them, so a later run with fewer
    samples would be a heap overrun.  The document's grid is three points here
    and the second run asks for two.
    """
    submitted = _compile(
        [{"name": "left"}, {"name": "right"}],
        [
            _wrench("left-wrench", 100.0, run="left"),
            _wrench("right-wrench", -100.0, run="right"),
        ],
    )

    def mutate(case, payload):
        shared_at = len(submitted.case_payload)
        run_at = shared_at + 24
        case["time"] = {"samples": "shared-times"}
        case["blobs"] = [
            {
                "name": "shared-times",
                "role": "sample_times",
                "offset": shared_at,
                "length": 24,
                "shape": [3],
                "dtype": "float64",
                "order": "C",
            }
        ]
        case["runs"][1]["blobs"] = [
            {
                "name": "shared-times",
                "role": "sample_times",
                "offset": run_at,
                "length": 16,
                "shape": [2],
                "dtype": "float64",
                "order": "C",
            }
        ]
        payload.extend(np.asarray(TIMES, dtype="<f8").tobytes())
        payload.extend(np.asarray(TIMES[:2], dtype="<f8").tobytes())

    error, run = _submit(submitted, mutate)
    assert run is None
    assert "every run must be solved on the document's time grid" in error, error


def test_the_same_short_grid_on_its_own_is_accepted():
    """
    Guard: the refusal above is about the *disagreement*, not about a short run.

    Every run on two points is a legal document -- the grid is simply the
    document's own -- so a guard that refused the shape outright would be
    rejecting a form the kernel can run.
    """
    compiled = _compile(
        [{"name": "left"}, {"name": "right"}],
        [
            _wrench("left-wrench", 100.0, run="left", times=TIMES[:2]),
            _wrench("right-wrench", -100.0, run="right", times=TIMES[:2]),
        ],
        times=TIMES[:2],
    )
    run = run_compiled(compiled)
    assert [entry["name"] for entry in run.raw.cases] == ["left", "right"]
    assert [int(entry["sample_count"]) for entry in run.raw.cases] == [2, 2]


def test_a_run_may_name_the_keys_its_own_family_reads(submitted):
    """
    The whitelist is not an empty list: the keys a family does read are allowed.

    Without this the refusals above would pass on a table that refused
    everything, which would make the form unreachable rather than safe.
    """

    def mutate(case, payload):
        case["runs"][0]["blobs"] = []
        case["runs"][0]["inputs"] = {}

    error, run = _submit(submitted, mutate)
    assert error is None, error
    assert [entry["name"] for entry in run.cases()] == ["left"]


# --------------------------------------------------------------------------- #
# one layer earlier, where the caller can still read the reason
# --------------------------------------------------------------------------- #


def test_the_compiler_refuses_a_run_list_on_a_grid_family():
    """
    A grid family names its cases from its own sweep, so a list is a second source.

    The case layer refuses the same document; this is the refusal one layer
    earlier, where the caller still has the document in hand.
    """
    pair = documents(benchmark_model(), "K")
    document = native_case(
        pair,
        {
            "contract": "multibody-case",
            "contract_version": 1,
            "kind": "case",
            "family": "kc_quasi_static",
            "name": "axes-grid",
            "time": {"start_s": 0.0, "end_s": 1e-3, "step_s": 1e-3},
            "k": {
                "axes": [{"coordinate": "rack_drive", "values_mm": [-5.0, 0.0, 5.0]}],
                "drive": "wheel_center",
            },
            "runs": [{"name": "left"}],
        },
    )
    with pytest.raises(ValueError, match="cannot take a runs list"):
        validate(pair[0], document)


def test_the_compiler_refuses_a_duplicate_run_name():
    """Two cases with one name are indistinguishable in the manifest."""
    model, plan = _plan([{"name": "a"}, {"name": "a"}])
    with pytest.raises(ValueError, match="two runs with one name"):
        compile_resolved(model, plan)

def test_a_table_that_names_an_undeclared_run_is_refused():
    """
    A table's run field has to point at a declared run.

    The alternative is the failure the whole form exists to prevent: the row's
    table is placed in the document, no case claims it, and the run reports
    success while its excitation never arrived.
    """
    model, plan = _plan([{"name": "left"}], [_wrench("w", 1.0, run="other")])
    with pytest.raises(ValueError, match="undeclared run 'other'"):
        compile_resolved(model, plan)


@pytest.mark.parametrize(
    "runs,message",
    [
        ([{}], "missing required field 'name'"),
        ([], "needs at least 1 items"),
        ([{"name": "a", "time": {}}], "unexpected fields ['time']"),
        ({"left": {}}, "expected array, got dict"),
    ],
)
def test_the_plan_schema_refuses_a_malformed_run_list(runs, message):
    """The published case schema is the first refusal, so it must name the field."""
    with pytest.raises(ContractError, match=re.escape(message)):
        _plan(runs)


def test_the_loaded_kernel_declares_the_run_list_version():
    """
    The capability is read from the library, and the check is fail-closed.

    A build that did not declare it would expand the shared excitation once and
    report a single case, quietly dropping every other run; this check is what
    turns that into a refusal instead.
    """
    assert 1 in kernel_capability_document()["case_runs_versions"]
    with pytest.raises(ValueError, match="does not support case runs version 2"):
        require_case_runs_version(2)
