"""The ordinary compiler/runner persists real accepted states on request."""
from __future__ import annotations

from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.api import validate
from suspension_multibody.authoring.migration import migrate_v1_dynamic_axle
from suspension_multibody.axle_dynamics import AxleDynamicsCase, AxleSolverSettings
from suspension_multibody.compilation.resolved import compile_resolved
from suspension_multibody.io.artifacts import read_artifact, write_artifact
from suspension_multibody.kernel import KernelContractError
from suspension_multibody.modeling.resolved import ResolvedSolvePlan
from suspension_multibody.results.characteristics import envelope_from_artifact
from suspension_multibody.results.envelope import ResultEnvelope
from suspension_multibody.results.sampling import select_samples
from suspension_multibody.simulation.runner import run_compiled
from tests.axle_dynamics.test_contact import _drop_model
from tests.axle_dynamics.test_driven_coordinate import _translation_model
from tests.axle_dynamics.test_integrator import _oscillator


def _compile(*, adaptive: bool, observed: bool, model=None, times=(0., .01, .02), wrench=None):
    model = _oscillator() if model is None else model
    settings = AxleSolverSettings(initialization_mode="provided_consistent_state",
        adaptive_step=adaptive, internal_step_s=.004 if adaptive else .0005,
        minimum_step_s=1e-9, maximum_step_s=.004, max_newton_iterations=30)
    assembly, case = migrate_v1_dynamic_axle(model, AxleDynamicsCase(
        name="observed-oscillator", times_s=times, solver=settings,
        body_wrench_n_n_m={} if wrench is None else wrench))
    original = validate(assembly, case)
    plan_doc = original.request.case.to_document()
    plan_doc["outputs"] = ["accepted_state"] if observed else []
    plan = ResolvedSolvePlan(plan_doc, original.request.case.input_payload)
    return compile_resolved(original.request.model, plan)


def _run(**kwargs):
    return run_compiled(_compile(**kwargs))


def _ledger_rows(raw, case_index):
    """
    Return the diagnostics rows holding ``case_index``'s samples.

    The ledger interleaves two per-case bookkeeping rows *after* that case's samples,
    so a bare sample offset reads the previous case's counters when there is more than
    one case.  The writer's layout is what this mirrors: ``_DIAGNOSTIC_TAIL_ROWS`` in
    ``raw.py`` applies the same shift to every ledger reader.
    """
    entry = raw.cases[case_index]
    start = int(entry["sample_offset"]) + 2*case_index
    return slice(start, start + int(entry["sample_count"]))


@pytest.mark.parametrize("adaptive", [False, True])
def test_original_dynamic_run_and_every_accepted_state_are_preserved(adaptive):
    plain = _run(adaptive=adaptive, observed=False)
    observed = _run(adaptive=adaptive, observed=True)
    assert "accepted_body_state" not in plain.raw.blocks
    for name, values in plain.raw.blocks.items():
        assert np.array_equal(values, observed.raw.block(name), equal_nan=True), name
    selection = select_samples(observed.result, "accepted")
    table = observed.raw.block("accepted_sample_identity")
    states = observed.raw.block("accepted_body_state")
    assert len(selection.identities) == len(table) == len(states)
    assert len(table) == 1 + int(np.nansum(observed.raw.diagnostics[:, 1]))
    assert len(table) > len(observed.raw.times_s)
    assert selection.identities[0].accepted_step_id == 0
    assert any(identity.output_sample_id is None for identity in selection.identities)
    for identity in selection.identities:
        if identity.output_sample_id is not None:
            np.testing.assert_array_equal(states[identity.index],
                observed.raw.states[identity.output_sample_id])
    assert np.max(np.abs(states[:, 1, 15])) > 1
    assert not states.flags.writeable
    if adaptive:
        assert not np.array_equal(states[1], states[2])


def test_request_cannot_silently_replace_accepted_states_with_output_rows():
    plain = _run(adaptive=False, observed=False)
    with pytest.raises(ValueError, match="no per-accepted-step states"):
        select_samples(plain.result, "accepted")


def test_multiple_output_observations_keep_one_original_initial_checkpoint():
    run = _run(adaptive=False, observed=True, times=(0., 5e-13, .01))
    selection = select_samples(run.result, "accepted")
    mapping = run.raw.block("accepted_output_map")
    np.testing.assert_array_equal(mapping[:2, :3], [[0, 0, 0], [0, 1, 0]])
    assert selection.identities[0].time_s == 0.
    assert selection.identities[0].output_sample_id == 0


def test_contact_buffer_replay_keeps_each_original_internal_checkpoint_once():
    model = _drop_model(height=.3073)
    plain = _run(adaptive=False, observed=False, model=model, times=(0., .008, .012))
    run = _run(adaptive=False, observed=True, model=model, times=(0., .008, .012))
    for name, values in plain.raw.blocks.items():
        assert np.array_equal(values, run.raw.block(name), equal_nan=True), name
    selection = select_samples(run.result, "accepted")
    events = run.raw.block("contact_events")
    assert len(events) > 0
    assert any(abs(point.time_s - events[0, 0]) < 1e-10 for point in selection.identities)
    layout = {entry["name"]: entry for entry in run.raw.metadata["accepted_internal_layout"]}
    assert layout["tire_sx"]["count"] == 1
    assert len(selection.identities) == 1 + int(np.nansum(run.raw.diagnostics[:, 1]))


@pytest.fixture
def constrained_checkpoint_run():
    model = _translation_model(driven=())
    times = (0., .01, .02)
    return _run(adaptive=False, observed=True, model=model, times=times,
        wrench={"slider": ((0., 100., 0., 0., 0., 0.),) * len(times)})


def test_cold_checkpoint_roundtrip_never_calls_the_solver(constrained_checkpoint_run, tmp_path, monkeypatch):
    run = constrained_checkpoint_run
    assert np.max(np.abs(run.raw.block("accepted_constraint_multiplier"))) > 1.
    write_artifact(run.result, tmp_path, formats=())
    def forbidden(*args, **kwargs):
        raise AssertionError("cold checkpoint reader called the solver")
    monkeypatch.setattr("suspension_multibody.simulation.runner.run_compiled", forbidden)
    payload = read_artifact(tmp_path)
    cold = envelope_from_artifact(payload)
    assert select_samples(cold, "accepted") == select_samples(run.result, "accepted")
    for name, values in run.raw.blocks.items():
        if name.startswith("accepted_"):
            np.testing.assert_array_equal(payload["arrays"][name], values)


@pytest.mark.parametrize("damage", ["multipliers", "internal_width", "layout", "truncated",
    "missing_body", "output_identity", "identity_columns", "output_map_columns",
    "status_value", "status_gap"])
def test_incomplete_cold_checkpoints_are_rejected(constrained_checkpoint_run, damage):
    raw = constrained_checkpoint_run.raw
    blocks = dict(raw.blocks)
    metadata = dict(raw.metadata)
    metadata["accepted_internal_layout"] = [dict(item)
        for item in metadata["accepted_internal_layout"]]
    if damage == "multipliers":
        blocks.pop("accepted_constraint_multiplier")
    elif damage == "internal_width":
        blocks["accepted_internal_state"] = blocks["accepted_internal_state"][:, :0]
    elif damage == "layout":
        metadata["accepted_internal_layout"][-1]["offset"] += 1
    elif damage == "missing_body":
        blocks["accepted_body_state"] = blocks["accepted_body_state"][:, :1]
    elif damage == "output_identity":
        blocks["accepted_sample_identity"] = blocks["accepted_sample_identity"].copy()
        blocks["accepted_sample_identity"][1, 3] = 999
    elif damage == "identity_columns":
        metadata["accepted_identity_columns"] = ["accepted_step_id", "case_index",
            "time_s", "first_output_sample_id"]
    elif damage == "output_map_columns":
        metadata.pop("accepted_output_map_columns")
    elif damage == "status_value":
        # The writer only ever emits 0.0 or 1.0; `2.0` is finite and would have made
        # "count the 1s" report one successful sample just as plausibly.
        rows = _ledger_rows(raw, 0)
        ledger = blocks["diagnostics"].copy()
        ledger[rows.start, 0] = 2.0
        blocks["diagnostics"] = ledger
    elif damage == "status_gap":
        # A NaN before a written row is not a legal failure tail: the writer fills a
        # case's rows in order and stops at the failure.
        rows = _ledger_rows(raw, 0)
        ledger = blocks["diagnostics"].copy()
        ledger[rows.start, 0] = np.nan
        ledger[rows.start + 1, 0] = 1.0 if rows.stop - rows.start > 1 else 0.0
        ledger[rows.start + 1, 1] = 0.0
        blocks["diagnostics"] = ledger
    else:
        for name in ("accepted_sample_identity", "accepted_body_state",
                "accepted_constraint_multiplier", "accepted_internal_state"):
            blocks[name] = blocks[name][:-1]
    cold = SimpleNamespace(blocks=blocks, metadata=metadata, cases=raw.cases)
    with pytest.raises(ValueError):
        select_samples(cold, "accepted")


def _synthetic_checkpoint(status):
    """
    Build a one-case accepted result whose ledger status column is chosen by the caller.

    Built so the *older* consistency checks all pass: the accepted identities and the
    per-interval step counts agree with each other, and the output map matches the
    number of successful intervals.  Only the writer's own validity rule on the status
    column can tell this ledger apart from a real one.
    """
    fields = ["tire_sx", "tire_sy", "tire_sx_dot", "tire_sy_dot", "tire_u", "tire_v",
        "tire_u_dot", "tire_v_dot", "tire_beta", "tire_beta_dot", "tire_u_ddot",
        "tire_v_ddot", "tire_beta_ddot", "tire_phi", "tire_phi_dot", "tire_maxwell",
        "step_size"]
    count = len(status)
    # One output observation per *successful* interval: the writer takes an output
    # snapshot only for samples it actually accepted, so every later step's identity
    # carries `-1` (no output sample).  An identity that nonetheless claimed an output
    # id would fail the primary-identity cross-check for a reason unrelated to the
    # status column this fixture exists to probe.
    successful = int(np.count_nonzero(np.asarray(status, dtype=float) == 1.0))
    identities = np.array([[0., step, float(step), float(step) if step < successful else -1.]
        for step in range(count)], dtype=float)
    ledger = np.full((count + 2, 16), np.nan)
    ledger[:count, 0] = status
    # One step per interval after the first: exactly what `identities` claims, so the
    # step-count cross-check is satisfied for every status the caller supplies.
    ledger[:count, 1] = [0.0] + [1.0]*(count - 1)
    return SimpleNamespace(
        blocks={
            "accepted_sample_identity": identities,
            "accepted_body_state": np.zeros((count, 1, 19)),
            "accepted_internal_state": np.zeros((count, 1)),
            # One output observation per *successful* interval: that is what the
            # output mapping counts, and what the completeness check compares
            # against.  Tying it to the number of write points would have made the
            # fixture self-inconsistent for reasons unrelated to the status column.
            "accepted_output_map": np.array([[0., float(i), float(i), float(i)]
                for i in range(successful)], dtype=float),
            "diagnostics": ledger,
        },
        metadata={"accepted_state_version": 1, "bodies": ["b"],
            "accepted_constraint_rows": 0,
            "accepted_internal_layout": [{"name": n, "offset": 0,
                "count": 1 if n == "step_size" else 0} for n in fields],
            "accepted_identity_columns": ["case_index", "accepted_step_id", "time_s",
                "first_output_sample_id"],
            "accepted_output_map_columns": ["case_index", "output_sample_id",
                "accepted_step_id", "time_s"]},
        cases=({"name": "c", "sample_offset": 0, "sample_count": count},),
    )


@pytest.mark.parametrize("status,label", [
    ([2.0, 1.0], "a status the writer never emits"),
    ([np.nan, 1.0], "a gap before a written row"),
    ([-1.0, 1.0], "a negative status"),
])
def test_ledger_status_column_must_be_a_written_prefix(status, label):
    """
    The status column says which samples a run actually produced.

    The writer emits exactly `1.0` (accepted) or `0.0` (the failing sample) and fills
    a case's rows in order, leaving everything after a failure at NaN.  A reader that
    only asks "is some row finite" and "how many are 1" accepts `[2.0, 1.0]` or a NaN
    gap before a written row, and would then report an accepted-step sequence the run
    never took.  Every other consistency check passes on this fixture, so the refusal
    can only come from the validity rule.
    """
    result = _synthetic_checkpoint(status)
    with pytest.raises(ValueError, match="accepted diagnostics are malformed"):
        select_samples(result, "accepted")


def test_ledger_status_column_accepts_the_real_writer_shapes():
    """
    The rule must not refuse what the writer really emits.

    Measured shapes: `[1, 1, 1]` for a clean case and `[1, 1, 0, NaN]` for one that
    fails partway -- the failing sample writes `0.0` and the rest stay NaN.  A rule
    demanding "all finite" or "no NaN" would refuse the second, which is a legal
    partial result the project deliberately preserves.
    """
    assert len(select_samples(_synthetic_checkpoint([1.0, 1.0, 1.0]),
        "accepted").identities) == 3
    assert len(select_samples(_synthetic_checkpoint([1.0, 1.0, 0.0, np.nan]),
        "accepted").identities) == 4


@pytest.mark.parametrize("times", [(0., .0005, .001), (0., .01, .02)])
@pytest.mark.parametrize("format_name", ["csv", "parquet"])
def test_accepted_tables_use_their_own_sampling_grid(times, format_name, tmp_path, monkeypatch):
    run = _run(adaptive=False, observed=True, times=times)
    points = select_samples(run.result, "accepted").identities
    if times[-1] == .001:
        assert len(points) == len(times)
    else:
        assert len(points) > len(times)
    def forbidden(*args, **kwargs):
        raise AssertionError("artifact export called the solver")
    monkeypatch.setattr("suspension_multibody.simulation.runner.run_compiled", forbidden)
    loaded = read_artifact(write_artifact(run.result, tmp_path, formats=(format_name,)))
    rows = loaded["tables"]["accepted_body_state"]
    width = int(np.prod(run.raw.block("accepted_body_state").shape[1:]))
    assert len(rows) == len(points)*width
    for point in points:
        row = rows[point.index*width]
        assert int(row["accepted_step_id"]) == point.accepted_step_id
        assert int(row["case_index"]) == point.case_index
        assert row["sample_role"] == ("initial" if point.accepted_step_id == 0 else "accepted")
        assert float(row["time_s"]) == point.time_s
        assert row["unit"] == "m" and row["frame_id"] == "world"
        assert float(row["value"]) == run.raw.block("accepted_body_state")[point.index, 0, 0]
    mapping = loaded["tables"]["accepted_output_map"]
    assert {int(row["output_sample_id"]) for row in mapping} == set(range(len(times)))
    assert {row["sample_role"] for row in mapping} == {"output_observation"}


def test_failed_run_preserves_a_real_accepted_prefix_and_cold_identity(tmp_path):
    model = _drop_model(height=.3073, maximum_compression=.0001)
    times = (0., .005, .01, .02)
    compiled = _compile(adaptive=False, observed=True, model=model, times=times)
    with pytest.raises(KernelContractError) as plain_failure:
        _run(adaptive=False, observed=False, model=model, times=times)
    with pytest.raises(KernelContractError) as observed_failure:
        run_compiled(compiled)
    plain = plain_failure.value.partial_raw_result
    raw = observed_failure.value.partial_raw_result
    for name, values in plain.blocks.items():
        assert np.array_equal(values, raw.block(name), equal_nan=True), name
    points = select_samples(raw, "accepted").identities
    assert len(points) > 2
    assert points[0].accepted_step_id == 0
    assert points[-1].time_s > .005
    assert points[-1].time_s < .02
    assert int(np.nansum(raw.diagnostics[:, 1])) == len(points)-1
    partial = ResultEnvelope(raw, compiled.request.model)
    payload = read_artifact(write_artifact(partial, tmp_path, formats=("csv",)))
    cold = envelope_from_artifact(payload)
    assert select_samples(cold, "accepted").identities == points
    assert payload["manifest"]["status"] == "failed"

@pytest.mark.parametrize("kind", ["output", "initial", "accepted"])
def test_both_stored_result_shapes_answer_the_same_selection(kind):
    """
    A stored run is reachable as the envelope and as the raw contract result.

    ``select_samples`` is handed both shapes in this suite, so the two have to
    name the same points.  Only the ``accepted`` branch unwrapped the raw result,
    which made the same call succeed or fail depending on the selection asked
    for -- an ``AttributeError`` rather than a refusal, so it read as a crash
    instead of a missing capability.
    """
    run = _run(adaptive=False, observed=True)
    assert select_samples(run.raw, kind).identities == \
        select_samples(run.result, kind).identities


def test_rejected_attempts_never_become_accepted_identities():
    """
    The accepted grid holds the states that were committed, not the attempts.

    An adaptive run really does reject attempts (asserted below so the check is
    not vacuous), and a rejected attempt has no state to persist: the integrator
    threw that trial away.  The identity count is therefore tied to the accepted
    step counter alone, and the ids stay contiguous -- a reader that counted
    attempts would report steps the run never took.
    """
    run = _run(adaptive=True, observed=True)
    accepted = int(np.nansum(run.raw.diagnostics[:, 1]))
    rejected = int(np.nansum(run.raw.diagnostics[:, 2]))
    assert rejected > 0
    points = select_samples(run.result, "accepted").identities
    assert len(points) == 1 + accepted
    assert len(points) < 1 + accepted + rejected
    assert [point.accepted_step_id for point in points] == list(range(len(points)))
    times = [point.time_s for point in points]
    assert all(later > earlier for earlier, later in zip(times, times[1:]))

def test_the_same_id_names_a_different_point_on_each_grid():
    """
    The two grids are separate, so id 1 is not one shared point.

    The accepted grid steps at the integrator's own pace (id 1 is the first
    commit) while the output grid holds the requested observations (id 1 is the
    second requested time).  Returning the accepted grid under an output id -- or
    the reverse -- would answer with another interval's state while every number
    stayed plausible, which is the confusion the separate ids exist to prevent.
    """
    run = _run(adaptive=False, observed=True, times=(0., .01, .02))
    accepted = select_samples(run.result, "accepted", ids=[1]).identities[0]
    output = select_samples(run.result, "output", ids=[1]).identities[0]
    assert accepted.time_s != output.time_s
    assert accepted.accepted_step_id == 1 and accepted.output_sample_id is None
    assert output.output_sample_id == 1 and output.accepted_step_id is None
    assert accepted.sample_kind == "accepted" and output.sample_kind == "output"
    states = run.raw.block("accepted_body_state")
    assert not np.array_equal(states[accepted.index], run.raw.states[output.index])
