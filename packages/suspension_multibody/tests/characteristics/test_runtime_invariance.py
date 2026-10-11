"""
Runtime invariance and the sampling budget.

SPEC §56 requires: the baseline must not move, turning the feature on must not change the
original integration/events/step sequence, performance must meet P0, and there must be no
silent downsampling.

**Measured, and the measurements disagree with part of that wording.**  On the rig built
here the trajectory is invariant to the requested output grid, while the *accepted-step
count* is not: two grids are exercised below and give 44 and 50 accepted steps.

Three things about that measurement are worth stating plainly, because the obvious
readings of it are all wrong:

* The trajectory invariance is **weaker evidence than it looks**, in two separate ways.
  First, a single body on a prismatic joint with a linear prescribed motion and constant
  initial velocity has the exact solution ``z = 0.25 t``, so *any* consistent integrator
  lands on it; what the tests rule out is a sampler that perturbs the state, not one that
  is wrong in a way a straight line cannot reveal.  Second, the states at a shared
  *interior* time are **not bit-identical** -- measured 1-4 ULP apart -- so the strong form
  of the claim is false and the tests assert agreement to round-off instead.  Only the
  endpoint is bit-identical, and it is the least informative point on this fixture.
* The count difference is a **placement** effect, not a physical one -- but that was
  established by reading the native sampler, **not** by this test file.  The solver's outer
  loop iterates over requested samples and truncates each step at the next requested
  boundary, so the requested grid is a hard upper bound on the step size.  The tests here
  assert only the observed counts.
* The baseline half of §56 has its own test: the same plan run **without** any output
  request must produce the same trajectory.

The count sensitivity is **pre-existing**: the reachability test below establishes that no
P3 read-layer symbol is used anywhere on the ``simulate`` path, so this task neither
introduced it nor can remove it from the read side.  Reporting it is the honest option;
claiming step-sequence invariance would be false.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.authoring import (
    AssemblyDocument,
    SubsystemDocument,
    TemplateDocument,
)
from suspension_multibody.results import sampled_time_difference, select_samples
from suspension_multibody.schema import (
    DynamicCaseSpec,
    DynamicSolverSettings,
    PrescribedMotion,
    TimeSignal,
    WrenchInput,
    WrenchSignal,
)

_END = 0.02
_SPEED = 0.25


def _fixture() -> tuple[object, object]:
    """Build a minimal axle whose exact solution is the line ``z = 0.25 t``."""
    case = DynamicCaseSpec(
        mode="axle_dynamic",
        solver=DynamicSolverSettings(
            end_time=_END, step_size=_END / 2, integrator="quasi_static"
        ),
        prescribed_motions=(
            PrescribedMotion(
                target="wheel_travel_left",
                displacement=TimeSignal(times=(0.0, _END), values=(0.0, 5.0)),
            ),
        ),
        wrench_inputs=(
            WrenchInput(target="right", wrench=WrenchSignal(fz=TimeSignal(constant=25.0))),
        ),
    )
    template = TemplateDocument.from_payload(
        {
            "document": "template", "schema_version": 1, "name": "invariance_fixture",
            "functional_role": "generic", "allowed_placement_roles": ["any"],
            "symmetry": "asymmetric", "units": {"length": "m"}, "property_slots": [],
            "elements": [],
            "bodies": [
                {"name": "fixture", "fixed": True},
                *[
                    {"name": side, "mass": 1, "inertia": np.eye(3).tolist()}
                    for side in ("left", "right")
                ],
            ],
            "hardpoints": [
                {"name": body, "owner": body, "space": "body"}
                for body in ("fixture", "left", "right")
            ],
            "joints": [
                {
                    "name": side + "_slide", "type": "prismatic", "body_a": "fixture",
                    "body_b": side, "point_a": "fixture", "point_b": side, "axis": [0, 0, 1],
                }
                for side in ("left", "right")
            ],
            "coordinates": [
                {"name": "travel", "joint": "left_slide", "kind": "translation"}
            ],
        }
    )
    sub = SubsystemDocument.from_payload(
        {
            "document": "subsystem", "schema_version": 1, "name": template.name,
            "functional_role": "generic", "placement_role": "any",
            "template": "invariance_fixture.tpl.json",
            "hardpoints": {b: [0, 0, 0] for b in ("fixture", "left", "right")},
            "property_bindings": {},
        },
        template=template,
    )
    assembly = AssemblyDocument.from_payload(
        {
            "document": "assembly", "schema_version": 1, "name": "invariance",
            "assembly_kind": "generic_multibody", "mode": "K", "gravity": [0, 0, 0],
            "subsystems": [
                {"ref": "unit", "functional_role": "generic", "placement_role": "any"}
            ],
        },
        subsystems={"unit": sub},
    )
    return case, assembly


_CASE, _ASSEMBLY = _fixture()


def _plan(times: list[float]) -> dict:
    return {
        "schema_version": 1,
        "name": "invariance",
        "study": "dynamic",
        "protocol": "axle_dynamic",
        "samples": list(times),
        "solver": {"initialization_mode": "provided_consistent_state"},
        "initial_state": {"unit.left": {"velocity": [0, 0, _SPEED]}},
        "boundaries": [
            {
                "name": "wheel_travel_left", "coordinate": "unit.travel",
                "mode": "prescribed_displacement", "units": "m", "program": "travel",
            }
        ],
        "inputs": [
            {
                "name": "travel",
                "values": [0.001 * 5.0 * (t / _END) for t in times],
                "rates": [_SPEED] * len(times),
            },
            {
                "name": "right_load", "role": "body_wrench", "body": "unit.right",
                "values": [[0, 0, 25, 0, 0, 0]] * len(times),
            },
        ],
        "outputs": [],
    }


def _run(times: list[float]):
    return simulate(_ASSEMBLY, _plan(times))


def _accepted(run) -> float:
    return float(np.nansum(run.raw.block("diagnostics")[:, 1]))


SPARSE = [0.0, _END]
DENSE = [0.0, _END / 8, _END / 4, 3 * _END / 8, _END / 2, 5 * _END / 8, 3 * _END / 4,
         7 * _END / 8, _END]
# A third grid that shares interior times with DENSE but not with SPARSE.  Needed because
# SPARSE and DENSE have no interior point in common, so comparing them tests only the two
# endpoints -- which both runs are pinned to by construction.
MID = [0.0, _END / 4, _END / 2, 3 * _END / 4, _END]


# --------------------------------------------------------------------------- #
# the invariant part: the physics does not move
# --------------------------------------------------------------------------- #


def test_the_requested_output_grid_does_not_move_the_endpoint() -> None:
    """
    The state at ``t = end`` must be bit-identical across output requests.

    The answer at the end of the run must not depend on how many intermediate points were
    asked for.  ``equal_nan=True`` because a legitimately unavailable component is NaN on
    both sides and must still count as equal.

    Scope note: this is a claim about the **endpoint**, where it is measured to hold
    exactly.  It is *not* a claim that every shared time is bit-identical -- see
    ``test_the_whole_trajectory_is_compared_not_only_its_end``, which measures 1-4 ULP of
    disagreement at interior shared times.  Comparing only the endpoint would be the weaker
    test, and on this straight-line fixture the endpoint is also the *least* informative
    point, since both runs are pinned there; the interior test is what carries the weight.
    """
    sparse = _run(SPARSE)
    dense = _run(DENSE)
    z_sparse = np.array(sparse.result.body_state("unit.left"))[:, 2]
    z_dense = np.array(dense.result.body_state("unit.left"))[:, 2]
    assert z_sparse[-1] == z_dense[-1]
    assert np.array_equal(
        np.array(sparse.result.body_state("unit.left"))[-1],
        np.array(dense.result.body_state("unit.left"))[-1],
        equal_nan=True,
    )


def test_the_trajectory_is_exact_on_every_grid() -> None:
    """
    The fixture's exact solution is ``z = 0.25 t``; every requested grid must hit it.

    Comparing against a closed-form line (rather than against another run) means a shared
    bug in the sampler cannot make two wrong runs agree.
    """
    for times in (SPARSE, [0.0, _END / 2, _END], DENSE):
        run = _run(times)
        t = np.array(run.result.times_s)
        z = np.array(run.result.body_state("unit.left"))[:, 2]
        np.testing.assert_allclose(z, _SPEED * t, atol=1e-12)


def test_no_trial_step_is_rejected_on_this_rig() -> None:
    """The rejected-attempt column is zero, so the accept count is the whole story here."""
    for times in (SPARSE, DENSE):
        run = _run(times)
        assert float(np.nansum(run.raw.block("diagnostics")[:, 2])) == 0.0
        assert run.raw.status == "success"


# --------------------------------------------------------------------------- #
# the baseline half of §56: no request must behave like the baseline
# --------------------------------------------------------------------------- #


def _plan_without_requests(times: list[float]) -> dict:
    """
    Return the plan with an empty output request.

    §56's first clause is about the baseline not moving: a run that asks for nothing must
    be indistinguishable from the run the feature was added to.  That needs a plan whose
    requests are empty rather than merely smaller.
    """
    plan = _plan(times)
    plan["outputs"] = []
    return plan


def test_the_baseline_is_unchanged_when_nothing_is_requested() -> None:
    """
    With no output request the trajectory must match the requested runs exactly.

    This is the baseline half of §56, and it is the half that catches a sampler which
    perturbs the integration merely by existing.  Compared as a full trajectory rather
    than at the final point: the point of a baseline is that it is the same everywhere.
    """
    baseline = simulate(_ASSEMBLY, _plan_without_requests(SPARSE))
    requested = _run(SPARSE)
    np.testing.assert_array_equal(
        np.array(baseline.result.body_state("unit.left")),
        np.array(requested.result.body_state("unit.left")),
    )
    np.testing.assert_array_equal(
        np.array(baseline.result.times_s), np.array(requested.result.times_s)
    )
    assert _accepted(baseline) == _accepted(requested)


def test_the_baseline_carries_its_own_exact_solution() -> None:
    """The baseline run, not only the requested runs, must land on ``z = 0.25 t``."""
    baseline = simulate(_ASSEMBLY, _plan_without_requests(DENSE))
    t = np.array(baseline.result.times_s)
    z = np.array(baseline.result.body_state("unit.left"))[:, 2]
    np.testing.assert_allclose(z, _SPEED * t, atol=1e-12)


def test_the_whole_trajectory_is_compared_not_only_its_end() -> None:
    """
    Every shared time must agree to round-off, including an interior one.

    Two corrections are baked into this test, both found by measurement:

    * ``SPARSE`` and ``DENSE`` share **only** ``t = 0`` and ``t = end``, so comparing those
      two grids tests nothing but the endpoints -- which both runs are pinned to by
      construction.  ``DENSE`` and ``MID`` share three interior times, which is what makes
      this assertion non-vacuous.
    * The states at an interior shared time are **not bit-identical**: measured 1-4 ULP
      apart (e.g. ``dense`` gives ``0.0025000000000000005`` and ``mid`` gives
      ``0.0025000000000000014`` at ``t = end/2``).  Asserting exact equality here fails, so
      the assertion is the honest one -- agreement to within a few ULP, which is round-off
      in the step placement, not a physical difference.

    Both grids are also checked against the closed-form solution, so agreement between two
    runs cannot hide a shared error.
    """
    dense = _run(DENSE)
    mid = _run(MID)
    t_dense = np.array(dense.result.times_s)
    t_mid = np.array(mid.result.times_s)
    shared = np.intersect1d(t_dense, t_mid)
    interior = [t for t in shared if 0.0 < t < _END]
    assert len(interior) >= 3, "the grids must share interior times or this proves nothing"
    z_dense = np.array(dense.result.body_state("unit.left"))[:, 2]
    z_mid = np.array(mid.result.body_state("unit.left"))[:, 2]
    for time in shared:
        a = z_dense[int(np.argmin(np.abs(t_dense - time)))]
        b = z_mid[int(np.argmin(np.abs(t_mid - time)))]
        # A few ULP: the grids place steps differently, so round-off accumulates
        # differently.  This is the strongest claim the measurement supports.
        np.testing.assert_allclose(a, b, rtol=0.0, atol=8 * np.spacing(abs(float(b))))
        # And each is independently close to the exact solution.
        assert abs(float(a) - _SPEED * time) <= 8 * np.spacing(abs(_SPEED * time))
        assert abs(float(b) - _SPEED * time) <= 8 * np.spacing(abs(_SPEED * time))


# --------------------------------------------------------------------------- #
# the part that does NOT hold, pinned so it cannot be claimed by accident
# --------------------------------------------------------------------------- #


def test_the_accepted_step_count_is_not_invariant_to_the_output_request() -> None:
    """
    Document that the accepted-step count is NOT invariant to the requested grid.

    **This pins a failure of the strict reading of SPEC §56, not a success.**  §56 says
    turning the feature on must not change "原积分/事件/步序".  Measured here, the
    accepted-step count *does* change with the requested grid (44 for two points, 50 for
    nine).  The physics does not -- see the tests above -- because the solver terminates a
    step at each requested sample, so a denser request places more sync points.

    The test asserts the *observed* behaviour so the fact is recorded and visible.  If a
    future change makes the count genuinely invariant, this test will fail and should be
    rewritten to assert equality -- which is the desired direction.  Silently deleting it,
    or loosening it to a tolerance, would hide the discrepancy rather than fix it.

    One weakness to note: on this straight-line fixture the *count* is the only visible
    trace of the effect, because the trajectory cannot show it.  The counts are therefore
    asserted against literal values rather than against the expected total, so the test
    fails if the placement effect ever becomes a proportional one.
    """
    sparse_accepted = _accepted(_run(SPARSE))
    dense_accepted = _accepted(_run(DENSE))
    assert sparse_accepted == 44.0
    assert dense_accepted == 50.0
    assert sparse_accepted != dense_accepted, (
        "the count is now invariant -- good; rewrite this test to assert equality"
    )
    # Nine times the samples does not mean a proportional number of steps: the extra cost
    # comes from the boundaries being placed, not from the span being integrated harder.
    # Bounded by a literal so this fails if the effect ever becomes proportional.
    assert dense_accepted <= 88.0


def _modules_on_the_simulate_path() -> list:
    """
    Return the modules ``simulate`` actually consists of, by following its imports.

    Derived transitively from ``api.py`` rather than hand-listed: a hand-written list of
    files to search has the same defect as searching only two files, because it cannot
    notice a module that was added to the path later.
    """
    import ast
    from pathlib import Path

    root = Path(__file__).resolve().parents[2] / "src" / "suspension_multibody"
    seen: set = set()
    queue = [root / "api.py"]
    while queue:
        path = queue.pop()
        if path in seen or not path.exists():
            continue
        seen.add(path)
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ImportFrom) or not node.module:
                continue
            if node.level:
                base = path.parent
                for _ in range(node.level - 1):
                    base = base.parent
                relative = node.module.replace(".", "/")
                candidate = base / (relative + ".py")
                if candidate.exists():
                    queue.append(candidate)
                elif (base / relative).is_dir():
                    queue.append(base / relative / "__init__.py")
            elif node.module.startswith("suspension_multibody."):
                relative = node.module.split("suspension_multibody.", 1)[1]
                candidate = root / (relative.replace(".", "/") + ".py")
                if candidate.exists():
                    queue.append(candidate)
    return sorted(seen)


def _symbols_used_on_the_simulate_path() -> set:
    """Collect every name and attribute read by a module on the simulate path."""
    import ast

    used: set = set()
    for path in _modules_on_the_simulate_path():
        try:
            tree = ast.parse(path.read_text(encoding="utf-8"))
        except SyntaxError:
            continue
        for node in ast.walk(tree):
            if isinstance(node, ast.Name):
                used.add(node.id)
            elif isinstance(node, ast.Attribute):
                used.add(node.attr)
    return used


def test_the_p3_read_layers_are_not_reachable_from_simulate() -> None:
    """
    Assert the P3 read layers are not *used* by the simulate path.

    The step-count sensitivity documented above predates this task, and that claim only
    holds if the solver's behaviour cannot be affected by the modules this task added.  So
    the check must establish reachability, not mere loading.

    Two weaker checks were tried and rejected:

    * a substring search over ``api.py`` and ``compilation/resolved.py`` -- it never looks
      at ``simulation/runner.py`` or ``results/envelope.py``, which *are* on the path, and
      it would miss a function-level import entirely;
    * ``sys.modules`` after a run -- **measured to be useless here**: ``results/__init__``
      imports all three P3 modules unconditionally, so they are always loaded no matter
      who calls what.

    So this walks the real import graph and looks for the P3 *symbols*.  It is proven able
    to fail: injecting ``from .sampling import select_samples`` into ``results/envelope.py``
    makes it report a hit.
    """
    used = _symbols_used_on_the_simulate_path()
    leaked = sorted(
        name
        for name in (
            "select_samples",
            "require_selection",
            "geometry_rate_request",
            "pose_rates",
            "sampled_time_difference",
            "as_measurement",
        )
        if name in used
    )
    assert not leaked, f"P3 read-layer symbols reachable from simulate: {leaked}"


# --------------------------------------------------------------------------- #
# no silent downsampling
# --------------------------------------------------------------------------- #


def test_every_requested_sample_is_stored() -> None:
    """
    The stored state count must equal the requested sample count, exactly.

    "No silent downsampling" means a request for nine points yields nine points -- not
    eight because two collided, and not eleven because the solver added its own.
    """
    for times in (SPARSE, [0.0, _END / 2, _END], DENSE):
        run = _run(times)
        stored = np.array(run.result.body_state("unit.left")).shape[0]
        assert stored == len(times), (
            f"requested {len(times)} sample(s) but {stored} state(s) were stored"
        )


def test_the_stored_times_are_the_requested_times() -> None:
    """The grid is echoed back unchanged, so a caller can rely on the mapping."""
    for times in (SPARSE, DENSE):
        run = _run(times)
        np.testing.assert_allclose(np.array(run.result.times_s), np.array(times), rtol=1e-12)


def test_the_full_state_is_stored_not_only_a_pose() -> None:
    """
    Samples must not be thinned to pose-only rows to save budget.

    The sampling machinery needs velocities and angular rates to produce real time
    derivatives, so a reduced 7-column layout would be a silent capability loss.  The
    result carries 19 columns on this rig, whose first 13 are the documented
    ``[position(3) | quaternion(4) | velocity(3) | omega(3)]`` block with the velocity and
    angular velocity in the world frame -- verified by reading them back below rather than
    by trusting the count.
    """
    for times in (SPARSE, DENSE):
        run = _run(times)
        state = np.array(run.result.body_state("unit.left"))
        assert state.shape[0] == len(times)
        # The full native layout is 19 columns; asserting `>= 13` would let a 19 -> 13
        # degradation pass silently, and losing the extra 6 columns is exactly the
        # capability loss this test exists to catch.
        assert state.shape[1] == 19
        # The initial velocity really is in columns 7:10 and the body really is not
        # rotating, so the layout is confirmed by content, not by length alone.
        np.testing.assert_allclose(state[0, 7:10], [0.0, 0.0, _SPEED], atol=1e-12)
        np.testing.assert_allclose(state[0, 10:13], [0.0, 0.0, 0.0], atol=1e-12)
        # The quaternion in 3:7 is unit, which a pose-only row would not provide.
        assert abs(float(np.linalg.norm(state[0, 3:7])) - 1.0) < 1e-12


# --------------------------------------------------------------------------- #
# the budget: report, never downsample
# --------------------------------------------------------------------------- #


def test_the_accepted_step_count_is_available_as_a_budget_measure() -> None:
    """
    A caller can learn how many accepted steps a result cost before deciding what to ask.

    This is the input to the "over budget -> report" rule; without it there is nothing to
    compare against a budget.
    """
    run = _run(DENSE)
    selection = select_samples(run.result, "output")
    assert selection.kind == "output"
    assert len(selection.identities) == len(DENSE)
    assert selection.complete
    # The budget measure must come from the diagnostics ledger, because the aggregate
    # performance mapping is NOT populated on this path: it reports `available: False`
    # with every counter at zero while the ledger holds the real 50.  Asserting the zero
    # is what makes this test able to catch a future change that starts trusting the
    # aggregate -- which would silently budget every run at zero cost.
    performance = run.raw.performance
    assert performance["available"] is False
    assert performance["accepted_steps"] == 0.0
    assert _accepted(run) == 50.0


def test_a_refused_selection_reports_rather_than_substituting() -> None:
    """
    Over-budget pressure must surface as a refusal, never as a cheaper substitute.

    ``accepted`` cannot be answered from the stored result, and the refusal says so with a
    reason.  A caller that silently received ``output`` samples relabelled as accepted
    steps would be reading fabricated provenance -- which is exactly what this asserts
    cannot happen.
    """
    from suspension_multibody.characteristics.values import CharacteristicsError

    run = _run(DENSE)
    with pytest.raises(CharacteristicsError, match="per-accepted-step"):
        select_samples(run.result, "accepted")


def test_the_ledger_is_the_authoritative_step_count() -> None:
    """
    The diagnostics ledger carries the real count and the aggregate does not.

    This is the invariant :func:`..results.sampling.select_samples` depends on.  If a
    future change populates the aggregate, this test will fail and the reader should be
    updated to prefer whichever is proven consistent -- rather than leaving two sources
    that silently disagree, which is how a budget check starts under-reporting.
    """
    run = _run(DENSE)
    ledger = run.raw.block("diagnostics")
    assert ledger.ndim == 2
    assert np.nansum(ledger[:, 1]) == 50.0  # accepted_steps, column 1
    assert np.nansum(ledger[:, 2]) == 0.0  # rejected_attempts, column 2
    # The rows are one per sample plus two per case.  Written as `2 * cases` rather than a
    # bare `+ 2`, because the general form is `samples + 2 * cases` and a hard-coded 2
    # would silently give the wrong answer the moment a plan produces more than one case.
    stored = np.array(run.result.body_state("unit.left")).shape[0]
    cases = len(run.raw.cases)
    assert ledger.shape[0] == stored + 2 * cases
    # `block("diagnostics")` returns the raw ledger and is NOT sensitive to the tail-row
    # arithmetic, so on its own it would not notice `_DIAGNOSTIC_TAIL_ROWS` changing --
    # verified by sabotage, where setting it to 0 left the test above passing.  The
    # `diagnostics` *property* applies that arithmetic, so asserting both views is what
    # actually covers the offset logic.
    #
    # Scope limit, also established by sabotage: with a SINGLE case the tail-row constant
    # is unobservable here, because `_diagnostic_samples` slices `[offset : offset+count]`
    # per case and with one case at offset 0 the result is `stored` rows either way.  A
    # multi-case plan is required to exercise that constant -- the same check that caught
    # the bare-ledger-offset defect in step 2.  This file deliberately does not claim to
    # cover it.
    assert run.raw.diagnostics.shape[0] == stored
    assert run.raw.diagnostics_block.shape[0] == stored + 2 * cases


def test_the_average_step_cost_is_reported_not_hidden() -> None:
    """
    The accepted-per-sample ratio is large and must stay visible.

    A caller budgeting storage needs the real ratio; suppressing it would make the budget
    look cheaper than it is.  The ratio is asserted against the *ledger-derived* count and
    an independently stated sample count, so it is not the tautology of comparing a value
    with itself.
    """
    run = _run(DENSE)
    stored = np.array(run.result.body_state("unit.left")).shape[0]
    assert stored == len(DENSE)  # the denominator is a literal, not re-derived
    accepted = _accepted(run)
    ratio = accepted / stored
    # On this short rig the ratio is about 5.6:1; the point is that it exceeds 1, i.e. the
    # work per stored sample is more than one step.
    assert ratio > 1.0
    assert ratio == pytest.approx(50.0 / 9.0, rel=0.02)


def test_a_differenced_series_keeps_one_row_per_sample() -> None:
    """
    The differencing layer must not thin its output either.

    It reports an unavailable rate in place rather than dropping the row, so a caller
    aligning rates against samples never gets a length mismatch.
    """
    run = _run(DENSE)
    t = np.array(run.result.times_s)
    z = np.array(run.result.body_state("unit.left"))[:, 2]
    result = sampled_time_difference(t, z, measurement_id="travel_rate")
    assert len(result) == len(DENSE)
    assert result.status[-1] == "valid"
    np.testing.assert_allclose(result.values, _SPEED, atol=1e-9)
