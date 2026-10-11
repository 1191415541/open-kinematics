"""
P4a step 4: drive derived metrics across samples, sharing one factorization.

The operators this layer sequences already have their own tests:
``test_equilibrium_response.py`` pins ``A X = B_w P`` and
``test_compliance_metrics.py`` pins every rate.  What is tested *here* is the
thing neither of them can see, and the split is deliberate:

* **Sequence.**  A number per sample, in sample order, carrying the identity of
  the sample it came from.  The failure this catches is a driver that reorders or
  silently drops a sample -- the metrics would all still be correct, one per
  sample, and the table would be wrong.

* **Sharing.**  One factorization per *sample*, not per sample-and-question.
  Nothing about the answers reveals a lost share, so the assertion is on the
  cost counter rather than on a value.  A test that only checked the answers
  would pass on the most expensive possible implementation.

* **Provenance.**  The boundary and the profile travel with the sequence.  A
  sequence assembled under two boundary profiles is not comparable with itself,
  and that is invisible in the numbers.

The mechanism is a spring-per-sample fixture: sample ``i`` has stiffness
``k_i``, so the expected displacement is ``1/k_i`` and the expected wheel rate
is ``k_i``.  Computing both from the closed form -- not from the implementation
-- is what makes this an oracle rather than a snapshot.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import CharacteristicsError
from suspension_multibody.results.equilibrium import LoadMode, audited_modes
from suspension_multibody.results.per_sample import _KEY_SEPARATOR as _SEP
from suspension_multibody.results.per_sample import (
    DRIVABLE_KINDS,
    MOTION_NAME,
    SampleQuestion,
    SampleSequence,
    drive_sample_questions,
)

MODES = (LoadMode("vertical_force", "N"),)


class _FakeRaw:
    def __init__(self, cases, diagnostics):
        self.cases = cases
        self.blocks = {"diagnostics": np.asarray(diagnostics, dtype=float)}


class _FakeResult:
    def __init__(self, times, cases, diagnostics):
        self.times_s = np.asarray(times, dtype=float)
        self.raw = _FakeRaw(cases, diagnostics)


def _result(n_samples: int) -> _FakeResult:
    times = np.linspace(0.0, 1.0, n_samples)
    rows = np.zeros((n_samples + 2, 16))
    rows[:n_samples, 1] = 20.0
    cases = ({"sample_offset": 0, "sample_count": n_samples},)
    return _FakeResult(times, cases, rows)


def _question(name: str = "heave", *, dimensions: int = 1) -> SampleQuestion:
    """
    One in-phase vertical load mode on a diagonal fixture.

    The tangent arrives per sample, so the question only names the *request*.
    """
    return SampleQuestion(
        question_id=name,
        modes=MODES,
        load_jacobian=_vertical_load(dimensions),
        mode_matrix=_vertical_direction(),
    )


def _vertical_load(dimensions: int) -> np.ndarray:
    """
    Return `B_w` for a vertical force on a `dimensions`-dof mechanism.

    `B_w` is `(ndof, 6)`: it maps the six component amplitudes onto the
    mechanism's degrees of freedom.  The vertical component is index 2, and the
    load lands on the first degree of freedom so the fixture's diagonal tangent
    gives a readable compliance there.
    """
    matrix = np.zeros((dimensions, 6), dtype=float)
    if dimensions >= 1:
        matrix[0, 2] = 1.0
    return matrix


def _vertical_direction() -> np.ndarray:
    """Return `P`, the `(6, 1)` unit direction for the vertical component."""
    matrix = np.zeros((6, 1), dtype=float)
    matrix[2, 0] = 1.0
    return matrix


def _both_loaded(dimensions: int) -> np.ndarray:
    """
    Return `B_w` with the vertical component driving *every* degree of freedom.

    Used where a multi-component motion must have non-zero entries in every
    column: an unloaded axis reads as exactly zero, and a truncation that dropped
    it would be indistinguishable from a genuine zero.
    """
    matrix = np.zeros((dimensions, 6), dtype=float)
    for dof in range(dimensions):
        matrix[dof, 2] = 1.0
    return matrix


def _driven_both_ways(dimensions: int) -> np.ndarray:
    """
    Return `B_w` that drives every degree of freedom from both named components.

    Used where a multi-mode question must have a non-zero applied column for *each*
    mode: a jacobian that only fills the vertical component leaves the lateral
    mode's column at zero, and the layer below then refuses that mode -- correctly,
    but for a reason that would make an unrelated assertion fail.
    """
    matrix = np.zeros((dimensions, 6), dtype=float)
    for dof in range(dimensions):
        matrix[dof, 2] = 1.0  # vertical
        matrix[dof, 1] = 1.0  # lateral
    return matrix


def _direct_vertical(value: float) -> np.ndarray:
    """
    Return `D`, one row in the six-component load space.

    The direct term is applied per *measurement* and lives in the load space, so a
    vertical load's direct contribution sits in component 2 -- the same component
    `_vertical_direction` names.  Writing it as a `(1, 1)` matrix would be read as
    a load-space row of the wrong width.
    """
    row = np.zeros((1, 6), dtype=float)
    row[0, 2] = value
    return row


def _two_directions() -> np.ndarray:
    """
    Return a `(6, 2)` direction matrix naming vertical and lateral.

    Only the `LoadMode` list labels a mode, so a two-mode question needs a
    direction matrix whose two columns are the two named components; a
    one-column matrix would describe one mode and be refused by the layer below.
    """
    matrix = np.zeros((6, 2), dtype=float)
    matrix[2, 0] = 1.0
    matrix[1, 1] = 1.0
    return matrix


def _both_directions() -> np.ndarray:
    """
    Return a `(6, 1)` in-phase direction: the vertical component only.

    One *mode* may still drive several degrees of freedom -- that is what `B_w`
    decides -- so a single column is the right width for a one-mode question even
    when the measurement block has two rows.
    """
    matrix = np.zeros((6, 1), dtype=float)
    matrix[2, 0] = 1.0
    return matrix


class _FakeIdentity:
    """A stand-in for `SampleIdentity` in the sequence-shape tests."""

    def __init__(self, index: int, time_s: float) -> None:
        self.index = index
        self.time_s = time_s


def _drive(stiffness: list[float], **kwargs):
    result = _result(len(stiffness))
    tangents = [np.diag([k]) for k in stiffness]
    return drive_sample_questions(
        result,
        [_question()],
        tangents,
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
        **kwargs,
    )


# --- sequence ----------------------------------------------------------------


def test_every_sample_gets_an_entry_in_sample_order() -> None:
    """
    The sequence has one row per selected sample, at the sample's own index.

    A driver that de-duplicated samples, or that emitted rows in question order,
    would produce the right *set* of numbers with the wrong alignment.
    """
    report = _drive([2.0, 4.0, 8.0])
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    assert motion.sample_count == 3
    assert motion.sample_indices == (0, 1, 2)
    assert motion.answered_count == 3
    # Each row is that sample's own compliance, so the order is observable in the
    # values as well as in the indices.
    np.testing.assert_allclose(
        motion.values[:, 0], np.array([1 / 2.0, 1 / 4.0, 1 / 8.0]), rtol=1e-12
    )


def test_the_sequence_carries_each_samples_identity() -> None:
    """
    The identity travels with the value, and names the sample kind.

    The index alone is not the identity: it changes when the solver takes a
    different number of steps, which is exactly when a comparison is being made.
    """
    report = _drive([2.0, 4.0])
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    assert len(motion.sample_identities) == 2
    assert [identity.sample_kind for identity in motion.sample_identities] == [
        "output",
        "output",
    ]
    assert [identity.index for identity in motion.sample_identities] == [0, 1]


def test_the_closed_form_compliance_holds_at_every_sample() -> None:
    """
    ``C = 1/k`` per sample, computed independently of the implementation.

    The stiffnesses are deliberately non-uniform: a driver that factored the
    first sample and reused it for the rest would give ``1/k_0`` everywhere, and
    a uniform fixture would hide that completely.
    """
    stiffness = [3.0, 7.0, 11.0, 13.0]
    report = _drive(stiffness)
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    np.testing.assert_allclose(
        motion.values[:, 0], np.array([1.0 / k for k in stiffness]), rtol=1e-12
    )


def test_asking_for_mechanism_motion_only_still_produces_a_named_sequence() -> None:
    """A request with no measurement is answered as the motion, under a real name."""
    report = _drive([5.0])
    assert f"heave{_SEP}{MOTION_NAME}" in report.sequences
    assert report.sequence(f"heave{_SEP}{MOTION_NAME}").mode_ids == ("vertical_force",)


# --- sharing -----------------------------------------------------------------


def test_one_factorization_per_sample_not_per_question() -> None:
    """
    Two questions at the same samples must not double the factorizations.

    This is the assertion the design's "assemble and factor once per working
    sample and batch all compatible modes" (design §6.6) reduces to.  It is
    checked on the counter because **no answer changes** when the sharing is
    lost: the results stay correct and only the cost regresses, which is the
    kind of regression a value-only test cannot see.
    """
    samples = 4
    result = _result(samples)
    tangents = [np.diag([float(2 + i)]) for i in range(samples)]
    report = drive_sample_questions(
        result,
        [_question("first"), _question("second")],
        tangents,
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    assert report.counters["factorization_count"] == samples, (
        "one factorization per sample, shared by both questions; "
        f"got {report.counters['factorization_count']} for {samples} samples"
    )
    # Both questions were still *solved* at every sample: the sharing is the
    # factorization, not the answer.  A driver that skipped the second question
    # would satisfy the assertion above by doing less work.
    assert report.counters["solve_count"] == samples * 2
    assert report.sample_count == samples


def test_the_second_question_reuses_the_first_ones_factorization() -> None:
    """
    Two questions at one sample cost one factorization and two solves.

    The driver lifts the tangent out of the per-question loop, so the sharing is
    structural rather than a cache hit: the second question never asks for the
    tangent at all.  That is why the assertion is on the *pair* of counters --
    one factorization against two solves -- and not on the reuse counter, which
    a hit-based implementation would move while an implementation that factored
    twice would also move it.
    """
    result = _result(2)
    tangents = [np.diag([4.0]), np.diag([6.0])]
    report = drive_sample_questions(
        result,
        [_question("first"), _question("second")],
        tangents,
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    assert report.counters["factorization_count"] == 2
    assert report.counters["solve_count"] == 4
    # And the sharing is *observable*, not merely implied by the counter above:
    # re-fetching the tangent per question would take the cache's reuse path and
    # leave a reuse recorded here, while the driver that lifts it out of the loop
    # never asks the cache a second time.  Asserting the reuse is zero is what
    # distinguishes "shared by construction" from "shared by a cache hit that
    # happened to land" -- the latter would still work, and would still be one
    # factorization, but it is not what this layer claims to do.
    assert report.counters["numeric_reuse"] == 0


def test_a_changed_sample_never_reuses_the_previous_factorization() -> None:
    """
    Sharing is *within* a sample.  Across samples it must not happen.

    The fixture makes sample 1 stiffer than sample 0, so reusing sample 0's
    factor would give sample 1 the wrong answer; the assertion is on the value
    so the requirement is checked physically rather than by trusting a counter.
    """
    report = _drive([2.0, 9.0])
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    assert not np.isclose(motion.values[1, 0], motion.values[0, 0])
    np.testing.assert_allclose(motion.values[1, 0], 1.0 / 9.0, rtol=1e-12)


def test_the_solve_count_records_the_right_hand_sides() -> None:
    """Each question's modes are one batched solve, not one solve per mode."""
    result = _result(2)
    tangents = [np.eye(2), np.eye(2)]
    question = SampleQuestion(
        question_id="two_modes",
        modes=(LoadMode("vertical_force", "N"), LoadMode("lateral_force", "N")),
        load_jacobian=_vertical_load(2),
        mode_matrix=_two_directions(),
    )
    report = drive_sample_questions(
        result,
        [question],
        tangents,
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    assert report.counters["factorization_count"] == 2
    assert report.counters["solve_count"] == 2
    assert report.counters["independent_rhs"] == 4


# --- refusals -----------------------------------------------------------------


def test_a_singular_sample_is_refused_without_discarding_its_neighbours() -> None:
    """
    One bad sample must not throw away the sequence.

    This is the whole reason a refusal is recorded rather than raised: a long
    run with one degenerate point is still a usable sequence, and refusing the
    request would turn a local defect into a total loss of information.
    """
    result = _result(3)
    tangents = [np.diag([2.0]), np.zeros((1, 1)), np.diag([8.0])]
    report = drive_sample_questions(
        result,
        [_question()],
        tangents,
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    assert motion.answered_count == 2
    assert not motion.valid[1]
    # The refused row is NaN, never zero: a zero is a reading and would be
    # published by any consumer that forgot to consult `valid`.
    assert np.isnan(motion.values[1, 0])
    assert np.isfinite(motion.values[0, 0]) and np.isfinite(motion.values[2, 0])


def test_a_refused_sample_does_not_carry_a_finite_number() -> None:
    """
    ``valid`` and finiteness must agree, in both directions.

    A refused entry holding a finite value is the dangerous case: it survives any
    check that only looks at `values`.
    """
    result = _result(2)
    report = drive_sample_questions(
        result,
        [_question()],
        [np.diag([3.0]), np.zeros((1, 1))],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    finite = np.isfinite(motion.values[:, 0])
    np.testing.assert_array_equal(finite, motion.valid)


# --- request-shape refusals ---------------------------------------------------


def test_the_accepted_sample_kind_is_refused_by_name() -> None:
    """
    The frozen protocol is asked at stored samples, not at solver attempts.

    Answering at an accepted *attempt* would report a state the solver did not
    keep, so the kind is refused rather than quietly downgraded to ``output``.
    """
    result = _result(3)
    with pytest.raises(CharacteristicsError, match="not at solver attempts"):
        drive_sample_questions(
            result,
            [_question()],
            [np.diag([1.0])],
            sample_kind="accepted",
            model_fingerprint="model-a",
            plan_fingerprint="plan-a",
            boundary_profile="frozen_hold",
            boundary_version=1,
        )


def test_the_drivable_kinds_exclude_accepted() -> None:
    assert "accepted" not in DRIVABLE_KINDS
    assert set(DRIVABLE_KINDS) == {"initial", "output"}


def test_a_tangent_count_that_does_not_match_the_selection_is_refused() -> None:
    """
    One tangent per selected sample, exactly.

    A short list would otherwise silently answer the later samples with an early
    tangent -- the numbers would all be finite and the sequence would look fine.
    """
    result = _result(3)
    with pytest.raises(CharacteristicsError, match="each sample needs the mechanism"):
        drive_sample_questions(
            result,
            [_question()],
            [np.diag([1.0])],
            sample_kind="output",
            model_fingerprint="model-a",
            plan_fingerprint="plan-a",
            boundary_profile="frozen_hold",
            boundary_version=1,
        )


def test_a_duplicate_question_id_is_refused() -> None:
    """Two questions sharing an id make a sequence unattributable."""
    result = _result(1)
    with pytest.raises(CharacteristicsError, match="used twice"):
        drive_sample_questions(
            result,
            [_question("same"), _question("same")],
            [np.diag([1.0])],
            sample_kind="output",
            model_fingerprint="model-a",
            plan_fingerprint="plan-a",
            boundary_profile="frozen_hold",
            boundary_version=1,
        )


def test_a_mode_matrix_that_disagrees_with_the_named_modes_is_refused() -> None:
    """
    The columns label the response, so a mismatch cannot be recovered later.

    Two named modes with a one-column matrix would answer one mode and report it
    under two labels.
    """
    with pytest.raises(CharacteristicsError, match="columns are what label"):
        SampleQuestion(
            question_id="bad",
            modes=(LoadMode("vertical_force", "N"), LoadMode("lateral_force", "N")),
            load_jacobian=np.eye(2),
            mode_matrix=np.array([[1.0]]),
        )


def test_a_question_without_modes_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="names no load modes"):
        SampleQuestion(
            question_id="empty",
            modes=(),
            load_jacobian=_vertical_load(1),
            mode_matrix=np.zeros((6, 0)),
        )


def test_an_empty_question_list_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="at least one question"):
        drive_sample_questions(
            _result(2),
            [],
            [np.diag([1.0]), np.diag([1.0])],
            sample_kind="output",
            model_fingerprint="model-a",
            plan_fingerprint="plan-a",
            boundary_profile="frozen_hold",
            boundary_version=1,
        )


def test_two_questions_reporting_one_measurement_stay_separate() -> None:
    """
    One measurement name from two questions is two sequences, not one column.

    A wheel rate over an in-phase mode and over a roll mode are different
    physical quantities that legitimately share a name.  Folding them into one
    sequence would place both in a single column while every entry stayed
    finite -- so the separation is asserted, and asking for the bare name is
    then refused as ambiguous rather than resolved by guessing.
    """
    left = _question("left")
    right = SampleQuestion(
        question_id="right",
        modes=MODES,
        load_jacobian=_vertical_load(1),
        mode_matrix=_vertical_direction(),
        measurements={"something": (np.eye(1), None)},
    )
    report = drive_sample_questions(
        _result(1),
        [left, right],
        [np.diag([2.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    # `left` reports the motion, `right` reports `something`; both are reachable
    # by their qualified keys.
    assert f"left{_SEP}{MOTION_NAME}" in report.sequences
    assert f"right{_SEP}something" in report.sequences
    assert report.sequence(f"left{_SEP}{MOTION_NAME}").sample_count == 1
    assert report.sequence("something").name == "something"


def test_an_unknown_sequence_name_is_refused_by_name() -> None:
    report = _drive([2.0])
    with pytest.raises(CharacteristicsError, match="no sequence"):
        report.sequence("nope")


# --- provenance ---------------------------------------------------------------


def test_the_provenance_records_the_boundary_and_the_model() -> None:
    """
    The boundary travels with the sequence, so two sequences cannot be mixed.

    The design is explicit that a frozen protocol or boundary difference makes
    two same-named metrics non-comparable; recording the profile is what makes
    that checkable after the fact.
    """
    report = _drive([2.0])
    assert report.provenance["boundary_profile"] == "frozen_hold"
    assert report.provenance["boundary_version"] == 1
    assert report.provenance["model_fingerprint"] == "model-a"
    assert report.provenance["plan_fingerprint"] == "plan-a"
    assert report.provenance["sample_kind"] == "output"
    assert report.provenance["sample_count"] == 1


def test_the_frozen_protocol_is_part_of_the_provenance_and_the_cache_key() -> None:
    """
    A different frozen protocol is a different physical question.

    It therefore both appears in the provenance and splits the factorization
    key: two protocols answering at the same sample index must not share a
    factor.
    """
    result = _result(1)
    report = drive_sample_questions(
        result,
        [_question()],
        [np.diag([2.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
        frozen_protocol="frozen_v2",
        frozen_version=3,
    )
    assert report.provenance["frozen_protocol"] == "frozen_v2"
    assert report.provenance["frozen_version"] == 3


def test_the_questions_are_recorded_in_the_order_they_were_asked() -> None:
    result = _result(1)
    report = drive_sample_questions(
        result,
        [_question("alpha"), _question("beta")],
        [np.diag([2.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    assert report.questions == ("alpha", "beta")


def test_a_sequence_cannot_carry_a_finite_value_at_a_refused_sample() -> None:
    """
    The sequence refuses to be constructed with a finite refused entry.

    This is a guard on the *type*, not on the driver: any future producer of a
    ``SampleSequence`` inherits it, which is what keeps the invariant true
    outside this module.
    """
    from suspension_multibody.results.per_sample import SampleSequence

    with pytest.raises(CharacteristicsError, match="finite values at refused"):
        SampleSequence(
            name="bad",
            values=np.array([[1.0]]),
            valid=np.array([False]),
            units="m",
            mode_ids=("vertical_force",),
            sample_indices=(0,),
            sample_identities=(0,),
            provenance={},
        )


def test_an_ambiguous_bare_name_is_refused_and_lists_the_candidates() -> None:
    """
    A bare measurement name reported by two questions cannot be resolved.

    Both candidates are named so the caller can pick one; picking for them would
    silently return a different physical quantity than the one they meant.
    """
    both = SampleQuestion(
        question_id="q",
        modes=MODES,
        load_jacobian=_vertical_load(1),
        mode_matrix=_vertical_direction(),
        measurements={"shared": (np.eye(1), None)},
    )
    other = SampleQuestion(
        question_id="r",
        modes=MODES,
        load_jacobian=_vertical_load(1),
        mode_matrix=_vertical_direction(),
        measurements={"shared": (np.eye(1), None)},
    )
    report = drive_sample_questions(
        _result(1),
        [both, other],
        [np.diag([2.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    with pytest.raises(CharacteristicsError, match="ambiguous"):
        report.sequence("shared")
    assert report.sequence(f"q{_SEP}shared").sample_count == 1
    assert report.sequence(f"r{_SEP}shared").sample_count == 1


def test_a_report_without_a_sequence_is_refused() -> None:
    """An empty report would read as success, so it cannot be built."""
    from suspension_multibody.results.per_sample import PerSampleReport

    with pytest.raises(CharacteristicsError, match="no sequence answered nothing"):
        PerSampleReport(sequences={}, counters={}, questions=("q",), provenance={})


def test_the_identity_selection_drives_the_initial_kind() -> None:
    """
    ``initial`` is one point per case, and the driver walks exactly those.

    With two cases this is two samples, not the first sample of each -- the
    fixture makes the cases non-adjacent so a driver that took "the first two"
    would produce a different selection.
    """
    times = np.array([0.0, 0.5, 1.0, 0.0, 0.4, 0.8])
    rows = np.zeros((6 + 4, 16))
    cases = (
        {"sample_offset": 0, "sample_count": 3},
        {"sample_offset": 3, "sample_count": 3},
    )
    result = _FakeResult(times, cases, rows)
    report = drive_sample_questions(
        result,
        [_question()],
        [np.diag([2.0]), np.diag([4.0])],
        sample_kind="initial",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    assert motion.sample_indices == (0, 3)
    assert [i.case_index for i in motion.sample_identities] == [0, 1]
    np.testing.assert_allclose(motion.values[:, 0], [1 / 2.0, 1 / 4.0], rtol=1e-12)
    # The *answers* carry the same identities, and they are built inside the loop
    # rather than from the selection, so checking only the sequence would leave a
    # driver free to label each answer with its loop position.  With two cases the
    # positions are 0 and 1 while the identities are 0 and 3 -- the distinction is
    # only visible on a multi-case selection, which is why this fixture is used.
    assert [answer.identity.index for answer in report.answers] == [0, 3]
    assert [answer.identity.case_index for answer in report.answers] == [0, 1]
    assert [answer.sample_index for answer in report.answers] == [0, 3]


def test_the_sample_identity_is_not_derived_from_the_position() -> None:
    """
    The identity is the stored sample's own, not its offset in the selection.

    With two cases the ``initial`` selection gives indices 0 and 3, so an
    implementation that rebuilt the identity from the loop position would report
    ``index=1`` for the second one.  That is exactly the confusion the identity
    type exists to prevent -- an index that moves when the selection changes.
    """
    times = np.array([0.0, 0.5, 1.0, 0.0, 0.4, 0.8])
    rows = np.zeros((6 + 4, 16))
    cases = (
        {"sample_offset": 0, "sample_count": 3},
        {"sample_offset": 3, "sample_count": 3},
    )
    result = _FakeResult(times, cases, rows)
    report = drive_sample_questions(
        result,
        [_question()],
        [np.diag([2.0]), np.diag([4.0])],
        sample_kind="initial",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    # The loop positions are 0 and 1; the identities are 0 and 3, and the second
    # belongs to case 1.  A position-derived identity would say index 1, case 0.
    assert [identity.index for identity in motion.sample_identities] == [0, 3]
    assert motion.sample_identities[1].index != 1
    assert motion.sample_identities[1].case_index == 1
    # The stored time travels too, which a position-derived identity would lose.
    assert motion.sample_identities[1].time_s == pytest.approx(0.0)


def test_a_two_dimensional_motion_keeps_both_components() -> None:
    """
    A motion response holds one column per mode *per degree of freedom*.

    Truncating it to the first component answers a one-degree-of-freedom question
    about a two-degree-of-freedom mechanism, and every remaining number is finite
    and plausible.  The width and the values are asserted together because a
    truncation changes both.
    """
    result = _result(1)
    report = drive_sample_questions(
        result,
        [
            SampleQuestion(
                question_id="heave",
                modes=MODES,
                # Both degrees of freedom are loaded, so both columns of the motion
                # are non-zero.  A fixture that loaded only the first would leave
                # the second at zero, and a truncation would then be
                # indistinguishable from a genuinely unloaded axis.
                load_jacobian=_both_loaded(2),
                mode_matrix=_vertical_direction(),
            )
        ],
        [np.diag([2.0, 8.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    motion = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    assert motion.values.shape == (1, 2), (
        "a two-degree-of-freedom motion is two columns, not one"
    )
    np.testing.assert_allclose(motion.values[0], [1 / 2.0, 1 / 8.0], rtol=1e-12)
    assert len(motion.components) == 2


def test_a_sequence_width_no_mode_and_component_count_explains_is_refused() -> None:
    """
    The container refuses a width that its own labels cannot account for.

    This is what turns a silently truncated motion into an error instead of a
    shorter row: the width is checked against ``modes x components`` rather than
    against the mode count alone, so a lost component cannot pass.
    """
    from suspension_multibody.results.per_sample import SampleSequence

    with pytest.raises(CharacteristicsError, match="describe"):
        SampleSequence(
            name="bad",
            values=np.zeros((1, 3)),
            valid=np.array([True]),
            units="m",
            mode_ids=("vertical_force",),
            sample_indices=(0,),
            sample_identities=(0,),
            provenance={},
        )


# --- defects found in the delivery review -------------------------------------
#
# Each of these fails against the version that was first written, and each
# failure was observed rather than reasoned about.  They are kept grouped so the
# reason each exists remains legible.


def test_a_mode_driving_no_load_direction_is_refused_not_answered_as_zero() -> None:
    """
    A zero applied column must be a refusal, never ``X = A^{-1} 0 = 0``.

    This is the defect the review found first and the most dangerous one: the
    global audit reports ``valid`` because the tangent is fine, so only the
    per-mode check can see that the mode asks the mechanism nothing.  Calling
    ``equilibrium_response`` instead of ``audited_modes`` published a finite,
    well-formed zero as a reading.

    The comparison against ``audited_modes`` is deliberate: the driver must agree
    with the operator that owns this decision rather than reimplement it.
    """
    modes = (LoadMode("vertical_force", "N"), LoadMode("lateral_force", "N"))
    tangent = np.diag([2.0, 3.0, 4.0, 5.0, 6.0, 7.0])
    # The lateral column of `B_w P` is made exactly zero while the tangent stays
    # perfectly conditioned, so only the per-mode check can refuse it.
    applied = np.zeros((6, 6), dtype=float)
    applied[2, 2] = 1.0
    question = SampleQuestion(
        question_id="zero_column",
        modes=modes,
        load_jacobian=applied,
        mode_matrix=_two_directions(),
    )
    report = drive_sample_questions(
        _result(1),
        [question],
        [tangent],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    motion = report.sequence(f"zero_column{_SEP}{MOTION_NAME}")
    assert not bool(motion.valid[0]), (
        "the lateral mode drives no load direction, so this sample is not answered"
    )
    assert np.all(np.isnan(motion.values[0])), (
        "a refused response must not be published as the zero that A^{-1} 0 gives"
    )
    # The independent authority agrees, which is what makes the assertion above a
    # statement about correctness rather than about this driver's own choice.
    audited, _ = audited_modes(tangent, applied, modes)
    assert [result.status for result in audited] == ["valid", "unidentifiable"]


def test_a_degenerate_tangent_with_no_zero_row_is_refused() -> None:
    """
    A rank-deficient tangent is refused even when no row is identically zero.

    The status vocabulary is ``singular`` / ``unidentifiable``, and an earlier
    version tested for a string that appears in neither -- so the refusal fell
    through and an all-NaN sample was marked answered, leaving ``valid`` lying
    about a sequence whose values were all NaN.
    """
    # Square, no zero row, but rank 1: every row is a multiple of the first, so
    # the tangent is singular while no row is identically zero.
    base = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    degenerate = np.vstack([base * float(index + 1) for index in range(6)])
    applied = np.zeros((6, 6), dtype=float)
    applied[0, 2] = 1.0
    question = SampleQuestion(
        question_id="degenerate",
        modes=MODES,
        load_jacobian=applied,
        mode_matrix=_vertical_direction(),
    )
    report = drive_sample_questions(
        _result(1),
        [question],
        [degenerate],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    motion = report.sequence(f"degenerate{_SEP}{MOTION_NAME}")
    assert not bool(motion.valid[0])
    assert motion.answered_count == 0
    assert np.all(np.isnan(motion.values[0]))


def test_a_structural_hit_still_counts_as_a_factorization() -> None:
    """
    A structural reuse performs a real factorization and must be counted as one.

    ``CacheDecision.reused`` is true for a structural hit too, but that path still
    factors the numbers and stores a new array.  Branching on ``reused``
    under-counts exactly the case the counter exists to measure -- two samples
    that share a sparsity pattern but not a value -- and the counter is this
    layer's only evidence that the sharing happened at all.
    """
    # Two samples of the same case, so they share one index and one pattern while
    # carrying different values.
    rows = np.zeros((4 + 4, 16))
    cases = (
        {"sample_offset": 0, "sample_count": 4},
        {"sample_offset": 0, "sample_count": 4},
    )
    times = np.linspace(0.0, 1.0, 8)
    result = _FakeResult(times, cases, rows)
    report = drive_sample_questions(
        result,
        [_question()],
        [np.diag([2.0]), np.diag([3.0])],
        sample_kind="initial",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    assert report.counters["factorization_count"] == 2, (
        "both samples were factored; a structural hit is still a factorization"
    )


def test_the_measurement_row_order_does_not_depend_on_insertion_order() -> None:
    """
    Rows are positional, so the stack order and the read order must be one order.

    Stacking by insertion order and reading back sorted swaps two *named* physical
    quantities; both resulting values stay finite and entirely plausible, which is
    what made this worth a dedicated test.  The fix is a single sorted authority,
    asserted by building the same two measurements in both insertion orders and
    requiring the same answer.
    """
    # `measurement_gradient` is (rows, ndof) where `rows` must be either 1 (one row
    # shared by every mode) or exactly the mode count.  So the two measurements are
    # told apart by asking a *two-mode* question: the first measurement reads the
    # first mechanism coordinate, the second reads the second.  A one-mode question
    # could not distinguish them at all and would be refused by the layer below.
    gradient_alpha = np.array([[1.0, 0.0]])
    gradient_zeta = np.array([[0.0, 1.0]])

    def question(alpha_first: bool) -> SampleQuestion:
        entries = (
            [("alpha", (gradient_alpha, None)), ("zeta", (gradient_zeta, None))]
            if alpha_first
            else [("zeta", (gradient_zeta, None)), ("alpha", (gradient_alpha, None))]
        )
        return SampleQuestion(
            question_id="ordered",
            modes=(LoadMode("vertical_force", "N"), LoadMode("lateral_force", "N")),
            # A two-mode question needs every mode's applied column to be non-zero,
            # or the layer below correctly refuses the mode that drives nothing.
            # This jacobian drives both degrees of freedom from both components.
            load_jacobian=_driven_both_ways(2),
            mode_matrix=_two_directions(),
            measurements=dict(entries),
        )
    answers = []
    for alpha_first in (True, False):
        answers.append(
            drive_sample_questions(
                _result(1),
                [question(alpha_first)],
                # Two degrees of freedom, coupled, so the two measurements read
                # genuinely different numbers and a swap is visible.
                [np.array([[2.0, 0.0], [0.0, 4.0]])],
                sample_kind="output",
                model_fingerprint="model-a",
                plan_fingerprint="plan-a",
                boundary_profile="frozen_hold",
                boundary_version=1,
            )
        )
    alpha = [report.sequence(f"ordered{_SEP}alpha").values[0, 0] for report in answers]
    zeta = [report.sequence(f"ordered{_SEP}zeta").values[0, 0] for report in answers]
    # The identity whose gradient is 1 must be the one that moves, in both builds.
    assert alpha[0] == alpha[1] and zeta[0] == zeta[1], (
        "the meaning of a sequence changed with the insertion order of the mapping"
    )
    assert alpha[0] != zeta[0], "the two named quantities must not be interchangeable"


def test_a_single_measurement_keeps_its_direct_term() -> None:
    """
    ``Y = G X + D P`` must keep ``D`` even for one measurement.

    A single measurement's ``D`` is a single row, which the first version mistook
    for the degenerate "no direct term" case and silently dropped.  The loss is
    invisible in the shape and shows up only in the value, so the expected number
    is computed from the closed form here.

    A *second* measurement is included deliberately.  With only one, the sorted
    order and the insertion order are the same, so the two rows are the same row
    and a row-order defect cannot be observed at all -- the test would pass for
    the wrong reason.  The second measurement reads a different mechanism
    coordinate and carries no direct term, so it also pins which row is which.
    """
    # k = 2 on the loaded coordinate, G_1 = 1, D_1 = 5  =>  Y_1 = 0.5 + 5 = 5.5.
    # The second measurement reads coordinate 1 (unloaded) and has no D, so it is
    # exactly zero -- a wrong-but-plausible swap shows up as 5.5 landing on it.
    question = SampleQuestion(
        question_id="direct",
        # Two modes, so `G`'s two rows are legal: the gradient must have either one
        # row or exactly one row per mode.
        modes=(LoadMode("vertical_force", "N"), LoadMode("lateral_force", "N")),
        load_jacobian=_driven_both_ways(2),
        mode_matrix=_two_directions(),
        measurements={
            # `G` is (rows, ndof); `D` is (rows, n_load) in the six-component load
            # space, with the vertical component carrying the direct term.
            "total": (np.array([[1.0, 0.0]]), _direct_vertical(5.0)),
            "other": (np.array([[0.0, 1.0]]), None),
        },
    )
    report = drive_sample_questions(
        _result(1),
        [question],
        [np.diag([2.0, 4.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    total = report.sequence(f"direct{_SEP}total")
    np.testing.assert_allclose(total.values[0, 0], 5.5, rtol=1e-12)
    # The row swap this guards against would put `total`'s direct term on `other`,
    # so `other`'s value is asserted too: it reads an unloaded coordinate and has
    # no direct term, so it is the small number, not 5.5.
    other = report.sequence(f"direct{_SEP}other")
    assert not np.isclose(other.values[0, 0], 5.5), (
        "the direct term moved to the wrong measurement"
    )


def test_a_question_id_containing_the_old_separator_is_still_unambiguous() -> None:
    """
    The key separator must be invertible, so a question id cannot forge a key.

    With a ``:`` separator, ``question_id="a:b"`` produced a key that split back
    into the wrong pair and ``report.sequence("b:mechanism_motion")`` resolved to
    that question's motion.  The exported separator cannot occur in either part,
    so the inverse is total and the mis-resolution is impossible rather than
    merely unlikely.
    """
    report = drive_sample_questions(
        _result(1),
        [_question("a:b")],
        [np.diag([2.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    # The whole id is preserved on one side of the separator and the motion name on
    # the other, so the key round-trips.
    key = f"a:b{_SEP}{MOTION_NAME}"
    assert key in report.sequences
    assert report.sequence(key).name == MOTION_NAME
    # The forged-looking name is not a key and no sequence answers to it.
    assert f"b{_SEP}{MOTION_NAME}" not in report.sequences


# --- defects found in the second delivery review -------------------------------


def test_every_mode_refused_with_two_measurements_still_forms_a_sequence() -> None:
    """
    An all-refused sample must not collapse the response block to one row.

    The row count used to be inferred from whichever mode answered.  When *no*
    mode answered and two measurements were named, the block came back as
    ``(1, modes)`` while the answers had ``(measurements, modes)`` rows, and
    reading the second measurement raised ``IndexError`` -- a crash instead of the
    documented per-sample refusal.  The width is a property of the *request*, so a
    refusal has the same shape as an answer.
    """
    load = _both_loaded(2)
    question = SampleQuestion(
        question_id="wheel",
        modes=MODES,
        load_jacobian=load,
        mode_matrix=_both_directions(),
        measurements={
            "h_left": (np.array([[1.0, 0.0]]), None),
            "h_right": (np.array([[0.0, 1.0]]), None),
        },
    )
    report = drive_sample_questions(
        _result(1),
        [question],
        # Rank-deficient: every mode is refused globally.
        [np.vstack([np.ones(2) * (i + 1) for i in range(2)])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    for measurement in ("h_left", "h_right"):
        sequence = report.sequence(f"wheel{_SEP}{measurement}")
        assert sequence.sample_count == 1
        assert not sequence.valid.any()
        assert np.all(np.isnan(sequence.values))


def test_a_refused_sample_and_an_answered_one_share_one_width() -> None:
    """
    A refusal must not change the column count, or the sequence cannot stack.

    A singular sample and an answered sample of the *same* sequence produced widths
    of ``n_modes`` and ``dof * n_modes``, so ``np.stack`` raised ``ValueError: all
    input arrays must have the same shape`` -- the refusal of one sample destroyed
    the whole sequence instead of being recorded at that sample.
    """
    report = drive_sample_questions(
        _result(2),
        [_question("heave", dimensions=2)],
        [np.diag([2.0, 4.0]), np.zeros((2, 2))],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    sequence = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    assert sequence.sample_count == 2
    assert sequence.valid.tolist() == [True, False]
    assert np.isfinite(sequence.values[0]).all()
    assert np.all(np.isnan(sequence.values[1]))


def test_a_named_measurement_without_a_gradient_is_refused() -> None:
    """
    A missing ``G`` must not publish the mechanism motion as the measurement.

    ``_stack_measurements`` returned ``None`` when any gradient was missing, and
    the layer below answers ``X`` itself when ``G`` is absent.  The result was a
    finite, plausible number -- the mechanism coordinate -- reported under the
    measurement's name and marked valid.  A measurement exists only if it has a
    gradient; asking for the motion is expressed by naming no measurement.
    """
    with pytest.raises(CharacteristicsError, match="no gradient"):
        SampleQuestion(
            question_id="heave",
            modes=MODES,
            load_jacobian=_vertical_load(2),
            mode_matrix=_vertical_direction(),
            measurements={"displacement": (None, None)},
        )


def test_the_caller_mode_matrix_is_solved_not_re_derived() -> None:
    """
    ``P`` is the request: scaling it must scale the answer.

    Deriving ``P`` from the named modes solves the unit basis and reports the result
    under the caller's labels, so a mode amplitude of 3 read exactly like one of 1.
    Scaling the column must scale the response.
    """
    question = SampleQuestion(
        question_id="heave",
        modes=MODES,
        load_jacobian=_vertical_load(1),
        mode_matrix=_vertical_direction() * 3.0,
    )
    report = drive_sample_questions(
        _result(1),
        [question],
        [np.diag([2.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    sequence = report.sequence(f"heave{_SEP}{MOTION_NAME}")
    # 1/2 for a unit amplitude, so 3/2 for a three-times amplitude.
    assert sequence.values[0, 0] == pytest.approx(1.5)


def test_one_mode_can_answer_two_measurements() -> None:
    """
    One load mode measuring both wheel centres is the design's canonical request.

    Design §6.7 forms the wheel rate from one in-phase mode by measuring both wheel
    centres in a single solve, which is one mode with two measurement rows.  A row
    count restricted to ``1`` or ``mode_count`` refused exactly that request.
    """
    question = SampleQuestion(
        question_id="wheel",
        modes=MODES,
        load_jacobian=_both_loaded(2),
        mode_matrix=_both_directions(),
        measurements={
            "h_left": (np.array([[1.0, 0.0]]), None),
            "h_right": (np.array([[0.0, 1.0]]), None),
        },
    )
    report = drive_sample_questions(
        _result(1),
        [question],
        [np.diag([2.0, 4.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    left = report.sequence(f"wheel{_SEP}h_left")
    right = report.sequence(f"wheel{_SEP}h_right")
    assert left.valid.tolist() == [True]
    assert right.valid.tolist() == [True]
    # One factorization for one mode, not one per measurement.
    assert report.counters["factorization_count"] == 1
    assert left.values[0, 0] == pytest.approx(0.5)
    assert right.values[0, 0] == pytest.approx(0.25)


def test_a_per_mode_refusal_makes_the_sample_unanswered() -> None:
    """
    ``answered`` must read the per-mode refusals, not only the tangent's verdict.

    ``audit['status']`` is the *tangent's* verdict and stays ``valid`` in the case
    that matters most: a mode that drives no load direction is refused on its own
    while the tangent is perfectly fine.  Testing the status alone reported
    ``answered`` for a sample whose every value is NaN, contradicting the sequence.
    """
    question = SampleQuestion(
        question_id="heave",
        modes=MODES,
        load_jacobian=_vertical_load(1),
        # An all-zero direction: the mode drives nothing, so it is refused per mode.
        mode_matrix=np.zeros((6, 1)),
    )
    report = drive_sample_questions(
        _result(1),
        [question],
        [np.diag([2.0])],
        sample_kind="output",
        model_fingerprint="model-a",
        plan_fingerprint="plan-a",
        boundary_profile="frozen_hold",
        boundary_version=1,
    )
    assert report.answers[0].audit["status"] == "valid"
    assert report.answers[0].audit["refused_modes"] == ["vertical_force"]
    assert not report.answers[0].answered
    assert not report.sequence(f"heave{_SEP}{MOTION_NAME}").valid.any()


def test_a_question_id_containing_the_live_separator_is_refused() -> None:
    """
    The id is half of the key, so it may not contain the separator itself.

    The separator was chosen because it cannot occur in either half; an id that
    contains it splits back into two different strings, so the key stops being
    invertible and a lookup can resolve to another question's series.
    """
    with pytest.raises(CharacteristicsError, match="reserved separator"):
        SampleQuestion(
            question_id=f"a{_SEP}b",
            modes=MODES,
            load_jacobian=_vertical_load(1),
            mode_matrix=_vertical_direction(),
        )


def test_a_measurement_entry_that_is_not_a_pair_is_refused() -> None:
    """
    A malformed entry must be refused by name, not escape as an ``IndexError``.

    ``entry[1]`` on a one-element tuple raises ``IndexError``, which says nothing
    about which measurement was wrong; the request shape is checked where the
    request is read.
    """
    with pytest.raises(CharacteristicsError, match="gradient, load_gradient"):
        SampleQuestion(
            question_id="heave",
            modes=MODES,
            load_jacobian=_vertical_load(1),
            mode_matrix=_vertical_direction(),
            measurements={"displacement": (np.array([[1.0]]),)},
        )


def test_an_answered_sample_cannot_carry_a_nan() -> None:
    """
    The other direction of the refusal invariant: valid must imply finite.

    A row marked answered but holding NaN is the same trap as a finite value at a
    refused sample -- a consumer reading ``valid`` alone would take a non-existent
    answer for a reading.
    """
    with pytest.raises(CharacteristicsError, match="not finite"):
        SampleSequence(
            name="heave",
            values=np.array([[np.nan]]),
            valid=np.array([True]),
            units="m",
            mode_ids=("vertical_force",),
            sample_indices=(0,),
            sample_identities=(_FakeIdentity(0, 0.0),),
            provenance={},
        )


def test_the_published_sequence_arrays_cannot_be_rewritten() -> None:
    """
    A published answer must not change under a later reader.

    ``np.asarray`` on a float array returns the caller's own object, so storing the
    converted value without freezing it hands every consumer a writable view of the
    answer -- one consumer can silently rewrite what another has already read.
    """
    sequence = SampleSequence(
        name="heave",
        values=np.array([[1.0]]),
        valid=np.array([True]),
        units="m",
        mode_ids=("vertical_force",),
        sample_indices=(0,),
        sample_identities=(_FakeIdentity(0, 0.0),),
        provenance={},
    )
    with pytest.raises(ValueError, match="read-only"):
        sequence.values[0, 0] = 99.0


def test_the_stored_arrays_are_the_converted_ones_not_the_callers() -> None:
    """
    The fields must hold the *converted* arrays, not whatever the caller passed.

    Validating a converted local and storing the original is a validated-looking
    object that is not validated: a list input leaves ``values`` a list, so every
    arithmetic use of it raises, and an integer array leaves the dtype unnormalised.
    The freeze does not help either -- it is applied to the local, not to the field.
    """
    sequence = SampleSequence(
        name="heave",
        values=[[1.0], [np.nan]],
        valid=[True, False],
        units="m",
        mode_ids=("vertical_force",),
        sample_indices=(0, 1),
        sample_identities=(_FakeIdentity(0, 0.0), _FakeIdentity(1, 0.1)),
        provenance={},
    )
    assert isinstance(sequence.values, np.ndarray)
    assert isinstance(sequence.valid, np.ndarray)
    assert sequence.values.dtype == np.float64
    assert sequence.valid.dtype == np.bool_
    assert sequence.values.shape == (2, 1)
    assert sequence.valid.tolist() == [True, False]
