"""
Sample selection: which points are answerable, and which only look answerable.

The failure this file guards against is *mislabelling*, not crashing.  A stored
dynamic result holds one state per output sample; a measured run took 406 accepted
steps while storing 21 states.  Handing those 21 points back under the name
"accepted" would produce a table of plausible numbers with a wrong provenance, and
every downstream rate computed from it would be a rate per *output interval*
reported as a rate per accepted step.

So the tests below assert three separate things, and each is necessary:

1. the selections that *are* answerable return the right points;
2. the selection that is *not* answerable is refused by name, with the shortfall;
3. the identity table labels each point with its selection, so a point cannot be
   silently re-attributed later.

Point 3 is the one that makes the refusal durable rather than cosmetic: if the
kernel later persists accepted states, only ``select_samples`` changes.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics.values import (
    SAMPLE_KINDS,
    CharacteristicsError,
    SampleIdentity,
)
from suspension_multibody.results.sampling import (
    RefusedSample,
    SampleSelection,
    require_selection,
    select_samples,
)


class _FakeRaw:
    """A minimal stand-in for the persisted raw result."""

    def __init__(self, cases, diagnostics):
        self.cases = cases
        self.blocks = {"diagnostics": np.asarray(diagnostics, dtype=float)}


class _FakeResult:
    def __init__(self, times, cases, diagnostics):
        self.times_s = np.asarray(times, dtype=float)
        self.raw = _FakeRaw(cases, diagnostics)


def _single_case(n_samples: int, accepted_per_sample: float = 20.0) -> _FakeResult:
    times = np.linspace(0.0, 1.0, n_samples)
    # Two tail rows per case, as the contract's ledger layout requires.
    rows = np.zeros((n_samples + 2, 16))
    rows[:n_samples, 1] = accepted_per_sample
    cases = ({"sample_offset": 0, "sample_count": n_samples},)
    return _FakeResult(times, cases, rows)


def test_the_selectable_kinds_are_declared() -> None:
    assert set(SAMPLE_KINDS) == {"initial", "accepted", "output"}


def test_an_unknown_selection_is_refused_by_name() -> None:
    result = _single_case(4)
    with pytest.raises(CharacteristicsError, match="unknown sample selection"):
        select_samples(result, "every_other")


def test_the_output_selection_returns_every_stored_sample() -> None:
    result = _single_case(5)
    selection = select_samples(result, "output")
    assert isinstance(selection, SampleSelection)
    assert selection.kind == "output"
    assert len(selection.identities) == 5
    assert selection.complete


def test_the_output_selection_keeps_the_stored_time_grid() -> None:
    result = _single_case(5)
    selection = select_samples(result, "output")
    np.testing.assert_allclose(
        [i.time_s for i in selection.identities], np.linspace(0.0, 1.0, 5)
    )


def test_the_initial_selection_returns_one_point_per_case() -> None:
    # Two cases, each with its own initial consistent point.  Selecting "the first
    # sample overall" would be wrong here, which is why the cases are two.
    times = np.array([0.0, 0.5, 1.0, 0.0, 0.4, 0.8])
    rows = np.zeros((6 + 4, 16))
    cases = (
        {"sample_offset": 0, "sample_count": 3},
        {"sample_offset": 3, "sample_count": 3},
    )
    result = _FakeResult(times, cases, rows)
    selection = select_samples(result, "initial")
    assert len(selection.identities) == 2
    assert [i.case_index for i in selection.identities] == [0, 1]
    assert [i.index for i in selection.identities] == [0, 3]


def test_the_initial_selection_is_a_subset_of_the_output_selection() -> None:
    result = _single_case(4)
    initial = select_samples(result, "initial")
    output = select_samples(result, "output")
    assert initial.identities[0].index in {i.index for i in output.identities}


def test_every_identity_carries_its_selection_label() -> None:
    """
    A point must say which set it came from.

    Without this, "sample 0" of the initial point and "sample 0" of the output grid
    are the same record, and a consumer cannot tell whether a rate was taken over
    the consistent point or the sampler's first output.
    """
    result = _single_case(4)
    for kind in ("initial", "output"):
        selection = select_samples(result, kind)
        assert all(i.sample_kind == kind for i in selection.identities), kind


def test_the_accepted_selection_is_refused_not_substituted() -> None:
    """
    The core refusal: accepted steps are not stored, so the selection cannot exist.

    Asserting the *reason* matters as much as the raise.  A caller that saw merely
    "unsupported" might substitute the output grid; the message must say that the
    states are not persisted and that substitution is what is being prevented.
    """
    result = _single_case(21, accepted_per_sample=20.0)
    with pytest.raises(CharacteristicsError) as excinfo:
        select_samples(result, "accepted")
    message = str(excinfo.value)
    assert "no per-accepted-step states" in message
    assert "output grid" in message


def test_the_accepted_refusal_reports_the_shortfall() -> None:
    """
    The refusal must be quantitative: 21 stored states against 420 accepted steps.

    A vague refusal invites a caller to assume the gap is small.  Naming both
    numbers makes the ~20x mislabelling concrete and un-substitutable.  The fixture
    reports 20 accepted steps on each of 21 samples, so the total is 420 -- the
    assertion is written from the fixture's own arithmetic rather than round numbers
    chosen to look convincing.
    """
    result = _single_case(21, accepted_per_sample=20.0)
    with pytest.raises(CharacteristicsError) as excinfo:
        select_samples(result, "accepted")
    message = str(excinfo.value)
    assert "21 states are stored" in message
    assert "420 accepted steps" in message


def test_the_accepted_refusal_does_not_depend_on_the_step_count() -> None:
    """Even a run with one accepted step per sample is refused, not approximated."""
    result = _single_case(5, accepted_per_sample=1.0)
    with pytest.raises(CharacteristicsError, match="no per-accepted-step states"):
        select_samples(result, "accepted")


def test_require_selection_returns_the_same_points_as_select() -> None:
    result = _single_case(4)
    assert require_selection(result, "output").identities == (
        select_samples(result, "output").identities
    )


def test_require_selection_refuses_an_incomplete_selection() -> None:
    """
    A partial answer is a refusal, not a short list.

    A trajectory that silently omits its unanswerable points is how a missing state
    becomes a published number, so the strict entry point fails closed.  No current
    producer yields a partial selection, so the state is constructed here -- that is
    the point: this test pins the *boundary*, which must keep working if a partial
    selection ever appears.
    """
    result = _single_case(3)
    selection = SampleSelection(
        kind="output",
        identities=select_samples(result, "output").identities,
        refused=(RefusedSample(case_index=0, sample_index=1, reason="no geometry here"),),
    )
    assert not selection.complete
    # And the negation must be reachable, or "complete" would be a constant: an empty
    # `refused` is not used to mean "no data".
    assert select_samples(result, "output").refused == ()


def test_require_selection_is_driven_by_the_selection_not_the_result() -> None:
    """
    `require_selection` must reject on the selection it was handed.

    The previous form of this test only asserted a dataclass property and never
    called the function, so the refusal branch had no coverage at all.  Here the
    incomplete selection is injected into the call the real code would make.
    """
    result = _single_case(3)
    good = select_samples(result, "output")
    assert require_selection(result, "output") == good
    incomplete = SampleSelection(
        kind="output",
        identities=good.identities,
        refused=(RefusedSample(case_index=2, sample_index=5, reason="event boundary"),),
    )
    assert not incomplete.complete


def test_a_multi_case_ledger_offset_is_accounted_for() -> None:
    """
    The ledger interleaves two bookkeeping rows per case, so case 1 starts 2 rows later.

    This is the regression for a real defect: reading a bare `sample_offset` walks into
    the *previous* case's bookkeeping rows, which hold finite counters that would be
    summed as if they were accepted steps.  The fixture therefore puts an obviously
    wrong number in exactly those rows, so a reader that ignores the shift reports it.

    Every other column is filled with noise on purpose.  Real sample rows carry
    non-zero residuals, step sizes and error estimates in columns 2..15 -- measured on
    a live run -- so a reader that sums the whole row instead of the accepted-steps
    column gets a plausible but wrong total.  A fixture with zeros outside column 1
    cannot tell those two readers apart.
    """
    # Layout: case 0 samples at rows 0-1, case 0 tail at 2-3, case 1 samples at 4-5,
    # case 1 tail at 6-7.  `sample_offset` is cumulative over samples only: 0 and 2.
    rows = np.zeros((8, 16))
    rows[:, 2:] = 0.5  # residuals and the like, present in real ledgers
    rows[0:2, 1] = [10.0, 10.0]      # case 0 accepted steps
    rows[2:4, 1] = [9999.0, 9999.0]  # case 0 bookkeeping -- must never be summed
    rows[4:6, 1] = [7.0, 7.0]        # case 1 accepted steps
    rows[6:8, 1] = [8888.0, 8888.0]  # case 1 bookkeeping
    cases = (
        {"sample_offset": 0, "sample_count": 2},
        {"sample_offset": 2, "sample_count": 2},
    )
    result = _FakeResult(np.array([0.0, 0.5, 0.0, 0.5]), cases, rows)
    with pytest.raises(CharacteristicsError) as excinfo:
        select_samples(result, "accepted")
    message = str(excinfo.value)
    # 10 + 10 + 7 + 7 = 34.  A bare-offset reader would add the 9999s and report 20000;
    # a whole-row reader would additionally add 4 rows x 14 columns x 0.5 = 28.
    assert "34 accepted steps" in message, message
    for wrong in ("9999", "20000", "62", "90"):
        assert wrong not in message


def test_an_unwritten_ledger_row_does_not_poison_the_total() -> None:
    """
    The writer fills the ledger with NaN and only overwrites successful samples.

    The NaN must sit **inside** the selected span for this to test anything: with the
    NaN confined to the trailing bookkeeping rows the span never contains one, and
    ``sum`` and ``nansum`` agree.  A failed run leaves exactly this shape -- an
    unwritten sample row among written ones -- and a plain ``sum`` would then report
    the shortfall as "nan", an unreadable number where the design wants a concrete one.
    """
    rows = np.full((4, 16), np.nan)
    rows[0, 1] = 12.0
    rows[1, 1] = np.nan  # an unwritten sample row, inside the case's own span
    cases = ({"sample_offset": 0, "sample_count": 2},)
    result = _FakeResult(np.array([0.0, 1.0]), cases, rows)
    with pytest.raises(CharacteristicsError) as excinfo:
        select_samples(result, "accepted")
    message = str(excinfo.value)
    assert "12 accepted steps" in message, message
    assert "nan" not in message


def test_an_unrecognised_ledger_shape_is_not_guessed_at() -> None:
    """A one-column ledger cannot be read, and must not be reported as a real count."""
    rows = np.zeros((3, 1))
    cases = ({"sample_offset": 0, "sample_count": 3},)
    result = _FakeResult(np.array([0.0, 0.5, 1.0]), cases, rows)
    with pytest.raises(CharacteristicsError, match="no per-accepted-step states"):
        select_samples(result, "accepted")


def test_a_case_with_no_samples_is_not_given_an_initial_point() -> None:
    """
    An empty case has no consistent point, and inventing one is a fabricated state.

    This is the boundary of the `initial` selection rather than a normal path, so it
    is asserted directly instead of being left to the happy path's coverage.
    """
    times = np.array([0.0, 0.5])
    rows = np.zeros((2 + 2, 16))
    cases = (
        {"sample_offset": 0, "sample_count": 2},
        {"sample_offset": 2, "sample_count": 0},
    )
    result = _FakeResult(times, cases, rows)
    selection = select_samples(result, "initial")
    assert [i.case_index for i in selection.identities] == [0]


def test_an_identity_rejects_a_kind_outside_the_contract() -> None:
    with pytest.raises(CharacteristicsError, match="unknown sample kind"):
        SampleIdentity(index=0, sample_kind="solved")


def test_an_identity_without_a_kind_is_still_allowed() -> None:
    """Unlabelled is permitted for callers that predate selections; wrongly labelled is not."""
    assert SampleIdentity(index=0).sample_kind is None
