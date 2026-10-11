"""
Sample *selection*: which points a request is answered at, and what each one is.

The stored native result holds one state per **output sample**.  It does not hold
one state per **accepted step**, and the difference is not academic: a measured run
took 406 accepted steps while storing 21 states, because the sampler only writes at
the output grid ``sample_times``.  So the three selections the contract names are
not three views of one array:

* ``initial``  -- the consistent point at index 0, which *is* stored.
* ``output``   -- the sampler's own grid, which is what the arrays contain.
* ``accepted`` -- every accepted step, which is **not** stored and therefore cannot
  be selected from a result.  It is refused, naming the missing capability, rather
  than approximated by the output grid.

Approximating it is the specific failure this module exists to prevent.  Returning
the output samples under the name ``accepted`` would look right -- both are
"moments during the run", both carry a time -- and would silently mislabel ~20
stored points as 406 accepted steps.  Downstream rates would then be computed over
*output intervals* while being reported as per-accepted-step evolution, in exactly
the way the contract forbids.

Refusal is also the durable choice: when the kernel does persist accepted states,
``require_selection`` starts succeeding without any caller having depended on a
fabricated mapping.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import numpy as np

from ..characteristics.values import SAMPLE_KINDS, CharacteristicsError, SampleIdentity

__all__ = [
    "SAMPLE_KINDS",
    "RefusedSample",
    "SampleSelection",
    "require_selection",
    "select_samples",
]


@dataclass(frozen=True)
class RefusedSample:
    """
    One point that was selected but could not be answered, and why.

    A named record rather than a positional tuple: the pair ``(case, sample)`` is two
    integers, so a tuple lets a producer and a consumer disagree about which is which
    and still type-check -- the resulting message would name the wrong sample, which
    is worse than no message because it looks specific.
    """

    case_index: int
    sample_index: int
    reason: str


@dataclass(frozen=True)
class SampleSelection:
    """
    A resolved set of samples: what was selected, and the identity of each point.

    ``kind`` is the selection that was asked for; every identity carries the same
    ``sample_kind`` so a table row remains attributable after the selection object
    is gone.  ``refused`` is empty on success and, on a partial selection, holds the
    reason per unanswerable point -- an empty tuple is *not* used to mean "no data",
    because that is indistinguishable from "no points selected".
    """

    kind: str
    identities: tuple[SampleIdentity, ...]
    refused: tuple[RefusedSample, ...] = ()

    @property
    def complete(self) -> bool:
        """True when every requested point was answerable."""
        return not self.refused


def _case_spans(result: Any) -> tuple[tuple[int, int], ...]:
    """
    Return ``(sample_offset, sample_count)`` per case, or one span if unknown.

    ``sample_offset`` is a *cumulative* offset into the sample axis -- the writer
    advances it by each case's sample count -- so it is **not** the row index of the
    case in the diagnostics ledger.  The ledger interleaves two per-case tail rows,
    which is why readers add ``2 * case_index``; that adjustment lives in
    :func:`_ledger_rows_for_case` rather than here, because the sample-axis offset is
    what most callers need.
    """
    cases = tuple(getattr(getattr(result, "raw", result), "cases", ()) or ())
    if not cases:
        times = np.asarray(result.times_s, dtype=float)
        return ((0, int(times.size)),)
    return tuple(
        (int(entry["sample_offset"]), int(entry["sample_count"])) for entry in cases
    )


#: Rows of per-case bookkeeping the ledger appends after each case's samples.
#: Mirrors ``_DIAGNOSTIC_TAIL_ROWS`` in ``raw.py``; the two are the same physical
#: layout, and a reader that disagrees with the writer reads another case's numbers.
_LEDGER_TAIL_ROWS = 2

#: Column of ``accepted_steps`` in the diagnostics ledger, per the writer's layout.
_ACCEPTED_STEPS_COLUMN = 1


def _ledger_rows_for_case(case_index: int, offset: int, count: int) -> slice:
    """
    Return the diagnostics-ledger rows holding ``case_index``'s samples.

    The ledger is *not* a flat list of samples with the bookkeeping on the end: each
    case's two bookkeeping rows are inserted immediately after that case's samples.
    Case 1's samples therefore begin two rows later than its sample-axis offset says.
    Reading a bare offset silently returns the previous case's bookkeeping -- numbers
    that are finite, plausible and wrong -- so the adjustment is centralised here and
    used by every ledger reader in this module.
    """
    start = offset + _LEDGER_TAIL_ROWS * case_index
    return slice(start, start + count)


def _stored_count(result: Any) -> int:
    """Return how many samples the arrays actually hold."""
    return sum(count for _, count in _case_spans(result))


def _all_identities(result: Any) -> tuple[SampleIdentity, ...]:
    """
    Build identities for the stored samples, reusing the query layer's own table.

    Imported lazily: ``characteristics`` imports this module's siblings, so a
    module-level import would close a cycle.
    """
    from .characteristics import sample_identities

    return tuple(sample_identities(result))


def select_samples(result: Any, kind: str) -> SampleSelection:
    """
    Return the samples of ``kind`` from a stored result.

    ``output`` returns every stored sample and ``initial`` returns only the first
    one of each case -- the consistent point every case starts from.  ``accepted``
    is refused: the stored result has no per-accepted-step states, and the count is
    available only as a per-sample scalar, so the set cannot be enumerated at all
    (not merely decoded).
    """
    if kind not in SAMPLE_KINDS:
        raise CharacteristicsError(
            f"unknown sample selection {kind!r}; the contract defines "
            f"{', '.join(SAMPLE_KINDS)}"
        )

    identities = _all_identities(result)

    if kind == "output":
        # The stored arrays *are* the output grid, so this is the one selection that
        # needs no translation.
        return SampleSelection(
            kind=kind,
            identities=tuple(
                _with_kind(identity, kind) for identity in identities
            ),
        )

    if kind == "initial":
        # One consistent point per case: each case's own first stored sample.
        #
        # This is taken as "the first identity belonging to each case" rather than by
        # looking up `sample_offset`.  Those two are equal today, but `sample_offset`
        # is a cumulative writer-side field while the identity's `index` comes from
        # whichever layout its producer used; matching them required two independent
        # notions of "where this case starts" to agree, and a legal artifact whose
        # offsets are case-relative would have turned that into a confusing
        # "no stored identity matches" refusal instead of the initial point.
        by_case: dict[int, SampleIdentity] = {}
        for identity in identities:
            by_case.setdefault(identity.case_index, identity)
        firsts = tuple(
            _with_kind(identity, kind) for _, identity in sorted(by_case.items())
        )
        if not firsts:
            raise CharacteristicsError(
                "the stored result holds no samples, so no initial consistent point "
                "can be selected"
            )
        return SampleSelection(kind=kind, identities=firsts)

    # `accepted`.  Report the shortfall concretely so the reason is actionable: the
    # caller needs to know how many points are missing, not merely that it failed.
    accepted = _accepted_step_total(result)
    stored = _stored_count(result)
    raise CharacteristicsError(
        "the stored result has no per-accepted-step states, so the accepted-step "
        f"selection cannot be enumerated: {stored} states are stored while the run "
        f"took {accepted} accepted steps. Selection 'accepted' requires the kernel to "
        "persist each accepted state; substituting the output grid would relabel "
        "output samples as accepted steps"
    )


def _with_kind(identity: SampleIdentity, kind: str) -> SampleIdentity:
    """Return a copy of ``identity`` labelled with its selection."""
    return SampleIdentity(
        index=identity.index,
        time_s=identity.time_s,
        case_index=identity.case_index,
        accepted_step_id=identity.accepted_step_id,
        output_sample_id=identity.output_sample_id,
        sample_kind=kind,
    )


def _accepted_step_total(result: Any) -> int:
    """
    Return the total accepted steps the run reports, or 0 when unreported.

    This reads the diagnostics ledger directly rather than a convenience accessor:
    the total is only used to make the refusal message concrete, and a missing
    ledger must not turn "cannot select accepted steps" into a different error.

    Correctness here is entirely about *which rows* are read.  A bare
    ``sample_offset`` walks into each earlier case's bookkeeping rows, which contain
    finite counters (linear solves and the like) that would be added in as if they
    were accepted steps -- a wrong total in a message whose whole purpose is to make
    the shortfall credible.
    """
    raw = getattr(result, "raw", result)
    block = getattr(raw, "blocks", {}).get("diagnostics")
    if block is None:
        return 0
    rows = np.asarray(block, dtype=float)
    if rows.ndim != 2 or rows.shape[1] <= _ACCEPTED_STEPS_COLUMN:
        # An unrecognised ledger shape is reported as "unreported" rather than
        # guessed at: this only feeds a message, and inventing a count would make the
        # refusal look better-evidenced than it is.
        return 0
    total = 0.0
    for case_index, (offset, count) in enumerate(_case_spans(result)):
        if count <= 0:
            continue
        span = _ledger_rows_for_case(case_index, offset, count)
        column = rows[span, _ACCEPTED_STEPS_COLUMN]
        # `nan` marks a sample the writer never filled, so it must not contribute; a
        # plain `sum` would turn an unwritten row into a NaN total.
        total += float(np.nansum(column))
    return int(total)


def require_selection(result: Any, kind: str) -> SampleSelection:
    """
    Return the samples of ``kind``, refusing when the selection is not answerable.

    The difference from :func:`select_samples` is the contract: callers that must
    not proceed on a partial answer call this one, and it fails closed.  A
    selection that succeeds but omits points is reported as a refusal rather than
    returned, because a partial trajectory that looks complete is how a missing
    state becomes a published number.

    No path in this module currently produces a partial selection -- every selection
    either resolves completely or raises -- so the branch below is defensive.  It is
    kept rather than deleted because the alternative, should a partial selection ever
    appear, is that it would be indistinguishable from a complete one at this
    boundary, which is the specific confusion this module exists to remove.
    """
    selection = select_samples(result, kind)
    if not selection.complete:
        missing = ", ".join(
            f"case {item.case_index} sample {item.sample_index}: {item.reason}"
            for item in selection.refused
        )
        raise CharacteristicsError(f"the {kind!r} selection is incomplete: {missing}")
    return selection
