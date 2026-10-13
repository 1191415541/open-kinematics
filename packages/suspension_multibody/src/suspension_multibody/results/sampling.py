"""
Select original output samples or the separately persisted accepted states.

Accepted selection requires the versioned complete checkpoint channel. Older
results without that channel are refused; output rows never substitute for it.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
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

#: Scale of the kernel's output-time tolerance: the integration loop stops when
#: ``remaining <= scale * max(1, |target|)`` (``kernel_abi.cpp:390``).  An output
#: sample may sit that far past the last accepted checkpoint it was written from, so
#: a reader comparing the two must use the *same* scaled form.  A fixed `1e-12` is a
#: different test that refuses legal long runs (measured: a 1.5e-12 gap at `t=2`).
_TIME_TOLERANCE_SCALE = 1e-12


#: Column names the writer publishes for the two accepted tables, in order.  They are
#: the version-1 schema; a reader that indexes by position must still confirm them.
_ACCEPTED_IDENTITY_COLUMNS = (
    "case_index", "accepted_step_id", "time_s", "first_output_sample_id"
)
_ACCEPTED_OUTPUT_MAP_COLUMNS = (
    "case_index", "output_sample_id", "accepted_step_id", "time_s"
)

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


def _grid_id(identity: SampleIdentity, kind: str) -> int | None:
    """
    Return the id the *requested* grid uses to name this point, or ``None``.

    ``output`` points are named by their own grid's sample id and ``accepted``
    points by their accepted-step id; both are per-case counters.  ``initial`` has
    no id of its own -- it is the first point of the stored grid -- so it returns
    ``None`` and an explicit id selection over it is refused rather than silently
    matching on array position, which is exactly the index/ID confusion the design
    forbids.
    """
    if kind == "output":
        return identity.output_sample_id
    if kind == "accepted":
        return identity.accepted_step_id
    return None


def _by_explicit_ids(
    identities: tuple[SampleIdentity, ...], kind: str,
    ids: Sequence[int], case_index: int | None,
) -> SampleSelection:
    """
    Select the requested grid ids, refusing anything ambiguous.

    Three refusals, each for its own reason:

    * a non-integer or negative id is not an id (the contract says non-negative);
    * a repeated id would silently select one point twice, and a trajectory built
      from it would report a zero rate that is an artifact of the request;
    * an id that matches **more than one case** cannot be resolved from the request
      alone.  The ids are per-case counters, so `0` names a real point in every
      case; picking one would attach the wrong case's state to the answer.  The
      caller resolves it by naming the case, not by the reader guessing.

    The selected points are returned in **grid order**, not request order: a
    reordered set is a different sequence, and a caller that wanted a reordering
    would be asking for a rate across an interval it did not name.  Ids listed out
    of sampling order are therefore refused instead of reordered.
    """
    wanted: list[int] = []
    seen: set[int] = set()
    for value in ids:
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise CharacteristicsError(
                f"sample ids must be non-negative integers; got {value!r}"
            )
        if value in seen:
            raise CharacteristicsError(
                f"sample id {value} is requested twice; a repeated id would select "
                "one point twice and report a rate that is an artifact of the request"
            )
        seen.add(value)
        wanted.append(value)
    if not wanted:
        raise CharacteristicsError("an explicit id selection needs at least one id")

    pool = [point for point in identities
            if case_index is None or point.case_index == case_index]
    chosen: dict[int, list[SampleIdentity]] = {value: [] for value in wanted}
    for point in pool:
        grid_id = _grid_id(point, kind)
        if grid_id in chosen:
            chosen[grid_id].append(point)
    missing = sorted(value for value in wanted if not chosen[value])
    if missing:
        available = sorted({_grid_id(point, kind) for point in pool}
                           - {None})
        scope = "" if case_index is None else f" in case {case_index}"
        raise CharacteristicsError(
            f"the {kind!r} selection has no sample id {missing}{scope}; the "
            f"available ids are {available[:8]}{'...' if len(available) > 8 else ''}"
        )
    ambiguous = sorted(value for value in wanted if len(chosen[value]) > 1)
    if ambiguous:
        cases = sorted({point.case_index for value in ambiguous
                        for point in chosen[value]})
        raise CharacteristicsError(
            f"sample id {ambiguous} exists in more than one case ({cases}); ids are "
            "per-case counters, so the request must name the case instead of the "
            "reader picking one"
        )
    selected = [chosen[value][0] for value in wanted]
    order = [point.index for point in selected]
    if order != sorted(order):
        raise CharacteristicsError(
            f"sample ids {wanted} are not listed in sampling order; the selection is "
            "a sequence, and reordering it would change the interval every rate is "
            "taken over"
        )
    return SampleSelection(
        kind=kind,
        identities=tuple(_with_kind(point, kind) for point in selected),
    )


def select_samples(
    result: Any, kind: str, *, ids: Sequence[int] | None = None,
    case_index: int | None = None,
) -> SampleSelection:
    """
    Return the samples of ``kind`` from a stored result.

    ``output`` returns every stored sample and ``initial`` returns only the first
    one of each case -- the consistent point every case starts from. ``accepted``
    requires the separately persisted complete checkpoint channel and validates
    its layout, multipliers, step counts and output mapping before enumeration.

    ``ids`` is the design's ``indices`` selection: an explicit set of **grid ids**
    (each grid's own per-case counter, not an array position).  It is a keyword
    rather than a fourth ``kind`` because the ids only mean something together with
    the grid they name: the same integer is a real output sample and a real accepted
    step, so a bare ``"indices"`` kind would leave the reader to guess which grid
    was meant.  ``case_index`` resolves ids that exist in more than one case.
    """
    if kind not in SAMPLE_KINDS:
        raise CharacteristicsError(
            f"unknown sample selection {kind!r}; the contract defines "
            f"{', '.join(SAMPLE_KINDS)}"
        )

    if ids is not None:
        if kind == "initial":
            raise CharacteristicsError(
                "the 'initial' selection has no ids of its own; it is defined as each "
                "case's own first stored sample, and matching a request id against an "
                "array position would be the index/ID confusion the contract forbids"
            )
        if kind == "accepted":
            persisted = _accepted_identities(result)
            if persisted is not None:
                return _by_explicit_ids(persisted, kind, ids, case_index)
            # No checkpoint channel: fall through so the refusal below names the
            # missing states instead of reporting an id that "does not exist".
        else:
            return _by_explicit_ids(
                tuple(_all_identities(result)), kind, ids, case_index
            )

    if kind == "accepted":
        persisted = _accepted_identities(result)
        if persisted is not None:
            return SampleSelection(kind=kind, identities=persisted)

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


def _accepted_identities(result: Any) -> tuple[SampleIdentity, ...] | None:
    raw = getattr(result, "raw", result)
    blocks = getattr(raw, "blocks", {})
    identity = blocks.get("accepted_sample_identity")
    if identity is None:
        return None
    metadata = getattr(raw, "metadata", {})
    if metadata.get("accepted_state_version") != 1:
        raise CharacteristicsError("unsupported accepted_state_version")
    # The writer publishes the column order it used, so a reader that hardcodes the
    # positions is trusting a name it never read.  Reordered or dropped columns
    # would then be interpreted as whatever happens to sit in that slot -- finite,
    # plausible and wrong.  The declared names are part of version 1, so they are
    # checked rather than assumed.
    declared_identity = tuple(metadata.get("accepted_identity_columns", ()))
    declared_map = tuple(metadata.get("accepted_output_map_columns", ()))
    if declared_identity != _ACCEPTED_IDENTITY_COLUMNS:
        raise CharacteristicsError("accepted identity columns do not match version 1")
    if declared_map != _ACCEPTED_OUTPUT_MAP_COLUMNS:
        raise CharacteristicsError("accepted output map columns do not match version 1")
    rows = np.asarray(identity, dtype=float)
    states = blocks.get("accepted_body_state")
    internal = blocks.get("accepted_internal_state")
    if (rows.ndim != 2 or rows.shape[1] != 4 or len(rows) == 0
            or not np.isfinite(rows).all()):
        raise CharacteristicsError("accepted sample identity must be a nonempty finite Nx4 table")
    if (states is None or np.ndim(states) != 3 or np.shape(states)[0] != len(rows)
            or np.shape(states)[1] != len(metadata.get("bodies", ()))
            or np.shape(states)[2] != 19 or not np.isfinite(states).all()):
        raise CharacteristicsError("accepted sample has no complete finite body state")
    if (internal is None or np.ndim(internal) != 2 or np.shape(internal)[0] != len(rows)
            or not np.isfinite(internal).all()):
        raise CharacteristicsError("accepted sample has no complete internal state")
    fields = ("tire_sx", "tire_sy", "tire_sx_dot", "tire_sy_dot", "tire_u", "tire_v",
        "tire_u_dot", "tire_v_dot", "tire_beta", "tire_beta_dot", "tire_u_ddot",
        "tire_v_ddot", "tire_beta_ddot", "tire_phi", "tire_phi_dot", "tire_maxwell",
        "step_size")
    layout = metadata.get("accepted_internal_layout", ())
    cursor = 0
    if not isinstance(layout, (list, tuple)) or len(layout) != len(fields):
        raise CharacteristicsError("accepted internal layout is incomplete")
    for name, item in zip(fields, layout, strict=True):
        if not isinstance(item, Mapping):
            raise CharacteristicsError("accepted internal layout is malformed")
        count = item.get("count")
        if (item.get("name") != name or item.get("offset") != cursor
                or type(count) is not int or count < 0 or (name == "step_size" and count != 1)):
            raise CharacteristicsError("accepted internal layout is malformed")
        cursor += count
    if np.shape(internal)[1] != cursor:
        raise CharacteristicsError("accepted internal state width disagrees with layout")
    constraint_rows = metadata.get("accepted_constraint_rows")
    multipliers = blocks.get("accepted_constraint_multiplier")
    if type(constraint_rows) is not int or constraint_rows < 0:
        raise CharacteristicsError("accepted constraint row count is missing")
    if constraint_rows > 0 and (multipliers is None
            or np.shape(multipliers) != (len(rows), constraint_rows)
            or not np.isfinite(multipliers).all()):
        raise CharacteristicsError("accepted sample has no complete finite multipliers")
    if constraint_rows == 0 and multipliers is not None:
        raise CharacteristicsError("unexpected accepted multipliers for a model with no rows")
    identifiers = rows[:, [0, 1, 3]]
    if not np.equal(identifiers, np.floor(identifiers)).all():
        raise CharacteristicsError("accepted sample IDs must be integers")
    case_count = len(_case_spans(result))
    expected: dict[int, int] = {}
    last_time: dict[int, float] = {}
    identities = []
    for index, (case, step, time, output) in enumerate(rows):
        case_id, step_id, output_id = int(case), int(step), int(output)
        if (case_id < 0 or case_id >= case_count or step_id != expected.get(case_id, 0)
                or output_id < -1 or (step_id == 0 and output_id != 0)
                or (case_id in last_time and time <= last_time[case_id])):
            raise CharacteristicsError("accepted identity is not the contiguous original step sequence")
        expected[case_id] = step_id + 1
        last_time[case_id] = float(time)
        identities.append(SampleIdentity(index=index, time_s=float(time), case_index=case_id,
            accepted_step_id=step_id, output_sample_id=None if output_id < 0 else output_id,
            sample_kind="accepted"))
    diagnostics = np.asarray(blocks.get("diagnostics"), dtype=float)
    if diagnostics.ndim != 2 or diagnostics.shape[1] <= _ACCEPTED_STEPS_COLUMN:
        raise CharacteristicsError("accepted sample has no step-count diagnostics")
    for case_id, (offset, count) in enumerate(_case_spans(result)):
        counters = diagnostics[_ledger_rows_for_case(case_id, offset, count),
            _ACCEPTED_STEPS_COLUMN]
        finite = counters[np.isfinite(counters)]
        if (len(counters) != count or (finite < 0).any()
                or not np.equal(finite, np.floor(finite)).all()
                or expected.get(case_id, 0) != 1 + int(finite.sum())):
            raise CharacteristicsError("accepted sample sequence disagrees with step-count diagnostics")
    output_map = np.asarray(blocks.get("accepted_output_map"), dtype=float)
    if (output_map.ndim != 2 or output_map.shape[1] != 4
            or not np.isfinite(output_map).all()
            or not np.equal(output_map[:, :3], np.floor(output_map[:, :3])).all()):
        raise CharacteristicsError("accepted output mapping is malformed")
    lookup = {(item.case_index, item.accepted_step_id): item for item in identities}
    next_output: dict[int, int] = {}
    first_outputs: dict[tuple[int, int], int] = {}
    for case, output, step, time in output_map:
        case_id, output_id, step_id = int(case), int(output), int(step)
        point = lookup.get((case_id, step_id))
        # The kernel's own tolerance, not a fixed one: its integration loop stops
        # when `remaining <= 1e-12 * max(1, |target|)` (`kernel_abi.cpp:390`), so an
        # output sample can legitimately be that far ahead of the last accepted
        # checkpoint it was taken from.  A fixed 1e-12 rejects exactly those legal
        # runs -- measured on a case ending at `t=2`, where the gap is 1.5e-12 and a
        # valid result was refused.
        tolerance = _TIME_TOLERANCE_SCALE * max(1.0, abs(float(time)))
        if (point is None or output_id != next_output.get(case_id, 0)
                or abs(point.time_s - time) > tolerance):
            raise CharacteristicsError(
                "accepted output mapping disagrees with original checkpoints"
            )
        next_output[case_id] = output_id + 1
        first_outputs.setdefault((case_id, step_id), output_id)
    for point in identities:
        if point.output_sample_id != first_outputs.get((point.case_index, point.accepted_step_id)):
            raise CharacteristicsError("accepted primary output identity disagrees with output mapping")
    for case_id, (offset, count) in enumerate(_case_spans(result)):
        status = diagnostics[_ledger_rows_for_case(case_id, offset, count), 0]
        # The writer fills a case's sample rows in order and writes exactly `1.0`
        # (accepted) or `0.0` (the one sample that failed); every row after a failure
        # is never touched and keeps the ledger's NaN.  Measured shapes are therefore
        # `[1,1,1]` for a clean case and `[1,1,0,NaN]` for a failed one.
        #
        # So the finite values must form a *prefix* and may only be those two.  A
        # ledger with `[1,2]` or a NaN gap before a written row passes both the
        # "some finite row exists" and the "count the 1s" tests, and would otherwise
        # have been enumerated as a complete accepted sequence.
        finite = np.isfinite(status)
        written = int(np.count_nonzero(finite))
        values = status[finite]
        if (not finite[:written].all() or finite[written:].any()
                or not (((values == 0.0) | (values == 1.0)).all())):
            raise CharacteristicsError("accepted diagnostics are malformed")
        # A failed interval records diagnostics but produces no output snapshot, so
        # only the samples that succeeded may be claimed by the output mapping.
        successful = int(np.count_nonzero(values == 1.0))
        if written == 0 or next_output.get(case_id, 0) != successful:
            raise CharacteristicsError("accepted output mapping is incomplete")
    return tuple(identities)


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


def require_selection(
    result: Any, kind: str, *, ids: Sequence[int] | None = None,
    case_index: int | None = None,
) -> SampleSelection:
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
    selection = select_samples(result, kind, ids=ids, case_index=case_index)
    if not selection.complete:
        missing = ", ".join(
            f"case {item.case_index} sample {item.sample_index}: {item.reason}"
            for item in selection.refused
        )
        raise CharacteristicsError(f"the {kind!r} selection is incomplete: {missing}")
    return selection
