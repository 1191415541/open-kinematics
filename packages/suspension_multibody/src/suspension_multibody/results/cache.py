"""
The shared factorization cache for one sample's equilibrium tangent.

What this exists for: a characteristics request asks a *set* of questions about one
frozen mechanism state -- wheel rate on both sides, roll rate, load angles, for
several load modes.  Every one of those is a different right-hand side against the
**same** tangent ``A``.  Solving them independently would factor ``A`` once per
question, which is the cost the design forbids (§5.3, §8).

Two caches, deliberately separate, because they have different lifetimes:

* **The numeric factorization** is keyed by everything that can change ``A`` and
  survives exactly as long as ``A`` is untouched.  Reusing it across a *different*
  sample would mean presenting the previous point's tangent as the current one's
  local stiffness -- the single most dangerous thing this module could do, because
  the answer would be smooth, plausible and wrong (§5.3, line 312).
* **The response** is keyed additionally by the load modes and measurement
  projections involved, since those are what distinguish two questions put to the
  same ``A``.  The design states the direction explicitly: *"cannot let a different
  metric ID block factorization sharing"* (§8, line 515).  So the response cache
  sits *above* the factorization cache, never the other way round.

Structural and numeric invalidation are also separate, for the same reason the
batch interface separates them: when the sparsity pattern changes the structural
analysis must be redone, whereas a changed value at the same pattern reuses the
structure and redoes only the numeric step.  Collapsing the two would either redo
work that is still valid or reuse an analysis that is not.
"""

from __future__ import annotations

import hashlib
import json
import time
from collections.abc import Iterable, Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from ..characteristics.values import CharacteristicsError

__all__ = [
    "CacheCounters",
    "CacheDecision",
    "FactorizationKey",
    "SampleTangentCache",
    "assert_no_condition_reuse",
    "matrix_pattern_digest",
]
def _digest(payload: Any) -> str:
    """
    Return a stable short digest of a JSON-able payload.

    Digests rather than the raw payloads: a cache key holding whole model documents
    would make every stored entry as large as the request that produced it, and the
    point of the cache is to make repeated work cheap, not to retain an audit copy.
    The digest is only ever compared for equality, never interpreted.
    """
    text = json.dumps(payload, sort_keys=True, default=str, separators=(",", ":"))
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:32]


def matrix_pattern_digest(matrix: Any) -> str:
    """
    Return a digest of *where* a matrix is non-zero, ignoring the values.

    This is what makes the structural/numeric split real rather than nominal: two
    tangents with the same pattern can share one symbolic analysis, and a tangent
    whose zeros moved cannot, however small the change in the numbers.
    """
    array = np.asarray(matrix, dtype=float)
    if array.ndim != 2:
        raise CharacteristicsError(
            f"a tangent pattern is defined for a matrix, got shape {array.shape}"
        )
    # `!= 0` rather than a magnitude threshold: a structurally absent entry is an
    # exact zero here, and introducing a tolerance would make two genuinely
    # different patterns look alike.
    occupied = np.argwhere(array != 0.0)
    return _digest([[int(row), int(column)] for row, column in occupied])


def _matrix_digest(matrix: Any) -> str:
    """Return a digest of a matrix's actual *values*."""
    array = np.asarray(matrix, dtype=float)
    return _digest(array.reshape(-1).tolist())


@dataclass(frozen=True)
class FactorizationKey:
    """
    Everything that can change ``A`` for one sample, and nothing else.

    Composed from the design's own list (§8, line 515): the model and solve plan,
    the sample identity, the response domain and its version, the boundary profile,
    the frozen protocol, and the active mode set.  A factor that is not in here
    would let two different tangents share one factorization; a factor that is in
    here but does not affect ``A`` would needlessly split the cache.

    ``active_modes`` is a tuple, not a set: the *order* of the active set is part of
    the assembly, so two samples with the same members in a different order are not
    the same tangent.
    """

    model_fingerprint: str
    plan_fingerprint: str
    sample_index: int
    domain: str
    definition_version: int
    boundary_profile: str
    boundary_version: int
    frozen_protocol: str = ""
    frozen_version: int = 1
    active_modes: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        for name in (
            "model_fingerprint",
            "plan_fingerprint",
            "domain",
            "boundary_profile",
        ):
            if not getattr(self, name):
                raise CharacteristicsError(
                    f"a factorization key must name its {name}; an empty value would "
                    "let unrelated tangents collide"
                )
        if self.sample_index < 0:
            raise CharacteristicsError("a sample index cannot be negative")
        if self.definition_version < 1 or self.boundary_version < 1:
            raise CharacteristicsError(
                "definition and boundary versions must be positive"
            )
        if self.frozen_version < 1:
            raise CharacteristicsError("a frozen protocol version must be positive")

    @property
    def structural(self) -> str:
        """
        The part of the key that survives a value change at a fixed pattern.

        Deliberately excludes the matrix values, so a numeric update at an unchanged
        pattern can reuse the symbolic analysis.

        It also excludes the pattern itself.  The pattern is *observed* from the
        matrix (``observe_pattern``), not supplied by the caller: a caller-provided
        pattern is either redundant or wrong, and when it is absent the reuse
        decision silently degrades to "assume the pattern did not change" -- which
        is how a dense-to-sparse change gets reported as a legitimate structural
        reuse while the symbolic analysis it licenses no longer describes the
        matrix.
        """
        return _digest(
            {
                "model": self.model_fingerprint,
                "plan": self.plan_fingerprint,
                "domain": self.domain,
                "version": self.definition_version,
                "boundary": self.boundary_profile,
                "boundary_version": self.boundary_version,
                "frozen_protocol": self.frozen_protocol,
                "frozen_version": self.frozen_version,
                "active_modes": list(self.active_modes),
            }
        )

    def with_pattern(self, pattern_digest: str) -> str:
        """
        Return the structural identity *including* an observed pattern digest.

        Kept a method rather than a field so the two facts stay distinguishable: the
        condition identity comes from the caller, the pattern comes from the matrix.
        """
        return _digest({"structural": self.structural, "pattern": pattern_digest})

    @property
    def numeric(self) -> str:
        """The full key: the structural part plus the sample and its values."""
        return _digest({"structural": self.structural, "sample": self.sample_index})


@dataclass
class CacheCounters:
    """
    The metering the design asks for, in one place.

    Every field counts a *fact about work performed*, and the two directions are
    distinguishable on purpose:

    * ``assembly_count`` / ``factorization_count`` count work that actually ran.
    * ``assembly_reuse`` / ``numeric_reuse`` / ``structural_reuse`` count work that
      did not have to run again.

    Counting a *reuse* as a factorization would make the number rise when less work
    happened, so a budget check reading it would see the cost move in the opposite
    direction to reality.  ``solve_count`` and ``projection_count`` are incremented
    by ``record_solve`` and ``record_projection``, which the response layer calls;
    they are not left to a caller's memory, because a counter nobody is obliged to
    maintain cannot serve as budget evidence.
    """

    assembly_count: int = 0
    factorization_count: int = 0
    solve_count: int = 0
    projection_count: int = 0
    independent_rhs: int = 0
    structural_reuse: int = 0
    numeric_reuse: int = 0
    response_hits: int = 0
    response_misses: int = 0
    structural_invalidations: int = 0
    numeric_invalidations: int = 0
    assembly_time_s: float = 0.0
    factorization_time_s: float = 0.0
    solve_time_s: float = 0.0
    peak_matrix_bytes: int = 0

    def record_factorization(self, *, dimension: int, seconds: float = 0.0) -> None:
        """
        Register one real numeric factorization.

        Called by whoever actually factors, so ``factorization_count`` agrees with
        the response layer's own count instead of being a guess made here.
        """
        self.factorization_count += 1
        self.factorization_time_s += max(0.0, float(seconds))
        self.peak_matrix_bytes = max(
            self.peak_matrix_bytes, int(dimension) * int(dimension) * 8
        )

    def record_solve(self, *, right_hand_sides: int, seconds: float = 0.0) -> None:
        """Register one back-substitution pass over ``right_hand_sides`` columns."""
        if right_hand_sides < 1:
            raise CharacteristicsError("a solve must cover at least one right-hand side")
        self.solve_count += 1
        self.independent_rhs += int(right_hand_sides)
        self.solve_time_s += max(0.0, float(seconds))

    def record_projection(self, *, count: int = 1) -> None:
        """Register measurement projections actually applied."""
        if count < 1:
            raise CharacteristicsError("a projection pass must cover at least one row")
        self.projection_count += int(count)

    def as_dict(self) -> dict[str, float | int]:
        """Return the counters in a stable shape for a report or a budget check."""
        return {
            name: getattr(self, name)
            for name in self.__dataclass_fields__
        }

    def merge(self, other: CacheCounters) -> None:
        """
        Add another counter set into this one, field by field.

        ``peak_matrix_bytes`` is a **maximum**, not a running total: adding two
        peaks would invent a matrix larger than any that was ever assembled, and a
        budget check comparing that against a limit would report a failure that
        never happened.
        """
        for name in self.__dataclass_fields__:
            if name == "peak_matrix_bytes":
                setattr(self, name, max(getattr(self, name), getattr(other, name)))
            else:
                setattr(self, name, getattr(self, name) + getattr(other, name))


@dataclass(frozen=True)
class CacheDecision:
    """
    Why a request was answered the way it was.

    ``source`` is the whole point of this record: a caller that is told "reused"
    without being told *what* was reused cannot tell a legitimate structural reuse
    from a stale numeric one.  The values are the three cases the design
    distinguishes, plus the two kinds of miss that must never be conflated.
    """

    source: str
    reason: str = ""
    structural_reused: bool = False
    numeric_reused: bool = False

    def __post_init__(self) -> None:
        allowed = (
            "response_hit",
            "numeric_reuse",
            "structural_reuse",
            "reassembled",
        )
        if self.source not in allowed:
            raise CharacteristicsError(
                f"unknown cache source {self.source!r}; known: {', '.join(allowed)}"
            )
        # A "hit" that reused nothing would be a lie in the audit trail.
        if self.source == "numeric_reuse" and not self.numeric_reused:
            raise CharacteristicsError(
                "a numeric reuse must report that it reused the factorization"
            )
        if self.source == "structural_reuse" and not self.structural_reused:
            raise CharacteristicsError(
                "a structural reuse must report that it reused the pattern analysis"
            )

    @property
    def reused(self) -> bool:
        """Whether any work was avoided at all."""
        return self.source != "reassembled"


@dataclass(frozen=True)
class _Entry:
    """
    One retained tangent.

    ``matrix`` is stored by the cache and is never handed out: ``tangent`` returns a
    copy, because a caller that mutated the returned array in place would otherwise
    be editing the matrix the next request is answered from -- silently, and with no
    trace in the key.
    """

    structural: str
    numeric: str
    sample_index: int
    matrix_digest: str
    matrix: np.ndarray


def assert_no_condition_reuse(
    first: FactorizationKey, second: FactorizationKey
) -> None:
    """
    Refuse to share a factorization across conditions that differ in ``A``.

    Exposed so tests can assert the refusal rather than only the positive path: the
    dangerous reuse is not "we forgot to cache" but "we cached across a condition
    boundary", and only the negative case proves the key is doing work.
    """
    if first.numeric == second.numeric:
        return
    # Derived from the dataclass rather than a copied list: a field added later
    # would otherwise be compared by nobody, and a key differing only in it would
    # slip through the guard unnoticed.
    differing = [
        name
        for name in first.__dataclass_fields__
        if getattr(first, name) != getattr(second, name)
    ]
    raise CharacteristicsError(
        "refusing to share a factorization across differing conditions: "
        + ", ".join(differing)
    )


class SampleTangentCache:
    """
    Share one sample's tangent factorization across the questions asked of it.

    The cache is a single-entry store per ``(structural, numeric)`` pair rather than
    an unbounded dict: the design allows reuse *within* a sample and forbids reuse
    *across* samples, so retaining old samples would only create the opportunity for
    the forbidden case.  ``max_entries`` exists so the eviction behaviour is
    exercised rather than assumed.
    """

    def __init__(self, *, max_entries: int = 4) -> None:
        if max_entries < 1:
            raise CharacteristicsError("the cache must be able to hold one entry")
        self._max_entries = max_entries
        self._entries: list[_Entry] = []
        self._responses: dict[str, Any] = {}
        self.counters = CacheCounters()

    # -- assembly ---------------------------------------------------------------

    def tangent(
        self,
        key: FactorizationKey,
        matrix: Any,
        *,
        assemble: Any = None,
    ) -> tuple[np.ndarray, CacheDecision]:
        """
        Return the tangent for ``key``, assembling it only when it is not held.

        ``assemble`` is the work that would have to run; passing it as a callable
        (rather than always computing the matrix) is what makes the saved assembly
        real.  When it is ``None`` the caller has already paid for the matrix and
        the cache only decides whether the *factorization* may be shared -- in that
        case the assembly is not counted, because counting it would report work this
        cache did not do.

        The sparsity pattern is always **observed from the matrix**, never taken
        from the key.  A caller-supplied pattern is either redundant or wrong, and
        when it is missing the reuse decision would quietly become "assume the
        pattern did not change" -- which reports a dense-to-sparse change as a
        legitimate structural reuse while licensing a symbolic analysis that no
        longer describes the matrix.
        """
        observed_pattern = matrix_pattern_digest(matrix)
        values = _matrix_digest(matrix)
        structural = key.with_pattern(observed_pattern)

        entry = self._find(key.numeric)
        if entry is not None and entry.matrix_digest == values:
            self.counters.numeric_reuse += 1
            return entry.matrix.copy(), CacheDecision(
                source="numeric_reuse",
                reason="the same sample tangent was already assembled and factored",
                structural_reused=True,
                numeric_reused=True,
            )

        previous = self._find_structural(structural, key.sample_index)
        if previous is not None:
            self.counters.structural_reuse += 1
            self.counters.numeric_invalidations += 1
            decision = CacheDecision(
                source="structural_reuse",
                reason=(
                    "the observed non-zero pattern is unchanged at this sample, so "
                    "the symbolic analysis is reused while the numeric values are "
                    "refreshed"
                ),
                structural_reused=True,
            )
        else:
            self.counters.structural_invalidations += 1
            decision = CacheDecision(source="reassembled", reason="no reusable entry")

        started = time.perf_counter()
        if assemble is not None:
            matrix = assemble()
            self.counters.assembly_count += 1
        self.counters.assembly_time_s += time.perf_counter() - started
        array = np.array(matrix, dtype=float, copy=True)
        self._store(key, array, values, structural)
        self.counters.peak_matrix_bytes = max(
            self.counters.peak_matrix_bytes, int(array.nbytes)
        )
        # Hand back a copy, never the stored array.  Returning the cache's own
        # storage would let a caller mutate it in place, and the next request would
        # then be answered from an edited matrix with no trace in the key -- the
        # mutation would look exactly like a legitimate numeric refresh.
        return array.copy(), decision

    def held_tangent(
        self, key: FactorizationKey, matrix: Any
    ) -> tuple[np.ndarray, CacheDecision]:
        """
        Return the retained tangent for ``key``, refusing when it is not held.

        Named ``held_tangent`` and not ``factorize`` on purpose: this layer never
        factors anything.  The factorization is ``numpy.linalg.solve`` inside the
        response layer, and the C++ layer's ``solve_many`` owns batched back
        substitution; presenting a *matrix* under the name "factorize" would let a
        caller believe they held a factorization handle when they held an array.

        The returned array is a copy: the retained matrix is the one the next
        request is answered from, so a caller mutating what they were handed would
        edit the cache while its value digest still recorded the original -- the
        edit would then be served as a legitimate numeric reuse.
        """
        values = _matrix_digest(matrix)
        entry = self._find(key.numeric)
        if entry is None or entry.matrix_digest != values:
            raise CharacteristicsError(
                "no retained tangent matches this key and these values; assemble it "
                "through `tangent` first so the reuse decision is recorded"
            )
        self.counters.numeric_reuse += 1
        return entry.matrix.copy(), CacheDecision(
            source="numeric_reuse",
            reason="the identical tangent was already assembled and is still held",
            structural_reused=True,
            numeric_reused=True,
        )

    # -- responses --------------------------------------------------------------

    @staticmethod
    def response_key(
        *,
        modes: Sequence[str],
        measurements: Iterable[str] = (),
        strategy: str = "direct",
    ) -> str:
        """
        Build the response key: the load modes and projections of one question.

        Kept separate from the factorization key on purpose.  Two requests that
        differ only in which *metric* they report still share the factorization; if
        this key were folded into the factorization key, asking for a second metric
        would silently re-factor the same matrix (design §8, line 515).
        """
        return _digest(
            {
                "modes": list(modes),
                "measurements": sorted(measurements),
                "strategy": strategy,
            }
        )

    def response(self, key: str) -> Any:
        """Return a cached response, or ``None`` when it was never computed."""
        if key in self._responses:
            self.counters.response_hits += 1
            return self._responses[key]
        self.counters.response_misses += 1
        return None

    def store_response(self, key: str, value: Any) -> None:
        """Retain a response under ``key`` for later identical questions."""
        self._responses[key] = value

    def invalidate(self) -> None:
        """
        Drop everything.

        Used when an event, a boundary change or a layout change makes *both* caches
        meaningless.  Structural and numeric invalidation are separate in the
        forward direction (see ``tangent``), but a caller that knows the state moved
        should not be forced to reason about which half survived.
        """
        self._entries.clear()
        self._responses.clear()

    @property
    def response_count(self) -> int:
        """How many responses are currently retained."""
        return len(self._responses)

    # -- introspection ----------------------------------------------------------

    @property
    def entry_count(self) -> int:
        """How many tangent entries are currently retained."""
        return len(self._entries)

    def patterns(self) -> tuple[str, ...]:
        """Return the retained structural identities, most recent last."""
        return tuple(entry.structural for entry in self._entries)

    # -- internals --------------------------------------------------------------

    def _find_structural(self, structural: str, sample_index: int) -> _Entry | None:
        """
        Find a retained entry whose *observed pattern* matches this sample's.

        The sample index is part of the match, not just the structural digest.  A
        structural hit on a *different* sample would be reported as "we reused the
        previous point's analysis", which is precisely the reuse the design forbids
        (§5.3, line 312): the new working point is a new point, and the honest
        description of assembling it is ``reassembled``, not a reuse.  What the
        structural digest is genuinely for is telling a *value* refresh at an
        unchanged pattern (a real structural hit) from a pattern change (which
        invalidates the symbolic analysis); both of those happen within one sample.
        """
        for entry in reversed(self._entries):
            if entry.structural == structural and entry.sample_index == sample_index:
                return entry
        return None

    def _find(self, numeric: str) -> _Entry | None:
        for entry in self._entries:
            if entry.numeric == numeric:
                return entry
        return None

    def _store(
        self,
        key: FactorizationKey,
        matrix: np.ndarray,
        values: str,
        structural: str,
    ) -> None:
        self._entries = [e for e in self._entries if e.numeric != key.numeric]
        self._entries.append(
            _Entry(
                structural=structural,
                numeric=key.numeric,
                sample_index=key.sample_index,
                matrix_digest=values,
                matrix=matrix,
            )
        )
        while len(self._entries) > self._max_entries:
            self._entries.pop(0)
