"""
Step 5: one tangent, several questions -- and never one sample's answer for another.

The physical claim under test is narrow and absolute: every metric requested at one
frozen sample is a different right-hand side against the *same* matrix, so the
matrix must be assembled and factored once and the questions must all be answered
from that one factorization.  The dangerous failure is not "the cache did not
help" -- a cold cache is merely slow -- but "the cache helped too much", answering
the current sample from a previous sample's tangent.  That answer is smooth,
plausible and wrong, so most of what follows is the negative direction.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics import CharacteristicsError
from suspension_multibody.results.cache import (
    CacheCounters,
    CacheDecision,
    FactorizationKey,
    SampleTangentCache,
    assert_no_condition_reuse,
    matrix_pattern_digest,
)

TOLERANCE = 1e-12


def key(
    *,
    sample: int = 0,
    domain: str = "equilibrium",
    boundary: str = "suspension_free",
    boundary_version: int = 1,
    modes: tuple[str, ...] = ("heave",),
    model: str = "model-a",
    plan: str = "plan-a",
    frozen: str = "",
    frozen_version: int = 1,
) -> FactorizationKey:
    """
    Build a key with everything spelled out, so a test names what it varies.

    Note there is no ``pattern`` parameter: the sparsity pattern is observed from
    the matrix, not declared by the caller.  A caller-supplied pattern is either
    redundant or wrong, and when absent it silently degrades the reuse decision to
    "assume the pattern did not change".
    """
    return FactorizationKey(
        model_fingerprint=model,
        plan_fingerprint=plan,
        sample_index=sample,
        domain=domain,
        definition_version=1,
        boundary_profile=boundary,
        boundary_version=boundary_version,
        frozen_protocol=frozen,
        frozen_version=frozen_version,
        active_modes=modes,
    )


# ---------------------------------------------------------------------------
# The key: what can change A and what cannot
# ---------------------------------------------------------------------------


def test_the_same_sample_and_boundary_shares_one_key() -> None:
    """Two descriptions of one condition must not produce two cache entries."""
    assert key().numeric == key().numeric
    assert key().structural == key().structural


@pytest.mark.parametrize(
    "changed",
    [
        {"sample": 1},
        {"domain": "dynamic"},
        {"boundary": "body_fixed"},
        {"boundary_version": 2},
        {"modes": ("heave", "roll")},
        {"model": "model-b"},
        {"plan": "plan-b"},
        {"frozen": "frozen_inertial_load_v1"},
        {"frozen_version": 2},
    ],
)
def test_every_condition_that_changes_a_changes_the_key(changed) -> None:
    """
    Each factor the design names must actually separate two tangents.

    A factor that was documented but forgotten in the key would let two different
    matrices share one factorization -- the silent wrong answer.  Parametrised so a
    future edit that drops one factor fails on that factor alone.
    """
    assert key(**changed).numeric != key().numeric


def test_the_structural_key_ignores_the_sample_but_the_numeric_key_does_not() -> None:
    """
    A different sample may reuse the *pattern* but never the *values*.

    This is the split that makes "reuse verified-unchanged structure, re-evaluate
    the new working point" (design §5.3, line 312) implementable rather than
    aspirational.
    """
    first, second = key(sample=0), key(sample=1)
    assert first.structural == second.structural
    assert first.numeric != second.numeric


def test_a_key_without_its_identity_is_refused() -> None:
    """An empty identity field would let unrelated tangents collide silently."""
    for field_name in ("model", "plan", "domain", "boundary"):
        with pytest.raises(CharacteristicsError):
            key(**{field_name: ""})


def test_a_key_cannot_have_negative_versions_or_samples() -> None:
    with pytest.raises(CharacteristicsError):
        key(sample=-1)
    with pytest.raises(CharacteristicsError):
        key(boundary_version=0)


def test_the_pattern_digest_sees_structure_not_values() -> None:
    """
    A rescaled matrix keeps its pattern; a moved zero does not.

    If this digest were value-sensitive, a legitimate numeric refresh would look
    like a structural change and be refused; if it were insensitive to position,
    two genuinely different sparsities would share one analysis.
    """
    base = np.array([[4.0, 1.0, 0.0], [0.5, 3.0, 0.0], [0.0, 0.0, 5.0]])
    rescaled = base * 7.5
    assert matrix_pattern_digest(base) == matrix_pattern_digest(rescaled)

    moved = np.array([[4.0, 0.0, 1.0], [0.5, 3.0, 0.0], [0.0, 0.0, 5.0]])
    assert matrix_pattern_digest(base) != matrix_pattern_digest(moved)

    denser = np.array([[4.0, 1.0, 1e-9], [0.5, 3.0, 0.0], [0.0, 0.0, 5.0]])
    assert matrix_pattern_digest(base) != matrix_pattern_digest(denser), (
        "a structurally new non-zero is not the same pattern, however small"
    )


def test_the_pattern_digest_refuses_a_vector() -> None:
    with pytest.raises(CharacteristicsError):
        matrix_pattern_digest(np.array([1.0, 2.0]))


# ---------------------------------------------------------------------------
# Sharing one factorization across the questions asked of one sample
# ---------------------------------------------------------------------------


def test_the_same_tangent_is_assembled_and_factored_once() -> None:
    """
    The design's central claim: one assembly, one factorization, several modes.

    Counted through the metering rather than asserted, because "we called it twice"
    is exactly the defect that a passing test would otherwise hide.
    """
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    calls = {"n": 0}

    def assemble() -> np.ndarray:
        calls["n"] += 1
        return matrix

    first, decision_first = cache.tangent(key(), matrix, assemble=assemble)
    second, decision_second = cache.tangent(
        key(), np.array([[4.0, 1.0], [0.5, 3.0]]), assemble=assemble
    )

    assert calls["n"] == 1, "the second identical request must not reassemble"
    assert decision_first.source == "reassembled"
    assert decision_second.source == "numeric_reuse"
    assert decision_second.numeric_reused and decision_second.structural_reused
    np.testing.assert_array_equal(first, second)
    assert cache.counters.assembly_count == 1, "one real assembly"
    assert cache.counters.numeric_reuse == 1


def test_a_caller_supplied_matrix_is_not_counted_as_this_cache_assembling_it() -> None:
    """
    ``assemble=None`` means the caller already paid for the matrix.

    Counting it again would report assembly work this cache did not do, and a
    budget check reading the total would see a cost that never occurred.
    """
    cache = SampleTangentCache()
    cache.tangent(key(), np.array([[4.0, 1.0], [0.5, 3.0]]))
    assert cache.counters.assembly_count == 0

    cache.tangent(key(sample=1), np.array([[2.0, 0.0], [0.0, 2.0]]))
    assert cache.counters.assembly_count == 0, (
        "a caller-supplied matrix must never be counted as a cache assembly"
    )


def test_the_returned_tangent_is_a_copy_the_caller_cannot_poison() -> None:
    """
    Handing out the cached array itself would let one request silently change the
    matrix another request is about to be answered from.
    """
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    handed_out, _ = cache.tangent(key(), matrix)
    handed_out[0, 0] = 999.0

    again, _ = cache.tangent(key(), matrix)
    assert again[0, 0] == 4.0, "the cache must not expose its own storage"


def test_a_changed_value_at_the_same_pattern_reuses_the_structure_only() -> None:
    """
    A numeric refresh is not a structural change.

    The decision must say *structural* reuse and must not claim numeric reuse --
    conflating them is how a stale numeric factorization gets presented as current.
    """
    cache = SampleTangentCache()
    first, _ = cache.tangent(key(), [[4.0, 1.0], [0.5, 3.0]])
    second, decision = cache.tangent(key(), [[4.5, 1.0], [0.5, 3.0]])

    assert decision.source == "structural_reuse"
    assert decision.structural_reused is True
    assert decision.numeric_reused is False, (
        "the values changed, so the numeric factorization did not carry over"
    )
    assert second[0, 0] == 4.5
    assert cache.counters.structural_reuse == 1
    assert cache.counters.numeric_invalidations == 1
    assert first[0, 0] == 4.0, "the earlier value must not be edited in place"


def test_a_changed_pattern_invalidates_the_structural_analysis_too() -> None:
    """
    A moved non-zero means the symbolic analysis no longer describes the matrix.

    This is the case that must *not* be served by ``structural_reuse``: the design
    makes event/boundary/layout/pattern changes invalidate the structural cache.
    """
    cache = SampleTangentCache()
    dense = np.array([[4.0, 1.0], [0.5, 3.0]])
    sparse = np.array([[4.0, 0.0], [0.5, 3.0]])
    cache.tangent(key(), dense)
    _, decision = cache.tangent(key(), sparse)

    assert decision.source == "reassembled"
    assert decision.structural_reused is False
    assert cache.counters.structural_invalidations == 2


def test_a_dense_to_sparse_change_is_not_reported_as_a_structural_reuse() -> None:
    """
    The defect this pins: with no caller-supplied pattern, a dropped non-zero was
    silently called an unchanged pattern.

    Because the old key carried an *optional* ``pattern_digest`` defaulting to
    ``""``, the structural digest of a sparse and a dense tangent were identical, so
    the decision said "the non-zero pattern is unchanged, so the symbolic analysis
    is reused" while the pattern had in fact changed.  That is the exact
    invalidation the design requires (§5.3, line 312), reported as its opposite --
    and it licenses reusing a symbolic analysis that no longer fits.

    The pattern is now observed from the matrix, so no caller can omit it.
    """
    cache = SampleTangentCache()
    cache.tangent(key(), [[4.0, 1.0], [0.5, 3.0]])
    _, decision = cache.tangent(key(), [[4.0, 0.0], [0.5, 3.0]])
    assert decision.source != "structural_reuse", (
        "a changed sparsity pattern is not a value refresh"
    )


def test_a_newly_nonzero_entry_is_a_pattern_change() -> None:
    """The reverse direction: a structurally new non-zero is equally a change."""
    cache = SampleTangentCache()
    cache.tangent(key(), [[4.0, 0.0], [0.5, 3.0]])
    _, decision = cache.tangent(key(), [[4.0, 0.0], [0.5, 3.0]])
    assert decision.source == "numeric_reuse"

    _, decision = cache.tangent(key(), [[4.0, 1e-30], [0.5, 3.0]])
    assert decision.source != "numeric_reuse"


def test_the_pattern_is_observed_from_the_matrix_not_declared() -> None:
    """
    The key must not accept a pattern at all, or the omission can go unnoticed.

    Asserted structurally: a version of this key that carried a pattern field would
    let a caller omit it, which is how the defect above became reachable.
    """
    fields = set(FactorizationKey.__dataclass_fields__)
    assert "pattern_digest" not in fields, (
        "the sparsity pattern is observed from the matrix; a declared one can be "
        "omitted and then silently assumed unchanged"
    )
    # The observed pattern still participates in the structural identity.
    same = key().with_pattern(matrix_pattern_digest(np.eye(2)))
    other = key().with_pattern(matrix_pattern_digest(np.array([[1.0, 1.0], [0.0, 1.0]])))
    assert same != other


def test_a_key_whose_pattern_is_observed_cannot_disagree_with_the_matrix() -> None:
    """
    The old failure mode -- a key declaring a pattern the matrix did not have -- is
    now structurally impossible, because the key carries no pattern to disagree.

    Kept as a regression on the *shape* of the fix rather than on an error message:
    the point is that there is nothing left to lie with.
    """
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    _, decision = cache.tangent(key(), matrix)
    assert decision.source == "reassembled"
    # Whatever the matrix's real pattern is, the cache agrees with it by
    # construction: a second identical call is a numeric hit.
    _, decision = cache.tangent(key(), matrix)
    assert decision.source == "numeric_reuse"


# ---------------------------------------------------------------------------
# The forbidden direction: reuse across conditions
# ---------------------------------------------------------------------------


def test_a_different_sample_never_reuses_the_previous_factorization() -> None:
    """
    The failure this whole module exists to prevent, asserted negatively.

    Presenting sample 1's tangent as sample 1's local stiffness when only sample 0
    was ever assembled would produce a perfectly well-formed, entirely wrong local
    response.  So the second sample must reassemble and must report a fresh
    factorization.
    """
    cache = SampleTangentCache()
    first, _ = cache.tangent(key(sample=0), np.array([[4.0, 1.0], [0.5, 3.0]]))
    second, decision = cache.tangent(key(sample=1), np.array([[2.0, 0.0], [0.0, 2.0]]))

    assert decision.source == "reassembled"
    assert decision.numeric_reused is False
    assert not np.array_equal(first, second)
    assert cache.counters.structural_reuse == 0, (
        "a different sample is a new working point, not a reuse of the old one's "
        "symbolic analysis"
    )


@pytest.mark.parametrize(
    "changed",
    [
        {"domain": "dynamic"},
        {"boundary": "body_fixed"},
        {"boundary_version": 2},
        {"modes": ("heave", "roll")},
        {"model": "model-b"},
        {"plan": "plan-b"},
        {"sample": 3},
        {"frozen": "frozen_inertial_load_v1"},
        {"frozen_version": 2},
    ],
)
def test_no_reuse_across_any_condition_boundary(changed) -> None:
    """
    Every boundary the key declares must be enforced, not merely recorded.

    ``frozen_protocol`` is in this list because it was previously absent from both
    the guard and the key: a frozen-protocol sample and a plain one shared a
    factorization, and nothing exercised the field.
    """
    with pytest.raises(CharacteristicsError, match="refusing to share"):
        assert_no_condition_reuse(key(), key(**changed))


def test_the_condition_guard_compares_every_key_field() -> None:
    """
    A field omitted from the guard would let two keys differing only in it pass.

    Enumerated against the dataclass itself rather than a copied list, so a field
    added later cannot silently escape the guard.
    """
    compared = {
        "model_fingerprint",
        "plan_fingerprint",
        "sample_index",
        "domain",
        "definition_version",
        "boundary_profile",
        "boundary_version",
        "frozen_protocol",
        "frozen_version",
        "active_modes",
    }
    assert set(FactorizationKey.__dataclass_fields__) == compared


def test_sharing_is_allowed_for_identical_conditions() -> None:
    """The guard must permit the legitimate case, or it would disable the cache."""
    assert_no_condition_reuse(key(), key())
    assert_no_condition_reuse(key(modes=("heave", "roll")), key(modes=("heave", "roll")))


def test_condition_reuse_reports_which_factor_differed() -> None:
    """
    Naming the differing factor is what makes the refusal actionable: "conditions
    differ" alone leaves the caller guessing which of eight fields to inspect.
    """
    with pytest.raises(CharacteristicsError) as error:
        assert_no_condition_reuse(key(), key(boundary="body_fixed", sample=2))
    message = str(error.value)
    assert "boundary_profile" in message
    assert "sample_index" in message
    assert "domain" not in message, "a factor that did not differ must not be listed"


# ---------------------------------------------------------------------------
# The response cache sits above the factorization cache, never below it
# ---------------------------------------------------------------------------


def test_a_second_metric_does_not_re_factor_the_same_matrix() -> None:
    """
    The direction the design fixes explicitly (§8, line 515).

    Asking a new question about the same ``A`` must reuse the factorization; if the
    metric identity were part of the factorization key, the second metric would
    silently re-factor, and the sharing the module exists to provide would never
    happen while every test still passed.
    """
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    calls = {"n": 0}

    def assemble() -> np.ndarray:
        calls["n"] += 1
        return matrix

    load_key = key(modes=("heavy_force",))
    cache.tangent(load_key, matrix, assemble=assemble)

    wheel = cache.response_key(modes=("heavy_force",), measurements=("wheel_rate",))
    assert wheel is not None
    cache.store_response(wheel, "wheel")
    cache.tangent(load_key, matrix, assemble=assemble)

    assert calls["n"] == 1, "a new metric must not re-assemble the shared tangent"

    roll = cache.response_key(modes=("heavy_force",), measurements=("roll_rate",))
    assert cache.response(roll) is None, "a different metric is a different question"
    assert cache.response(wheel) == "wheel"


def test_the_response_key_distinguishes_modes_measurements_and_strategy() -> None:
    """Two questions that differ in any of these are not the same question."""
    base = SampleTangentCache.response_key(modes=["heave"], measurements=["h_left"])
    assert SampleTangentCache.response_key(
        modes=["heave"], measurements=["h_right"]
    ) != base
    assert SampleTangentCache.response_key(
        modes=["roll"], measurements=["h_left"]
    ) != base
    assert SampleTangentCache.response_key(
        modes=["heave"], measurements=["h_left"], strategy="transpose"
    ) != base


def test_the_response_key_ignores_measurement_order() -> None:
    """
    Measurement order does not change which quantities are asked for, so treating
    it as significant would create a miss for an identical question.
    """
    first = SampleTangentCache.response_key(
        modes=["heave"], measurements=["h_left", "h_right"]
    )
    second = SampleTangentCache.response_key(
        modes=["heave"], measurements=["h_right", "h_left"]
    )
    assert first == second


def test_a_response_hit_is_counted_apart_from_a_miss() -> None:
    """Counters must let a caller tell sharing from recomputation."""
    cache = SampleTangentCache()
    name = SampleTangentCache.response_key(modes=["heave"], measurements=["h"])
    assert cache.response(name) is None
    cache.store_response(name, 1.0)
    assert cache.response(name) == 1.0
    assert cache.counters.response_misses == 1
    assert cache.counters.response_hits == 1


def test_invalidate_clears_both_layers() -> None:
    """
    A caller that knows the state moved must not have to reason about which half
    survived; leaving the response cache behind would answer a new state from an
    old question.
    """
    cache = SampleTangentCache()
    name = SampleTangentCache.response_key(modes=["heave"], measurements=["h"])
    cache.tangent(key(), np.array([[4.0, 1.0], [0.5, 3.0]]))
    cache.store_response(name, 1.0)

    cache.invalidate()

    assert cache.entry_count == 0
    assert cache.response_count == 0, "the response cache must be cleared too"
    assert cache.response(name) is None
    _, decision = cache.tangent(key(), np.array([[4.0, 1.0], [0.5, 3.0]]))
    assert decision.source == "reassembled"


# ---------------------------------------------------------------------------
# Factorization reuse, and the entry bound
# ---------------------------------------------------------------------------


def test_held_tangent_serves_a_held_tangent_and_refuses_an_unheld_one() -> None:
    """
    The held-tangent lookup answers only for tangents the cache actually holds.

    Synthesising a tangent for a matrix nobody assembled would break the "reuse must
    have a provenance" rule the design states, so the miss is refused by name
    instead of being computed.

    The method is deliberately *not* named ``factorize``: this layer never factors
    anything, and a matrix handed out under that name would let a caller believe
    they held a factorization handle.
    """
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    with pytest.raises(CharacteristicsError, match="no retained tangent"):
        cache.held_tangent(key(), matrix)

    cache.tangent(key(), matrix)
    _, decision = cache.held_tangent(key(), np.array([[4.0, 1.0], [0.5, 3.0]]))
    assert decision.source == "numeric_reuse"
    assert decision.numeric_reused is True
    assert not hasattr(cache, "factorize"), (
        "a matrix must not be handed out under the name 'factorize'"
    )


def test_a_held_tangent_is_a_copy_the_caller_cannot_poison() -> None:
    """
    The held-tangent path must copy too.

    It previously returned the cache's own array while its value digest still
    recorded the original, so a mutation was served afterwards as a legitimate
    numeric reuse -- more hidden than the same mistake on the ``tangent`` path,
    because the digest never noticed.
    """
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    cache.tangent(key(), matrix)

    held, _ = cache.held_tangent(key(), matrix)
    held[0, 0] = -999.0

    again, _ = cache.tangent(key(), matrix)
    assert again[0, 0] == 4.0, "the retained matrix must not be reachable by mutation"


def test_the_cache_evicts_old_entries_once_it_is_full() -> None:
    """An unbounded store would let a long run retain stale tangents indefinitely."""
    cache = SampleTangentCache(max_entries=2)
    for sample in range(3):
        cache.tangent(key(sample=sample), np.eye(2) * (sample + 1))
    assert cache.entry_count == 2

    # Sample 0 was evicted, so it is assembled afresh rather than reused.
    _, decision = cache.tangent(key(sample=0), np.eye(2))
    assert decision.source == "reassembled"


def test_the_cache_refuses_a_capacity_that_cannot_hold_anything() -> None:
    with pytest.raises(CharacteristicsError):
        SampleTangentCache(max_entries=0)


# ---------------------------------------------------------------------------
# The decision record must not overstate what happened
# ---------------------------------------------------------------------------


def test_a_reuse_decision_that_reused_nothing_is_refused() -> None:
    """
    A decision source claiming reuse while flagging nothing reused would put a lie
    in the audit trail a budget check reads.
    """
    with pytest.raises(CharacteristicsError):
        CacheDecision(source="numeric_reuse", numeric_reused=False)
    with pytest.raises(CharacteristicsError):
        CacheDecision(source="structural_reuse", structural_reused=False)
    with pytest.raises(CharacteristicsError):
        CacheDecision(source="teleportation")


def test_a_cold_cache_does_not_claim_reuse() -> None:
    assert CacheDecision(source="reassembled").reused is False
    assert CacheDecision(source="numeric_reuse", numeric_reused=True).reused is True


def test_a_real_factorization_is_counted_by_whoever_factored() -> None:
    """
    ``factorization_count`` must track real factorizations, not reuses.

    Previously a *reuse* incremented it while the cold path did not, so the number
    rose when less work happened -- the opposite of what a budget check needs.
    """
    counters = CacheCounters()
    assert counters.factorization_count == 0
    counters.record_factorization(dimension=8, seconds=0.25)
    assert counters.factorization_count == 1
    assert counters.factorization_time_s == pytest.approx(0.25)
    assert counters.peak_matrix_bytes == 8 * 8 * 8


def test_a_pure_reuse_does_not_increase_the_factorization_count() -> None:
    """Reusing a factorization is the absence of a factorization, not one of them."""
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    cache.tangent(key(), matrix)
    before = cache.counters.factorization_count
    cache.tangent(key(), matrix)
    assert cache.counters.factorization_count == before
    assert cache.counters.numeric_reuse == 1


def test_a_solve_is_recorded_with_its_right_hand_side_count() -> None:
    """
    One solve over several columns is one solve and several independent RHS.

    Conflating the two would either overstate the linear-algebra cost or hide how
    much work the sharing actually saved.
    """
    counters = CacheCounters()
    counters.record_solve(right_hand_sides=6, seconds=0.5)
    assert counters.solve_count == 1
    assert counters.independent_rhs == 6
    assert counters.solve_time_s == pytest.approx(0.5)


def test_recording_nothing_is_refused() -> None:
    """A zero-column solve or projection is not a fact about work performed."""
    counters = CacheCounters()
    with pytest.raises(CharacteristicsError):
        counters.record_solve(right_hand_sides=0)
    with pytest.raises(CharacteristicsError):
        counters.record_projection(count=0)


def test_the_projection_count_is_recorded_by_the_layer_that_projects() -> None:
    counters = CacheCounters()
    counters.record_projection(count=3)
    assert counters.projection_count == 3


def test_peak_memory_is_tracked_over_time_not_summed() -> None:
    """Peak memory is a high-water mark; two records do not add."""
    counters = CacheCounters()
    counters.record_factorization(dimension=4)
    counters.record_factorization(dimension=2)
    assert counters.peak_matrix_bytes == 4 * 4 * 8


def test_counters_merge_field_by_field() -> None:
    """
    Merging is how per-sample metering rolls up to a whole-run budget check, so a
    field silently dropped from the merge would under-report the cost.
    """
    total, part = CacheCounters(), CacheCounters()
    part.assembly_count, part.factorization_count = 2, 3
    part.assembly_time_s, part.peak_matrix_bytes = 1.5, 4096
    total.merge(part)
    total.merge(part)

    assert total.assembly_count == 4
    assert total.factorization_count == 6
    assert total.assembly_time_s == pytest.approx(3.0)
    assert total.peak_matrix_bytes == 4096, "peak memory is a maximum, not a sum"


def test_the_counter_report_has_a_stable_shape() -> None:
    """A budget check reads these by name, so the names are the contract."""
    report = CacheCounters().as_dict()
    for name in (
        "assembly_count",
        "factorization_count",
        "solve_count",
        "projection_count",
        "independent_rhs",
        "response_hits",
        "response_misses",
        "peak_matrix_bytes",
    ):
        assert name in report, f"the metering report must carry {name}"
        assert report[name] == 0


def test_independent_right_hand_sides_are_counted_not_questions() -> None:
    """
    A repeated question costs no solve, so counting questions would overstate the
    work and hide whether the sharing happened at all.
    """
    cache = SampleTangentCache()
    matrix = np.array([[4.0, 1.0], [0.5, 3.0]])
    cache.tangent(key(), matrix)

    name = SampleTangentCache.response_key(modes=["heave"], measurements=["h"])
    if cache.response(name) is None:
        cache.counters.record_solve(right_hand_sides=1)
        cache.store_response(name, 1.0)
    # The identical question again: no new independent right-hand side.
    assert cache.response(name) == 1.0

    assert cache.counters.independent_rhs == 1
    assert cache.counters.solve_count == 1
    assert cache.counters.response_hits == 1
