"""
Drive the frozen-response request over a stored sample sequence.

The operators in this package each answer **one** question about **one** working
sample: :func:`~.equilibrium.equilibrium_response` solves ``A X = B_w P`` and
``Y = G X + D P`` at a single tangent, and :mod:`~.compliance` turns those
responses into rates.  What was missing is the layer that walks a stored result's
samples, asks each of them the same set of questions, and reports the answers as
**sequences** in sample order with their provenance attached.

That layer is this module.  It is deliberately thin, because everything
interesting already exists and two properties must not be re-invented:

* **One factorization per sample, shared by every question.**  The design asks
  for this directly (design §6.6 line 424-429): assemble and factor once per
  working sample and batch all compatible modes.  Asking a second metric must not
  re-factor the same matrix.  That is what ``SampleTangentCache`` is for, and this
  module drives it rather than bypassing it.
* **Provenance is recorded, not inferred.**  A rate is only meaningful together
  with the sample, boundary, mode amplitude and profile that produced it.  The
  design is explicit that a metric name and a mode kind are different things and
  that deriving one from the other is the confusion to prevent
  (:func:`~.compliance.metrics_request` says so in its own docstring).  Every
  result here carries the identity of the sample it came from, so two numbers can
  never be combined across samples by accident.

The driver never computes a physical quantity itself.  It sequences, it shares,
and it records.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from ..characteristics.values import CharacteristicsError
from .cache import FactorizationKey, SampleTangentCache
from .compliance import RateMetric
from .equilibrium import LoadMode, ModeResult, audited_modes

#: The sample kinds this driver will walk.  ``accepted`` is deliberately absent:
#: the frozen protocol is asked at stored samples, not at solver attempts, and
#: :mod:`~.sampling` refuses ``accepted`` for the same reason.
DRIVABLE_KINDS = ("initial", "output")

#: The sequence name a request with no measurement is answered under.
MOTION_NAME = "mechanism_motion"

#: The separator inside a qualified sequence key.  ASCII unit separator: it cannot
#: occur in a question id or a measurement name, so the key is invertible.
_KEY_SEPARATOR = "\x1f"

#: The single component a one-row-per-mode sequence declares.
ONLY_COMPONENT = "value"


@dataclass(frozen=True)
class SampleQuestion:
    """
    One question asked of every sample: a set of modes and the metrics over them.

    ``modes`` is the physical request -- named load modes with their amplitudes --
    and ``load_jacobian`` is its ``B_w``-shaped realisation, one column per mode.
    ``mode_matrix`` holds the per-mode amplitudes and is what labels the response
    columns, so it must agree with ``modes`` in length.

    The **tangent is not here**.  It belongs to the *sample*, not to the question,
    and putting it here would invite a caller to pass a different matrix per
    question -- which is exactly the re-assembly the design forbids.  It arrives
    through ``tangents`` on :func:`drive_sample_questions`, one per sample.

    ``measurements`` maps a measurement ID to ``(gradient, load_gradient)``, the
    ``G`` and ``D`` of ``Y = G X + D P``.  A measurement exists only if it has a
    gradient: ``gradient=None`` is refused rather than accepted, because the
    response layer answers ``X`` itself when ``G`` is absent, and publishing a
    mechanism coordinate under a measurement's name is a wrong number that looks
    perfectly plausible.  The "I only want the motion" request is expressed by
    asking no measurement at all.
    """

    question_id: str
    modes: tuple[LoadMode, ...]
    load_jacobian: Any
    mode_matrix: Any
    measurements: Mapping[str, tuple[Any, Any]] = field(default_factory=dict)
    measurement_units: Mapping[str, str] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.question_id:
            raise CharacteristicsError("a sample question needs an id to be traceable")
        if _KEY_SEPARATOR in self.question_id:
            # The id is one half of the sequence key, whose halves are joined by the
            # separator.  An id containing it would split back into two different
            # strings, so the key would stop being invertible and `sequence()` could
            # resolve a name to another question's series.
            raise CharacteristicsError(
                f"question id {self.question_id!r} contains the reserved separator "
                f"{_KEY_SEPARATOR!r}; the id is one half of the sequence key and the "
                "key has to be invertible"
            )
        if not self.modes:
            raise CharacteristicsError(
                f"question {self.question_id!r} names no load modes; a response "
                "request without a mode has nothing to answer"
            )
        # The columns of `mode_matrix` are what label the response, and nothing
        # downstream can recover a mismatch: a matrix with more columns than named
        # modes would be answered and then reported under the wrong labels.
        directions = np.asarray(self.mode_matrix, dtype=float)
        if directions.ndim != 2 or directions.shape[1] != len(self.modes):
            raise CharacteristicsError(
                f"question {self.question_id!r} has a mode matrix of shape "
                f"{directions.shape} for {len(self.modes)} named load mode(s); the "
                "columns are what label the response"
            )
        for name, entry in self.measurements.items():
            if _KEY_SEPARATOR in name:
                raise CharacteristicsError(
                    f"measurement name {name!r} contains the reserved separator "
                    f"{_KEY_SEPARATOR!r}; it is the other half of the sequence key"
                )
            if not isinstance(entry, tuple) or len(entry) != 2:
                # A malformed entry would otherwise escape later as an `IndexError`
                # from `entry[1]`, which says nothing about which measurement was
                # wrong.  The shape is part of the request, so it is checked here.
                raise CharacteristicsError(
                    f"measurement {name!r} must be a (gradient, load_gradient) pair, "
                    f"got {entry!r}"
                )
            if entry[0] is None:
                raise CharacteristicsError(
                    f"measurement {name!r} has no gradient; a named measurement must "
                    "carry the G of Y = G X + D P, and a missing G would make the "
                    "mechanism motion be published under this measurement's name. "
                    "Ask for no measurement to get the motion itself"
                )


@dataclass(frozen=True)
class SampleAnswer:
    """
    What one sample answered, with the identity of that sample attached.

    ``responses`` maps a measurement ID to the response values in mode order (or
    to ``X`` when the question asked for no measurement).  ``audit`` is the
    boundary audit the response layer returned, kept verbatim: a refusal must be
    able to say *why* it refused, and collapsing it to a status would lose that.

    ``sample_index`` is the index within the sequence this driver walked, and
    ``identity`` is the sample's own identity from the result.  Both are carried
    because they answer different questions -- "which of the samples I asked?" and
    "which stored sample was that?" -- and a sequence stitched together across two
    results needs the second.
    """

    sample_index: int
    identity: Any
    responses: Mapping[str, np.ndarray]
    audit: Mapping[str, Any]
    metrics: tuple[RateMetric, ...] = ()

    @property
    def answered(self) -> bool:
        """
        Whether the boundary determined an answer at this sample.

        This is decided from the audit **and** the per-mode refusals, not from the
        tangent's verdict alone.  ``audit['status']`` is the *tangent's* verdict: it
        stays ``valid`` in the case that matters most, where a mode drives no load
        direction and is refused on its own while the tangent is perfectly fine.
        Reading only the status would report ``answered`` for a sample whose every
        value is NaN, contradicting the sequence's own ``valid`` flag.
        """
        return self.audit.get("status") == "valid" and not self.audit.get(
            "refused_modes"
        )


@dataclass(frozen=True)
class SampleSequence:
    """
    A metric or response across samples, in sample order, with provenance.

    This is the shape the design's "actual Wheel/Ride/Roll/Toe/Camber sequence"
    refers to.  The two arrays are parallel and the same length: ``values`` is
    indexed by sample and ``valid`` says whether that entry is an answer or a
    refusal.  A refused entry holds ``NaN`` rather than zero, for the reason the
    response layer gives -- a zero-filled refusal is numerically indistinguishable
    from a genuinely zero response.

    ``mode_ids`` and the units are carried alongside so a caller can label a column
    without guessing, and ``provenance`` records the boundary and profile the whole
    sequence was formed under.  Mixing samples from two boundary profiles into one
    sequence is what this field exists to make visible.
    """

    name: str
    values: np.ndarray
    valid: np.ndarray
    units: str
    mode_ids: tuple[str, ...]
    sample_indices: tuple[int, ...]
    sample_identities: tuple[Any, ...]
    provenance: Mapping[str, Any]
    components: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        values = np.asarray(self.values, dtype=float)
        valid = np.asarray(self.valid, dtype=bool)
        # The converted arrays are written *back* onto the frozen instance.  Without
        # this the field kept whatever the caller passed -- a list, or the caller's
        # own writable array -- so `values` could be a list (and every arithmetic use
        # of it a `TypeError`) or could be mutated in place after publication, which
        # is exactly the aliasing the response layer's own `_readonly` exists to
        # prevent.  Validating a converted local and storing the original is the
        # classic way to get a validated-looking object that is not validated.
        object.__setattr__(self, "values", values)
        object.__setattr__(self, "valid", valid)
        # `valid` is per *sample*: a refusal is a property of the working sample
        # -- the boundary did not determine a motion -- and not of one mode column
        # within it.  So it is one flag per row of `values`, not the same shape.
        if valid.ndim != 1 or valid.shape[0] != values.shape[0]:
            raise CharacteristicsError(
                f"sequence {self.name!r} has {values.shape[0]} sample row(s) but "
                f"validity {valid.shape}; validity is one flag per sample"
            )
        if values.ndim != 2 or values.shape[0] != len(self.sample_indices):
            raise CharacteristicsError(
                f"sequence {self.name!r} must be (samples, columns) matching its "
                f"{len(self.sample_indices)} sample indices, got {values.shape}"
            )
        # The column count is the mode count times the component count.  Both are
        # *declared* by the caller and neither is derived from `values`, so the
        # product is an independent claim about the width rather than a restatement
        # of it.  Deriving `components` from the width (as an earlier version did)
        # makes this check a tautology that passes for any multiple of the mode
        # count -- including a silently truncated multi-component motion.
        #
        # An empty `components` means exactly one column per mode, so the product
        # still constrains the width; it is not a wildcard.
        components = self.components if self.components else (ONLY_COMPONENT,)
        expected = len(self.mode_ids) * len(components)
        if values.shape[1] != expected:
            raise CharacteristicsError(
                f"sequence {self.name!r} has {values.shape[1]} column(s) but "
                f"{len(self.mode_ids)} mode id(s) and {len(components)} "
                f"component(s) describe {expected}"
            )
        if not np.all(np.isnan(values[~valid])):
            # A refused entry that carries a finite number would be published as a
            # reading by any caller that forgot to consult `valid`.
            raise CharacteristicsError(
                f"sequence {self.name!r} carries finite values at refused samples; a "
                "refusal must not be representable as a number"
            )
        # The converse is checked too.  A sample marked answered must actually carry
        # numbers: today the driver cannot produce that combination, but the type is
        # publicly constructible and a `valid=True` row of NaN is the same trap in
        # the other direction -- a consumer reading `valid` alone would take a
        # non-existent answer for a reading.
        if valid.any() and not np.isfinite(values[valid]).all():
            raise CharacteristicsError(
                f"sequence {self.name!r} marks a sample answered but its values are "
                "not finite; an answered sample must carry a number"
            )
        # Frozen, so a consumer cannot rewrite a published answer and a later reader
        # cannot be handed a different number than an earlier one was.
        values.setflags(write=False)

    @property
    def sample_count(self) -> int:
        return len(self.sample_indices)

    @property
    def answered_count(self) -> int:
        return int(np.count_nonzero(self.valid))

    def at_sample(self, index: int) -> tuple[np.ndarray, bool]:
        """Return the values at one sample and whether they are an answer."""
        if index < 0 or index >= self.sample_count:
            raise CharacteristicsError(
                f"sample {index} is outside this sequence of {self.sample_count}"
            )
        return self.values[index], bool(self.valid[index])


@dataclass(frozen=True)
class PerSampleReport:
    """
    Every sequence one driving pass produced, plus what the pass cost.

    ``sequences`` is keyed by measurement ID.  ``counters`` is the cache's own
    counter report, so the factorization sharing the design requires is
    **observable** rather than asserted: ``factorizations`` should equal the number
    of samples asked, not the number of questions times the number of samples.

    ``questions`` records the question IDs in the order they were asked, and
    ``provenance`` the boundary the whole pass ran under.
    """

    sequences: Mapping[str, SampleSequence]
    counters: Mapping[str, float | int]
    questions: tuple[str, ...]
    provenance: Mapping[str, Any]
    answers: tuple[SampleAnswer, ...] = ()

    def __post_init__(self) -> None:
        if not self.sequences:
            raise CharacteristicsError(
                "a per-sample report with no sequence answered nothing; refusing "
                "rather than returning an empty report that reads as success"
            )
        if not self.questions:
            raise CharacteristicsError("a per-sample report must name its questions")

    def sequence(self, name: str) -> SampleSequence:
        """
        Return one sequence, by qualified key or by bare measurement name.

        A bare name is accepted only when exactly one question reported it.  With
        two questions reporting the same measurement the name is genuinely
        ambiguous -- the two are different physical quantities -- so resolving it
        by picking one would be a guess, and the error names both candidates
        instead.
        """
        if name in self.sequences:
            return self.sequences[name]
        matches = [
            key for key, sequence in self.sequences.items() if sequence.name == name
        ]
        if len(matches) == 1:
            return self.sequences[matches[0]]
        if len(matches) > 1:
            raise CharacteristicsError(
                f"sequence name {name!r} is ambiguous; it was reported by "
                f"{len(matches)} questions ({', '.join(sorted(matches))}), which are "
                "different physical questions -- name one by its qualified key"
            )
        raise CharacteristicsError(
            f"no sequence {name!r} in this report; it holds "
            f"{', '.join(sorted(self.sequences))}"
        )

    @property
    def sample_count(self) -> int:
        return next(iter(self.sequences.values())).sample_count


def drive_sample_questions(
    result: Any,
    questions: Sequence[SampleQuestion],
    tangents: Sequence[Any],
    *,
    sample_kind: str,
    model_fingerprint: str,
    plan_fingerprint: str,
    boundary_profile: str,
    boundary_version: int,
    definition_version: int = 1,
    frozen_protocol: str = "",
    frozen_version: int = 1,
    max_entries: int = 4,
) -> PerSampleReport:
    """
    Ask every question of every stored sample, sharing one factorization per sample.

    ``result`` is a stored run -- anything exposing the sample-selection interface
    :func:`~.sampling.select_samples` uses -- and ``sample_kind`` selects which
    samples are walked.  ``accepted`` is refused: the frozen protocol is asked at
    stored samples, not at solver attempts, and answering at an attempt would
    report a state the solver did not keep.

    ``tangents[i]`` is the physical tangent at sample ``i`` of the selection.  It is
    an argument of its own rather than a field of a question because it belongs to
    the sample: making it per-question would let two questions at the same sample
    disagree about the mechanism they are asking about, and the answers would still
    look plausible.

    The loop is *sample-outer, question-inner*, which is the whole point.  Asking
    question-outer would re-factor each sample once per question and quietly
    destroy the sharing; nothing about the answers would look wrong, which is why
    the counter report is returned rather than left to be inferred.

    A question that the boundary cannot determine is recorded as a refusal at that
    sample, not raised: one singular sample in a long sequence must not discard the
    samples that did answer.  A malformed *question* -- wrong mode count, wrong
    dimensions -- is raised, because that is a mistake in the request rather than a
    property of any sample.
    """
    from .sampling import select_samples

    if sample_kind not in DRIVABLE_KINDS:
        raise CharacteristicsError(
            f"cannot drive frozen responses over {sample_kind!r} samples; the frozen "
            f"protocol is asked at {', '.join(DRIVABLE_KINDS)} samples, not at solver "
            "attempts"
        )
    if not questions:
        raise CharacteristicsError("a driving pass needs at least one question")

    seen: set[str] = set()
    for question in questions:
        if question.question_id in seen:
            raise CharacteristicsError(
                f"question id {question.question_id!r} is used twice; the id is what "
                "makes a sequence traceable and a duplicate makes it ambiguous"
            )
        seen.add(question.question_id)

    selection = select_samples(result, sample_kind)
    identities = tuple(selection.identities)
    if not identities:
        raise CharacteristicsError(
            f"the sample selection {sample_kind!r} chose no samples; there is no "
            "sequence to form"
        )
    if len(tangents) != len(identities):
        raise CharacteristicsError(
            f"{len(tangents)} tangent(s) for {len(identities)} selected sample(s); "
            "each sample needs the mechanism it is being asked about"
        )

    sequential: dict[str, dict[str, list[Any]]] = {}
    for question in questions:
        for name in _names_of(question):
            sequential.setdefault(name, {"values": [], "valid": []})
    cache = SampleTangentCache(max_entries=max_entries)
    answers: list[SampleAnswer] = []
    collected: dict[str, dict[str, list[Any]]] = sequential

    for position, identity in enumerate(identities):
        sample_index = int(identity.index)
        key = FactorizationKey(
            model_fingerprint=model_fingerprint,
            plan_fingerprint=plan_fingerprint,
            sample_index=sample_index,
            domain="frozen_local_response",
            definition_version=definition_version,
            boundary_profile=boundary_profile,
            boundary_version=boundary_version,
            frozen_protocol=frozen_protocol,
            frozen_version=frozen_version,
            active_modes=tuple(
                mode.kind for question in questions for mode in question.modes
            ),
        )
        # Factored once, at this sample, before any question is asked of it.  The
        # factorization counter belongs to whoever factored, and the cache cannot
        # know whether its caller reused or rebuilt -- so the driver records it
        # exactly when the cache reports that this sample was not a reuse.  Doing
        # it unconditionally would report one factorization per *question*, which
        # is precisely the regression this counter exists to expose.
        #
        # The predicate is `numeric_reused`, not `reused`.  `reused` is true for a
        # *structural* hit as well, and that path still performs a real numeric
        # factorization (cache.py records `numeric_invalidations` and stores a new
        # array).  Branching on `reused` would therefore under-count precisely in
        # the case the counter exists to measure -- two samples that share a
        # sparsity pattern but not a value.
        tangent, decision = cache.tangent(
            key, np.asarray(tangents[position], dtype=float)
        )
        if not decision.numeric_reused:
            cache.counters.record_factorization(dimension=int(tangent.shape[0]))
        for question in questions:
            # `audited_modes`, not `equilibrium_response`.  The two are not
            # interchangeable: `equilibrium_response` audits the tangent globally,
            # while the per-mode "this mode drives no load direction, so its
            # response is undefined rather than zero" refusal lives only in
            # `audited_modes` (equilibrium.py:427-460).  Calling the lower-level
            # entry point here would publish `X = A^{-1} 0 = 0` as a finite,
            # valid-looking zero -- the exact failure the equilibrium module
            # documents itself as existing to prevent.
            modes, audit = audited_modes(
                tangent,
                question.load_jacobian,
                question.modes,
                # `P` is passed through rather than re-derived from `modes`.  The
                # amplitude, the wheel end and the axis all live in this matrix
                # (design §6.6), so re-deriving it would solve the unit basis and
                # report the result under the caller's mode names -- a wrong answer
                # with correct-looking labels.
                mode_matrix=question.mode_matrix,
                measurement_gradient=_stack_measurements(question),
                measurement_units=_units_of(question),
                measurement_load_gradient=_stack_load_terms(question),
            )
            responses = _responses_from_modes(
                question, modes, rows=_answer_rows(question, tangent)
            )
            cache.counters.record_solve(
                right_hand_sides=int(np.asarray(question.mode_matrix, dtype=float).shape[1])
            )
            _collect(question, responses, modes, audit, collected)
            answers.append(
                SampleAnswer(
                    # The *stored* sample index, not the loop position.  The two
                    # differ whenever a selection skips samples -- `initial` over
                    # two cases gives 0 and 3 -- and a record whose two "which
                    # sample" fields disagree would be read as naming two samples
                    # or one, depending on which field the consumer happened to
                    # trust.
                    sample_index=sample_index,
                    identity=identity,
                    responses=_split_responses(question, responses),
                    audit=dict(audit),
                )
            )

    sequences = _build_sequences(questions, collected, identities, answers)
    return PerSampleReport(
        sequences=sequences,
        counters=cache.counters.as_dict(),
        questions=tuple(question.question_id for question in questions),
        provenance={
            "sample_kind": sample_kind,
            "model_fingerprint": model_fingerprint,
            "plan_fingerprint": plan_fingerprint,
            "boundary_profile": boundary_profile,
            "boundary_version": boundary_version,
            "definition_version": definition_version,
            "frozen_protocol": frozen_protocol,
            "frozen_version": frozen_version,
            "sample_count": len(identities),
        },
        answers=tuple(answers),
    )


def _names_of(question: SampleQuestion) -> tuple[str, ...]:
    """
    Return the sequence names one question contributes, qualified by its id.

    The qualification is not decoration.  Two questions asked at the same samples
    may legitimately report a measurement with the same *name* -- a wheel rate
    over an in-phase mode and over a roll mode, say -- and folding them into one
    column would place two different physical quantities in a single sequence
    while every entry stayed finite.  Naming the question in the key keeps them
    apart, and the sequence keeps the unqualified measurement name alongside its
    question id so a caller can still find it by the name they used.
    """
    if not question.measurements:
        return (_qualified(question.question_id, MOTION_NAME),)
    return tuple(
        _qualified(question.question_id, name)
        for name in _measurement_order(question)
    )


def _qualified(question_id: str, measurement: str) -> str:
    r"""
    Join a question id and a measurement name into one unambiguous key.

    A ``:`` separator is not safe: a measurement name may legitimately contain one,
    and a question id is not validated against it, so ``"a:b" + ":" + "x"`` cannot
    be split back into the parts it came from.  ``\x1f`` (the ASCII unit
    separator) cannot appear in either part, which is what makes the inverse in
    :func:`_split_key` total rather than best-effort.
    """
    return f"{question_id}{_KEY_SEPARATOR}{measurement}"


def _split_key(key: str) -> tuple[str, str]:
    """Split a qualified key back into ``(question_id, measurement)``."""
    question_id, _, measurement = key.partition(_KEY_SEPARATOR)
    return question_id, measurement


def _measurement_order(question: SampleQuestion) -> tuple[str, ...]:
    """
    Return the one order that measurement rows are stacked and read back in.

    Every producer and consumer of a measurement row must agree, because the rows
    are positional: stacking by insertion order and reading back by sorted order
    silently swaps two named physical quantities, and both values stay finite and
    plausible.  Sorting is chosen as the single authority -- it is reproducible
    from the names alone, so a caller cannot change the meaning of a sequence by
    rebuilding an equal dictionary in a different order.
    """
    return tuple(sorted(question.measurements))


def _stack_measurements(question: SampleQuestion) -> Any:
    """
    Stack the per-measurement gradients ``G`` into one ``(measurements, ndof)`` block.

    There is no "absent entry" case to handle here: :class:`SampleQuestion` refuses
    a named measurement without a gradient, precisely so that this block cannot
    silently degrade to ``None``.  Returning ``None`` used to make the layer below
    early-return the *mechanism motion* in place of the measurement, which is a
    finite, plausible, wrong number under the measurement's name.
    """
    rows = [question.measurements[name][0] for name in _measurement_order(question)]
    if not rows:
        return None
    return np.vstack([np.asarray(row, dtype=float) for row in rows])


def _units_of(question: SampleQuestion) -> str:
    """
    Return the units token to publish for a question's measurements.

    A single measurement's units travel through unchanged.  Several measurements
    may legitimately disagree -- a translation in metres and an angle in radians are
    both plausible members of one request -- and then no single token is true, so
    the empty string is returned and the per-measurement units stay on the request
    rather than being asserted as one value for the whole block.  Claiming the first
    measurement's unit for all of them would put "m" on an angle.
    """
    units = question.measurement_units
    if not question.measurements:
        return ""
    tokens = {units.get(name, "") for name in _measurement_order(question)}
    return tokens.pop() if len(tokens) == 1 else ""


def _stack_load_terms(question: SampleQuestion) -> Any:
    """
    Stack the per-measurement direct terms ``D``, preserving "absent" as a unit.

    ``None`` on a single measurement means *that measurement has no direct term*,
    which is a zero row -- not a reason to discard the whole block.  Returning
    ``None`` as soon as one entry was missing dropped a real direct term belonging
    to a different measurement, so ``Y = G X + D P`` silently lost the term for
    every measurement that had one as soon as a sibling had none.  The block is
    returned only when at least one entry is present; if none is, there is no
    direct term at all and ``None`` is the honest answer.

    The earlier version also tested the entries' *size*, which discarded a genuine
    single-measurement ``D`` because one row of shape ``(1, n)`` is
    indistinguishable from the degenerate case by size alone.
    """
    entries = [question.measurements[name][1] for name in _measurement_order(question)]
    if not entries or all(entry is None for entry in entries):
        return None
    return np.vstack(
        [
            np.zeros((1, 6), dtype=float)
            if entry is None
            else np.asarray(entry, dtype=float)
            for entry in entries
        ]
    )


def _split_responses(
    question: SampleQuestion, responses: Any
) -> dict[str, np.ndarray]:
    array = np.asarray(responses, dtype=float)
    if not question.measurements:
        return {"mechanism_motion": array}
    names = tuple(sorted(question.measurements))
    if array.ndim == 1:
        return {names[0]: array}
    return {name: array[position] for position, name in enumerate(names)}


def _answer_rows(question: SampleQuestion, tangent: Any) -> int:
    """
    Return how many response rows a question has, from the *request* not the answer.

    This is the count that keeps a refusal the same shape as an answer.  Deriving it
    from whichever mode happened to answer (as an earlier version did) makes an
    all-refused sample collapse to a single row, and the shape then differs between
    the refused and answered samples of the same sequence -- so stacking them either
    raises deep inside numpy or, worse with one measurement, silently reads the
    wrong row.  The count is a property of the request:

    * a named measurement contributes one row per measurement;
    * the motion contributes one row per mechanism degree of freedom, which the
      caller's tangent defines.
    """
    if question.measurements:
        return len(question.measurements)
    matrix = np.asarray(tangent, dtype=float)
    if matrix.ndim == 2 and matrix.shape[0] == matrix.shape[1] and matrix.shape[0] > 0:
        return int(matrix.shape[0])
    # The tangent is not a usable square matrix; the layer below will refuse the
    # request with a reason, and one row is the shape that keeps this function
    # total rather than guessing a width that would then be reported as a reading.
    return 1


def _responses_from_modes(
    question: SampleQuestion, modes: Sequence[ModeResult], *, rows: int
) -> np.ndarray:
    """
    Rebuild the ``(rows, modes)`` response block from audited per-mode results.

    ``audited_modes`` answers mode by mode, and a refused mode carries ``None``
    values rather than a number.  Those ``None``s are turned into ``NaN`` here so
    the block stays rectangular; the per-mode *status* travels alongside in
    :func:`_collect`, because a NaN alone cannot say whether the sample answered.

    ``rows`` is supplied by the caller from the request rather than inferred from
    the answers, so a refused sample has the same shape as an answered one and the
    two can be stacked into one sequence.
    """
    count = len(modes)
    width = max(1, rows)
    block = np.full((width, max(1, count)), np.nan, dtype=float)
    for index, result in enumerate(modes):
        if result.values is None:
            continue
        column = np.atleast_1d(np.asarray(result.values, dtype=float))
        if column.size == width:
            block[:, index] = column
        else:
            # The layer below answered with a width this request did not ask for.
            # Leaving the column as NaN is the only honest option -- writing a
            # truncated or padded column would put a wrong number where a refusal
            # belongs -- and it keeps the refusal visible in the sequence.
            continue
    return block


def _collect(
    question: SampleQuestion,
    responses: Any,
    modes: Sequence[ModeResult],
    audit: Mapping[str, Any],
    collected: dict[str, dict[str, list[Any]]],
) -> None:
    """
    Record one sample's answers into the per-name column lists.

    Usability is decided **per mode**, from the mode's own status, and only then
    folded per sample.  Using the global audit alone would be wrong in both
    directions: it is ``valid`` for a request whose individual modes were refused
    (the zero-load-direction case, which ``audited_modes`` catches and the global
    audit cannot see), and it can be ``unidentifiable`` for a request every one of
    whose modes still answered.  A sample is usable only when *every* mode it was
    asked about answered.
    """
    array = np.asarray(responses, dtype=float)
    per_mode = tuple(result.status == "valid" for result in modes)
    usable = bool(per_mode) and all(per_mode)
    names = _measurement_order(question)
    for key in _names_of(question):
        _question_id, measurement = _split_key(key)
        if measurement == MOTION_NAME and not question.measurements:
            column = _row(array, None)
        else:
            column = _row(
                array, names.index(measurement) if measurement in names else None
            )
        if not usable:
            column = np.full(np.shape(column), np.nan, dtype=float)
        collected[key]["values"].append(
            np.atleast_1d(np.asarray(column, dtype=float))
        )
        collected[key]["valid"].append(usable)
        collected[key].setdefault("mode_valid", []).append(per_mode)


def _row(array: np.ndarray, index: int | None) -> np.ndarray:
    """
    Reduce a response array to the row a sequence stores.

    The response layer returns ``(rows, modes)``.  With a measurement named,
    ``rows`` are the measurement rows the caller asked for and the named one is
    selected.  With no measurement, ``rows`` are the mechanism's own degrees of
    freedom -- there is no name to select among them and *all* of them are the
    answer, so the whole block is kept and transposed to one column per mode.
    Taking only the first row there would silently answer a one-degree-of-freedom
    question and look entirely plausible.
    """
    if array.ndim == 1:
        return array
    if index is not None:
        return array[index]
    # `(rows, modes)` -> flattened `(modes * rows,)`: mode-major, so the component
    # index advances within a mode and the caller's mode list still labels blocks.
    return array.T.reshape(-1)


def _build_sequences(
    questions: Sequence[SampleQuestion],
    collected: dict[str, dict[str, list[Any]]],
    identities: tuple[Any, ...],
    answers: Sequence[SampleAnswer],
) -> dict[str, SampleSequence]:
    modes_of = {
        question.question_id: tuple(mode.kind for mode in question.modes)
        for question in questions
    }
    indices = tuple(int(identity.index) for identity in identities)
    sequences: dict[str, SampleSequence] = {}
    for key, columns in collected.items():
        if not columns["values"]:
            continue
        question_id, measurement = _split_key(key)
        values = np.stack(
            [np.atleast_1d(np.asarray(column, dtype=float)) for column in columns["values"]]
        )
        valid = np.array(columns["valid"], dtype=bool)
        sequences[key] = SampleSequence(
            name=measurement,
            values=values,
            valid=valid,
            units="",
            mode_ids=modes_of.get(question_id, ()),
            sample_indices=indices,
            sample_identities=identities,
            provenance={
                "question_id": question_id,
                "measurement": measurement,
            },
            components=_components_of(questions, question_id, measurement, values.shape[1]),
        )
    return sequences


def _components_of(
    questions: Sequence[SampleQuestion],
    question_id: str,
    measurement: str,
    width: int,
) -> tuple[str, ...]:
    """
    Name the columns of a sequence so its width is checkable, not assumed.

    A named measurement contributes one column per mode.  A motion contributes one
    column per mode *per mechanism degree of freedom*, and the degrees of freedom
    have no names in this layer -- the caller's tangent defines them -- so they are
    labelled positionally.  Both cases must produce a tuple whose length times the
    mode count is the sequence width; that product is the check that catches a
    silently truncated motion.
    """
    modes = next(
        (question.modes for question in questions if question.question_id == question_id),
        (),
    )
    mode_count = max(1, len(modes))
    components = width // mode_count
    if components <= 1:
        return (ONLY_COMPONENT,)
    return tuple(f"component_{index}" for index in range(components))


__all__ = [
    "DRIVABLE_KINDS",
    "PerSampleReport",
    "SampleAnswer",
    "SampleQuestion",
    "SampleSequence",
    "drive_sample_questions",
]
