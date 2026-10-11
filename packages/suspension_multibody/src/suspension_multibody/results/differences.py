"""
Explicit central differencing over a sampled sequence, with its own provenance.

This module exists because a difference quotient is sometimes the only available answer
-- for a quantity the solver did not store a rate for -- and the danger is not the
arithmetic but the *silence*.  A quotient silently assumes four things that are often
false:

* that the sequence is ordered and strictly increasing in time (a repeated or reversed
  timestamp makes the divisor zero or negative);
* that neighbouring samples are on the same branch of the motion (a contact event or an
  activity-mode switch between them makes the quotient an average across a
  discontinuity, which is not the derivative on either side);
* that the neighbours are valid (interpolating a value from an unavailable sample, or
  from a point the contract already refused, invents data);
* that the result is a derivative (a central quotient is second-order accurate in the
  step, not exact, and the estimate of that error needs the third difference).

So this operator refuses all four rather than returning a plausible number, and it
records what it did: which template was used at each sample, which endpoints were
one-sided, the real time differences, and an error scale.  It never claims to be
analytic -- :func:`..trajectory.geometry_rate_request` is the analytic path, and the two
are deliberately separate so a caller cannot mistake one for the other.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from ..characteristics.values import CharacteristicsError, MeasurementValue

__all__ = [
    "DIFFERENCE_TEMPLATES",
    "SampledDifference",
    "sampled_time_difference",
]

#: The templates this operator will use, named so a result can be audited.
#: ``central`` uses both neighbours, the two ``one_sided`` forms use one, and a sample
#: with no usable neighbour is ``undifferenced`` -- reported, never guessed.
DIFFERENCE_TEMPLATES = (
    "central",
    "forward_one_sided",
    "backward_one_sided",
    "undifferenced",
)

#: Below this step a division is treated as undefined rather than large.  A repeated
#: timestamp is a real defect in the sequence, and reporting a huge rate hides it.
MIN_STEP_S = 1e-12


@dataclass(frozen=True)
class SampledDifference:
    """
    A differenced quantity with the full account of how each sample was obtained.

    Every per-sample array is the same length as the input.  ``template`` names the
    stencil used at that sample (see :data:`DIFFERENCE_TEMPLATES`) and ``time_step`` is
    the **span the stencil reached across**: one interval for a one-sided sample, two for
    a central one.  It is a measure of the stencil's extent, not a divisor -- a central
    sample's quotient divides by ``h+ h- (h+ + h-)``, of which the span is one factor --
    so a caller must read the template to know how the value was formed.  Samples where
    only one side was available are marked in ``one_sided``.  A sample that could not be
    differenced carries NaN and a reason, in the same shape as any other unavailable
    reading.

    ``curvature_scale`` measures how much the local slope changes across the stencil, in
    the units of the rate.  It is NaN for an undifferenced sample.  It is deliberately
    **not** named an error: a three-point stencil cannot observe the third derivative that
    sets the central quotient's truncation error, so no honest error estimate is available
    here, and a field that claimed to be one would drift away from the real error on
    exactly the functions where it matters (measured: on ``f = t^3`` the true error is
    constant while any curvature-based quantity grows with ``t``).  A caller that needs a
    truncation bound must obtain it by refining the grid and comparing runs.
    """

    measurement_id: str
    values: np.ndarray
    units: str
    frame_id: str
    template: tuple[str, ...]
    time_step: np.ndarray
    one_sided: np.ndarray
    curvature_scale: np.ndarray
    status: tuple[str, ...]
    reason: tuple[str, ...]

    def __len__(self) -> int:
        return len(self.values)

    @property
    def one_sided_mask(self) -> np.ndarray:
        """Which samples were differenced with a single-sided template."""
        return self.one_sided


def _as_sequence(values: Any, label: str) -> np.ndarray:
    """Validate a sampled sequence and return it as a float array."""
    array = np.asarray(values, dtype=float)
    if array.ndim != 1:
        raise CharacteristicsError(f"{label} must be one-dimensional per sample")
    if array.size == 0:
        raise CharacteristicsError(f"{label} carries no samples")
    return array


def _check_time_axis(times: np.ndarray) -> None:
    """
    Refuse a time axis that cannot be differenced against.

    A non-increasing axis is a defect in the sequence, not something to work around: the
    divisor would be zero or negative, and a negative rate is indistinguishable from a
    real reversal in the data.
    """
    steps = np.diff(times)
    if np.any(steps <= 0.0):
        bad = int(np.sum(steps <= 0.0))
        raise CharacteristicsError(
            f"the time axis is not strictly increasing at {bad} step(s); a difference "
            "quotient needs a positive real time step, and a repeated timestamp is a "
            "defect in the sequence rather than a rate"
        )


def sampled_time_difference(
    times: Any,
    values: Any,
    *,
    blocks: Sequence[int] | None = None,
    valid: Any | None = None,
    measurement_id: str = "sampled_time_difference",
    units: str = "1/s",
    frame_id: str = "world",
) -> SampledDifference:
    """
    Difference a sampled sequence against its own real times.

    ``blocks`` marks the sample indices at which the motion is *not* differentiable
    across the boundary -- an event, an activity-mode switch, or any other declared
    discontinuity.  Each index in ``blocks`` starts a new block, and **no stencil ever
    crosses a block boundary**; the affected neighbours become one-sided or
    undifferenced.  This is the whole reason the parameter is required for a sequence
    that has events: without it a central quotient silently averages across the
    discontinuity.

    ``valid`` marks samples whose reading is available.  A sample next to an invalid one
    is differenced from the far side where that is possible, and otherwise reported as
    undifferenced with a reason -- an invalid neighbour is never substituted or
    interpolated, because filling it in would fabricate the very data the contract marks
    as missing.

    The result carries the template, the real time step actually used, the one-sided
    flags and an error scale per sample, so its provenance is auditable rather than
    implied.
    """
    time_axis = _as_sequence(times, "the time axis")
    sequence = _as_sequence(values, "the sampled sequence")
    if sequence.shape != time_axis.shape:
        raise CharacteristicsError(
            "a difference needs one value per timestamp; the two sequences disagree"
        )
    _check_time_axis(time_axis)
    count = sequence.size

    block_start = np.zeros(count, dtype=bool)
    if blocks is not None:
        for index in blocks:
            position = int(index)
            if position < 0 or position >= count:
                raise CharacteristicsError(
                    f"block boundary {position} is outside the sequence of {count} sample(s)"
                )
            block_start[position] = True
    # Sample 0 always starts a block: there is nothing before it to difference against.
    block_start[0] = True
    block_id = np.cumsum(block_start) - 1

    if valid is None:
        usable = np.isfinite(sequence)
    else:
        raw_valid = np.asarray(valid)
        if raw_valid.dtype != np.bool_:
            # Accepting a float array would silently reinterpret it by truthiness, and a
            # caller passing 0/1 weights would get "everything is valid" -- the opposite
            # of what such a mask usually means.  The contract elsewhere refuses
            # ill-typed input rather than coercing it, so do the same here.
            raise CharacteristicsError(
                f"the validity mask must be boolean, not {raw_valid.dtype}; a numeric "
                "mask would be reinterpreted by truthiness rather than by intent"
            )
        if raw_valid.shape != sequence.shape:
            raise CharacteristicsError("the validity mask must cover every sample")
        # A sample the caller marked valid but which holds no number is not usable;
        # saying so here keeps a NaN from entering the arithmetic as if it were a value.
        usable = raw_valid & np.isfinite(sequence)

    rates = np.full(count, np.nan)
    error = np.full(count, np.nan)
    steps = np.full(count, np.nan)
    templates: list[str] = []
    status: list[str] = []
    reasons: list[str] = []

    for index in range(count):
        if not usable[index]:
            templates.append("undifferenced")
            status.append("undefined_geometry")
            reasons.append("this sample has no value of its own, so it has no rate")
            continue
        first = index - 1
        second = index + 1
        has_before = (
            first >= 0 and usable[first] and block_id[first] == block_id[index]
        )
        has_after = (
            second < count and usable[second] and block_id[second] == block_id[index]
        )
        if has_before and has_after:
            # On a non-uniform grid the two half-steps differ, and the plain
            # (v[+1] - v[-1]) / (t[+1] - t[-1]) quotient is only first-order accurate --
            # it is the average of two different one-sided slopes weighted wrongly.  The
            # second-order form weights each side by the *other* side's step:
            #   f' ~ [ (h-)^2 * (f+ - f) + (h+)^2 * (f - f-) ] / [ h+ h- (h+ + h-) ]
            # with h- = t[i]-t[i-1] and h+ = t[i+1]-t[i].  On a uniform grid this
            # collapses to the familiar central quotient.
            before_step = time_axis[index] - time_axis[first]
            after_step = time_axis[second] - time_axis[index]
            span = before_step + after_step
            if before_step <= MIN_STEP_S or after_step <= MIN_STEP_S:
                templates.append("undifferenced")
                status.append("undefined_geometry")
                reasons.append("a neighbouring sample shares this timestamp")
                continue
            forward = sequence[second] - sequence[index]
            backward = sequence[index] - sequence[first]
            rate = (
                before_step * before_step * forward + after_step * after_step * backward
            ) / (before_step * after_step * span)
            # Curvature scale for this sample.
            #
            # The central quotient's leading truncation term is (h+ h- / 6) f''', and a
            # three-point stencil *cannot* see f''' -- it needs a fourth sample.  So this
            # is deliberately NOT an estimate of the truncation error, and naming it as
            # one would be a false claim: on f = t^3 (where f''' is constant and f'' grows
            # with t) a curvature-based scale drifts away from the true error as t grows.
            #
            # What three points can do is measure how much the local slope is changing,
            # which is the term that dominates the quotient's difference from a
            # one-sided slope.  The two one-sided slopes differ by (h+ + h-) f'' + O(h^2),
            # so half their spread times the span is a stable per-sample curvature scale
            # with the units of the rate.
            forward_slope = forward / after_step
            backward_slope = backward / before_step
            error[index] = 0.5 * abs(forward_slope - backward_slope) * span / 2.0
            steps[index] = span
            templates.append("central")
        elif has_after:
            span = time_axis[second] - time_axis[index]
            if span <= MIN_STEP_S:
                templates.append("undifferenced")
                status.append("undefined_geometry")
                reasons.append("the following sample shares this timestamp")
                continue
            rate = (sequence[second] - sequence[index]) / span
            steps[index] = span
            templates.append("forward_one_sided")
        elif has_before:
            span = time_axis[index] - time_axis[first]
            if span <= MIN_STEP_S:
                templates.append("undifferenced")
                status.append("undefined_geometry")
                reasons.append("the preceding sample shares this timestamp")
                continue
            rate = (sequence[index] - sequence[first]) / span
            steps[index] = span
            templates.append("backward_one_sided")
        else:
            templates.append("undifferenced")
            status.append("undefined_geometry")
            reasons.append(
                "no differencing neighbour lies in this block and is available, so the "
                "rate is not defined here rather than averaged across the boundary"
            )
            continue
        rates[index] = rate
        status.append("valid")
        reasons.append("")

    assert len(templates) == count
    values_array = np.array(rates, dtype=float, copy=True)
    values_array.setflags(write=False)
    one_sided = np.asarray(
        [
            name in ("forward_one_sided", "backward_one_sided")
            for name in templates
        ],
        dtype=bool,
    )
    return SampledDifference(
        measurement_id=measurement_id,
        values=values_array,
        units=units,
        frame_id=frame_id,
        template=tuple(templates),
        time_step=steps,
        one_sided=one_sided,
        curvature_scale=error,
        status=tuple(status),
        reason=tuple(reasons),
    )


def as_measurement(difference: SampledDifference) -> MeasurementValue:
    """
    Project a difference onto the contract's ordinary measurement shape.

    Kept separate on purpose: the projection drops the template, the one-sided flags and
    the error scale, so a caller that only needs a table gets one, and a caller auditing
    provenance must keep the :class:`SampledDifference` itself.  Making the difference a
    ``MeasurementValue`` directly would hide that loss.
    """
    return MeasurementValue(
        measurement_id=difference.measurement_id,
        values=difference.values,
        units=difference.units,
        frame_id=difference.frame_id,
        status=difference.status,
        reason=difference.reason,
    )
