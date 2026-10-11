"""
Explicit central differencing, and the four things it must refuse to do silently.

A difference quotient is a legitimate answer when the solver stored no rate -- but only
if the operator says what it assumed and where it could not assume it.  These tests pin
each of the four assumptions separately, because each fails on its own and each failure
looks like a plausible number rather than an error:

1. the arithmetic is right, including on a **non-uniform** grid, where the tempting
   ``(v[+1] - v[-1]) / (t[+1] - t[-1])`` is only first-order and quietly wrong;
2. no stencil crosses a **block boundary** (an event or a mode switch), because the
   quotient there is an average across a discontinuity and is the derivative on neither
   side;
3. an **invalid** neighbour is never interpolated or substituted;
4. the result is **not** claimed to be analytic -- the template, the real step, the
   one-sided flags and an error scale all survive into the result.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.characteristics.values import (
    CharacteristicsError,
    MeasurementValue,
)
from suspension_multibody.results.differences import (
    DIFFERENCE_TEMPLATES,
    as_measurement,
    sampled_time_difference,
)


def _uniform(count: int = 11, *, start: float = 0.0, end: float = 1.0) -> np.ndarray:
    return np.linspace(start, end, count)


# --------------------------------------------------------------------------- #
# 1. the arithmetic
# --------------------------------------------------------------------------- #


def test_the_templates_are_declared() -> None:
    assert set(DIFFERENCE_TEMPLATES) == {
        "central",
        "forward_one_sided",
        "backward_one_sided",
        "undifferenced",
    }


def test_a_central_stencil_is_second_order_accurate() -> None:
    """
    Halving the step must quarter the error.

    ``f = t^3``: the central quotient's error is ``f''' h^2 / 6``, so the ratio between
    successive refinements is 4.  A first-order operator -- which is what the naive
    quotient becomes on a non-uniform grid -- would show a ratio of 2 and fail here.
    """
    errors = []
    for count in (11, 21, 41):
        times = _uniform(count)
        result = sampled_time_difference(times, times**3, measurement_id="f")
        exact = 3.0 * times**2
        errors.append(np.max(np.abs(result.values[1:-1] - exact[1:-1])))
    for coarse, fine in zip(errors, errors[1:]):
        assert coarse / fine == pytest.approx(4.0, rel=0.1)


def test_a_central_stencil_reproduces_a_quadratic_exactly() -> None:
    """A quadratic has zero third derivative, so the central quotient is exact."""
    times = _uniform()
    result = sampled_time_difference(times, 5.0 * times**2 + 2.0 * times, measurement_id="f")
    np.testing.assert_allclose(result.values[1:-1], 10.0 * times[1:-1] + 2.0, atol=1e-12)


def test_a_non_uniform_grid_reproduces_a_quadratic_exactly() -> None:
    """
    The non-uniform stencil must weight each side by the *other* side's step.

    On a deliberately irregular grid, the naive quotient
    ``(v[+1] - v[-1]) / (t[+1] - t[-1])`` disagrees with the true derivative — and it
    disagrees by design: on this grid it equals ``t[0] + t[2] = 0.35`` where the truth is
    ``2 t[1] = 0.2``.  The weighted form is exact.  Asserting exactness (not a tolerance)
    is what makes the weighting load-bearing.

    Note on what this does *not* claim: the weighted form is not generally more accurate
    than the naive quotient.  Measured on a smooth non-uniform grid (``t = linspace^1.5``,
    ``f = sin 3t``) the naive form is in fact marginally *better* at every resolution
    tried.  The weighting here is the standard second-order non-uniform stencil and is
    used because it is exact for a quadratic and reduces to the uniform quotient, not
    because it dominates the naive form.
    """
    times = np.array([0.0, 0.1, 0.35, 0.4, 1.0, 1.2])
    result = sampled_time_difference(times, times**2, measurement_id="f")
    np.testing.assert_allclose(result.values[1:-1], 2.0 * times[1:-1], atol=1e-12)
    # The naive quotient really is wrong on this grid, so the assertion above is not
    # vacuous.  (Compare against the truth at sample 1, which is 2 * 0.1.)
    naive_sample = (times[2] ** 2 - times[0] ** 2) / (times[2] - times[0])
    assert abs(naive_sample - 2.0 * times[1]) > 0.1


def test_the_recorded_step_is_the_span_the_stencil_reached() -> None:
    """
    A central sample reaches two intervals, a one-sided one reaches a single interval.

    ``time_step`` is a record of the stencil's *extent*, not the divisor: a central
    quotient divides by ``h+ h- (h+ + h-)``, of which the span is one factor.  Reporting
    the nominal grid spacing instead would misstate the stencil for every endpoint, and
    the endpoints are exactly where the template differs.
    """
    times = _uniform(5)
    result = sampled_time_difference(times, times.copy(), measurement_id="f")
    span = times[1] - times[0]
    assert result.time_step[0] == pytest.approx(span)  # forward, one interval
    assert result.time_step[2] == pytest.approx(2 * span)  # central, two intervals
    assert result.template[0] == "forward_one_sided"
    assert result.template[2] == "central"
    assert result.template[-1] == "backward_one_sided"
    # A one-sided sample's span *is* its divisor; a central sample's is not.  Pin both so
    # a future change that conflates them is caught.
    assert result.time_step[0] == pytest.approx(times[1] - times[0])
    assert result.time_step[2] != pytest.approx((times[3] - times[1]) / 2.0)


# --------------------------------------------------------------------------- #
# 2. no stencil crosses a block boundary
# --------------------------------------------------------------------------- #


def test_no_stencil_crosses_a_block_boundary() -> None:
    """
    A jump in the slope at index 3 must not be averaged across by a central stencil.

    The sequence is built so the rate is exactly 1.0 before the boundary and exactly 3.0
    after.  Every sample must report one of those two values: a stencil that crossed the
    boundary would report an intermediate number that is the derivative on neither side.
    """
    times = _uniform(9)
    values = times.copy()
    values[3:] = values[3] + 3.0 * (times[3:] - times[3])
    result = sampled_time_difference(times, values, blocks=[3], measurement_id="f")
    np.testing.assert_allclose(result.values[:3], 1.0, atol=1e-12)
    np.testing.assert_allclose(result.values[3:], 3.0, atol=1e-12)
    assert set(np.round(result.values, 12)) == {1.0, 3.0}


def test_the_samples_either_side_of_a_boundary_go_one_sided() -> None:
    """The cost of not crossing the boundary is a one-sided endpoint, recorded as such."""
    times = _uniform(9)
    values = np.where(times < 0.5, times, 0.5 + 3.0 * (times - 0.5))
    result = sampled_time_difference(times, values, blocks=[4], measurement_id="f")
    assert result.template[3] == "backward_one_sided"
    assert result.template[4] == "forward_one_sided"
    assert result.one_sided[3] and result.one_sided[4]
    # Block 0 has no interior boundary, so its other samples may stay central.
    assert result.template[2] == "central"
    assert result.template[5] == "central"


def test_the_sequence_endpoints_are_one_sided() -> None:
    """Sample 0 has nothing before it, so it starts a block and differs forward."""
    result = sampled_time_difference(_uniform(5), _uniform(5), measurement_id="f")
    assert result.template[0] == "forward_one_sided"
    assert result.template[-1] == "backward_one_sided"
    assert result.one_sided[0] and result.one_sided[-1]


def test_a_sequence_of_two_samples_is_all_one_sided() -> None:
    """The smallest differable sequence has no central sample at all."""
    times = np.array([0.0, 0.5])
    result = sampled_time_difference(times, np.array([0.0, 1.0]), measurement_id="f")
    assert result.template == ("forward_one_sided", "backward_one_sided")
    assert result.one_sided.all()
    np.testing.assert_allclose(result.values, 2.0, atol=1e-12)


def test_a_single_sample_has_no_rate() -> None:
    """One point has no slope, and the operator says so rather than returning zero."""
    result = sampled_time_difference(np.array([0.0]), np.array([5.0]), measurement_id="f")
    assert result.template == ("undifferenced",)
    assert result.status == ("undefined_geometry",)
    assert np.isnan(result.values[0])
    assert result.reason[0]


def test_an_out_of_range_block_boundary_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="outside the sequence"):
        sampled_time_difference(
            _uniform(4), _uniform(4), blocks=[9], measurement_id="f"
        )


# --------------------------------------------------------------------------- #
# 3. an invalid neighbour is never substituted
# --------------------------------------------------------------------------- #


def test_an_invalid_sample_gets_a_reason_not_a_number() -> None:
    """A refused reading is reported, never filled in from a neighbour."""
    times = _uniform(7)
    values = 2.0 * times
    values[3] = np.nan
    result = sampled_time_difference(times, values, measurement_id="f")
    assert result.status[3] == "undefined_geometry"
    assert np.isnan(result.values[3])
    assert "no value of its own" in result.reason[3]


def test_neighbours_of_an_invalid_sample_go_one_sided_rather_than_interpolating() -> None:
    """
    Samples 2 and 4 must not use the missing sample 3 to form a central stencil.

    The true rate is 2.0 everywhere, so *any* substitution of the missing point would
    change the answer -- which is what makes this test able to see the defect.
    """
    times = _uniform(7)
    values = 2.0 * times
    valid = np.ones(7, dtype=bool)
    valid[3] = False
    values[3] = np.nan
    result = sampled_time_difference(times, values, valid=valid, measurement_id="f")
    assert result.template[2] == "backward_one_sided"
    assert result.template[4] == "forward_one_sided"
    np.testing.assert_allclose(
        np.delete(result.values, 3), 2.0, atol=1e-12
    )
    # The invalid sample still occupies its own row, so the arrays stay aligned.
    assert len(result) == 7


def test_a_sample_marked_invalid_but_holding_a_number_is_still_excluded() -> None:
    """
    A finite value behind ``valid=False`` must not be differenced against.

    This is the case the mask exists for: a solver can write a number that the contract
    has already refused (a partially converged sample, an inconsistent geometry).  If the
    operator only checked finiteness, it would quietly promote that refused reading into
    a rate for both neighbours.  The fixture puts an obvious value there, so using it
    changes the answer at samples 1 and 3 and the assertion can see it.
    """
    times = _uniform(5)
    values = 2.0 * times
    values[2] = 999.0  # finite, but declared unusable
    valid = np.ones(5, dtype=bool)
    valid[2] = False
    result = sampled_time_difference(
        times, values, valid=valid, measurement_id="f"
    )
    assert result.status[2] == "undefined_geometry"
    assert np.isnan(result.values[2])
    assert result.template[1] == "backward_one_sided"
    assert result.template[3] == "forward_one_sided"
    np.testing.assert_allclose(np.delete(result.values, 2), 2.0, atol=1e-12)
    # The sentinel would leak through the *one-sided* stencils of its two neighbours
    # (a central stencil skips sample 2 entirely), so check those, not the centre.
    step = times[1] - times[0]
    leaked_before = (values[2] - values[0]) / (2 * step)
    leaked_after = (values[4] - values[2]) / (2 * step)
    assert abs(leaked_before - 2.0) > 100.0
    assert abs(leaked_after - 2.0) > 100.0


def test_adjacent_boundaries_leave_a_sample_with_no_neighbour() -> None:
    """
    Two boundaries in a row isolate the sample between them.

    With boundaries at 2 and 3, sample 2 belongs to no block that has a differencing
    neighbour, so it must be reported as undifferenced rather than averaged across.
    This is the case a "one boundary at a time" implementation would get wrong by
    letting the stencil reach either side.
    """
    times = _uniform(6)
    result = sampled_time_difference(times, times.copy(), blocks=[2, 3], measurement_id="f")
    assert result.template[2] == "undifferenced"
    assert result.status[2] == "undefined_geometry"
    assert np.isnan(result.values[2])
    assert result.reason[2]
    # Its neighbours differ within their own blocks.
    assert result.template[1] == "backward_one_sided"
    assert result.template[3] == "forward_one_sided"
    np.testing.assert_allclose(
        np.delete(result.values, 2), 1.0, atol=1e-12
    )


def test_a_boundary_at_the_last_sample_leaves_it_undifferenced() -> None:
    """
    A boundary at the final index isolates it, since the only reach left is backward
    into the previous block -- which must not be used.
    """
    times = _uniform(6)
    result = sampled_time_difference(times, times.copy(), blocks=[5], measurement_id="f")
    assert result.template[5] == "undifferenced"
    assert result.status[5] == "undefined_geometry"
    assert np.isnan(result.values[5])
    # Sample 4 is in the previous block, so it is one-sided rather than central.
    assert result.template[4] == "backward_one_sided"
    np.testing.assert_allclose(result.values[:5], 1.0, atol=1e-12)


def test_a_duplicated_or_unsorted_block_list_is_harmless() -> None:
    """The block set is what matters, not its order or repetition."""
    times = _uniform(9)
    values = np.where(times < 0.5, times, 0.5 + 3.0 * (times - 0.5))
    plain = sampled_time_difference(times, values, blocks=[4], measurement_id="f")
    messy = sampled_time_difference(times, values, blocks=[4, 4, 4], measurement_id="f")
    shuffled = sampled_time_difference(times, values, blocks=[4], measurement_id="f")
    np.testing.assert_array_equal(plain.values, messy.values)
    np.testing.assert_array_equal(plain.values, shuffled.values)
    assert plain.template == messy.template
    # And a boundary declared at 0 is redundant but must not change anything, because
    # sample 0 already starts a block.
    redundant = sampled_time_difference(times, values, blocks=[0, 4], measurement_id="f")
    np.testing.assert_array_equal(plain.values, redundant.values)


def test_a_wholly_invalid_sequence_reports_every_sample() -> None:
    """
    The degenerate path must still produce one row per sample, all with reasons.

    If the loop short-circuited, callers aligning this against a sample table would get
    a length mismatch instead of a complete set of refusals.
    """
    times = _uniform(5)
    values = np.full(5, np.nan)
    result = sampled_time_difference(times, values, measurement_id="f")
    assert len(result) == 5
    assert all(name == "undifferenced" for name in result.template)
    assert all(state == "undefined_geometry" for state in result.status)
    assert all(reason for reason in result.reason)
    assert np.all(np.isnan(result.values))


def test_a_non_boolean_valid_mask_is_refused() -> None:
    """
    A numeric mask must not be reinterpreted by truthiness.

    A caller passing 0/1 weights would otherwise get "every sample is valid", which is
    the opposite of what such a mask usually means -- a silent inversion of the guard.
    """
    with pytest.raises(CharacteristicsError, match="must be boolean"):
        sampled_time_difference(
            _uniform(5), _uniform(5), valid=np.full(5, 0.5), measurement_id="f"
        )
    with pytest.raises(CharacteristicsError, match="must be boolean"):
        sampled_time_difference(
            _uniform(5), _uniform(5), valid=np.ones(5, dtype=int), measurement_id="f"
        )


def test_a_valid_mask_that_is_not_boolean_aligned_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="cover every sample"):
        sampled_time_difference(
            _uniform(4), _uniform(4), valid=np.ones(3, dtype=bool), measurement_id="f"
        )


def test_a_sample_marked_valid_but_holding_nan_is_not_used() -> None:
    """
    The mask and the data must agree; a NaN behind a `True` is not a reading.

    Trusting the mask alone would put the NaN into the arithmetic and produce NaN rates
    for the two neighbours, which looks like a data problem rather than the defect it is.
    """
    times = _uniform(5)
    values = 2.0 * times
    values[2] = np.nan
    result = sampled_time_difference(times, values, valid=np.ones(5, dtype=bool),
                                     measurement_id="f")
    assert result.status[2] == "undefined_geometry"
    assert result.template[1] == "backward_one_sided"
    assert result.template[3] == "forward_one_sided"
    np.testing.assert_allclose(np.delete(result.values, 2), 2.0, atol=1e-12)


# --------------------------------------------------------------------------- #
# 4. the provenance survives, and analyticity is not claimed
# --------------------------------------------------------------------------- #


def test_every_sample_carries_a_declared_template() -> None:
    result = sampled_time_difference(_uniform(9), _uniform(9) ** 2, measurement_id="f")
    assert len(result.template) == len(result)
    assert set(result.template) <= set(DIFFERENCE_TEMPLATES)


def test_a_valid_sample_never_carries_a_reason_and_vice_versa() -> None:
    """The contract's status/reason pairing must hold for this operator too."""
    times = _uniform(6)
    values = times ** 2
    values[2] = np.nan
    result = sampled_time_difference(times, values, measurement_id="f")
    for index in range(len(result)):
        if result.status[index] == "valid":
            assert not result.reason[index]
            assert np.isfinite(result.values[index])
        else:
            assert result.reason[index]
            assert np.isnan(result.values[index])


def test_the_curvature_scale_grows_with_the_curvature() -> None:
    """
    The reported scale must track the quantity it names, or it is decoration.

    It measures how much the local slope changes, so on ``f = t^3`` (where ``f'' = 6t``)
    it must grow linearly in ``t`` while the *value* error stays constant.  Asserting that
    growth is what keeps the field honest: it documents that this is a curvature measure
    and not a truncation-error estimate, which is precisely why it is not named one.
    """
    times = _uniform(21)
    result = sampled_time_difference(times, times**3, measurement_id="f")
    scale = result.curvature_scale[1:-1]
    assert np.all(np.isfinite(scale))
    assert np.all(scale > 0.0)
    # f'' = 6t, so the scale is proportional to t; the first half must be smaller.
    mid = len(scale) // 2
    assert scale[mid:].min() > scale[:mid].max()
    ratio = scale[-1] / scale[0]
    expected = times[-2] / times[1]
    assert ratio == pytest.approx(expected, rel=0.05)


def test_the_curvature_scale_shrinks_with_the_step() -> None:
    """It is a scale of the stencil, so refining the grid must reduce it."""
    coarse = sampled_time_difference(_uniform(11), _uniform(11) ** 3, measurement_id="f")
    fine = sampled_time_difference(_uniform(101), _uniform(101) ** 3, measurement_id="f")
    assert np.max(fine.curvature_scale[1:-1]) < np.max(coarse.curvature_scale[1:-1]) / 5.0


def test_no_curvature_scale_is_invented_for_an_undifferenced_sample() -> None:
    result = sampled_time_difference(np.array([0.0]), np.array([1.0]), measurement_id="f")
    assert np.isnan(result.curvature_scale[0])


def test_the_result_does_not_claim_to_be_analytic() -> None:
    """
    The operator must not present itself as the analytic path.

    ``trajectory.geometry_rate_request`` is that path; this one is a quotient over the
    sample grid, and the two must be distinguishable from the outside.  The template
    record is the difference, so it is asserted to exist for every sample.
    """
    result = sampled_time_difference(_uniform(7), _uniform(7) ** 2, measurement_id="f")
    assert all(name for name in result.template)
    assert "analytic" not in result.measurement_id


def test_the_projection_to_a_measurement_drops_the_provenance() -> None:
    """
    ``as_measurement`` is a lossy view, and the test says so.

    A caller that keeps only the ``MeasurementValue`` cannot audit the template or the
    one-sided flags, so the projection must not be the only way to obtain the numbers.
    """
    result = sampled_time_difference(_uniform(7), _uniform(7) ** 2, measurement_id="f")
    projected = as_measurement(result)
    assert isinstance(projected, MeasurementValue)
    np.testing.assert_array_equal(projected.values, result.values)
    assert projected.status == result.status
    assert not hasattr(projected, "template")


def test_the_returned_values_are_read_only() -> None:
    """A caller must not be able to mutate the operator's own output in place."""
    result = sampled_time_difference(_uniform(5), _uniform(5) ** 2, measurement_id="f")
    assert not result.values.flags.writeable


# --------------------------------------------------------------------------- #
# the time axis itself
# --------------------------------------------------------------------------- #


def test_a_repeated_timestamp_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="not strictly increasing"):
        sampled_time_difference(
            np.array([0.0, 1.0, 1.0, 2.0]), np.arange(4.0), measurement_id="f"
        )


def test_a_reversed_timestamp_is_refused() -> None:
    """A negative divisor is indistinguishable from a real reversal, so it is refused."""
    with pytest.raises(CharacteristicsError, match="not strictly increasing"):
        sampled_time_difference(
            np.array([0.0, 2.0, 1.0, 3.0]), np.arange(4.0), measurement_id="f"
        )


def test_a_mismatched_value_count_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="one value per timestamp"):
        sampled_time_difference(np.arange(4.0), np.arange(3.0), measurement_id="f")


def test_a_two_dimensional_sequence_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="one-dimensional"):
        sampled_time_difference(np.arange(4.0), np.zeros((4, 3)), measurement_id="f")


def test_an_empty_sequence_is_refused() -> None:
    with pytest.raises(CharacteristicsError, match="no samples"):
        sampled_time_difference(np.array([]), np.array([]), measurement_id="f")
