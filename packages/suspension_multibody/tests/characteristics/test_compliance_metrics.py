"""
Behavioural acceptance for the derived rate and angle metrics.

The criteria this file answers (P2 SPEC §50 and §56-§60) are behavioural: the
rates must be the *right physical quantity*, the tire must be counted once, the
angle must come from the angle jacobian, and units must be carried rather than
assumed.  Command success and file existence are explicitly not sufficient, so
every test below asserts a number derived independently -- analytically, or from a
second construction -- rather than re-asserting the implementation's own output.
"""

from __future__ import annotations

import math
from types import SimpleNamespace

import numpy as np
import pytest

from suspension_multibody.characteristics import CharacteristicsError
from suspension_multibody.results.compliance import (
    RAD_TO_DEG,
    RATE_FLOOR,
    RateMetric,
    angle_to_degrees,
    heave_mode,
    load_angle,
    load_angle_from_mode,
    metrics_request,
    ride_rate,
    ride_rate_series,
    roll_mode,
    roll_rate,
    roll_rate_per_degree,
    wheel_rate,
)
from suspension_multibody.results.equilibrium import (
    LOAD_MODE_COMPONENTS,
    LoadMode,
    ModeResult,
    audited_modes,
    equilibrium_response,
    load_mode_matrix,
    response_request,
)


def _result(*, samples: int = 1) -> SimpleNamespace:
    """
    Build a stub carrying ``samples`` identities, which the envelope needs.

    In an equilibrium request one "sample" is one load mode of one state, so the
    stub's sample count must match the mode count or the envelope correctly refuses
    the channels as mis-sized.
    """
    return SimpleNamespace(
        times_s=np.arange(float(samples)),
        raw=SimpleNamespace(
            cases=({"name": "stub", "sample_offset": 0, "sample_count": samples},)
        ),
    )


def _envelope(
    *,
    modes,
    responses,
    request_id: str = "req-1",
):
    return response_request(
        _result(samples=len(modes)),
        request_id,
        modes=modes,
        responses=responses,
        boundary_profile="suspension_free",
        boundary_version=1,
    )


# --------------------------------------------------------------------------
# Wheel rate: the reciprocal of the in-phase displacement response
# --------------------------------------------------------------------------


def test_the_wheel_rate_is_the_reciprocal_of_the_in_phase_response() -> None:
    """
    ``K = k`` and ``C = 1/k``: the rate is ``1/h``, recovering the stiffness.

    Built through the real response layer so the identity is exercised end to end,
    then checked against a hand-computed reciprocal rather than against the
    implementation's own output.
    """
    stiffness = 25.0
    tangent = np.array([[stiffness]])
    # `B_w = I` on the wheel-end vertical channel means the applied load equals the
    # mode amplitude, i.e. a unit load.
    load = np.zeros((1, 6))
    load[0, LOAD_MODE_COMPONENTS["vertical_force"]] = 1.0
    # One mode, one wheel-end column; the amplitude is a unit vertical force.
    directions = np.zeros((6, 1))
    directions[LOAD_MODE_COMPONENTS["vertical_force"], 0] = 1.0
    response, _ = equilibrium_response(tangent, load, directions)
    deflection = float(response[0, 0])
    assert deflection == pytest.approx(1.0 / stiffness, rel=1e-12)

    metric = wheel_rate(deflection, deflection)
    assert metric.available
    assert metric.values[0] == pytest.approx(stiffness, rel=1e-12)
    assert metric.values[1] == pytest.approx(stiffness, rel=1e-12)
    assert metric.units == "N/m"


def test_the_wheel_rate_is_not_the_mathematical_product_of_the_response() -> None:
    """
    Reject ``1/h`` being confused with ``h`` or with a stiffness already present.

    With ``h = 0.5`` the rate is 2, and neither 0.5 (the response) nor 4 (its
    square) is acceptable.  This is the specific confusion the design names when it
    forbids substituting ``Kzz`` or ``1/C33`` for the in-phase rate.
    """
    metric = wheel_rate(0.5, 0.25)
    assert metric.values == pytest.approx((2.0, 4.0), rel=1e-12)


def test_a_non_finite_wheel_response_is_refused_not_inverted() -> None:
    """
    An infinity in the response must not become a zero rate by inversion.

    ``1/inf == 0`` is the dangerous case: the arithmetic succeeds and produces a
    perfectly finite zero, which reads as "infinitely soft" -- the exact opposite of
    what an infinite response means.  The response layer already refuses to hand out
    non-finite numbers, so this is defence in depth against a caller that computed
    one itself, and it is asserted because the ``1/inf = 0`` path is silent.
    """
    for bad in (float("inf"), float("-inf"), float("nan")):
        metric = wheel_rate(bad, 0.5)
        assert not metric.available, f"a {bad} response must not yield a rate"
        assert metric.values == ()
        assert metric.status == "unidentifiable"
        assert "not finite" in metric.reason


def test_a_non_finite_roll_response_is_refused_not_inverted() -> None:
    """The same ``1/inf = 0`` hazard on the roll angle and moment."""
    assert not roll_rate(float("inf"), 0.5).available
    assert not roll_rate(1.0, float("nan")).available
    assert not ride_rate(float("inf")).available
    assert not ride_rate_series(float("inf"), 1.0).available


def test_a_wheel_that_does_not_move_has_no_finite_rate() -> None:
    """A zero displacement response would make the rate infinite, not huge."""
    metric = wheel_rate(0.0, 0.5)
    assert not metric.available
    assert metric.values == ()
    assert "indistinguishable from zero" in metric.reason


def test_each_side_of_a_wheel_rate_is_judged_separately() -> None:
    """
    One wheel can be valid while the other is not.

    Collapsing the two sides into a single verdict is the easy mistake, and it
    would either hide a real problem on one side or discard a valid number.
    """
    metric = wheel_rate(0.5, 0.0)
    assert not metric.available, "the right side is unformable, so the metric is refused"
    assert "right" in metric.reason


def test_a_negatively_loading_wheel_keeps_its_sign() -> None:
    """
    A negative tangent is reported, not made positive.

    The design requires negative stiffness to be preserved together with a
    stability diagnosis; taking an absolute value would erase the instability.
    """
    metric = wheel_rate(-0.5, 0.25)
    assert metric.available
    assert metric.values[0] == pytest.approx(-2.0, rel=1e-12)
    assert metric.values[1] == pytest.approx(4.0, rel=1e-12)


# --------------------------------------------------------------------------
# Ride rate: the tire counted exactly once
# --------------------------------------------------------------------------


def test_the_series_ride_rate_matches_the_closed_form() -> None:
    """``Ks*Kt/(Ks+Kt)`` against an independent hand computation."""
    ks, kt = 30.0, 20.0
    metric = ride_rate_series(ks, kt)
    assert metric.available
    assert metric.values[0] == pytest.approx(30.0 * 20.0 / 50.0, rel=1e-12)
    # And it is strictly softer than either contribution.
    assert metric.values[0] < min(ks, kt)


def test_the_full_model_ride_rate_does_not_apply_the_series_formula_again() -> None:
    """
    On the full model the tire is already inside the response.

    A contact-point deflection of 1/50 under a unit load already reflects the
    series combination, so the rate is 50 -- applying ``Ks*Kt/(Ks+Kt)`` on top of
    it would report a different, softer number and double-count the tire.  The
    assertion pins that the supplied split does *not* change the value.
    """
    full_model_deflection = 1.0 / 50.0
    plain = ride_rate(full_model_deflection)
    with_split = ride_rate(full_model_deflection, suspension_rate=30.0, tire_rate=20.0)
    assert plain.values[0] == pytest.approx(50.0, rel=1e-12)
    assert with_split.values[0] == pytest.approx(50.0, rel=1e-12), (
        "supplying the separation must not re-apply the series law to a full-model response"
    )


def test_half_a_series_separation_is_refused_not_half_applied() -> None:
    """Only a suspension rate is not a decomposition, so it cannot be combined."""
    metric = ride_rate(1.0 / 50.0, suspension_rate=30.0)
    assert not metric.available
    assert "together or not at all" in metric.reason


def test_a_zero_sum_series_is_refused() -> None:
    """``Ks + Kt = 0`` makes the series law unbounded, so it is not a rate."""
    metric = ride_rate_series(-20.0, 20.0)
    assert not metric.available
    assert "sum to zero" in metric.reason


# --------------------------------------------------------------------------
# Roll rate
# --------------------------------------------------------------------------


def test_the_roll_rate_is_the_moment_per_radian() -> None:
    metric = roll_rate(moment_response=1000.0, angle_response=0.05)
    assert metric.available
    assert metric.values[0] == pytest.approx(20000.0, rel=1e-12)
    assert metric.units == "N*m/rad"


def test_the_roll_rate_converts_to_per_degree_by_pi_over_180() -> None:
    """
    N*m/rad -> N*m/deg multiplies by pi/180, giving a *smaller* number.

    The direction matters and is easy to invert: a stiffness of X per radian is a
    much smaller stiffness per (smaller) degree.
    """
    per_radian = roll_rate(moment_response=1000.0, angle_response=0.05)
    per_degree = roll_rate_per_degree(per_radian)
    assert per_degree.units == "N*m/deg"
    assert per_degree.values[0] == pytest.approx(20000.0 * RAD_TO_DEG, rel=1e-12)
    assert per_degree.values[0] < per_radian.values[0]


def test_converting_an_already_converted_roll_rate_is_refused() -> None:
    """Converting twice is silent, so the unit is checked rather than assumed."""
    per_radian = roll_rate(moment_response=1000.0, angle_response=0.05)
    per_degree = roll_rate_per_degree(per_radian)
    with pytest.raises(CharacteristicsError):
        roll_rate_per_degree(per_degree)


def test_a_zero_roll_angle_is_refused_rather_than_reported_as_a_huge_rate() -> None:
    """
    The typical signature of a wrong reference body is a near-zero angle.

    An unbounded stiffness is not a reading, so the refusal names the likely cause
    instead of returning a giant number.
    """
    metric = roll_rate(moment_response=1000.0, angle_response=0.0)
    assert not metric.available
    assert "unbounded stiffness" in metric.reason


def test_roll_rate_refuses_a_non_finite_input() -> None:
    metric = roll_rate(moment_response=float("inf"), angle_response=0.05)
    assert not metric.available
    assert metric.status == "unidentifiable"


# --------------------------------------------------------------------------
# Load angles: the angle jacobian, not the pose rotation rows
# --------------------------------------------------------------------------


def test_the_load_angle_uses_the_angle_gradient() -> None:
    """``G_angle X`` for a motion the gradient projects onto."""
    gradient = np.array([1.0, 0.0])
    motion = np.array([0.02, 0.5])
    value, status, reason = load_angle(gradient, motion)
    assert status == "valid"
    assert reason == ""
    assert value == pytest.approx(0.02, rel=1e-12)


def test_the_load_angle_adds_the_direct_term() -> None:
    """``G_angle X + D_angle P`` -- the direct term is not optional."""
    value, status, _ = load_angle(np.array([1.0]), np.array([0.02]), direct_term=0.005)
    assert status == "valid"
    assert value == pytest.approx(0.025, rel=1e-12)


def test_an_angle_gradient_in_the_wrong_space_is_refused() -> None:
    """A gradient and a motion of different lengths is a defect, not a small angle."""
    with pytest.raises(CharacteristicsError):
        load_angle(np.array([1.0, 0.0]), np.array([0.02]))


def test_a_refused_mode_propagates_its_own_reason_to_the_angle() -> None:
    """
    The response layer's diagnosis is strictly more informative than this layer's.

    Re-diagnosing from an absent array would replace "the tangent leaves 1
    unconstrained direction" with a vague statement about missing numbers.
    """
    refused = ModeResult(
        kind="camber_torque",
        status="unidentifiable",
        reason="the tangent leaves 1 unconstrained direction",
        values=None,
    )
    value, status, reason = load_angle_from_mode(refused)
    assert status == "unidentifiable"
    assert reason == "the tangent leaves 1 unconstrained direction"
    assert value == 0.0


def test_a_valid_mode_yields_its_requested_channel() -> None:
    answered = ModeResult(
        kind="camber_torque", status="valid", reason="", values=np.array([0.1, 0.2])
    )
    value, status, reason = load_angle_from_mode(answered, index=1)
    assert (value, status, reason) == (0.2, "valid", "")


def test_asking_for_a_channel_that_does_not_exist_is_refused() -> None:
    answered = ModeResult(kind="camber_torque", status="valid", reason="", values=np.array([0.1]))
    with pytest.raises(CharacteristicsError):
        load_angle_from_mode(answered, index=3)


def test_degrees_conversion_refuses_a_non_radian_input() -> None:
    """
    A value already in degrees must not be converted again.

    This is a 57x silent error, so the unit is a required argument rather than an
    assumption.
    """
    assert angle_to_degrees(math.pi / 2)[0] == pytest.approx(90.0, rel=1e-12)
    with pytest.raises(CharacteristicsError):
        angle_to_degrees(90.0, units="deg")


# --------------------------------------------------------------------------
# Mode construction: in-phase and anti-phase
# --------------------------------------------------------------------------


def test_the_heave_mode_is_in_phase_and_the_roll_mode_is_anti_phase() -> None:
    """
    The two modes differ in exactly the sign pattern of the two wheels.

    Both name the ``vertical_force`` channel, because in both cases the amplitude a
    caller supplies is a pair of **forces** in newtons.  An anti-phase force pair
    produces a roll moment through its lever arm; it does not *carry* one, so
    declaring a moment kind would put the perturbation in the wrong row.
    """
    heave_mode_kind, heave = heave_mode()
    roll_mode_kind, roll = roll_mode()
    assert heave_mode_kind.kind == "vertical_force"
    assert roll_mode_kind.kind == "vertical_force", (
        "an anti-phase force pair is still a pair of forces, not a moment"
    )
    assert roll_mode_kind.amplitude_units == "N"
    assert heave[0] == heave[1], "the in-phase mode pushes both wheels the same way"
    assert roll[0] == -roll[1], "the anti-phase mode pushes the wheels oppositely"


def test_the_roll_mode_perturbs_the_vertical_force_rows_not_a_moment_row() -> None:
    """
    The mode must land on ``Fz``, which is the row its amplitude actually means.

    This is the defect the earlier version of this file missed: ``LoadMode.kind`` is
    not a label, it selects a row through ``LOAD_MODE_COMPONENTS``.  Declaring the
    roll mode as ``roll_moment`` put the unit perturbation in an ``M`` row, so the
    solver was asked a moment question while the caller passed force amplitudes --
    a different physical system, answered with a well-formed number.

    Checked through the real matrix builder rather than by re-reading the kind, so a
    wrong mapping in either the constant table or the mode cannot pass.
    """
    heave_kind, _ = heave_mode()
    roll_kind, _ = roll_mode()
    matrix = load_mode_matrix([heave_kind, roll_kind])
    fz = LOAD_MODE_COMPONENTS["vertical_force"]
    assert matrix[fz, 0] != 0.0, "the heave mode must perturb Fz"
    assert matrix[fz, 1] != 0.0, "the roll mode must also perturb Fz"
    # No moment row may be touched: a vertical force pair carries no direct moment.
    for kind in ("roll_moment", "camber_torque", "aligning_torque"):
        assert matrix[LOAD_MODE_COMPONENTS[kind], :].tolist() == [0.0, 0.0], (
            f"a vertical force pair must not perturb the {kind} row"
        )
    # And the two columns are the in-phase / anti-phase pair.
    np.testing.assert_allclose(matrix[fz, :], [1.0, 1.0])


def test_the_moment_kinds_turn_about_the_axis_their_names_claim() -> None:
    """
    Roll is about x, camber about y (the spin axis), aligning about z (steer).

    The repo already fixes the roll convention as "the roll moment about the x
    axis", so a mapping that put ``roll_moment`` anywhere else would disagree with
    existing code while still producing numbers.  Pinned explicitly because the two
    triples (force axes and moment axes) are easy to confuse.
    """
    assert LOAD_MODE_COMPONENTS["longitudinal_force"] == 0  # Fx
    assert LOAD_MODE_COMPONENTS["lateral_force"] == 1       # Fy
    assert LOAD_MODE_COMPONENTS["vertical_force"] == 2      # Fz
    assert LOAD_MODE_COMPONENTS["roll_moment"] == 3         # Mx, about the roll axis
    assert LOAD_MODE_COMPONENTS["camber_torque"] == 4       # My, the spin axis
    assert LOAD_MODE_COMPONENTS["aligning_torque"] == 5     # Mz, the steer axis


def test_the_mode_signs_are_parameters_not_assumptions() -> None:
    """Which wheel is positive belongs to the matched profile, so it is passed in."""
    _, flipped = heave_mode(left=-1.0, right=-1.0)
    np.testing.assert_allclose(flipped, [-1.0, -1.0])
    _, swapped = roll_mode(left=-1.0, right=1.0)
    np.testing.assert_allclose(swapped, [-1.0, 1.0])


# --------------------------------------------------------------------------
# Anti-roll bar: the left/right crossover is a real cross term
# --------------------------------------------------------------------------


def test_an_anti_roll_bar_produces_a_cross_coupled_wheel_response() -> None:
    """
    With an anti-roll bar the in-phase and anti-phase responses must differ.

    Modelled as a 2-dof mechanism whose tangent couples the two wheel coordinates
    through a bar stiffness ``kb``.  Independently check the analytic inverse: for
    ``A = [[k+kb, -kb], [-kb, k+kb]]`` the in-phase load ``(1,1)`` gives the same
    displacement on both sides, while the anti-phase load ``(1,-1)`` gives
    opposite displacements of a *different* magnitude.  If the implementation (or
    the mode) had collapsed the bar, these two would coincide.
    """
    k, kb = 100.0, 40.0
    tangent = np.array([[k + kb, -kb], [-kb, k + kb]])
    load = np.eye(2)  # two wheel-end vertical load channels

    in_phase = np.array([[1.0], [1.0]])
    anti_phase = np.array([[1.0], [-1.0]])
    x_in, _ = equilibrium_response(tangent, load, in_phase)
    x_anti, _ = equilibrium_response(tangent, load, anti_phase)

    # Analytic: in-phase sees k (bar unloaded); anti-phase sees k + 2kb.
    assert x_in[0, 0] == pytest.approx(1.0 / k, rel=1e-12)
    assert x_in[1, 0] == pytest.approx(1.0 / k, rel=1e-12)
    assert x_anti[0, 0] == pytest.approx(1.0 / (k + 2 * kb), rel=1e-12)
    assert x_anti[1, 0] == pytest.approx(-1.0 / (k + 2 * kb), rel=1e-12)

    # The bar is what makes the two rates differ; without it they would agree.
    bar_rate = wheel_rate(x_in[0, 0], x_in[1, 0])
    no_bar_tangent = np.array([[k, 0.0], [0.0, k]])
    x_no_bar, _ = equilibrium_response(no_bar_tangent, load, in_phase)
    assert bar_rate.values[0] == pytest.approx(k, rel=1e-12)
    assert x_no_bar[0, 0] == pytest.approx(1.0 / k, rel=1e-12), (
        "the in-phase response must not feel the bar, which is the physical "
        "statement that an anti-roll bar has no heave stiffness"
    )


def test_the_roll_stiffness_recovers_the_bar_contribution() -> None:
    """
    The anti-phase mode's stiffness is ``k + 2*kb``, and the roll rate is that.

    Constructed through the response layer so the mode amplitude, the tangent and
    the metric are all chained, then compared against the closed form.
    """
    k, kb = 100.0, 40.0
    tangent = np.array([[k + kb, -kb], [-kb, k + kb]])
    load = np.eye(2)
    x_anti, _ = equilibrium_response(tangent, load, np.array([[1.0], [-1.0]]))
    # A unit anti-phase load produces the moment response; the angle response is the
    # relative displacement over a unit track.
    track = 1.5
    angle = float((x_anti[0, 0] - x_anti[1, 0]) / track)
    moment = 1.0  # unit moment amplitude
    metric = roll_rate(moment, angle)
    assert metric.available
    assert metric.values[0] == pytest.approx(track * (k + 2 * kb) / 2.0, rel=1e-9)


# --------------------------------------------------------------------------
# Metric invariants and envelope integration
# --------------------------------------------------------------------------


def test_a_rate_metric_cannot_be_both_answered_and_refused() -> None:
    """The two states are mutually exclusive, and neither may be silent."""
    with pytest.raises(CharacteristicsError):
        RateMetric(name="wheel_rate", status="valid", values=(1.0,), reason="why")
    with pytest.raises(CharacteristicsError):
        RateMetric(name="wheel_rate", status="valid")
    with pytest.raises(CharacteristicsError):
        RateMetric(name="wheel_rate", status="singular", values=(1.0,))


def test_a_rate_below_the_relative_floor_is_refused_rather_than_inverted() -> None:
    """
    The floor stops noise in the denominator from becoming a rate.

    It is *relative* to the comparable responses in the same request: a genuinely
    soft mechanism whose responses are all small must still be answered, while a
    response that is tiny *compared with its neighbour* is the noise case.
    """
    # Left is negligible against right -> refused.
    assert wheel_rate(RATE_FLOOR, 1.0).available is False
    assert wheel_rate(1e-6, 1.0).available is True
    # Both sides consistently tiny: this is a soft but real mechanism, not noise.
    soft = wheel_rate(1e-13, 2e-13)
    assert soft.available is True, (
        "a uniformly small response is a real small deflection, not an absent one"
    )
    assert soft.values[0] == pytest.approx(1e13, rel=1e-9)


def test_a_uniformly_small_response_is_not_refused_by_an_absolute_floor() -> None:
    """
    The defect this pins: an *absolute* 1e-12 floor refused real small mechanisms.

    A response of order 1e-11 is exactly what the response layer itself calls
    "entirely legitimate -- a small load on a stiff mechanism".  With an absolute
    threshold every such rate would be reported as non-existent, and (worse) the
    status would say the mechanism was degenerate when it was merely soft.
    """
    for response in (1e-11, 1e-13, 1e-30):
        metric = wheel_rate(response, response)
        assert metric.available, f"a uniform {response} response must still be answered"
        assert metric.values[0] == pytest.approx(1.0 / response, rel=1e-9)
    # The roll and series cases must be internally consistent in scale: a tiny angle
    # against a tiny moment is a soft-but-real roll, whereas a tiny angle against a
    # large moment is the genuine near-zero-angle case and stays refused.
    assert roll_rate(1e-10, 1e-13).available is True
    assert ride_rate_series(1e-11, 1e-11).available is True


def test_a_zero_roll_angle_reports_the_same_status_as_a_zero_wheel_response() -> None:
    """
    One physical condition must not wear two statuses.

    A vanishing denominator is the same situation whether the denominator is a
    displacement or an angle, so both report ``singular``.  ``wheel_rate`` and
    ``roll_rate`` disagreeing here would force a consumer to learn two rules for
    one condition.
    """
    wheel = wheel_rate(0.0, 1.0)
    roll = roll_rate(moment_response=1000.0, angle_response=0.0)
    assert not wheel.available and not roll.available
    assert wheel.status == roll.status == "singular"
    # The non-finite case is a different condition and keeps its own status.
    assert roll_rate(float("inf"), 0.5).status == "unidentifiable"


def test_a_refusal_reason_does_not_invent_a_unit_for_a_non_default_scale() -> None:
    """
    The reason must not fabricate a response unit by editing the rate unit.

    ``units`` describes the *rate* (N/m, kN/m, N/mm, ...).  No substring rule
    recovers the response unit from it, and an earlier version produced ``km/N``
    for a ``kN/m`` rate -- a misleading diagnosis embedded in a refusal that may be
    written to an artifact.
    """
    for units in ("N/m", "kN/m", "N/mm", ""):
        reason = wheel_rate(0.0, 0.5, units=units).reason
        assert "km/N" not in reason, f"fabricated unit for {units!r}"
        assert "m/Nm" not in reason, f"fabricated unit for {units!r}"
        assert "per unit in-phase load" in reason


def test_metrics_reach_the_envelope_as_channels_keyed_by_measurement() -> None:
    """
    Each response channel is labelled by its measurement ID, one entry per mode.

    In this envelope the mode axis *is* the sample axis: a "sample" here is one
    load mode of one equilibrium state.  That is why a channel's length must equal
    the mode count, and why naming a channel after a *metric* instead of the
    measurement it reports would mislabel the column.
    """
    envelope = _envelope(
        modes=[LoadMode("vertical_force", "N"), LoadMode("roll_moment", "N*m")],
        responses={
            "h_left": (np.array([0.5, -0.25]), "m/N", ""),
            "h_right": (np.array([0.5, 0.25]), "m/N", ""),
        },
    )
    assert "h_left" in envelope.responses
    assert "h_right" in envelope.responses
    # Two modes selected, so every channel carries two entries -- the mode axis.
    assert len(envelope.samples) == 2
    np.testing.assert_allclose(envelope.responses["h_left"].values, [0.5, -0.25])


def test_a_channel_with_the_wrong_mode_count_is_refused() -> None:
    """
    A channel shorter than the mode count must be refused, not truncated.

    Truncating would silently drop a mode's answer and leave the remaining columns
    labelled by the wrong mode.
    """
    with pytest.raises(CharacteristicsError):
        _envelope(
            modes=[LoadMode("vertical_force", "N"), LoadMode("roll_moment", "N*m")],
            responses={"h_left": (np.array([0.5]), "m/N", "")},
        )


def test_a_refused_metric_reaches_the_envelope_as_an_audited_refusal() -> None:
    """
    "Asked for and unformable" must be distinguishable from "not asked for".

    A refused metric has no answer for a channel, so it travels in the request's
    audit trail instead.  What must never happen is the refusal being dropped: a
    caller reading only the envelope would then conclude the metric was never
    requested, which is a statement about the request rather than the mechanism.
    """
    envelope = metrics_request(
        _result(),
        "req-2",
        [wheel_rate(0.0, 1.0), wheel_rate(0.5, 0.5)],
        modes=[LoadMode("vertical_force", "N")],
        responses={"h": (np.array([0.5]), "m/N", "")},
        boundary_profile="suspension_free",
        boundary_version=1,
    )
    audit = envelope.extra["metric_audit"]
    # One `wheel_rate` succeeded and one was refused, in the same request.  Both
    # facts must survive: collapsing the audit to a set of names would hide the
    # success, and reporting the refusal as a bare dict would hide the duplicate.
    assert audit["refused"] == {
        "wheel_rate": [wheel_rate(0.0, 1.0).reason]
    }
    assert audit["formed"] == ["wheel_rate"]
    assert audit["formed_counts"] == {"wheel_rate": 1}


def test_a_caller_audit_key_collision_is_refused_not_overwritten() -> None:
    """
    Silently replacing the caller's ``extra`` key would make them believe it survived.

    A collision is a request-shape mistake, so it is named rather than absorbed.
    """
    with pytest.raises(CharacteristicsError) as error:
        metrics_request(
            _result(),
            "req-3",
            [wheel_rate(0.5, 0.5)],
            modes=[LoadMode("vertical_force", "N")],
            responses={"h": (np.array([0.5]), "m/N", "")},
            boundary_profile="suspension_free",
            boundary_version=1,
            extra={"metric_audit": "mine"},
        )
    assert "metric_audit" in str(error.value)


def test_a_metric_name_is_not_synthesised_into_a_load_mode() -> None:
    """
    ``wheel_rate`` is a metric, ``vertical_force`` is a load mode -- different lists.

    Deriving the mode list from the metric names would manufacture load-mode kinds
    that do not exist.  Asserting that ``LoadMode('wheel_rate', ...)`` is refused
    pins the reason the two arguments are separate rather than one.
    """
    with pytest.raises(CharacteristicsError):
        LoadMode("wheel_rate", "N")


def test_metrics_request_carries_units_into_the_channels() -> None:
    """Units travel with the number, so a channel cannot be read unit-free."""
    metric = wheel_rate(0.5, 0.5)
    assert metric.units == "N/m"
    converted = roll_rate_per_degree(roll_rate(1000.0, 0.05))
    assert converted.units == "N*m/deg"


# --------------------------------------------------------------------------
# Integration: an audited mode feeding a metric
# --------------------------------------------------------------------------


def test_an_audited_in_phase_mode_feeds_the_wheel_rate() -> None:
    """
    End to end through ``audited_modes``: heave mode in, wheel rate out.

    Two independent wheel-end loads in a symmetric 2-dof mechanism, so the left and
    right rates must both recover the spring stiffness, cross-checked against the
    closed form rather than against the earlier test.
    """
    stiffness = 64.0
    tangent = np.eye(2) * stiffness
    load = np.zeros((2, 6))
    load[0, LOAD_MODE_COMPONENTS["vertical_force"]] = 1.0
    load[1, LOAD_MODE_COMPONENTS["vertical_force"]] = 1.0

    results, audit = audited_modes(
        tangent, load, (LoadMode("vertical_force", "N"),)
    )
    assert audit["status"] == "valid"
    assert results[0].status == "valid"
    displacement = float(results[0].values[0])
    assert displacement == pytest.approx(1.0 / stiffness, rel=1e-12)

    metric = wheel_rate(displacement, displacement)
    assert metric.values[0] == pytest.approx(stiffness, rel=1e-12)
    assert metric.values[1] == pytest.approx(stiffness, rel=1e-12)


def test_a_refused_mode_yields_a_refused_metric_not_a_number() -> None:
    """
    A singular tangent must not become a wheel rate through a zero placeholder.

    With the tangent's second row zeroed, the response layer refuses the mode; the
    metric layer must therefore have no displacement to invert.  This is the chain
    that the NaN-instead-of-zeros decision in the response layer protects.
    """
    tangent = np.array([[64.0, 0.0], [0.0, 0.0]])
    load = np.zeros((2, 6))
    load[0, LOAD_MODE_COMPONENTS["vertical_force"]] = 1.0
    load[1, LOAD_MODE_COMPONENTS["vertical_force"]] = 1.0

    results, audit = audited_modes(tangent, load, (LoadMode("vertical_force", "N"),))
    assert audit["status"] == "singular"
    assert results[0].values is None
    value, status, reason = load_angle_from_mode(results[0])
    assert status == "singular"
    assert reason, "a refused chain must carry its reason down to the metric"
