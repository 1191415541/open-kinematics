"""
Rate and angle metrics derived from a load-mode response.

This module turns named load modes into **physical rates** -- wheel rate, ride
rate, roll rate -- and into load-angle readings.  It is a pure layer: every
number it needs arrives as an already-computed response, and it never asks the
mechanism another question.  That separation matters because the metrics are
where a plausible-looking wrong number is easiest to produce (a wheel rate is a
*reciprocal*, so a tiny displacement response becomes a huge rate), and it is
also what lets the same response serve several metrics at once.

Two conventions are load-bearing and are stated once here rather than repeated:

* **A rate is ``1/h``, never ``K``.**  The design is explicit that a wheel rate
  must come from the named in-phase mode's displacement response, and must not be
  replaced by a port ``Kzz``, a single-wheel mode, or ``1/C33``.  Those are
  different physical questions that happen to share a name in some references.
* **The tire is counted exactly once.**  A wheel rate is the *suspension*
  quantity and excludes the tire.  A ride rate measured at the contact patch on
  the full model already contains the tire, so applying the series formula a
  second time would double-count it.  When the series decomposition cannot be
  justified the metrics are reported as unavailable, not silently simplified --
  see :func:`ride_rate`.

Angles are in radians throughout and only converted at the boundary, because
"is this number a rate or an angle?" is not recoverable from the value alone.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from ..characteristics.values import CharacteristicsError
from .equilibrium import LoadMode, ModeResult

#: The unit conversion the design pins for torque per angle: N*m/rad to
#: N*m/deg.  Written as ``pi/180`` following the design text so that a reader can
#: check it against the document directly.
RAD_TO_DEG = math.pi / 180.0

#: Relative floor for a response that stands in a denominator.  A rate is formed by
#: dividing by a response, so a response that is zero *to rounding* would turn noise
#: into an unbounded rate.
#:
#: It is **relative**, not absolute: the threshold for a quantity is
#: ``RATE_FLOOR * scale`` where ``scale`` is the largest comparable response in the
#: same request.  An absolute floor would refuse a legitimately soft mechanism -- a
#: response of order 1e-11 is a real small deflection, not an absent one -- and
#: reporting "no finite rate exists" for a perfectly finite large stiffness is a
#: wrong answer, not a conservative one.  This mirrors the relative comparison the
#: equilibrium response layer makes for the same reason.
RATE_FLOOR = 1e-12


def _scale(values: Sequence[float]) -> float:
    """
    Return the reference magnitude for a relative floor comparison.

    The maximum absolute value of the comparable responses, so that the smallest
    quantity in a request is judged against the largest one rather than against an
    absolute constant.  Falls back to 1.0 when every candidate is zero, which keeps
    the comparison meaningful (0 <= 0 still refuses).
    """
    magnitudes = [abs(float(value)) for value in values if math.isfinite(float(value))]
    if not magnitudes:
        return 1.0
    largest = max(magnitudes)
    return largest if largest > 0.0 else 1.0


def _negligible(value: float, scale: float) -> bool:
    """Whether ``value`` is indistinguishable from zero at ``scale``."""
    return abs(value) <= RATE_FLOOR * scale


def _flatten(values: Any, *, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.ndim != 1 or array.size == 0:
        raise CharacteristicsError(f"{name} must be a non-empty one-dimensional array")
    if not np.isfinite(array).all():
        raise CharacteristicsError(f"{name} must be finite")
    return array


def heave_mode(left: float = 1.0, right: float = 1.0) -> tuple[LoadMode, np.ndarray]:
    """
    Build the in-phase vertical load mode: both wheels pushed up equally.

    Returned as ``(mode, amplitude)`` where ``amplitude`` is the column the
    response layer multiplies by ``B_w``.  The amplitude is expressed in the
    wheel-end load basis ``[F_left, F_right]`` with the load axis fixed by the
    caller's ``B_w``; keeping it a two-vector rather than a hard-coded ``e3+e9``
    is what lets a non-12-port model use the same mode.
    """
    return (
        LoadMode("vertical_force", "N"),
        np.array([left, right], dtype=float),
    )


def roll_mode(left: float = 1.0, right: float = -1.0) -> tuple[LoadMode, np.ndarray]:
    """
    Build the anti-phase vertical load mode: the wheels pushed in opposite senses.

    The load kind is ``vertical_force``, **not** ``roll_moment``, because the
    amplitude vector is a pair of *forces* in newtons applied at the two wheel
    centres.  The anti-phase pair only produces a roll moment through its lever
    arm; declaring a moment kind here would claim a component the amplitude does
    not carry, and would put the unit perturbation in the ``Mx`` row instead of the
    two ``Fz`` rows -- solving a different physical system with a well-formed
    number as the answer.

    The signs are parameters because "which wheel is positive" is a sign
    convention the matched profile owns, not something this module may assume.
    """
    return (
        LoadMode("vertical_force", "N"),
        np.array([left, right], dtype=float),
    )


@dataclass(frozen=True)
class RateMetric:
    """
    A physical rate, or a named refusal explaining why it could not be formed.

    ``values`` is a tuple of per-channel numbers in ``units``; it is empty when
    ``status`` is not ``"valid"`` and ``reason`` says why.  The refusal carries no
    numbers for the same reason the response layer's refusals do not: a zero rate
    and an unformed rate must not be confused, and a rate is a reciprocal, so a
    placeholder zero would be maximally misleading.
    """

    name: str
    status: str
    reason: str = ""
    values: tuple[float, ...] = ()
    units: str = ""

    def __post_init__(self) -> None:
        if self.values and self.reason:
            raise CharacteristicsError(
                f"rate {self.name!r} cannot both carry values and a reason"
            )
        if not self.values and not self.reason:
            raise CharacteristicsError(
                f"rate {self.name!r} must carry either values or a reason"
            )
        if self.values and self.status != "valid":
            raise CharacteristicsError(
                f"rate {self.name!r} carries values but is not valid"
            )
        if self.values and not np.isfinite(np.asarray(self.values, dtype=float)).all():
            raise CharacteristicsError(f"rate {self.name!r} values must be finite")

    @property
    def available(self) -> bool:
        return self.status == "valid"


def _refused(name: str, reason: str, *, status: str = "singular") -> RateMetric:
    """
    Build a refusal for a rate that could not be formed.

    ``singular`` is the default because the dominant cause is a denominator that
    vanishes -- a zero displacement or angle response -- which is the same physical
    situation the response layer calls singular.  ``unidentifiable`` is passed
    explicitly for the non-finite cases, where the quantity exists but is not
    representable.  Both statuses are in the shared sample-status vocabulary, so a
    consumer switches on them the same way it does for a response.
    """
    return RateMetric(name=name, status=status, reason=reason)


def wheel_rate(
    left_response: Any,
    right_response: Any,
    *,
    units: str = "N/m",
) -> RateMetric:
    """
    Return the in-phase wheel rate from the two vertical displacement responses.

    ``h_left``/``h_right`` are displacements per unit in-phase load (m/N), so the
    rate is their reciprocal (N/m).  Each side is judged separately: one wheel can
    have a valid rate while the other does not, and collapsing them into a single
    verdict would hide that.
    """
    left = float(np.asarray(left_response, dtype=float).ravel()[0])
    right = float(np.asarray(right_response, dtype=float).ravel()[0])
    scale = _scale((left, right))
    for value, side in ((left, "left"), (right, "right")):
        if not math.isfinite(value):
            return _refused(
                "wheel_rate",
                f"the {side} wheel vertical response is not finite, so no rate exists",
                status="unidentifiable",
            )
        if _negligible(value, scale):
            # A zero displacement response means the wheel does not move under the
            # in-phase load.  The rate would be infinite, which is not a stiffness;
            # it usually means the mode is not compatible with the boundary or the
            # response was taken in the wrong units.
            #
            # The response unit is *not* derived from `units` by string surgery: the
            # caller-supplied token describes the rate (N/m, kN/m, ...) and no
            # substring rule recovers the response unit from it.  Guessing produced
            # ``km/N`` for ``kN/m`` before, which is a misleading diagnosis inside a
            # refusal.
            return _refused(
                "wheel_rate",
                f"the {side} wheel vertical response is {value:.3e} "
                "(a displacement per unit in-phase load), i.e. indistinguishable "
                "from zero; the in-phase mode does not move it, so no finite rate "
                "exists",
            )
    return RateMetric(
        name="wheel_rate",
        status="valid",
        values=(1.0 / left, 1.0 / right),
        units=units,
    )


def ride_rate_series(
    suspension_rate: float,
    tire_rate: float,
    *,
    units: str = "N/m",
) -> RateMetric:
    """
    Return the **series** ride rate from an already-separated suspension and tire rate.

    ``Ks*Kt/(Ks+Kt)``.  This is the local-help formula, and it is only valid when
    the two contributions have genuinely been identified separately on a matched
    sample.  It is therefore never called automatically by :func:`ride_rate`; a
    caller that cannot separate the contributions must not reach for it, because
    the alternative -- removing the tire and recomputing a "suspension" rate --
    answers a different physical question while wearing the same name.
    """
    if not (math.isfinite(suspension_rate) and math.isfinite(tire_rate)):
        return _refused(
            "ride_rate", "the suspension and tire rates must both be finite",
            status="unidentifiable",
        )
    total = suspension_rate + tire_rate
    if _negligible(total, _scale((suspension_rate, tire_rate))):
        return _refused(
            "ride_rate",
            "the suspension and tire rates sum to zero, so the series combination is "
            "unbounded; the two contributions are not both physical in this configuration",
        )
    return RateMetric(
        name="ride_rate",
        status="valid",
        values=(suspension_rate * tire_rate / total,),
        units=units,
    )


def ride_rate(
    contact_point_response: float,
    *,
    suspension_rate: float | None = None,
    tire_rate: float | None = None,
    units: str = "N/m",
) -> RateMetric:
    """
    Return the ride rate measured at the contact patch on the full model.

    The full-model contact-point response already contains the tire, so the answer
    is its reciprocal and **no series formula is applied**.  ``suspension_rate``
    and ``tire_rate`` are accepted only to record the separation when the caller
    genuinely has it; they do not change the value, and supplying only one of them
    is refused rather than half-applied, because a half-series is not a formula at
    all.

    When the configuration makes the series decomposition invalid (coupled,
    inclined tire, free body) the caller is expected not to pass the split at
    all -- the full response is still the right answer.
    """
    if (suspension_rate is None) != (tire_rate is None):
        return _refused(
            "ride_rate",
            "the suspension and tire rates must be supplied together or not at all; "
            "a half-series combination is not the series law and would double-count "
            "or omit the tire",
            status="unidentifiable",
        )
    value = float(np.asarray(contact_point_response, dtype=float).ravel()[0])
    if not math.isfinite(value):
        return _refused(
            "ride_rate",
            "the contact-point vertical response is not finite, so no rate exists",
            status="unidentifiable",
        )
    if _negligible(value, _scale((value,))):
        return _refused(
            "ride_rate",
            "the contact-point vertical response is indistinguishable from zero, so "
            "no finite ride rate exists",
        )
    return RateMetric(
        name="ride_rate", status="valid", values=(1.0 / value,), units=units
    )


def roll_rate(
    moment_response: float,
    angle_response: float,
    *,
    units: str = "N*m/rad",
) -> RateMetric:
    """
    Return the roll rate as a stiffness: ``delta M_roll / delta phi``.

    Both inputs come from the same anti-phase mode.  The result is in N*m/rad; the
    caller converts to N*m/deg when the *output* convention calls for it, because
    converting here would make a stiffness indistinguishable from a moment.  A
    near-zero angle response is refused rather than reported as a huge stiffness --
    the roll angle is the measurement most easily made meaningless by a bad frame
    or a wrong reference body, and a giant roll rate is its typical signature.
    """
    moment = float(np.asarray(moment_response, dtype=float).ravel()[0])
    angle = float(np.asarray(angle_response, dtype=float).ravel()[0])
    if not (math.isfinite(moment) and math.isfinite(angle)):
        return _refused(
            "roll_rate",
            "the roll moment or roll angle response is not finite, so no rate exists",
            status="unidentifiable",
        )
    if _negligible(angle, _scale((angle, moment))):
        # Same physical situation as the wheel-rate floor: a vanishing denominator.
        # It is reported with the same `singular` status so a consumer does not have
        # to learn two statuses for one condition.
        return _refused(
            "roll_rate",
            "the roll angle response is indistinguishable from zero; a near-zero "
            "angle would report an unbounded stiffness, which usually means the "
            "angle was measured against the wrong reference body or frame",
        )
    return RateMetric(
        name="roll_rate", status="valid", values=(moment / angle,), units=units
    )


def roll_rate_per_degree(metric: RateMetric) -> RateMetric:
    """
    Convert a roll rate from N*m/rad to N*m/deg.

    A separate function rather than a flag so that the unit of every returned
    number stays a property of the number.  Refuses to convert a metric that was
    not expressed per radian, because converting twice is silent.
    """
    if not metric.available:
        return metric
    if metric.units != "N*m/rad":
        raise CharacteristicsError(
            f"roll rate must be per radian to convert, got {metric.units!r}"
        )
    return RateMetric(
        name=metric.name,
        status="valid",
        values=tuple(value * RAD_TO_DEG for value in metric.values),
        units="N*m/deg",
    )


def load_angle(
    angle_gradient: Any,
    motion: Any,
    *,
    direct_term: Any | None = None,
    units: str = "rad",
) -> tuple[float, str, str]:
    """
    Return one load-angle reading from the angle measurement chain.

    The angle must come from the **angle measurement jacobian** ``G_angle``, not
    from the wheel-centre ``rx``/``ry``/``rz`` rows.  Those rows are the rotation
    components of a pose, which coincide with toe/camber only in a limiting
    configuration; using them directly is the specific error the design calls out.

    Returns ``(value, status, reason)`` -- ``reason`` empty exactly when the status
    is ``valid`` -- so a refused reading cannot be consumed as a number.
    """
    gradient = np.asarray(angle_gradient, dtype=float).ravel()
    vector = np.asarray(motion, dtype=float).ravel()
    if gradient.shape[0] != vector.shape[0]:
        raise CharacteristicsError(
            f"the angle gradient has {gradient.shape[0]} entries for a motion of "
            f"{vector.shape[0]}; they must live in the same coordinate space"
        )
    if not (np.isfinite(gradient).all() and np.isfinite(vector).all()):
        return 0.0, "unidentifiable", "the angle chain carries a non-finite entry"
    value = float(gradient @ vector)
    if direct_term is not None:
        extra = float(np.asarray(direct_term, dtype=float).ravel()[0])
        if not math.isfinite(extra):
            return 0.0, "unidentifiable", "the direct angle term is not finite"
        value += extra
    if not math.isfinite(value):
        return 0.0, "unidentifiable", (
            "the angle reading is not representable in double precision"
        )
    return value, "valid", ""


def load_angle_from_mode(
    result: ModeResult,
    *,
    index: int = 0,
    units: str = "rad",
) -> tuple[float, str, str]:
    """
    Read one angle channel out of an already-computed mode result.

    Kept separate from :func:`load_angle` so that a refused mode propagates its own
    reason instead of being re-diagnosed from an empty array -- the original reason
    ("the tangent leaves N unconstrained directions") is strictly more informative
    than anything this layer could infer.
    """
    if result.values is None:
        return 0.0, result.status, result.reason
    values = np.asarray(result.values, dtype=float).ravel()
    if not 0 <= index < values.shape[0]:
        raise CharacteristicsError(
            f"channel {index} does not exist in a mode result with {values.shape[0]} channels"
        )
    value = float(values[index])
    if not math.isfinite(value):
        return 0.0, "unidentifiable", (
            f"angle channel {index} of mode {result.kind!r} is not finite"
        )
    return value, "valid", ""


def angle_to_degrees(value: float, *, units: str = "rad") -> tuple[float, str]:
    """
    Convert a validated angle to degrees, refusing a non-radian input.

    The unit of the input is required rather than assumed: a value that is already
    in degrees would otherwise be converted a second time, which is a silent error
    of exactly 57x.
    """
    if units != "rad":
        raise CharacteristicsError(
            f"only an angle in radians can be converted to degrees, got {units!r}"
        )
    if not math.isfinite(value):
        raise CharacteristicsError("a non-finite angle cannot be converted")
    return math.degrees(value), "deg"


def metrics_request(
    result: Any,
    request_id: str,
    metrics: Sequence[RateMetric],
    *,
    modes: Sequence[LoadMode],
    responses: dict[str, tuple[Any, str, str]],
    boundary_profile: str,
    boundary_version: int,
    definition_kind: str = "equilibrium_load_metrics",
    definition_version: int = 1,
    boundary_fingerprint: str = "",
    extra: dict[str, Any] | None = None,
) -> Any:
    """
    Wrap named metrics and the load modes that produced them into the envelope.

    ``metrics`` is the audit trail (which metrics were formed, and which were
    refused with what reason) while ``modes`` and ``responses`` are the physical
    request that was actually answered.  Those are different things and are passed
    separately on purpose: a metric **name** such as ``wheel_rate`` is not a load
    mode **kind** such as ``vertical_force``, and deriving one from the other is
    exactly the confusion this signature prevents.  In particular ``metrics`` is
    not used to synthesise the mode list.

    A refused metric is recorded in the audit rather than injected as a channel,
    because a refused metric has no answer to place in a response channel.
    """
    from .equilibrium import response_request

    # Accumulate rather than derive one list from the other.  Two metrics may share
    # a name (a valid `wheel_rate` and a refused one, for different configurations),
    # and a name that appears in both lists is not a contradiction -- it means one
    # request for that metric succeeded and another did not.  Building `formed` by
    # excluding refused names would hide the successful one; reporting the refusal
    # as a single dict would hide that there was more than one.
    formed: list[str] = []
    refused: dict[str, list[str]] = {}
    for metric in metrics:
        if metric.available:
            formed.append(metric.name)
        else:
            refused.setdefault(metric.name, []).append(metric.reason)
    payload = dict(extra or {})
    if "metric_audit" in payload:
        # Overwriting silently would make the caller believe their own audit survived.
        # A collision is a request-shape mistake, so it is refused by name.
        raise CharacteristicsError(
            "the caller-supplied `extra` already contains 'metric_audit', which this "
            "function owns; pass it under a different key"
        )
    payload["metric_audit"] = {
        "formed": formed,
        "refused": {name: reasons for name, reasons in refused.items()},
        "formed_counts": {
            name: formed.count(name) for name in dict.fromkeys(formed)
        },
    }
    return response_request(
        result,
        request_id,
        modes=modes,
        responses=responses,
        boundary_profile=boundary_profile,
        boundary_version=boundary_version,
        definition_kind=definition_kind,
        definition_version=definition_version,
        boundary_fingerprint=boundary_fingerprint,
        extra=payload,
    )
