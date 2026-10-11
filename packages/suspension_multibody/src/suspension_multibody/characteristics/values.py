"""
Read-only value types for characteristics.

These are the *runtime expression of the contract*, not a second contract.  Every
field here mirrors a field the `suspension_contracts` schemas already define, and
nothing in this module may add a name the contract does not have: two field
definitions for one concept drift apart, and the drift is invisible until one
side is serialised and the other is read back.

Two deliberate design choices:

* **Frozen and array-owning.**  A measurement is handed to callers who will
  compare it against a reference and then keep it.  A mutable, aliasing value
  makes a later in-place edit look like a change of physics, so arrays are stored
  read-only and copies are made on the way in.

* **No solver behaviour.**  Nothing in this module reads a result envelope,
  imports `results`, or issues a native request.  A value type that can compute
  is a value type that can hide a second implementation of a formula.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

import numpy as np

#: The sample statuses the contract defines.  A point is never silently dropped:
#: an unavailable value carries one of these instead of a zero or a repeated
#: previous sample, because a filled-in number is indistinguishable from a
#: measured one once it is written to a table.
SAMPLE_STATUSES = (
    "valid",
    "unidentifiable",
    "singular",
    "nonsmooth",
    "not_equilibrium",
    "inconsistent_sample",
    "undefined_geometry",
)

#: The sample *selections* the contract names, in the order it lists them.  These
#: are not interchangeable views of one array: ``output`` is what the sampler
#: stores, ``initial`` is that grid's first consistent point, and ``accepted``
#: refers to per-accepted-step states the native result does not persist.  Keeping
#: one definition here means a refusal and the value that triggered it cannot
#: disagree about what the names are.
SAMPLE_KINDS = ("initial", "accepted", "output")

#: The four response domains, in the order the contract lists them.
DOMAINS = ("kinematic", "equilibrium", "trajectory", "dynamic")

#: Below this a resultant (force, moment, or a vector norm) is treated as absent
#: rather than merely small.  A centre with a zero resultant is undefined, not
#: far away, so the same floor is shared by every operator that has to make that
#: call instead of each choosing its own.
RESULTANT_FLOOR = 1e-12

#: Below this angular speed a motion has no instantaneous axis.  A pure
#: translation is not a screw with a distant axis; it is not a screw at all.
ANGULAR_SPEED_FLOOR = 1e-12


def _readonly(values: Any, dtype: Any = float) -> np.ndarray:
    """Copy into a read-only array so a caller cannot alias shared state."""
    array = np.array(values, dtype=dtype, copy=True)
    array.setflags(write=False)
    return array


class CharacteristicsError(ValueError):
    """Raised when a value cannot be expressed as the contract requires."""


@dataclass(frozen=True)
class MeasurementValue:
    """
    One named physical quantity over samples, with its unit and status.

    ``status`` is per sample and has the same length as ``values``.  A value row
    is present for every sample -- including the invalid ones -- so a table
    writer can emit a reason where a number is missing.  ``values`` for an
    invalid sample is NaN rather than zero: zero is a legitimate reading.
    """

    measurement_id: str
    values: np.ndarray
    units: str
    frame_id: str
    status: tuple[str, ...]
    reason: tuple[str, ...]
    definition_version: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", _readonly(self.values))
        if self.values.ndim != 1:
            raise CharacteristicsError("measurement values must be one-dimensional")
        if len(self.status) != len(self.values):
            raise CharacteristicsError(
                f"measurement {self.measurement_id!r} has {len(self.status)} statuses "
                f"for {len(self.values)} samples"
            )
        if len(self.reason) != len(self.values):
            raise CharacteristicsError(
                f"measurement {self.measurement_id!r} has {len(self.reason)} reasons "
                f"for {len(self.values)} samples"
            )
        unknown = {name for name in self.status if name not in SAMPLE_STATUSES}
        if unknown:
            raise CharacteristicsError(
                f"measurement {self.measurement_id!r} uses unknown status(es) {sorted(unknown)}"
            )
        # An invalid sample must say why, and a valid one must not claim a reason.
        for index, name in enumerate(self.status):
            if name == "valid" and self.reason[index]:
                raise CharacteristicsError(
                    f"measurement {self.measurement_id!r} sample {index} is valid but carries "
                    f"a reason {self.reason[index]!r}"
                )
            if name != "valid" and not self.reason[index]:
                raise CharacteristicsError(
                    f"measurement {self.measurement_id!r} sample {index} is {name!r} "
                    "without a reason"
                )
            if name != "valid" and not np.isnan(self.values[index]):
                raise CharacteristicsError(
                    f"measurement {self.measurement_id!r} sample {index} is {name!r} but "
                    "carries a numeric value; an unavailable reading must not be filled in"
                )
        if not self.units:
            raise CharacteristicsError("measurement units are required")
        if self.definition_version < 1:
            raise CharacteristicsError("definition_version must be a positive integer")

    def __len__(self) -> int:
        return len(self.values)

    @property
    def valid_mask(self) -> np.ndarray:
        """Which samples carry a usable number."""
        return np.asarray([name == "valid" for name in self.status], dtype=bool)

    @classmethod
    def all_valid(
        cls,
        measurement_id: str,
        values: Any,
        units: str,
        frame_id: str,
        definition_version: int = 1,
    ) -> MeasurementValue:
        """Build a fully valid measurement, for the common case."""
        array = _readonly(values)
        return cls(
            measurement_id=measurement_id,
            values=array,
            units=units,
            frame_id=frame_id,
            status=("valid",) * len(array),
            reason=("",) * len(array),
            definition_version=definition_version,
        )


@dataclass(frozen=True)
class VectorValue:
    """A vector-valued quantity over samples, shape ``(samples, components)``."""

    measurement_id: str
    values: np.ndarray
    units: tuple[str, ...]
    frame_id: str
    status: tuple[str, ...]
    reason: tuple[str, ...]
    definition_version: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", _readonly(self.values))
        if self.values.ndim != 2:
            raise CharacteristicsError("vector values must be two-dimensional")
        components = self.values.shape[1]
        if len(self.units) != components:
            raise CharacteristicsError(
                f"vector {self.measurement_id!r} declares {len(self.units)} units for "
                f"{components} components; a six-axis quantity has no single unit"
            )
        if len(self.status) != self.values.shape[0]:
            raise CharacteristicsError(
                f"vector {self.measurement_id!r} status length disagrees with its samples"
            )
        if len(self.reason) != self.values.shape[0]:
            raise CharacteristicsError(
                f"vector {self.measurement_id!r} reason length disagrees with its samples"
            )
        unknown = {name for name in self.status if name not in SAMPLE_STATUSES}
        if unknown:
            raise CharacteristicsError(
                f"vector {self.measurement_id!r} uses unknown status(es) {sorted(unknown)}"
            )

    def __len__(self) -> int:
        return self.values.shape[0]

    @property
    def valid_mask(self) -> np.ndarray:
        """
        Which samples carry a usable number.

        A vector row is usable only as a whole: one missing component makes the
        row unavailable, because a partially-filled vector that reported its
        finite components would look like the missing one was zero.
        """
        return np.asarray([name == "valid" for name in self.status], dtype=bool)

    @classmethod
    def all_valid(
        cls,
        measurement_id: str,
        values: Any,
        units: tuple[str, ...],
        frame_id: str,
        definition_version: int = 1,
    ) -> VectorValue:
        """Build a fully valid vector, for the common case."""
        array = _readonly(values)
        return cls(
            measurement_id=measurement_id,
            values=array,
            units=tuple(units),
            frame_id=frame_id,
            status=("valid",) * array.shape[0],
            reason=("",) * array.shape[0],
            definition_version=definition_version,
        )


@dataclass(frozen=True)
class DerivativeValue:
    """
    A measured quantity together with its derivative with respect to an input.

    The numerator and denominator units are kept separate: collapsing them into
    one token loses which side was differentiated, and that is exactly what a
    consumer needs in order to check a chain rule.
    """

    measurement_id: str
    perturbation_id: str
    values: np.ndarray
    numerator_units: str
    denominator_units: str
    method: str
    status: tuple[str, ...]
    reason: tuple[str, ...]
    step: float | None = None
    definition_version: int = 1

    def __post_init__(self) -> None:
        object.__setattr__(self, "values", _readonly(self.values))
        if self.values.ndim != 1:
            raise CharacteristicsError("derivative values must be one-dimensional")
        if not self.numerator_units or not self.denominator_units:
            raise CharacteristicsError("derivative units require both sides")
        if self.method not in ("analytic", "transpose", "oracle_difference"):
            raise CharacteristicsError(
                f"unknown derivative method {self.method!r}; production responses are "
                "analytic, and a difference quotient is an oracle, not a fallback"
            )
        if len(self.status) != len(self.values):
            raise CharacteristicsError("derivative status length disagrees with its samples")
        unknown = {name for name in self.status if name not in SAMPLE_STATUSES}
        if unknown:
            raise CharacteristicsError(f"derivative uses unknown status(es) {sorted(unknown)}")

    def __len__(self) -> int:
        return len(self.values)


@dataclass(frozen=True)
class SampleIdentity:
    """
    What a sample *is*, independent of where it sits in an array.

    A dynamic point identified only by its array index cannot be compared against
    a reference run, because the index changes when the solver takes a different
    number of accepted steps.  The identity travels with the value.
    """

    index: int
    time_s: float | None = None
    case_index: int = 0
    accepted_step_id: int | None = None
    output_sample_id: int | None = None
    #: Which *selection* this point came from: ``initial``, ``accepted`` or
    #: ``output``.  Recorded because a point's array position does not say which
    #: set it belongs to -- the initial consistent point and an ordinary output
    #: sample are both "sample 0 of some array", and a rate computed over the wrong
    #: set is a plausible number rather than an error.
    sample_kind: str | None = None

    def __post_init__(self) -> None:
        if self.index < 0:
            raise CharacteristicsError("sample index must not be negative")
        if self.case_index < 0:
            raise CharacteristicsError("case index must not be negative")
        if self.sample_kind is not None and self.sample_kind not in SAMPLE_KINDS:
            raise CharacteristicsError(
                f"unknown sample kind {self.sample_kind!r}; the contract defines "
                f"{', '.join(SAMPLE_KINDS)}"
            )


@dataclass(frozen=True)
class CharacteristicResult:
    """
    One answered characteristic request, keyed to the identity that was asked for.

    ``responses`` is keyed by measurement ID and holds either scalar or vector
    values; the query layer hands these back without recomputing anything.  The
    provenance fields exist so a consumer can tell *which* frozen protocol and
    boundary a number came from rather than trusting that it came from the one
    it asked for.
    """

    request_id: str
    domain: str
    definition_kind: str
    definition_version: int
    samples: tuple[SampleIdentity, ...]
    responses: Mapping[str, MeasurementValue | VectorValue]
    boundary_profile: str
    boundary_version: int
    boundary_fingerprint: str = ""
    frozen_protocol: str | None = None
    frozen_version: int | None = None
    response_operator: str | None = None
    operator_version: int | None = None
    model_fingerprint: str = ""
    extra: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        object.__setattr__(self, "samples", tuple(self.samples))
        object.__setattr__(self, "responses", dict(self.responses))
        if self.domain not in DOMAINS:
            raise CharacteristicsError(f"unknown response domain {self.domain!r}")
        if self.definition_version < 1 or self.boundary_version < 1:
            raise CharacteristicsError("definition and boundary versions must be positive")
        if not self.request_id:
            raise CharacteristicsError("a result must name the request it answers")
        for name, value in self.responses.items():
            if name != value.measurement_id:
                raise CharacteristicsError(
                    f"response key {name!r} disagrees with the measurement it holds "
                    f"({value.measurement_id!r})"
                )
            if len(value) != len(self.samples):
                raise CharacteristicsError(
                    f"response {name!r} has {len(value)} samples but the request selected "
                    f"{len(self.samples)}"
                )

    def __len__(self) -> int:
        return len(self.samples)

    def measurement(self, measurement_id: str) -> MeasurementValue | VectorValue:
        """Return one named response, or fail rather than guess."""
        if measurement_id not in self.responses:
            available = ", ".join(sorted(self.responses)) or "none"
            raise CharacteristicsError(
                f"request {self.request_id!r} did not answer {measurement_id!r}; "
                f"answered: {available}"
            )
        return self.responses[measurement_id]

    def provenance(self) -> dict[str, Any]:
        """Return the identity fields a table row or a report needs to be traceable."""
        return {
            "request_id": self.request_id,
            "domain": self.domain,
            "definition_kind": self.definition_kind,
            "definition_version": self.definition_version,
            "boundary_profile": self.boundary_profile,
            "boundary_version": self.boundary_version,
            "boundary_fingerprint": self.boundary_fingerprint,
            "frozen_protocol": self.frozen_protocol,
            "frozen_version": self.frozen_version,
            "response_operator": self.response_operator,
            "operator_version": self.operator_version,
            "model_fingerprint": self.model_fingerprint,
            "sample_count": len(self.samples),
        }


def statuses_for(values: Any, reasons: Sequence[str] | None = None) -> tuple[tuple[str, ...], tuple[str, ...]]:
    """
    Derive per-sample status from a raw array, where NaN means "not available".

    This is the only permitted way to turn a numeric array into a measurement:
    it makes the unavailable samples explicit instead of letting a NaN reach a
    table or a comparison as if it were a reading.
    """
    array = np.asarray(values, dtype=float)
    given = tuple(reasons) if reasons is not None else ("",) * array.size
    if len(given) != array.size:
        raise CharacteristicsError("reason length must match the value count")
    status: list[str] = []
    for index, value in enumerate(array.ravel()):
        if np.isfinite(value):
            status.append("valid")
        elif not given[index]:
            raise CharacteristicsError(
                f"sample {index} has no numeric value but no reason; an unavailable "
                "reading must carry one of the contract's sample statuses"
            )
        else:
            status.append("undefined_geometry")
    return tuple(status), given


__all__ = [
    "DOMAINS",
    "SAMPLE_KINDS",
    "SAMPLE_STATUSES",
    "CharacteristicResult",
    "CharacteristicsError",
    "DerivativeValue",
    "MeasurementValue",
    "SampleIdentity",
    "VectorValue",
    "statuses_for",
]
