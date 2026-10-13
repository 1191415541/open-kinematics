"""
The read-only query layer: turn persisted native channels into answered requests.

This module is the single place where a *stored* result is read back as a
characteristic.  It exists because reading a request and answering it are two
separate capabilities (P0's capability gate makes that explicit), and because a
query that quietly re-solves would be indistinguishable from one that reads --
until the numbers drifted.

Three duties, in order:

* **Validate the channel before decoding it.**  A missing channel is a query
  error naming the channel, never an empty array and never a fallback
  recomputation.  The kernel may legitimately not have been asked for
  ``constraint_jacobian``; that is different from it having produced a zero one.
* **Attach identity to every sample.**  A value's position in an array is not its
  identity: a dynamic point is identified by its case, time and output-sample id.
  The identity travels with the value so a table row is traceable.
* **Refuse to compute.**  Nothing here calls the kernel, re-balances, or reloads
  modelling resources.  Pure algebraic measurements over channels that are already
  present are allowed -- that is what ``measure`` does -- but a request whose
  underlying response was never produced is reported missing.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from ..characteristics.values import (
    DOMAINS,
    CharacteristicResult,
    CharacteristicsError,
    MeasurementValue,
    SampleIdentity,
    VectorValue,
    statuses_for,
)


def sample_identities(result: Any) -> tuple[SampleIdentity, ...]:
    """
    Return one identity per stored sample, keyed by case, time and sample index.

    Two index spaces are reported and they are deliberately different:

    * ``index`` is the position in the *native* blocks, which is what a caller
      needs in order to subscript ``raw.block(...)``.
    * ``output_sample_id`` is the position within the sample's own case, which is
      the only stable identifier for a trajectory comparison -- a solver that
      takes a different number of accepted steps keeps the same ids.

    The time comes from the result's own per-case grid.  A case whose grid is
    shorter than the case's sample count is a *structural* inconsistency, and is
    reported as one: quietly leaving ``time_s`` empty would make a mis-recorded
    offset indistinguishable from a sample that legitimately has no time.
    """
    times = np.asarray(result.times_s, dtype=float)
    cases = tuple(getattr(getattr(result, "raw", result), "cases", ()) or ())
    identities: list[SampleIdentity] = []
    if not cases:
        for index in range(times.size):
            identities.append(SampleIdentity(index=index, time_s=float(times[index])))
        return tuple(identities)
    # `RawContractResult.times_s` concatenates one grid *per case*, each truncated
    # to that case's own sample count -- so the times must be walked case by case
    # rather than indexed by the flattened offset.
    cursor = 0
    for case_index, entry in enumerate(cases):
        count = int(entry["sample_count"])
        offset = int(entry["sample_offset"])
        if cursor + count > times.size:
            raise CharacteristicsError(
                f"case {case_index} declares {count} samples but the decoded time grid "
                f"only has {times.size - cursor} left; the case spans and the time grid "
                "disagree, so sample identity cannot be assigned"
            )
        for within in range(count):
            identities.append(
                SampleIdentity(
                    index=offset + within,
                    time_s=float(times[cursor + within]),
                    case_index=case_index,
                    output_sample_id=within,
                )
            )
        cursor += count
    return tuple(identities)


def require_channel(result: Any, name: str, *, request_id: str) -> np.ndarray:
    """
    Return a stored native channel, or report the missing channel as a query error.

    A channel that was never requested is *absent*, not zero.  Returning an empty
    or zero-filled array here would let a caller treat "the kernel was not asked"
    as "the answer is zero", which is the failure this function exists to prevent.

    The stored result reaches this module as either the envelope or the raw
    contract result; naming the envelope only would make the same query crash on
    the other shape.
    """
    raw = getattr(result, "raw", result)
    blocks = getattr(raw, "blocks", None)
    if blocks is None or name not in blocks:
        available = ", ".join(sorted(blocks)) if blocks else "none"
        raise CharacteristicsError(
            f"request {request_id!r} needs the native channel {name!r}, which this "
            f"result does not contain; available channels: {available}"
        )
    return np.asarray(raw.block(name), dtype=float)


def measurement_from_channel(
    result: Any,
    name: str,
    values: Any,
    *,
    units: str | tuple[str, ...],
    frame_id: str,
    definition_version: int = 1,
    reasons: Any = None,
) -> MeasurementValue | VectorValue:
    """
    Wrap an already-computed value array as a measurement with per-sample status.

    A one-dimensional array becomes a ``MeasurementValue``; an ``(samples, n)``
    array becomes a ``VectorValue`` carrying one unit per component.  The
    distinction is not cosmetic: a roll centre is a two-component quantity, and
    flattening it into a single-unit measurement would claim its vertical and
    lateral parts share one unit token.

    Non-finite entries are turned into the contract's unavailable status, and they
    must carry a reason: ``statuses_for`` refuses to invent one.  Callers that can
    say *why* a sample is unavailable pass ``reasons``; callers that cannot get a
    refusal rather than a quiet ``undefined_geometry``.
    """
    array = np.asarray(values, dtype=float)
    if array.ndim == 1:
        if not isinstance(units, str):
            raise CharacteristicsError(
                f"measurement {name!r} is scalar but declares {len(units)} units"
            )
        status, reason = statuses_for(array, reasons)
        return MeasurementValue(
            measurement_id=name,
            values=array,
            units=units,
            frame_id=frame_id,
            status=status,
            reason=reason,
            definition_version=definition_version,
        )
    if array.ndim != 2:
        raise CharacteristicsError(
            f"measurement {name!r} must be one- or two-dimensional, got shape {array.shape}"
        )
    components = array.shape[1]
    if reasons is not None:
        raise CharacteristicsError(
            f"measurement {name!r} is a {components}-component vector; per-component "
            "unavailability is reported as a whole row rather than one reason per component"
        )
    component_units = (units,) * components if isinstance(units, str) else tuple(units)
    statuses: list[str] = []
    notes: list[str] = []
    for row in array:
        # A vector row counts as available only if every component is a number:
        # a half-present vector is not a reading, and returning the finite half
        # would look like the missing component was simply zero.
        if np.isfinite(row).all():
            statuses.append("valid")
            notes.append("")
        else:
            statuses.append("undefined_geometry")
            notes.append("one or more components are not available")
    return VectorValue(
        measurement_id=name,
        values=array,
        units=component_units,
        frame_id=frame_id,
        status=tuple(statuses),
        reason=tuple(notes),
        definition_version=definition_version,
    )


def characteristic(
    result: Any,
    request_id: str,
    *,
    domain: str,
    definition_kind: str,
    definition_version: int,
    channels: Any,
    boundary_profile: str,
    boundary_version: int,
    boundary_fingerprint: str = "",
    frozen_protocol: str | None = None,
    frozen_version: int | None = None,
    response_operator: str | None = None,
    operator_version: int | None = None,
    extra: Any = None,
) -> CharacteristicResult:
    """
    Assemble one answered request from channels this result already holds.

    ``channels`` maps a measurement ID to ``(values, units, frame_id)``.  Every
    value must already exist: this function does not compute a response, it
    *presents* one, which is why it takes the numbers rather than a model.  The
    provenance fields are mandatory where they exist so a consumer can tell which
    frozen protocol and boundary a number came from.
    """
    if domain not in DOMAINS:
        raise CharacteristicsError(
            f"unknown response domain {domain!r}; the contract defines {list(DOMAINS)}"
        )
    identities = sample_identities(result)
    responses: dict[str, MeasurementValue | VectorValue] = {}
    for name, payload in dict(channels).items():
        values, units, frame_id = payload
        # The sample count check lives in `CharacteristicResult`: a response whose
        # length disagrees with the request's selected samples is refused there,
        # because that is where both numbers are finally known.
        responses[name] = measurement_from_channel(
            result,
            name,
            values,
            units=units,
            frame_id=frame_id,
            definition_version=definition_version,
        )
    return CharacteristicResult(
        request_id=request_id,
        domain=domain,
        definition_kind=definition_kind,
        definition_version=definition_version,
        samples=identities,
        responses=responses,
        boundary_profile=boundary_profile,
        boundary_version=boundary_version,
        boundary_fingerprint=boundary_fingerprint,
        frozen_protocol=frozen_protocol,
        frozen_version=frozen_version,
        response_operator=response_operator,
        operator_version=operator_version,
        model_fingerprint=str(getattr(result, "model_fingerprint", "")),
        extra=dict(extra or {}),
    )


def envelope_from_artifact(payload: Any) -> Any:
    """
    Rebuild a queryable result from a written artifact, without solving.

    The artifact stores everything a query needs and nothing more: the native
    channels (``arrays.npz``), the channel identity manifest, the time grid, and
    the model document.  Nothing here recompiles the model, re-balances, or
    touches the kernel, so an answer read from an artifact is the stored answer
    rather than a recomputation of it.

    The model document is re-validated through ``ResolvedModel``: an artifact
    whose model no longer satisfies the contract is refused instead of being
    half-read, and an artifact written without a model is refused by name, since
    without declarations there is nothing to answer queries against.
    """
    from types import MappingProxyType

    from ..modeling.resolved import ResolvedModel
    from .envelope import ResultEnvelope
    from .raw import RawContractResult

    manifest = payload["manifest"]
    model_document = manifest.get("model")
    if model_document is None:
        raise CharacteristicsError(
            "this artifact carries no model document, so its measurements cannot be "
            "answered; it was written without a model"
        )
    channels = manifest.get("channels") or {}
    arrays = payload["arrays"]
    times = arrays.get("times_s")
    document = MappingProxyType(
        {
            "contract_version": int(manifest.get("contract_version", 2)),
            "status": str(manifest.get("status", "failed")),
            # `channels` is the native document's own manifest, which is where the
            # constraint identities, free bodies and case spans live.
            "manifest": MappingProxyType(dict(channels)),
        }
    )
    raw = RawContractResult(
        document=document,
        blocks=MappingProxyType(
            {
                name: np.asarray(value)
                for name, value in arrays.items()
                if name != "times_s"
            }
        ),
        model_document=MappingProxyType(dict(model_document)),
        case_document=None,
        _times_s=None if times is None else np.asarray(times, dtype=float),
    )
    return ResultEnvelope(raw=raw, model=ResolvedModel(model_document))


def measurement_query(
    result: Any,
    name: str,
    *,
    request_id: str | None = None,
    definition_version: int = 1,
) -> CharacteristicResult:
    """
    Answer a declared measurement from this result's own channels, without solving.

    This is the minimum query P1 promises: the declaration names a frame, an axis
    and a sign, and the envelope already holds the poses -- so the answer is pure
    algebra over stored channels.  A measurement whose primitive needs a channel
    the result does not carry raises here rather than at export time.
    """
    measured = result.measure(name)
    return characteristic(
        result,
        request_id or name,
        domain="kinematic",
        definition_kind=measured.name,
        definition_version=definition_version,
        channels={name: (measured.values, measured.units, measured.frame_id)},
        boundary_profile="stored-samples",
        boundary_version=1,
        response_operator="ResultEnvelope.measure",
        operator_version=1,
    )


__all__ = [
    "characteristic",
    "measurement_from_channel",
    "measurement_query",
    "require_channel",
    "sample_identities",
]
