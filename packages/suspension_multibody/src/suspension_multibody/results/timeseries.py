"""Immutable multi-sample result aggregation for replay and time-domain runs."""

import uuid
from dataclasses import dataclass, field, is_dataclass
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np

_ALLOWED_STATUSES = frozenset(
    {"success", "partial", "failed", "unavailable", "not_applicable"}
)


def _freeze(value: Any) -> Any:
    if isinstance(value, Mapping):
        return MappingProxyType({str(key): _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, tuple):
        return tuple(_freeze(item) for item in value)
    if isinstance(value, np.ndarray):
        array = np.asarray(value).copy()
        array.setflags(write=False)
        return array
    return value


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if is_dataclass(value):
        return {
            key: _jsonable(item)
            for key, item in value.__dict__.items()
            if not key.startswith("_")
        }
    if hasattr(value, "model_dump"):
        return _jsonable(value.model_dump(mode="json"))
    if isinstance(value, (np.floating, np.integer, np.bool_)):
        return value.item()
    return value


@dataclass(frozen=True)
class TimeSeriesSample:
    """One formally named replay/time-domain result sample."""

    time: float
    body: str
    pose: Any = None
    velocity: Any = None
    acceleration: Any = None
    loads: Mapping[str, Any] = field(default_factory=dict)
    metrics: Mapping[str, Any] = field(default_factory=dict)
    events: tuple[str, ...] = ()
    converged: bool = True
    result: Any = None

    def __post_init__(self) -> None:
        if not np.isfinite(float(self.time)):
            raise ValueError("time-series sample time must be finite")
        if not self.body:
            raise ValueError("time-series sample body cannot be empty")
        object.__setattr__(self, "time", float(self.time))
        object.__setattr__(self, "loads", _freeze(dict(self.loads)))
        object.__setattr__(self, "metrics", _freeze(dict(self.metrics)))
        object.__setattr__(self, "events", tuple(str(item) for item in self.events))

    @property
    def result_object(self) -> Any:
        """Return the formal result object represented by this sample."""
        return self if self.result is None else self.result

    def as_dict(self) -> dict[str, Any]:
        return {
            "time": self.time,
            "body": self.body,
            "pose": _jsonable(self.pose),
            "velocity": _jsonable(self.velocity),
            "acceleration": _jsonable(self.acceleration),
            "loads": _jsonable(self.loads),
            "metrics": _jsonable(self.metrics),
            "events": list(self.events),
            "converged": self.converged,
        }


@dataclass(frozen=True)
class TimeSeriesManifest:
    """Compatibility manifest surface for dynamic callers."""

    run_id: str
    mode: str | None
    sample_count: int
    provenance: Mapping[str, Any]
    performance: Mapping[str, Any] = field(default_factory=dict)
    status: str = "success"
    tables: tuple[str, ...] = ("time_samples", "diagnostics")


@dataclass(frozen=True)
class TimeSeriesResult:
    """Read-only aggregate of formal samples from one replay/time-domain run."""

    times_s: np.ndarray
    samples: tuple[TimeSeriesSample, ...]
    diagnostics: Any = ()
    metrics: Mapping[str, Any] = field(default_factory=dict)
    performance: Mapping[str, Any] = field(default_factory=dict)
    provenance: Mapping[str, Any] = field(default_factory=dict)
    status: str = "success"
    mode: str | None = None
    failure_evidence: Mapping[str, Any] = field(default_factory=dict)
    partial_evidence: Mapping[str, Any] = field(default_factory=dict)
    run_id: str = field(default_factory=lambda: uuid.uuid4().hex)

    def __post_init__(self) -> None:
        status = str(self.status).strip().lower()
        if status not in _ALLOWED_STATUSES:
            raise ValueError(f"unsupported time-series status {status!r}")
        times = np.asarray(self.times_s, dtype=np.float64).copy()
        if times.size and (
            not np.all(np.isfinite(times))
            or np.any(np.diff(times) <= 0.0)
        ):
            raise ValueError("time-series times_s must be finite and strictly increasing")
        samples = tuple(
            sample if isinstance(sample, TimeSeriesSample) else _coerce_sample(sample)
            for sample in self.samples
        )
        sample_times = tuple(sorted({sample.time for sample in samples}))
        if times.size and sample_times and tuple(times.tolist()) != sample_times:
            raise ValueError("times_s must match the unique sample time grid")
        if not times.size and sample_times:
            times = np.asarray(sample_times, dtype=np.float64)
        times.setflags(write=False)
        object.__setattr__(self, "times_s", times)
        object.__setattr__(self, "samples", samples)
        object.__setattr__(self, "diagnostics", _freeze(self.diagnostics))
        object.__setattr__(self, "metrics", _freeze(dict(self.metrics)))
        object.__setattr__(self, "performance", _freeze(dict(self.performance)))
        object.__setattr__(self, "provenance", _freeze(dict(self.provenance)))
        object.__setattr__(self, "failure_evidence", _freeze(dict(self.failure_evidence)))
        object.__setattr__(self, "partial_evidence", _freeze(dict(self.partial_evidence)))
        object.__setattr__(self, "status", status)

    @classmethod
    def from_samples(
        cls,
        samples: tuple[TimeSeriesSample, ...] | list[TimeSeriesSample],
        *,
        times_s: np.ndarray | tuple[float, ...] | list[float] | None = None,
        diagnostics: Any = (),
        metrics: Mapping[str, Any] | None = None,
        performance: Mapping[str, Any] | None = None,
        provenance: Mapping[str, Any] | None = None,
        status: str = "success",
        mode: str | None = None,
        failure_evidence: Mapping[str, Any] | None = None,
        partial_evidence: Mapping[str, Any] | None = None,
        run_id: str | None = None,
    ) -> "TimeSeriesResult":
        values = tuple(samples)
        grid = (
            np.asarray(times_s, dtype=np.float64)
            if times_s is not None
            else np.asarray(
                sorted({float(sample.time) for sample in values}), dtype=np.float64
            )
        )
        return cls(
            times_s=grid,
            samples=values,
            diagnostics=diagnostics,
            metrics={} if metrics is None else metrics,
            performance={} if performance is None else performance,
            provenance={} if provenance is None else provenance,
            status=status,
            mode=mode,
            failure_evidence={} if failure_evidence is None else failure_evidence,
            partial_evidence={} if partial_evidence is None else partial_evidence,
            run_id=uuid.uuid4().hex if run_id is None else run_id,
        )

    @property
    def manifest(self) -> TimeSeriesManifest:
        return TimeSeriesManifest(
            run_id=self.run_id,
            mode=self.mode,
            sample_count=len(self.samples),
            provenance=self.provenance,
            performance=self.performance,
            status=self.status,
        )

    @property
    def sample_count(self) -> int:
        return len(self.samples)

    @property
    def result_objects(self) -> tuple[Any, ...]:
        return tuple(sample.result_object for sample in self.samples)

    @property
    def is_partial(self) -> bool:
        return self.status == "partial" or bool(self.partial_evidence)

    @property
    def is_failed(self) -> bool:
        return self.status == "failed"

    @property
    def failure(self) -> Mapping[str, Any] | None:
        return self.failure_evidence or None

    @property
    def partial(self) -> Mapping[str, Any] | None:
        return self.partial_evidence or None

    def samples_for_body(self, body: str) -> tuple[TimeSeriesSample, ...]:
        return tuple(sample for sample in self.samples if sample.body == body)

    def as_dict(self) -> dict[str, Any]:
        return {
            "manifest": {
                "run_id": self.run_id,
                "mode": self.mode,
                "sample_count": len(self.samples),
                "provenance": _jsonable(self.provenance),
                "performance": _jsonable(self.performance),
                "tables": list(self.manifest.tables),
                "status": self.status,
                "failure_evidence": _jsonable(self.failure_evidence),
                "partial_evidence": _jsonable(self.partial_evidence),
            },
            "times_s": self.times_s.tolist(),
            "samples": [sample.as_dict() for sample in self.samples],
            "diagnostics": _jsonable(self.diagnostics),
            "metrics": _jsonable(self.metrics),
            "performance": _jsonable(self.performance),
        }


def aggregate_replay_samples(
    samples: tuple[TimeSeriesSample, ...] | list[TimeSeriesSample],
    *,
    mode: str | None,
    provenance: Mapping[str, Any] | None = None,
    metrics: Mapping[str, Any] | None = None,
    diagnostics: Any = (),
    performance: Mapping[str, Any] | None = None,
    status: str = "success",
) -> TimeSeriesResult:
    """
    Aggregate formally built replay samples into one immutable result.

    The aggregation protocol is fixed and shared: the time grid is exactly the
    sample times in order, and the default metric is the sample count.  Replay
    orchestration supplies the samples; it does not choose the aggregation.
    """
    values = tuple(samples)
    return TimeSeriesResult.from_samples(
        values,
        times_s=tuple(sample.time for sample in values),
        diagnostics=diagnostics,
        metrics={"sample_count": len(values)} if metrics is None else metrics,
        performance=performance,
        provenance=provenance,
        status=status,
        mode=mode,
    )


def _coerce_sample(value: Any) -> TimeSeriesSample:
    if isinstance(value, Mapping):
        payload = dict(value)
    else:
        payload = {
            name: getattr(value, name)
            for name in (
                "time",
                "body",
                "pose",
                "velocity",
                "acceleration",
                "loads",
                "metrics",
                "events",
                "converged",
                "result",
            )
            if hasattr(value, name)
        }
    return TimeSeriesSample(**payload)  # ty: ignore[missing-argument]


__all__ = [
    "TimeSeriesManifest",
    "TimeSeriesResult",
    "TimeSeriesSample",
    "aggregate_replay_samples",
]
