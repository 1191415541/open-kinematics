"""Neutral adaptation of the versioned kernel result document."""

from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

import numpy as np

from ..kernel import ContractRun, KernelContractError

_DIAGNOSTIC_TAIL_ROWS = 2
_PERFORMANCE_FIELDS: tuple[tuple[str, bool], ...] = (
    ("residual_calls", True),
    ("residual_time_s", False),
    ("constraint_jacobian_calls", True),
    ("constraint_jacobian_time_s", False),
    ("force_evaluations", True),
    ("force_time_s", False),
    ("mass_inverse_calls", True),
    ("mass_inverse_time_s", False),
    ("reaction_time_s", False),
    ("linear_factorizations", True),
    ("linear_factorization_time_s", False),
    ("linear_solves", True),
    ("linear_solve_time_s", False),
    ("line_search_trials", True),
    ("newton_iterations", True),
    ("accepted_steps", True),
    ("rejected_attempts", True),
    ("analytic_jacobian_columns", True),
    ("finite_difference_jacobian_columns", True),
    ("nonsmooth_fallback_columns", True),
    ("analytic_jacobian_time_s", False),
    ("finite_difference_jacobian_time_s", False),
    ("dynamic_integration_time_s", False),
)


def _readonly(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({key: _readonly(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_readonly(item) for item in value)
    return value


def _readonly_array(value: np.ndarray) -> np.ndarray:
    array = np.asarray(value).copy()
    array.setflags(write=False)
    return array


class _CaseSequence(tuple[Mapping[str, Any], ...]):
    """Tuple of case spans from the native manifest."""


def _case_spans(document: Mapping[str, Any]) -> _CaseSequence:
    manifest = document.get("manifest", {})
    return _CaseSequence(manifest.get("cases", ()))

def _diagnostic_samples(
    diagnostics: np.ndarray | None, cases: tuple[Mapping[str, Any], ...]
) -> np.ndarray | None:
    if diagnostics is None:
        return None
    rows = []
    for index, entry in enumerate(cases):
        start = int(entry["sample_offset"]) + _DIAGNOSTIC_TAIL_ROWS * index
        count = int(entry["sample_count"])
        rows.append(diagnostics[start : start + count])
    if not rows:
        return diagnostics[:0]
    return np.concatenate(rows, axis=0)


def _performance_for_case(
    diagnostics: np.ndarray, entry: Mapping[str, Any], case_index: int
) -> Mapping[str, Any]:
    start = int(entry["sample_offset"]) + _DIAGNOSTIC_TAIL_ROWS * case_index
    count = int(entry["sample_count"])
    tail = diagnostics[start + count : start + count + _DIAGNOSTIC_TAIL_ROWS]
    if tail.shape[0] < _DIAGNOSTIC_TAIL_ROWS:
        return MappingProxyType({"available": False})
    row = np.concatenate((tail[0], tail[1][:8]))
    values: dict[str, Any] = {
        "available": bool(np.isfinite(row[0]) and row[0] > 0.5)
    }
    for index, (name, is_integer) in enumerate(_PERFORMANCE_FIELDS, start=1):
        value = row[index]
        if not np.isfinite(value):
            values[name] = 0 if is_integer else 0.0
        else:
            values[name] = int(value) if is_integer else float(value)
    return MappingProxyType(values)


@dataclass(frozen=True)
class RawContractResult:
    """Read-only neutral view over one parsed :class:`ContractRun`."""

    document: Mapping[str, Any]
    blocks: Mapping[str, np.ndarray]
    model_document: Mapping[str, Any] | None = None
    case_document: Mapping[str, Any] | None = None
    _times_s: np.ndarray | None = None

    @property
    def named_blocks(self) -> Mapping[str, np.ndarray]:
        return self.blocks

    @property
    def status(self) -> str:
        return str(self.document.get("status", "failed"))

    @property
    def cases(self) -> _CaseSequence:
        return _case_spans(self.document)

    @property
    def times_s(self) -> np.ndarray:
        if self._times_s is None:
            return np.zeros(0, dtype=np.float64)
        base = np.asarray(self._times_s, dtype=np.float64)
        if not self.cases:
            return _readonly_array(base)
        spans = []
        for entry in self.cases:
            count = int(entry["sample_count"])
            if count > base.size:
                raise ValueError("result case sample count exceeds decoded time grid")
            spans.append(base[:count])
        return _readonly_array(np.concatenate(spans) if spans else base[:0])
    @property
    def body_names(self) -> tuple[str, ...]:
        manifest = self.document.get("manifest", {})
        return tuple(str(name) for name in manifest.get("bodies", ()))

    @property
    def tire_names(self) -> tuple[str, ...]:
        source = self.model_document or {}
        return tuple(
            str(item["name"])
            for item in source.get("tires", ())
            if isinstance(item, Mapping) and "name" in item
        )

    @property
    def metadata(self) -> Mapping[str, Any]:
        manifest = self.document.get("manifest", {})
        return manifest if isinstance(manifest, Mapping) else MappingProxyType({})

    @property
    def model_metadata(self) -> Mapping[str, Any]:
        return self.model_document or MappingProxyType({})

    @property
    def body_metadata(self) -> tuple[Mapping[str, Any], ...]:
        values = self.model_metadata.get("bodies", ())
        return tuple(item for item in values if isinstance(item, Mapping))

    @property
    def tire_metadata(self) -> tuple[Mapping[str, Any], ...]:
        values = self.model_metadata.get("tires", ())
        return tuple(item for item in values if isinstance(item, Mapping))

    @property
    def failure_evidence(self) -> Mapping[str, Any]:
        manifest = self.metadata
        keys = (
            "error",
            "failure",
            "failed_status",
            "failed_sample_index",
            "failed_time_s",
        )
        return MappingProxyType({key: manifest[key] for key in keys if key in manifest})

    @property
    def partial_evidence(self) -> Mapping[str, Any]:
        manifest = self.metadata
        keys = ("completed_sample_count", "partial", "status")
        return MappingProxyType({key: manifest[key] for key in keys if key in manifest})

    @property
    def states(self) -> np.ndarray:
        return self.block("body_state")

    @property
    def diagnostics(self) -> np.ndarray | None:
        return _diagnostic_samples(self.blocks.get("diagnostics"), self.cases)

    @property
    def performance(self) -> Any:
        diagnostics = self.blocks.get("diagnostics")
        if diagnostics is None:
            return None
        values = tuple(
            _performance_for_case(diagnostics, entry, index)
            for index, entry in enumerate(self.cases)
        )
        if len(values) == 1:
            return values[0]
        return values

    def block(self, name: str) -> np.ndarray:
        try:
            return self.blocks[name]
        except KeyError as error:
            raise KernelContractError(f"result has no block {name!r}") from error

    def body_state(self, body: str) -> np.ndarray:
        return self.states[:, self.body_names.index(body), :]

    def tire_state(self, tire: str) -> np.ndarray:
        if tire not in self.tire_names:
            raise KeyError(f"unknown tire {tire!r}")
        return self.block("tire_output")[:, self.tire_names.index(tire), :]


    @property
    def diagnostics_block(self) -> np.ndarray:
        """Return the immutable native diagnostics ledger."""
        return self.block("diagnostics")

    def case_body_state(self, case_index: int = 0, sample_index: int = -1) -> np.ndarray:
        """Return one body-state row for a contract case."""
        entry = self.cases[case_index]
        count = int(entry["sample_count"])
        offset = sample_index if sample_index >= 0 else count + sample_index
        if offset < 0 or offset >= count:
            raise IndexError("case sample index out of range")
        first = int(entry["sample_offset"])
        return self.states[first + offset].copy()

    def case_residuals(self, case_index: int = 0) -> tuple[float, float, float]:
        """Return constraint, dynamics, and moment residuals for one case."""
        entry = self.cases[case_index]
        first = int(entry["sample_offset"]) + _DIAGNOSTIC_TAIL_ROWS * case_index
        row = self.block("diagnostics")[first + int(entry["sample_count"]) - 1]
        return float(row[7]), float(row[9]), 0.0



def _decode_contract_run(run: ContractRun | RawContractResult) -> RawContractResult:
    """Adapt one parsed kernel run to the neutral result surface."""
    if isinstance(run, RawContractResult):
        return run
    if not isinstance(run, ContractRun):
        raise TypeError("the internal result adapter expects a parsed kernel ContractRun")
    document = _readonly(run.document)
    blocks = MappingProxyType(
        {name: _readonly_array(value) for name, value in run.blocks.items()}
    )
    return RawContractResult(
        document=document,
        blocks=blocks,
        model_document=_readonly(run.model_document) if run.model_document is not None else None,
        case_document=_readonly(run.case_document) if run.case_document is not None else None,
        _times_s=_readonly_array(run.times_s) if run.times_s is not None else None,
    )
