"""Compatibility forwarding surface for historical K&C metric imports."""

from __future__ import annotations

from typing import Any

from ..report.metrics.case_specific import (
    compute_k_metrics as _compute_k_metrics,
)
from ..report.metrics.case_specific import (
    wheel_metrics as _wheel_metrics,
)


def wheel_metrics(state: Any, assembly: Any, side: str) -> dict[str, float]:
    """Forward the historical helper to the registered K&C metric owner."""
    return _wheel_metrics(state, assembly, side)


def compute_k_metrics(state: Any, assembly: Any) -> dict[str, float]:
    """Forward the historical helper to the registered K&C metric owner."""
    return _compute_k_metrics(state, assembly)


__all__ = ["compute_k_metrics", "wheel_metrics"]
