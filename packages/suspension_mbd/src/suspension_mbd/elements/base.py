"""Shared force-element result types."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class ForceEvaluation:
    """Force-element output at one quasi-static state."""

    name: str
    energy: float
    body_wrenches_global: dict[str, np.ndarray] = field(default_factory=dict)
    active: bool = True
    event: str | None = None
    tangent: np.ndarray | None = None


class ElementError(ValueError):
    """Raised when a force element is singular or invalid."""
