"""Neutral result protocols shared by simulation families."""

from __future__ import annotations

from typing import Any, Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class CommonResult(Protocol):
    """
    Minimal result surface shared by dynamic simulation results.

    Axis-, solver-, and physics-family-specific ledgers deliberately stay out of
    this protocol.  Consumers that need those ledgers should depend on the
    concrete result type that owns them.
    """

    @property
    def times_s(self) -> np.ndarray:
        """Accepted output sample times in seconds."""
        ...

    @property
    def body_names(self) -> tuple[str, ...]:
        """Stable body names in state-array order."""
        ...

    @property
    def tire_names(self) -> tuple[str, ...]:
        """Stable tire names in tire-state-array order."""
        ...

    @property
    def states(self) -> np.ndarray:
        """Body state samples in the result's documented layout."""
        ...

    @property
    def diagnostics(self) -> Any:
        """Diagnostics associated with the accepted samples."""
        ...

    @property
    def performance(self) -> Any:
        """Optional aggregate run-performance counters."""
        ...

    def body_state(self, body: str) -> np.ndarray:
        """Return the state samples for one named body."""
        ...

    def tire_state(self, tire: str) -> np.ndarray:
        """Return the state samples for one named tire."""
        ...
