"""Shared cancellation helpers for long-running GUI tasks."""

from __future__ import annotations

import threading


class OptimizationCancelledError(RuntimeError):
    """Raised when a GUI optimization task is cancelled by the user."""


def raise_if_cancelled(
    cancel_event: threading.Event | None,
    *,
    message: str = "Optimization cancelled",
) -> None:
    """Raise when a cooperative cancellation event has been signalled."""
    if cancel_event is not None and cancel_event.is_set():
        raise OptimizationCancelledError(message)
