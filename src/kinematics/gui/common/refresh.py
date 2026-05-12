"""Shared refresh and preview-throttling helpers for GUI pages."""

from __future__ import annotations

import tkinter as tk
from typing import Callable, Iterable

from kinematics.gui.common.entry_commit import bind_entry_commit_events


class RefreshWorkflowMixin:
    """Reusable control-change, refresh, and debounced-preview workflow helpers."""

    PREVIEW_REFRESH_DELAY_MS = 16
    updating_controls: bool
    pending_preview_refresh: str | None

    def bind_control_var_traces(
        self,
        variables: Iterable[tk.Variable],
        callback: Callable[..., None],
    ) -> None:
        """Bind variable write-traces to one callback."""
        for var in variables:
            var.trace_add("write", callback)

    def bind_entry_commit_refresh(self, entries: Iterable[tk.Widget]) -> None:
        """
        Bind entry widgets so simulation refresh happens only on commit.

        During typing, only entry-local text updates happen; refresh is deferred
        until ``Return`` or ``FocusOut``.
        """

        self.bind_entry_commit_callback(
            entries,
            callback=lambda _event: self.trigger_refresh_if_ready(),
        )

    def bind_entry_commit_callback(
        self,
        entries: Iterable[tk.Widget],
        *,
        callback: Callable[[tk.Event], None],
    ) -> None:
        """Bind entry widgets so only commit actions invoke one callback."""

        def _ignore_live_edit(_event: tk.Event) -> None:
            return

        for entry in entries:
            bind_entry_commit_events(
                entry,
                on_live_edit=_ignore_live_edit,
                on_commit=callback,
            )

    def trigger_refresh_if_ready(self) -> None:
        """Run full refresh when controls are not in an internal update phase."""
        if not self.updating_controls:
            self.refresh()

    def schedule_preview_refresh(
        self,
        *,
        scheduler: Callable[[int, Callable[[], None]], str],
        callback: Callable[[], None],
    ) -> None:
        """Schedule one debounced preview refresh callback."""
        if self.pending_preview_refresh is not None:
            return
        self.pending_preview_refresh = scheduler(self.PREVIEW_REFRESH_DELAY_MS, callback)

    def clear_pending_preview_refresh(self) -> None:
        """Clear pending preview callback handle at callback entry."""
        self.pending_preview_refresh = None

    def run_guarded(
        self,
        *,
        action: Callable[[], None],
        on_error: Callable[[Exception], None],
    ) -> None:
        """Run one action and route exceptions to the GUI-specific error sink."""
        try:
            action()
        except Exception as exc:  # noqa: BLE001 - GUI error sink handles all failures.
            on_error(exc)
