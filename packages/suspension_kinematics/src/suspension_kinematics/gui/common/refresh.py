"""Shared refresh and preview-throttling helpers for GUI pages."""

from __future__ import annotations

import tkinter as tk
from typing import Callable, Iterable

from suspension_kinematics.gui.common.entry_commit import bind_entry_commit_events


class RefreshWorkflowMixin:
    """Reusable control-change, refresh, and debounced-preview workflow helpers."""

    PREVIEW_REFRESH_DELAY_MS = 16
    HARDPOINT_PREVIEW_DELAY_MS = 80
    HARDPOINT_FULL_REFRESH_DELAY_MS = 400
    updating_controls: bool
    pending_preview_refresh: str | None
    pending_hardpoint_full_refresh: str | None

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
        self.pending_preview_refresh = scheduler(
            self.PREVIEW_REFRESH_DELAY_MS, callback
        )

    def clear_pending_preview_refresh(self) -> None:
        """Clear pending preview callback handle at callback entry."""
        self.pending_preview_refresh = None

    def schedule_hardpoint_edit_refresh(
        self,
        *,
        scheduler: Callable[[int, Callable[[], None]], str],
        cancel: Callable[[str], None],
        preview_callback: Callable[[], None],
        full_callback: Callable[[], None],
    ) -> None:
        """
        Keep the UI responsive while hardpoints are edited.

        - Trailing-debounced preview keeps the geometry view current.
        - Trailing-debounced full refresh regenerates outputs and full curves
          after editing stops.
        """
        self._reschedule_callback(
            handle_attr="pending_preview_refresh",
            delay_ms=self.HARDPOINT_PREVIEW_DELAY_MS,
            scheduler=scheduler,
            cancel=cancel,
            callback=preview_callback,
        )

        def run_full_refresh() -> None:
            self.pending_hardpoint_full_refresh = None
            preview_handle = self.pending_preview_refresh
            if preview_handle is not None:
                try:
                    cancel(preview_handle)
                except Exception:  # noqa: BLE001 - stale after() handles are fine.
                    pass
                self.pending_preview_refresh = None
            full_callback()

        self._reschedule_callback(
            handle_attr="pending_hardpoint_full_refresh",
            delay_ms=self.HARDPOINT_FULL_REFRESH_DELAY_MS,
            scheduler=scheduler,
            cancel=cancel,
            callback=run_full_refresh,
        )

    def _reschedule_callback(
        self,
        *,
        handle_attr: str,
        delay_ms: int,
        scheduler: Callable[[int, Callable[[], None]], str],
        cancel: Callable[[str], None],
        callback: Callable[[], None],
    ) -> None:
        """Cancel any pending handle and schedule a new trailing callback."""
        handle = getattr(self, handle_attr, None)
        if handle is not None:
            try:
                cancel(handle)
            except Exception:  # noqa: BLE001 - stale after() handles are fine.
                pass
            setattr(self, handle_attr, None)

        def run_callback() -> None:
            setattr(self, handle_attr, None)
            callback()

        setattr(self, handle_attr, scheduler(delay_ms, run_callback))

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
