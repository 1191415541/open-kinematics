"""Shared helpers for entry widgets that commit edits on explicit events."""

from __future__ import annotations

import tkinter as tk
from typing import Callable


def bind_entry_commit_events(
    entry: tk.Widget,
    *,
    on_live_edit: Callable[[tk.Event], None],
    on_commit: Callable[[tk.Event], None],
) -> None:
    """
    Bind one entry to live-edit and commit callbacks.

    - ``on_live_edit`` runs on each key release (typing phase).
    - ``on_commit`` runs on Return and FocusOut (commit phase).
    """
    entry.bind("<KeyRelease>", on_live_edit)
    entry.bind("<Return>", on_commit)
    entry.bind("<FocusOut>", on_commit)
