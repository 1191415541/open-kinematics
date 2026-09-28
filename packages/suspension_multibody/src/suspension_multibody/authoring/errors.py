"""
The authoring layer's error base.

One base for every failure the file driven layer produces, so a caller that
handles "this project does not load" catches one type rather than enumerating
each document kind.  The subclasses stay distinct because the repairs differ: a
malformed property file is a different edit from an illegal assembly.
"""

from __future__ import annotations

__all__ = ["AuthoringError", "ElementPropertyError", "TemplateAuthoringError"]


class AuthoringError(ValueError):
    """A file document is invalid or violates an authoring boundary."""


class ElementPropertyError(AuthoringError):
    """An element property document is malformed or inconsistent with its type."""


class TemplateAuthoringError(AuthoringError):
    """A template document is malformed, or violates its own declared topology."""
