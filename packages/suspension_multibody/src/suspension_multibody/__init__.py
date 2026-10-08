"""
Independent quasi-static suspension K&C and load analysis package.

The public names below are reached through :pep:`562` module ``__getattr__``
rather than imported here.  The distinction is not stylistic: importing ``api``
at the package root executes most of the product, so *any* submodule import --
including the low ``modeling`` layer, which is meant to be usable on its own --
paid for the whole chain, and the resulting half-initialised modules are what
made import order load-bearing.  Resolving on first access keeps the same public
surface for callers while letting a leaf module be a leaf.

``TYPE_CHECKING`` imports give static tools the real types without running
anything.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .api import simulate, validate
    from .io.artifacts import read_artifact, write_artifact
    from .results.envelope import ResultEnvelope

__version__ = "0.1.0"

#: Public name -> the module attribute path that defines it.  Kept as data so
#: the surface is inspectable and a typo is a lookup error rather than a silent
#: missing attribute.
_PUBLIC_NAMES: dict[str, tuple[str, str]] = {
    "ResultEnvelope": (".results.envelope", "ResultEnvelope"),
    "read_artifact": (".io.artifacts", "read_artifact"),
    "open_bus": (".signal_bus", "open_bus"),
    "simulate": (".api", "simulate"),
    "validate": (".api", "validate"),
    "write_artifact": (".io.artifacts", "write_artifact"),
}

__all__ = [
    "ResultEnvelope",
    "__version__",
    "open_bus",
    "read_artifact",
    "simulate",
    "validate",
    "write_artifact",
]


def __getattr__(name: str) -> Any:
    """Resolve a public name on first access, importing only its own module."""
    target = _PUBLIC_NAMES.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute = target
    from importlib import import_module

    value = getattr(import_module(module_name, __name__), attribute)
    # Cache on the module so the lookup happens once, exactly as an eager import
    # would have left it.
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    """List the public surface without importing any of it."""
    return sorted({*globals(), *__all__})
