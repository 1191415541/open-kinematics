"""
Error types raised by the kernel binding layer.

These are deliberately generic: they describe "the shared library is missing,
unusable, or does not match the expected ABI", with no axle semantics.  Product
layers may subclass or translate them.
"""

from __future__ import annotations


class NativeKernelUnavailableError(RuntimeError):
    """Raised when the kernel shared library cannot be loaded at all."""


class KernelSymbolMissingError(NativeKernelUnavailableError):
    """
    Raised when the library loads but does not export a required symbol.

    Subclasses :class:`NativeKernelUnavailableError` so that existing callers
    which catch the broad "kernel is unavailable" condition keep working while
    the specific cause stays inspectable.
    """

    def __init__(self, symbol: str, path: str) -> None:
        super().__init__(
            f"kernel library {path} does not export {symbol!r}; the installed "
            "library is older or built with a different symbol set"
        )
        self.symbol = symbol
        self.path = path


class KernelAbiMismatchError(NativeKernelUnavailableError):
    """Raised when an exported ABI version does not match what the caller wants."""

    def __init__(self, symbol: str, expected: int, observed: object) -> None:
        # `observed` is whatever the metadata mapping held: the metadata is JSON, so
        # the value that disagrees may not be an integer at all, and the message is
        # more useful when it shows what was actually found.
        super().__init__(
            f"{symbol} reports ABI {observed}, expected {expected}; rebuild the "
            "kernel or install the matching wheel"
        )
        self.symbol = symbol
        self.expected = expected
        self.observed = observed
