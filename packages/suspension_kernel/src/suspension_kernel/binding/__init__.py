"""
Generic ctypes binding for the suspension kernel shared library.

This package holds only what is independent of any particular product's
semantics: where the shared library lives, how it is loaded, how exported
symbols are probed, how the ABI version gate is applied, and how the recorded
build metadata is read.  It must never import `suspension_multibody`; the axle
product owns its own structure mirrors and marshalling on top of this surface.

The public names are re-exported here so callers have a single import root.
"""

from __future__ import annotations

from .errors import (
    KernelAbiMismatchError,
    KernelSymbolMissingError,
    NativeKernelUnavailableError,
)
from .library import (
    DEFAULT_LIBRARY_STEM,
    KernelLibrary,
    LibraryIdentity,
    library_path,
    load_kernel_library,
    native_build_metadata,
    native_directory,
)

__all__ = [
    "DEFAULT_LIBRARY_STEM",
    "KernelAbiMismatchError",
    "KernelLibrary",
    "KernelSymbolMissingError",
    "LibraryIdentity",
    "NativeKernelUnavailableError",
    "library_path",
    "load_kernel_library",
    "native_build_metadata",
    "native_directory",
]
