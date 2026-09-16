"""
Generic multibody kernel: a reusable C++ core with a thin ctypes binding.

The package owns the C++ sources, the CMake build, the single shared library,
and the parts of the ctypes boundary that carry no product semantics.  Products
with their own element vocabulary (for example the axle dynamics solver) build
their structure mirrors and marshalling on top of `suspension_kernel.binding`.
"""

from __future__ import annotations

from .binding import (
    DEFAULT_LIBRARY_STEM,
    KernelAbiMismatchError,
    KernelLibrary,
    KernelSymbolMissingError,
    LibraryIdentity,
    NativeKernelUnavailableError,
    library_path,
    load_kernel_library,
    native_build_metadata,
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
]
