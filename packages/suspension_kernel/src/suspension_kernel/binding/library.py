"""
Locate, load, and probe the kernel shared library.

The loader is written so that every failure mode is explicit and typed:

* the library file is absent            -> :class:`NativeKernelUnavailableError`
* a required exported symbol is absent  -> :class:`KernelSymbolMissingError`
* an exported ABI version is unexpected -> :class:`KernelAbiMismatchError`

Symbols are probed with ``getattr(library, name, None)`` rather than by direct
attribute access, because ``ctypes.CDLL.__getattr__`` raises a bare
``AttributeError`` that callers cannot distinguish from a typo.
"""

from __future__ import annotations

import ctypes
import json
import platform
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable, Mapping

from .errors import (
    KernelAbiMismatchError,
    KernelSymbolMissingError,
    NativeKernelUnavailableError,
)

#: Base name of the shared library, without platform decoration or extension.
DEFAULT_LIBRARY_STEM = "suspension_kernel"

_METADATA_NAME = "native_build.json"


def native_directory() -> Path:
    """Return the directory that holds the packaged shared library."""
    return Path(__file__).resolve().parent.parent / "native"


def library_path(stem: str = DEFAULT_LIBRARY_STEM, directory: Path | None = None) -> Path:
    """Return the platform-specific path of the shared library."""
    root = native_directory() if directory is None else Path(directory)
    system = platform.system()
    if system == "Windows":
        return root / f"{stem}.dll"
    if system == "Darwin":
        return root / f"lib{stem}.dylib"
    return root / f"lib{stem}.so"


def native_build_metadata(directory: Path | None = None) -> dict[str, Any]:
    """
    Return the recorded build metadata, or raise if it is absent.

    The previous behaviour of returning ``{}`` for a missing file is what let a
    metadata contract break go unnoticed, so absence is now an explicit error.
    """
    path = library_path(directory=directory).with_name(_METADATA_NAME)
    if not path.is_file():
        raise NativeKernelUnavailableError(
            f"kernel build metadata is missing at {path}; "
            "build the kernel before reading its metadata"
        )
    return json.loads(path.read_text(encoding="utf-8-sig"))


@dataclass(frozen=True)
class LibraryIdentity:
    """What the loaded library says about itself."""

    path: Path
    abi_versions: Mapping[str, int]
    metadata: Mapping[str, Any]


class KernelLibrary:
    """A loaded kernel library plus the symbols that were probed on it."""

    def __init__(
        self,
        library: ctypes.CDLL,
        path: Path,
        abi_versions: Mapping[str, int],
        metadata: Mapping[str, Any],
    ) -> None:
        self._library = library
        self.identity = LibraryIdentity(
            path=path, abi_versions=dict(abi_versions), metadata=dict(metadata)
        )

    @property
    def handle(self) -> ctypes.CDLL:
        """Return the underlying ``ctypes`` handle."""
        return self._library

    def symbol(self, name: str) -> Any:
        """Return an exported symbol, raising if the library does not export it."""
        found = getattr(self._library, name, None)
        if found is None:
            raise KernelSymbolMissingError(name, str(self.identity.path))
        return found

    def require(self, names: Iterable[str]) -> None:
        """Assert that every named symbol is exported."""
        for name in names:
            self.symbol(name)


def load_kernel_library(
    *,
    abi_symbols: Mapping[str, int] | None = None,
    required_symbols: Iterable[str] = (),
    stem: str = DEFAULT_LIBRARY_STEM,
    directory: Path | None = None,
) -> KernelLibrary:
    """
    Load the kernel library and apply the symbol and ABI gates.

    Parameters
    ----------
    abi_symbols
        Mapping of version-exporting symbol name to the expected value, for
        example ``{"axle_kernel_abi_version": 15}``.  Each symbol must exist,
        must take no arguments, and must return the expected integer.
    required_symbols
        Names that must exist on the library before it is handed back.
    stem, directory
        Overrides for the library base name and containing directory, used by
        tests that point the loader at a deliberately reduced stub library.

    """
    path = library_path(stem, directory)
    if not path.is_file():
        raise NativeKernelUnavailableError(
            f"native kernel is unavailable at {path}; build it with "
            "`just build` (or `uv run python packages/suspension_kernel/"
            "scripts/build_suspension_kernel.py`)"
        )
    try:
        handle = ctypes.CDLL(str(path))
    except OSError as error:  # pragma: no cover - depends on host loader
        raise NativeKernelUnavailableError(
            f"native kernel at {path} could not be loaded: {error}"
        ) from error

    abi_versions: dict[str, int] = {}
    for symbol_name, expected in (abi_symbols or {}).items():
        probe = getattr(handle, symbol_name, None)
        if probe is None:
            raise KernelSymbolMissingError(symbol_name, str(path))
        probe.argtypes = []
        probe.restype = ctypes.c_int
        observed = int(probe())
        abi_versions[symbol_name] = observed
        if observed != expected:
            raise KernelAbiMismatchError(symbol_name, expected, observed)

    for symbol_name in required_symbols:
        if getattr(handle, symbol_name, None) is None:
            raise KernelSymbolMissingError(symbol_name, str(path))

    # The metadata is part of the shipped artefact, so its absence is a broken
    # install rather than an acceptable state: reading it is deliberately not
    # wrapped, and ``native_build_metadata`` raises.  A silent ``{}`` here would
    # hide exactly the contract break this loader exists to surface.
    metadata = native_build_metadata(directory)
    return KernelLibrary(handle, path, abi_versions, metadata)
