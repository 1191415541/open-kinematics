"""
Locate and load the packaged native kernel library.

The Python product no longer mirrors the kernel's flat input/output structures.
This module is deliberately only the shared-library boundary: where the axle
package keeps its copy, how that copy is checked against the canonical build,
and which contract symbols must be present before the library is returned.
All model and result translation lives in the contract/document layer.
"""

from __future__ import annotations

import ctypes
from pathlib import Path

from suspension_kernel.binding import (
    KernelAbiMismatchError,
    KernelSymbolMissingError,
)
from suspension_kernel.binding import (
    NativeKernelUnavailableError as KernelUnavailableError,
)
from suspension_kernel.binding import (
    library_path as kernel_library_path,
)
from suspension_kernel.binding import (
    load_kernel_library as kernel_load_library,
)
from suspension_kernel.binding import (
    native_build_metadata as kernel_build_metadata,
)
from suspension_kernel.binding import (
    native_directory as kernel_native_directory,
)

_LIBRARY_STEM = "suspension_kernel"
_NATIVE_DIR = Path(__file__).resolve().parent.parent / "native"

# These are the internal ABI versions recorded in the build metadata.  The
# contract entry point itself has its own version; checking both keeps a
# stale or mismatched DLL from crossing the Python boundary.
_NATIVE_KERNEL_ABI_VERSION = 15
_NATIVE_VEHICLE_KERNEL_ABI_VERSION = 30
_NATIVE_CORE_ABI_VERSION = 1


class NativeKernelUnavailableError(KernelUnavailableError):
    """Raised when the packaged native kernel cannot be loaded."""


def _library_path() -> Path:
    """Return this package's copy of the kernel library."""
    return kernel_library_path(_LIBRARY_STEM, _NATIVE_DIR)


def native_build_metadata() -> dict[str, object]:
    """Return the recorded compiler and ABI metadata."""
    return kernel_build_metadata(_NATIVE_DIR)


def _require_matching_metadata_versions() -> None:
    """Assert that module constants agree with the recorded build metadata."""
    metadata = native_build_metadata()
    for key, expected in (
        ("abi_version", _NATIVE_KERNEL_ABI_VERSION),
        ("vehicle_abi_version", _NATIVE_VEHICLE_KERNEL_ABI_VERSION),
        ("core_abi_version", _NATIVE_CORE_ABI_VERSION),
    ):
        observed = metadata.get(key)
        if observed != expected:
            raise KernelAbiMismatchError(key, expected, observed)


def _canonical_kernel_library() -> Path:
    """Return the kernel package's own copy of the built library."""
    return kernel_native_directory() / _library_path().name


def _same_content(left: Path, right: Path) -> bool:
    """Return whether two files hold identical bytes."""
    if left.stat().st_size != right.stat().st_size:
        return False
    with left.open("rb") as first, right.open("rb") as second:
        while True:
            a = first.read(1 << 20)
            b = second.read(1 << 20)
            if a != b:
                return False
            if not a:
                return True


def _require_fresh_mirror() -> None:
    """Refuse to load a mirror that is older and different from its source."""
    mirror = _library_path()
    canonical = _canonical_kernel_library()
    if not canonical.is_file() or not mirror.is_file():
        return
    if canonical.stat().st_mtime <= mirror.stat().st_mtime:
        return
    if _same_content(mirror, canonical):
        return
    raise NativeKernelUnavailableError(
        f"the native kernel mirror at {mirror} is older and different from "
        f"{canonical}; run `just build-axle-native` (or `just build-kernel` "
        "followed by `just build-axle-native`) so the axle package loads the "
        "library it was built against"
    )


def _load_library() -> ctypes.CDLL:
    """Load the kernel and require the contract entry-point surface."""
    path = _library_path()
    if not path.exists():
        raise NativeKernelUnavailableError(
            f"native axle kernel is unavailable at {path}; "
            "run packages/suspension_multibody/scripts/build_axle_native.py"
        )
    _require_fresh_mirror()
    try:
        return kernel_load_library(
            stem=_LIBRARY_STEM,
            directory=_NATIVE_DIR,
            abi_symbols={
                "axle_kernel_abi_version": _NATIVE_KERNEL_ABI_VERSION,
                "vehicle_kernel_abi_version": _NATIVE_VEHICLE_KERNEL_ABI_VERSION,
            },
            required_symbols=(
                "suspension_kernel_run",
                "suspension_kernel_contract_version",
                "suspension_kernel_capabilities",
            ),
        ).handle
    except (KernelSymbolMissingError, KernelAbiMismatchError) as error:
        raise NativeKernelUnavailableError(str(error)) from error
