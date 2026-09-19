"""Neutral native-kernel loading runtime."""

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

__all__ = ["NativeKernelUnavailableError", "native_build_metadata", "load_library"]

_LIBRARY_STEM = "suspension_kernel"
_NATIVE_DIR = Path(__file__).resolve().parent.parent / "native"

_NATIVE_KERNEL_ABI_VERSION = 15
_NATIVE_VEHICLE_KERNEL_ABI_VERSION = 30
_NATIVE_CORE_ABI_VERSION = 1


NativeKernelUnavailableError = KernelUnavailableError



def _library_path() -> Path:
    """Return the packaged product mirror of the kernel library."""
    return kernel_library_path(_LIBRARY_STEM, _NATIVE_DIR)


def native_build_metadata() -> dict[str, object]:
    """Return the recorded compiler and ABI metadata."""
    return kernel_build_metadata(_NATIVE_DIR)

def _require_matching_metadata_versions() -> None:
    """Assert that shipped metadata matches the loader ABI constants."""
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
    """Return the canonical kernel package's copy of the built library."""
    return kernel_native_directory() / _library_path().name


def _same_content(left: Path, right: Path) -> bool:
    """Return whether two files hold identical bytes."""
    if left.stat().st_size != right.stat().st_size:
        return False
    with left.open("rb") as first, right.open("rb") as second:
        while True:
            left_chunk = first.read(1 << 20)
            right_chunk = second.read(1 << 20)
            if left_chunk != right_chunk:
                return False
            if not left_chunk:
                return True


def require_fresh_mirror(
    mirror: Path,
    canonical: Path,
    *,
    error_type: type[Exception] = NativeKernelUnavailableError,
    same_content=None,
) -> None:
    """Refuse to load an older mirror whose bytes differ from its source."""
    if not canonical.is_file() or not mirror.is_file():
        return
    if canonical.stat().st_mtime <= mirror.stat().st_mtime:
        return
    comparator = _same_content if same_content is None else same_content
    if comparator(mirror, canonical):
        return
    raise error_type(
        f"the native kernel mirror at {mirror} is older and different from "
        f"{canonical}; run `just build-axle-native` (or `just build-kernel` "
        "followed by `just build-axle-native`) so the package loads the "
        "library it was built against"
    )



def _require_fresh_mirror() -> None:
    """Apply the freshness guard to the packaged mirror."""
    require_fresh_mirror(_library_path(), _canonical_kernel_library())


def load_library(
    *,
    stem: str = _LIBRARY_STEM,
    directory: Path = _NATIVE_DIR,
    require_fresh: bool = True,
) -> ctypes.CDLL:
    """Load the kernel with the shared ABI and symbol gates."""
    path = kernel_library_path(stem, directory)
    if not path.exists():
        raise NativeKernelUnavailableError(
            f"native kernel is unavailable at {path}; "
            "run packages/suspension_multibody/scripts/build_axle_native.py"
        )
    if require_fresh and stem == _LIBRARY_STEM and Path(directory) == _NATIVE_DIR:
        _require_fresh_mirror()
    try:
        if stem == _LIBRARY_STEM and Path(directory) == _NATIVE_DIR:
            _require_matching_metadata_versions()
        return kernel_load_library(
            stem=stem,
            directory=directory,
            abi_symbols={
                "axle_kernel_abi_version": _NATIVE_KERNEL_ABI_VERSION,
                "vehicle_kernel_abi_version": _NATIVE_VEHICLE_KERNEL_ABI_VERSION,
                "mb_core_abi_version": _NATIVE_CORE_ABI_VERSION,
            },
            required_symbols=(
                "suspension_kernel_run",
                "suspension_kernel_contract_version",
                "suspension_kernel_capabilities",
            ),
        ).handle
    except (KernelSymbolMissingError, KernelAbiMismatchError) as error:
        raise NativeKernelUnavailableError(str(error)) from error
