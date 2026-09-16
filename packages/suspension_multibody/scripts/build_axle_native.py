"""
Build the axle native library for the current host platform.

Compatibility wrapper.  The real build lives in `packages/suspension_kernel`
(CMake + Ninja); this script delegates to it and then copies the product into
this package's `native` directory, which is where the axle ctypes boundary has
always loaded it from and where the multibody wheel packages it.
"""

from __future__ import annotations

import argparse
import shutil
import struct
import sys
from pathlib import Path

REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
KERNEL_PACKAGE = REPOSITORY_ROOT / "packages" / "suspension_kernel"
KERNEL_SOURCE = KERNEL_PACKAGE / "src"
NATIVE_DIR = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "suspension_multibody"
    / "native"
)

sys.path.insert(0, str(KERNEL_SOURCE))

from suspension_kernel.binding.build import build  # noqa: E402


def main() -> int:
    """Build the kernel and mirror its product into this package."""
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--configuration",
        choices=("Release", "Debug"),
        default="Release",
    )
    args = parser.parse_args()
    if struct.calcsize("P") != 8:
        raise RuntimeError("the axle native kernel requires a 64-bit host")

    produced = build(args.configuration)
    native_dir = NATIVE_DIR
    native_dir.mkdir(parents=True, exist_ok=True)
    destination = native_dir / produced.name
    shutil.copy2(produced, destination)
    # The metadata must sit next to the copy so the axle loader still finds it.
    shutil.copy2(produced.with_name("native_build.json"), native_dir / "native_build.json")
    _remove_stale_library_names(destination)
    print(destination)
    return 0


def _remove_stale_library_names(destination: Path) -> None:
    """Drop the pre-rename library file if a previous build left one behind."""
    stale = NATIVE_DIR / "axle_dynamics_native.dll"
    if stale.exists() and stale != destination:
        stale.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
