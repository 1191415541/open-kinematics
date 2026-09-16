"""
Build the suspension kernel with CMake + Ninja and write its metadata.

The flag list this records is the compiled reality, not a hand-maintained copy:
CMake is configured, then the resolved ``CXX_FLAGS``/``CXX_DEFINES`` from the
generated build files are captured and stored in ``native_build.json`` together
with the compiler identity, so a caller can assert on what was really used.
"""

from __future__ import annotations

import argparse
import ctypes
import json
import os
import platform
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path
from typing import Any

#: Literal recorded as the metadata `source` key.  Kept stable on purpose: the
#: key is part of the metadata contract consumed by artifact writers, and the
#: optional `sources` list carries the build-time truth separately.
LEGACY_SOURCE_LITERAL = "cpp/axle_dynamics/axle_kernel.cpp"

PACKAGE_ROOT = Path(__file__).resolve().parents[3]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]
NATIVE_DIR = PACKAGE_ROOT / "src" / "suspension_kernel" / "native"
#: K2 split the kernel into one translation unit per module and declared every
#: free function in a transitional aggregate header; K6 routed those declarations
#: to the modules that define them and deleted the aggregate.  The list is explicit
#: because CMake does not glob: a new file that is missing here compiles nowhere
#: and shows up only as an undefined symbol.
SOURCE_RELATIVES = (
    "packages/suspension_kernel/cpp/src/base/angles.cpp",
    "packages/suspension_kernel/cpp/src/base/kernel_base.cpp",
    "packages/suspension_kernel/cpp/src/tire_state/kernel_tire_state.cpp",
    "packages/suspension_kernel/cpp/src/model/kernel_model.cpp",
    "packages/suspension_kernel/cpp/src/model/kernel_model_accessors.cpp",
    "packages/suspension_kernel/cpp/src/constraint/kernel_model_constraint.cpp",
    "packages/suspension_kernel/cpp/src/constraint/registry.cpp",
    "packages/suspension_kernel/cpp/src/tire/brush/model.cpp",
    "packages/suspension_kernel/cpp/src/tire/fiala/relaxation.cpp",
    "packages/suspension_kernel/cpp/src/tire/fiala/directional.cpp",
    "packages/suspension_kernel/cpp/src/tire/fiala/model.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/contact_mass.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/turn_slip.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/directional.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/law.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/model.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/spin.cpp",
    "packages/suspension_kernel/cpp/src/tire/assemble.cpp",
    "packages/suspension_kernel/cpp/src/vehicle/force_assembly.cpp",
    "packages/suspension_kernel/cpp/src/tire/kernel_tire_assembly.cpp",
    "packages/suspension_kernel/cpp/src/suspension/spring.cpp",
    "packages/suspension_kernel/cpp/src/suspension/curves.cpp",
    "packages/suspension_kernel/cpp/src/suspension/bushing.cpp",
    "packages/suspension_kernel/cpp/src/suspension/anti_roll.cpp",
    "packages/suspension_kernel/cpp/src/vehicle/steering.cpp",
    "packages/suspension_kernel/cpp/src/vehicle/drive_brake.cpp",
    "packages/suspension_kernel/cpp/src/vehicle/layout.cpp",
    "packages/suspension_kernel/cpp/src/vehicle/kernel_directional.cpp",
    "packages/suspension_kernel/cpp/src/base/kernel_dual_algebra.cpp",
    "packages/suspension_kernel/cpp/src/constraint/kernel_directional_constraint.cpp",
    "packages/suspension_kernel/cpp/src/linalg/kernel_linalg.cpp",
    "packages/suspension_kernel/cpp/src/integrator/kernel_integrator.cpp",
    "packages/suspension_kernel/cpp/src/integrator/kernel_integrator_residual.cpp",
    "packages/suspension_kernel/cpp/src/integrator/kernel_integrator_input.cpp",
    "packages/suspension_kernel/cpp/src/integrator/kernel_integrator_newton.cpp",
    "packages/suspension_kernel/cpp/src/integrator/kernel_integrator_step.cpp",
    "packages/suspension_kernel/cpp/src/abi/kernel_model_build.cpp",
    "packages/suspension_kernel/cpp/src/static/kernel_static_trim.cpp",
    "packages/suspension_kernel/cpp/src/static/kernel_static_contact.cpp",
    "packages/suspension_kernel/cpp/src/static/kernel_static_projection.cpp",
    "packages/suspension_kernel/cpp/src/vehicle/kernel_registration.cpp",
    "packages/suspension_kernel/cpp/src/integrator/kernel_events.cpp",
    "packages/suspension_kernel/cpp/src/output/kernel_output.cpp",
    "packages/suspension_kernel/cpp/src/abi/kernel_core.cpp",
    "packages/suspension_kernel/cpp/src/tire/fiala/modes.cpp",
    "packages/suspension_kernel/cpp/src/tire/fiala/forces.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/parameters.cpp",
    "packages/suspension_kernel/cpp/src/tire/pac2002/vertical.cpp",
    "packages/suspension_kernel/cpp/src/base/dual_geometry.cpp",
    "packages/suspension_kernel/cpp/src/model/directional.cpp",
    "packages/suspension_kernel/cpp/src/model/road.cpp",
    "packages/suspension_kernel/cpp/src/tire/common/kinematics.cpp",
    "packages/suspension_kernel/cpp/src/base/curves.cpp",
    "packages/suspension_kernel/cpp/src/abi/element_reader.cpp",
    "packages/suspension_kernel/cpp/src/abi/kernel_abi.cpp",
    "packages/suspension_kernel/cpp/include/abi/functions.hpp",
    "packages/suspension_kernel/cpp/include/abi/version.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/constants.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/diagnostics.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/dual.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/dual_geometry.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/env.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/monotone_cubic.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/prelude.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/util.hpp",
    "packages/suspension_kernel/cpp/include/mb_base/vector.hpp",
    "packages/suspension_kernel/cpp/include/mb_constraint/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_constraint/registry.hpp",
    "packages/suspension_kernel/cpp/include/mb_constraint/types.hpp",
    "packages/suspension_kernel/cpp/include/mb_integrator/context.hpp",
    "packages/suspension_kernel/cpp/include/mb_integrator/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_linalg/factorization_types.hpp",
    "packages/suspension_kernel/cpp/include/mb_linalg/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_model/enums.hpp",
    "packages/suspension_kernel/cpp/include/mb_model/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_model/types.hpp",
    "packages/suspension_kernel/cpp/include/mb_output/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_output/measurement.hpp",
    "packages/suspension_kernel/cpp/include/mb_static/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_suspension/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/assembly.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/brush/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/common/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/common/kinematics.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/fiala/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/fiala/parameters.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/force_context.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/model.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/pac2002/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/pac2002/parameters.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/pac2002/spin.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire/pac2002/turn_slip.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire_state/functions.hpp",
    "packages/suspension_kernel/cpp/include/mb_tire_state/tire_state.hpp",
    "packages/suspension_kernel/cpp/include/mb_vehicle/energy.hpp",
    "packages/suspension_kernel/cpp/include/mb_vehicle/functions.hpp",
    "packages/suspension_kernel/cpp/axle_dynamics/axle_kernel.hpp",
    "packages/suspension_kernel/cpp/axle_dynamics/core_abi.hpp",
)


def discover_compiler(explicit: str | None = None) -> Path:
    """Find a 64-bit C++ compiler, in the historical discovery order."""
    if explicit:
        return Path(explicit).resolve()
    for variable in ("CXX",):
        value = os.environ.get(variable)
        if value:
            return Path(value).resolve()
    for name in ("x86_64-w64-mingw32-g++", "c++", "g++", "clang++"):
        found = shutil.which(name)
        if found:
            return Path(found)
    raise RuntimeError("no C++ compiler found in CXX or PATH")


def _assert_64_bit(compiler: Path) -> None:
    if struct.calcsize("P") != 8:
        raise RuntimeError("the suspension kernel requires a 64-bit host")
    completed = subprocess.run(
        [str(compiler), "-dumpmachine"],
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    machine = completed.stdout.strip()
    if not re.search(r"x86_64|amd64|aarch64|arm64", machine):
        raise RuntimeError(
            "the suspension kernel requires a 64-bit compiler; "
            f"detected target {machine!r}"
        )


def _run(command: list[str], *, cwd: Path) -> subprocess.CompletedProcess[str]:
    """
    Run one build step, surfacing the compiler's own output on failure.

    The text is decoded as UTF-8 with replacement rather than by the locale
    encoding: this repository lives under a path with non-ASCII characters, and a
    locale codec (GBK on the machine this was fixed on) raises
    ``UnicodeDecodeError`` on the compiler's own diagnostics.  That turned a
    compile error into a crash inside this function, which reads as a broken build
    driver instead of a broken build -- the same failure mode as indexing a
    missing key in the comparison tool.
    """
    completed = subprocess.run(
        command,
        cwd=cwd,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
        check=False,
    )
    if completed.returncode != 0:
        # Forward the compiler's output as bytes rather than as text.  Writing it
        # through the text layer re-encodes with the console codec, which cannot
        # represent the replacement characters that an undecodable diagnostic
        # produced -- so the decoder fix above would merely move the crash to the
        # printer.  The bytes are what the compiler said; pass them on unaltered.
        for stream, payload in (
            (sys.stdout, completed.stdout),
            (sys.stderr, completed.stderr),
        ):
            buffer = getattr(stream, "buffer", None)
            if buffer is not None:
                buffer.write((payload or "").encode("utf-8", "replace"))
                buffer.flush()
            else:  # pragma: no cover - only when stdout has been replaced
                stream.write(payload or "")
        raise RuntimeError(
            f"command failed with exit code {completed.returncode}: "
            + " ".join(command)
        )
    return completed


class _BuildLock:
    """
    Serialise concurrent builds that share one CMake binary directory.

    `just test-kernel` and `just test-multibody` both depend on a kernel build,
    and a parallel `just` runs them at the same time.  Two concurrent CMake
    configure passes in the same `-B` directory corrupt each other: the LTO
    capability probe writes and then removes `CMakeFiles/_CMakeLTOTest-CXX`,
    which the other pass is still using.  The symptom is a misleading
    "can't create CMakeFiles/foo.dir/foo.cpp.obj" during configure.

    An advisory exclusive lock on a sibling file keeps the common case (one
    build, cached) free of cost and turns the concurrent case into a queue.
    """

    def __init__(self, path: Path) -> None:
        self._path = path
        self._handle: Any = None

    def __enter__(self) -> _BuildLock:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._handle = self._path.open("a+b")
        if sys.platform == "win32":
            import msvcrt

            # Block until the byte range is available.
            while True:
                try:
                    self._handle.seek(0)
                    msvcrt.locking(self._handle.fileno(), msvcrt.LK_LOCK, 1)
                    break
                except OSError:  # pragma: no cover - retry on contention
                    continue
        else:  # pragma: no cover - exercised on POSIX runners
            import fcntl

            fcntl.flock(self._handle.fileno(), fcntl.LOCK_EX)
        return self

    def __exit__(self, *exc_info: object) -> None:
        if self._handle is None:
            return
        try:
            if sys.platform == "win32":
                import msvcrt

                self._handle.seek(0)
                msvcrt.locking(self._handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:  # pragma: no cover - exercised on POSIX runners
                import fcntl

                fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        finally:
            self._handle.close()
            self._handle = None


def _resolved_flags(build_dir: Path) -> list[str]:
    """
    Return the compile flags CMake actually recorded for the kernel target.

    `build.ninja` is the authoritative record here: with the Ninja generator
    CMake does not emit the `flags.make` file that the Makefile generators
    produce, and the flags are stored as a single space-separated `FLAGS` value.
    """
    ninja = build_dir / "build.ninja"
    if not ninja.is_file():
        return []
    for line in ninja.read_text(encoding="utf-8", errors="replace").splitlines():
        stripped = line.strip()
        if stripped.startswith("FLAGS = "):
            return stripped[len("FLAGS = ") :].split()
    return []


def build(configuration: str = "Release", compiler: str | None = None) -> Path:
    """Configure and build the kernel, returning the shared library path."""
    resolved = discover_compiler(compiler)
    _assert_64_bit(resolved)
    # The CMake target writes straight into the package, so the directory has to
    # exist before the link step; ninja fails with a bare "No such file or
    # directory" from `ld` otherwise.
    NATIVE_DIR.mkdir(parents=True, exist_ok=True)
    build_dir = PACKAGE_ROOT / "build" / configuration
    build_dir.mkdir(parents=True, exist_ok=True)

    cmake = shutil.which("cmake")
    if cmake is None:
        raise RuntimeError("cmake is required to build the suspension kernel")
    lock_path = build_dir / ".suspension_kernel_build.lock"
    with _BuildLock(lock_path):
        _run(
            [
                cmake,
                "-G",
                "Ninja",
                f"-DCMAKE_CXX_COMPILER={resolved}",
                f"-DCMAKE_BUILD_TYPE={configuration}",
                "-S",
                str(PACKAGE_ROOT),
                "-B",
                str(build_dir),
            ],
            cwd=REPOSITORY_ROOT,
        )
        _run([cmake, "--build", str(build_dir)], cwd=REPOSITORY_ROOT)

    produced = _locate_product(build_dir, configuration)
    if produced is None:
        raise RuntimeError(f"the build produced no shared library under {build_dir}")

    destination = NATIVE_DIR / produced.name
    shutil.copy2(produced, destination)
    _remove_stale_products(destination)

    version = _run([str(resolved), "--version"], cwd=REPOSITORY_ROOT).stdout.splitlines()[
        0
    ]
    # The ABI versions are read back out of the artefact that was just built, not
    # copied from a Python constant.  A hand-maintained second copy is exactly
    # how a wheel ends up advertising one ABI while exporting another, so the
    # metadata now cannot disagree with the library it describes.
    abi = _readback_abi_versions(destination)
    metadata: dict[str, Any] = {
        "abi_version": abi["abi_version"],
        "vehicle_abi_version": abi["vehicle_abi_version"],
        "compiler": str(resolved),
        "compiler_version": version,
        "configuration": configuration,
        "flags": _resolved_flags(build_dir),
        "source": LEGACY_SOURCE_LITERAL,
        # Appended fields; the seven above are a frozen contract.
        "core_abi_version": abi["core_abi_version"],
        "platform": platform.platform(),
        "architecture": platform.machine(),
        "sources": list(SOURCE_RELATIVES),
        "targets": ["suspension_kernel"],
    }
    (NATIVE_DIR / "native_build.json").write_text(
        json.dumps(metadata, indent=2, sort_keys=True), encoding="utf-8"
    )
    return destination


#: Version exports read back from the built library, in metadata-key order.
_ABI_EXPORTS = (
    ("axle_kernel_abi_version", "abi_version"),
    ("vehicle_kernel_abi_version", "vehicle_abi_version"),
    ("mb_core_abi_version", "core_abi_version"),
)


def _readback_abi_versions(library: Path) -> dict[str, int]:
    """
    Load the built library and read the ABI versions it actually exports.

    The library is loaded here directly rather than through
    :func:`load_kernel_library` on purpose: that path re-reads this very
    metadata file, which does not exist yet at this point.
    """
    handle = ctypes.CDLL(str(library))
    exported: dict[str, int] = {}
    for symbol, key in _ABI_EXPORTS:
        probe = getattr(handle, symbol, None)
        if probe is None:
            raise RuntimeError(
                f"the built library at {library} does not export {symbol!r}, "
                "so its ABI version cannot be recorded"
            )
        probe.argtypes = []
        probe.restype = ctypes.c_int
        exported[key] = int(probe())
    return exported


def _remove_stale_products(destination: Path) -> None:
    """Drop same-extension products from an earlier name or generator."""
    for pattern in ("*.dll", "*.dylib", "*.so"):
        for candidate in NATIVE_DIR.glob(pattern):
            if candidate != destination:
                candidate.unlink()


def _locate_product(build_dir: Path, configuration: str) -> Path | None:
    """Find the freshly built shared library, whatever its extension."""
    for pattern in (
        "**/suspension_kernel.dll",
        "**/libsuspension_kernel.dylib",
        "**/libsuspension_kernel.so",
    ):
        candidates = [
            path
            for path in build_dir.glob(pattern)
            if path.is_file() and "CMakeFiles" not in path.parts
        ]
        if candidates:
            return max(candidates, key=lambda path: path.stat().st_mtime)
    return None


def main(argv: list[str] | None = None) -> int:
    """Command-line entry point for the kernel build."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configuration", choices=("Release", "Debug"), default="Release")
    parser.add_argument("--compiler", default=None)
    args = parser.parse_args(argv)
    print(build(args.configuration, args.compiler))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
